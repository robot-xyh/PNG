#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

from analyze_betaflight_blackbox_flight import (  # noqa: E402
    _fit_throttle_alignment,
    _host_intervals,
    _read_host_rows,
)


REQUIRED_BLACKBOX_FIELDS = (
    "time (us)",
    "rcCommand[3]",
    "vbatLatest (V)",
    "accSmooth[0]",
    "accSmooth[1]",
    "accSmooth[2]",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Audit voltage/throttle coverage across paired Betaflight flights without "
            "building a release thrust LUT."
        )
    )
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--decoder", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = audit(
        manifest_path=Path(args.manifest),
        decoder_path=Path(args.decoder),
    )
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(output_path)
    print(f"coverage_passed={int(result['coverage']['passed'])}")


def audit(*, manifest_path: Path, decoder_path: Path) -> dict[str, Any]:
    manifest_path = manifest_path.expanduser().resolve()
    decoder_path = decoder_path.expanduser().resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    _validate_manifest(manifest)
    if not decoder_path.is_file():
        raise RuntimeError(f"Blackbox decoder does not exist: {decoder_path}")

    included = [source for source in manifest["sources"] if source.get("include", True)]
    excluded = [source for source in manifest["sources"] if not source.get("include", True)]
    with tempfile.TemporaryDirectory(prefix="betaflight_thrust_coverage_") as directory:
        decoded_root = Path(directory)
        _decode_sources(decoder_path, included, decoded_root)
        source_results = [
            _audit_source(source, decoded_root, manifest) for source in included
        ]

    requirements = manifest["requirements"]
    voltage_range = tuple(float(value) for value in requirements["voltage_v"])
    throttle_range = tuple(float(value) for value in requirements["throttle_us"])
    force_range_g = tuple(float(value) for value in requirements["specific_force_g"])
    sample_groups = []
    for result in source_results:
        source_samples = result.pop("_samples")
        source_mask = _coverage_mask(
            source_samples,
            voltage_range=voltage_range,
            throttle_range=throttle_range,
            force_range_g=force_range_g,
        )
        result["coverage_box_sample_count"] = int(np.count_nonzero(source_mask))
        sample_groups.append(source_samples)
    all_samples = np.concatenate(sample_groups, axis=0)
    force_valid = (
        (all_samples[:, 3] >= force_range_g[0])
        & (all_samples[:, 3] <= force_range_g[1])
    )
    in_requested_box = (
        force_valid
        & (all_samples[:, 1] >= voltage_range[0])
        & (all_samples[:, 1] <= voltage_range[1])
        & (all_samples[:, 2] >= throttle_range[0])
        & (all_samples[:, 2] <= throttle_range[1])
    )
    samples = all_samples[in_requested_box]
    summary = _summarize_coverage(samples, manifest)
    minimum_samples = int(requirements["minimum_effective_samples"])
    minimum_holdout = int(requirements["minimum_holdout_samples"])
    minimum_cell_samples = int(requirements.get("minimum_cell_samples", 1))
    voltage_endpoint_tolerance_v = float(
        requirements.get("voltage_endpoint_tolerance_v", 0.0)
    )
    throttle_endpoint_tolerance_us = float(
        requirements.get("throttle_endpoint_tolerance_us", 0.0)
    )
    potential_holdout = int(math.ceil(len(samples) / 5.0))
    checks = {
        "voltage_min_reached": bool(
            len(samples)
            and float(np.min(samples[:, 1]))
            <= voltage_range[0] + voltage_endpoint_tolerance_v
        ),
        "voltage_max_reached": bool(
            len(samples)
            and float(np.max(samples[:, 1]))
            >= voltage_range[1] - voltage_endpoint_tolerance_v
        ),
        "throttle_min_reached": bool(
            len(samples)
            and float(np.min(samples[:, 2]))
            <= throttle_range[0] + throttle_endpoint_tolerance_us
        ),
        "throttle_max_reached": bool(
            len(samples)
            and float(np.max(samples[:, 2]))
            >= throttle_range[1] - throttle_endpoint_tolerance_us
        ),
        "effective_sample_count": len(samples) >= minimum_samples,
        "potential_holdout_sample_count": potential_holdout >= minimum_holdout,
    }
    blockers = [name for name, passed in checks.items() if not passed]
    if summary["two_dimensional_minimum_cell_count"] < minimum_cell_samples:
        blockers.append("three_voltage_by_five_throttle_coverage_insufficient")

    return {
        "schema_version": 1,
        "purpose": "coverage_audit_only_not_a_release_thrust_lut",
        "manifest": _metadata(manifest_path),
        "decoder": {
            **_metadata(decoder_path),
            "commit": str(manifest.get("decoder_commit", "")),
        },
        "requirements": requirements,
        "sampling": {
            "resample_hz": float(manifest["resample_hz"]),
            "method": "per-flight fixed-width time buckets with per-field medians",
            "raw_blackbox_rows_are_not_counted_as_independent_samples": True,
        },
        "sources": source_results,
        "excluded_sources": excluded,
        "coverage": {
            "passed": not blockers,
            "blockers": blockers,
            "checks": checks,
            "effective_sample_count": int(len(samples)),
            "potential_every_fifth_holdout_sample_count": potential_holdout,
            "minimum_cell_samples_required": minimum_cell_samples,
            "force_rejected_sample_count": int(np.count_nonzero(~force_valid)),
            **summary,
            "fit_error_evaluation": "not_run_until_coverage_is_complete",
        },
        "interpretation": [
            "Only hashable host/Blackbox pairs selected in the manifest contribute.",
            "LOG00106 is truncated before the first interceptor motor-boundary contact response.",
            "This audit does not create a LUT and cannot satisfy active-flight authorization.",
        ],
    }


def _coverage_mask(
    samples: np.ndarray,
    *,
    voltage_range: tuple[float, float],
    throttle_range: tuple[float, float],
    force_range_g: tuple[float, float],
) -> np.ndarray:
    return (
        (samples[:, 3] >= force_range_g[0])
        & (samples[:, 3] <= force_range_g[1])
        & (samples[:, 1] >= voltage_range[0])
        & (samples[:, 1] <= voltage_range[1])
        & (samples[:, 2] >= throttle_range[0])
        & (samples[:, 2] <= throttle_range[1])
    )


def _validate_manifest(manifest: dict[str, Any]) -> None:
    if int(manifest.get("schema_version", 0)) != 1:
        raise RuntimeError("coverage manifest schema_version must be 1")
    if float(manifest.get("resample_hz", 0.0)) <= 0.0:
        raise RuntimeError("resample_hz must be positive")
    requirements = manifest.get("requirements")
    if not isinstance(requirements, dict):
        raise RuntimeError("requirements must be an object")
    for key in ("voltage_v", "throttle_us", "specific_force_g"):
        values = requirements.get(key)
        if not isinstance(values, list) or len(values) != 2 or values[0] >= values[1]:
            raise RuntimeError(f"requirements.{key} must contain an increasing pair")
    sources = manifest.get("sources")
    if not isinstance(sources, list) or not sources:
        raise RuntimeError("sources must be a non-empty list")
    labels = [str(source.get("id", "")) for source in sources]
    if any(not label for label in labels) or len(labels) != len(set(labels)):
        raise RuntimeError("source ids must be non-empty and unique")


def _decode_sources(
    decoder_path: Path,
    sources: list[dict[str, Any]],
    output_root: Path,
) -> None:
    bfl_paths = [ROOT / str(source["blackbox_bfl"]) for source in sources]
    command = [
        str(decoder_path),
        "--unit-rotation",
        "raw",
        "--unit-height",
        "m",
        "--unit-gps-speed",
        "mps",
        "--merge-gps",
        "--save-headers",
        "--output-dir",
        str(output_root),
        *(str(path) for path in bfl_paths),
    ]
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        raise RuntimeError(f"blackbox_decode failed: {completed.stderr[-2000:]}")


def _audit_source(
    source: dict[str, Any],
    decoded_root: Path,
    manifest: dict[str, Any],
) -> dict[str, Any]:
    source_id = str(source["id"])
    bfl_path = (ROOT / str(source["blackbox_bfl"])).resolve()
    host_path = (ROOT / str(source["host_csv"])).resolve()
    decoded_path = decoded_root / f"{bfl_path.stem}.01.csv"
    blackbox = _read_blackbox(decoded_path)
    time_s = (blackbox[:, 0] - blackbox[0, 0]) / 1.0e6
    host_rows = _read_host_rows(host_path, "rc_in_ch4")
    armed_intervals = _host_intervals(host_rows, "armed", 1)
    arm_index = int(source["host_arm_interval_index"])
    if arm_index < 0 or arm_index >= len(armed_intervals):
        raise RuntimeError(f"{source_id}: host ARM interval index is out of range")
    armed_interval = armed_intervals[arm_index]
    alignment = _fit_throttle_alignment(
        host_rows,
        host_throttle_field="rc_in_ch4",
        blackbox_time_s=time_s,
        blackbox_throttle=blackbox[:, 1],
        armed_interval=armed_interval,
        min_check_us=1050.0,
        max_pwm_us=2000.0,
        idle_command=1000.0,
        search_s=1.0,
        step_s=0.001,
    )
    host_time, host_throttle = _host_samples(host_rows)
    offset_s = float(alignment["host_minus_blackbox_s"])
    throttle_us = np.interp(time_s + offset_s, host_time, host_throttle)
    specific_force_g = np.linalg.norm(blackbox[:, 3:6], axis=1) / float(
        manifest["acc_1g_raw"]
    )
    selected = (
        (time_s + offset_s >= armed_interval[0])
        & (time_s + offset_s <= armed_interval[1])
        & np.isfinite(blackbox[:, 2])
        & np.isfinite(throttle_us)
        & np.isfinite(specific_force_g)
        & (specific_force_g > 0.1)
    )
    cutoff_s = source.get("max_blackbox_time_s")
    if cutoff_s is not None:
        selected &= time_s < float(cutoff_s)
    samples = np.column_stack(
        [time_s[selected], blackbox[selected, 2], throttle_us[selected], specific_force_g[selected]]
    )
    samples = _resample_medians(samples, float(manifest["resample_hz"]))
    voltage = samples[:, 1]
    throttle = samples[:, 2]
    force = samples[:, 3]
    return {
        "id": source_id,
        "blackbox_bfl": _metadata(bfl_path),
        "host_csv": _metadata(host_path),
        "host_arm_interval_index": arm_index,
        "host_arm_interval_s": list(armed_interval),
        "max_blackbox_time_s": cutoff_s,
        "resampled_sample_count": int(len(samples)),
        "observed_voltage_v": [float(np.min(voltage)), float(np.max(voltage))],
        "observed_throttle_us": [float(np.min(throttle)), float(np.max(throttle))],
        "observed_specific_force_g": [float(np.min(force)), float(np.max(force))],
        "alignment": alignment,
        "_samples": samples,
    }


def _read_blackbox(path: Path) -> np.ndarray:
    with path.open("r", newline="", encoding="utf-8") as stream:
        header = [value.strip() for value in next(csv.reader(stream))]
    missing = [field for field in REQUIRED_BLACKBOX_FIELDS if field not in header]
    if missing:
        raise RuntimeError(f"{path.name} is missing: {', '.join(missing)}")
    indexes = [header.index(field) for field in REQUIRED_BLACKBOX_FIELDS]
    values = np.loadtxt(path, delimiter=",", skiprows=1, usecols=indexes)
    if values.ndim != 2 or values.shape[0] < 20:
        raise RuntimeError(f"{path.name} has insufficient rows")
    return values


def _host_samples(rows: list[dict[str, str]]) -> tuple[np.ndarray, np.ndarray]:
    values = []
    for row in rows:
        try:
            timestamp = float(row["elapsed_s"])
            throttle = float(row["rc_in_ch4"])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(timestamp) and math.isfinite(throttle):
            values.append((timestamp, throttle))
    result = np.asarray(values, dtype=float)
    if result.ndim != 2 or len(result) < 2:
        raise RuntimeError("host CSV has insufficient finite throttle samples")
    order = np.argsort(result[:, 0])
    return result[order, 0], result[order, 1]


def _resample_medians(samples: np.ndarray, rate_hz: float) -> np.ndarray:
    if samples.ndim != 2 or samples.shape[1] != 4:
        raise ValueError("samples must have time, voltage, throttle, and force columns")
    if len(samples) == 0:
        return samples.copy()
    buckets = np.floor(samples[:, 0] * rate_hz).astype(np.int64)
    rows = [np.median(samples[buckets == bucket], axis=0) for bucket in np.unique(buckets)]
    return np.asarray(rows, dtype=float)


def _summarize_coverage(samples: np.ndarray, manifest: dict[str, Any]) -> dict[str, Any]:
    if len(samples) == 0:
        raise RuntimeError("no samples remain inside the requested coverage box")
    voltage_edges = np.asarray(manifest["voltage_bin_edges_v"], dtype=float)
    throttle_edges = np.asarray(manifest["throttle_bin_edges_us"], dtype=float)
    histogram, _, _ = np.histogram2d(
        samples[:, 1], samples[:, 2], bins=(voltage_edges, throttle_edges)
    )
    counts = histogram.astype(int)
    return {
        "observed_voltage_coverage_v": [
            float(np.min(samples[:, 1])),
            float(np.max(samples[:, 1])),
        ],
        "observed_throttle_coverage_us": [
            float(np.min(samples[:, 2])),
            float(np.max(samples[:, 2])),
        ],
        "observed_specific_force_coverage_g": [
            float(np.min(samples[:, 3])),
            float(np.max(samples[:, 3])),
        ],
        "voltage_bin_edges_v": voltage_edges.tolist(),
        "throttle_bin_edges_us": throttle_edges.tolist(),
        "two_dimensional_sample_counts": counts.tolist(),
        "voltage_bin_sample_counts": np.sum(counts, axis=1).tolist(),
        "throttle_bin_sample_counts": np.sum(counts, axis=0).tolist(),
        "two_dimensional_empty_cell_count": int(np.count_nonzero(counts == 0)),
        "two_dimensional_insufficient_cell_count": int(
            np.count_nonzero(
                counts
                < int(
                    dict(manifest.get("requirements", {})).get(
                        "minimum_cell_samples", 1
                    )
                )
            )
        ),
        "two_dimensional_minimum_cell_count": int(np.min(counts)),
    }


def _metadata(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise RuntimeError(f"evidence file does not exist: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    try:
        display_path = str(path.resolve().relative_to(ROOT))
    except ValueError:
        display_path = str(path.resolve())
    return {"path": display_path, "sha256": digest.hexdigest(), "bytes": path.stat().st_size}


if __name__ == "__main__":
    main()
