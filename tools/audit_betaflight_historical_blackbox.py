#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
ACC_1G_RAW = 2048.0
REQUIRED_FIELDS = (
    "time (us)",
    "rcCommand[3]",
    "vbatLatest (V)",
    "amperageLatest (A)",
    "baroAlt",
    "gyroADC[0]",
    "gyroADC[1]",
    "gyroADC[2]",
    "accSmooth[0]",
    "accSmooth[1]",
    "accSmooth[2]",
    "motor[0]",
    "motor[1]",
    "motor[2]",
    "motor[3]",
)
OPTIONAL_FIELDS = (
    "GPS_numSat",
    "GPS_coord[0]",
    "GPS_coord[1]",
    "GPS_speed (m/s)",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Classify every decoded historical Betaflight Blackbox flight and "
            "quantify diagnostic voltage/throttle coverage."
        )
    )
    parser.add_argument("--decoded-dir", required=True)
    parser.add_argument(
        "--blackbox-dir", default=str(ROOT / "logs" / "blackbox_import")
    )
    parser.add_argument("--context", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = audit_historical_blackbox(
        decoded_dir=Path(args.decoded_dir),
        blackbox_dir=Path(args.blackbox_dir),
        context_path=Path(args.context),
    )
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(output_path)
    print(f"files={result['summary']['file_count']}")
    print(
        "supplemental_flight_like="
        f"{result['summary']['supplemental_flight_like_count']}"
    )


def audit_historical_blackbox(
    *, decoded_dir: Path, blackbox_dir: Path, context_path: Path
) -> dict[str, Any]:
    decoded_dir = decoded_dir.expanduser().resolve()
    blackbox_dir = blackbox_dir.expanduser().resolve()
    context_path = context_path.expanduser().resolve()
    context_document = json.loads(context_path.read_text(encoding="utf-8"))
    contexts = context_document.get("historical_blackbox_context", {})
    if not isinstance(contexts, dict):
        raise RuntimeError("historical_blackbox_context must be an object")
    formal_bfl_stems = {
        Path(str(source["blackbox_bfl"])).stem
        for source in context_document.get("sources", [])
        if source.get("include", True) and source.get("blackbox_bfl")
    }

    bfl_paths = sorted(blackbox_dir.glob("LOG*.BFL"))
    if not bfl_paths:
        raise RuntimeError(f"no BFL files found under {blackbox_dir}")

    entries = []
    sample_groups: dict[str, list[np.ndarray]] = {
        "all": [],
        "flight_like": [],
        "known_outdoor": [],
        "supplemental_flight_like": [],
    }
    for bfl_path in bfl_paths:
        log_id = bfl_path.stem
        decoded_path = decoded_dir / f"{log_id}.01.csv"
        if not decoded_path.is_file():
            raise RuntimeError(f"decoded CSV is missing: {decoded_path}")
        context = contexts.get(log_id, {})
        if not isinstance(context, dict):
            raise RuntimeError(f"context for {log_id} must be an object")
        entry, samples = _audit_file(
            bfl_path,
            decoded_path,
            context,
            formal_coverage_included=bfl_path.stem in formal_bfl_stems,
        )
        entries.append(entry)
        sample_groups["all"].append(samples)
        if entry["classification"]["flight_like"]:
            sample_groups["flight_like"].append(samples)
        if entry["context"]["venue"] == "outdoor":
            sample_groups["known_outdoor"].append(samples)
        if entry["classification"]["release_evidence_class"] == "supplemental_only":
            if entry["classification"]["flight_like"]:
                sample_groups["supplemental_flight_like"].append(samples)

    release_counts: dict[str, int] = {}
    physical_counts: dict[str, int] = {}
    venue_counts: dict[str, int] = {}
    for entry in entries:
        release_class = entry["classification"]["release_evidence_class"]
        physical_class = entry["classification"]["physical_class"]
        venue = entry["context"]["venue"]
        release_counts[release_class] = release_counts.get(release_class, 0) + 1
        physical_counts[physical_class] = physical_counts.get(physical_class, 0) + 1
        venue_counts[venue] = venue_counts.get(venue, 0) + 1

    supplemental_ids = [
        entry["id"]
        for entry in entries
        if entry["classification"]["release_evidence_class"]
        == "supplemental_only"
        and entry["classification"]["flight_like"]
    ]
    return {
        "schema_version": 1,
        "purpose": "historical_blackbox_screening_not_release_lut_calibration",
        "context": _metadata(context_path),
        "method": {
            "decoded_csv_directory": str(decoded_dir),
            "resample_hz": 10.0,
            "throttle_source": "Betaflight internal rcCommand[3]",
            "specific_force": "norm(accSmooth)/2048",
            "baro_altitude_scale": "baroAlt raw centimeters divided by 100",
            "limitations": [
                "Internal rcCommand[3] is not interchangeable with receiver PWM at the endpoints.",
                "A flight-like physics signature does not prove an outdoor venue.",
                "Blackbox-only files cannot become hash-bound release LUT evidence without a unique host CSV and scene provenance.",
            ],
        },
        "summary": {
            "file_count": len(entries),
            "physical_class_counts": physical_counts,
            "venue_counts": venue_counts,
            "release_evidence_class_counts": release_counts,
            "supplemental_flight_like_count": len(supplemental_ids),
            "supplemental_flight_like_ids": supplemental_ids,
            "formal_coverage_included_count": sum(
                int(entry["classification"]["formal_coverage_included"])
                for entry in entries
            ),
            "clean_end_count": sum(int(entry["clean_end"]) for entry in entries),
        },
        "diagnostic_internal_throttle_coverage": {
            name: _summarize_samples(groups)
            for name, groups in sample_groups.items()
        },
        "files": entries,
        "interpretation": [
            "Formal release coverage remains defined by paired host receiver PWM and Blackbox data in the primary coverage audit.",
            "This scan may identify old flights for independent model validation or for locating missing host archives.",
            "It must not be added arithmetically to the formal effective-sample total.",
        ],
    }


def _audit_file(
    bfl_path: Path,
    decoded_path: Path,
    context: dict[str, Any],
    *,
    formal_coverage_included: bool = False,
) -> tuple[dict[str, Any], np.ndarray]:
    fields, values = _read_numeric_fields(decoded_path)
    column = {name: values[:, index] for index, name in enumerate(fields)}
    time_s = (column["time (us)"] - column["time (us)"][0]) / 1.0e6
    throttle = column["rcCommand[3]"]
    voltage = column["vbatLatest (V)"]
    current = column["amperageLatest (A)"]
    gyro = np.column_stack([column[f"gyroADC[{axis}]"] for axis in range(3)])
    acceleration = np.column_stack(
        [column[f"accSmooth[{axis}]"] for axis in range(3)]
    )
    motors = np.column_stack([column[f"motor[{axis}]"] for axis in range(4)])
    force_g = np.linalg.norm(acceleration, axis=1) / ACC_1G_RAW
    gyro_norm = np.linalg.norm(gyro, axis=1)
    finite = np.isfinite(time_s) & np.isfinite(throttle) & np.isfinite(voltage)
    powered = finite & (throttle > 1050.0)
    powered_duration_s = _masked_duration(time_s, powered)
    powered_count = int(np.count_nonzero(powered))

    if powered_count:
        powered_force = force_g[powered & np.isfinite(force_g)]
        powered_gyro = gyro_norm[powered & np.isfinite(gyro_norm)]
        powered_baro = column["baroAlt"][powered & np.isfinite(column["baroAlt"])]
        powered_voltage = voltage[powered]
        powered_throttle = throttle[powered]
        powered_current = current[powered & np.isfinite(current)]
    else:
        powered_force = np.asarray([], dtype=float)
        powered_gyro = np.asarray([], dtype=float)
        powered_baro = np.asarray([], dtype=float)
        powered_voltage = np.asarray([], dtype=float)
        powered_throttle = np.asarray([], dtype=float)
        powered_current = np.asarray([], dtype=float)

    gps = _gps_metrics(column)
    physics = {
        "powered_duration_s": powered_duration_s,
        "internal_throttle_max": _finite_max(throttle),
        "powered_voltage_v": _finite_range(powered_voltage),
        "powered_current_max_a": _finite_max(powered_current),
        "powered_specific_force_g_p50_p95_max": _finite_quantiles(
            powered_force, (0.5, 0.95, 1.0)
        ),
        "powered_gyro_norm_raw_p95_max": _finite_quantiles(
            powered_gyro, (0.95, 1.0)
        ),
        "powered_baro_altitude_range_m": (
            float(np.ptp(powered_baro) / 100.0) if len(powered_baro) else None
        ),
        "motor_raw_max": _finite_max(motors),
        **gps,
    }
    automatic_class = _classify_physics(physics)
    physical_class = str(context.get("physical_class", automatic_class))
    flight_like = physical_class in {
        "dynamic_flight",
        "hover_flight",
        "flight_or_unrestrained_dynamic",
    }
    host_pair = str(context.get("host_pair", "none"))
    release_evidence_class = str(
        context.get(
            "release_evidence_class",
            "paired_primary_candidate" if host_pair == "unique" else "not_usable",
        )
    )
    venue = str(context.get("venue", "unknown"))
    scene = str(context.get("scene", "unidentified"))

    diagnostic_selected = (
        powered
        & np.isfinite(force_g)
        & (voltage >= 20.0)
        & (voltage <= 25.2)
        & (throttle >= 1200.0)
        & (throttle <= 1500.0)
        & (force_g >= 0.3)
        & (force_g <= 3.0)
    )
    samples = np.column_stack(
        [time_s[diagnostic_selected], voltage[diagnostic_selected],
         throttle[diagnostic_selected], force_g[diagnostic_selected]]
    )
    samples = _resample_medians(samples, 10.0)
    entry = {
        "id": bfl_path.stem,
        "blackbox_bfl": _metadata(bfl_path),
        "decoded_row_count": int(len(values)),
        "duration_s": float(time_s[-1] - time_s[0]),
        "clean_end": _has_clean_end(decoded_path),
        "context": {
            "venue": venue,
            "scene": scene,
            "host_pair": host_pair,
            "source": context.get("source", "physics_screen_only"),
            "note": context.get("note"),
        },
        "classification": {
            "physical_class": physical_class,
            "automatic_physical_class": automatic_class,
            "flight_like": flight_like,
            "release_evidence_class": release_evidence_class,
            "formal_coverage_included": formal_coverage_included,
        },
        "physics": physics,
        "diagnostic_internal_throttle_sample_count_10hz": int(len(samples)),
        "diagnostic_internal_throttle_coverage": _summarize_samples([samples]),
    }
    return entry, samples


def _classify_physics(metrics: dict[str, Any]) -> str:
    duration = float(metrics["powered_duration_s"])
    throttle_max = metrics["internal_throttle_max"]
    if throttle_max is None or throttle_max < 1200.0 or duration < 1.0:
        return "no_material_powered_excitation"

    force = metrics["powered_specific_force_g_p50_p95_max"]
    gyro = metrics["powered_gyro_norm_raw_p95_max"]
    baro_range = metrics["powered_baro_altitude_range_m"]
    dynamic_signals = 0
    if force and force[1] is not None and force[1] >= 1.2:
        dynamic_signals += 1
    if gyro and gyro[0] is not None and gyro[0] >= 20.0:
        dynamic_signals += 1
    if baro_range is not None and baro_range >= 0.5:
        dynamic_signals += 1
    if metrics["gps_distinct_update_count"] >= 5:
        dynamic_signals += 1
    if duration >= 5.0 and dynamic_signals >= 2:
        return "flight_or_unrestrained_dynamic"
    return "brief_or_fixed_powered_test"


def _read_numeric_fields(path: Path) -> tuple[list[str], np.ndarray]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        header = [value.strip() for value in next(csv.reader(stream))]
    missing = [field for field in REQUIRED_FIELDS if field not in header]
    if missing:
        raise RuntimeError(f"{path.name} is missing: {', '.join(missing)}")
    selected = list(REQUIRED_FIELDS)
    selected.extend(field for field in OPTIONAL_FIELDS if field in header)
    indexes = [header.index(field) for field in selected]
    values = np.loadtxt(path, delimiter=",", skiprows=1, usecols=indexes)
    if values.ndim == 1:
        values = values.reshape(1, -1)
    if values.ndim != 2 or len(values) < 2:
        raise RuntimeError(f"{path.name} has insufficient rows")
    return selected, values


def _gps_metrics(column: dict[str, np.ndarray]) -> dict[str, Any]:
    if "GPS_numSat" not in column:
        return {
            "gps_nonzero_row_count": 0,
            "gps_distinct_update_count": 0,
            "gps_satellite_max": None,
            "gps_speed_max_m_s": None,
        }
    satellites = column["GPS_numSat"]
    valid = np.isfinite(satellites) & (satellites > 0)
    if not np.any(valid):
        return {
            "gps_nonzero_row_count": 0,
            "gps_distinct_update_count": 0,
            "gps_satellite_max": 0,
            "gps_speed_max_m_s": 0.0,
        }
    names = [name for name in OPTIONAL_FIELDS if name in column]
    values = np.column_stack([column[name][valid] for name in names])
    changes = np.ones(len(values), dtype=bool)
    if len(values) > 1:
        changes[1:] = np.any(values[1:] != values[:-1], axis=1)
    speed = column.get("GPS_speed (m/s)")
    speed_values = speed[valid] if speed is not None else np.asarray([])
    return {
        "gps_nonzero_row_count": int(np.count_nonzero(valid)),
        "gps_distinct_update_count": int(np.count_nonzero(changes)),
        "gps_satellite_max": int(np.nanmax(satellites[valid])),
        "gps_speed_max_m_s": _finite_max(speed_values),
    }


def _masked_duration(time_s: np.ndarray, mask: np.ndarray) -> float:
    if len(time_s) < 2 or not np.any(mask):
        return 0.0
    dt = np.diff(time_s, append=time_s[-1])
    finite_positive = dt[np.isfinite(dt) & (dt > 0.0)]
    cap = float(np.median(finite_positive) * 4.0) if len(finite_positive) else 0.0
    if cap > 0.0:
        dt = np.clip(dt, 0.0, cap)
    return float(np.sum(dt[mask]))


def _resample_medians(samples: np.ndarray, rate_hz: float) -> np.ndarray:
    if len(samples) == 0:
        return np.empty((0, 4), dtype=float)
    buckets = np.floor(samples[:, 0] * rate_hz).astype(np.int64)
    rows = [np.median(samples[buckets == bucket], axis=0) for bucket in np.unique(buckets)]
    return np.asarray(rows, dtype=float)


def _summarize_samples(groups: list[np.ndarray]) -> dict[str, Any]:
    nonempty = [group for group in groups if len(group)]
    if not nonempty:
        return {
            "sample_count_10hz": 0,
            "voltage_v": None,
            "internal_throttle": None,
            "throttle_bin_counts": [0, 0, 0, 0, 0, 0],
        }
    samples = np.concatenate(nonempty, axis=0)
    throttle_edges = np.asarray(
        [1200.0, 1250.0, 1300.0, 1350.0, 1400.0, 1450.0, 1500.000001]
    )
    counts, _ = np.histogram(samples[:, 2], bins=throttle_edges)
    return {
        "sample_count_10hz": int(len(samples)),
        "voltage_v": [float(np.min(samples[:, 1])), float(np.max(samples[:, 1]))],
        "internal_throttle": [
            float(np.min(samples[:, 2])),
            float(np.max(samples[:, 2])),
        ],
        "throttle_bin_edges": throttle_edges.tolist(),
        "throttle_bin_counts": counts.astype(int).tolist(),
    }


def _finite_range(values: np.ndarray) -> list[float] | None:
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return [float(np.min(values)), float(np.max(values))]


def _finite_max(values: np.ndarray) -> float | None:
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    return float(np.max(values)) if len(values) else None


def _finite_quantiles(
    values: np.ndarray, quantiles: tuple[float, ...]
) -> list[float] | None:
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return [float(value) for value in np.quantile(values, quantiles)]


def _has_clean_end(decoded_path: Path) -> bool:
    event_path = decoded_path.with_suffix(".event")
    if not event_path.is_file():
        return False
    return "Log clean end" in event_path.read_text(encoding="utf-8", errors="replace")


def _metadata(path: Path) -> dict[str, Any]:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    try:
        display_path = str(path.resolve().relative_to(ROOT))
    except ValueError:
        display_path = str(path.resolve())
    return {
        "path": display_path,
        "sha256": digest.hexdigest(),
        "bytes": path.stat().st_size,
    }


if __name__ == "__main__":
    main()
