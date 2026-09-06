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
import time
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
from vision_guidance.thrust_model import (  # noqa: E402
    THRUST_LUT_MODEL_TYPE,
    THRUST_LUT_SCHEMA_VERSION,
)


REQUIRED_BLACKBOX_FIELDS = (
    "time (us)",
    "rcCommand[3]",
    "vbatLatest (V)",
    "accSmooth[0]",
    "accSmooth[1]",
    "accSmooth[2]",
    "gyroADC[0]",
    "gyroADC[1]",
    "gyroADC[2]",
    "motor[0]",
    "motor[1]",
    "motor[2]",
    "motor[3]",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a hash-bound voltage/throttle/specific-force LUT from flight evidence."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--host-csv")
    source.add_argument(
        "--manifest",
        help="Multi-flight source manifest accepted by audit_betaflight_thrust_coverage.py.",
    )
    parser.add_argument("--blackbox-csv")
    parser.add_argument("--blackbox-bfl", default="")
    parser.add_argument(
        "--decoder",
        default="",
        help="blackbox_decode executable; required with --manifest.",
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--calibration-id", required=True)
    parser.add_argument("--host-throttle-field", default="rc_in_ch4")
    parser.add_argument("--acc-1g-raw", type=float, default=2048.0)
    parser.add_argument("--min-check-us", type=float, default=1050.0)
    parser.add_argument("--max-pwm-us", type=float, default=2000.0)
    parser.add_argument("--idle-command", type=float, default=1000.0)
    parser.add_argument("--alignment-search-s", type=float, default=1.0)
    parser.add_argument("--alignment-step-s", type=float, default=0.001)
    parser.add_argument("--voltage-knots", type=int, default=3)
    parser.add_argument("--throttle-knots", type=int, default=5)
    parser.add_argument("--minimum-samples", type=int, default=500)
    parser.add_argument("--required-voltage-min-v", type=float, default=22.0)
    parser.add_argument("--required-voltage-max-v", type=float, default=25.2)
    parser.add_argument("--required-throttle-min-us", type=float, default=1200.0)
    parser.add_argument("--required-throttle-max-us", type=float, default=1500.0)
    parser.add_argument("--resample-hz", type=float, default=10.0)
    parser.add_argument("--armed-edge-trim-s", type=float, default=2.0)
    parser.add_argument("--minimum-specific-force-g", type=float, default=0.3)
    parser.add_argument("--maximum-specific-force-g", type=float, default=2.5)
    parser.add_argument("--maximum-gyro-deg-s", type=float, default=120.0)
    parser.add_argument("--motor-saturation-raw", type=float, default=1900.0)
    parser.add_argument("--minimum-cell-samples", type=int, default=5)
    parser.add_argument("--voltage-endpoint-tolerance-v", type=float, default=0.15)
    parser.add_argument("--throttle-endpoint-tolerance-us", type=float, default=10.0)
    parser.add_argument(
        "--maximum-voltage-extrapolation-v",
        type=float,
        default=0.30,
        help=(
            "Maximum bounded voltage-only extrapolation permitted by the "
            "physics-constrained fit."
        ),
    )
    parser.add_argument("--minimum-high-throttle-samples", type=int, default=5)
    parser.add_argument(
        "--exclude-blackbox-interval",
        action="append",
        default=[],
        metavar="START:END",
        help="Exclude a known landing/contact interval in Blackbox-relative seconds.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    common = dict(
        calibration_id=str(args.calibration_id),
        host_throttle_field=str(args.host_throttle_field),
        acc_1g_raw=float(args.acc_1g_raw),
        min_check_us=float(args.min_check_us),
        max_pwm_us=float(args.max_pwm_us),
        idle_command=float(args.idle_command),
        alignment_search_s=float(args.alignment_search_s),
        alignment_step_s=float(args.alignment_step_s),
        voltage_knot_count=int(args.voltage_knots),
        throttle_knot_count=int(args.throttle_knots),
        minimum_samples=int(args.minimum_samples),
        required_voltage_v=(
            float(args.required_voltage_min_v),
            float(args.required_voltage_max_v),
        ),
        required_throttle_us=(
            float(args.required_throttle_min_us),
            float(args.required_throttle_max_us),
        ),
        resample_hz=float(args.resample_hz),
        armed_edge_trim_s=float(args.armed_edge_trim_s),
        specific_force_g_range=(
            float(args.minimum_specific_force_g),
            float(args.maximum_specific_force_g),
        ),
        maximum_gyro_deg_s=float(args.maximum_gyro_deg_s),
        motor_saturation_raw=float(args.motor_saturation_raw),
        minimum_cell_samples=int(args.minimum_cell_samples),
        voltage_endpoint_tolerance_v=float(args.voltage_endpoint_tolerance_v),
        throttle_endpoint_tolerance_us=float(args.throttle_endpoint_tolerance_us),
        maximum_voltage_extrapolation_v=float(args.maximum_voltage_extrapolation_v),
        minimum_high_throttle_samples=int(args.minimum_high_throttle_samples),
        excluded_blackbox_intervals=_parse_intervals(args.exclude_blackbox_interval),
    )
    if args.manifest:
        if not args.decoder:
            raise RuntimeError("--decoder is required with --manifest")
        if args.blackbox_csv or args.blackbox_bfl or args.exclude_blackbox_interval:
            raise RuntimeError(
                "--blackbox-csv, --blackbox-bfl, and --exclude-blackbox-interval "
                "cannot be combined with --manifest"
            )
        output = calibrate_manifest(
            manifest_path=Path(args.manifest),
            decoder_path=Path(args.decoder),
            **common,
        )
    else:
        if not args.blackbox_csv:
            raise RuntimeError("--blackbox-csv is required with --host-csv")
        output = calibrate(
            host_csv=Path(args.host_csv),
            blackbox_csv=Path(args.blackbox_csv),
            blackbox_bfl=Path(args.blackbox_bfl) if args.blackbox_bfl else None,
            **common,
        )
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(output_path)
    print(f"sha256={_sha256(output_path)}")
    print(f"validation_passed={int(output['validation']['passed'])}")


def calibrate(
    *,
    host_csv: Path,
    blackbox_csv: Path,
    blackbox_bfl: Path | None,
    calibration_id: str,
    host_throttle_field: str,
    acc_1g_raw: float,
    min_check_us: float,
    max_pwm_us: float,
    idle_command: float,
    alignment_search_s: float,
    alignment_step_s: float,
    voltage_knot_count: int,
    throttle_knot_count: int,
    minimum_samples: int,
    required_voltage_v: tuple[float, float],
    required_throttle_us: tuple[float, float],
    resample_hz: float = 10.0,
    armed_edge_trim_s: float = 2.0,
    specific_force_g_range: tuple[float, float] = (0.3, 2.5),
    maximum_gyro_deg_s: float = 120.0,
    motor_saturation_raw: float = 1900.0,
    minimum_cell_samples: int = 5,
    voltage_endpoint_tolerance_v: float = 0.15,
    throttle_endpoint_tolerance_us: float = 10.0,
    maximum_voltage_extrapolation_v: float = 0.30,
    minimum_high_throttle_samples: int = 5,
    excluded_blackbox_intervals: tuple[tuple[float, float], ...] = (),
) -> dict[str, Any]:
    if not calibration_id.strip():
        raise ValueError("calibration_id is required")
    if acc_1g_raw <= 0.0:
        raise ValueError("acc_1g_raw must be positive")
    if voltage_knot_count < 2 or throttle_knot_count < 3:
        raise ValueError("at least two voltage and three throttle knots are required")
    if minimum_samples < 20:
        raise ValueError("minimum_samples must be at least 20")
    if required_voltage_v[0] >= required_voltage_v[1]:
        raise ValueError("required voltage range is invalid")
    if required_throttle_us[0] >= required_throttle_us[1]:
        raise ValueError("required throttle range is invalid")
    if resample_hz <= 0.0:
        raise ValueError("resample_hz must be positive")
    if armed_edge_trim_s < 0.0:
        raise ValueError("armed_edge_trim_s must be non-negative")
    if specific_force_g_range[0] <= 0.0 or specific_force_g_range[0] >= specific_force_g_range[1]:
        raise ValueError("specific force filter range is invalid")
    if maximum_gyro_deg_s <= 0.0 or motor_saturation_raw <= 0.0:
        raise ValueError("gyro and motor saturation limits must be positive")
    if minimum_cell_samples < 1:
        raise ValueError("minimum_cell_samples must be positive")
    if voltage_endpoint_tolerance_v < 0.0 or throttle_endpoint_tolerance_us < 0.0:
        raise ValueError("endpoint tolerances must be non-negative")
    if maximum_voltage_extrapolation_v < 0.0:
        raise ValueError("maximum voltage extrapolation must be non-negative")
    if minimum_high_throttle_samples < 1:
        raise ValueError("minimum high-throttle samples must be positive")

    host_csv = host_csv.expanduser().resolve()
    blackbox_csv = blackbox_csv.expanduser().resolve()
    host_rows = _read_host_rows(host_csv, host_throttle_field)
    blackbox = _read_blackbox_fields(blackbox_csv)
    time_s = (blackbox["time (us)"] - blackbox["time (us)"][0]) / 1.0e6
    if np.any(np.diff(time_s) <= 0.0):
        raise RuntimeError("Blackbox time must be strictly increasing")
    armed_intervals = _host_intervals(host_rows, "armed", 1)
    if not armed_intervals:
        raise RuntimeError("host CSV does not contain an armed interval")
    duration_s = float(time_s[-1])
    armed_interval = min(
        armed_intervals,
        key=lambda interval: abs((interval[1] - interval[0]) - duration_s),
    )
    alignment = _fit_throttle_alignment(
        host_rows,
        host_throttle_field=host_throttle_field,
        blackbox_time_s=time_s,
        blackbox_throttle=blackbox["rcCommand[3]"],
        armed_interval=armed_interval,
        min_check_us=min_check_us,
        max_pwm_us=max_pwm_us,
        idle_command=idle_command,
        search_s=alignment_search_s,
        step_s=alignment_step_s,
    )
    offset_s = float(alignment["host_minus_blackbox_s"])
    host_time, host_throttle = _host_numeric_samples(host_rows, host_throttle_field)
    throttle_us = np.interp(time_s + offset_s, host_time, host_throttle)
    voltage_v = blackbox["vbatLatest (V)"]
    accelerometer = np.column_stack(
        [blackbox[f"accSmooth[{axis}]"] for axis in range(3)]
    )
    force_m_s2 = np.linalg.norm(accelerometer, axis=1) / acc_1g_raw * 9.80665
    force_g = force_m_s2 / 9.80665
    gyro_norm_deg_s = np.linalg.norm(
        np.column_stack([blackbox[f"gyroADC[{axis}]"] for axis in range(3)]),
        axis=1,
    )
    maximum_motor_raw = np.max(
        np.column_stack([blackbox[f"motor[{index}]"] for index in range(4)]),
        axis=1,
    )

    in_armed_time = (
        (time_s + offset_s >= armed_interval[0])
        & (time_s + offset_s <= armed_interval[1])
    )
    finite = (
        np.isfinite(throttle_us)
        & np.isfinite(voltage_v)
        & np.isfinite(force_m_s2)
        & np.isfinite(gyro_norm_deg_s)
        & np.isfinite(maximum_motor_raw)
    )
    armed_edge = (
        (time_s + offset_s >= armed_interval[0] + armed_edge_trim_s)
        & (time_s + offset_s <= armed_interval[1] - armed_edge_trim_s)
    )
    in_calibration_box = (
        (voltage_v >= required_voltage_v[0])
        & (voltage_v <= required_voltage_v[1])
        & (throttle_us >= required_throttle_us[0])
        & (throttle_us <= required_throttle_us[1])
    )
    force_valid = (force_g >= specific_force_g_range[0]) & (
        force_g <= specific_force_g_range[1]
    )
    gyro_valid = gyro_norm_deg_s <= maximum_gyro_deg_s
    motors_valid = maximum_motor_raw < motor_saturation_raw
    explicit_interval_valid = np.ones(len(time_s), dtype=bool)
    for start_s, end_s in excluded_blackbox_intervals:
        explicit_interval_valid &= ~((time_s >= start_s) & (time_s <= end_s))
    selected = (
        in_armed_time
        & armed_edge
        & finite
        & in_calibration_box
        & force_valid
        & gyro_valid
        & motors_valid
        & explicit_interval_valid
    )
    filter_counts = {
        "raw_blackbox_sample_count": int(len(time_s)),
        "outside_armed_interval": int(np.count_nonzero(~in_armed_time)),
        "armed_edge_takeoff_landing_trim": int(np.count_nonzero(in_armed_time & ~armed_edge)),
        "nonfinite": int(np.count_nonzero(in_armed_time & armed_edge & ~finite)),
        "outside_required_voltage_throttle_box": int(
            np.count_nonzero(in_armed_time & armed_edge & finite & ~in_calibration_box)
        ),
        "collision_or_force_outlier": int(
            np.count_nonzero(in_armed_time & armed_edge & finite & in_calibration_box & ~force_valid)
        ),
        "high_angular_rate": int(
            np.count_nonzero(in_armed_time & armed_edge & finite & in_calibration_box & ~gyro_valid)
        ),
        "motor_saturation": int(
            np.count_nonzero(in_armed_time & armed_edge & finite & in_calibration_box & ~motors_valid)
        ),
        "operator_excluded_interval": int(np.count_nonzero(~explicit_interval_valid)),
    }
    samples = np.column_stack(
        [time_s[selected], voltage_v[selected], throttle_us[selected], force_m_s2[selected]]
    )
    samples = _resample_medians(samples, resample_hz)
    if samples.size:
        time_s = samples[:, 0]
        voltage_v = samples[:, 1]
        throttle_us = samples[:, 2]
        force_m_s2 = samples[:, 3]
    else:
        time_s = voltage_v = throttle_us = force_m_s2 = np.asarray([], dtype=float)
    if len(force_m_s2) < 20:
        raise RuntimeError("insufficient filtered effective calibration samples")

    return _fit_lut(
        time_s=time_s,
        voltage_v=voltage_v,
        throttle_us=throttle_us,
        force_m_s2=force_m_s2,
        source_indexes=np.zeros(len(force_m_s2), dtype=int),
        calibration_id=calibration_id,
        voltage_knot_count=voltage_knot_count,
        throttle_knot_count=throttle_knot_count,
        minimum_samples=minimum_samples,
        required_voltage_v=required_voltage_v,
        required_throttle_us=required_throttle_us,
        resample_hz=resample_hz,
        minimum_cell_samples=minimum_cell_samples,
        voltage_endpoint_tolerance_v=voltage_endpoint_tolerance_v,
        throttle_endpoint_tolerance_us=throttle_endpoint_tolerance_us,
        maximum_voltage_extrapolation_v=maximum_voltage_extrapolation_v,
        minimum_high_throttle_samples=minimum_high_throttle_samples,
        effective_zero_throttle_us=idle_command,
        alignment_correlation=float(alignment.get("correlation", 0.0)),
        filter_counts=filter_counts,
        provenance={
            "host_csv": _file_metadata(host_csv),
            "blackbox_csv": _file_metadata(blackbox_csv),
            "blackbox_bfl": (
                None
                if blackbox_bfl is None
                else _file_metadata(blackbox_bfl.expanduser().resolve())
            ),
            "acc_1g_raw": acc_1g_raw,
            "host_throttle_field": host_throttle_field,
            "alignment": alignment,
            "holdout_method": (
                "every_fifth_10hz_sample_within_each_source_and_throttle_band"
            ),
            "excluded_blackbox_intervals_s": [
                list(value) for value in excluded_blackbox_intervals
            ],
            "filter_limits": {
                "armed_edge_trim_s": armed_edge_trim_s,
                "specific_force_g": list(specific_force_g_range),
                "maximum_gyro_deg_s": maximum_gyro_deg_s,
                "motor_saturation_raw": motor_saturation_raw,
            },
        },
    )


def calibrate_manifest(
    *,
    manifest_path: Path,
    decoder_path: Path,
    calibration_id: str,
    host_throttle_field: str,
    acc_1g_raw: float,
    min_check_us: float,
    max_pwm_us: float,
    idle_command: float,
    alignment_search_s: float,
    alignment_step_s: float,
    voltage_knot_count: int,
    throttle_knot_count: int,
    minimum_samples: int,
    required_voltage_v: tuple[float, float],
    required_throttle_us: tuple[float, float],
    resample_hz: float = 10.0,
    armed_edge_trim_s: float = 2.0,
    specific_force_g_range: tuple[float, float] = (0.3, 2.5),
    maximum_gyro_deg_s: float = 120.0,
    motor_saturation_raw: float = 1900.0,
    minimum_cell_samples: int = 5,
    voltage_endpoint_tolerance_v: float = 0.15,
    throttle_endpoint_tolerance_us: float = 10.0,
    maximum_voltage_extrapolation_v: float = 0.30,
    minimum_high_throttle_samples: int = 5,
    excluded_blackbox_intervals: tuple[tuple[float, float], ...] = (),
) -> dict[str, Any]:
    if excluded_blackbox_intervals:
        raise ValueError("manifest sources must declare their own cutoff intervals")
    if not calibration_id.strip():
        raise ValueError("calibration_id is required")
    if acc_1g_raw <= 0.0:
        raise ValueError("acc_1g_raw must be positive")
    if voltage_knot_count < 2 or throttle_knot_count < 3:
        raise ValueError("at least two voltage and three throttle knots are required")
    if minimum_samples < 20:
        raise ValueError("minimum_samples must be at least 20")
    if required_voltage_v[0] >= required_voltage_v[1]:
        raise ValueError("required voltage range is invalid")
    if required_throttle_us[0] >= required_throttle_us[1]:
        raise ValueError("required throttle range is invalid")
    if resample_hz <= 0.0 or armed_edge_trim_s < 0.0:
        raise ValueError("resampling rate and ARM edge trim are invalid")
    if (
        specific_force_g_range[0] <= 0.0
        or specific_force_g_range[0] >= specific_force_g_range[1]
    ):
        raise ValueError("specific force filter range is invalid")
    if maximum_gyro_deg_s <= 0.0 or motor_saturation_raw <= 0.0:
        raise ValueError("gyro and motor saturation limits must be positive")
    if minimum_cell_samples < 1 or minimum_high_throttle_samples < 1:
        raise ValueError("coverage sample requirements must be positive")
    if (
        voltage_endpoint_tolerance_v < 0.0
        or throttle_endpoint_tolerance_us < 0.0
        or maximum_voltage_extrapolation_v < 0.0
    ):
        raise ValueError("coverage tolerances must be non-negative")
    manifest_path = manifest_path.expanduser().resolve()
    decoder_path = decoder_path.expanduser().resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if int(manifest.get("schema_version", 0)) != 1:
        raise RuntimeError("thrust source manifest schema_version must be 1")
    raw_sources = manifest.get("sources")
    if not isinstance(raw_sources, list):
        raise RuntimeError("thrust source manifest sources must be a list")
    sources = [source for source in raw_sources if source.get("include", True)]
    if not sources:
        raise RuntimeError("thrust source manifest has no included sources")
    if not decoder_path.is_file():
        raise RuntimeError(f"Blackbox decoder does not exist: {decoder_path}")

    with tempfile.TemporaryDirectory(prefix="betaflight_thrust_fit_") as directory:
        decoded_root = Path(directory)
        _decode_manifest_sources(decoder_path, sources, decoded_root)
        extracted = [
            _extract_manifest_source(
                source=source,
                decoded_root=decoded_root,
                host_throttle_field=host_throttle_field,
                acc_1g_raw=acc_1g_raw,
                min_check_us=min_check_us,
                max_pwm_us=max_pwm_us,
                idle_command=idle_command,
                alignment_search_s=alignment_search_s,
                alignment_step_s=alignment_step_s,
                required_voltage_v=required_voltage_v,
                required_throttle_us=required_throttle_us,
                resample_hz=resample_hz,
                armed_edge_trim_s=armed_edge_trim_s,
                specific_force_g_range=specific_force_g_range,
                maximum_gyro_deg_s=maximum_gyro_deg_s,
                motor_saturation_raw=motor_saturation_raw,
            )
            for source in sources
        ]

    sample_groups = [result.pop("_samples") for result in extracted]
    nonempty = [index for index, samples in enumerate(sample_groups) if len(samples)]
    if not nonempty:
        raise RuntimeError("manifest sources contain no filtered calibration samples")
    combined = []
    source_indexes = []
    cursor_s = 0.0
    for source_index in nonempty:
        samples = sample_groups[source_index].copy()
        samples[:, 0] = samples[:, 0] - samples[0, 0] + cursor_s
        cursor_s = float(samples[-1, 0]) + 1.0
        combined.append(samples)
        source_indexes.append(np.full(len(samples), source_index, dtype=int))
    samples = np.concatenate(combined, axis=0)
    indexes = np.concatenate(source_indexes, axis=0)
    filter_counts: dict[str, int] = {}
    for result in extracted:
        for key, value in result["filter_counts"].items():
            filter_counts[key] = filter_counts.get(key, 0) + int(value)
    correlations = [float(result["alignment"]["correlation"]) for result in extracted]
    return _fit_lut(
        time_s=samples[:, 0],
        voltage_v=samples[:, 1],
        throttle_us=samples[:, 2],
        force_m_s2=samples[:, 3],
        source_indexes=indexes,
        calibration_id=calibration_id,
        voltage_knot_count=voltage_knot_count,
        throttle_knot_count=throttle_knot_count,
        minimum_samples=minimum_samples,
        required_voltage_v=required_voltage_v,
        required_throttle_us=required_throttle_us,
        resample_hz=resample_hz,
        minimum_cell_samples=minimum_cell_samples,
        voltage_endpoint_tolerance_v=voltage_endpoint_tolerance_v,
        throttle_endpoint_tolerance_us=throttle_endpoint_tolerance_us,
        maximum_voltage_extrapolation_v=maximum_voltage_extrapolation_v,
        minimum_high_throttle_samples=minimum_high_throttle_samples,
        effective_zero_throttle_us=idle_command,
        alignment_correlation=min(correlations),
        filter_counts=filter_counts,
        provenance={
            "manifest": _file_metadata(manifest_path),
            "decoder": _file_metadata(decoder_path),
            "source_count": len(extracted),
            "sources": extracted,
            "acc_1g_raw": acc_1g_raw,
            "host_throttle_field": host_throttle_field,
            "holdout_method": (
                "every_fifth_10hz_sample_within_each_source_and_throttle_band"
            ),
            "filter_limits": {
                "armed_edge_trim_s": armed_edge_trim_s,
                "specific_force_g": list(specific_force_g_range),
                "maximum_gyro_deg_s": maximum_gyro_deg_s,
                "motor_saturation_raw": motor_saturation_raw,
            },
        },
    )


def _decode_manifest_sources(
    decoder_path: Path,
    sources: list[dict[str, Any]],
    output_root: Path,
) -> None:
    bfl_paths = [(ROOT / str(source["blackbox_bfl"])).resolve() for source in sources]
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


def _extract_manifest_source(
    *,
    source: dict[str, Any],
    decoded_root: Path,
    host_throttle_field: str,
    acc_1g_raw: float,
    min_check_us: float,
    max_pwm_us: float,
    idle_command: float,
    alignment_search_s: float,
    alignment_step_s: float,
    required_voltage_v: tuple[float, float],
    required_throttle_us: tuple[float, float],
    resample_hz: float,
    armed_edge_trim_s: float,
    specific_force_g_range: tuple[float, float],
    maximum_gyro_deg_s: float,
    motor_saturation_raw: float,
) -> dict[str, Any]:
    source_id = str(source.get("id", "")).strip()
    if not source_id:
        raise RuntimeError("manifest source id is required")
    bfl_path = (ROOT / str(source["blackbox_bfl"])).resolve()
    host_path = (ROOT / str(source["host_csv"])).resolve()
    decoded_path = decoded_root / f"{bfl_path.stem}.01.csv"
    blackbox = _read_blackbox_fields(decoded_path)
    time_s = (blackbox["time (us)"] - blackbox["time (us)"][0]) / 1.0e6
    if np.any(np.diff(time_s) <= 0.0):
        raise RuntimeError(f"{source_id}: Blackbox time must be strictly increasing")
    host_rows = _read_host_rows(host_path, host_throttle_field)
    armed_intervals = _host_intervals(host_rows, "armed", 1)
    arm_index = int(source.get("host_arm_interval_index", 0))
    if arm_index < 0 or arm_index >= len(armed_intervals):
        raise RuntimeError(f"{source_id}: host ARM interval index is out of range")
    armed_interval = armed_intervals[arm_index]
    alignment = _fit_throttle_alignment(
        host_rows,
        host_throttle_field=host_throttle_field,
        blackbox_time_s=time_s,
        blackbox_throttle=blackbox["rcCommand[3]"],
        armed_interval=armed_interval,
        min_check_us=min_check_us,
        max_pwm_us=max_pwm_us,
        idle_command=idle_command,
        search_s=alignment_search_s,
        step_s=alignment_step_s,
    )
    offset_s = float(alignment["host_minus_blackbox_s"])
    host_time, host_throttle = _host_numeric_samples(host_rows, host_throttle_field)
    throttle_us = np.interp(time_s + offset_s, host_time, host_throttle)
    voltage_v = blackbox["vbatLatest (V)"]
    force_m_s2 = (
        np.linalg.norm(
            np.column_stack(
                [blackbox[f"accSmooth[{axis}]"] for axis in range(3)]
            ),
            axis=1,
        )
        / acc_1g_raw
        * 9.80665
    )
    force_g = force_m_s2 / 9.80665
    gyro_norm_deg_s = np.linalg.norm(
        np.column_stack([blackbox[f"gyroADC[{axis}]"] for axis in range(3)]),
        axis=1,
    )
    maximum_motor_raw = np.max(
        np.column_stack([blackbox[f"motor[{index}]"] for index in range(4)]),
        axis=1,
    )
    in_armed_time = (
        (time_s + offset_s >= armed_interval[0])
        & (time_s + offset_s <= armed_interval[1])
    )
    armed_edge = (
        (time_s + offset_s >= armed_interval[0] + armed_edge_trim_s)
        & (time_s + offset_s <= armed_interval[1] - armed_edge_trim_s)
    )
    finite = (
        np.isfinite(throttle_us)
        & np.isfinite(voltage_v)
        & np.isfinite(force_m_s2)
        & np.isfinite(gyro_norm_deg_s)
        & np.isfinite(maximum_motor_raw)
    )
    in_calibration_box = (
        (voltage_v >= required_voltage_v[0])
        & (voltage_v <= required_voltage_v[1])
        & (throttle_us >= required_throttle_us[0])
        & (throttle_us <= required_throttle_us[1])
    )
    force_valid = (force_g >= specific_force_g_range[0]) & (
        force_g <= specific_force_g_range[1]
    )
    gyro_valid = gyro_norm_deg_s <= maximum_gyro_deg_s
    motors_valid = maximum_motor_raw < motor_saturation_raw
    cutoff_s = source.get("max_blackbox_time_s")
    operator_valid = np.ones(len(time_s), dtype=bool)
    if cutoff_s is not None:
        operator_valid &= time_s < float(cutoff_s)
    selected = (
        in_armed_time
        & armed_edge
        & finite
        & in_calibration_box
        & force_valid
        & gyro_valid
        & motors_valid
        & operator_valid
    )
    filter_counts = {
        "raw_blackbox_sample_count": int(len(time_s)),
        "outside_armed_interval": int(np.count_nonzero(~in_armed_time)),
        "armed_edge_takeoff_landing_trim": int(np.count_nonzero(in_armed_time & ~armed_edge)),
        "nonfinite": int(np.count_nonzero(in_armed_time & armed_edge & ~finite)),
        "outside_required_voltage_throttle_box": int(
            np.count_nonzero(in_armed_time & armed_edge & finite & ~in_calibration_box)
        ),
        "collision_or_force_outlier": int(
            np.count_nonzero(in_armed_time & armed_edge & finite & in_calibration_box & ~force_valid)
        ),
        "high_angular_rate": int(
            np.count_nonzero(in_armed_time & armed_edge & finite & in_calibration_box & ~gyro_valid)
        ),
        "motor_saturation": int(
            np.count_nonzero(in_armed_time & armed_edge & finite & in_calibration_box & ~motors_valid)
        ),
        "operator_excluded_interval": int(np.count_nonzero(~operator_valid)),
    }
    samples = np.column_stack(
        [time_s[selected], voltage_v[selected], throttle_us[selected], force_m_s2[selected]]
    )
    samples = _resample_medians(samples, resample_hz)
    return {
        "id": source_id,
        "blackbox_bfl": _file_metadata(bfl_path),
        "host_csv": _file_metadata(host_path),
        "host_arm_interval_index": arm_index,
        "host_arm_interval_s": list(armed_interval),
        "max_blackbox_time_s": cutoff_s,
        "filtered_sample_count": int(len(samples)),
        "alignment": alignment,
        "filter_counts": filter_counts,
        "_samples": samples,
    }


def _fit_lut(
    *,
    time_s: np.ndarray,
    voltage_v: np.ndarray,
    throttle_us: np.ndarray,
    force_m_s2: np.ndarray,
    source_indexes: np.ndarray,
    calibration_id: str,
    voltage_knot_count: int,
    throttle_knot_count: int,
    minimum_samples: int,
    required_voltage_v: tuple[float, float],
    required_throttle_us: tuple[float, float],
    resample_hz: float,
    minimum_cell_samples: int,
    voltage_endpoint_tolerance_v: float,
    throttle_endpoint_tolerance_us: float,
    maximum_voltage_extrapolation_v: float,
    minimum_high_throttle_samples: int,
    effective_zero_throttle_us: float,
    alignment_correlation: float,
    filter_counts: dict[str, int],
    provenance: dict[str, Any],
) -> dict[str, Any]:
    if not (
        len(time_s)
        == len(voltage_v)
        == len(throttle_us)
        == len(force_m_s2)
        == len(source_indexes)
    ):
        raise ValueError("calibration sample arrays must have equal length")
    if len(force_m_s2) < 20:
        raise RuntimeError("insufficient filtered effective calibration samples")

    voltage_knots = np.linspace(required_voltage_v[0], required_voltage_v[1], voltage_knot_count)
    throttle_knots = np.linspace(
        required_throttle_us[0], required_throttle_us[1], throttle_knot_count
    )
    throttle_edges = _centered_bin_edges(throttle_knots)
    throttle_bands = np.digitize(
        throttle_us,
        throttle_edges[1:-1],
        right=False,
    )
    holdout = np.zeros(len(force_m_s2), dtype=bool)
    for source_index in np.unique(source_indexes):
        for throttle_band in range(len(throttle_knots)):
            indexes = np.flatnonzero(
                (source_indexes == source_index)
                & (throttle_bands == throttle_band)
            )
            holdout[indexes[::5]] = True
    training = ~holdout
    force_grid, fit_diagnostics = _fit_voltage_compensated_grid(
        voltage_v[training],
        throttle_us[training],
        force_m_s2[training],
        voltage_knots,
        throttle_knots,
        effective_zero_throttle_us=effective_zero_throttle_us,
    )
    predicted = _interpolate_grid(
        voltage_v[holdout],
        throttle_us[holdout],
        voltage_knots,
        throttle_knots,
        force_grid,
    )
    relative_error = np.abs(predicted - force_m_s2[holdout]) / np.maximum(
        force_m_s2[holdout], 0.5
    )
    median_error = float(np.median(relative_error))
    p95_error = float(np.percentile(relative_error, 95))
    static_force = _interpolate_grid(
        voltage_v,
        throttle_us,
        voltage_knots,
        throttle_knots,
        force_grid,
    )
    dynamics = _fit_first_order_time_constant(time_s, static_force, force_m_s2)
    voltage_edges = np.linspace(required_voltage_v[0], required_voltage_v[1], 4)
    coverage_counts, _, _ = np.histogram2d(
        voltage_v,
        throttle_us,
        bins=(voltage_edges, throttle_edges),
    )
    coverage_counts = coverage_counts.astype(int)
    holdout_metrics = _binned_error_metrics(
        voltage_v=voltage_v[holdout],
        throttle_us=throttle_us[holdout],
        relative_error=relative_error,
        voltage_edges=voltage_edges,
        throttle_edges=throttle_edges,
    )
    all_predicted = _interpolate_grid(
        voltage_v,
        throttle_us,
        voltage_knots,
        throttle_knots,
        force_grid,
    )
    all_relative_error = np.abs(all_predicted - force_m_s2) / np.maximum(
        force_m_s2, 0.5
    )
    high_throttle = throttle_us >= throttle_edges[-2]
    high_throttle_count = int(np.count_nonzero(high_throttle))
    high_throttle_median_error = _optional_percentile(
        all_relative_error[high_throttle], 50
    )
    high_throttle_p95_error = _optional_percentile(
        all_relative_error[high_throttle], 95
    )
    holdout_high_throttle = throttle_us[holdout] >= throttle_edges[-2]
    high_throttle_holdout_count = int(np.count_nonzero(holdout_high_throttle))
    high_throttle_holdout_median_error = _optional_percentile(
        relative_error[holdout_high_throttle], 50
    )
    high_throttle_holdout_p95_error = _optional_percentile(
        relative_error[holdout_high_throttle], 95
    )
    cells_covered = bool(np.all(coverage_counts >= minimum_cell_samples))
    throttle_band_counts = np.sum(coverage_counts, axis=0).astype(int)
    throttle_bands_covered = bool(np.all(throttle_band_counts > 0))
    direct_voltage_coverage = bool(
        float(np.min(voltage_v)) <= required_voltage_v[0] + voltage_endpoint_tolerance_v
        and float(np.max(voltage_v)) >= required_voltage_v[1] - voltage_endpoint_tolerance_v
    )
    voltage_extrapolation_v = [
        max(0.0, float(np.min(voltage_v)) - required_voltage_v[0]),
        max(0.0, required_voltage_v[1] - float(np.max(voltage_v))),
    ]
    voltage_supported = max(voltage_extrapolation_v) <= maximum_voltage_extrapolation_v
    throttle_covered = bool(
        float(np.min(throttle_us)) <= required_throttle_us[0] + throttle_endpoint_tolerance_us
        and float(np.max(throttle_us)) >= required_throttle_us[1] - throttle_endpoint_tolerance_us
    )
    high_throttle_supported = bool(
        high_throttle_count >= minimum_high_throttle_samples
        and high_throttle_p95_error is not None
        and high_throttle_p95_error <= 0.25
        and high_throttle_holdout_count >= 1
        and high_throttle_holdout_p95_error is not None
        and high_throttle_holdout_p95_error <= 0.25
    )
    sample_count_ok = len(force_m_s2) >= minimum_samples and int(np.count_nonzero(holdout)) >= 100
    passed = bool(
        voltage_supported
        and throttle_covered
        and throttle_bands_covered
        and high_throttle_supported
        and sample_count_ok
        and alignment_correlation >= 0.8
        and median_error <= 0.10
        and p95_error <= 0.20
    )
    blockers = []
    if not voltage_supported:
        blockers.append("voltage_extrapolation_exceeds_limit")
    if not throttle_covered:
        blockers.append("throttle_coverage_insufficient")
    if not throttle_bands_covered:
        blockers.append("throttle_anchor_band_missing")
    if high_throttle_count < minimum_high_throttle_samples:
        blockers.append("high_throttle_anchor_insufficient")
    elif high_throttle_p95_error is None or high_throttle_p95_error > 0.25:
        blockers.append("high_throttle_support_error_exceeds_25_percent")
    elif (
        high_throttle_holdout_count < 1
        or high_throttle_holdout_p95_error is None
        or high_throttle_holdout_p95_error > 0.25
    ):
        blockers.append("high_throttle_holdout_validation_insufficient")
    if not sample_count_ok:
        blockers.append("sample_count_insufficient")
    if alignment_correlation < 0.8:
        blockers.append("throttle_alignment_weak")
    if median_error > 0.10:
        blockers.append("median_relative_error_exceeds_10_percent")
    if p95_error > 0.20:
        blockers.append("p95_relative_error_exceeds_20_percent")

    return {
        "schema_version": THRUST_LUT_SCHEMA_VERSION,
        "model_type": THRUST_LUT_MODEL_TYPE,
        "calibration_id": calibration_id.strip(),
        "created_unix_s": time.time(),
        "voltage_v": voltage_knots.tolist(),
        "throttle_us": throttle_knots.tolist(),
        "specific_force_m_s2": force_grid.tolist(),
        "fit": fit_diagnostics,
        "dynamics": dynamics,
        "validation": {
            "passed": passed,
            "blockers": blockers,
            "sample_count": int(np.count_nonzero(holdout)),
            "total_aligned_sample_count": int(len(force_m_s2)),
            "median_relative_error": median_error,
            "p95_relative_error": p95_error,
            "required_voltage_coverage_v": list(required_voltage_v),
            "observed_voltage_coverage_v": [
                float(np.min(voltage_v)),
                float(np.max(voltage_v)),
            ],
            "required_throttle_coverage_us": list(required_throttle_us),
            "observed_throttle_coverage_us": [
                float(np.min(throttle_us)),
                float(np.max(throttle_us)),
            ],
            "coverage_policy": "bounded_physics_constrained_sparse_surface_v1",
            "direct_voltage_endpoint_coverage": direct_voltage_coverage,
            "voltage_extrapolation_v": voltage_extrapolation_v,
            "maximum_voltage_extrapolation_v": maximum_voltage_extrapolation_v,
            "alignment_correlation": alignment_correlation,
            "effective_sample_rate_hz": resample_hz,
            "voltage_band_edges_v": voltage_edges.tolist(),
            "throttle_node_edges_us": throttle_edges.tolist(),
            "three_by_five_sample_counts": coverage_counts.tolist(),
            "three_by_five_complete": cells_covered,
            "minimum_cell_samples": minimum_cell_samples,
            "throttle_band_sample_counts": throttle_band_counts.tolist(),
            "high_throttle_band_min_us": float(throttle_edges[-2]),
            "high_throttle_sample_count": high_throttle_count,
            "minimum_high_throttle_samples": minimum_high_throttle_samples,
            "maximum_high_throttle_p95_relative_error": 0.25,
            "high_throttle_support_median_relative_error": high_throttle_median_error,
            "high_throttle_support_p95_relative_error": high_throttle_p95_error,
            "high_throttle_holdout_sample_count": high_throttle_holdout_count,
            "high_throttle_holdout_median_relative_error": (
                high_throttle_holdout_median_error
            ),
            "high_throttle_holdout_p95_relative_error": (
                high_throttle_holdout_p95_error
            ),
            "holdout_three_by_five_sample_counts": holdout_metrics["counts"],
            "holdout_three_by_five_median_relative_error": holdout_metrics[
                "median_relative_error"
            ],
            "holdout_three_by_five_p95_relative_error": holdout_metrics[
                "p95_relative_error"
            ],
            "filter_counts": filter_counts,
        },
        "provenance": provenance,
    }


def _read_blackbox_fields(path: Path) -> dict[str, np.ndarray]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        header = [value.strip() for value in next(csv.reader(stream))]
    resolved = {field: _resolve_header_field(header, field) for field in REQUIRED_BLACKBOX_FIELDS}
    missing = [field for field, actual in resolved.items() if actual is None]
    if missing:
        raise RuntimeError(f"Blackbox CSV is missing fields: {', '.join(missing)}")
    indexes = [header.index(str(resolved[field])) for field in REQUIRED_BLACKBOX_FIELDS]
    values = np.loadtxt(path, delimiter=",", skiprows=1, usecols=indexes)
    if values.ndim != 2 or values.shape[0] < 20:
        raise RuntimeError("Blackbox CSV has insufficient rows")
    return {field: values[:, index] for index, field in enumerate(REQUIRED_BLACKBOX_FIELDS)}


def _resolve_header_field(header: list[str], field: str) -> str | None:
    if field in header:
        return field
    matches = [value for value in header if value.startswith(f"{field} ")]
    return matches[0] if len(matches) == 1 else None


def _parse_intervals(values: list[str]) -> tuple[tuple[float, float], ...]:
    result = []
    for raw in values:
        start_text, separator, end_text = str(raw).partition(":")
        if not separator:
            raise ValueError(f"invalid exclusion interval: {raw}")
        start_s = float(start_text)
        end_s = float(end_text)
        if not math.isfinite(start_s) or not math.isfinite(end_s) or start_s < 0.0 or end_s <= start_s:
            raise ValueError(f"invalid exclusion interval: {raw}")
        result.append((start_s, end_s))
    return tuple(result)


def _resample_medians(samples: np.ndarray, rate_hz: float) -> np.ndarray:
    if samples.ndim != 2 or samples.shape[1] != 4:
        raise ValueError("samples must contain time, voltage, throttle, and force")
    if len(samples) == 0:
        return samples.copy()
    buckets = np.floor(samples[:, 0] * rate_hz).astype(np.int64)
    return np.asarray(
        [np.median(samples[buckets == bucket], axis=0) for bucket in np.unique(buckets)],
        dtype=float,
    )


def _centered_bin_edges(knots: np.ndarray) -> np.ndarray:
    midpoints = 0.5 * (knots[:-1] + knots[1:])
    return np.concatenate(([knots[0]], midpoints, [np.nextafter(knots[-1], math.inf)]))


def _fit_first_order_time_constant(
    time_s: np.ndarray,
    static_force_m_s2: np.ndarray,
    measured_force_m_s2: np.ndarray,
) -> dict[str, Any]:
    if len(time_s) < 20:
        raise RuntimeError("insufficient samples for propulsion time-constant fit")
    candidates = np.geomspace(0.01, 0.50, 80)
    best_tau = None
    best_rmse = math.inf
    best_prediction = None
    for tau_s in candidates:
        prediction = np.empty_like(measured_force_m_s2)
        prediction[0] = measured_force_m_s2[0]
        for index in range(1, len(prediction)):
            dt_s = float(time_s[index] - time_s[index - 1])
            if dt_s <= 0.0 or dt_s > 0.5:
                prediction[index] = measured_force_m_s2[index]
                continue
            alpha = 1.0 - math.exp(-dt_s / float(tau_s))
            prediction[index] = prediction[index - 1] + alpha * (
                static_force_m_s2[index] - prediction[index - 1]
            )
        rmse = float(np.sqrt(np.mean((prediction - measured_force_m_s2) ** 2)))
        if rmse < best_rmse:
            best_tau = float(tau_s)
            best_rmse = rmse
            best_prediction = prediction
    assert best_tau is not None and best_prediction is not None
    baseline_rmse = float(
        np.sqrt(np.mean((static_force_m_s2 - measured_force_m_s2) ** 2))
    )
    return {
        "model": "first_order_specific_force",
        "first_order_time_constant_s": best_tau,
        "fit_sample_count": int(len(time_s)),
        "rmse_m_s2": best_rmse,
        "instantaneous_rmse_m_s2": baseline_rmse,
    }


def _host_numeric_samples(
    rows: list[dict[str, str]], field: str
) -> tuple[np.ndarray, np.ndarray]:
    values = []
    for row in rows:
        try:
            timestamp = float(row["elapsed_s"])
            throttle = float(row[field])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(timestamp) and math.isfinite(throttle):
            values.append((timestamp, throttle))
    if len(values) < 2:
        raise RuntimeError("host CSV has insufficient finite throttle samples")
    result = np.asarray(values, dtype=float)
    order = np.argsort(result[:, 0])
    return result[order, 0], result[order, 1]


def _quantile_knots(values: np.ndarray, count: int) -> np.ndarray:
    knots = np.quantile(values, np.linspace(0.0, 1.0, count))
    return np.unique(np.round(knots.astype(float), decimals=6))


def _fit_monotonic_grid(
    voltage_v: np.ndarray,
    throttle_us: np.ndarray,
    force_m_s2: np.ndarray,
    voltage_knots: np.ndarray,
    throttle_knots: np.ndarray,
) -> tuple[np.ndarray, dict[str, Any]]:
    return _fit_voltage_compensated_grid(
        voltage_v,
        throttle_us,
        force_m_s2,
        voltage_knots,
        throttle_knots,
        effective_zero_throttle_us=1000.0,
    )


def _fit_voltage_compensated_grid(
    voltage_v: np.ndarray,
    throttle_us: np.ndarray,
    force_m_s2: np.ndarray,
    voltage_knots: np.ndarray,
    throttle_knots: np.ndarray,
    *,
    effective_zero_throttle_us: float,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Fit force against voltage-scaled motor input with balanced throttle evidence."""
    if len(force_m_s2) < 20:
        raise RuntimeError("insufficient samples for physics-constrained thrust fit")
    if effective_zero_throttle_us >= float(throttle_knots[0]):
        raise ValueError("effective zero throttle must be below the LUT range")

    effective_input = voltage_v * (throttle_us - effective_zero_throttle_us)
    input_center = float(np.median(effective_input))
    input_scale = max(float(np.ptp(effective_input)), 1.0)
    normalized_input = (effective_input - input_center) / input_scale

    # Equalize broad throttle regions so thousands of hover samples do not erase
    # the measured 1500 us endpoint. Empty regions do not contribute synthetic data.
    balance_edges = np.linspace(
        float(throttle_knots[0]),
        float(throttle_knots[-1]),
        max(13, 2 * len(throttle_knots) + 1) + 1,
    )
    balance_indexes = np.clip(
        np.digitize(throttle_us, balance_edges) - 1,
        0,
        len(balance_edges) - 2,
    )
    balance_counts = np.bincount(
        balance_indexes,
        minlength=len(balance_edges) - 1,
    )
    sample_weights = 1.0 / np.maximum(balance_counts[balance_indexes], 1)
    design = np.column_stack(
        (
            np.ones(len(normalized_input)),
            normalized_input,
            normalized_input**2,
        )
    )
    sqrt_weights = np.sqrt(sample_weights)
    coefficients = np.linalg.lstsq(
        design * sqrt_weights[:, None],
        force_m_s2 * sqrt_weights,
        rcond=None,
    )[0]

    knot_voltage, knot_throttle = np.meshgrid(
        voltage_knots,
        throttle_knots,
        indexing="ij",
    )
    knot_input = knot_voltage * (knot_throttle - effective_zero_throttle_us)
    knot_normalized = (knot_input - input_center) / input_scale
    grid = (
        coefficients[0]
        + coefficients[1] * knot_normalized
        + coefficients[2] * knot_normalized**2
    )
    grid = np.maximum.accumulate(grid, axis=0)
    grid = np.maximum.accumulate(grid, axis=1)
    for column in range(1, grid.shape[1]):
        grid[:, column] = np.maximum(grid[:, column], grid[:, column - 1] + 1.0e-6)
    return grid, {
        "method": "voltage_scaled_effective_input_quadratic_v1",
        "effective_input": "voltage_v * (throttle_us - effective_zero_throttle_us)",
        "effective_zero_throttle_us": float(effective_zero_throttle_us),
        "polynomial_degree": 2,
        "input_center": input_center,
        "input_scale": input_scale,
        "coefficients": coefficients.tolist(),
        "throttle_balance_edges_us": balance_edges.tolist(),
        "throttle_balance_sample_counts": balance_counts.astype(int).tolist(),
    }


def _binned_error_metrics(
    *,
    voltage_v: np.ndarray,
    throttle_us: np.ndarray,
    relative_error: np.ndarray,
    voltage_edges: np.ndarray,
    throttle_edges: np.ndarray,
) -> dict[str, list[list[int | float | None]]]:
    row_indexes = np.digitize(voltage_v, voltage_edges[1:-1], right=False)
    column_indexes = np.digitize(throttle_us, throttle_edges[1:-1], right=False)
    counts: list[list[int | float | None]] = []
    medians: list[list[int | float | None]] = []
    p95_values: list[list[int | float | None]] = []
    for row in range(len(voltage_edges) - 1):
        count_row: list[int | float | None] = []
        median_row: list[int | float | None] = []
        p95_row: list[int | float | None] = []
        for column in range(len(throttle_edges) - 1):
            selected = (row_indexes == row) & (column_indexes == column)
            values = relative_error[selected]
            count_row.append(int(len(values)))
            median_row.append(_optional_percentile(values, 50))
            p95_row.append(_optional_percentile(values, 95))
        counts.append(count_row)
        medians.append(median_row)
        p95_values.append(p95_row)
    return {
        "counts": counts,
        "median_relative_error": medians,
        "p95_relative_error": p95_values,
    }


def _optional_percentile(values: np.ndarray, percentile: float) -> float | None:
    if len(values) == 0:
        return None
    return float(np.percentile(values, percentile))


def _interpolate_grid(
    voltage_v: np.ndarray,
    throttle_us: np.ndarray,
    voltage_knots: np.ndarray,
    throttle_knots: np.ndarray,
    force_grid: np.ndarray,
) -> np.ndarray:
    result = np.empty(len(voltage_v), dtype=float)
    bounded_voltage = np.clip(voltage_v, voltage_knots[0], voltage_knots[-1])
    bounded_throttle = np.clip(throttle_us, throttle_knots[0], throttle_knots[-1])
    for index, (voltage, throttle) in enumerate(zip(bounded_voltage, bounded_throttle)):
        row = np.asarray(
            [
                np.interp(voltage, voltage_knots, force_grid[:, column])
                for column in range(len(throttle_knots))
            ]
        )
        result[index] = np.interp(throttle, throttle_knots, row)
    return result


def _file_metadata(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise RuntimeError(f"evidence file does not exist: {path}")
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


if __name__ == "__main__":
    main()
