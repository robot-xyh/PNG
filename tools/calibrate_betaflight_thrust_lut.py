#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
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
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a hash-bound voltage/throttle/specific-force LUT from flight evidence."
    )
    parser.add_argument("--host-csv", required=True)
    parser.add_argument("--blackbox-csv", required=True)
    parser.add_argument("--blackbox-bfl", default="")
    parser.add_argument("--output", required=True)
    parser.add_argument("--calibration-id", required=True)
    parser.add_argument("--host-throttle-field", default="rc_in_ch4")
    parser.add_argument("--acc-1g-raw", type=float, default=2048.0)
    parser.add_argument("--min-check-us", type=float, default=1050.0)
    parser.add_argument("--max-pwm-us", type=float, default=2000.0)
    parser.add_argument("--idle-command", type=float, default=1000.0)
    parser.add_argument("--alignment-search-s", type=float, default=1.0)
    parser.add_argument("--alignment-step-s", type=float, default=0.001)
    parser.add_argument("--voltage-knots", type=int, default=5)
    parser.add_argument("--throttle-knots", type=int, default=7)
    parser.add_argument("--minimum-samples", type=int, default=500)
    parser.add_argument("--required-voltage-min-v", type=float, default=20.0)
    parser.add_argument("--required-voltage-max-v", type=float, default=25.2)
    parser.add_argument("--required-throttle-min-us", type=float, default=1200.0)
    parser.add_argument("--required-throttle-max-us", type=float, default=1500.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output = calibrate(
        host_csv=Path(args.host_csv),
        blackbox_csv=Path(args.blackbox_csv),
        blackbox_bfl=Path(args.blackbox_bfl) if args.blackbox_bfl else None,
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

    in_armed_time = (
        (time_s + offset_s >= armed_interval[0])
        & (time_s + offset_s <= armed_interval[1])
    )
    finite = (
        np.isfinite(throttle_us)
        & np.isfinite(voltage_v)
        & np.isfinite(force_m_s2)
        & (force_m_s2 > 0.1)
    )
    selected = in_armed_time & finite
    throttle_us = throttle_us[selected]
    voltage_v = voltage_v[selected]
    force_m_s2 = force_m_s2[selected]
    if len(force_m_s2) < 20:
        raise RuntimeError("insufficient aligned finite calibration samples")

    holdout = np.arange(len(force_m_s2)) % 5 == 0
    training = ~holdout
    voltage_knots = _quantile_knots(voltage_v[training], voltage_knot_count)
    throttle_knots = _quantile_knots(throttle_us[training], throttle_knot_count)
    if len(voltage_knots) < 2 or len(throttle_knots) < 3:
        raise RuntimeError("insufficient independent voltage/throttle excitation")
    force_grid = _fit_monotonic_grid(
        voltage_v[training],
        throttle_us[training],
        force_m_s2[training],
        voltage_knots,
        throttle_knots,
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
    voltage_covered = bool(
        float(np.min(voltage_v)) <= required_voltage_v[0]
        and float(np.max(voltage_v)) >= required_voltage_v[1]
    )
    throttle_covered = bool(
        float(np.min(throttle_us)) <= required_throttle_us[0]
        and float(np.max(throttle_us)) >= required_throttle_us[1]
    )
    alignment_correlation = float(alignment.get("correlation", 0.0))
    sample_count_ok = len(force_m_s2) >= minimum_samples and int(np.count_nonzero(holdout)) >= 100
    passed = bool(
        voltage_covered
        and throttle_covered
        and sample_count_ok
        and alignment_correlation >= 0.8
        and median_error <= 0.10
        and p95_error <= 0.20
    )
    blockers = []
    if not voltage_covered:
        blockers.append("voltage_coverage_insufficient")
    if not throttle_covered:
        blockers.append("throttle_coverage_insufficient")
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
            "alignment_correlation": alignment_correlation,
        },
        "provenance": {
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
        },
    }


def _read_blackbox_fields(path: Path) -> dict[str, np.ndarray]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        header = [value.strip() for value in next(csv.reader(stream))]
    missing = [field for field in REQUIRED_BLACKBOX_FIELDS if field not in header]
    if missing:
        raise RuntimeError(f"Blackbox CSV is missing fields: {', '.join(missing)}")
    indexes = [header.index(field) for field in REQUIRED_BLACKBOX_FIELDS]
    values = np.loadtxt(path, delimiter=",", skiprows=1, usecols=indexes)
    if values.ndim != 2 or values.shape[0] < 20:
        raise RuntimeError("Blackbox CSV has insufficient rows")
    return {field: values[:, index] for index, field in enumerate(REQUIRED_BLACKBOX_FIELDS)}


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
) -> np.ndarray:
    voltage_scale = max(float(np.ptp(voltage_v)), 0.1)
    throttle_scale = max(float(np.ptp(throttle_us)), 10.0)
    grid = np.empty((len(voltage_knots), len(throttle_knots)), dtype=float)
    neighbor_count = min(len(force_m_s2), max(30, len(force_m_s2) // 40))
    for row, voltage in enumerate(voltage_knots):
        for column, throttle in enumerate(throttle_knots):
            distance = np.sqrt(
                ((voltage_v - voltage) / voltage_scale) ** 2
                + ((throttle_us - throttle) / throttle_scale) ** 2
            )
            nearest = np.argpartition(distance, neighbor_count - 1)[:neighbor_count]
            weights = 1.0 / np.maximum(distance[nearest], 1.0e-3)
            grid[row, column] = float(
                np.sum(weights * force_m_s2[nearest]) / np.sum(weights)
            )
    grid = np.maximum.accumulate(grid, axis=0)
    grid = np.maximum.accumulate(grid, axis=1)
    for column in range(1, grid.shape[1]):
        grid[:, column] = np.maximum(grid[:, column], grid[:, column - 1] + 1.0e-6)
    return grid


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
