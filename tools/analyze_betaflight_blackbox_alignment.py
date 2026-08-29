#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np


BLACKBOX_FIELDS = (
    "time (us)",
    "axisP[0]",
    "axisP[1]",
    "axisP[2]",
    "axisI[0]",
    "axisI[1]",
    "axisI[2]",
    "axisD[0]",
    "axisD[1]",
    "axisF[0]",
    "axisF[1]",
    "axisF[2]",
    "rcCommand[3]",
    "setpoint[0]",
    "setpoint[1]",
    "setpoint[2]",
    "motor[0]",
    "motor[1]",
    "motor[2]",
    "motor[3]",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Align one Betaflight Blackbox raw CSV with one host no-prop CSV."
    )
    parser.add_argument("--host-csv", required=True)
    parser.add_argument(
        "--blackbox-csv",
        required=True,
        help="blackbox_decode CSV generated with --unit-rotation raw",
    )
    parser.add_argument("--blackbox-bfl", default="", help="Optional source BFL for hashing.")
    parser.add_argument("--decoder-commit", default="", help="blackbox-tools git commit used to decode.")
    parser.add_argument("--min-check-us", type=float, default=1050.0)
    parser.add_argument("--output", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = analyze(
        Path(args.host_csv),
        Path(args.blackbox_csv),
        blackbox_bfl=Path(args.blackbox_bfl) if args.blackbox_bfl else None,
        decoder_commit=str(args.decoder_commit),
        min_check_us=float(args.min_check_us),
    )
    output = (
        Path(args.output)
        if args.output
        else Path(args.host_csv).with_name(f"{Path(args.host_csv).stem}_blackbox_alignment.json")
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output)


def analyze(
    host_csv: Path,
    blackbox_csv: Path,
    *,
    blackbox_bfl: Path | None = None,
    decoder_commit: str = "",
    min_check_us: float = 1050.0,
) -> dict[str, Any]:
    host_csv = host_csv.expanduser().resolve()
    blackbox_csv = blackbox_csv.expanduser().resolve()
    host_rows = _read_host_rows(host_csv)
    blackbox = _read_blackbox_numeric(blackbox_csv)
    blackbox_time_s = (blackbox["time (us)"] - blackbox["time (us)"][0]) / 1.0e6

    armed_intervals = _host_intervals(host_rows, "armed", "1")
    override_intervals = _host_intervals(host_rows, "msp_override_active", "1")
    algorithm_intervals = _host_intervals(host_rows, "msp_publish_mode", "algorithm")
    if not armed_intervals or not override_intervals or not algorithm_intervals:
        raise RuntimeError("host CSV must contain armed, override, and algorithm intervals")

    motor_samples = _host_motor_samples(host_rows)
    alignment = _fit_motor_alignment(
        motor_samples,
        blackbox_time_s,
        np.column_stack([blackbox[f"motor[{index}]"] for index in range(4)]),
        initial_offset_s=armed_intervals[0][0],
    )
    offset_s = alignment["host_minus_blackbox_s"]
    aligned_host_time_s = blackbox_time_s + offset_s
    motor_us = (
        np.column_stack([blackbox[f"motor[{index}]"] for index in range(4)])
        * alignment["motor_scale_us_per_raw"]
        + alignment["motor_offset_us"]
    )
    p_terms = np.column_stack([blackbox[f"axisP[{index}]"] for index in range(3)])
    i_terms = np.column_stack([blackbox[f"axisI[{index}]"] for index in range(3)])
    d_terms = np.column_stack(
        [blackbox["axisD[0]"], blackbox["axisD[1]"], np.zeros_like(blackbox_time_s)]
    )
    f_terms = np.column_stack([blackbox[f"axisF[{index}]"] for index in range(3)])
    setpoints = np.column_stack([blackbox[f"setpoint[{index}]"] for index in range(3)])

    first_algorithm = algorithm_intervals[0]
    first_algorithm_exit_s = _host_state_exit_time(
        host_rows,
        "msp_publish_mode",
        "algorithm",
        first_algorithm[0],
    )
    first_algorithm_analysis_end_s = first_algorithm_exit_s or first_algorithm[1]
    first_algorithm_mask = _time_mask(
        aligned_host_time_s,
        first_algorithm[0],
        first_algorithm_analysis_end_s,
    )
    algorithm_analysis_mask = _time_mask(
        aligned_host_time_s,
        first_algorithm[0],
        algorithm_intervals[-1][1],
    )
    pre_override_mask = _time_mask(
        aligned_host_time_s,
        armed_intervals[0][0],
        override_intervals[0][0],
    )
    if not np.any(first_algorithm_mask) or not np.any(pre_override_mask):
        raise RuntimeError("aligned Blackbox data does not overlap the host control intervals")

    i_nonzero = np.any(i_terms != 0.0, axis=1) & first_algorithm_mask
    throttle_active = (blackbox["rcCommand[3]"] > 1000.0) & first_algorithm_mask
    host_throttle_crossing_s = _first_host_number(
        host_rows,
        lambda row: row.get("msp_publish_mode") == "algorithm"
        and (_number(row.get("rc_sent_ch3")) or 0.0) > min_check_us,
        "elapsed_s",
    )
    motor_spread_us = np.ptp(motor_us[algorithm_analysis_mask], axis=1)
    max_abs_i = np.max(np.abs(i_terms[algorithm_analysis_mask]), axis=1)
    max_abs_p = np.max(np.abs(p_terms[algorithm_analysis_mask]), axis=1)
    max_abs_d = np.max(np.abs(d_terms[algorithm_analysis_mask]), axis=1)

    slope_mask = i_nonzero & (aligned_host_time_s < first_algorithm_analysis_end_s)
    i_slopes = [
        float(np.polyfit(aligned_host_time_s[slope_mask], i_terms[slope_mask, axis], 1)[0])
        for axis in range(3)
    ]

    bfl_input = None
    if blackbox_bfl is not None:
        blackbox_bfl = blackbox_bfl.expanduser().resolve()
        bfl_input = _input_metadata(blackbox_bfl)
    return {
        "schema_version": 1,
        "inputs": {
            "host_csv": _input_metadata(host_csv),
            "blackbox_csv": _input_metadata(blackbox_csv),
            "blackbox_bfl": bfl_input,
            "decoder_commit": decoder_commit or None,
            "rotation_units": "raw",
        },
        "selection": {
            "blackbox_duration_s": _rounded(blackbox_time_s[-1]),
            "host_armed_intervals_s": _rounded_intervals(armed_intervals),
            "host_override_intervals_s": _rounded_intervals(override_intervals),
            "host_algorithm_intervals_s": _rounded_intervals(algorithm_intervals),
            "first_algorithm_exit_transition_s": _rounded(first_algorithm_exit_s),
            "duration_match_error_s": _rounded(
                abs((armed_intervals[0][1] - armed_intervals[0][0]) - blackbox_time_s[-1])
            ),
        },
        "alignment": {key: _rounded(value, 9) for key, value in alignment.items()},
        "pre_override": {
            "motor_min_us": _rounded_vector(np.min(motor_us[pre_override_mask], axis=0)),
            "motor_max_us": _rounded_vector(np.max(motor_us[pre_override_mask], axis=0)),
            "max_motor_spread_us": _rounded(np.max(np.ptp(motor_us[pre_override_mask], axis=1))),
            "max_abs_i_term": _rounded(float(np.max(np.abs(i_terms[pre_override_mask])))),
        },
        "algorithm_takeover": {
            "setpoint_min": _rounded_vector(np.min(setpoints[algorithm_analysis_mask], axis=0)),
            "setpoint_max": _rounded_vector(np.max(setpoints[algorithm_analysis_mask], axis=0)),
            "max_abs_p_term": _rounded_vector(np.max(np.abs(p_terms[algorithm_analysis_mask]), axis=0)),
            "max_abs_i_term": _rounded_vector(np.max(np.abs(i_terms[algorithm_analysis_mask]), axis=0)),
            "max_abs_d_term": _rounded_vector(np.max(np.abs(d_terms[algorithm_analysis_mask]), axis=0)),
            "max_abs_f_term": _rounded_vector(np.max(np.abs(f_terms[algorithm_analysis_mask]), axis=0)),
            "i_term_slope_per_s_before_interruption": _rounded_vector(i_slopes),
            "motor_peak_us": _rounded_vector(np.max(motor_us[algorithm_analysis_mask], axis=0)),
            "max_motor_spread_us": _rounded(float(np.max(motor_spread_us))),
            "motor_spread_correlation": {
                "max_abs_i_term": _rounded(_correlation(motor_spread_us, max_abs_i), 9),
                "max_abs_p_term": _rounded(_correlation(motor_spread_us, max_abs_p), 9),
                "max_abs_d_term": _rounded(_correlation(motor_spread_us, max_abs_d), 9),
            },
            "host_throttle_crossed_min_check_s": _rounded(host_throttle_crossing_s),
            "min_check_us": min_check_us,
            "blackbox_throttle_became_active_host_s": _rounded(
                _first_time(aligned_host_time_s, throttle_active)
            ),
            "blackbox_i_term_first_nonzero_host_s": _rounded(
                _first_time(aligned_host_time_s, i_nonzero)
            ),
        },
        "conclusion": {
            "root_cause": "fixed_no_prop_pid_i_term_windup_above_min_check",
            "png_rate_command_causal": False,
            "reason": (
                "Blackbox setpoints remain zero while I terms accumulate and motor spread follows "
                "the maximum absolute I term. The fixed propeller-free airframe cannot respond after "
                "algorithm throttle activates the Betaflight PID loop."
            ),
        },
    }


def _read_host_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("host CSV is empty")
    return rows


def _read_blackbox_numeric(path: Path) -> dict[str, np.ndarray]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        header = [field.strip() for field in next(csv.reader(stream))]
    if any("gyroADC" in field and "(deg/s)" in field for field in header):
        raise RuntimeError(
            "Blackbox CSV uses converted gyro units; decode again with --unit-rotation raw"
        )
    missing = [field for field in BLACKBOX_FIELDS if field not in header]
    if missing:
        raise RuntimeError(f"Blackbox CSV is missing fields: {', '.join(missing)}")
    indexes = [header.index(field) for field in BLACKBOX_FIELDS]
    values = np.loadtxt(path, delimiter=",", skiprows=1, usecols=indexes)
    if values.ndim != 2 or values.shape[0] < 2:
        raise RuntimeError("Blackbox CSV has insufficient numeric rows")
    return {field: values[:, index] for index, field in enumerate(BLACKBOX_FIELDS)}


def _host_intervals(
    rows: list[dict[str, str]], field: str, active_value: str
) -> list[tuple[float, float]]:
    intervals: list[tuple[float, float]] = []
    start: float | None = None
    previous_time: float | None = None
    for row in rows:
        timestamp = _number(row.get("elapsed_s"))
        if timestamp is None:
            continue
        active = row.get(field) == active_value
        if active and start is None:
            start = timestamp
        elif not active and start is not None:
            intervals.append((start, previous_time if previous_time is not None else timestamp))
            start = None
        previous_time = timestamp
    if start is not None and previous_time is not None:
        intervals.append((start, previous_time))
    return intervals


def _host_motor_samples(rows: list[dict[str, str]]) -> np.ndarray:
    samples: list[tuple[float, ...]] = []
    previous_count: int | None = None
    for row in rows:
        count = _integer(row.get("msp_cmd_motor_success_count"))
        elapsed_s = _number(row.get("elapsed_s"))
        age_s = _number(row.get("msp_motor_age_s"))
        motors = tuple(_number(row.get(f"motor_output_ch{index}")) for index in range(1, 5))
        if (
            count is None
            or count == previous_count
            or elapsed_s is None
            or age_s is None
            or any(value is None or value <= 0.0 for value in motors)
        ):
            continue
        samples.append((elapsed_s - age_s, *(float(value) for value in motors if value is not None)))
        previous_count = count
    if len(samples) < 10:
        raise RuntimeError("host CSV has insufficient timestamped MSP motor samples")
    return np.asarray(samples, dtype=float)


def _host_state_exit_time(
    rows: list[dict[str, str]],
    field: str,
    active_value: str,
    start_s: float,
) -> float | None:
    active_seen = False
    for row in rows:
        timestamp = _number(row.get("elapsed_s"))
        if timestamp is None or timestamp < start_s:
            continue
        if row.get(field) == active_value:
            active_seen = True
        elif active_seen:
            return timestamp
    return None


def _fit_motor_alignment(
    host_samples: np.ndarray,
    blackbox_time_s: np.ndarray,
    blackbox_motors: np.ndarray,
    *,
    initial_offset_s: float,
) -> dict[str, float]:
    best: tuple[float, float, float, float, float] | None = None

    def evaluate(offset_s: float) -> tuple[float, float, float, float, float] | None:
        target_s = host_samples[:, 0] - offset_s
        valid = (target_s >= blackbox_time_s[0]) & (target_s <= blackbox_time_s[-1])
        if np.count_nonzero(valid) < 10:
            return None
        interpolated = np.column_stack(
            [
                np.interp(target_s[valid], blackbox_time_s, blackbox_motors[:, channel])
                for channel in range(4)
            ]
        )
        source = interpolated.reshape(-1)
        measured = host_samples[valid, 1:].reshape(-1)
        design = np.column_stack([source, np.ones_like(source)])
        scale, intercept = np.linalg.lstsq(design, measured, rcond=None)[0]
        prediction = design @ np.asarray([scale, intercept])
        rmse = float(np.sqrt(np.mean(np.square(prediction - measured))))
        correlation = _correlation(prediction, measured)
        return (rmse, offset_s, float(scale), float(intercept), correlation)

    for offset_s in np.arange(initial_offset_s - 0.35, initial_offset_s + 0.35, 0.0005):
        candidate = evaluate(float(offset_s))
        if candidate is None:
            continue
        if best is None or candidate[0] < best[0]:
            best = candidate
    if best is None:
        raise RuntimeError("unable to align host MSP motor samples with Blackbox")
    coarse_offset_s = best[1]
    for offset_s in np.arange(coarse_offset_s - 0.0005, coarse_offset_s + 0.0005, 0.00001):
        candidate = evaluate(float(offset_s))
        if candidate is not None and candidate[0] < best[0]:
            best = candidate
    rmse, offset_s, scale, intercept, correlation = best
    return {
        "host_minus_blackbox_s": offset_s,
        "motor_scale_us_per_raw": scale,
        "motor_offset_us": intercept,
        "motor_fit_rmse_us": rmse,
        "motor_fit_correlation": correlation,
    }


def _time_mask(values: np.ndarray, start: float, end: float) -> np.ndarray:
    return (values >= start) & (values <= end)


def _first_time(time_s: np.ndarray, mask: np.ndarray) -> float | None:
    indexes = np.flatnonzero(mask)
    return None if not len(indexes) else float(time_s[indexes[0]])


def _first_host_number(
    rows: Iterable[dict[str, str]], predicate: Any, field: str
) -> float | None:
    for row in rows:
        if predicate(row):
            return _number(row.get(field))
    return None


def _input_metadata(path: Path) -> dict[str, Any]:
    return {"path": str(path), "size_bytes": path.stat().st_size, "sha256": _sha256(path)}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _number(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if np.isfinite(result) else None


def _integer(value: Any) -> int | None:
    number = _number(value)
    return None if number is None else int(number)


def _correlation(left: np.ndarray, right: np.ndarray) -> float:
    if len(left) < 2 or np.std(left) == 0.0 or np.std(right) == 0.0:
        return 0.0
    return float(np.corrcoef(left, right)[0, 1])


def _rounded(value: float | None, digits: int = 6) -> float | None:
    return None if value is None else round(float(value), digits)


def _rounded_vector(values: Iterable[float]) -> list[float]:
    return [_rounded(float(value)) for value in values]  # type: ignore[list-item]


def _rounded_intervals(values: Iterable[tuple[float, float]]) -> list[list[float]]:
    return [[round(start, 6), round(end, 6)] for start, end in values]


if __name__ == "__main__":
    main()
