#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np


BLACKBOX_NUMERIC_FIELDS = (
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
    "vbatLatest (V)",
    "amperageLatest (A)",
    "rssi",
    "gyroADC[0]",
    "gyroADC[1]",
    "gyroADC[2]",
    "gyroUnfilt[0]",
    "gyroUnfilt[1]",
    "gyroUnfilt[2]",
    "motor[0]",
    "motor[1]",
    "motor[2]",
    "motor[3]",
    "energyCumulative (mAh)",
    "rxSignalReceived",
    "rxFlightChannelsValid",
    "GPS_numSat",
)

BLACKBOX_CATEGORY_FIELDS = (
    "flightModeFlags (flags)",
    "stateFlags (flags)",
    "failsafePhase (flags)",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Align a manual Betaflight flight with a host CSV and summarize Blackbox phases. "
            "Decode the BFL with blackbox_decode --unit-rotation raw first."
        )
    )
    parser.add_argument("--host-csv", required=True)
    parser.add_argument("--blackbox-csv", required=True)
    parser.add_argument("--blackbox-bfl", default="")
    parser.add_argument("--decoder-commit", default="")
    parser.add_argument("--host-throttle-field", default="rc_in_ch4")
    parser.add_argument("--min-check-us", type=float, default=1050.0)
    parser.add_argument("--max-pwm-us", type=float, default=2000.0)
    parser.add_argument("--idle-command", type=float, default=1000.0)
    parser.add_argument("--stable-throttle-command", type=float, default=1200.0)
    parser.add_argument("--endpoint-window-s", type=float, default=1.0)
    parser.add_argument("--alignment-search-s", type=float, default=1.0)
    parser.add_argument("--alignment-step-s", type=float, default=0.001)
    parser.add_argument("--motor-high-raw", type=float, default=1800.0)
    parser.add_argument("--motor-saturation-raw", type=float, default=2040.0)
    parser.add_argument("--current-high-a", type=float, default=20.0)
    parser.add_argument("--vbat-low-v", type=float, default=23.0)
    parser.add_argument("--motor-scale-us-per-raw", type=float)
    parser.add_argument("--motor-offset-us", type=float)
    parser.add_argument("--output", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = analyze(
        Path(args.host_csv),
        Path(args.blackbox_csv),
        blackbox_bfl=Path(args.blackbox_bfl) if args.blackbox_bfl else None,
        decoder_commit=str(args.decoder_commit),
        host_throttle_field=str(args.host_throttle_field),
        min_check_us=float(args.min_check_us),
        max_pwm_us=float(args.max_pwm_us),
        idle_command=float(args.idle_command),
        stable_throttle_command=float(args.stable_throttle_command),
        endpoint_window_s=float(args.endpoint_window_s),
        alignment_search_s=float(args.alignment_search_s),
        alignment_step_s=float(args.alignment_step_s),
        motor_high_raw=float(args.motor_high_raw),
        motor_saturation_raw=float(args.motor_saturation_raw),
        current_high_a=float(args.current_high_a),
        vbat_low_v=float(args.vbat_low_v),
        motor_scale_us_per_raw=args.motor_scale_us_per_raw,
        motor_offset_us=args.motor_offset_us,
    )
    output = (
        Path(args.output)
        if args.output
        else Path(args.host_csv).with_name(f"{Path(args.host_csv).stem}_blackbox_flight.json")
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
    host_throttle_field: str = "rc_in_ch4",
    min_check_us: float = 1050.0,
    max_pwm_us: float = 2000.0,
    idle_command: float = 1000.0,
    stable_throttle_command: float = 1200.0,
    endpoint_window_s: float = 1.0,
    alignment_search_s: float = 1.0,
    alignment_step_s: float = 0.001,
    motor_high_raw: float = 1800.0,
    motor_saturation_raw: float = 2040.0,
    current_high_a: float = 20.0,
    vbat_low_v: float = 23.0,
    motor_scale_us_per_raw: float | None = None,
    motor_offset_us: float | None = None,
) -> dict[str, Any]:
    _validate_parameters(
        min_check_us=min_check_us,
        max_pwm_us=max_pwm_us,
        endpoint_window_s=endpoint_window_s,
        alignment_search_s=alignment_search_s,
        alignment_step_s=alignment_step_s,
        motor_scale_us_per_raw=motor_scale_us_per_raw,
        motor_offset_us=motor_offset_us,
    )
    host_csv = host_csv.expanduser().resolve()
    blackbox_csv = blackbox_csv.expanduser().resolve()
    host_rows = _read_host_rows(host_csv, host_throttle_field)
    blackbox = _read_blackbox_numeric(blackbox_csv)
    categories = _read_blackbox_categories(blackbox_csv)

    blackbox_time_s = (blackbox["time (us)"] - blackbox["time (us)"][0]) / 1.0e6
    if np.any(np.diff(blackbox_time_s) <= 0.0):
        raise RuntimeError("Blackbox time must be strictly increasing")
    sample_dt_s = np.diff(blackbox_time_s, prepend=blackbox_time_s[0])
    sample_dt_s[0] = float(np.median(np.diff(blackbox_time_s)))
    blackbox_duration_s = float(blackbox_time_s[-1])

    armed_intervals = _host_intervals(host_rows, "armed", 1)
    if not armed_intervals:
        raise RuntimeError("host CSV does not contain an armed interval")
    selected_index = min(
        range(len(armed_intervals)),
        key=lambda index: abs(_interval_duration(armed_intervals[index]) - blackbox_duration_s),
    )
    selected_armed = armed_intervals[selected_index]
    alignment = _fit_throttle_alignment(
        host_rows,
        host_throttle_field=host_throttle_field,
        blackbox_time_s=blackbox_time_s,
        blackbox_throttle=blackbox["rcCommand[3]"],
        armed_interval=selected_armed,
        min_check_us=min_check_us,
        max_pwm_us=max_pwm_us,
        idle_command=idle_command,
        search_s=alignment_search_s,
        step_s=alignment_step_s,
    )

    throttle = blackbox["rcCommand[3]"]
    motors = np.column_stack([blackbox[f"motor[{index}]"] for index in range(4)])
    active_runs = _mask_runs(throttle > idle_command, blackbox_time_s, sample_dt_s)
    primary_active = max(active_runs, key=lambda item: item["duration_s"]) if active_runs else None
    phase_masks = _phase_masks(
        blackbox_time_s,
        throttle,
        primary_active=primary_active,
        stable_throttle_command=stable_throttle_command,
        endpoint_window_s=endpoint_window_s,
    )
    segments = {
        name: _segment_metrics(mask, sample_dt_s, blackbox, motors)
        for name, mask in phase_masks.items()
        if np.any(mask)
    }
    endpoint_mask = phase_masks.get("endpoint", np.zeros(len(blackbox_time_s), dtype=bool))
    steady_mask = phase_masks.get("steady_hover", np.zeros(len(blackbox_time_s), dtype=bool))

    result: dict[str, Any] = {
        "schema_version": 1,
        "inputs": {
            "host_csv": _input_metadata(host_csv),
            "blackbox_csv": _input_metadata(blackbox_csv),
            "blackbox_bfl": (
                None
                if blackbox_bfl is None
                else _input_metadata(blackbox_bfl.expanduser().resolve())
            ),
            "decoder_commit": decoder_commit or None,
            "rotation_units": "raw",
        },
        "selection": {
            "blackbox_duration_s": _rounded(blackbox_duration_s),
            "host_armed_intervals_s": _rounded_intervals(armed_intervals),
            "selected_host_armed_interval_index": selected_index,
            "selected_host_armed_interval_s": _rounded_interval(selected_armed),
            "duration_match_error_s": _rounded(
                abs(_interval_duration(selected_armed) - blackbox_duration_s)
            ),
            "throttle_alignment": {key: _rounded(value, 9) for key, value in alignment.items()},
        },
        "blackbox": {
            "rows": len(blackbox_time_s),
            "median_sample_rate_hz": _rounded(1.0 / float(np.median(np.diff(blackbox_time_s))), 3),
            "active_intervals_s": [
                [_rounded(run["start_s"]), _rounded(run["end_s"]), _rounded(run["duration_s"])]
                for run in active_runs
            ],
            "failsafe_values": categories.get("failsafePhase (flags)", []),
            "flight_mode_values": categories.get("flightModeFlags (flags)", []),
            "state_flag_values": categories.get("stateFlags (flags)", []),
            "gps_satellites_max": int(np.max(blackbox["GPS_numSat"])),
            "rx_invalid_duration_s": _rounded(
                np.sum(
                    sample_dt_s[
                        (blackbox["rxSignalReceived"] != 1.0)
                        | (blackbox["rxFlightChannelsValid"] != 1.0)
                    ]
                )
            ),
        },
        "segments": segments,
        "endpoint_transients": _endpoint_metrics(
            endpoint_mask,
            blackbox_time_s,
            sample_dt_s,
            blackbox,
            motors,
            motor_high_raw=motor_high_raw,
            motor_saturation_raw=motor_saturation_raw,
            current_high_a=current_high_a,
            vbat_low_v=vbat_low_v,
        ),
        "host": _host_metrics(
            host_rows,
            blackbox,
            blackbox_time_s,
            steady_mask,
            alignment_offset_s=float(alignment["host_minus_blackbox_s"]),
        ),
        "control_evidence": {
            "set_raw_rc_write_success_max": _host_maximum(
                host_rows, "msp_set_raw_rc_write_success_count"
            ),
            "override_active_rows": sum(
                1 for row in host_rows if _integer(row.get("msp_override_active")) == 1
            ),
        },
    }
    if motor_scale_us_per_raw is not None and motor_offset_us is not None:
        result["motor_conversion"] = {
            "formula": "motor_us = scale_us_per_raw * motor_raw + offset_us",
            "scale_us_per_raw": motor_scale_us_per_raw,
            "offset_us": motor_offset_us,
            "steady_motor_p50_us": (
                None
                if not np.any(steady_mask)
                else _rounded_vector(
                    np.percentile(motors[steady_mask], 50.0, axis=0)
                    * motor_scale_us_per_raw
                    + motor_offset_us
                )
            ),
            "steady_motor_spread_p95_us": (
                None
                if not np.any(steady_mask)
                else _rounded(
                    np.percentile(np.ptp(motors[steady_mask], axis=1), 95.0)
                    * motor_scale_us_per_raw
                )
            ),
        }
    return result


def _validate_parameters(**values: Any) -> None:
    if values["max_pwm_us"] <= values["min_check_us"]:
        raise ValueError("max_pwm_us must exceed min_check_us")
    if values["endpoint_window_s"] < 0.0:
        raise ValueError("endpoint_window_s must be non-negative")
    if values["alignment_search_s"] < 0.0 or values["alignment_step_s"] <= 0.0:
        raise ValueError("alignment search values are invalid")
    scale = values["motor_scale_us_per_raw"]
    offset = values["motor_offset_us"]
    if (scale is None) != (offset is None):
        raise ValueError("motor scale and offset must be provided together")


def _read_host_rows(path: Path, throttle_field: str) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fields = set(reader.fieldnames or [])
    required = {"elapsed_s", "armed", throttle_field}
    missing = sorted(required - fields)
    if missing:
        raise RuntimeError(f"host CSV is missing fields: {', '.join(missing)}")
    if not rows:
        raise RuntimeError("host CSV is empty")
    return rows


def _read_blackbox_numeric(path: Path) -> dict[str, np.ndarray]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        header = [field.strip() for field in next(csv.reader(stream))]
    if any("gyroADC" in field and "(deg/s)" in field for field in header):
        raise RuntimeError("Blackbox CSV uses converted gyro units; decode with --unit-rotation raw")
    missing = [field for field in BLACKBOX_NUMERIC_FIELDS if field not in header]
    if missing:
        raise RuntimeError(f"Blackbox CSV is missing fields: {', '.join(missing)}")
    indexes = [header.index(field) for field in BLACKBOX_NUMERIC_FIELDS]
    values = np.loadtxt(path, delimiter=",", skiprows=1, usecols=indexes)
    if values.ndim != 2 or values.shape[0] < 2:
        raise RuntimeError("Blackbox CSV has insufficient numeric rows")
    return {field: values[:, index] for index, field in enumerate(BLACKBOX_NUMERIC_FIELDS)}


def _read_blackbox_categories(path: Path) -> dict[str, list[str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        header = [field.strip() for field in next(reader)]
        indexes = {
            field: header.index(field) for field in BLACKBOX_CATEGORY_FIELDS if field in header
        }
        result = {field: [] for field in indexes}
        seen = {field: set() for field in indexes}
        for row in reader:
            for field, index in indexes.items():
                value = row[index].strip()
                if value not in seen[field]:
                    seen[field].add(value)
                    result[field].append(value)
    return result


def _host_intervals(
    rows: Iterable[dict[str, str]], field: str, active_value: int
) -> list[tuple[float, float]]:
    intervals: list[tuple[float, float]] = []
    start: float | None = None
    previous: float | None = None
    for row in rows:
        timestamp = _number(row.get("elapsed_s"))
        if timestamp is None:
            continue
        active = _integer(row.get(field)) == active_value
        if active and start is None:
            start = timestamp
        elif not active and start is not None:
            intervals.append((start, previous if previous is not None else timestamp))
            start = None
        previous = timestamp
    if start is not None and previous is not None:
        intervals.append((start, previous))
    return intervals


def _fit_throttle_alignment(
    host_rows: list[dict[str, str]],
    *,
    host_throttle_field: str,
    blackbox_time_s: np.ndarray,
    blackbox_throttle: np.ndarray,
    armed_interval: tuple[float, float],
    min_check_us: float,
    max_pwm_us: float,
    idle_command: float,
    search_s: float,
    step_s: float,
) -> dict[str, float | str]:
    samples = [
        (_number(row.get("elapsed_s")), _number(row.get(host_throttle_field)))
        for row in host_rows
    ]
    host = np.asarray(
        [(timestamp, pwm) for timestamp, pwm in samples if timestamp is not None and pwm is not None],
        dtype=float,
    )
    if len(host) < 2:
        raise RuntimeError("host CSV has insufficient throttle samples")
    host_command = np.where(
        host[:, 1] <= min_check_us,
        idle_command,
        idle_command
        + (host[:, 1] - min_check_us) * (2000.0 - idle_command) / (max_pwm_us - min_check_us),
    )
    if np.std(blackbox_throttle) < 1.0 or np.std(host_command) < 1.0:
        return {
            "host_minus_blackbox_s": armed_interval[0],
            "rmse_command_units": 0.0,
            "correlation": 0.0,
            "sample_count": 0.0,
            "method": "arm_start_fallback",
        }

    offsets = np.arange(
        armed_interval[0] - search_s,
        armed_interval[0] + search_s + step_s * 0.5,
        step_s,
    )
    best: tuple[float, float, float, int] | None = None
    duration_s = float(blackbox_time_s[-1])
    for offset in offsets:
        selected = (
            (host[:, 0] >= max(armed_interval[0], offset))
            & (host[:, 0] <= min(armed_interval[1], offset + duration_s))
        )
        if np.count_nonzero(selected) < 10:
            continue
        observed = host_command[selected]
        predicted = np.interp(host[selected, 0] - offset, blackbox_time_s, blackbox_throttle)
        rmse = float(np.sqrt(np.mean((predicted - observed) ** 2)))
        correlation = _correlation(predicted, observed)
        candidate = (rmse, float(offset), correlation, int(np.count_nonzero(selected)))
        if best is None or candidate[0] < best[0]:
            best = candidate
    if best is None:
        raise RuntimeError("unable to align host and Blackbox throttle profiles")
    return {
        "host_minus_blackbox_s": best[1],
        "rmse_command_units": best[0],
        "correlation": best[2],
        "sample_count": float(best[3]),
        "method": "throttle_grid_search",
    }


def _mask_runs(mask: np.ndarray, time_s: np.ndarray, dt_s: np.ndarray) -> list[dict[str, float]]:
    transitions = np.diff(np.r_[False, mask, False].astype(int))
    starts = np.flatnonzero(transitions == 1)
    ends = np.flatnonzero(transitions == -1) - 1
    return [
        {
            "start_s": float(time_s[start]),
            "end_s": float(time_s[end]),
            "duration_s": float(np.sum(dt_s[start : end + 1])),
        }
        for start, end in zip(starts, ends)
    ]


def _phase_masks(
    time_s: np.ndarray,
    throttle: np.ndarray,
    *,
    primary_active: dict[str, float] | None,
    stable_throttle_command: float,
    endpoint_window_s: float,
) -> dict[str, np.ndarray]:
    masks: dict[str, np.ndarray] = {}
    if primary_active is None:
        masks["idle"] = np.ones(len(time_s), dtype=bool)
        return masks
    active_start = primary_active["start_s"]
    active_end = primary_active["end_s"]
    masks["idle"] = time_s < active_start
    stable_candidates = np.flatnonzero(
        (time_s >= active_start)
        & (time_s <= active_end)
        & (throttle >= stable_throttle_command)
    )
    if not len(stable_candidates):
        masks["active"] = (time_s >= active_start) & (time_s <= active_end)
    else:
        stable_start = float(time_s[stable_candidates[0]])
        stable_end = max(stable_start, active_end - endpoint_window_s)
        masks["ramp"] = (time_s >= active_start) & (time_s < stable_start)
        masks["steady_hover"] = (time_s >= stable_start) & (time_s < stable_end)
        masks["endpoint"] = (time_s >= stable_end) & (time_s <= active_end)
    masks["post_throttle"] = time_s > active_end
    return masks


def _segment_metrics(
    mask: np.ndarray,
    dt_s: np.ndarray,
    blackbox: dict[str, np.ndarray],
    motors: np.ndarray,
) -> dict[str, Any]:
    setpoints = np.column_stack([blackbox[f"setpoint[{index}]"] for index in range(3)])
    gyro = np.column_stack([blackbox[f"gyroADC[{index}]"] for index in range(3)])
    p_terms = np.column_stack([blackbox[f"axisP[{index}]"] for index in range(3)])
    i_terms = np.column_stack([blackbox[f"axisI[{index}]"] for index in range(3)])
    d_terms = np.column_stack(
        [blackbox["axisD[0]"], blackbox["axisD[1]"], np.zeros(len(mask))]
    )
    return {
        "duration_s": _rounded(np.sum(dt_s[mask])),
        "throttle_command": _percentiles(blackbox["rcCommand[3]"][mask]),
        "vbat_v": _percentiles(blackbox["vbatLatest (V)"][mask]),
        "current_a": _percentiles(blackbox["amperageLatest (A)"][mask]),
        "motor_raw": {
            "p50": _rounded_vector(np.percentile(motors[mask], 50.0, axis=0)),
            "p95": _rounded_vector(np.percentile(motors[mask], 95.0, axis=0)),
            "max": _rounded_vector(np.max(motors[mask], axis=0)),
            "spread": _percentiles(np.ptp(motors[mask], axis=1)),
        },
        "setpoint_abs": _absolute_metrics(setpoints[mask]),
        "gyro_raw_abs": _absolute_metrics(gyro[mask]),
        "pid_abs": {
            "p": _absolute_metrics(p_terms[mask]),
            "i": _absolute_metrics(i_terms[mask]),
            "d": _absolute_metrics(d_terms[mask]),
        },
    }


def _endpoint_metrics(
    mask: np.ndarray,
    time_s: np.ndarray,
    dt_s: np.ndarray,
    blackbox: dict[str, np.ndarray],
    motors: np.ndarray,
    *,
    motor_high_raw: float,
    motor_saturation_raw: float,
    current_high_a: float,
    vbat_low_v: float,
) -> dict[str, Any] | None:
    if not np.any(mask):
        return None
    motor_high = mask & np.any(motors >= motor_high_raw, axis=1)
    motor_saturated = mask & np.any(motors >= motor_saturation_raw, axis=1)
    current_high = mask & (blackbox["amperageLatest (A)"] > current_high_a)
    voltage_low = mask & (blackbox["vbatLatest (V)"] < vbat_low_v)
    return {
        "interval_s": [_rounded(time_s[mask][0]), _rounded(time_s[mask][-1])],
        "thresholds": {
            "motor_high_raw": motor_high_raw,
            "motor_saturation_raw": motor_saturation_raw,
            "current_high_a": current_high_a,
            "vbat_low_v": vbat_low_v,
        },
        "motor_high_duration_s": _rounded(np.sum(dt_s[motor_high])),
        "motor_saturation_duration_s": _rounded(np.sum(dt_s[motor_saturated])),
        "current_high_duration_s": _rounded(np.sum(dt_s[current_high])),
        "vbat_low_duration_s": _rounded(np.sum(dt_s[voltage_low])),
        "maximum_motor_raw": _rounded(np.max(motors[mask])),
        "maximum_motor_spread_raw": _rounded(np.max(np.ptp(motors[mask], axis=1))),
        "maximum_current_a": _rounded(np.max(blackbox["amperageLatest (A)"][mask])),
        "minimum_vbat_v": _rounded(np.min(blackbox["vbatLatest (V)"][mask])),
        "first_motor_high_s": _first_time(time_s, motor_high),
        "first_current_high_s": _first_time(time_s, current_high),
        "first_vbat_low_s": _first_time(time_s, voltage_low),
    }


def _host_metrics(
    rows: list[dict[str, str]],
    blackbox: dict[str, np.ndarray],
    blackbox_time_s: np.ndarray,
    steady_mask: np.ndarray,
    *,
    alignment_offset_s: float,
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    if np.any(steady_mask):
        host_start = alignment_offset_s + float(blackbox_time_s[steady_mask][0])
        host_end = alignment_offset_s + float(blackbox_time_s[steady_mask][-1])
        selected = [
            row
            for row in rows
            if (_number(row.get("elapsed_s")) or -1.0) >= host_start
            and (_number(row.get("elapsed_s")) or -1.0) <= host_end
        ]
        attitude = {}
        for axis in ("roll", "pitch", "yaw"):
            values = [_number(row.get(f"{axis}_deg")) for row in selected]
            finite = np.asarray([value for value in values if value is not None], dtype=float)
            if len(finite):
                attitude[axis] = {
                    "min_deg": _rounded(np.min(finite)),
                    "max_deg": _rounded(np.max(finite)),
                    "p95_abs_deg": _rounded(np.percentile(np.abs(finite), 95.0)),
                }
        if attitude:
            result["steady_attitude"] = attitude
    analog = _host_analog_alignment(
        rows,
        blackbox,
        blackbox_time_s,
        alignment_offset_s=alignment_offset_s,
    )
    if analog is not None:
        result["analog_alignment"] = analog
    return result


def _host_analog_alignment(
    rows: list[dict[str, str]],
    blackbox: dict[str, np.ndarray],
    blackbox_time_s: np.ndarray,
    *,
    alignment_offset_s: float,
) -> dict[str, Any] | None:
    samples: list[tuple[float, float, float]] = []
    previous_timestamp: float | None = None
    for row in rows:
        elapsed = _number(row.get("elapsed_s"))
        age = _number(row.get("msp_analog_age_s"))
        voltage = _number(row.get("vbat_v"))
        current = _number(row.get("amperage_a"))
        if None in (elapsed, age, voltage, current):
            continue
        timestamp = float(elapsed) - float(age)
        if previous_timestamp is not None and abs(timestamp - previous_timestamp) <= 1.0e-4:
            continue
        samples.append((timestamp, float(voltage), float(current)))
        previous_timestamp = timestamp
    if not samples:
        return None
    values = np.asarray(samples, dtype=float)
    selected = (
        (values[:, 0] >= alignment_offset_s)
        & (values[:, 0] <= alignment_offset_s + float(blackbox_time_s[-1]))
    )
    values = values[selected]
    if len(values) < 2:
        return None
    blackbox_voltage = np.interp(
        values[:, 0] - alignment_offset_s,
        blackbox_time_s,
        blackbox["vbatLatest (V)"],
    )
    blackbox_current = np.interp(
        values[:, 0] - alignment_offset_s,
        blackbox_time_s,
        blackbox["amperageLatest (A)"],
    )
    return {
        "sample_count": len(values),
        "vbat_correlation": _rounded(_correlation(values[:, 1], blackbox_voltage), 9),
        "current_correlation": _rounded(_correlation(values[:, 2], blackbox_current), 9),
        "host_vbat_min_v": _rounded(np.min(values[:, 1])),
        "host_current_max_a": _rounded(np.max(values[:, 2])),
    }


def _host_maximum(rows: Iterable[dict[str, str]], field: str) -> float | None:
    values = [_number(row.get(field)) for row in rows]
    finite = [value for value in values if value is not None]
    return None if not finite else _rounded(max(finite))


def _absolute_metrics(values: np.ndarray) -> dict[str, list[float]]:
    return {
        "p95": _rounded_vector(np.percentile(np.abs(values), 95.0, axis=0)),
        "max": _rounded_vector(np.max(np.abs(values), axis=0)),
    }


def _percentiles(values: np.ndarray) -> dict[str, float]:
    return {
        f"p{label}": _rounded(value)
        for label, value in zip((0, 50, 95, 99, 100), np.percentile(values, (0, 50, 95, 99, 100)))
    }


def _first_time(time_s: np.ndarray, mask: np.ndarray) -> float | None:
    indexes = np.flatnonzero(mask)
    return None if not len(indexes) else _rounded(time_s[indexes[0]])


def _interval_duration(interval: tuple[float, float]) -> float:
    return max(0.0, interval[1] - interval[0])


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


def _rounded(value: Any, digits: int = 6) -> Any:
    if isinstance(value, str):
        return value
    return None if value is None else round(float(value), digits)


def _rounded_vector(values: Iterable[float]) -> list[float]:
    return [round(float(value), 6) for value in values]


def _rounded_interval(value: tuple[float, float]) -> list[float]:
    return [round(value[0], 6), round(value[1], 6)]


def _rounded_intervals(values: Iterable[tuple[float, float]]) -> list[list[float]]:
    return [_rounded_interval(value) for value in values]


if __name__ == "__main__":
    main()
