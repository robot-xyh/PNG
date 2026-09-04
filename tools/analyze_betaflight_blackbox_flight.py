#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision_guidance.runtime_evidence import validate_blackbox_mode_binding  # noqa: E402


DEFAULT_MODE_BINDING = (
    ROOT
    / "config"
    / "betaflight.blackbox_mode_binding.btfl-25.12.2-micoair743v2.json"
)


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
    "accSmooth[0]",
    "accSmooth[1]",
    "accSmooth[2]",
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
    parser.add_argument(
        "--blackbox-mode-binding",
        default=str(DEFAULT_MODE_BINDING),
        help="Firmware-specific contract for Blackbox mode and gyro interpretation.",
    )
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
    parser.add_argument("--thrust-pulse-threshold-us", type=float)
    parser.add_argument("--thrust-plateau-threshold-us", type=float)
    parser.add_argument("--acc-1g-raw", type=float, default=2048.0)
    parser.add_argument("--thrust-hover-window-s", type=float, default=8.0)
    parser.add_argument("--thrust-hover-gap-s", type=float, default=2.0)
    parser.add_argument("--thrust-post-delay-s", type=float, default=1.0)
    parser.add_argument("--output", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = analyze(
        Path(args.host_csv),
        Path(args.blackbox_csv),
        blackbox_bfl=Path(args.blackbox_bfl) if args.blackbox_bfl else None,
        decoder_commit=str(args.decoder_commit),
        blackbox_mode_binding=Path(args.blackbox_mode_binding),
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
        thrust_pulse_threshold_us=args.thrust_pulse_threshold_us,
        thrust_plateau_threshold_us=args.thrust_plateau_threshold_us,
        acc_1g_raw=float(args.acc_1g_raw),
        thrust_hover_window_s=float(args.thrust_hover_window_s),
        thrust_hover_gap_s=float(args.thrust_hover_gap_s),
        thrust_post_delay_s=float(args.thrust_post_delay_s),
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
    blackbox_mode_binding: Path | None = DEFAULT_MODE_BINDING,
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
    thrust_pulse_threshold_us: float | None = None,
    thrust_plateau_threshold_us: float | None = None,
    acc_1g_raw: float = 2048.0,
    thrust_hover_window_s: float = 8.0,
    thrust_hover_gap_s: float = 2.0,
    thrust_post_delay_s: float = 1.0,
) -> dict[str, Any]:
    _validate_parameters(
        min_check_us=min_check_us,
        max_pwm_us=max_pwm_us,
        endpoint_window_s=endpoint_window_s,
        alignment_search_s=alignment_search_s,
        alignment_step_s=alignment_step_s,
        motor_scale_us_per_raw=motor_scale_us_per_raw,
        motor_offset_us=motor_offset_us,
        thrust_pulse_threshold_us=thrust_pulse_threshold_us,
        thrust_plateau_threshold_us=thrust_plateau_threshold_us,
        acc_1g_raw=acc_1g_raw,
        thrust_hover_window_s=thrust_hover_window_s,
        thrust_hover_gap_s=thrust_hover_gap_s,
        thrust_post_delay_s=thrust_post_delay_s,
    )
    host_csv = host_csv.expanduser().resolve()
    blackbox_csv = blackbox_csv.expanduser().resolve()
    host_rows = _read_host_rows(host_csv, host_throttle_field)
    blackbox = _read_blackbox_numeric(blackbox_csv)
    categories = _read_blackbox_categories(blackbox_csv)
    mode_interpretation = _blackbox_mode_interpretation(
        host_csv,
        host_rows,
        categories,
        blackbox_mode_binding,
    )

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
            "mode_interpretation": mode_interpretation,
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
    if thrust_pulse_threshold_us is not None:
        result["thrust_envelope"] = _thrust_envelope_metrics(
            host_rows,
            host_throttle_field=host_throttle_field,
            selected_armed=selected_armed,
            blackbox=blackbox,
            blackbox_time_s=blackbox_time_s,
            sample_dt_s=sample_dt_s,
            motors=motors,
            alignment_offset_s=float(alignment["host_minus_blackbox_s"]),
            pulse_threshold_us=thrust_pulse_threshold_us,
            plateau_threshold_us=thrust_plateau_threshold_us,
            acc_1g_raw=acc_1g_raw,
            hover_window_s=thrust_hover_window_s,
            hover_gap_s=thrust_hover_gap_s,
            post_delay_s=thrust_post_delay_s,
            motor_high_raw=motor_high_raw,
            motor_saturation_raw=motor_saturation_raw,
            motor_scale_us_per_raw=motor_scale_us_per_raw,
            motor_offset_us=motor_offset_us,
        )
    return result


def _blackbox_mode_interpretation(
    host_csv: Path,
    host_rows: list[dict[str, str]],
    categories: dict[str, list[str]],
    binding_path: Path | None,
) -> dict[str, Any]:
    if binding_path is None:
        raise RuntimeError("a firmware-specific Blackbox mode binding is required")
    meta_path = host_csv.with_name(f"{host_csv.stem}_meta.json")
    fc_identity = None
    if meta_path.is_file():
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        candidate = meta.get("fc_identity") if isinstance(meta, dict) else None
        if isinstance(candidate, dict) and candidate:
            fc_identity = candidate
    binding = validate_blackbox_mode_binding(
        binding_path,
        fc_identity=fc_identity,
    )
    required_fields = binding["required_host_fields"]
    available_fields = set(host_rows[0]) if host_rows else set()
    missing_fields = sorted(set(required_fields) - available_fields)
    return {
        **binding,
        "observed_decoder_labels": categories.get("flightModeFlags (flags)", []),
        "host_meta_path": str(meta_path) if meta_path.is_file() else "",
        "host_identity_verified": fc_identity is not None,
        "required_host_fields_missing": missing_fields,
        "authoritative_mode_decision_available": not missing_fields,
    }


def _thrust_envelope_metrics(
    host_rows: list[dict[str, str]],
    *,
    host_throttle_field: str,
    selected_armed: tuple[float, float],
    blackbox: dict[str, np.ndarray],
    blackbox_time_s: np.ndarray,
    sample_dt_s: np.ndarray,
    motors: np.ndarray,
    alignment_offset_s: float,
    pulse_threshold_us: float,
    plateau_threshold_us: float | None,
    acc_1g_raw: float,
    hover_window_s: float,
    hover_gap_s: float,
    post_delay_s: float,
    motor_high_raw: float,
    motor_saturation_raw: float,
    motor_scale_us_per_raw: float | None,
    motor_offset_us: float | None,
) -> dict[str, Any]:
    pulse = _select_host_throttle_run(
        host_rows,
        throttle_field=host_throttle_field,
        armed_interval=selected_armed,
        threshold_us=pulse_threshold_us,
    )
    if pulse is None:
        raise RuntimeError(
            f"host CSV has no armed throttle run at or above {pulse_threshold_us:g} us"
        )
    plateau = None
    if plateau_threshold_us is not None:
        plateau = _select_host_throttle_run(
            host_rows,
            throttle_field=host_throttle_field,
            armed_interval=(pulse[0], pulse[1]),
            threshold_us=plateau_threshold_us,
        )

    host_windows: dict[str, tuple[float, float]] = {
        "hover_pre": (
            pulse[0] - hover_gap_s - hover_window_s,
            pulse[0] - hover_gap_s,
        ),
        "pulse": (pulse[0], pulse[1]),
        "hover_post": (
            pulse[1] + post_delay_s,
            pulse[1] + post_delay_s + hover_window_s,
        ),
    }
    if plateau is not None:
        host_windows["peak_plateau"] = (plateau[0], plateau[1])

    acceleration = np.column_stack(
        [blackbox[f"accSmooth[{index}]"] for index in range(3)]
    )
    load_factor = np.linalg.norm(acceleration, axis=1) / acc_1g_raw
    windows: dict[str, Any] = {}
    for name, host_interval in host_windows.items():
        blackbox_interval = (
            host_interval[0] - alignment_offset_s,
            host_interval[1] - alignment_offset_s,
        )
        mask = (blackbox_time_s >= blackbox_interval[0]) & (
            blackbox_time_s <= blackbox_interval[1]
        )
        if not np.any(mask):
            continue
        window_metrics = _thrust_window_metrics(
            mask,
            blackbox_time_s=blackbox_time_s,
            sample_dt_s=sample_dt_s,
            blackbox=blackbox,
            motors=motors,
            acceleration=acceleration,
            load_factor=load_factor,
            acc_1g_raw=acc_1g_raw,
            host_interval=host_interval,
            blackbox_interval=blackbox_interval,
            pulse_window=name in {"pulse", "peak_plateau"},
            motor_high_raw=motor_high_raw,
            motor_saturation_raw=motor_saturation_raw,
            motor_scale_us_per_raw=motor_scale_us_per_raw,
            motor_offset_us=motor_offset_us,
        )
        window_metrics["host"] = _host_window_metrics(
            host_rows,
            throttle_field=host_throttle_field,
            interval=host_interval,
        )
        windows[name] = window_metrics

    return {
        "parameters": {
            "host_throttle_field": host_throttle_field,
            "pulse_threshold_us": pulse_threshold_us,
            "plateau_threshold_us": plateau_threshold_us,
            "acc_1g_raw": acc_1g_raw,
            "hover_window_s": hover_window_s,
            "hover_gap_s": hover_gap_s,
            "post_delay_s": post_delay_s,
        },
        "pulse_host_interval_s": _rounded_interval((pulse[0], pulse[1])),
        "pulse_host_duration_s": _rounded(pulse[1] - pulse[0]),
        "pulse_host_max_throttle_us": _rounded(pulse[2]),
        "plateau_host_interval_s": (
            None if plateau is None else _rounded_interval((plateau[0], plateau[1]))
        ),
        "plateau_host_duration_s": (
            None if plateau is None else _rounded(plateau[1] - plateau[0])
        ),
        "windows": windows,
    }


def _select_host_throttle_run(
    rows: list[dict[str, str]],
    *,
    throttle_field: str,
    armed_interval: tuple[float, float],
    threshold_us: float,
) -> tuple[float, float, float] | None:
    samples = [
        (elapsed, throttle)
        for row in rows
        if (elapsed := _number(row.get("elapsed_s"))) is not None
        and (throttle := _number(row.get(throttle_field))) is not None
        and armed_interval[0] <= elapsed <= armed_interval[1]
        and _integer(row.get("armed")) == 1
    ]
    if not samples:
        return None
    timestamps = np.asarray([sample[0] for sample in samples], dtype=float)
    throttle = np.asarray([sample[1] for sample in samples], dtype=float)
    selected = throttle >= threshold_us
    if not np.any(selected):
        return None
    positive_steps = np.diff(timestamps)
    positive_steps = positive_steps[positive_steps > 0.0]
    gap_limit_s = max(
        0.2,
        3.0 * float(np.median(positive_steps)) if len(positive_steps) else 0.2,
    )
    runs: list[tuple[int, int]] = []
    start: int | None = None
    previous: int | None = None
    for index in np.flatnonzero(selected):
        if start is None:
            start = int(index)
        elif previous is not None and (
            index != previous + 1 or timestamps[index] - timestamps[previous] > gap_limit_s
        ):
            runs.append((start, previous))
            start = int(index)
        previous = int(index)
    if start is not None and previous is not None:
        runs.append((start, previous))
    best = max(
        runs,
        key=lambda run: (
            float(np.max(throttle[run[0] : run[1] + 1])),
            timestamps[run[1]] - timestamps[run[0]],
        ),
    )
    return (
        float(timestamps[best[0]]),
        float(timestamps[best[1]]),
        float(np.max(throttle[best[0] : best[1] + 1])),
    )


def _host_window_metrics(
    rows: list[dict[str, str]],
    *,
    throttle_field: str,
    interval: tuple[float, float],
) -> dict[str, Any]:
    selected = [
        row
        for row in rows
        if (timestamp := _number(row.get("elapsed_s"))) is not None
        and interval[0] <= timestamp <= interval[1]
    ]
    result: dict[str, Any] = {}
    throttle = np.asarray(
        [
            value
            for row in selected
            if (value := _number(row.get(throttle_field))) is not None
        ],
        dtype=float,
    )
    if len(throttle):
        result["throttle_us"] = _percentiles(throttle)
    attitude: dict[str, Any] = {}
    for axis in ("roll", "pitch", "yaw"):
        values = np.asarray(
            [
                value
                for row in selected
                if (value := _number(row.get(f"{axis}_deg"))) is not None
            ],
            dtype=float,
        )
        if len(values):
            attitude[axis] = {
                "min_deg": _rounded(np.min(values)),
                "max_deg": _rounded(np.max(values)),
                "p95_abs_deg": _rounded(np.percentile(np.abs(values), 95.0)),
            }
    if attitude:
        result["attitude"] = attitude
    return result


def _thrust_window_metrics(
    mask: np.ndarray,
    *,
    blackbox_time_s: np.ndarray,
    sample_dt_s: np.ndarray,
    blackbox: dict[str, np.ndarray],
    motors: np.ndarray,
    acceleration: np.ndarray,
    load_factor: np.ndarray,
    acc_1g_raw: float,
    host_interval: tuple[float, float],
    blackbox_interval: tuple[float, float],
    pulse_window: bool,
    motor_high_raw: float,
    motor_saturation_raw: float,
    motor_scale_us_per_raw: float | None,
    motor_offset_us: float | None,
) -> dict[str, Any]:
    selected_motors = motors[mask]
    current = blackbox["amperageLatest (A)"][mask]
    voltage = blackbox["vbatLatest (V)"][mask]
    result = _segment_metrics(mask, sample_dt_s, blackbox, motors)
    result.update(
        {
            "host_interval_s": _rounded_interval(host_interval),
            "blackbox_interval_s": _rounded_interval(blackbox_interval),
            "load_factor_g": _percentiles(load_factor[mask]),
            "acceleration_axes_g": {
                f"axis_{index}": _percentiles(acceleration[mask, index] / acc_1g_raw)
                for index in range(3)
            },
            "electrical_power_w": _percentiles(current * voltage),
            "motor_high_duration_s": _rounded(
                np.sum(sample_dt_s[mask][np.any(selected_motors >= motor_high_raw, axis=1)])
            ),
            "motor_saturation_duration_s": _rounded(
                np.sum(
                    sample_dt_s[mask][
                        np.any(selected_motors >= motor_saturation_raw, axis=1)
                    ]
                )
            ),
        }
    )
    if motor_scale_us_per_raw is not None and motor_offset_us is not None:
        result["motor_us_approx"] = {
            "p50": _rounded_vector(
                np.percentile(selected_motors, 50.0, axis=0) * motor_scale_us_per_raw
                + motor_offset_us
            ),
            "p95": _rounded_vector(
                np.percentile(selected_motors, 95.0, axis=0) * motor_scale_us_per_raw
                + motor_offset_us
            ),
            "max": _rounded_vector(
                np.max(selected_motors, axis=0) * motor_scale_us_per_raw
                + motor_offset_us
            ),
        }
    if pulse_window:
        selected_load = load_factor[mask]
        median_dt_s = float(np.median(np.diff(blackbox_time_s)))
        result["load_factor_filtered_max_g"] = {
            f"{window_ms}ms": _rounded(
                _rolling_mean_max(selected_load, window_ms / 1000.0, median_dt_s)
            )
            for window_ms in (20, 50, 100, 200)
        }
        result["load_factor_thresholds"] = {
            f"{threshold_g:g}": {
                "duration_s": _rounded(
                    np.sum(sample_dt_s[mask][selected_load >= threshold_g])
                ),
                "max_contiguous_s": _rounded(
                    _maximum_contiguous_duration(
                        selected_load >= threshold_g, sample_dt_s[mask]
                    )
                ),
            }
            for threshold_g in (1.2, 1.5, 2.0, 2.25)
        }
    return result


def _rolling_mean_max(values: np.ndarray, window_s: float, sample_dt_s: float) -> float:
    width = max(1, min(len(values), int(round(window_s / sample_dt_s))))
    return float(np.max(np.convolve(values, np.ones(width) / width, mode="valid")))


def _maximum_contiguous_duration(mask: np.ndarray, dt_s: np.ndarray) -> float:
    best = 0.0
    current = 0.0
    for active, duration in zip(mask, dt_s):
        if active:
            current += float(duration)
            best = max(best, current)
        else:
            current = 0.0
    return best


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
    pulse_threshold = values["thrust_pulse_threshold_us"]
    plateau_threshold = values["thrust_plateau_threshold_us"]
    if pulse_threshold is None and plateau_threshold is not None:
        raise ValueError("thrust pulse threshold is required with plateau threshold")
    if (
        pulse_threshold is not None
        and plateau_threshold is not None
        and plateau_threshold < pulse_threshold
    ):
        raise ValueError("thrust plateau threshold must not be below pulse threshold")
    if values["acc_1g_raw"] <= 0.0:
        raise ValueError("acc_1g_raw must be positive")
    for field in ("thrust_hover_window_s", "thrust_hover_gap_s", "thrust_post_delay_s"):
        if values[field] < 0.0:
            raise ValueError(f"{field} must be non-negative")


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
