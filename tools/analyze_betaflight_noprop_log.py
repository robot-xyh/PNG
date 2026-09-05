#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit a Betaflight no-prop CSV log against its bound config.")
    parser.add_argument("--csv", required=True, help="CSV produced by examples/run_betaflight_log_only.py")
    parser.add_argument("--output", default="", help="Output JSON; defaults to <csv-stem>_audit.json")
    return parser.parse_args()


def analyze_log(csv_path: Path) -> dict[str, Any]:
    csv_path = csv_path.expanduser().resolve()
    with csv_path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    meta_path = csv_path.with_name(f"{csv_path.stem}_meta.json")
    meta = _read_json(meta_path)
    config = dict(meta.get("config", {}))
    bench = dict(config.get("bench_profile", {}))
    runtime = dict(config.get("msp_runtime", {}))
    mapping = dict(config.get("rc_mapping", {}))
    guidance_command = dict(config.get("guidance_command", {}))
    safety = dict(config.get("safety", {}))
    motor_interlock = dict(safety.get("motor_output_interlock", {}))
    takeover_interlock = dict(safety.get("takeover_duration_interlock", {}))
    tilt_config = dict(guidance_command.get("tilt_envelope", {}))
    web_config = dict(config.get("telemetry_web", {}))
    web_enabled = bool(web_config.get("enabled", False))
    publish_hz = float(runtime.get("control_publish_hz", 50.0))
    valid_min_us = int(runtime.get("prefill_valid_min_us", 900))
    valid_max_us = int(runtime.get("prefill_valid_max_us", 2100))
    throttle_channel = int(runtime.get("throttle_channel_zero_based", 2)) + 1
    throttle_max_us = int(mapping.get("throttle_max_us", 1100))
    motor_channel_count = min(
        8,
        max(
            1,
            int(
                motor_interlock.get(
                    "channel_count",
                    bench.get("motor_output_channel_count", 4),
                )
            ),
        ),
    )
    motor_output_max_us = max(
        1000,
        int(
            motor_interlock.get(
                "max_output_us",
                bench.get("max_armed_motor_output_us", throttle_max_us + 100),
            )
        ),
    )
    motor_spread_max_us = max(
        0,
        int(
            motor_interlock.get(
                "max_spread_us",
                bench.get("max_armed_motor_spread_us", 150),
            )
        ),
    )
    rate_limits = {
        "roll": float(mapping.get("roll_command_limit_deg_s", 3.0)),
        "pitch": float(mapping.get("pitch_command_limit_deg_s", 3.0)),
        "yaw": float(mapping.get("yaw_command_limit_deg_s", 0.0)),
    }
    takeover_duration_enabled = bool(takeover_interlock.get("enabled", False))
    raw_takeover_duration_max_s = takeover_interlock.get("max_duration_s", 3.0)
    thresholds = {
        "publish_hz": publish_hz,
        "max_send_gap_s": 3.0 / max(1.0, publish_hz),
        "minimum_set_write_rate_hz": max(1.0, publish_hz - 1.0),
        "p999_set_write_gap_s": min(0.04, 2.0 / max(1.0, publish_hz)),
        "maximum_ack_age_s": float(runtime.get("response_stale_s", 0.25)),
        "valid_rc_min_us": valid_min_us,
        "valid_rc_max_us": valid_max_us,
        "algorithm_throttle_max_us": throttle_max_us,
        "armed_motor_output_max_us": motor_output_max_us,
        "armed_motor_spread_max_us": motor_spread_max_us,
        "motor_output_channel_count": motor_channel_count,
        "takeover_duration_enabled": takeover_duration_enabled,
        "takeover_duration_max_s": (
            None
            if raw_takeover_duration_max_s is None
            else float(raw_takeover_duration_max_s)
        ),
        "rate_limits_deg_s": rate_limits,
        "tilt_envelope": {
            "enabled": bool(tilt_config.get("enabled", False)),
            "max_roll_angle_deg": float(tilt_config.get("max_roll_angle_deg", 35.0)),
            "max_pitch_angle_deg": float(tilt_config.get("max_pitch_angle_deg", 35.0)),
            "hardcap_margin_deg": float(tilt_config.get("hardcap_margin_deg", 5.0)),
        },
    }

    violations: list[dict[str, Any]] = []
    warnings: list[str] = []
    if not rows:
        violations.append({"code": "empty_log", "count": 1, "first_elapsed_s": None})
    if not meta:
        warnings.append(f"meta_missing_or_invalid:{meta_path}")
    schema_version = _integer(meta.get("log_schema_version"))
    if schema_version is not None and schema_version not in range(2, 20):
        warnings.append(f"unsupported_log_schema_version:{schema_version}")

    invalid_rc_rows = []
    exact_885_rows = []
    algorithm_throttle_rows = []
    gate_rows = []
    legacy_algorithm_rows = []
    invalid_hold_rows = []
    shaping_nonfinite_rows = []
    shaping_factor_rows = []
    invalid_algorithm_shaping_rows = []
    hardcap_not_leveling_rows = []
    hardcap_flag_rows = []
    invalid_guidance_frame_rows = []
    missing_body_guidance_rows = []
    invalid_command_mapping_rows = []
    accel_tilt_rate_nonfinite_rows = []
    armed_motor_output_rows: list[dict[str, str]] = []
    armed_motor_spread_rows: list[dict[str, str]] = []
    algorithm_motor_interlock_rows: list[dict[str, str]] = []
    active_takeover_interlock_rows: list[dict[str, str]] = []
    rate_rows: dict[str, list[dict[str, str]]] = {axis: [] for axis in rate_limits}
    for row in rows:
        sent = [_number(row.get(f"rc_sent_ch{index}")) for index in range(1, 5)]
        if any(value is not None and (value < valid_min_us or value > valid_max_us) for value in sent):
            invalid_rc_rows.append(row)
        if any(value == 885.0 for value in sent):
            exact_885_rows.append(row)
        if row.get("msp_publish_mode") == "algorithm":
            throttle = _number(row.get(f"rc_sent_ch{throttle_channel}"))
            if throttle is not None and throttle > throttle_max_us:
                algorithm_throttle_rows.append(row)
            if schema_version is not None and schema_version >= 4:
                required_publish_gates = (
                    "msp_last_publish_output_enabled",
                    "msp_last_publish_algorithm_authorized",
                    "msp_last_publish_override_active",
                    "msp_last_publish_prefill_ready",
                    "msp_last_publish_physical_rc_fresh",
                    "msp_last_publish_command_fresh",
                )
                if schema_version >= 7:
                    required_publish_gates += ("msp_last_publish_set_raw_rc_ack_fresh",)
                if any(_integer(row.get(field)) != 1 for field in required_publish_gates):
                    gate_rows.append(row)
            else:
                legacy_algorithm_rows.append(row)
            if schema_version is not None and schema_version >= 13 and (
                _integer(row.get("motor_interlock_ok")) != 1
                or _integer(row.get("motor_interlock_latched")) == 1
            ):
                algorithm_motor_interlock_rows.append(row)
        if schema_version is not None and schema_version >= 14 and row.get("safety_state") == "ACTIVE":
            duration_s = _number(row.get("takeover_duration_s"))
            if (
                _integer(row.get("takeover_duration_interlock_ok")) != 1
                or _integer(row.get("takeover_duration_interlock_latched")) == 1
                or duration_s is None
                or (
                    thresholds["takeover_duration_enabled"]
                    and thresholds["takeover_duration_max_s"] is not None
                    and duration_s > thresholds["takeover_duration_max_s"] + 1.0e-6
                )
            ):
                active_takeover_interlock_rows.append(row)
        motor_outputs = [
            _number(row.get(f"motor_output_ch{index}"))
            for index in range(1, motor_channel_count + 1)
        ]
        valid_motor_outputs = [value for value in motor_outputs if value is not None and value > 0.0]
        motor_output_active = (
            _integer(row.get("armed")) == 1 or row.get("msp_publish_mode") == "algorithm"
        )
        if motor_output_active and valid_motor_outputs:
            if max(valid_motor_outputs) > motor_output_max_us:
                armed_motor_output_rows.append(row)
            if max(valid_motor_outputs) - min(valid_motor_outputs) > motor_spread_max_us:
                armed_motor_spread_rows.append(row)
        if schema_version is not None and schema_version >= 4 and row.get("sp_source") == "guidance_hold":
            allowed_gap_reasons = {"perception_no_new_result"}
            if schema_version >= 6:
                allowed_gap_reasons.add("fusion_waiting_for_attitude")
            if (
                row.get("detector_reject_reason") not in allowed_gap_reasons
                or _integer(row.get("perception_new_result")) != 0
                or _integer(row.get("watchdog_ok")) != 1
            ):
                invalid_hold_rows.append(row)
        for axis, limit in rate_limits.items():
            value = _number(row.get(f"map_limited_{axis}_rate_deg_s"))
            if value is not None and abs(value) > limit + 1.0e-6:
                rate_rows[axis].append(row)
        if schema_version is not None and schema_version >= 8:
            shaping_numbers = tuple(
                _number(row.get(field))
                for field in (
                    "pre_shape_sp_roll_rate_deg_s",
                    "pre_shape_sp_pitch_rate_deg_s",
                    "sp_roll_rate_deg_s",
                    "sp_pitch_rate_deg_s",
                    "entry_handoff_progress",
                    "tilt_roll_softcap_factor",
                    "tilt_pitch_softcap_factor",
                    "tilt_roll_level_weight",
                    "tilt_pitch_level_weight",
                )
            )
            shaping_valid = _integer(row.get("shaping_valid")) == 1
            shaping_nonfinite = any(value is None for value in shaping_numbers) and (
                schema_version is None or schema_version < 12 or shaping_valid
            )
            factors = shaping_numbers[4:]
            if (schema_version is None or schema_version < 12 or shaping_valid) and any(
                value is not None and not 0.0 <= value <= 1.0 for value in factors
            ):
                shaping_factor_rows.append(row)
            if row.get("msp_publish_mode") == "algorithm":
                if schema_version is not None and schema_version >= 12:
                    worker_command_valid = (
                        _integer(row.get("msp_last_publish_command_active")) == 1
                    )
                    if not worker_command_valid:
                        invalid_algorithm_shaping_rows.append(row)
                else:
                    if not shaping_valid:
                        invalid_algorithm_shaping_rows.append(row)
                # The asynchronous MSP worker can still report the preceding
                # algorithm command for one row after the main-loop gate closes.
                # In that transition row no current attitude is consumed by the
                # shaper, so validate the published command through the worker
                # timebase above instead of requiring empty current-frame inputs.
                current_shaping_applied = (
                    shaping_valid and row.get("shaping_reason") != "gate_closed"
                )
                if current_shaping_applied and bool(tilt_config.get("enabled", False)):
                    axis_values = (
                        (
                            _number(row.get("tilt_roll_attitude_deg")),
                            _number(row.get("sp_roll_rate_deg_s")),
                            float(tilt_config.get("max_roll_angle_deg", 35.0)),
                        ),
                        (
                            _number(row.get("tilt_pitch_attitude_deg")),
                            _number(row.get("sp_pitch_rate_deg_s")),
                            float(tilt_config.get("max_pitch_angle_deg", 35.0)),
                        ),
                    )
                    hard_margin = float(tilt_config.get("hardcap_margin_deg", 5.0))
                    hard_region = False
                    hardcap_not_leveling = False
                    for attitude, output_rate, max_angle in axis_values:
                        if attitude is None or output_rate is None:
                            shaping_nonfinite = True
                            continue
                        if abs(attitude) + 1.0e-6 >= max_angle + hard_margin:
                            hard_region = True
                            if attitude * output_rate >= -1.0e-6:
                                hardcap_not_leveling = True
                    if hardcap_not_leveling:
                        hardcap_not_leveling_rows.append(row)
                    if hard_region and _integer(row.get("tilt_hardcap_active")) != 1:
                        hardcap_flag_rows.append(row)
            if shaping_nonfinite:
                shaping_nonfinite_rows.append(row)
        if schema_version is not None and schema_version >= 11:
            if (
                row.get("guidance_eval_frame") != "inertial_ned"
                or row.get("rate_gain_input_frame") != "body_frd"
            ):
                invalid_guidance_frame_rows.append(row)
            if _integer(row.get("guidance_valid")) == 1 and any(
                _number(row.get(field)) is None
                for field in (
                    "g_eval_body_frd_x",
                    "g_eval_body_frd_y",
                    "g_eval_body_frd_z",
                )
            ):
                missing_body_guidance_rows.append(row)
        if schema_version is not None and schema_version >= 15:
            command_mapping = row.get("command_mapping_type")
            if command_mapping not in {"direct_rate_matrix", "accel_tilt_rate"}:
                invalid_command_mapping_rows.append(row)
            if command_mapping == "accel_tilt_rate" and _integer(
                row.get("pre_shape_sp_valid")
            ) == 1:
                required_accel_mapping_fields = (
                    "command_desired_roll_angle_deg",
                    "command_desired_pitch_angle_deg",
                    "command_current_roll_angle_deg",
                    "command_current_pitch_angle_deg",
                    "command_roll_attitude_error_deg",
                    "command_pitch_attitude_error_deg",
                )
                if any(
                    _number(row.get(field)) is None
                    for field in required_accel_mapping_fields
                ):
                    accel_tilt_rate_nonfinite_rows.append(row)

    _append_violation(violations, "invalid_sent_rc", invalid_rc_rows)
    _append_violation(violations, "sent_885_us", exact_885_rows)
    _append_violation(violations, "algorithm_throttle_envelope", algorithm_throttle_rows)
    _append_violation(violations, "algorithm_without_worker_gates", gate_rows)
    _append_violation(violations, "publish_gate_timebase_unavailable", legacy_algorithm_rows)
    _append_violation(violations, "guidance_hold_outside_perception_gap", invalid_hold_rows)
    _append_violation(violations, "command_shaping_nonfinite", shaping_nonfinite_rows)
    _append_violation(violations, "command_shaping_factor_out_of_range", shaping_factor_rows)
    _append_violation(violations, "algorithm_with_invalid_command_shaping", invalid_algorithm_shaping_rows)
    legacy_shaping_mismatch_rows = [
        row
        for row in rows
        if schema_version is not None
        and 8 <= schema_version < 12
        and row.get("msp_publish_mode") == "algorithm"
        and _integer(row.get("shaping_valid")) != 1
    ]
    if legacy_shaping_mismatch_rows:
        warnings.append(
            f"algorithm_shaping_timebase_unavailable:{len(legacy_shaping_mismatch_rows)}"
        )
    _append_violation(violations, "tilt_hardcap_not_leveling", hardcap_not_leveling_rows)
    _append_violation(violations, "tilt_hardcap_flag_missing", hardcap_flag_rows)
    _append_violation(violations, "invalid_guidance_command_frames", invalid_guidance_frame_rows)
    _append_violation(violations, "guidance_body_vector_missing", missing_body_guidance_rows)
    _append_violation(violations, "invalid_guidance_command_mapping", invalid_command_mapping_rows)
    _append_violation(
        violations,
        "accel_tilt_rate_diagnostics_nonfinite",
        accel_tilt_rate_nonfinite_rows,
    )
    _append_violation(
        violations,
        "algorithm_without_motor_interlock",
        algorithm_motor_interlock_rows,
    )
    _append_violation(
        violations,
        "active_without_takeover_duration_interlock",
        active_takeover_interlock_rows,
    )
    _append_threshold_violation(
        violations,
        "armed_motor_output_high",
        armed_motor_output_rows,
        observed=_maximum_abs(
            armed_motor_output_rows,
            tuple(f"motor_output_ch{index}" for index in range(1, motor_channel_count + 1)),
        ),
        limit=motor_output_max_us,
    )
    _append_threshold_violation(
        violations,
        "armed_motor_spread_high",
        armed_motor_spread_rows,
        observed=_maximum_motor_spread(armed_motor_spread_rows, motor_channel_count),
        limit=motor_spread_max_us,
    )
    for axis, failed_rows in rate_rows.items():
        _append_violation(violations, f"{axis}_rate_limit", failed_rows)

    max_send_gap_field = (
        "msp_set_raw_rc_write_max_interval_s"
        if schema_version is not None and schema_version >= 7
        else "msp_send_success_max_interval_s"
    )
    max_send_gap_s = _maximum(rows, max_send_gap_field)
    if max_send_gap_s is not None and max_send_gap_s > thresholds["max_send_gap_s"]:
        first_gap_row = next(
            (
                row
                for row in rows
                if (_number(row.get(max_send_gap_field)) or 0.0) > thresholds["max_send_gap_s"]
            ),
            None,
        )
        violations.append(
            {
                "code": "set_raw_rc_gap",
                "count": 1,
                "first_elapsed_s": (
                    None if first_gap_row is None else _number(first_gap_row.get("elapsed_s"))
                ),
                "observed": max_send_gap_s,
                "limit": thresholds["max_send_gap_s"],
            }
        )
    final = rows[-1] if rows else {}
    send_errors = _integer(final.get("msp_worker_send_error_count")) or 0
    set_errors = _integer(final.get("msp_cmd_set_raw_rc_error_count")) or 0
    write_errors = _integer(final.get("msp_set_raw_rc_write_error_count")) or 0
    if send_errors > 0 or set_errors > 0 or write_errors > 0:
        violations.append(
            {
                "code": "set_raw_rc_errors",
                "count": max(send_errors, set_errors, write_errors),
                "first_elapsed_s": None,
            }
        )
    if schema_version is not None and schema_version >= 7:
        write_rate_hz = _number(final.get("msp_set_raw_rc_write_rate_hz"))
        if write_rate_hz is not None and write_rate_hz < thresholds["minimum_set_write_rate_hz"]:
            violations.append(
                {
                    "code": "set_raw_rc_write_rate_low",
                    "count": 1,
                    "first_elapsed_s": None,
                    "observed": write_rate_hz,
                    "limit": thresholds["minimum_set_write_rate_hz"],
                }
            )
        p999_gap_s = _maximum(rows, "msp_set_raw_rc_write_p999_interval_s")
        if p999_gap_s is not None and p999_gap_s > thresholds["p999_set_write_gap_s"]:
            first_p999_row = next(
                (
                    row
                    for row in rows
                    if (_number(row.get("msp_set_raw_rc_write_p999_interval_s")) or 0.0)
                    > thresholds["p999_set_write_gap_s"]
                ),
                None,
            )
            violations.append(
                {
                    "code": "set_raw_rc_write_p999_gap",
                    "count": 1,
                    "first_elapsed_s": (
                        None if first_p999_row is None else _number(first_p999_row.get("elapsed_s"))
                    ),
                    "observed": p999_gap_s,
                    "limit": thresholds["p999_set_write_gap_s"],
                }
            )
        output_rows = [row for row in rows if _integer(row.get("msp_output_enabled")) == 1]
        max_ack_age_s = _maximum(output_rows, "msp_set_raw_rc_ack_age_s")
        stale_ack_rows = [
            row
            for row in rows
            if row.get("msp_publish_mode") == "algorithm"
            and _integer(row.get("msp_last_publish_set_raw_rc_ack_fresh")) != 1
        ]
        _append_violation(violations, "algorithm_with_stale_set_ack", stale_ack_rows)
        if max_ack_age_s is not None and max_ack_age_s > thresholds["maximum_ack_age_s"]:
            violations.append(
                {
                    "code": "set_raw_rc_ack_stall",
                    "count": 1,
                    "first_elapsed_s": None,
                    "observed": max_ack_age_s,
                    "limit": thresholds["maximum_ack_age_s"],
                }
            )
        parser_error_count = max(
            _integer(final.get("msp_rx_checksum_error_count")) or 0,
            _integer(final.get("msp_rx_parser_error_count")) or 0,
        )
        if parser_error_count > 0:
            violations.append(
                {"code": "msp_response_parser_errors", "count": parser_error_count, "first_elapsed_s": None}
            )
    motor_poll_hz = float(runtime.get("motor_poll_hz", 0.0))
    motor_errors = _integer(final.get("msp_cmd_motor_error_count")) or 0
    motor_success = _integer(final.get("msp_cmd_motor_success_count")) or 0
    if motor_poll_hz > 0.0 and motor_success <= 0:
        violations.append(
            {"code": "motor_telemetry_missing", "count": 1, "first_elapsed_s": None}
        )
    if motor_errors > 0:
        violations.append(
            {"code": "motor_telemetry_errors", "count": motor_errors, "first_elapsed_s": None}
        )
    web_errors = _integer(final.get("web_error_count")) or 0
    web_publish_count = _integer(final.get("web_publish_count")) or 0
    if web_enabled and web_publish_count <= 0:
        violations.append(
            {
                "code": "web_no_telemetry_published",
                "count": 1,
                "first_elapsed_s": None,
            }
        )
    if web_enabled and web_errors > 0:
        violations.append(
            {
                "code": "web_runtime_errors",
                "count": web_errors,
                "first_elapsed_s": None,
                "last_error": final.get("web_last_error", ""),
            }
        )

    post_disarm_tail_configured_s = _number(
        dict(meta.get("args", {})).get("stop_after_disarm_s")
    )
    post_disarm_tail_logged_s: float | None = None
    post_disarm_edge_elapsed_s: float | None = None
    armed_seen = False
    previous_armed: int | None = None
    for row in rows:
        current_armed = _integer(row.get("armed"))
        if current_armed not in (0, 1):
            continue
        armed_seen = armed_seen or current_armed == 1
        if previous_armed == 1 and current_armed == 0:
            post_disarm_edge_elapsed_s = _number(row.get("elapsed_s"))
        previous_armed = current_armed
    if post_disarm_edge_elapsed_s is not None and rows:
        final_elapsed_s = _number(rows[-1].get("elapsed_s"))
        if final_elapsed_s is not None:
            post_disarm_tail_logged_s = max(0.0, final_elapsed_s - post_disarm_edge_elapsed_s)
    completion = dict(meta.get("completion", {}))
    if (
        schema_version is not None
        and schema_version >= 17
        and post_disarm_tail_configured_s is not None
        and post_disarm_tail_configured_s > 0.0
        and armed_seen
    ):
        tail_complete = (
            completion.get("complete") is True
            and completion.get("stop_reason") == "post_disarm_tail_complete"
            and completion.get("post_disarm_tail_completed") is True
            and post_disarm_tail_logged_s is not None
            and post_disarm_tail_logged_s + 1.0e-6 >= post_disarm_tail_configured_s
            and previous_armed == 0
        )
        if not tail_complete:
            violations.append(
                {
                    "code": "post_disarm_log_tail_incomplete",
                    "count": 1,
                    "first_elapsed_s": post_disarm_edge_elapsed_s,
                    "observed": post_disarm_tail_logged_s,
                    "limit": post_disarm_tail_configured_s,
                    "stop_reason": completion.get("stop_reason"),
                }
            )

    metrics = {
        "rows": len(rows),
        "duration_s": _maximum(rows, "elapsed_s"),
        "algorithm_rows": sum(row.get("msp_publish_mode") == "algorithm" for row in rows),
        "guidance_hold_rows": sum(row.get("sp_source") == "guidance_hold" for row in rows),
        "entry_handoff_rows": sum(_integer(row.get("entry_handoff_active")) == 1 for row in rows),
        "tilt_hardcap_rows": sum(_integer(row.get("tilt_hardcap_active")) == 1 for row in rows),
        "set_raw_rc_success_count": _integer(final.get("msp_set_raw_rc_success_count")) or 0,
        "set_raw_rc_write_success_count": _integer(final.get("msp_set_raw_rc_write_success_count")) or 0,
        "set_raw_rc_ack_count": _integer(final.get("msp_set_raw_rc_ack_count")) or 0,
        "set_raw_rc_error_count": set_errors,
        "set_raw_rc_write_error_count": write_errors,
        "max_send_gap_s": max_send_gap_s,
        "set_raw_rc_write_rate_hz": _number(final.get("msp_set_raw_rc_write_rate_hz")),
        "set_raw_rc_write_p999_interval_s": _maximum(rows, "msp_set_raw_rc_write_p999_interval_s"),
        "evidence_frame_write_count": _integer(final.get("evidence_frame_write_count")) or 0,
        "evidence_frame_error_count": _integer(final.get("evidence_frame_error_count")) or 0,
        "max_set_raw_rc_ack_age_s": (
            _maximum(
                (row for row in rows if _integer(row.get("msp_output_enabled")) == 1),
                "msp_set_raw_rc_ack_age_s",
            )
        ),
        "msp_rx_discarded_bytes": _integer(final.get("msp_rx_discarded_bytes")) or 0,
        "msp_rx_checksum_error_count": _integer(final.get("msp_rx_checksum_error_count")) or 0,
        "msp_rx_parser_error_count": _integer(final.get("msp_rx_parser_error_count")) or 0,
        "publish_deadline_miss_count": _integer(final.get("msp_publish_deadline_miss_count")) or 0,
        "max_set_raw_rc_rtt_ms": _maximum(rows, "msp_cmd_set_raw_rc_max_rtt_ms"),
        "max_raw_imu_rtt_ms": _maximum(rows, "msp_cmd_raw_imu_max_rtt_ms"),
        "max_rknn_total_ms": _maximum(rows, "rknn_total_ms"),
        "max_thermal_c": _maximum(rows, "host_thermal_max_c"),
        "max_gyro_abs_deg_s": _maximum_abs(
            rows,
            ("gyro_roll_deg_s", "gyro_pitch_deg_s", "gyro_yaw_deg_s"),
        ),
        "max_gyro_msp_raw_abs": _maximum_abs(
            rows,
            ("gyro_msp_raw_x", "gyro_msp_raw_y", "gyro_msp_raw_z"),
        ),
        "motor_telemetry_rows": sum(
            _integer(row.get("motor_output_count")) not in (None, 0) for row in rows
        ),
        "max_motor_output": _maximum_abs(
            rows,
            tuple(f"motor_output_ch{index}" for index in range(1, 9)),
        ),
        "max_armed_motor_output": _maximum_abs(
            (
                row
                for row in rows
                if _integer(row.get("armed")) == 1 or row.get("msp_publish_mode") == "algorithm"
            ),
            tuple(f"motor_output_ch{index}" for index in range(1, motor_channel_count + 1)),
        ),
        "max_armed_motor_spread": _maximum_motor_spread(
            (
                row
                for row in rows
                if _integer(row.get("armed")) == 1 or row.get("msp_publish_mode") == "algorithm"
            ),
            motor_channel_count,
        ),
        "max_takeover_duration_s": _maximum(rows, "takeover_duration_s"),
        "max_loop_period_s": _maximum(rows, "loop_period_s"),
        "max_python_gc_pause_ms": _maximum(rows, "python_gc_max_pause_ms"),
        "log_schema_version": schema_version,
        "post_disarm_tail_configured_s": post_disarm_tail_configured_s,
        "post_disarm_tail_logged_s": post_disarm_tail_logged_s,
        "post_disarm_tail_completed": completion.get("post_disarm_tail_completed"),
        "run_stop_reason": completion.get("stop_reason"),
        "web_enabled": web_enabled,
        "web_publish_count": web_publish_count,
        "web_preview_encode_count": _integer(final.get("web_preview_encode_count")) or 0,
        "web_preview_drop_count": _integer(final.get("web_preview_drop_count")) or 0,
        "web_error_count": web_errors,
    }
    events_path = str(meta.get("log_events_jsonl", ""))
    if events_path and not Path(events_path).expanduser().is_file():
        warnings.append(f"events_missing:{events_path}")
    return {
        "audit_schema_version": 1,
        "source_csv": str(csv_path),
        "source_meta": str(meta_path),
        "source_bindings": {
            "csv": _file_binding(csv_path),
            "meta": _file_binding(meta_path),
        },
        "passed": not violations,
        "thresholds": thresholds,
        "metrics": metrics,
        "violations": violations,
        "warnings": warnings,
    }


def _file_binding(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"path": str(path), "sha256": "", "bytes": 0}
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {
        "path": str(path.resolve()),
        "sha256": digest.hexdigest(),
        "bytes": path.stat().st_size,
    }


def _append_violation(violations: list[dict[str, Any]], code: str, rows: list[dict[str, str]]) -> None:
    if rows:
        violations.append(
            {
                "code": code,
                "count": len(rows),
                "first_elapsed_s": _number(rows[0].get("elapsed_s")),
            }
        )


def _append_threshold_violation(
    violations: list[dict[str, Any]],
    code: str,
    rows: list[dict[str, str]],
    *,
    observed: float | None,
    limit: float,
) -> None:
    if not rows:
        return
    violations.append(
        {
            "code": code,
            "count": len(rows),
            "first_elapsed_s": _number(rows[0].get("elapsed_s")),
            "observed": observed,
            "limit": limit,
        }
    )


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _number(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _integer(value: Any) -> int | None:
    number = _number(value)
    return None if number is None else int(number)


def _maximum(rows: Iterable[dict[str, str]], field: str) -> float | None:
    values = [_number(row.get(field)) for row in rows]
    return max((value for value in values if value is not None), default=None)


def _maximum_abs(rows: Iterable[dict[str, str]], fields: Iterable[str]) -> float | None:
    values = [_number(row.get(field)) for row in rows for field in fields]
    return max((abs(value) for value in values if value is not None), default=None)


def _maximum_motor_spread(rows: Iterable[dict[str, str]], channel_count: int) -> float | None:
    maximum = None
    for row in rows:
        values = [
            _number(row.get(f"motor_output_ch{index}"))
            for index in range(1, int(channel_count) + 1)
        ]
        valid = [value for value in values if value is not None and value > 0.0]
        if len(valid) < 2:
            continue
        spread = max(valid) - min(valid)
        maximum = spread if maximum is None else max(maximum, spread)
    return maximum


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv)
    result = analyze_log(csv_path)
    output_path = (
        Path(args.output).expanduser()
        if args.output
        else csv_path.expanduser().with_name(f"{csv_path.stem}_audit.json")
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"audit_json={output_path}")
    print(f"passed={int(result['passed'])} violations={len(result['violations'])}")
    for violation in result["violations"]:
        print(f"violation={violation['code']} count={violation['count']}")
    raise SystemExit(0 if result["passed"] else 2)


if __name__ == "__main__":
    main()
