#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
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
    runtime = dict(config.get("msp_runtime", {}))
    mapping = dict(config.get("rc_mapping", {}))
    web_config = dict(config.get("telemetry_web", {}))
    web_enabled = bool(web_config.get("enabled", False))
    publish_hz = float(runtime.get("control_publish_hz", 50.0))
    valid_min_us = int(runtime.get("prefill_valid_min_us", 900))
    valid_max_us = int(runtime.get("prefill_valid_max_us", 2100))
    throttle_channel = int(runtime.get("throttle_channel_zero_based", 2)) + 1
    throttle_max_us = int(mapping.get("throttle_max_us", 1100))
    rate_limits = {
        "roll": float(mapping.get("roll_command_limit_deg_s", 3.0)),
        "pitch": float(mapping.get("pitch_command_limit_deg_s", 3.0)),
        "yaw": float(mapping.get("yaw_command_limit_deg_s", 0.0)),
    }
    thresholds = {
        "publish_hz": publish_hz,
        "max_send_gap_s": 3.0 / max(1.0, publish_hz),
        "valid_rc_min_us": valid_min_us,
        "valid_rc_max_us": valid_max_us,
        "algorithm_throttle_max_us": throttle_max_us,
        "rate_limits_deg_s": rate_limits,
    }

    violations: list[dict[str, Any]] = []
    warnings: list[str] = []
    if not rows:
        violations.append({"code": "empty_log", "count": 1, "first_elapsed_s": None})
    if not meta:
        warnings.append(f"meta_missing_or_invalid:{meta_path}")
    schema_version = _integer(meta.get("log_schema_version"))
    if schema_version is not None and schema_version not in (2, 3):
        warnings.append(f"unsupported_log_schema_version:{schema_version}")

    invalid_rc_rows = []
    exact_885_rows = []
    algorithm_throttle_rows = []
    gate_rows = []
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
            required_worker_gates = (
                "msp_output_enabled",
                "msp_algorithm_authorized",
                "msp_worker_override_active",
                "msp_prefill_ready",
                "physical_rc_fresh",
            )
            if any(_integer(row.get(field)) != 1 for field in required_worker_gates):
                gate_rows.append(row)
        for axis, limit in rate_limits.items():
            value = _number(row.get(f"map_limited_{axis}_rate_deg_s"))
            if value is not None and abs(value) > limit + 1.0e-6:
                rate_rows[axis].append(row)

    _append_violation(violations, "invalid_sent_rc", invalid_rc_rows)
    _append_violation(violations, "sent_885_us", exact_885_rows)
    _append_violation(violations, "algorithm_throttle_envelope", algorithm_throttle_rows)
    _append_violation(violations, "algorithm_without_worker_gates", gate_rows)
    for axis, failed_rows in rate_rows.items():
        _append_violation(violations, f"{axis}_rate_limit", failed_rows)

    max_send_gap_s = _maximum(rows, "msp_send_success_max_interval_s")
    if max_send_gap_s is not None and max_send_gap_s > thresholds["max_send_gap_s"]:
        violations.append(
            {
                "code": "set_raw_rc_gap",
                "count": 1,
                "first_elapsed_s": None,
                "observed": max_send_gap_s,
                "limit": thresholds["max_send_gap_s"],
            }
        )
    final = rows[-1] if rows else {}
    send_errors = _integer(final.get("msp_worker_send_error_count")) or 0
    set_errors = _integer(final.get("msp_cmd_set_raw_rc_error_count")) or 0
    if send_errors > 0 or set_errors > 0:
        violations.append(
            {
                "code": "set_raw_rc_errors",
                "count": max(send_errors, set_errors),
                "first_elapsed_s": None,
            }
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

    metrics = {
        "rows": len(rows),
        "duration_s": _maximum(rows, "elapsed_s"),
        "algorithm_rows": sum(row.get("msp_publish_mode") == "algorithm" for row in rows),
        "set_raw_rc_success_count": _integer(final.get("msp_set_raw_rc_success_count")) or 0,
        "set_raw_rc_error_count": set_errors,
        "max_send_gap_s": max_send_gap_s,
        "publish_deadline_miss_count": _integer(final.get("msp_publish_deadline_miss_count")) or 0,
        "max_set_raw_rc_rtt_ms": _maximum(rows, "msp_cmd_set_raw_rc_max_rtt_ms"),
        "max_raw_imu_rtt_ms": _maximum(rows, "msp_cmd_raw_imu_max_rtt_ms"),
        "max_rknn_total_ms": _maximum(rows, "rknn_total_ms"),
        "max_thermal_c": _maximum(rows, "host_thermal_max_c"),
        "max_gyro_abs_deg_s": _maximum_abs(
            rows,
            ("gyro_roll_deg_s", "gyro_pitch_deg_s", "gyro_yaw_deg_s"),
        ),
        "log_schema_version": schema_version,
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
        "passed": not violations,
        "thresholds": thresholds,
        "metrics": metrics,
        "violations": violations,
        "warnings": warnings,
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
