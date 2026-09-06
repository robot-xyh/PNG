#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import time
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision_guidance.betaflight_sitl import SITL_SCOPE  # noqa: E402


AUDIT_SCHEMA_VERSION = 1
EVIDENCE_TYPE = "betaflight_gazebo_sil_audit"
OFFICIAL_BETAFLIGHT_COMMIT = "79065c96ba0bb5cdc675e67d7093e05dab8b330e"
OFFICIAL_BETAFLIGHT_ELF_SHA256 = (
    "f4e4456aae4f079d1349dc7bc4037211897260eeeb8cc9c4e5691949996212be"
)
REQUIRED_ARTIFACTS = (
    "flight_config",
    "sitl_config",
    "configuration_manifest",
    "betaflight_binary",
    "betaflight_cli",
    "eeprom",
    "gazebo_world",
    "gazebo_bridge_source",
    "gazebo_bridge_library",
    "interceptor_model",
    "target_model",
    "runner_csv",
    "runner_meta",
    "runner_manifest",
    "betaflight_console",
    "gazebo_console",
    "runner_console",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Audit one official Betaflight/Gazebo SIL run and bind its evidence."
    )
    parser.add_argument("--orchestration-manifest", required=True)
    parser.add_argument("--output", default="")
    return parser.parse_args()


def sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError(f"JSON root must be an object: {path}")
    return value


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _artifact_path(
    artifacts: dict[str, Any],
    name: str,
    violations: list[str],
) -> Path | None:
    value = artifacts.get(name)
    if not isinstance(value, dict):
        violations.append(f"artifact_missing:{name}")
        return None
    path = Path(str(value.get("path", ""))).expanduser()
    if not path.is_file():
        violations.append(f"artifact_file_missing:{name}")
        return None
    actual = sha256_path(path)
    if actual != str(value.get("sha256", "")):
        violations.append(f"artifact_sha256_mismatch:{name}")
        return None
    return path.resolve()


def _float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return default


def _int(row: dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(float(row.get(key, "")))
    except (TypeError, ValueError):
        return default


def _transition_count(rows: list[dict[str, str]], field: str, active: str = "1") -> int:
    previous = ""
    count = 0
    for row in rows:
        value = row.get(field, "")
        if value == active and previous != active:
            count += 1
        previous = value
    return count


def _contiguous_segments(rows: list[dict[str, str]], field: str, value: str) -> list[list[dict[str, str]]]:
    segments: list[list[dict[str, str]]] = []
    current: list[dict[str, str]] = []
    for row in rows:
        if row.get(field) == value:
            current.append(row)
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)
    return segments


def _pearson(left: Iterable[float], right: Iterable[float]) -> float:
    x = list(left)
    y = list(right)
    if len(x) != len(y) or len(x) < 4:
        return math.nan
    mean_x = statistics.fmean(x)
    mean_y = statistics.fmean(y)
    centered_x = [value - mean_x for value in x]
    centered_y = [value - mean_y for value in y]
    scale_x = math.sqrt(sum(value * value for value in centered_x))
    scale_y = math.sqrt(sum(value * value for value in centered_y))
    if scale_x <= 1.0e-9 or scale_y <= 1.0e-9:
        return math.nan
    return sum(a * b for a, b in zip(centered_x, centered_y)) / (scale_x * scale_y)


def _best_delayed_correlation(
    rows: list[dict[str, str]],
    *,
    command_field: str,
    response_field: str,
    active_field: str = "msp_publish_mode",
    active_value: str = "algorithm",
    maximum_lag_s: float = 0.4,
    command_scale: float = 1.0,
) -> dict[str, float | int]:
    elapsed = [_float(row, "elapsed_s") for row in rows]
    periods = [
        elapsed[index] - elapsed[index - 1]
        for index in range(1, len(elapsed))
        if math.isfinite(elapsed[index])
        and math.isfinite(elapsed[index - 1])
        and elapsed[index] > elapsed[index - 1]
    ]
    period_s = statistics.median(periods) if periods else 0.02
    maximum_lag_rows = max(0, int(math.ceil(maximum_lag_s / period_s)))
    best = {"correlation": math.nan, "lag_s": math.nan, "sample_count": 0}
    for lag in range(maximum_lag_rows + 1):
        commands: list[float] = []
        responses: list[float] = []
        limit = len(rows) - lag
        for index in range(max(0, limit)):
            if rows[index].get(active_field) != active_value:
                continue
            command = command_scale * _float(rows[index], command_field)
            response = _float(rows[index + lag], response_field)
            if math.isfinite(command) and math.isfinite(response):
                commands.append(command)
                responses.append(response)
        correlation = _pearson(commands, responses)
        if math.isfinite(correlation) and (
            not math.isfinite(float(best["correlation"]))
            or correlation > float(best["correlation"])
        ):
            best = {
                "correlation": correlation,
                "lag_s": lag * period_s,
                "sample_count": len(commands),
            }
    return best


def _motor_direction_metrics(
    rows: list[dict[str, str]],
    axis: str,
    *,
    maximum_lag_s: float = 0.25,
    onset_window_s: float = 0.30,
) -> dict[str, float | int]:
    elapsed = [_float(row, "elapsed_s") for row in rows]
    periods = [
        elapsed[index] - elapsed[index - 1]
        for index in range(1, len(elapsed))
        if math.isfinite(elapsed[index])
        and math.isfinite(elapsed[index - 1])
        and elapsed[index] > elapsed[index - 1]
    ]
    period_s = statistics.median(periods) if periods else 0.02
    maximum_lag_rows = max(0, int(math.ceil(maximum_lag_s / period_s)))
    best: dict[str, float | int] = {
        "correlation": math.nan,
        "lag_s": math.nan,
        "sample_count": 0,
    }
    segment_started_s: float | None = None
    command_eligible: list[bool] = []
    for row in rows:
        active = row.get("msp_publish_mode") == "algorithm"
        elapsed_s = _float(row, "elapsed_s")
        if active and segment_started_s is None:
            segment_started_s = elapsed_s
        command_eligible.append(
            bool(
                active
                and segment_started_s is not None
                and math.isfinite(elapsed_s)
                and elapsed_s - segment_started_s < onset_window_s
            )
        )
        if not active:
            segment_started_s = None
    for lag in range(maximum_lag_rows + 1):
        commands: list[float] = []
        differentials: list[float] = []
        for index in range(0, len(rows) - lag):
            row = rows[index]
            if not command_eligible[index]:
                continue
            command = _float(row, f"sp_{axis}_rate_deg_s")
            response = rows[index + lag]
            motors = [
                _float(response, f"motor_output_ch{motor_index}")
                for motor_index in range(1, 5)
            ]
            if not math.isfinite(command) or not all(
                math.isfinite(value) for value in motors
            ):
                continue
            if axis == "roll":
                differential = motors[2] + motors[3] - motors[0] - motors[1]
            elif axis == "pitch":
                differential = motors[0] + motors[2] - motors[1] - motors[3]
            else:
                raise ValueError(f"unsupported motor direction axis: {axis}")
            commands.append(command)
            differentials.append(differential)
        correlation = _pearson(commands, differentials)
        if math.isfinite(correlation) and (
            not math.isfinite(float(best["correlation"]))
            or correlation > float(best["correlation"])
        ):
            best = {
                "correlation": correlation,
                "lag_s": lag * period_s,
                "sample_count": len(commands),
            }
    return best


def _ned_truth_metrics(
    rows: list[dict[str, str]], *, maximum_lag_s: float = 0.5
) -> dict[str, Any]:
    elapsed = [_float(row, "elapsed_s") for row in rows]
    periods = [
        elapsed[index] - elapsed[index - 1]
        for index in range(1, len(elapsed))
        if math.isfinite(elapsed[index])
        and math.isfinite(elapsed[index - 1])
        and elapsed[index] > elapsed[index - 1]
    ]
    period_s = statistics.median(periods) if periods else 0.02
    maximum_lag_rows = max(0, int(math.ceil(maximum_lag_s / period_s)))
    axes = {"n": "n", "e": "e", "d": "d"}
    results: dict[str, Any] = {}
    horizontal_axes_with_motion = 0
    for axis, suffix in axes.items():
        candidates: list[dict[str, float | int]] = []
        for lag in range(maximum_lag_rows + 1):
            sign_matches = 0
            absolute_errors: list[float] = []
            for index in range(0, len(rows) - lag):
                truth = _float(
                    rows[index], f"sitl_expected_velocity_ned_{suffix}_m_s"
                )
                future_truth = _float(
                    rows[index + lag],
                    f"sitl_expected_velocity_ned_{suffix}_m_s",
                )
                measured = _float(
                    rows[index + lag],
                    f"kinematics_velocity_filtered_{suffix}_m_s",
                )
                if (
                    not math.isfinite(truth)
                    or not math.isfinite(future_truth)
                    or not math.isfinite(measured)
                    or abs(truth) < 0.25
                    or abs(future_truth) < 0.25
                    or math.copysign(1.0, truth)
                    != math.copysign(1.0, future_truth)
                ):
                    continue
                sign_matches += int(
                    math.copysign(1.0, truth)
                    == math.copysign(1.0, measured)
                )
                absolute_errors.append(abs(truth - measured))
            count = len(absolute_errors)
            candidates.append(
                {
                    "sample_count": count,
                    "sign_match_fraction": (
                        sign_matches / count if count else math.nan
                    ),
                    "median_absolute_error_m_s": (
                        statistics.median(absolute_errors)
                        if absolute_errors
                        else math.nan
                    ),
                    "lag_s": lag * period_s,
                }
            )
        maximum_count = max(
            (int(candidate["sample_count"]) for candidate in candidates),
            default=0,
        )
        minimum_count = max(5, int(math.ceil(0.6 * maximum_count)))
        eligible = [
            candidate
            for candidate in candidates
            if int(candidate["sample_count"]) >= minimum_count
            and math.isfinite(float(candidate["sign_match_fraction"]))
            and math.isfinite(float(candidate["median_absolute_error_m_s"]))
        ]
        best = (
            max(
                eligible,
                key=lambda candidate: (
                    float(candidate["sign_match_fraction"]),
                    -float(candidate["median_absolute_error_m_s"]),
                    -float(candidate["lag_s"]),
                ),
            )
            if eligible
            else {
                "sample_count": 0,
                "sign_match_fraction": math.nan,
                "median_absolute_error_m_s": math.nan,
                "lag_s": math.nan,
            }
        )
        results[axis] = best
        if axis in {"n", "e"} and int(best["sample_count"]) >= 5:
            horizontal_axes_with_motion += 1
    results["horizontal_axes_with_motion"] = horizontal_axes_with_motion
    return results


def evaluate_msp_timing(final_row: dict[str, str]) -> tuple[dict[str, float | int], list[str]]:
    timing: dict[str, float | int] = {
        "mean_write_rate_hz": _float(final_row, "msp_set_raw_rc_write_rate_hz"),
        "p99_9_write_interval_ms": 1000.0
        * _float(final_row, "msp_set_raw_rc_write_p999_interval_s"),
        "maximum_write_interval_ms": 1000.0
        * _float(final_row, "msp_set_raw_rc_write_max_interval_s"),
        "write_attempt_count": _int(final_row, "msp_set_raw_rc_write_attempt_count"),
        "write_success_count": _int(final_row, "msp_set_raw_rc_write_success_count"),
        "ack_count": _int(final_row, "msp_set_raw_rc_ack_count"),
        "pending_depth": _int(final_row, "msp_set_raw_rc_pending_depth"),
    }
    violations: list[str] = []
    if (
        not math.isfinite(float(timing["mean_write_rate_hz"]))
        or float(timing["mean_write_rate_hz"]) < 49.0
    ):
        violations.append("msp_write_rate_below_49_hz")
    if (
        not math.isfinite(float(timing["p99_9_write_interval_ms"]))
        or float(timing["p99_9_write_interval_ms"]) > 40.0
    ):
        violations.append("msp_write_p99_9_above_40_ms")
    if (
        not math.isfinite(float(timing["maximum_write_interval_ms"]))
        or float(timing["maximum_write_interval_ms"]) > 60.0
    ):
        violations.append("msp_write_max_above_60_ms")
    return timing, violations


def _verify_runner_manifest(
    manifest: dict[str, Any], violations: list[str]
) -> None:
    completion = manifest.get("completion", {})
    if not isinstance(completion, dict) or completion.get("complete") is not True:
        violations.append("runner_incomplete")
    for item in manifest.get("artifacts", []):
        if not isinstance(item, dict):
            violations.append("runner_artifact_invalid")
            continue
        path = Path(str(item.get("path", ""))).expanduser()
        if not path.is_file():
            violations.append(f"runner_artifact_missing:{path.name}")
        elif sha256_path(path) != str(item.get("sha256", "")):
            violations.append(f"runner_artifact_sha256_mismatch:{path.name}")


def audit_sil_run(manifest_path: Path) -> dict[str, Any]:
    manifest_path = manifest_path.expanduser().resolve()
    manifest = _read_json(manifest_path)
    violations: list[str] = []
    if manifest.get("schema_version") != 1:
        violations.append("orchestration_schema_invalid")
    if manifest.get("evidence_type") != "betaflight_gazebo_sil_run":
        violations.append("orchestration_type_invalid")
    if manifest.get("scope") != SITL_SCOPE:
        violations.append("orchestration_scope_invalid")
    if manifest.get("completed") is not True:
        violations.append("orchestration_incomplete")
    expected_sequence = [
        "configure_betaflight",
        "start_betaflight",
        "start_gazebo",
        "start_runner",
        "unpause_gazebo_after_runner_ready",
    ]
    if manifest.get("startup_sequence") != expected_sequence:
        violations.append("startup_sequence_invalid")
    cleanup = manifest.get("cleanup", {})
    if not isinstance(cleanup, dict) or cleanup.get("ports_released") is not True:
        violations.append("cleanup_not_verified")

    policy = str(manifest.get("policy", ""))
    if policy not in {"noncollision", "contact"}:
        violations.append("policy_invalid")
    detector_mode = str(manifest.get("detector_mode", ""))
    if detector_mode not in {"projected", "rendered"}:
        violations.append("detector_mode_invalid")

    artifacts = manifest.get("artifacts", {})
    if not isinstance(artifacts, dict):
        artifacts = {}
        violations.append("artifacts_invalid")
    paths = {
        name: _artifact_path(artifacts, name, violations)
        for name in REQUIRED_ARTIFACTS
    }

    configuration_manifest: dict[str, Any] = {}
    config: dict[str, Any] = {}
    runner_meta: dict[str, Any] = {}
    rows: list[dict[str, str]] = []
    if paths["configuration_manifest"] is not None:
        configuration_manifest = _read_json(paths["configuration_manifest"])
        if configuration_manifest.get("official_source_commit") != OFFICIAL_BETAFLIGHT_COMMIT:
            violations.append("official_betaflight_commit_mismatch")
        official_binary = configuration_manifest.get("official_binary", {})
        if not isinstance(official_binary, dict) or official_binary.get("sha256") != OFFICIAL_BETAFLIGHT_ELF_SHA256:
            violations.append("official_betaflight_elf_mismatch")
        for name, key in (("betaflight_cli", "cli"), ("eeprom", "eeprom")):
            internal = configuration_manifest.get(key, {})
            artifact = artifacts.get(name, {})
            if not isinstance(internal, dict) or internal.get("sha256") != artifact.get("sha256"):
                violations.append(f"configuration_binding_mismatch:{name}")
    if paths["sitl_config"] is not None:
        config = _read_json(paths["sitl_config"])
        profile = config.get("sitl_profile", {})
        generated = profile.get("generated_from", {}) if isinstance(profile, dict) else {}
        if (
            not isinstance(profile, dict)
            or profile.get("scope") != SITL_SCOPE
            or profile.get("loopback_only") is not True
            or not isinstance(generated, dict)
            or generated.get("engagement_policy") != policy
            or generated.get("sha256") != artifacts.get("flight_config", {}).get("sha256")
        ):
            violations.append("sitl_config_binding_invalid")
        gyro_binding = profile.get("raw_imu_axis_binding", {})
        raw_imu = dict(config.get("msp_runtime", {})).get("raw_imu_gyro", {})
        if (
            not isinstance(gyro_binding, dict)
            or gyro_binding.get("axis_sign") != [1, -1, 1]
            or not isinstance(raw_imu, dict)
            or raw_imu.get("axis_sign") != [1, -1, 1]
        ):
            violations.append("sitl_raw_imu_binding_invalid")
    if paths["runner_meta"] is not None:
        runner_meta = _read_json(paths["runner_meta"])
        if runner_meta.get("config_sha256") != artifacts.get("sitl_config", {}).get("sha256"):
            violations.append("runner_config_binding_mismatch")
        completion = runner_meta.get("completion", {})
        if not isinstance(completion, dict) or completion.get("complete") is not True:
            violations.append("runner_meta_incomplete")
    if paths["runner_manifest"] is not None:
        _verify_runner_manifest(_read_json(paths["runner_manifest"]), violations)
    if paths["runner_csv"] is not None:
        with paths["runner_csv"].open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
    if len(rows) < 100:
        violations.append("runner_csv_too_short")

    arm_count = _transition_count(rows, "armed")
    takeover_count = _transition_count(rows, "msp_override_active")
    algorithm_segments = _contiguous_segments(rows, "msp_publish_mode", "algorithm")
    if arm_count != 1:
        violations.append(f"arm_count:{arm_count}")
    if takeover_count != 1:
        violations.append(f"takeover_count:{takeover_count}")
    if len(algorithm_segments) != 1:
        violations.append(f"algorithm_segment_count:{len(algorithm_segments)}")

    algorithm_duration_s = math.nan
    active_rows = (
        [
            row
            for row in algorithm_segments[0]
            if row.get("rc_active") == "1"
            and row.get("msp_last_publish_command_active") == "1"
        ]
        if len(algorithm_segments) == 1
        else []
    )
    if active_rows:
        periods = [_float(row, "loop_period_s") for row in active_rows]
        valid_periods = [
            value for value in periods if math.isfinite(value) and value > 0.0
        ]
        period_s = statistics.median(valid_periods) if valid_periods else 0.0
        algorithm_duration_s = (
            _float(active_rows[-1], "elapsed_s")
            - _float(active_rows[0], "elapsed_s")
            + period_s
        )
    if not math.isfinite(algorithm_duration_s) or not 0.0 < algorithm_duration_s <= 0.9:
        violations.append("algorithm_duration_invalid")

    final_row = rows[-1] if rows else {}
    timing, timing_violations = evaluate_msp_timing(final_row)
    violations.extend(timing_violations)
    error_fields = (
        "msp_set_raw_rc_write_error_count",
        "msp_request_error_count",
        "msp_rx_checksum_error_count",
        "msp_rx_parser_error_count",
        "msp_worker_poll_error_count",
        "msp_worker_send_error_count",
        "msp_consecutive_send_error_count",
    )
    nonzero_errors = {field: _int(final_row, field) for field in error_fields if _int(final_row, field) != 0}
    if nonzero_errors:
        violations.append("msp_errors_nonzero")
    if any(row.get("msp_worker_error", "").strip() for row in rows):
        violations.append("msp_worker_error_reported")

    guidance_config = dict(
        dict(config.get("guidance", {})).get("velocity_establishing_png", {})
    )
    result_age_limit_s = float(
        guidance_config.get("detection_result_age_limit_s", 0.2)
    )
    update_age_limit_s = float(guidance_config.get("detection_timeout_s", 0.15))
    invalid_active_rows = 0
    nonfinite_active_fields: set[str] = set()
    required_active_fields = (
        "sp_roll_rate_deg_s",
        "sp_pitch_rate_deg_s",
        "map_requested_throttle_us",
        "rc_sent_ch1",
        "rc_sent_ch2",
        "rc_sent_ch3",
        "gyro_roll_deg_s",
        "gyro_pitch_deg_s",
    )
    for row in active_rows:
        detection_age = _float(row, "intercept_detection_age_s")
        update_age = _float(row, "intercept_detection_update_age_s")
        result_age_at_delivery = detection_age - update_age
        if (
            row.get("sp_valid") != "1"
            or row.get("rc_active") != "1"
            or not math.isfinite(detection_age)
            or not math.isfinite(update_age)
            or update_age > update_age_limit_s
            or result_age_at_delivery < -1.0e-6
            or result_age_at_delivery > result_age_limit_s
        ):
            invalid_active_rows += 1
        for field in required_active_fields:
            if not math.isfinite(_float(row, field)):
                nonfinite_active_fields.add(field)
    if invalid_active_rows:
        violations.append(f"stale_or_invalid_active_rows:{invalid_active_rows}")
    if nonfinite_active_fields:
        violations.append("nonfinite_active_commands:" + ",".join(sorted(nonfinite_active_fields)))

    gps_active_valid = sum(
        1
        for row in active_rows
        if _int(row, "gps_fix") >= 1
        and _int(row, "gps_satellites") >= 6
        and row.get("kinematics_origin_locked") == "1"
        and row.get("kinematics_valid") == "1"
    )
    if active_rows and gps_active_valid != len(active_rows):
        violations.append("active_gps_or_kinematics_invalid")

    accel_tilt_rate = dict(config.get("guidance_command", {})).get(
        "accel_tilt_rate", {}
    )
    command_signs = {
        "roll": float(accel_tilt_rate.get("roll_rate_sign", math.nan)),
        "pitch": float(accel_tilt_rate.get("pitch_rate_sign", math.nan)),
    }
    command_response = {
        axis: _best_delayed_correlation(
            rows,
            command_field=f"sp_{axis}_rate_deg_s",
            response_field=f"gyro_{axis}_deg_s",
            command_scale=command_signs[axis],
        )
        for axis in ("roll", "pitch")
    }
    motor_direction = {
        axis: _motor_direction_metrics(rows, axis) for axis in ("roll", "pitch")
    }
    for axis, metric in command_response.items():
        if (
            int(metric["sample_count"]) < 10
            or not math.isfinite(float(metric["correlation"]))
            or float(metric["correlation"]) < 0.25
        ):
            violations.append(f"command_to_gyro_direction_invalid:{axis}")
    for axis, metric in motor_direction.items():
        if (
            int(metric["sample_count"]) < 10
            or not math.isfinite(float(metric["correlation"]))
            or float(metric["correlation"]) < 0.25
        ):
            violations.append(f"motor_differential_direction_invalid:{axis}")

    ned_truth = _ned_truth_metrics(rows)
    if int(ned_truth["horizontal_axes_with_motion"]) < 1:
        violations.append("ned_truth_motion_missing")
    for axis in ("n", "e"):
        metric = ned_truth[axis]
        if int(metric["sample_count"]) >= 5 and float(metric["sign_match_fraction"]) < 0.8:
            violations.append(f"ned_truth_sign_invalid:{axis}")

    phases = {row.get("intercept_phase", "") for row in rows}
    terminal_triggers = {
        row.get("intercept_terminal_trigger", "")
        for row in rows
        if row.get("intercept_terminal_trigger", "")
    }
    if policy == "noncollision":
        terminal_passed = "ABORT" in phases and any(
            trigger.startswith("noncollision_") for trigger in terminal_triggers
        )
    else:
        terminal_passed = {
            "TERMINAL_VISUAL",
            "COMPLETE",
        }.issubset(phases) and any(
            trigger.startswith("contact_") for trigger in terminal_triggers
        )
    if not terminal_passed:
        violations.append(f"terminal_policy_not_exercised:{policy}")

    rendered_detection_count = sum(
        1 for row in rows if row.get("bbox_x1", "") and row.get("detector_source", "")
    )
    rendered_detector_exercised = detector_mode == "rendered" and rendered_detection_count > 0
    if detector_mode == "rendered" and not rendered_detector_exercised:
        violations.append("rendered_detector_produced_no_detections")

    software_binding = manifest.get("software_binding", {})
    if not isinstance(software_binding, dict):
        software_binding = {}
        violations.append("software_binding_invalid")
    flight_binding = {
        "path": str(paths["flight_config"] or ""),
        "sha256": str(artifacts.get("flight_config", {}).get("sha256", "")),
        "engagement_policy": policy,
    }
    report = {
        "schema_version": AUDIT_SCHEMA_VERSION,
        "evidence_type": EVIDENCE_TYPE,
        "scope": SITL_SCOPE,
        "created_unix_s": time.time(),
        "passed": not violations,
        "violations": violations,
        "policy": policy,
        "detector_mode": detector_mode,
        "detector_representative": False,
        "detector_representative_reason": (
            "Gazebo uses a deterministic synthetic detector stimulus; rendered SIL validates "
            "the pixel-processing integration path, not real-world detector performance."
        ),
        "hardware_authorization": False,
        "hardware_authorization_reason": (
            "SIL validates software integration and direction/timing contracts only; "
            "it cannot replace no-prop hardware timing, a fresh FC snapshot, or policy-specific approval."
        ),
        "orchestration_manifest": {
            "path": str(manifest_path),
            "sha256": sha256_path(manifest_path),
        },
        "software_binding": software_binding,
        "flight_candidate_binding": flight_binding,
        "betaflight_binding": {
            "source_commit": configuration_manifest.get("official_source_commit", ""),
            "elf_sha256": str(
                dict(configuration_manifest.get("official_binary", {})).get("sha256", "")
            ),
            "cli_sha256": str(artifacts.get("betaflight_cli", {}).get("sha256", "")),
            "eeprom_sha256": str(artifacts.get("eeprom", {}).get("sha256", "")),
        },
        "gazebo_binding": {
            name: str(artifacts.get(name, {}).get("sha256", ""))
            for name in (
                "gazebo_world",
                "gazebo_bridge_source",
                "gazebo_bridge_library",
                "interceptor_model",
                "target_model",
            )
        },
        "metrics": {
            "row_count": len(rows),
            "arm_count": arm_count,
            "takeover_count": takeover_count,
            "algorithm_segment_count": len(algorithm_segments),
            "algorithm_duration_s": algorithm_duration_s,
            "active_row_count": len(active_rows),
            "active_gps_kinematics_valid_rows": gps_active_valid,
            "invalid_active_row_count": invalid_active_rows,
            "rendered_detection_count": rendered_detection_count,
            "msp_timing": timing,
            "msp_nonzero_errors": nonzero_errors,
            "command_to_gyro": command_response,
            "motor_differential": motor_direction,
            "ned_truth": ned_truth,
            "terminal": {
                "passed": terminal_passed,
                "phases": sorted(phases),
                "triggers": sorted(terminal_triggers),
            },
        },
    }
    return _json_safe(report)


def main() -> None:
    args = parse_args()
    manifest_path = Path(args.orchestration_manifest)
    report = audit_sil_run(manifest_path)
    output_path = (
        Path(args.output).expanduser()
        if args.output
        else manifest_path.with_name("betaflight_gazebo_sil_audit.json")
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output_path)
    print(f"passed={int(report['passed'])}")
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
