#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from create_betaflight_noprop_approval import (  # noqa: E402
    _configured_override_mode_cli_id,
    _read_json,
    _sha256,
    _validate_camera_extrinsic,
    _validate_guidance_command_frames,
    _validate_rate_profile,
    _validate_raw_imu_gyro_binding,
    _validate_snapshot,
)
from vision_guidance.betaflight_intercept_controller import (  # noqa: E402
    VelocityEstablishingPngConfig,
)
from vision_guidance.betaflight_runtime import MspRuntimeConfig  # noqa: E402
from vision_guidance.flight_control import (  # noqa: E402
    AccelerationTiltRateConfig,
    RcMappingConfig,
)


SCOPE = "flight_active_supervised"
OVERRIDE_CHANNELS_MASK = 15
MAX_RATE_DEG_S = 60.0
MAX_TILT_DEG = 35.0
MAX_GUIDANCE_ACCEL_MPS2 = 7.0
THROTTLE_MIN_US = 1200
THROTTLE_HOVER_US = 1275
THROTTLE_MAX_US = 1500
THROTTLE_SLEW_US_PER_S = 600.0
MIN_VBAT_V = 20.0
MIN_GPS_SATELLITES = 6
THRUST_CALIBRATION_ID = "LOG00062_1275_1500"
RELEASE_HIT_RATE_MIN = 0.80
RELEASE_FOV_HIT_RATE_MIN = 0.80
RELEASE_TRIALS_PER_CASE_MIN = 100
RELEASE_CASE_COUNT_MIN = 30
RELEASE_ROW_COUNT_MIN = 18000
RELEASE_REQUIRED_SCENARIOS = {
    "final_chain_software_p95",
    "observed_active_flight_p95",
    "conservative_physical_p95_budget",
}
EXPECTED_POLL_HZ = {
    "status_poll_hz": 5.0,
    "attitude_poll_hz": 20.0,
    "raw_imu_poll_hz": 5.0,
    "raw_gps_poll_hz": 5.0,
    "altitude_poll_hz": 5.0,
    "motor_poll_hz": 0.0,
    "rc_poll_hz": 5.0,
    "analog_poll_hz": 1.0,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a hash-bound approval for supervised velocity-PNG flight."
    )
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--release-evidence", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--acknowledge-supervised-flight", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.acknowledge_supervised_flight:
        raise RuntimeError("--acknowledge-supervised-flight is required")

    snapshot_path = Path(args.snapshot).expanduser().resolve()
    config_path = Path(args.config).expanduser().resolve()
    release_evidence_path = Path(args.release_evidence).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    snapshot = _read_json(snapshot_path)
    config = _read_json(config_path)
    config_sha256 = _sha256(config_path)
    release_evidence = validate_release_evidence(
        _read_json(release_evidence_path),
        release_evidence_path,
        runtime_config_sha256=config_sha256,
    )
    parsed_cli = _validate_snapshot(
        snapshot,
        snapshot_path,
        expected_override_mode_cli_id=_configured_override_mode_cli_id(config),
        expected_override_channels_mask=OVERRIDE_CHANNELS_MASK,
    )
    gps_evidence = validate_snapshot_flight_state(snapshot, snapshot_path)
    evidence = validate_flight_supervised_config(
        config,
        output_path=output_path,
        parsed_cli=parsed_cli,
        fc_identity=dict(snapshot["fc_identity"]),
    )
    approval = {
        "schema_version": 1,
        "approved": True,
        "scope": SCOPE,
        "created_unix_s": time.time(),
        "created_local": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "operator": str(args.operator),
        "operator_acknowledgement": (
            "professional pilot, supervised non-collision flight, "
            "immediate RC7 release on anomaly"
        ),
        "source_conflicts_resolved": True,
        "snapshot_manifest": str(snapshot_path),
        "snapshot_sha256": _sha256(snapshot_path),
        "expected_fc_identity": dict(snapshot["fc_identity"]),
        "parameters_path": str(config_path),
        "parameters_sha256": config_sha256,
        "release_evidence": release_evidence,
        "limits": {
            "override_channels_mask": OVERRIDE_CHANNELS_MASK,
            "actual_algorithm_publication_limit_s": None,
            "duration_interlock_enabled": False,
            "roll_pitch_rate_deg_s": MAX_RATE_DEG_S,
            "tilt_envelope_deg": MAX_TILT_DEG,
            "total_guidance_accel_mps2": MAX_GUIDANCE_ACCEL_MPS2,
            "throttle_us": [THROTTLE_MIN_US, THROTTLE_HOVER_US, THROTTLE_MAX_US],
            "throttle_slew_us_per_s": THROTTLE_SLEW_US_PER_S,
            "minimum_vbat_v": MIN_VBAT_V,
            "minimum_gps_satellites": MIN_GPS_SATELLITES,
            "acro_rate_mode_required": True,
        },
        "snapshot_flight_state": gps_evidence,
        **evidence,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(approval, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output_path)
    print(f"approval_sha256={_sha256(output_path)}")


def validate_release_evidence(
    report: dict[str, Any],
    report_path: Path,
    *,
    runtime_config_sha256: str,
) -> dict[str, Any]:
    if report.get("schema_version") != 2:
        raise RuntimeError("release evidence schema_version must be 2")
    if report.get("purpose") != "stochastic interception release evaluation":
        raise RuntimeError("release evidence purpose mismatch")
    if report.get("release_passed") is not True:
        raise RuntimeError("release evidence did not pass")

    runtime_binding = _release_mapping(report.get("runtime_binding"), "runtime_binding")
    if runtime_binding.get("sha256") != runtime_config_sha256:
        raise RuntimeError("release evidence runtime config SHA256 mismatch")
    acceptance = _release_mapping(report.get("acceptance"), "acceptance")
    try:
        hit_rate_min = float(acceptance["initially_visible_hit_rate_min"])
        fov_hit_rate_min = float(acceptance["initially_visible_fov_hit_rate_min"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("release evidence acceptance is invalid") from exc
    if (
        hit_rate_min != RELEASE_HIT_RATE_MIN
        or fov_hit_rate_min != RELEASE_FOV_HIT_RATE_MIN
        or acceptance.get("worst_minimum_range_m_max", "missing") is not None
    ):
        raise RuntimeError("release evidence must use the formal 80% probabilistic policy")
    paired = _release_mapping(report.get("paired_screening"), "paired_screening")
    if paired.get("passed") is not True:
        raise RuntimeError("release evidence paired screening did not pass")
    try:
        trials_per_case = int(report["trials_per_case"])
        case_count = int(report["case_count"])
        row_count = int(report["row_count"])
        required_summary_count = int(report["required_summary_count"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("release evidence Monte Carlo coverage is invalid") from exc
    if (
        trials_per_case < RELEASE_TRIALS_PER_CASE_MIN
        or case_count < RELEASE_CASE_COUNT_MIN
        or row_count < RELEASE_ROW_COUNT_MIN
    ):
        raise RuntimeError("release evidence Monte Carlo coverage is insufficient")

    raw_summaries = report.get("summaries")
    if not isinstance(raw_summaries, list) or any(
        not isinstance(summary, dict) for summary in raw_summaries
    ):
        raise RuntimeError("release evidence summaries must be a list of objects")
    summaries = [
        summary
        for summary in raw_summaries
        if summary.get("required_for_release") is True
    ]
    scenario_names = {summary.get("scenario_name") for summary in summaries}
    selected_evaluation = paired.get("selected_evaluation")
    if (
        len(summaries) != len(RELEASE_REQUIRED_SCENARIOS)
        or len(summaries) != required_summary_count
        or scenario_names != RELEASE_REQUIRED_SCENARIOS
        or not isinstance(selected_evaluation, str)
        or not selected_evaluation
        or any(
            summary.get("evaluation_name") != selected_evaluation
            for summary in summaries
        )
    ):
        raise RuntimeError("release evidence required scenario coverage is incomplete")
    for summary in summaries:
        if summary.get("passed") is not True:
            raise RuntimeError("release evidence contains a failed required scenario")
        try:
            hit_rate = float(summary["initially_visible_hit_rate"])
            fov_hit_rate = float(summary["initially_visible_fov_hit_rate"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("release evidence scenario rates are invalid") from exc
        if hit_rate < RELEASE_HIT_RATE_MIN or fov_hit_rate < RELEASE_FOV_HIT_RATE_MIN:
            raise RuntimeError("release evidence scenario is below the formal 80% policy")
        checks = _release_mapping(summary.get("checks"), "summary checks")
        range_check = _release_mapping(
            checks.get("worst_minimum_range_m"), "worst range check"
        )
        if (
            range_check.get("operator") != "report_only"
            or range_check.get("threshold") is not None
            or range_check.get("required") is not False
        ):
            raise RuntimeError("release evidence worst range must remain report-only")
    return {
        "path": str(report_path),
        "sha256": _sha256(report_path),
        "schema_version": 2,
        "runtime_config_sha256": runtime_config_sha256,
        "formal_hit_rate_min": RELEASE_HIT_RATE_MIN,
        "formal_fov_hit_rate_min": RELEASE_FOV_HIT_RATE_MIN,
        "trials_per_case": trials_per_case,
        "case_count": case_count,
        "row_count": row_count,
        "required_scenario_count": len(summaries),
    }


def _release_mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise RuntimeError(f"release evidence {name} must be an object")
    return value


def validate_snapshot_flight_state(
    snapshot: dict[str, Any],
    snapshot_path: Path,
) -> dict[str, Any]:
    capture = dict(snapshot.get("capture", {}))
    if capture.get("include_kinematics") is not True:
        raise RuntimeError("fresh flight snapshot requires --include-kinematics")
    telemetry_name = "telemetry.csv"
    telemetry_path = snapshot_path.parent / telemetry_name
    expected_hash = str(snapshot.get("artifacts", {}).get(telemetry_name, ""))
    if not telemetry_path.is_file() or not expected_hash or _sha256(telemetry_path) != expected_hash:
        raise RuntimeError("snapshot telemetry.csv is missing or changed")

    valid_rows: list[tuple[int, int, float]] = []
    with telemetry_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            try:
                fix = int(row.get("gps_fix", ""))
                satellites = int(row.get("gps_satellites", ""))
                voltage = float(row.get("vbat_v", ""))
            except (TypeError, ValueError):
                continue
            if fix >= 1 and satellites >= MIN_GPS_SATELLITES and voltage >= MIN_VBAT_V:
                valid_rows.append((fix, satellites, voltage))
    if len(valid_rows) < 3:
        raise RuntimeError(
            "snapshot needs at least three samples with GPS >=6 satellites and VBAT >=20 V"
        )
    return {
        "valid_sample_count": len(valid_rows),
        "minimum_fix": min(value[0] for value in valid_rows),
        "minimum_satellites": min(value[1] for value in valid_rows),
        "minimum_vbat_v": min(value[2] for value in valid_rows),
        "telemetry_sha256": expected_hash,
    }


def validate_flight_supervised_config(
    config: dict[str, Any],
    *,
    output_path: Path,
    parsed_cli: dict[str, Any],
    fc_identity: dict[str, Any],
) -> dict[str, Any]:
    candidate = dict(config.get("candidate_profile", {}))
    profile = dict(config.get("flight_profile", {}))
    policy = dict(config.get("runtime_policy", {}))
    authorization = dict(config.get("control_authorization", {}))
    safety = dict(config.get("safety", {}))
    logging = dict(config.get("logging", {}))

    if candidate.get("scope") != SCOPE or profile.get("scope") != SCOPE:
        raise RuntimeError(f"candidate_profile and flight_profile must use scope {SCOPE}")
    for key in ("propellers_installed", "professional_pilot_required", "acro_rate_mode_required"):
        if profile.get(key) is not True:
            raise RuntimeError(f"flight_profile.{key}=true is required")
    if profile.get("controlled_axes") != ["roll", "pitch", "throttle", "yaw"]:
        raise RuntimeError("supervised profile must control Roll/Pitch/Throttle/Yaw")
    if profile.get("yaw_policy") != "neutral_1500_us":
        raise RuntimeError("supervised profile must hold Yaw neutral")
    if int(profile.get("override_channels_mask", -1)) != OVERRIDE_CHANNELS_MASK:
        raise RuntimeError("supervised profile must use mask 15")
    if (
        profile.get("max_takeover_duration_s") is not None
        or profile.get("takeover_time_basis") != "unbounded_while_safety_gates_healthy"
        or float(profile.get("rc7_release_rearm_s", math.nan)) != 0.0
    ):
        raise RuntimeError("supervised profile must declare an unbounded takeover duration")

    if policy.get("required_authorization_scope") != SCOPE:
        raise RuntimeError("runtime policy scope mismatch")
    if policy.get("allow_control_flag_permitted") is not True:
        raise RuntimeError("runtime policy must permit --allow-control")
    if policy.get("msp_set_raw_rc_permitted") is not True:
        raise RuntimeError("runtime policy must permit MSP_SET_RAW_RC")
    if authorization.get("enabled") is not True or authorization.get("required_scope") != SCOPE:
        raise RuntimeError("control_authorization scope mismatch")
    if authorization.get("release_evidence_required") is not True:
        raise RuntimeError("control_authorization must require release evidence")
    configured_output = Path(str(authorization.get("approval_manifest", ""))).expanduser().resolve()
    if configured_output != output_path:
        raise RuntimeError("approval output must match control_authorization.approval_manifest")

    try:
        runtime = MspRuntimeConfig.from_mapping(dict(config.get("msp_runtime", {})))
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid msp_runtime: {exc}") from exc
    if not runtime.io_worker_enabled or not runtime.prefill_enabled:
        raise RuntimeError("MSP worker and physical-RC prefill are required")
    if runtime.transport_mode != "async_pipeline":
        raise RuntimeError("asynchronous MSP pipeline is required")
    if runtime.override_channels_mask != OVERRIDE_CHANNELS_MASK:
        raise RuntimeError("msp_runtime.override_channels_mask must be mask 15")
    if runtime.set_raw_rc_channel_map != "AETR1234":
        raise RuntimeError("SET_RAW_RC wire order must be AETR1234")
    if runtime.aux_arm_channel_zero_based != 4 or runtime.throttle_channel_zero_based != 2:
        raise RuntimeError("ARM and Throttle channel indices do not match AETR1234")
    if runtime.control_publish_hz != 50.0:
        raise RuntimeError("control_publish_hz must be exactly 50 Hz")
    for key, expected in EXPECTED_POLL_HZ.items():
        if not math.isclose(float(getattr(runtime, key)), expected, abs_tol=1.0e-9):
            raise RuntimeError(f"msp_runtime.{key} must be exactly {expected:g} Hz")
    if runtime.prefill_min_frames < 10 or runtime.response_stale_s > 0.25:
        raise RuntimeError("prefill or MSP acknowledgement freshness is too weak")
    if runtime.throttle_relative_limit_us != 0:
        raise RuntimeError("supervised profile must disable relative throttle limiting")
    if (
        runtime.throttle_reference_min_us != THROTTLE_MIN_US
        or runtime.throttle_reference_max_us != 1400
        or runtime.throttle_command_min_us != THROTTLE_MIN_US
        or runtime.throttle_command_max_us != THROTTLE_MAX_US
        or not math.isclose(
            runtime.throttle_slew_limit_us_per_s,
            THROTTLE_SLEW_US_PER_S,
            abs_tol=1.0e-9,
        )
    ):
        raise RuntimeError("supervised throttle runtime envelope mismatch")

    takeover = dict(safety.get("takeover_duration_interlock", {}))
    if (
        takeover.get("enabled") is not False
        or takeover.get("latch_until_disarm") is not False
        or takeover.get("max_duration_s") is not None
        or float(takeover.get("rearm_release_s", math.nan)) != 0.0
    ):
        raise RuntimeError("takeover duration interlock must be explicitly disabled")
    if safety.get("require_acro_rate_mode") is not True:
        raise RuntimeError("Acro/Rate mode is required")
    if float(safety.get("min_vbat_v", 0.0)) < MIN_VBAT_V:
        raise RuntimeError("minimum battery gate must be at least 20 V")
    aux = dict(safety.get("aux_enable", {}))
    if (
        int(aux.get("channel_index", -1)) != 7
        or int(aux.get("min_us", 0)) != 1700
        or int(aux.get("max_us", 0)) != 2100
        or aux.get("satisfied_by_override_mode") is not True
    ):
        raise RuntimeError("takeover gate must be MSP OVERRIDE on RC7/AUX3 high")

    try:
        rc_values = dict(config.get("rc_mapping", {}))
        rc = RcMappingConfig(
            **{
                key: value
                for key, value in rc_values.items()
                if key in RcMappingConfig.__dataclass_fields__
            }
        )
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid rc_mapping: {exc}") from exc
    if rc.channel_map != "AETR1234" or rc.rate_mapping_type != "betaflight":
        raise RuntimeError("RC mapping must use AETR1234 Betaflight rate inversion")
    if tuple(rc.betaflight_rc_rate) != (1.0, 1.0, 1.0):
        raise RuntimeError("RC Rate must be 1.0 on all axes")
    if tuple(rc.betaflight_super_rate) != (0.7, 0.7, 0.7):
        raise RuntimeError("Super Rate must be 0.7 on all axes")
    if tuple(rc.betaflight_expo) != (0.0, 0.0, 0.0):
        raise RuntimeError("Expo must be zero on all axes")
    if (
        rc.roll_command_limit_deg_s != MAX_RATE_DEG_S
        or rc.pitch_command_limit_deg_s != MAX_RATE_DEG_S
        or rc.yaw_command_limit_deg_s != 0.0
    ):
        raise RuntimeError("Roll/Pitch/Yaw command limits must be 60/60/0 deg/s")
    if (
        rc.throttle_min_us != THROTTLE_MIN_US
        or rc.throttle_hover_us != THROTTLE_HOVER_US
        or rc.throttle_max_us != THROTTLE_MAX_US
        or rc.neutral_throttle_us != THROTTLE_MIN_US
        or rc.max_delta_us_per_s != THROTTLE_SLEW_US_PER_S
    ):
        raise RuntimeError("RC measured throttle mapping must be 1200/1275/1500 at 600 us/s")
    _validate_rate_profile(dict(config.get("rc_mapping", {})), parsed_cli)

    guidance = _validate_guidance(config)
    command = _validate_guidance_command(config)
    kinematics = dict(config.get("kinematics", {}))
    if int(kinematics.get("minimum_satellites", 0)) < MIN_GPS_SATELLITES:
        raise RuntimeError("at least six GPS satellites are required")
    if int(kinematics.get("origin_lock_samples", 0)) < 3:
        raise RuntimeError("at least three stable origin samples are required")
    if not math.isclose(float(logging.get("csv_flush_interval_s", math.nan)), 1.0):
        raise RuntimeError("logging.csv_flush_interval_s must be exactly 1 s")

    rknn = dict(config.get("rknn_detector", {}))
    torch_runtime = dict(config.get("torch_runtime", {}))
    if not str(rknn.get("model", "")).endswith(".rknn") or not str(
        rknn.get("library", "")
    ).endswith(".so"):
        raise RuntimeError("real RKNN model and native detector library are required")
    if torch_runtime.get("allow_cpu_inference") is not False:
        raise RuntimeError("CPU detector fallback must remain disabled")

    return {
        "camera_extrinsic": _validate_camera_extrinsic(config),
        "guidance": guidance,
        "guidance_command": command,
        "guidance_command_frames": _validate_guidance_command_frames(config),
        "msp_runtime": {
            "poll_total_hz": sum(EXPECTED_POLL_HZ.values()),
            "control_publish_hz": runtime.control_publish_hz,
            "throttle_slew_limit_us_per_s": runtime.throttle_slew_limit_us_per_s,
        },
        "msp_raw_imu_gyro": _validate_raw_imu_gyro_binding(
            dict(config.get("msp_runtime", {})),
            fc_identity=fc_identity,
        ),
    }


def _validate_guidance(config: dict[str, Any]) -> dict[str, Any]:
    guidance = dict(config.get("guidance", {}))
    if guidance.get("law") != "velocity_establishing_png":
        raise RuntimeError("guidance.law must be velocity_establishing_png")
    if guidance.get("velocity_source") != "msp_kinematics":
        raise RuntimeError("guidance.velocity_source must be msp_kinematics")
    if not math.isclose(
        float(guidance.get("max_guidance_accel_mps2", math.nan)),
        MAX_GUIDANCE_ACCEL_MPS2,
    ):
        raise RuntimeError("guidance.max_guidance_accel_mps2 must be exactly 7")
    raw = dict(guidance.get("velocity_establishing_png", {}))
    try:
        value = VelocityEstablishingPngConfig(**raw)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid velocity-establishing PNG config: {exc}") from exc
    if not math.isclose(value.fixed_vm_m_s, 10.0) or not math.isclose(
        value.navigation_constant, 3.0
    ):
        raise RuntimeError("supervised candidate requires Vm=10 m/s and N=3")
    for key in (
        "speed_accel_limit_m_s2",
        "png_accel_limit_m_s2",
        "fov_centering_accel_limit_m_s2",
        "total_accel_limit_m_s2",
    ):
        if float(getattr(value, key)) > MAX_GUIDANCE_ACCEL_MPS2:
            raise RuntimeError(f"{key} exceeds 7 m/s2")
    if not math.isclose(value.total_accel_limit_m_s2, MAX_GUIDANCE_ACCEL_MPS2):
        raise RuntimeError("total_accel_limit_m_s2 must be exactly 7 m/s2")
    return {"law": guidance["law"], "velocity_source": guidance["velocity_source"], **asdict(value)}


def _validate_guidance_command(config: dict[str, Any]) -> dict[str, Any]:
    command = dict(config.get("guidance_command", {}))
    if not math.isclose(float(command.get("hover_thrust", math.nan)), 0.5):
        raise RuntimeError("hover_thrust must map to measured 1275 us")
    try:
        accel = AccelerationTiltRateConfig.from_mapping(
            dict(command.get("accel_tilt_rate", {}))
        )
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid accel_tilt_rate config: {exc}") from exc
    if (
        accel.max_roll_tilt_deg != MAX_TILT_DEG
        or accel.max_pitch_tilt_deg != MAX_TILT_DEG
        or accel.max_roll_rate_deg_s != MAX_RATE_DEG_S
        or accel.max_pitch_rate_deg_s != MAX_RATE_DEG_S
        or accel.roll_rate_sign != 1.0
        or accel.pitch_rate_sign != -1.0
    ):
        raise RuntimeError("accel_tilt_rate limits/signs must be 35/35 deg, 60/60 deg/s, +1/-1")
    thrust = accel.thrust_feedforward
    if (
        not thrust.enabled
        or thrust.model != "measured_load_factor"
        or not math.isclose(thrust.hover_load_factor_g, 1.0)
        or not math.isclose(thrust.max_load_factor_g, 2.37)
        or not math.isclose(thrust.minimum_tilt_cosine, 0.5)
        or thrust.calibration_id != THRUST_CALIBRATION_ID
    ):
        raise RuntimeError("measured LOG00062 thrust feedforward binding mismatch")
    tilt = dict(command.get("tilt_envelope", {}))
    if (
        tilt.get("enabled") is not True
        or float(tilt.get("max_roll_angle_deg", math.nan)) != MAX_TILT_DEG
        or float(tilt.get("max_pitch_angle_deg", math.nan)) != MAX_TILT_DEG
        or float(tilt.get("hardcap_max_level_rate_deg_s", math.nan)) != MAX_RATE_DEG_S
    ):
        raise RuntimeError("tilt envelope must be enabled at 35 deg with 60 deg/s leveling")
    entry = dict(command.get("entry_handoff", {}))
    if (
        entry.get("enabled") is not True
        or entry.get("rate_source") != "gyro"
        or float(entry.get("duration_s", 0.0)) < 0.8
        or float(entry.get("gyro_max_age_s", math.inf)) > 0.25
    ):
        raise RuntimeError("gyro-based entry handoff must remain enabled and fresh")
    return {
        "mapping_type": command.get("mapping_type"),
        "hover_thrust": command.get("hover_thrust"),
        "accel_tilt_rate": asdict(accel),
    }


if __name__ == "__main__":
    main()
