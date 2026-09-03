#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from create_betaflight_noprop_approval import (  # noqa: E402
    _configured_override_channels_mask,
    _configured_override_mode_cli_id,
    _read_json,
    _sha256,
    _validate_camera_extrinsic,
    _validate_guidance_command_frames,
    _validate_guidance_config,
    _validate_rate_profile,
    _validate_raw_imu_gyro_binding,
    _validate_snapshot,
)


SCOPE = "prop_rig_active"
MAX_RATE_DEG_S = 3.0
MAX_ACCEL_MPS2 = 1.0
MAX_TAKEOVER_S = 100.0
MIN_REARM_RELEASE_S = 0.5
MAX_THROTTLE_COMMAND_US = 1500
MAX_MOTOR_OUTPUT_US = 1500
MAX_MOTOR_SPREAD_US = 250
MAX_MOTOR_SPREAD_GRACE_S = 1.0
MAX_MOTOR_TELEMETRY_AGE_S = 0.25
MIN_VBAT_V = 20.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a hash-bound approval for one guarded propeller-rig response test."
    )
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--acknowledge-guarded-prop-rig", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.acknowledge_guarded_prop_rig:
        raise RuntimeError("--acknowledge-guarded-prop-rig is required")

    snapshot_path = Path(args.snapshot).expanduser().resolve()
    config_path = Path(args.config).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    snapshot = _read_json(snapshot_path)
    config = _read_json(config_path)
    mode_id = _configured_override_mode_cli_id(config)
    mask = _configured_override_channels_mask(config)
    parsed_cli = _validate_snapshot(
        snapshot,
        snapshot_path,
        expected_override_mode_cli_id=mode_id,
        expected_override_channels_mask=mask,
    )
    evidence = validate_prop_rig_config(
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
            "rated anchored two-axis rig, mechanical stops, guarding, independent "
            "emergency stop, and personnel clear of propeller planes"
        ),
        "source_conflicts_resolved": True,
        "snapshot_manifest": str(snapshot_path),
        "snapshot_sha256": _sha256(snapshot_path),
        "expected_fc_identity": dict(snapshot["fc_identity"]),
        "parameters_path": str(config_path),
        "parameters_sha256": _sha256(config_path),
        "limits": {
            "override_channels_mask": mask,
            "override_mode_cli_id": mode_id,
            "max_rate_deg_s": MAX_RATE_DEG_S,
            "max_guidance_accel_mps2": MAX_ACCEL_MPS2,
            "max_takeover_duration_s": MAX_TAKEOVER_S,
            "minimum_rearm_release_s": MIN_REARM_RELEASE_S,
            "max_motor_output_us": MAX_MOTOR_OUTPUT_US,
            "max_throttle_command_us": MAX_THROTTLE_COMMAND_US,
            "max_motor_spread_us": MAX_MOTOR_SPREAD_US,
            "max_motor_spread_grace_s": MAX_MOTOR_SPREAD_GRACE_S,
            "max_motor_telemetry_age_s": MAX_MOTOR_TELEMETRY_AGE_S,
            "minimum_vbat_v": MIN_VBAT_V,
            "acro_rate_mode_required": True,
        },
        **evidence,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(approval, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output_path)
    print(f"approval_sha256={_sha256(output_path)}")


def validate_prop_rig_config(
    config: dict[str, Any],
    *,
    output_path: Path,
    parsed_cli: dict[str, Any],
    fc_identity: dict[str, Any],
) -> dict[str, Any]:
    candidate = dict(config.get("candidate_profile", {}))
    profile = dict(config.get("bench_profile", {}))
    runtime_policy = dict(config.get("runtime_policy", {}))
    authorization = dict(config.get("control_authorization", {}))
    runtime = dict(config.get("msp_runtime", {}))
    safety = dict(config.get("safety", {}))
    rc = dict(config.get("rc_mapping", {}))

    if candidate.get("scope") != SCOPE or profile.get("scope") != SCOPE:
        raise RuntimeError(f"candidate_profile and bench_profile must use scope {SCOPE}")
    required_true = (
        "propellers_installed",
        "acro_rate_mode_required",
        "rated_anchored_two_axis_rig_required",
        "independent_physical_emergency_stop_required",
        "personnel_clear_of_propeller_planes_required",
    )
    if any(profile.get(key) is not True for key in required_true):
        raise RuntimeError("prop-rig physical constraints must all be explicit")
    if int(profile.get("override_channels_mask", -1)) != 15:
        raise RuntimeError("bench_profile.override_channels_mask must be 15")
    if int(profile.get("max_throttle_command_us", 9999)) != MAX_THROTTLE_COMMAND_US:
        raise RuntimeError("bench_profile max throttle command must be 1500 us")
    if runtime_policy.get("required_authorization_scope") != SCOPE:
        raise RuntimeError("runtime policy must require prop_rig_active")
    if runtime_policy.get("allow_control_flag_permitted") is not True:
        raise RuntimeError("runtime policy does not permit the explicit control flag")
    if runtime_policy.get("msp_set_raw_rc_permitted") is not True:
        raise RuntimeError("runtime policy does not permit MSP_SET_RAW_RC")
    if authorization.get("enabled") is not True or authorization.get("required_scope") != SCOPE:
        raise RuntimeError("control_authorization must require prop_rig_active")
    configured_output = Path(str(authorization.get("approval_manifest", ""))).expanduser().resolve()
    if configured_output != output_path:
        raise RuntimeError("approval output must match control_authorization.approval_manifest")

    if runtime.get("io_worker_enabled") is not True or runtime.get("prefill_enabled") is not True:
        raise RuntimeError("MSP worker and prefill must be enabled")
    if runtime.get("transport_mode") != "async_pipeline":
        raise RuntimeError("prop-rig control requires the asynchronous MSP pipeline")
    if _configured_override_channels_mask(config) != 15:
        raise RuntimeError("prop-rig control must use the reviewed four-axis mask 15")
    if runtime.get("set_raw_rc_channel_map") != "AETR1234":
        raise RuntimeError("SET_RAW_RC wire order must be AETR1234")
    if int(runtime.get("aux_arm_channel_zero_based", -1)) != 4:
        raise RuntimeError("ARM must remain on physical RC5/AUX1")
    if int(runtime.get("throttle_channel_zero_based", -1)) != 2:
        raise RuntimeError("Throttle must remain AETR wire channel index 2")
    if int(runtime.get("prefill_min_frames", 0)) < 10:
        raise RuntimeError("at least 10 physical-RC prefill frames are required")
    if float(runtime.get("response_stale_s", math.inf)) > 0.25:
        raise RuntimeError("MSP acknowledgement timeout must not exceed 0.25 s")
    if float(runtime.get("throttle_handover_s", 0.0)) < 0.8:
        raise RuntimeError("Throttle handover must last at least 0.8 s")
    if float(runtime.get("motor_poll_hz", 0.0)) < 10.0:
        raise RuntimeError("prop-rig motor telemetry must run at least 10 Hz")

    if safety.get("require_acro_rate_mode") is not True:
        raise RuntimeError("safety.require_acro_rate_mode=true is required")
    if float(safety.get("min_vbat_v", 0.0)) < MIN_VBAT_V:
        raise RuntimeError(f"safety.min_vbat_v must be at least {MIN_VBAT_V}")
    aux = dict(safety.get("aux_enable", {}))
    if int(aux.get("channel_index", -1)) != 7 or int(aux.get("min_us", 0)) != 1700:
        raise RuntimeError("the takeover gate must remain RC7/AUX3 high")
    if aux.get("satisfied_by_override_mode") is not True:
        raise RuntimeError("MSP OVERRIDE must satisfy the RC7 gate")

    motor = dict(safety.get("motor_output_interlock", {}))
    if motor.get("enabled") is not True or motor.get("latch_until_disarm") is not True:
        raise RuntimeError("a latched motor-output interlock is required")
    if int(motor.get("channel_count", 0)) != 4:
        raise RuntimeError("the motor interlock must monitor four motors")
    if int(motor.get("max_output_us", 9999)) > MAX_MOTOR_OUTPUT_US:
        raise RuntimeError("motor output limit exceeds 1500 us")
    if int(motor.get("max_spread_us", 9999)) > MAX_MOTOR_SPREAD_US:
        raise RuntimeError("motor spread limit exceeds 250 us")
    if not 0.0 <= float(motor.get("violation_grace_s", 9999.0)) <= MAX_MOTOR_SPREAD_GRACE_S:
        raise RuntimeError("motor spread grace must be in [0, 1] s")
    if not 0.0 < float(motor.get("telemetry_timeout_s", 9999.0)) <= MAX_MOTOR_TELEMETRY_AGE_S:
        raise RuntimeError("motor telemetry timeout must be in (0, 0.25] s")

    takeover = dict(safety.get("takeover_duration_interlock", {}))
    if takeover.get("enabled") is not True:
        raise RuntimeError("a takeover-duration interlock is required")
    if takeover.get("latch_until_disarm") is not False:
        raise RuntimeError("repeated prop-rig pulses must rearm on RC7 release")
    if not 0.0 < float(takeover.get("max_duration_s", 9999.0)) <= MAX_TAKEOVER_S:
        raise RuntimeError("takeover duration must be in (0, 100] s")
    if float(takeover.get("rearm_release_s", 0.0)) < MIN_REARM_RELEASE_S:
        raise RuntimeError("RC7 release rearm dwell must be at least 0.5 s")

    if rc.get("channel_map") != "AETR1234" or rc.get("rate_mapping_type") != "betaflight":
        raise RuntimeError("RC mapping must use AETR1234 Betaflight rate inversion")
    for key in ("roll_command_limit_deg_s", "pitch_command_limit_deg_s"):
        if not 0.0 < float(rc.get(key, math.inf)) <= MAX_RATE_DEG_S:
            raise RuntimeError(f"{key} must be in (0, 3] deg/s")
    if float(rc.get("yaw_command_limit_deg_s", math.inf)) != 0.0:
        raise RuntimeError("Yaw command limit must be zero")
    if int(rc.get("throttle_max_us", 9999)) != MAX_THROTTLE_COMMAND_US:
        raise RuntimeError("Throttle command limit must be 1500 us")
    if int(rc.get("throttle_hover_us", 0)) != 1275:
        raise RuntimeError("the measured hover reference must remain 1275 us")
    if float(rc.get("max_delta_us_per_s", math.inf)) > 100.0:
        raise RuntimeError("RC slew limit must not exceed 100 us/s")
    command = dict(config.get("guidance_command", {}))
    if not math.isclose(float(command.get("hover_thrust", math.nan)), 0.5):
        raise RuntimeError("prop-rig logical Throttle must remain at the 1275 us hover reference")
    _validate_rate_profile(rc, parsed_cli)

    guidance = _validate_guidance_config(config)
    if guidance.get("velocity_source") != "bench_zero_velocity":
        raise RuntimeError("this fixed rig requires bench_zero_velocity")
    if float(guidance.get("max_guidance_accel_mps2", math.inf)) > MAX_ACCEL_MPS2:
        raise RuntimeError("guidance acceleration exceeds 1 m/s2")
    frames = _validate_guidance_command_frames(config)
    camera = _validate_camera_extrinsic(config)
    gyro = _validate_raw_imu_gyro_binding(runtime, fc_identity=fc_identity)
    return {
        "camera_extrinsic": camera,
        "guidance": guidance,
        "guidance_command_frames": frames,
        "msp_raw_imu_gyro": gyro,
    }


if __name__ == "__main__":
    main()
