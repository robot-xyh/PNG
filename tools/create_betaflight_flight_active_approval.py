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


SCOPE = "flight_active_1s"
OVERRIDE_CHANNELS_MASK = 15
MAX_TAKEOVER_S = 1.0
MAX_RATE_DEG_S = 3.0
MAX_GUIDANCE_ACCEL_MPS2 = 1.0
MAX_THROTTLE_RELATIVE_US = 40
MAX_THROTTLE_COMMAND_US = 1500
MIN_THROTTLE_HANDOVER_S = 0.8
MIN_VBAT_V = 20.0
MIN_GPS_SATELLITES = 6


def _configured_flight_override_channels_mask(config: dict[str, Any]) -> int:
    runtime = dict(config.get("msp_runtime", {}))
    mask = int(runtime.get("override_channels_mask", -1))
    if mask != OVERRIDE_CHANNELS_MASK:
        raise RuntimeError("msp_runtime.override_channels_mask must be mask 15")
    return mask


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a hash-bound approval for one supervised one-second PNG flight handover."
    )
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--acknowledge-supervised-free-flight", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.acknowledge_supervised_free_flight:
        raise RuntimeError("--acknowledge-supervised-free-flight is required")

    snapshot_path = Path(args.snapshot).expanduser().resolve()
    config_path = Path(args.config).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    snapshot = _read_json(snapshot_path)
    config = _read_json(config_path)
    parsed_cli = _validate_snapshot(
        snapshot,
        snapshot_path,
        expected_override_mode_cli_id=_configured_override_mode_cli_id(config),
        expected_override_channels_mask=_configured_flight_override_channels_mask(config),
    )
    evidence = validate_flight_active_config(
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
            "professional pilot, open flight area, immediate RC7 release, and one takeover "
            "per arming cycle"
        ),
        "source_conflicts_resolved": True,
        "snapshot_manifest": str(snapshot_path),
        "snapshot_sha256": _sha256(snapshot_path),
        "expected_fc_identity": dict(snapshot["fc_identity"]),
        "parameters_path": str(config_path),
        "parameters_sha256": _sha256(config_path),
        "limits": {
            "override_channels_mask": OVERRIDE_CHANNELS_MASK,
            "max_takeover_duration_s": MAX_TAKEOVER_S,
            "max_rate_deg_s": MAX_RATE_DEG_S,
            "max_guidance_accel_mps2": MAX_GUIDANCE_ACCEL_MPS2,
            "max_throttle_relative_us": MAX_THROTTLE_RELATIVE_US,
            "max_throttle_command_us": MAX_THROTTLE_COMMAND_US,
            "minimum_throttle_handover_s": MIN_THROTTLE_HANDOVER_S,
            "minimum_vbat_v": MIN_VBAT_V,
            "minimum_gps_satellites": MIN_GPS_SATELLITES,
            "acro_rate_mode_required": True,
        },
        **evidence,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(approval, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output_path)
    print(f"approval_sha256={_sha256(output_path)}")


def validate_flight_active_config(
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
    runtime = dict(config.get("msp_runtime", {}))
    safety = dict(config.get("safety", {}))
    rc = dict(config.get("rc_mapping", {}))

    if candidate.get("scope") != SCOPE or profile.get("scope") != SCOPE:
        raise RuntimeError(f"candidate_profile and flight_profile must use scope {SCOPE}")
    for key in ("propellers_installed", "professional_pilot_required", "acro_rate_mode_required"):
        if profile.get(key) is not True:
            raise RuntimeError(f"flight_profile.{key}=true is required")
    if profile.get("controlled_axes") != ["roll", "pitch", "throttle", "yaw"]:
        raise RuntimeError("Roll/Pitch/Throttle/Yaw must match mask 15")
    if profile.get("yaw_policy") != "neutral_1500_us":
        raise RuntimeError("Yaw must remain neutral during the one-second takeover")
    if int(profile.get("override_channels_mask", -1)) != OVERRIDE_CHANNELS_MASK:
        raise RuntimeError("flight profile must use four-channel mask 15")

    if policy.get("required_authorization_scope") != SCOPE:
        raise RuntimeError("runtime policy scope mismatch")
    if policy.get("allow_control_flag_permitted") is not True:
        raise RuntimeError("runtime policy must explicitly permit --allow-control")
    if policy.get("msp_set_raw_rc_permitted") is not True:
        raise RuntimeError("runtime policy must explicitly permit MSP_SET_RAW_RC")
    if authorization.get("enabled") is not True or authorization.get("required_scope") != SCOPE:
        raise RuntimeError("control_authorization scope mismatch")
    configured_output = Path(str(authorization.get("approval_manifest", ""))).expanduser().resolve()
    if configured_output != output_path:
        raise RuntimeError("approval output must match control_authorization.approval_manifest")

    if runtime.get("io_worker_enabled") is not True or runtime.get("prefill_enabled") is not True:
        raise RuntimeError("MSP worker and physical-RC prefill are required")
    if runtime.get("transport_mode") != "async_pipeline":
        raise RuntimeError("the asynchronous MSP pipeline is required")
    _configured_flight_override_channels_mask(config)
    if runtime.get("set_raw_rc_channel_map") != "AETR1234":
        raise RuntimeError("SET_RAW_RC wire order must be AETR1234")
    if int(runtime.get("aux_arm_channel_zero_based", -1)) != 4:
        raise RuntimeError("ARM must remain on physical RC5/AUX1")
    if int(runtime.get("throttle_channel_zero_based", -1)) != 2:
        raise RuntimeError("Throttle must remain AETR wire index 2")
    if int(runtime.get("prefill_min_frames", 0)) < 10:
        raise RuntimeError("at least 10 physical-RC prefill frames are required")
    if float(runtime.get("response_stale_s", math.inf)) > 0.25:
        raise RuntimeError("MSP acknowledgement timeout cannot exceed 0.25 s")
    if float(runtime.get("control_publish_hz", 0.0)) < 50.0:
        raise RuntimeError("control publish rate must be at least 50 Hz")
    if float(runtime.get("throttle_handover_s", 0.0)) < MIN_THROTTLE_HANDOVER_S:
        raise RuntimeError("throttle handover must last at least 0.8 s")
    if not 0 < int(runtime.get("throttle_relative_limit_us", 9999)) <= MAX_THROTTLE_RELATIVE_US:
        raise RuntimeError("relative throttle limit must be in (0, 40] us")
    if int(runtime.get("throttle_reference_min_us", 0)) < 1200:
        raise RuntimeError("throttle takeover reference must be at least 1200 us")
    if int(runtime.get("throttle_reference_max_us", 9999)) > 1400:
        raise RuntimeError("throttle takeover reference must not exceed 1400 us")
    if int(runtime.get("throttle_command_max_us", 9999)) > MAX_THROTTLE_COMMAND_US:
        raise RuntimeError("algorithm throttle ceiling exceeds 1500 us")

    if safety.get("require_acro_rate_mode") is not True:
        raise RuntimeError("Acro/Rate mode is required")
    if float(safety.get("min_vbat_v", 0.0)) < MIN_VBAT_V:
        raise RuntimeError("minimum battery gate must be at least 20 V")
    takeover = dict(safety.get("takeover_duration_interlock", {}))
    if takeover.get("enabled") is not True or takeover.get("latch_until_disarm") is not True:
        raise RuntimeError("the one-second takeover interlock must latch until DISARM")
    if not 0.0 < float(takeover.get("max_duration_s", math.inf)) <= MAX_TAKEOVER_S:
        raise RuntimeError("takeover duration must be in (0, 1] s")
    aux = dict(safety.get("aux_enable", {}))
    if int(aux.get("channel_index", -1)) != 7 or int(aux.get("min_us", 0)) != 1700:
        raise RuntimeError("the takeover gate must remain RC7/AUX3 high")
    if aux.get("satisfied_by_override_mode") is not True:
        raise RuntimeError("MSP OVERRIDE must satisfy the RC7 gate")

    if rc.get("channel_map") != "AETR1234" or rc.get("rate_mapping_type") != "betaflight":
        raise RuntimeError("RC mapping must use AETR1234 Betaflight rate inversion")
    for key in ("roll_command_limit_deg_s", "pitch_command_limit_deg_s"):
        if not 0.0 < float(rc.get(key, math.inf)) <= MAX_RATE_DEG_S:
            raise RuntimeError(f"{key} must be in (0, 3] deg/s")
    if float(rc.get("yaw_command_limit_deg_s", math.inf)) != 0.0:
        raise RuntimeError("Yaw command must remain zero")
    if int(rc.get("throttle_hover_us", 0)) != 1275:
        raise RuntimeError("measured hover reference must remain 1275 us")
    if int(rc.get("throttle_max_us", 9999)) > MAX_THROTTLE_COMMAND_US:
        raise RuntimeError("RC throttle mapping exceeds 1500 us")
    if float(rc.get("max_delta_us_per_s", math.inf)) > 100.0:
        raise RuntimeError("RC slew limit cannot exceed 100 us/s")
    _validate_rate_profile(rc, parsed_cli)

    command = dict(config.get("guidance_command", {}))
    if not math.isclose(float(command.get("hover_thrust", math.nan)), 0.5):
        raise RuntimeError("logical throttle must remain at the 1275 us hover reference")
    guidance = _validate_guidance_config(config)
    if guidance.get("velocity_source") != "msp_kinematics":
        raise RuntimeError("free flight requires velocity_source=msp_kinematics")
    if float(guidance.get("max_guidance_accel_mps2", math.inf)) > MAX_GUIDANCE_ACCEL_MPS2:
        raise RuntimeError("guidance acceleration exceeds 1 m/s2")
    kinematics = dict(config.get("kinematics", {}))
    if int(kinematics.get("minimum_satellites", 0)) < MIN_GPS_SATELLITES:
        raise RuntimeError("at least six GPS satellites are required")
    if int(kinematics.get("origin_lock_samples", 0)) < 3:
        raise RuntimeError("at least three stable origin samples are required")

    return {
        "camera_extrinsic": _validate_camera_extrinsic(config),
        "guidance": guidance,
        "guidance_command_frames": _validate_guidance_command_frames(config),
        "msp_raw_imu_gyro": _validate_raw_imu_gyro_binding(runtime, fc_identity=fc_identity),
    }


if __name__ == "__main__":
    main()
