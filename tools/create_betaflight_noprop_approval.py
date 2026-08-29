#!/usr/bin/env python3
from __future__ import annotations

import argparse
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

from vision_guidance.geometry import camera_mount_diagnostics, validated_rotation_matrix  # noqa: E402


MSP_OVERRIDE_PERMANENT_ID = 50
MAX_NOPROP_RATE_DEG_S = 3.0
MAX_NOPROP_THROTTLE_US = 1100
MIN_ENTRY_HANDOFF_DURATION_S = 0.8
MAX_ENTRY_GYRO_AGE_S = 0.25
MAX_NOPROP_TILT_ANGLE_DEG = 35.0
MAX_NOPROP_HARDCAP_MARGIN_DEG = 5.0
MAX_NOPROP_GUIDANCE_ACCEL_MPS2 = 1.0
GUIDANCE_EVAL_FRAME = "inertial_ned"
RATE_GAIN_INPUT_FRAME = "body_frd"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Create a hash-bound Betaflight no-prop bench approval.")
    parser.add_argument("--snapshot", required=True, help="Fresh capture_betaflight_snapshot.py manifest.json.")
    parser.add_argument("--config", required=True, help="No-prop JSON configuration to authorize.")
    parser.add_argument("--output", default="logs/betaflight_noprop_approval.json")
    parser.add_argument("--operator", default="")
    parser.add_argument("--acknowledge-props-removed", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.acknowledge_props_removed:
        raise RuntimeError("--acknowledge-props-removed is required")
    snapshot_path = Path(args.snapshot).expanduser().resolve()
    config_path = Path(args.config).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    snapshot = _read_json(snapshot_path)
    config = _read_json(config_path)
    override_mode_cli_id = _configured_override_mode_cli_id(config)
    parsed_cli = _validate_snapshot(
        snapshot,
        snapshot_path,
        expected_override_mode_cli_id=override_mode_cli_id,
    )
    camera_extrinsic = _validate_noprop_config(config, output_path, parsed_cli=parsed_cli)
    guidance = _validate_guidance_config(config)
    guidance_command_frames = _validate_guidance_command_frames(config)

    approval = {
        "schema_version": 1,
        "approved": True,
        "scope": "noprop_bench",
        "created_unix_s": time.time(),
        "created_local": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "operator": str(args.operator),
        "operator_acknowledgement": "all_propellers_physically_removed",
        "source_conflicts_resolved": True,
        "snapshot_manifest": str(snapshot_path),
        "snapshot_sha256": _sha256(snapshot_path),
        "expected_fc_identity": dict(snapshot["fc_identity"]),
        "parameters_path": str(config_path),
        "parameters_sha256": _sha256(config_path),
        "limits": {
            "max_rate_deg_s": MAX_NOPROP_RATE_DEG_S,
            "max_throttle_us": MAX_NOPROP_THROTTLE_US,
            "prefill_required": True,
            "msp_override_permanent_id": MSP_OVERRIDE_PERMANENT_ID,
            "msp_override_cli_mode_id": override_mode_cli_id,
            "entry_handoff_min_duration_s": MIN_ENTRY_HANDOFF_DURATION_S,
            "entry_handoff_max_gyro_age_s": MAX_ENTRY_GYRO_AGE_S,
            "max_tilt_angle_deg": MAX_NOPROP_TILT_ANGLE_DEG,
            "max_hardcap_level_rate_deg_s": MAX_NOPROP_RATE_DEG_S,
            "max_guidance_accel_mps2": MAX_NOPROP_GUIDANCE_ACCEL_MPS2,
        },
        "camera_extrinsic": camera_extrinsic,
        "guidance": guidance,
        "guidance_command_frames": guidance_command_frames,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(approval, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output_path)
    print(f"approval_sha256={_sha256(output_path)}")


def _validate_snapshot(
    snapshot: dict[str, Any],
    snapshot_path: Path,
    *,
    expected_override_mode_cli_id: int,
) -> dict[str, Any]:
    if snapshot.get("readiness", {}).get("log_only_ready") is not True:
        raise RuntimeError("snapshot is not log-only ready")
    if not snapshot.get("fc_identity") or "error" in snapshot.get("fc_identity", {}):
        raise RuntimeError("snapshot flight-controller identity is missing")
    box_ids = tuple(int(value) for value in snapshot.get("box_ids", ()))
    if MSP_OVERRIDE_PERMANENT_ID not in box_ids or snapshot.get("msp_override_available") is not True:
        raise RuntimeError("fresh snapshot must contain MSP OVERRIDE permanent ID 50")
    override_mode = dict(snapshot.get("msp_override_mode") or {})
    if int(override_mode.get("permanent_id", -1)) != MSP_OVERRIDE_PERMANENT_ID:
        raise RuntimeError("snapshot MSP OVERRIDE mode metadata is inconsistent")
    if int(snapshot.get("capture", {}).get("error_count", 1)) != 0:
        raise RuntimeError("snapshot contains MSP capture errors")

    cli = dict(snapshot.get("cli_configuration", {}))
    if cli.get("configuration_evidence_complete") is not True:
        raise RuntimeError("snapshot must include conflict-free diff all and dump all")
    review_name = str(cli.get("review_artifact", ""))
    review_path = snapshot_path.parent / review_name
    expected_hash = str(snapshot.get("artifacts", {}).get(review_name, ""))
    if not review_name or not review_path.is_file() or _sha256(review_path) != expected_hash:
        raise RuntimeError("snapshot configuration review artifact is missing or changed")
    review = _read_json(review_path)
    exports = dict(review.get("exports", {}))
    parsed = dict(exports.get("dump_all") or exports.get("diff_all") or {})
    if not parsed:
        raise RuntimeError("snapshot configuration review has no parsed CLI export")
    settings = dict(parsed.get("settings", {}))
    if int(settings.get("msp_override_channels_mask", -1)) != 15:
        raise RuntimeError("flight controller msp_override_channels_mask must be 15")
    if str(settings.get("msp_override_failsafe", "")).upper() != "OFF":
        raise RuntimeError("flight controller msp_override_failsafe must be OFF for this bench profile")
    if str(parsed.get("receiver", {}).get("channel_map", "")).upper() != "AETR1234":
        raise RuntimeError("flight controller receiver map must be AETR1234")
    aux_ranges = tuple(dict(value) for value in parsed.get("aux_ranges", ()))
    if not any(
        int(aux.get("mode_id", -1)) == expected_override_mode_cli_id
        and int(aux.get("aux_channel_index", -1)) == 2
        and int(aux.get("range_start_us", 0)) == 1700
        and int(aux.get("range_end_us", 0)) == 2100
        for aux in aux_ranges
    ):
        raise RuntimeError("MSP OVERRIDE must be assigned to AUX3/RC7 high (1700-2100 us)")
    return parsed


def _validate_noprop_config(
    config: dict[str, Any],
    output_path: Path,
    *,
    parsed_cli: dict[str, Any] | None = None,
) -> dict[str, Any]:
    authorization = dict(config.get("control_authorization", {}))
    if authorization.get("enabled") is not True or authorization.get("required_scope") != "noprop_bench":
        raise RuntimeError("control_authorization must require noprop_bench")
    configured_approval = Path(str(authorization.get("approval_manifest", ""))).expanduser().resolve()
    if configured_approval != output_path:
        raise RuntimeError("approval output must match control_authorization.approval_manifest")
    camera_extrinsic = _validate_camera_extrinsic(config)

    runtime = dict(config.get("msp_runtime", {}))
    _configured_override_mode_cli_id(config)
    if runtime.get("io_worker_enabled") is not True or runtime.get("prefill_enabled") is not True:
        raise RuntimeError("MSP worker and prefill must be enabled")
    if runtime.get("transport_mode") != "async_pipeline":
        raise RuntimeError("no-prop control requires the asynchronous MSP pipeline")
    if float(runtime.get("response_drain_budget_ms", 0.0)) <= 0.0:
        raise RuntimeError("MSP async response drain budget must be positive")
    if float(runtime.get("response_stale_s", 999.0)) > 0.25:
        raise RuntimeError("MSP SET_RAW_RC acknowledgement timeout must not exceed 0.25 s")
    aux_enable = dict(config.get("safety", {}).get("aux_enable", {}))
    if int(aux_enable.get("channel_index", -1)) != 7:
        raise RuntimeError("no-prop control enable must remain on physical RC7/AUX3")
    if aux_enable.get("satisfied_by_override_mode") is not True:
        raise RuntimeError("RC7 MSP OVERRIDE mode must explicitly satisfy the no-prop AUX gate")
    if int(runtime.get("prefill_min_frames", 0)) < 10:
        raise RuntimeError("no-prop prefill_min_frames must be at least 10")
    if int(runtime.get("override_channels_mask", 0)) != 15:
        raise RuntimeError("no-prop override_channels_mask must be 15")
    if runtime.get("set_raw_rc_channel_map") != "AETR1234":
        raise RuntimeError("no-prop SET_RAW_RC wire order must be AETR1234")
    if int(runtime.get("throttle_channel_zero_based", -1)) != 2:
        raise RuntimeError("AETR SET_RAW_RC throttle channel must be zero-based index 2")
    if int(runtime.get("aux_arm_channel_zero_based", -1)) != 4:
        raise RuntimeError("ARM must remain on physical RC5/AUX1")
    if int(runtime.get("prefill_valid_min_us", 0)) < 900:
        raise RuntimeError("no-prop prefill must reject 885 us startup values")

    rc = dict(config.get("rc_mapping", {}))
    if rc.get("rate_mapping_type") != "betaflight":
        raise RuntimeError("no-prop profile must use Betaflight rate inversion")
    if rc.get("channel_map") != runtime.get("set_raw_rc_channel_map"):
        raise RuntimeError("RC mapper and SET_RAW_RC wire channel maps must match")
    for key in ("roll_command_limit_deg_s", "pitch_command_limit_deg_s", "yaw_command_limit_deg_s"):
        value = float(rc.get(key, float("inf")))
        if value < 0.0 or value > MAX_NOPROP_RATE_DEG_S:
            raise RuntimeError(f"{key} exceeds no-prop limit")
    throttle_min_us = int(rc.get("throttle_min_us", 0))
    throttle_hover_us = int(rc.get("throttle_hover_us", 0))
    throttle_max_us = int(rc.get("throttle_max_us", 9999))
    if not 1000 <= throttle_min_us <= throttle_hover_us <= throttle_max_us <= MAX_NOPROP_THROTTLE_US:
        raise RuntimeError("no-prop throttle PWM envelope must stay within 1000-1100 us")
    if int(rc.get("neutral_throttle_us", 0)) != 1000:
        raise RuntimeError("no-prop neutral_throttle_us must be 1000")
    if float(rc.get("max_delta_us_per_s", 9999.0)) > 100.0:
        raise RuntimeError("no-prop RC slew limit must not exceed 100 us/s")
    if float(rc.get("thrust_max", 9999.0)) > 0.10:
        raise RuntimeError("thrust_max exceeds no-prop limit")
    if parsed_cli is not None:
        _validate_rate_profile(rc, parsed_cli)

    _validate_guidance_config(config)

    guidance_command = dict(config.get("guidance_command", {}))
    _validate_guidance_command_frames(config)
    entry = dict(guidance_command.get("entry_handoff", {}))
    if entry.get("enabled") is not True:
        raise RuntimeError("no-prop entry_handoff must be enabled")
    if entry.get("rate_source") != "zero":
        raise RuntimeError("no-prop entry_handoff rate_source must be zero")
    entry_duration_s = _finite_float(entry, "duration_s")
    if entry_duration_s < MIN_ENTRY_HANDOFF_DURATION_S:
        raise RuntimeError(
            f"no-prop entry_handoff duration_s must be at least {MIN_ENTRY_HANDOFF_DURATION_S} s"
        )
    gyro_max_age_s = _finite_float(entry, "gyro_max_age_s")
    if not 0.0 < gyro_max_age_s <= MAX_ENTRY_GYRO_AGE_S:
        raise RuntimeError(
            f"no-prop entry_handoff gyro_max_age_s must be in (0, {MAX_ENTRY_GYRO_AGE_S}]"
        )

    tilt = dict(guidance_command.get("tilt_envelope", {}))
    if tilt.get("enabled") is not True:
        raise RuntimeError("no-prop tilt_envelope must be enabled")
    max_roll_deg = _finite_float(tilt, "max_roll_angle_deg")
    max_pitch_deg = _finite_float(tilt, "max_pitch_angle_deg")
    if not 0.0 < max_roll_deg <= MAX_NOPROP_TILT_ANGLE_DEG:
        raise RuntimeError("no-prop max_roll_angle_deg exceeds the approved tilt envelope")
    if not 0.0 < max_pitch_deg <= MAX_NOPROP_TILT_ANGLE_DEG:
        raise RuntimeError("no-prop max_pitch_angle_deg exceeds the approved tilt envelope")
    softcap_band_deg = _finite_float(tilt, "softcap_band_deg")
    if not 0.0 < softcap_band_deg < min(max_roll_deg, max_pitch_deg):
        raise RuntimeError("no-prop softcap_band_deg must be positive and below both tilt limits")
    hardcap_margin_deg = _finite_float(tilt, "hardcap_margin_deg")
    if not 0.0 <= hardcap_margin_deg <= MAX_NOPROP_HARDCAP_MARGIN_DEG:
        raise RuntimeError("no-prop hardcap_margin_deg exceeds the approved tilt envelope")
    if _finite_float(tilt, "hardcap_level_kp") <= 0.0:
        raise RuntimeError("no-prop hardcap_level_kp must be positive")
    max_level_rate_deg_s = _finite_float(tilt, "hardcap_max_level_rate_deg_s")
    if not 0.0 < max_level_rate_deg_s <= MAX_NOPROP_RATE_DEG_S:
        raise RuntimeError("no-prop hardcap_max_level_rate_deg_s exceeds no-prop rate limit")

    safety = dict(config.get("safety", {}))
    aux = dict(safety.get("aux_enable", {}))
    if int(aux.get("channel_index", 0)) != 7 or int(aux.get("min_us", 0)) != 1700:
        raise RuntimeError("no-prop takeover gate must use RC7/AUX3 high")
    return camera_extrinsic


def _validate_guidance_config(config: dict[str, Any]) -> dict[str, Any]:
    values = dict(config.get("guidance", {}))
    if "law" not in values:
        raise RuntimeError("guidance.law must be explicitly configured for no-prop approval")
    law = str(values["law"]).strip().lower()
    if law not in {"ttc_png", "fixed_vm_png"}:
        raise RuntimeError(
            f"unsupported guidance.law={law!r}; expected 'ttc_png' or 'fixed_vm_png'"
        )
    max_accel = _positive_guidance_float(values, "max_guidance_accel_mps2")
    if max_accel > MAX_NOPROP_GUIDANCE_ACCEL_MPS2:
        raise RuntimeError(
            "guidance.max_guidance_accel_mps2 exceeds no-prop limit "
            f"{MAX_NOPROP_GUIDANCE_ACCEL_MPS2}"
        )

    metadata = {
        "law": law,
        "navigation_constant": None,
        "fixed_vm_m_s": None,
        "fixed_gain": None,
        "max_guidance_accel_mps2": max_accel,
        "ttc_required": law == "ttc_png",
    }
    if law == "fixed_vm_png":
        navigation_constant = _positive_guidance_float(values, "navigation_constant")
        fixed_vm_m_s = _positive_guidance_float(values, "fixed_vm_m_s")
        fixed_gain = navigation_constant * fixed_vm_m_s
        if not math.isfinite(fixed_gain):
            raise RuntimeError("guidance navigation_constant * fixed_vm_m_s must be finite")
        metadata.update(
            navigation_constant=navigation_constant,
            fixed_vm_m_s=fixed_vm_m_s,
            fixed_gain=fixed_gain,
        )
    return metadata


def _validate_guidance_command_frames(config: dict[str, Any]) -> dict[str, str]:
    values = dict(config.get("guidance_command", {}))
    guidance_eval_frame = str(values.get("guidance_eval_frame", "")).strip().lower()
    rate_gain_input_frame = str(values.get("rate_gain_input_frame", "")).strip().lower()
    if guidance_eval_frame != GUIDANCE_EVAL_FRAME:
        raise RuntimeError(
            f"guidance_command.guidance_eval_frame must be {GUIDANCE_EVAL_FRAME!r}"
        )
    if rate_gain_input_frame != RATE_GAIN_INPUT_FRAME:
        raise RuntimeError(
            f"guidance_command.rate_gain_input_frame must be {RATE_GAIN_INPUT_FRAME!r}"
        )
    return {
        "guidance_eval_frame": guidance_eval_frame,
        "rate_gain_input_frame": rate_gain_input_frame,
    }


def _positive_guidance_float(values: dict[str, Any], key: str) -> float:
    try:
        value = float(values[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"guidance.{key} must be a finite positive number") from exc
    if not math.isfinite(value) or value <= 0.0:
        raise RuntimeError(f"guidance.{key} must be a finite positive number")
    return value


def _validate_camera_extrinsic(config: dict[str, Any]) -> dict[str, Any]:
    camera = dict(config.get("camera", {}))
    if camera.get("R_BC") is None:
        raise RuntimeError("camera.R_BC must be explicit before no-prop RC approval")
    try:
        rotation = validated_rotation_matrix(np.asarray(camera["R_BC"], dtype=float), name="camera.R_BC")
    except (TypeError, ValueError) as exc:
        raise RuntimeError(str(exc)) from exc

    validation = dict(camera.get("extrinsic_validation", {}))
    if validation.get("verified") is not True:
        raise RuntimeError("camera.extrinsic_validation.verified=true is required for no-prop RC approval")
    if str(validation.get("body_frame", "")).upper() != "FRD":
        raise RuntimeError("camera extrinsic body_frame must be FRD")
    if str(validation.get("camera_frame", "")) != "opencv_x_right_y_down_z_forward":
        raise RuntimeError("camera extrinsic camera_frame must use the OpenCV ray convention")
    expected_axis = validation.get("expected_optical_axis_body", [0.0, 0.0, -1.0])
    try:
        diagnostics = camera_mount_diagnostics(rotation, expected_optical_axis_body=expected_axis)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"camera expected optical axis is invalid: {exc}") from exc
    max_error_deg = _finite_float(validation, "max_optical_axis_error_deg")
    if not 0.0 <= max_error_deg <= 10.0:
        raise RuntimeError("camera max_optical_axis_error_deg must be in [0, 10]")
    if float(diagnostics["optical_axis_error_deg"]) > max_error_deg:
        raise RuntimeError(
            "camera optical axis is not aligned with body-up: "
            f"{diagnostics['optical_axis_error_deg']:.3f} deg"
        )
    return {
        "R_BC": [[float(value) for value in row] for row in rotation],
        "body_frame": "FRD",
        "camera_frame": "opencv_x_right_y_down_z_forward",
        "verified": True,
        "max_optical_axis_error_deg": max_error_deg,
        **diagnostics,
    }


def _configured_override_mode_cli_id(config: dict[str, Any]) -> int:
    runtime = dict(config.get("msp_runtime", {}))
    if "override_mode_cli_id" not in runtime:
        raise RuntimeError("msp_runtime.override_mode_cli_id must be explicitly configured")
    try:
        mode_id = int(runtime["override_mode_cli_id"])
    except (TypeError, ValueError) as exc:
        raise RuntimeError("msp_runtime.override_mode_cli_id must be an integer") from exc
    if not 0 <= mode_id <= 255:
        raise RuntimeError("msp_runtime.override_mode_cli_id must be in range 0-255")
    return mode_id


def _finite_float(values: dict[str, Any], key: str) -> float:
    try:
        value = float(values[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"{key} must be a finite number") from exc
    if not math.isfinite(value):
        raise RuntimeError(f"{key} must be a finite number")
    return value


def _validate_rate_profile(rc: dict[str, Any], parsed_cli: dict[str, Any]) -> None:
    profile_index = str(int(rc.get("betaflight_rate_profile_index", -1)))
    profile = dict(parsed_cli.get("rate_profiles", {}).get(profile_index, {}))
    if not profile:
        raise RuntimeError(f"Betaflight rate profile {profile_index} is missing from the CLI snapshot")
    expected_groups = (
        ("betaflight_rc_rate", ("roll_rc_rate", "pitch_rc_rate", "yaw_rc_rate")),
        ("betaflight_super_rate", ("roll_srate", "pitch_srate", "yaw_srate")),
        ("betaflight_expo", ("roll_expo", "pitch_expo", "yaw_expo")),
    )
    for config_key, cli_keys in expected_groups:
        expected = tuple(float(value) for value in rc.get(config_key, ()))
        if len(expected) != 3:
            raise RuntimeError(f"{config_key} must contain three values")
        for expected_value, cli_key in zip(expected, cli_keys):
            try:
                actual_value = float(profile[cli_key]) / 100.0
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError(f"{cli_key} is missing or invalid in rate profile {profile_index}") from exc
            if abs(actual_value - expected_value) > 1.0e-6:
                raise RuntimeError(
                    f"{config_key} does not match Betaflight rate profile {profile_index}: "
                    f"{cli_key}={actual_value}"
                )


def _read_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"JSON root must be an object: {path}")
    return data


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


if __name__ == "__main__":
    main()
