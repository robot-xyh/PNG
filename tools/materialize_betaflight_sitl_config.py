#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Any


SITL_SCOPE = "betaflight_sitl_loopback_v1"
SITL_MSP_URL = "socket://127.0.0.1:5761"
SITL_TAKEOVER_AFTER_S = {
    "noncollision": 7.35,
    "contact": 7.70,
}
SITL_TAKEOVER_DURATION_S = {
    "noncollision": 0.7,
    "contact": 0.9,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Materialize an isolated Betaflight/Gazebo SIL config from a flight candidate."
    )
    parser.add_argument("--base", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--policy", choices=("noncollision", "contact"), required=True)
    parser.add_argument("--simulated-vbat-v", type=float, default=23.6)
    return parser.parse_args()


def sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def materialize_sitl_config(
    base: dict[str, Any],
    *,
    policy: str,
    source_path: Path,
    source_sha256: str,
    simulated_vbat_v: float,
) -> dict[str, Any]:
    if policy not in {"noncollision", "contact"}:
        raise ValueError(f"unsupported SIL policy: {policy}")
    engagement = str(
        dict(dict(base.get("guidance", {})).get("velocity_establishing_png", {})).get(
            "engagement_policy", ""
        )
    )
    if engagement != policy:
        raise RuntimeError(
            f"base engagement_policy={engagement!r} does not match requested {policy!r}"
        )
    if not 22.0 <= float(simulated_vbat_v) <= 25.2:
        raise ValueError("simulated voltage must remain within 22.0-25.2 V")

    result = copy.deepcopy(base)
    result.pop("flight_profile", None)
    result["candidate_profile"] = {
        "id": f"velocity_png_{policy}_betaflight_gazebo_sil_v1",
        "scope": SITL_SCOPE,
        "runnable_as_log_only": True,
        "active_control_runnable_after_fresh_approval": False,
        "control_authorized_without_manifest": False,
        "propellers_installed": False,
        "purpose": (
            "Loopback-only software-in-the-loop validation. It cannot authorize real hardware."
        ),
    }
    result["runtime_policy"] = {
        "allowed_control_modes": ["log_only", "msp_raw_rc"],
        "allow_control_flag_permitted": True,
        "msp_set_raw_rc_permitted": True,
        "required_authorization_scope": SITL_SCOPE,
    }
    result["serial"] = {
        "port": SITL_MSP_URL,
        "baud": 115200,
        "timeout_s": 0.1,
    }
    msp_runtime = dict(result.get("msp_runtime", {}))
    msp_runtime["motor_poll_hz"] = 20
    msp_runtime["raw_imu_poll_hz"] = 20
    raw_imu = dict(msp_runtime.get("raw_imu_gyro", {}))
    raw_imu_axis_sign = list(raw_imu.get("axis_sign", []))
    if raw_imu_axis_sign != [1, -1, 1]:
        raise RuntimeError(
            "SIL requires the flight candidate RAW_IMU axis_sign [1,-1,1]"
        )
    msp_runtime["raw_imu_gyro"] = raw_imu
    result["msp_runtime"] = msp_runtime
    result["control_authorization"] = {
        "enabled": False,
        "required_scope": SITL_SCOPE,
        "approval_manifest": "",
        "note": "SIL scope is isolated from all real-flight approval artifacts.",
    }
    result["sitl_profile"] = {
        "scope": SITL_SCOPE,
        "loopback_only": True,
        "simulated_telemetry_provenance": "gazebo_truth",
        "simulated_vbat_v": float(simulated_vbat_v),
        "simulated_voltage_provenance": "sitl_config_only",
        "projected_detection_latency_s": 0.04,
        "sitl_serial_update_rate_hz": 2000,
        "physics_step_s": 0.0003125,
        "raw_imu_poll_hz": float(msp_runtime["raw_imu_poll_hz"]),
        "raw_imu_axis_binding": {
            "axis_sign": raw_imu_axis_sign,
            "reason": "official_sitl_virtual_sensor_matches_flight_candidate_board_binding",
        },
        "detector_modes": ["sitl_projected", "gazebo_yolo_bytetrack"],
        "target_size_m": [0.55, 0.55, 0.20],
        "pilot_rc": {
            "rate_hz": 100,
            "arm_after_s": 4.5,
            "throttle_after_arm_s": 4.7,
            "takeover_after_s": SITL_TAKEOVER_AFTER_S[policy],
            "takeover_duration_s": SITL_TAKEOVER_DURATION_S[policy],
            "disarm_after_s": 11.5,
            "throttle_us": 1275,
        },
        "generated_from": {
            "path": str(source_path),
            "sha256": str(source_sha256),
            "engagement_policy": policy,
        },
    }
    web = dict(result.get("telemetry_web", {}))
    web["bind"] = "127.0.0.1"
    result["telemetry_web"] = web
    torch_runtime = dict(result.get("torch_runtime", {}))
    torch_runtime["allow_cpu_inference"] = True
    result["torch_runtime"] = torch_runtime
    return result


def main() -> None:
    args = parse_args()
    source_path = Path(args.base).expanduser().resolve()
    output_path = Path(args.output).expanduser()
    base = json.loads(source_path.read_text(encoding="utf-8"))
    result = materialize_sitl_config(
        base,
        policy=args.policy,
        source_path=source_path,
        source_sha256=sha256_path(source_path),
        simulated_vbat_v=args.simulated_vbat_v,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(result, indent=2, sort_keys=False) + "\n",
        encoding="utf-8",
    )
    print(output_path)


if __name__ == "__main__":
    main()
