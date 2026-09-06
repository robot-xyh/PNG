#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys
import time
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from vision_guidance.airsim_adapter import (  # noqa: E402
    AirSimDetectionConfig,
    airsim_orientation_to_R_IB,
    choose_detection,
    configure_detection_filter,
    get_detections,
)
from vision_guidance.airsim_log00106_log_only import (  # noqa: E402
    AlgorithmExitStateMachine,
    CSV_FIELDS,
    DelayedVectorQueue,
    GRAVITY_M_S2,
    LOG00106_ALGORITHM_STOP_S,
    Log00106ControllerAdapter,
    LowRateVelocityObserver,
    R_BC_UPWARD_FRD,
    REAL_INTRINSICS,
    ReplayTimingSchedule,
    ThrottleCalibrationTable,
    ThrottleHandover,
    airsim_flu_rates_to_frd,
    bbox_center_and_area,
    closest_point_confirmed,
    euler_frd_from_R_IB,
    frd_rates_to_airsim_flu,
    load_log00106_replay_timing,
    measured_los_ned_from_bbox,
    model_load_factor_from_pwm,
    project_los_to_real_pixel,
    pwm_from_normalized_thrust,
    remap_render_bbox_to_real_intrinsics,
    render_intrinsics,
    sha256_file,
    validate_csv_row,
)
from vision_guidance.attitude_buffer import AttitudeHistoryBuffer  # noqa: E402
from vision_guidance.flight_control import (  # noqa: E402
    AccelerationTiltRateConfig,
    EntryHandoffConfig,
    GuidanceCommandShaper,
    GuidanceCommandShaperConfig,
    ThrustFeedforwardConfig,
    TiltEnvelopeConfig,
    guidance_eval_to_setpoint,
)
from vision_guidance.fusion import PureVisionGuidancePipeline  # noqa: E402
from vision_guidance.los_filter import LOSFilterConfig, LOSKalmanFilter6D  # noqa: E402
from vision_guidance.types import AttitudeSample, FrameDetection, GuidanceEval  # noqa: E402


DEFAULT_CONFIG = PROJECT_ROOT / "config/airsim_log00106_log_only_cases.json"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "logs/analysis/LOG00106_airsim_log_only"
EXPECTED_INPUT_HASHES = {
    "main_csv": "2f4cba58655be4d142237fc5e05c7ad16b0c3c9131a838d1413e9676cdc557a6",
    "meta_json": "109766c2c003e6c67060dc06e538c2c33944cf66b661b22e204a2708d7292238",
    "interceptor_blackbox": "fc58d049df776a6a771f312ec5ad71bbd96763b5db7ed60f1b00116f6ed748ec",
    "target_ulog": "366fbea3ab9d322efe7e597161a32ad48e1caaf3a627b9bed990f715ced7ec96",
}


@dataclass(frozen=True)
class PendingDetection:
    available_time_s: float
    sample_time_s: float
    measurement_age_s: float
    fusion_wait_s: float
    extrapolated: bool
    detection: FrameDetection
    measured_los_ned: np.ndarray


@dataclass(frozen=True)
class DeliveredDetection:
    available_time_s: float
    sample_time_s: float
    measurement_age_s: float
    fusion_wait_s: float
    extrapolated: bool
    detection: FrameDetection
    measured_los_ned: np.ndarray
    filtered_los_ned: np.ndarray
    los_rate_ned_s: np.ndarray
    omega_los_ned_rad_s: np.ndarray
    ttc_valid: bool
    ttc_s: float | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the isolated LOG00106 AirSim LOG_ONLY trend reproduction."
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--case-id", action="append", default=[])
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--run-id", default="")
    parser.add_argument("--skip-throttle-calibration", action="store_true")
    parser.add_argument("--connection-timeout-s", type=float, default=30.0)
    return parser.parse_args()


def _load_config(path: Path) -> dict[str, Any]:
    config = json.loads(path.read_text(encoding="utf-8"))
    if config.get("seed") != 106:
        raise ValueError("LOG00106 reproduction requires seed=106")
    guidance = config["guidance"]
    camera = config["camera"]
    required = (
        (guidance["navigation_constant"], 3.0, "N"),
        (guidance["fixed_vm_m_s"], 10.0, "fixed_vm"),
        (guidance["maximum_guidance_accel_m_s2"], 7.0, "acceleration limit"),
        (camera["width_px"], 640, "image width"),
        (camera["height_px"], 512, "image height"),
    )
    for actual, expected, label in required:
        if not math.isclose(float(actual), float(expected), abs_tol=1.0e-12):
            raise ValueError(f"{label} must remain fixed at {expected}")
    matrix = np.asarray(camera["R_BC_camera_to_body_frd"], dtype=float)
    if not np.allclose(matrix, R_BC_UPWARD_FRD, atol=1.0e-12):
        raise ValueError("camera R_BC does not match the verified upward mount")
    return config


def _resolve_paths(config: Mapping[str, Any], config_path: Path) -> dict[str, Path]:
    paths: dict[str, Path] = {"config": config_path.resolve()}
    settings = Path(config["airsim"]["settings_path"])
    paths["settings"] = settings if settings.is_absolute() else PROJECT_ROOT / settings
    for name, raw_path in config["input_paths"].items():
        path = Path(raw_path)
        paths[name] = path if path.is_absolute() else PROJECT_ROOT / path
    missing = [f"{name}={path}" for name, path in paths.items() if not path.exists()]
    if missing:
        raise FileNotFoundError("missing LOG00106 inputs: " + ", ".join(missing))
    hashes = {name: sha256_file(path) for name, path in paths.items()}
    for name, expected in EXPECTED_INPUT_HASHES.items():
        if hashes.get(name) != expected:
            raise ValueError(f"unexpected SHA256 for {name}: {hashes.get(name)}")
    return paths


def _git_metadata() -> dict[str, Any]:
    def run(*args: str) -> str:
        result = subprocess.run(
            ["git", *args], cwd=PROJECT_ROOT, text=True, capture_output=True, check=False
        )
        return result.stdout.strip()

    return {
        "commit": run("rev-parse", "HEAD"),
        "branch": run("branch", "--show-current"),
        "status_porcelain": run("status", "--porcelain"),
    }


def _client(airsim_module: Any, config: Mapping[str, Any]):
    host = str(config["airsim"]["host"])
    port = int(config["airsim"]["port"])
    if host != "127.0.0.2":
        raise RuntimeError("LOG00106 AirSim runner is pinned to 127.0.0.2")
    env_host = os.environ.get("AIRSIM_RPC_HOST", host)
    env_port = int(os.environ.get("AIRSIM_RPC_PORT", str(port)))
    if env_host != host or env_port != port:
        raise RuntimeError(
            f"RPC endpoint mismatch: config={host}:{port}, env={env_host}:{env_port}"
        )
    return airsim_module.MultirotorClient(ip=host, port=port)


def _connect_when_ready(airsim_module: Any, config: Mapping[str, Any], timeout_s: float):
    deadline = time.monotonic() + float(timeout_s)
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        client = _client(airsim_module, config)
        try:
            client.ping()
            client.confirmConnection()
            return client
        except Exception as exc:  # AirSim RPC exposes several transport exception types.
            last_error = exc
            time.sleep(0.5)
    raise RuntimeError(f"AirSim RPC did not become ready: {last_error}")


def _xyz(vector: Any) -> np.ndarray:
    return np.array([float(vector.x_val), float(vector.y_val), float(vector.z_val)], dtype=float)


def _sim_timestamp_s(state: Any) -> float:
    return float(state.timestamp) * 1.0e-9


def _pose(airsim_module: Any, position: Sequence[float], attitude_frd_deg: Sequence[float]):
    roll_deg, pitch_deg, yaw_deg = (float(value) for value in attitude_frd_deg)
    return airsim_module.Pose(
        airsim_module.Vector3r(*(float(value) for value in position)),
        airsim_module.to_quaternion(
            math.radians(pitch_deg), math.radians(roll_deg), math.radians(yaw_deg)
        ),
    )


def _kinematics_state(
    airsim_module: Any,
    position: Sequence[float],
    velocity: Sequence[float],
    attitude_frd_deg: Sequence[float],
):
    state = airsim_module.KinematicsState()
    state.position = airsim_module.Vector3r(*(float(value) for value in position))
    roll_deg, pitch_deg, yaw_deg = (float(value) for value in attitude_frd_deg)
    state.orientation = airsim_module.to_quaternion(
        math.radians(pitch_deg), math.radians(roll_deg), math.radians(yaw_deg)
    )
    state.linear_velocity = airsim_module.Vector3r(*(float(value) for value in velocity))
    state.angular_velocity = airsim_module.Vector3r(0.0, 0.0, 0.0)
    state.linear_acceleration = airsim_module.Vector3r(0.0, 0.0, 0.0)
    state.angular_acceleration = airsim_module.Vector3r(0.0, 0.0, 0.0)
    return state


def _set_vehicle_state(
    client: Any,
    airsim_module: Any,
    vehicle_name: str,
    position: Sequence[float],
    velocity: Sequence[float],
    attitude_frd_deg: Sequence[float],
) -> None:
    client.simSetKinematics(
        _kinematics_state(airsim_module, position, velocity, attitude_frd_deg),
        True,
        vehicle_name=vehicle_name,
    )


def _set_upward_camera(client: Any, airsim_module: Any, config: Mapping[str, Any]) -> dict[str, Any]:
    camera = config["camera"]
    vehicle_name = str(config["airsim"]["vehicle_name"])
    camera_name = str(config["airsim"]["camera_name"])
    pitch_deg = float(camera["airsim_api_pitch_deg"])
    client.simSetCameraPose(
        camera_name,
        airsim_module.Pose(
            airsim_module.Vector3r(0.0, 0.0, 0.0),
            airsim_module.to_quaternion(math.radians(pitch_deg), 0.0, 0.0),
        ),
        vehicle_name=vehicle_name,
    )
    client.simSetCameraFov(camera_name, float(camera["horizontal_fov_deg"]), vehicle_name=vehicle_name)
    info = client.simGetCameraInfo(camera_name, vehicle_name=vehicle_name)
    actual_fov = float(info.fov)
    expected_fov = float(camera["horizontal_fov_deg"])
    if abs(actual_fov - expected_fov) > 0.5:
        raise RuntimeError(f"AirSim camera FOV mismatch: expected {expected_fov}, got {actual_fov}")
    responses = client.simGetImages(
        [airsim_module.ImageRequest(camera_name, airsim_module.ImageType.Scene, False, False)],
        vehicle_name=vehicle_name,
    )
    if not responses:
        raise RuntimeError("AirSim camera did not return a Scene image for geometry validation")
    width = int(responses[0].width)
    height = int(responses[0].height)
    if width != int(camera["width_px"]) or height != int(camera["height_px"]):
        raise RuntimeError(
            f"AirSim image size mismatch: expected {camera['width_px']}x{camera['height_px']}, got {width}x{height}"
        )
    return {
        "requested_api_pitch_deg": pitch_deg,
        "reported_fov_deg": actual_fov,
        "reported_image_size_px": [width, height],
    }


def _spawn_actor(
    client: Any,
    airsim_module: Any,
    config: Mapping[str, Any],
    position_world_ned: Sequence[float],
    scale_xyz: Sequence[float],
) -> None:
    airsim_cfg = config["airsim"]
    actor_name = str(airsim_cfg["actor_name"])
    pose = airsim_module.Pose(
        airsim_module.Vector3r(*(float(value) for value in position_world_ned)),
        airsim_module.to_quaternion(0.0, 0.0, 0.0),
    )
    scale = airsim_module.Vector3r(*(float(value) for value in scale_xyz))

    # AirSim 1.8.1 keeps runtime-spawned Actors across reset(). Destroying and
    # immediately respawning the same name can leave the UE Actor pending kill
    # and crash Blocks with a duplicate-name fatal error. Reuse it instead.
    existing = client.simListSceneObjects(f"^{re.escape(actor_name)}$")
    if actor_name in existing:
        if not client.simSetObjectPose(actor_name, pose, True):
            raise RuntimeError(f"failed to update target Actor pose for {actor_name!r}")
        if not client.simSetObjectScale(actor_name, scale):
            raise RuntimeError(f"failed to update target Actor scale for {actor_name!r}")
        return

    spawned = bool(
        client.simSpawnObject(
            actor_name,
            str(airsim_cfg["actor_asset"]),
            pose,
            scale,
            bool(airsim_cfg["actor_physics"]),
            False,
        )
    )
    if not spawned:
        raise RuntimeError(
            f"failed to spawn target Actor {actor_name!r} from {airsim_cfg['actor_asset']!r}"
        )


def _detection_config(config: Mapping[str, Any]) -> AirSimDetectionConfig:
    airsim_cfg = config["airsim"]
    return AirSimDetectionConfig(
        camera_name=str(airsim_cfg["camera_name"]),
        image_type_name=str(airsim_cfg["image_type"]),
        detection_radius_cm=float(airsim_cfg["detection_radius_cm"]),
        mesh_name_pattern=str(airsim_cfg["actor_name"]),
        vehicle_name=str(airsim_cfg["vehicle_name"]),
    )


def _configure_actor_detection(client: Any, airsim_module: Any, config: Mapping[str, Any]) -> AirSimDetectionConfig:
    detection_config = _detection_config(config)
    configure_detection_filter(client, detection_config)
    image_type = getattr(airsim_module.ImageType, detection_config.image_type_name)
    for alias in (config["airsim"]["actor_name"], config["airsim"]["actor_asset"]):
        client.simAddDetectionFilterMeshName(
            detection_config.camera_name,
            image_type,
            str(alias),
            vehicle_name=detection_config.vehicle_name,
        )
    return detection_config


def _selected_bbox(client: Any, detection_config: AirSimDetectionConfig, config: Mapping[str, Any]):
    detections = list(get_detections(client, detection_config))
    selected = choose_detection(detections, preferred_name=str(config["airsim"]["actor_name"]))
    if selected is None:
        return None
    box = selected.box2D
    return (
        float(box.min.x_val),
        float(box.min.y_val),
        float(box.max.x_val),
        float(box.max.y_val),
    )


def _initial_world_position(client: Any, vehicle_name: str) -> np.ndarray:
    return _xyz(client.simGetObjectPose(vehicle_name).position)


def _prepare_case_geometry(
    client: Any,
    airsim_module: Any,
    config: Mapping[str, Any],
    relative_position_ned_m: Sequence[float],
    actor_scale_xyz: Sequence[float],
    *,
    initial_velocity_ned_m_s: Sequence[float],
) -> tuple[np.ndarray, np.ndarray, AirSimDetectionConfig, dict[str, Any]]:
    airsim_cfg = config["airsim"]
    vehicle_name = str(airsim_cfg["vehicle_name"])
    local_position = airsim_cfg["initial_local_position_ned_m"]
    attitude = config["initial_conditions"]["interceptor_attitude_frd_deg"]
    client.reset()
    client.simPause(False)
    time.sleep(0.6)
    client.enableApiControl(True, vehicle_name=vehicle_name)
    client.armDisarm(True, vehicle_name=vehicle_name)
    _set_upward_camera(client, airsim_module, config)
    _set_vehicle_state(client, airsim_module, vehicle_name, local_position, [0.0, 0.0, 0.0], attitude)
    time.sleep(0.15)
    # SimpleFlight starts integrating as soon as it is armed. Reapply the exact
    # state before deriving the Actor world pose so setup time cannot alter range.
    _set_vehicle_state(client, airsim_module, vehicle_name, local_position, [0.0, 0.0, 0.0], attitude)
    interceptor_world = _initial_world_position(client, vehicle_name)
    target_world = interceptor_world + np.asarray(relative_position_ned_m, dtype=float)
    _spawn_actor(client, airsim_module, config, target_world, actor_scale_xyz)
    detection_config = _configure_actor_detection(client, airsim_module, config)
    time.sleep(0.15)
    client.simPause(True)
    _set_vehicle_state(
        client,
        airsim_module,
        vehicle_name,
        local_position,
        initial_velocity_ned_m_s,
        attitude,
    )
    interceptor_world = _initial_world_position(client, vehicle_name)
    expected_relative = np.asarray(relative_position_ned_m, dtype=float)
    _spawn_actor(
        client,
        airsim_module,
        config,
        interceptor_world + expected_relative,
        actor_scale_xyz,
    )
    # Refresh one rendered frame after changing poses while paused. The short
    # step is outside the case clock; reset both poses immediately afterward.
    client.simContinueForTime(0.003)
    _set_vehicle_state(
        client,
        airsim_module,
        vehicle_name,
        local_position,
        initial_velocity_ned_m_s,
        attitude,
    )
    interceptor_world = _initial_world_position(client, vehicle_name)
    _spawn_actor(
        client,
        airsim_module,
        config,
        interceptor_world + expected_relative,
        actor_scale_xyz,
    )
    target_world = _xyz(client.simGetObjectPose(str(airsim_cfg["actor_name"])).position)
    relative_error = target_world - interceptor_world - expected_relative
    if np.linalg.norm(relative_error) > 0.01:
        raise RuntimeError(
            "initial AirSim relative-position assertion failed: "
            f"error_ned_m={relative_error.tolist()}"
        )
    state = client.getMultirotorState(vehicle_name=vehicle_name)
    R_IB = airsim_orientation_to_R_IB(state.kinematics_estimated.orientation)
    truth_los = (target_world - interceptor_world) / np.linalg.norm(target_world - interceptor_world)
    expected_u, expected_v = project_los_to_real_pixel(truth_los, R_IB)
    target_box = config["camera"]["first_bbox_target_xyxy_px"]
    target_u = 0.5 * (float(target_box[0]) + float(target_box[2]))
    target_v = 0.5 * (float(target_box[1]) + float(target_box[3]))
    projection_error = math.hypot(expected_u - target_u, expected_v - target_v)
    if projection_error > float(config["camera"]["first_bbox_center_tolerance_px"]):
        raise RuntimeError(
            "upward projection assertion failed: "
            f"predicted=({expected_u:.3f},{expected_v:.3f}), target=({target_u:.3f},{target_v:.3f})"
        )
    return interceptor_world, target_world, detection_config, {
        "truth_los_ned": truth_los.tolist(),
        "projected_pixel_real_intrinsics": [expected_u, expected_v],
        "projection_center_error_px": projection_error,
        "relative_position_error_ned_m": relative_error.tolist(),
    }


def _calibrate_actor_scale(
    client: Any,
    airsim_module: Any,
    config: Mapping[str, Any],
) -> tuple[tuple[float, float, float], dict[str, Any]]:
    initial = config["initial_conditions"]
    scale = np.asarray(config["airsim"]["actor_scale_xyz"], dtype=float)
    relative = initial["relative_position_5p06_ned_m"]
    target_box = tuple(float(value) for value in config["camera"]["first_bbox_target_xyxy_px"])
    target_width = target_box[2] - target_box[0]
    target_height = target_box[3] - target_box[1]
    history: list[dict[str, Any]] = []
    final_box: tuple[float, float, float, float] | None = None
    projection = {}
    for iteration in range(5):
        _, _, detection_config, projection = _prepare_case_geometry(
            client,
            airsim_module,
            config,
            relative,
            scale,
            initial_velocity_ned_m_s=[0.0, 0.0, 0.0],
        )
        raw_box = None
        for _ in range(20):
            raw_box = _selected_bbox(client, detection_config, config)
            if raw_box is not None:
                break
            time.sleep(0.05)
        if raw_box is None:
            raise RuntimeError("AirSim simGetDetections did not return the LOG00106 target Actor")
        source_intrinsics = render_intrinsics(
            int(config["camera"]["width_px"]),
            int(config["camera"]["height_px"]),
            float(config["camera"]["horizontal_fov_deg"]),
        )
        final_box = remap_render_bbox_to_real_intrinsics(raw_box, source_intrinsics)
        width = final_box[2] - final_box[0]
        height = final_box[3] - final_box[1]
        center_u, center_v, _ = bbox_center_and_area(final_box)
        target_u, target_v, _ = bbox_center_and_area(target_box)
        width_error = abs(width / target_width - 1.0)
        height_error = abs(height / target_height - 1.0)
        center_error = math.hypot(center_u - target_u, center_v - target_v)
        history.append(
            {
                "iteration": iteration,
                "scale_xyz": scale.tolist(),
                "bbox_real_intrinsics_xyxy_px": list(final_box),
                "center_error_px": center_error,
                "width_relative_error": width_error,
                "height_relative_error": height_error,
            }
        )
        if (
            center_error <= float(config["camera"]["first_bbox_center_tolerance_px"])
            and width_error <= float(config["camera"]["first_bbox_size_relative_tolerance"])
            and height_error <= float(config["camera"]["first_bbox_size_relative_tolerance"])
        ):
            break
        factor = math.sqrt(max(0.1, target_width / max(width, 1.0e-6)) * max(0.1, target_height / max(height, 1.0e-6)))
        scale *= float(np.clip(factor, 0.5, 2.0))
    else:
        raise RuntimeError(f"target Actor visual scale calibration failed: {history[-1]}")
    return tuple(float(value) for value in scale), {
        "status": "passed",
        "visual_proxy_only": True,
        "target_bbox_xyxy_px": list(target_box),
        "final_bbox_xyxy_px": list(final_box or ()),
        "iterations": history,
        "projection_assertion": projection,
    }


def _calibrate_throttle(
    client: Any,
    airsim_module: Any,
    config: Mapping[str, Any],
) -> tuple[ThrottleCalibrationTable, dict[str, Any]]:
    vehicle_name = str(config["airsim"]["vehicle_name"])
    position = config["airsim"]["initial_local_position_ned_m"]
    calibration_position = [float(position[0]), float(position[1]), float(position[2]) - 10.0]
    commands = (0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.0)
    raw_loads: list[float] = []
    samples: list[dict[str, float]] = []
    try:
        client.simDestroyObject(str(config["airsim"]["actor_name"]))
    except Exception:
        pass
    client.reset()
    time.sleep(0.5)
    client.enableApiControl(True, vehicle_name=vehicle_name)
    client.armDisarm(True, vehicle_name=vehicle_name)
    for command in commands:
        _set_vehicle_state(client, airsim_module, vehicle_name, calibration_position, [0.0, 0.0, 0.0], [0.0, 0.0, 333.0])
        client.moveByAngleRatesThrottleAsync(0.0, 0.0, 0.0, command, 0.55, vehicle_name=vehicle_name)
        time.sleep(0.18)
        first = client.getMultirotorState(vehicle_name=vehicle_name)
        first_velocity = _xyz(first.kinematics_estimated.linear_velocity)
        first_time = _sim_timestamp_s(first)
        time.sleep(0.24)
        second = client.getMultirotorState(vehicle_name=vehicle_name)
        second_velocity = _xyz(second.kinematics_estimated.linear_velocity)
        second_time = _sim_timestamp_s(second)
        dt = max(1.0e-3, second_time - first_time)
        acceleration_d = float((second_velocity[2] - first_velocity[2]) / dt)
        load_factor = max(0.0, (GRAVITY_M_S2 - acceleration_d) / GRAVITY_M_S2)
        raw_loads.append(load_factor)
        samples.append(
            {
                "airsim_throttle_command": command,
                "sample_dt_s": dt,
                "acceleration_d_m_s2": acceleration_d,
                "raw_load_factor_g": load_factor,
            }
        )
    monotonic_loads = np.maximum.accumulate(np.asarray(raw_loads, dtype=float))
    for index in range(1, len(monotonic_loads)):
        if monotonic_loads[index] <= monotonic_loads[index - 1]:
            monotonic_loads[index] = monotonic_loads[index - 1] + 1.0e-6
    table = ThrottleCalibrationTable(
        tuple(commands),
        tuple(float(value) for value in monotonic_loads),
        "AirSim level-flight finite-difference startup calibration",
    )
    return table, {
        "source": table.source,
        "raw_samples": samples,
        "monotonic_commands": list(table.commands),
        "monotonic_load_factors_g": list(table.load_factors_g),
    }


def _fallback_throttle_calibration() -> tuple[ThrottleCalibrationTable, dict[str, Any]]:
    hover = 0.5865
    commands = (0.35, hover, 0.70, 0.80, 0.90, 1.0)
    loads = tuple(command / hover for command in commands)
    table = ThrottleCalibrationTable(commands, loads, "AirSim generic quad nominal thrust ratio")
    return table, {
        "source": table.source,
        "warning": "startup finite-difference calibration was explicitly skipped",
        "monotonic_commands": list(table.commands),
        "monotonic_load_factors_g": list(table.load_factors_g),
    }


def _pipeline() -> tuple[PureVisionGuidancePipeline, AttitudeHistoryBuffer]:
    attitude_buffer = AttitudeHistoryBuffer(duration_s=2.0)
    los_filter = LOSKalmanFilter6D(
        LOSFilterConfig(
            process_lambda=1.0e-4,
            process_lambda_dot=5.0e-3,
            measurement_noise=5.0e-3,
            innovation_reject=0.25,
        )
    )
    return (
        PureVisionGuidancePipeline(
            REAL_INTRINSICS,
            R_BC_UPWARD_FRD,
            attitude_buffer,
            los_filter=los_filter,
        ),
        attitude_buffer,
    )


def _mapping_config() -> AccelerationTiltRateConfig:
    return AccelerationTiltRateConfig(
        gravity_mps2=GRAVITY_M_S2,
        roll_attitude_kp_s_inv=4.0,
        pitch_attitude_kp_s_inv=4.0,
        max_roll_tilt_deg=35.0,
        max_pitch_tilt_deg=35.0,
        max_roll_rate_deg_s=60.0,
        max_pitch_rate_deg_s=60.0,
        roll_rate_sign=1.0,
        # The production -1 setting compensates the Betaflight/MSP pitch display
        # convention. This AirSim path is already normalized to physical FRD.
        pitch_rate_sign=1.0,
        min_vertical_specific_force_mps2=0.5,
        thrust_feedforward=ThrustFeedforwardConfig(
            enabled=True,
            model="measured_load_factor",
            hover_load_factor_g=1.0,
            max_load_factor_g=2.37,
            minimum_tilt_cosine=0.5,
            calibration_id="LOG00062_1275_1500",
        ),
    )


def _command_shaper() -> GuidanceCommandShaper:
    return GuidanceCommandShaper(
        GuidanceCommandShaperConfig(
            entry_handoff=EntryHandoffConfig(
                enabled=True,
                duration_s=0.8,
                gyro_max_age_s=0.25,
                rate_source="gyro",
            ),
            tilt_envelope=TiltEnvelopeConfig(
                enabled=True,
                max_roll_angle_deg=35.0,
                max_pitch_angle_deg=35.0,
                softcap_band_deg=10.0,
                hardcap_margin_deg=5.0,
                hardcap_level_kp=3.0,
                hardcap_max_level_rate_deg_s=60.0,
            ),
        )
    )


def _capture_detection(
    client: Any,
    detection_config: AirSimDetectionConfig,
    config: Mapping[str, Any],
    source_intrinsics: Any,
    R_IB: np.ndarray,
    frame_id: int,
    sample_time_s: float,
    available_time_s: float,
    measurement_age_s: float,
    fusion_wait_s: float,
    extrapolated: bool,
) -> PendingDetection | None:
    raw_box = _selected_bbox(client, detection_config, config)
    if raw_box is None:
        return None
    box = remap_render_bbox_to_real_intrinsics(raw_box, source_intrinsics)
    detection = FrameDetection(
        frame_id=frame_id,
        exposure_ts=float(sample_time_s),
        bbox_xyxy=box,
        track_id=106,
        score=1.0,
    )
    measured_los = measured_los_ned_from_bbox(box, R_IB)
    return PendingDetection(
        available_time_s=float(available_time_s),
        sample_time_s=float(sample_time_s),
        measurement_age_s=float(measurement_age_s),
        fusion_wait_s=float(fusion_wait_s),
        extrapolated=bool(extrapolated),
        detection=detection,
        measured_los_ned=measured_los,
    )


def _deliver_pending(
    pending: deque[PendingDetection],
    timestamp_s: float,
    pipeline: PureVisionGuidancePipeline,
    last: DeliveredDetection | None,
) -> DeliveredDetection | None:
    delivered = last
    while pending and pending[0].available_time_s <= timestamp_s + 1.0e-9:
        item = pending.popleft()
        result = pipeline.process(item.detection)
        if result.los is None or not result.los.valid:
            continue
        delivered = DeliveredDetection(
            available_time_s=item.available_time_s,
            sample_time_s=item.sample_time_s,
            measurement_age_s=item.measurement_age_s,
            fusion_wait_s=item.fusion_wait_s,
            extrapolated=item.extrapolated,
            detection=item.detection,
            measured_los_ned=item.measured_los_ned,
            filtered_los_ned=np.asarray(result.los.lambda_I, dtype=float),
            los_rate_ned_s=np.asarray(result.los.lambda_dot_I, dtype=float),
            omega_los_ned_rad_s=np.asarray(result.los.omega_los, dtype=float),
            ttc_valid=bool(result.ttc is not None and result.ttc.valid),
            ttc_s=None if result.ttc is None else result.ttc.ttc,
        )
    return delivered


def _pair_collision(client: Any, config: Mapping[str, Any]) -> tuple[bool, str]:
    info = client.simGetCollisionInfo(vehicle_name=str(config["airsim"]["vehicle_name"]))
    if not bool(getattr(info, "has_collided", False)):
        return False, ""
    object_name = str(getattr(info, "object_name", "") or "")
    actor = str(config["airsim"]["actor_name"]).lower()
    asset = str(config["airsim"]["actor_asset"]).lower()
    normalized = object_name.lower()
    return actor in normalized or asset in normalized, object_name


def _vector_fields(row: dict[str, Any], prefix: str, vector: Sequence[float], suffixes: Sequence[str]) -> None:
    for suffix, value in zip(suffixes, vector):
        row[f"{prefix}_{suffix}"] = float(value)


def _empty_vector() -> np.ndarray:
    return np.full(3, np.nan, dtype=float)


def _as_float(row: Mapping[str, Any], key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def _best_rate_fit(rows: Sequence[Mapping[str, Any]], axis: str) -> dict[str, float | None]:
    time_values = np.array([_as_float(row, "t_algorithm_s") for row in rows], dtype=float)
    setpoints = np.array([_as_float(row, f"{axis}_rate_setpoint_deg_s") for row in rows], dtype=float)
    actual = np.array([_as_float(row, f"{axis}_rate_actual_deg_s") for row in rows], dtype=float)
    mask = np.isfinite(time_values) & np.isfinite(setpoints) & np.isfinite(actual)
    time_values, setpoints, actual = time_values[mask], setpoints[mask], actual[mask]
    if len(time_values) < 10:
        return {"lag_ms": None, "correlation": None, "gain": None}
    best: tuple[float, float, float] | None = None
    for lag_s in np.arange(0.0, 0.101, 0.001):
        shifted = np.interp(time_values + lag_s, time_values, actual, left=np.nan, right=np.nan)
        valid = np.isfinite(shifted)
        if np.count_nonzero(valid) < 8 or np.std(setpoints[valid]) < 1.0e-6:
            continue
        correlation = float(np.corrcoef(setpoints[valid], shifted[valid])[0, 1])
        gain = float(np.dot(setpoints[valid], shifted[valid]) / max(np.dot(setpoints[valid], setpoints[valid]), 1.0e-12))
        if best is None or correlation > best[1]:
            best = (float(lag_s), correlation, gain)
    if best is None:
        return {"lag_ms": None, "correlation": None, "gain": None}
    return {"lag_ms": 1000.0 * best[0], "correlation": best[1], "gain": best[2]}


def _metrics(
    rows: Sequence[Mapping[str, Any]],
    case: Mapping[str, Any],
    collision_object_name: str,
) -> dict[str, Any]:
    pre_contact = [row for row in rows if not bool(int(float(row["post_contact"])))]
    active = [row for row in pre_contact if bool(int(float(row["algorithm_active"])))]

    def values(key: str, selected: Sequence[Mapping[str, Any]] = active) -> np.ndarray:
        result = np.array([_as_float(row, key) for row in selected], dtype=float)
        return result[np.isfinite(result)]

    contact_rows = [row for row in rows if bool(int(float(row["contact_detected"])))]
    exit_rows = [row for row in rows if bool(int(float(row["algorithm_exit_event"])))]
    minimum_range = min((_as_float(row, "relative_range_m") for row in rows), default=float("nan"))
    total_accel = values("total_accel_norm_m_s2")
    speed_accel = values("speed_accel_norm_m_s2")
    png_accel = values("png_accel_norm_m_s2")
    fov_accel = values("fov_accel_norm_m_s2")
    initial_active = [row for row in active if _as_float(row, "t_algorithm_s") <= 0.5]
    initial_speed_accel = values("speed_accel_norm_m_s2", initial_active)
    initial_png_accel = values("png_accel_norm_m_s2", initial_active)
    initial_fov_accel = values("fov_accel_norm_m_s2", initial_active)
    throttle = values("throttle_applied_us", pre_contact)
    force = values("specific_force_actual_g", pre_contact)
    velocity_d = values("interceptor_velocity_d_m_s", pre_contact)
    velocity_ref_d = values("velocity_reference_d_m_s")
    los_rate_components = [values(f"lambda_dot_{axis}_s") for axis in ("n", "e", "d")]
    los_rate_length = min((len(component) for component in los_rate_components), default=0)
    los_rate_norm = (
        np.linalg.norm(
            np.column_stack([component[:los_rate_length] for component in los_rate_components]),
            axis=1,
        )
        if los_rate_length
        else np.array([], dtype=float)
    )
    active_times = values("t_algorithm_s")
    control_steps = np.diff(active_times)
    sample_times = np.unique(values("sample_time_s"))
    vision_steps = np.diff(sample_times)
    saturation_denominator = max(1, len(active))
    rate_tracking = {
        "roll": _best_rate_fit(pre_contact, "roll"),
        "pitch": _best_rate_fit(pre_contact, "pitch"),
    }
    result = {
        "case_id": case["case_id"],
        "distance_label": case["distance_label"],
        "exit_mode": case["exit_mode"],
        "timing_profile": case["timing_profile"],
        "force_ratio": float(case["force_ratio"]),
        "outcome": "contact" if contact_rows else "closest_point_or_timeout",
        "contact_detected": bool(contact_rows),
        "contact_time_s": None if not contact_rows else _as_float(contact_rows[0], "t_algorithm_s"),
        "collision_object_name": collision_object_name,
        "minimum_truth_range_m": minimum_range,
        "minimum_truth_range_time_s": _as_float(min(rows, key=lambda row: _as_float(row, "relative_range_m")), "t_algorithm_s") if rows else None,
        "algorithm_exit_time_s": None if not exit_rows else _as_float(exit_rows[0], "t_algorithm_s"),
        "algorithm_exit_remaining_range_m": None if not exit_rows else _as_float(exit_rows[0], "relative_range_m"),
        "active_sample_count": len(active),
        "statistics_window": "algorithm-active, pre-contact unless field says otherwise",
        "maximum_total_accel_m_s2": float(np.max(total_accel)) if len(total_accel) else None,
        "maximum_speed_accel_m_s2": float(np.max(speed_accel)) if len(speed_accel) else None,
        "maximum_png_accel_m_s2": float(np.max(png_accel)) if len(png_accel) else None,
        "maximum_fov_accel_m_s2": float(np.max(fov_accel)) if len(fov_accel) else None,
        "maximum_los_rate_norm_s_inv": float(np.max(los_rate_norm)) if len(los_rate_norm) else None,
        "initial_0p5s_guidance_peak_m_s2": {
            "speed": float(np.max(initial_speed_accel)) if len(initial_speed_accel) else None,
            "png": float(np.max(initial_png_accel)) if len(initial_png_accel) else None,
            "fov": float(np.max(initial_fov_accel)) if len(initial_fov_accel) else None,
        },
        "speed_saturation_fraction": sum(int(float(row["speed_saturated"])) for row in active) / saturation_denominator,
        "png_saturation_fraction": sum(int(float(row["png_saturated"])) for row in active) / saturation_denominator,
        "fov_saturation_fraction": sum(int(float(row["fov_saturated"])) for row in active) / saturation_denominator,
        "total_saturation_fraction": sum(int(float(row["total_saturated"])) for row in active) / saturation_denominator,
        "bbox_in_fov_fraction": sum(int(float(row["bbox_in_fov"])) for row in active) / saturation_denominator,
        "guidance_valid_fraction": sum(int(float(row["guidance_valid"])) for row in active) / saturation_denominator,
        "minimum_velocity_reference_d_m_s": float(np.min(velocity_ref_d)) if len(velocity_ref_d) else None,
        "minimum_interceptor_velocity_d_m_s": float(np.min(velocity_d)) if len(velocity_d) else None,
        "maximum_throttle_us_pre_contact": float(np.max(throttle)) if len(throttle) else None,
        "specific_force_actual_g_pre_contact": {
            "p50": float(np.percentile(force, 50)) if len(force) else None,
            "p95": float(np.percentile(force, 95)) if len(force) else None,
            "maximum": float(np.max(force)) if len(force) else None,
        },
        "control_timing": {
            "configured_rate_hz": 50.0,
            "sample_period_mean_ms": float(1000.0 * np.mean(control_steps)) if len(control_steps) else None,
            "sample_period_p50_ms": float(1000.0 * np.percentile(control_steps, 50)) if len(control_steps) else None,
            "sample_period_p95_ms": float(1000.0 * np.percentile(control_steps, 95)) if len(control_steps) else None,
            "sample_period_max_ms": float(1000.0 * np.max(control_steps)) if len(control_steps) else None,
        },
        "vision_timing": {
            "configured_rate_hz": 30.0 if case["timing_profile"] == "ideal" else None,
            "source": "instantaneous_30_hz" if case["timing_profile"] == "ideal" else "LOG00106_paired_replay",
            "sample_period_mean_ms": float(1000.0 * np.mean(vision_steps)) if len(vision_steps) else None,
            "sample_period_p50_ms": float(1000.0 * np.percentile(vision_steps, 50)) if len(vision_steps) else None,
            "sample_period_p95_ms": float(1000.0 * np.percentile(vision_steps, 95)) if len(vision_steps) else None,
        },
        "rate_tracking": rate_tracking,
    }
    rate_sign_consistent = all(
        item["correlation"] is not None
        and item["gain"] is not None
        and item["correlation"] > 0.0
        and item["gain"] > 0.0
        for item in rate_tracking.values()
    )
    result["trend_checks"] = {
        "upward_reference_d_negative": result["minimum_velocity_reference_d_m_s"] is not None and result["minimum_velocity_reference_d_m_s"] < 0.0,
        "upward_velocity_established": result["minimum_interceptor_velocity_d_m_s"] is not None and result["minimum_interceptor_velocity_d_m_s"] < -0.2,
        "total_accel_within_7_m_s2": result["maximum_total_accel_m_s2"] is not None and result["maximum_total_accel_m_s2"] <= 7.0 + 1.0e-6,
        "throttle_within_1500_us": result["maximum_throttle_us_pre_contact"] is not None and result["maximum_throttle_us_pre_contact"] <= 1500.0 + 1.0e-6,
        "speed_term_initially_dominant": bool(
            len(initial_speed_accel)
            and np.max(initial_speed_accel)
            >= max(
                float(np.max(initial_png_accel)) if len(initial_png_accel) else 0.0,
                float(np.max(initial_fov_accel)) if len(initial_fov_accel) else 0.0,
            )
        ),
        "roll_pitch_command_response_same_sign": rate_sign_consistent,
        "control_period_is_20_ms": bool(
            len(control_steps)
            and np.max(np.abs(control_steps - 0.02)) <= 2.0e-3
        ),
        "ideal_bbox_has_no_dropout": bool(
            case["timing_profile"] != "ideal"
            or result["bbox_in_fov_fraction"] >= 1.0 - 1.0e-12
        ),
    }
    return result


def _run_case(
    client: Any,
    airsim_module: Any,
    config: Mapping[str, Any],
    paths: Mapping[str, Path],
    case: Mapping[str, Any],
    run_id: str,
    run_dir: Path,
    actor_scale_xyz: Sequence[float],
    calibration: ThrottleCalibrationTable,
    provenance: Mapping[str, Any],
    *,
    smoke: bool,
) -> dict[str, Any]:
    case_id = str(case["case_id"])
    case_dir = run_dir / case_id
    case_dir.mkdir(parents=True, exist_ok=True)
    initial = config["initial_conditions"]
    relative_position = initial[str(case["relative_position_key"])]
    interceptor_velocity_initial = initial["interceptor_physical_velocity_ned_m_s"]
    interceptor_world_initial, target_world_initial, detection_config, projection = _prepare_case_geometry(
        client,
        airsim_module,
        config,
        relative_position,
        actor_scale_xyz,
        initial_velocity_ned_m_s=interceptor_velocity_initial,
    )
    client.simPause(True)
    source_intrinsics = render_intrinsics(
        int(config["camera"]["width_px"]),
        int(config["camera"]["height_px"]),
        float(config["camera"]["horizontal_fov_deg"]),
    )
    pipeline, attitude_buffer = _pipeline()
    controller = Log00106ControllerAdapter()
    shaper = _command_shaper()
    mapping_config = _mapping_config()
    measured_timing = str(case["timing_profile"]) == "measured"
    timing_schedule = ReplayTimingSchedule(load_log00106_replay_timing(paths["main_csv"])) if measured_timing else None
    velocity_observer = LowRateVelocityObserver(
        initial["controller_filtered_velocity_ned_m_s"],
        update_rate_hz=float(config["sensitivity"]["velocity_update_rate_hz"]),
        time_constant_s=float(config["sensitivity"]["velocity_filter_tau_s"]),
    )
    rate_delay = float(config["sensitivity"]["rate_command_delay_s"]) if measured_timing else 0.0
    rate_queue = DelayedVectorQueue(rate_delay)
    throttle_handover = ThrottleHandover(
        source_us=float(config["guidance"]["throttle_source_us"]),
        duration_s=float(config["guidance"]["throttle_handover_s"]),
        minimum_us=float(config["guidance"]["throttle_min_us"]),
        maximum_us=float(config["guidance"]["throttle_max_us"]),
        slew_limit_us_s=float(config["guidance"]["throttle_slew_limit_us_s"]),
    )
    exit_state = AlgorithmExitStateMachine(
        early_exit=str(case["exit_mode"]) == "early",
        stop_time_s=float(config["guidance"]["early_exit_time_s"]),
        post_exit_min_s=float(config["airsim"]["post_exit_minimum_s"]),
    )
    control_period_s = 1.0 / float(config["airsim"]["control_rate_hz"])
    vision_period_s = 1.0 / float(config["airsim"]["vision_rate_hz"])
    duration_s = min(0.8, float(config["airsim"]["maximum_duration_s"])) if smoke else float(config["airsim"]["maximum_duration_s"])
    vehicle_name = str(config["airsim"]["vehicle_name"])
    actor_name = str(config["airsim"]["actor_name"])
    force_ratio = float(case["force_ratio"])
    voltage_min, voltage_max = (float(value) for value in config["sensitivity"]["voltage_label_v"])
    pending: deque[PendingDetection] = deque()
    delivered: DeliveredDetection | None = None
    rows: list[dict[str, Any]] = []
    closing_history: deque[float] = deque(maxlen=8)
    next_ideal_capture_s = 0.0
    bbox_in_fov_current = False
    frame_id = 0
    contact_time_s: float | None = None
    collision_object_name = ""
    minimum_range = float("inf")
    previous_velocity: np.ndarray | None = None
    previous_time_s: float | None = None
    specific_force_actual_g = float("nan")
    start_state = client.getMultirotorState(vehicle_name=vehicle_name)
    sim_start_abs_s = _sim_timestamp_s(start_state)
    csv_path = case_dir / "timeseries.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        while True:
            state = client.getMultirotorState(vehicle_name=vehicle_name)
            sim_time_s = max(0.0, _sim_timestamp_s(state) - sim_start_abs_s)
            kin = state.kinematics_estimated
            R_IB = airsim_orientation_to_R_IB(kin.orientation)
            if attitude_buffer.latest_timestamp is None or sim_time_s > attitude_buffer.latest_timestamp + 1.0e-9:
                attitude_buffer.push(AttitudeSample(sim_time_s, R_IB))
            interceptor_world = _initial_world_position(client, vehicle_name)
            target_world = _xyz(client.simGetObjectPose(actor_name).position)
            interceptor_position = interceptor_world - interceptor_world_initial
            target_position = target_world - interceptor_world_initial
            interceptor_velocity = _xyz(kin.linear_velocity)
            target_velocity = np.zeros(3, dtype=float)
            relative = target_world - interceptor_world
            relative_range = float(np.linalg.norm(relative))
            truth_los = relative / max(relative_range, 1.0e-12)
            closing_speed = -float(np.dot(relative, target_velocity - interceptor_velocity)) / max(relative_range, 1.0e-12)
            closing_history.append(closing_speed)
            minimum_range = min(minimum_range, relative_range)
            if previous_velocity is not None and previous_time_s is not None:
                dt = max(1.0e-4, sim_time_s - previous_time_s)
                acceleration_ned = (interceptor_velocity - previous_velocity) / dt
                specific_force_actual_g = float(
                    np.linalg.norm(acceleration_ned - np.array([0.0, 0.0, GRAVITY_M_S2])) / GRAVITY_M_S2
                )
            previous_velocity = interceptor_velocity.copy()
            previous_time_s = sim_time_s
            # AirSim's API accepts FLU rates, but KinematicsState reports the
            # simulator's internal body rates in NED-compatible FRD axes.
            actual_rates_frd = _xyz(kin.angular_velocity)
            roll_deg, pitch_deg, yaw_deg = euler_frd_from_R_IB(R_IB)

            capture_timings: list[tuple[float, float, float, float, bool]] = []
            if measured_timing and timing_schedule is not None:
                for timing in timing_schedule.pop_due(sim_time_s):
                    capture_timings.append(
                        (
                            sim_time_s,
                            sim_time_s + timing.measurement_age_s,
                            timing.measurement_age_s,
                            timing.fusion_wait_s,
                            timing.extrapolated,
                        )
                    )
            else:
                while next_ideal_capture_s <= sim_time_s + 1.0e-9:
                    capture_timings.append((sim_time_s, sim_time_s, 0.0, 0.0, False))
                    next_ideal_capture_s += vision_period_s
            for sample_time_s, available_time_s, age_s, fusion_wait_s, extrapolated in capture_timings:
                item = _capture_detection(
                    client,
                    detection_config,
                    config,
                    source_intrinsics,
                    R_IB,
                    frame_id,
                    sample_time_s,
                    available_time_s,
                    age_s,
                    fusion_wait_s,
                    extrapolated,
                )
                frame_id += 1
                bbox_in_fov_current = item is not None
                if item is not None:
                    pending.append(item)
            delivered = _deliver_pending(pending, sim_time_s, pipeline, delivered)

            exit_decision = exit_state.update(sim_time_s)
            if exit_decision.exit_event:
                rate_queue.clear()
                shaper.reset()
            algorithm_active = exit_decision.algorithm_active
            if measured_timing:
                observed_velocity, _ = velocity_observer.update(sim_time_s, interceptor_velocity)
                velocity_timestamp_s = velocity_observer.last_update_s
            else:
                observed_velocity = interceptor_velocity.copy()
                velocity_timestamp_s = sim_time_s

            if delivered is None:
                controller_input = None
                guidance = controller.update(
                    _controller_input(
                        sim_time_s,
                        None,
                        observed_velocity,
                        velocity_timestamp_s,
                        R_IB,
                    )
                )
            else:
                controller_input = delivered
                guidance = controller.update(
                    _controller_input(
                        sim_time_s,
                        delivered,
                        observed_velocity,
                        velocity_timestamp_s,
                        R_IB,
                    )
                )
            guidance_valid = bool(guidance.valid and algorithm_active)
            guidance_eval = GuidanceEval(
                sim_time_s,
                guidance.acceleration_ned_m_s2 if guidance_valid else np.zeros(3),
                guidance_valid,
                1.0 if guidance_valid else 0.0,
                None if guidance_valid else ("algorithm_exited" if not algorithm_active else guidance.reason),
            )
            generated = guidance_eval_to_setpoint(
                guidance_eval,
                R_IB=R_IB,
                rate_gain_matrix=np.zeros((3, 3), dtype=float),
                hover_thrust=0.5,
                yaw_rate_deg_s=0.0,
                mapping_type="accel_tilt_rate",
                accel_tilt_rate=mapping_config,
            )
            shaped, shaping = shaper.update(
                generated,
                timestamp=sim_time_s,
                gate_open=guidance_valid,
                attitude_deg=(roll_deg, pitch_deg),
                gyro_deg_s=np.degrees(actual_rates_frd),
                gyro_age_s=0.0,
            )
            generated_rates_deg_s = np.array(
                [generated.roll_rate_deg_s, generated.pitch_rate_deg_s, generated.yaw_rate_deg_s], dtype=float
            ) if generated.valid else np.zeros(3, dtype=float)
            shaped_rates_deg_s = np.array(
                [shaped.roll_rate_deg_s, shaped.pitch_rate_deg_s, shaped.yaw_rate_deg_s], dtype=float
            ) if shaped.valid and algorithm_active else np.zeros(3, dtype=float)
            if algorithm_active:
                rate_queue.push(sim_time_s, np.radians(shaped_rates_deg_s))
            applied_rates_frd = rate_queue.output(sim_time_s) if algorithm_active else np.zeros(3, dtype=float)

            model_target_us = pwm_from_normalized_thrust(generated.thrust) if generated.valid else float(config["guidance"]["throttle_source_us"])
            if algorithm_active:
                handover = throttle_handover.update(sim_time_s, model_target_us)
                applied_pwm_us = handover.output_us
                handover_alpha = handover.alpha
                throttle_limited = handover.limited
            else:
                applied_pwm_us = float(config["guidance"]["throttle_source_us"])
                handover_alpha = 1.0
                throttle_limited = False
            model_load = model_load_factor_from_pwm(applied_pwm_us)
            effective_force_ratio = 1.0 + handover_alpha * (force_ratio - 1.0)
            desired_actual_load = model_load * effective_force_ratio
            airsim_throttle, calibration_limited = calibration.command_for_load(desired_actual_load)
            throttle_limited = bool(throttle_limited or calibration_limited)
            rates_flu = frd_rates_to_airsim_flu(applied_rates_frd)
            client.moveByAngleRatesThrottleAsync(
                float(rates_flu[0]),
                float(rates_flu[1]),
                float(rates_flu[2]),
                float(np.clip(airsim_throttle, 0.0, 1.0)),
                max(0.04, 2.0 * control_period_s),
                vehicle_name=vehicle_name,
            )

            collided, collision_name = _pair_collision(client, config)
            contact_event = bool(collided and contact_time_s is None)
            if contact_event:
                contact_time_s = sim_time_s
                collision_object_name = collision_name
            post_contact = contact_time_s is not None and sim_time_s > contact_time_s + 1.0e-9
            closest_event = closest_point_confirmed(closing_history)

            row = {field: "" for field in CSV_FIELDS}
            row.update(
                {
                    "case_id": case_id,
                    "run_id": run_id,
                    "seed": int(config["seed"]),
                    "t_sim_s": sim_time_s,
                    "t_algorithm_s": sim_time_s,
                    "t_contact_s": "" if contact_time_s is None else sim_time_s - contact_time_s,
                    "sample_time_s": "" if delivered is None else delivered.sample_time_s,
                    "available_time_s": "" if delivered is None else delivered.available_time_s,
                    "measurement_age_ms": "" if delivered is None else 1000.0 * delivered.measurement_age_s,
                    "fusion_wait_ms": "" if delivered is None else 1000.0 * delivered.fusion_wait_s,
                    "timing_replay_extrapolated": int(bool(delivered and delivered.extrapolated)),
                    "bbox_measurement_source": "airsim_truth_box" if delivered else "none",
                    "controller_phase": guidance.phase,
                    "guidance_valid": int(guidance_valid),
                    "guidance_reason": "active" if guidance_valid else ("algorithm_exited" if not algorithm_active else guidance.reason),
                    "algorithm_active": int(algorithm_active),
                    "contact_detected": int(contact_time_s is not None),
                    "post_contact": int(post_contact),
                    "relative_range_m": relative_range,
                    "closing_speed_m_s": closing_speed,
                    "miss_distance_truth_m": minimum_range,
                    "bbox_in_fov": int(bbox_in_fov_current),
                    "fov_priority_active": int(guidance.fov_priority_active),
                    "fov_priority_weight": guidance.fov_priority_weight,
                    "speed_saturated": int(guidance.speed_saturated),
                    "png_saturated": int(guidance.png_saturated),
                    "fov_saturated": int(guidance.fov_saturated),
                    "total_saturated": int(guidance.total_saturated),
                    "roll_frd_deg": roll_deg,
                    "pitch_frd_deg": pitch_deg,
                    "yaw_ned_deg": yaw_deg % 360.0,
                    "desired_roll_frd_deg": "" if generated.desired_roll_angle_deg is None else generated.desired_roll_angle_deg,
                    "desired_pitch_frd_deg": "" if generated.desired_pitch_angle_deg is None else generated.desired_pitch_angle_deg,
                    "roll_rate_generated_deg_s": generated_rates_deg_s[0],
                    "pitch_rate_generated_deg_s": generated_rates_deg_s[1],
                    "yaw_rate_generated_deg_s": generated_rates_deg_s[2],
                    "roll_rate_setpoint_deg_s": math.degrees(applied_rates_frd[0]),
                    "pitch_rate_setpoint_deg_s": math.degrees(applied_rates_frd[1]),
                    "yaw_rate_setpoint_deg_s": math.degrees(applied_rates_frd[2]),
                    "roll_rate_actual_deg_s": math.degrees(actual_rates_frd[0]),
                    "pitch_rate_actual_deg_s": math.degrees(actual_rates_frd[1]),
                    "yaw_rate_actual_deg_s": math.degrees(actual_rates_frd[2]),
                    "throttle_model_target_us": model_target_us,
                    "throttle_handover_output_us": applied_pwm_us,
                    "throttle_applied_us": applied_pwm_us,
                    "throttle_handover_alpha": handover_alpha,
                    "airsim_throttle_command_0_1": airsim_throttle,
                    "thrust_model_load_factor_g": model_load,
                    "specific_force_actual_g": specific_force_actual_g if math.isfinite(specific_force_actual_g) else "",
                    "thrust_model_ratio": effective_force_ratio,
                    "specific_force_estimate_source": "truth_velocity_finite_difference",
                    "voltage_label_min_v": voltage_min,
                    "voltage_label_max_v": voltage_max,
                    "rate_limited": int(
                        np.any(np.abs(generated_rates_deg_s) >= np.array([60.0, 60.0, 1.0e9]) - 1.0e-9)
                    ),
                    "tilt_limited": int(
                        abs(float(generated.desired_roll_angle_deg or 0.0)) >= 35.0 - 1.0e-9
                        or abs(float(generated.desired_pitch_angle_deg or 0.0)) >= 35.0 - 1.0e-9
                    ),
                    "throttle_limited": int(throttle_limited),
                    "algorithm_exit_event": int(exit_decision.exit_event),
                    "contact_event": int(contact_event),
                    "closest_point_event": int(closest_event),
                }
            )
            _vector_fields(row, "interceptor_position", interceptor_position, ("n_m", "e_m", "d_m"))
            _vector_fields(row, "interceptor_velocity", interceptor_velocity, ("n_m_s", "e_m_s", "d_m_s"))
            _vector_fields(row, "interceptor_velocity_observed", observed_velocity, ("n_m_s", "e_m_s", "d_m_s"))
            _vector_fields(row, "target_position", target_position, ("n_m", "e_m", "d_m"))
            _vector_fields(row, "target_velocity", target_velocity, ("n_m_s", "e_m_s", "d_m_s"))
            _vector_fields(row, "relative_position", relative, ("n_m", "e_m", "d_m"))
            _vector_fields(row, "lambda_truth", truth_los, ("n", "e", "d"))
            if delivered is not None:
                box = delivered.detection.bbox_xyxy
                center_u, center_v, area_ratio = bbox_center_and_area(box)
                row.update(
                    {
                        "bbox_x1_px": box[0],
                        "bbox_y1_px": box[1],
                        "bbox_x2_px": box[2],
                        "bbox_y2_px": box[3],
                        "bbox_center_u_px": center_u,
                        "bbox_center_v_px": center_v,
                        "bbox_area_ratio": area_ratio,
                    }
                )
                _vector_fields(row, "lambda_measured", delivered.measured_los_ned, ("n", "e", "d"))
                _vector_fields(row, "lambda_filtered", delivered.filtered_los_ned, ("n", "e", "d"))
                _vector_fields(row, "lambda_dot", delivered.los_rate_ned_s, ("n_s", "e_s", "d_s"))
                _vector_fields(row, "omega_los", delivered.omega_los_ned_rad_s, ("n_rad_s", "e_rad_s", "d_rad_s"))
            _guidance_vector_fields(row, "velocity_reference", guidance.velocity_reference_ned_m_s, "m_s")
            _guidance_vector_fields(row, "speed_accel", guidance.speed_acceleration_ned_m_s2, "m_s2")
            _guidance_vector_fields(row, "png_accel", guidance.png_acceleration_ned_m_s2, "m_s2")
            _guidance_vector_fields(row, "fov_accel", guidance.fov_acceleration_ned_m_s2, "m_s2")
            _guidance_vector_fields(row, "total_accel", guidance.acceleration_ned_m_s2, "m_s2")
            validate_csv_row(row)
            writer.writerow(row)
            rows.append(row)

            if smoke and sim_time_s >= duration_s:
                break
            if sim_time_s >= duration_s:
                break
            if str(case["exit_mode"]) == "continuous" and (contact_time_s is not None or closest_event):
                break
            if str(case["exit_mode"]) == "early" and exit_decision.may_stop_run and (
                contact_time_s is not None or closest_event
            ):
                break
            client.simContinueForTime(control_period_s)

    contact_reference_kind = "airsim_actor_contact"
    contact_reference_time_s = contact_time_s
    if contact_reference_time_s is None:
        contact_reference_kind = "airsim_truth_closest_point"
        contact_reference_time_s = _as_float(
            min(rows, key=lambda row: _as_float(row, "relative_range_m")),
            "t_algorithm_s",
        )
    for row in rows:
        row["t_contact_s"] = _as_float(row, "t_algorithm_s") - float(contact_reference_time_s)
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    metrics = _metrics(rows, case, collision_object_name)
    metrics["smoke"] = bool(smoke)
    metrics["projection_assertion"] = projection
    metrics["csv_path"] = str(csv_path.relative_to(PROJECT_ROOT))
    metadata = {
        **dict(provenance),
        "case": dict(case),
        "run_id": run_id,
        "smoke": bool(smoke),
        "actor_scale_xyz": list(actor_scale_xyz),
        "interceptor_initial_world_ned_m": interceptor_world_initial.tolist(),
        "target_initial_world_ned_m": target_world_initial.tolist(),
        "rate_command_delay_s": rate_delay,
        "velocity_observer": "truth_high_rate" if not measured_timing else "5 Hz, first-order tau=0.25 s",
        "bbox_source": "airsim_truth_box",
        "t_contact_reference": contact_reference_kind,
        "t_contact_reference_time_algorithm_s": contact_reference_time_s,
        "real_detector_used": False,
        "real_flight_control_transport_used": False,
    }
    (case_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8"
    )
    (case_dir / "run_metadata.json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8"
    )
    if not metrics["trend_checks"]["control_period_is_20_ms"]:
        raise RuntimeError(f"{case_id}: SteppableClock did not hold the 50 Hz control period")
    if not metrics["trend_checks"]["roll_pitch_command_response_same_sign"]:
        raise RuntimeError(f"{case_id}: FRD command/AirSim angular-rate sign assertion failed")
    print(
        f"case={case_id} outcome={metrics['outcome']} min_range={metrics['minimum_truth_range_m']:.3f}m "
        f"contact_time={metrics['contact_time_s']}"
    )
    return metrics


def _controller_input(
    timestamp_s: float,
    detection: DeliveredDetection | None,
    velocity_ned_m_s: np.ndarray,
    velocity_timestamp_s: float,
    R_IB: np.ndarray,
):
    from vision_guidance.betaflight_intercept_controller import VelocityEstablishingPngInput

    return VelocityEstablishingPngInput(
        timestamp_s=float(timestamp_s),
        los_timestamp_s=None if detection is None else detection.sample_time_s,
        los_update_timestamp_s=(
            None if detection is None else detection.available_time_s
        ),
        lambda_ned=None if detection is None else detection.filtered_los_ned,
        lambda_dot_ned_s=None if detection is None else detection.los_rate_ned_s,
        tracking_valid=detection is not None,
        bbox_area_ratio=None if detection is None else bbox_center_and_area(detection.detection.bbox_xyxy)[2],
        attitude_R_IB=R_IB,
        attitude_valid=True,
        velocity_timestamp_s=float(velocity_timestamp_s),
        velocity_ned_m_s=velocity_ned_m_s,
        velocity_valid=True,
        tracking_reason=None if detection is not None else "no_detection",
        ttc_valid=bool(detection and detection.ttc_valid),
        ttc_s=None if detection is None else detection.ttc_s,
        track_id=None if detection is None else detection.detection.track_id,
    )


def _guidance_vector_fields(row: dict[str, Any], prefix: str, vector: Sequence[float], unit: str) -> None:
    array = np.asarray(vector, dtype=float)
    row[f"{prefix}_n_{unit}"] = float(array[0])
    row[f"{prefix}_e_{unit}"] = float(array[1])
    row[f"{prefix}_d_{unit}"] = float(array[2])
    norm_key = f"{prefix}_norm_{unit}"
    if norm_key in row:
        row[norm_key] = float(np.linalg.norm(array))


def main() -> int:
    args = parse_args()
    config_path = Path(args.config).expanduser().resolve()
    config = _load_config(config_path)
    paths = _resolve_paths(config, config_path)
    try:
        import airsim
    except ImportError as exc:
        raise SystemExit("AirSim Python 1.8.1 is required") from exc
    if str(getattr(airsim, "__version__", "unknown")) != "1.8.1":
        raise RuntimeError(f"expected AirSim Python 1.8.1, got {getattr(airsim, '__version__', 'unknown')}")
    client = _connect_when_ready(airsim, config, args.connection_timeout_s)
    server_version = int(client.getServerVersion())
    client_version = int(client.getClientVersion())
    run_id = args.run_id or datetime.now(timezone.utc).strftime("run_%Y%m%dT%H%M%SZ")
    run_dir = Path(args.output_root).expanduser().resolve() / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    input_hashes = {name: sha256_file(path) for name, path in paths.items()}
    provenance = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "seed": int(config["seed"]),
        "run_command": " ".join(sys.argv),
        "config_path": str(config_path),
        "config_sha256": input_hashes["config"],
        "settings_path": str(paths["settings"]),
        "settings_sha256": input_hashes["settings"],
        "input_sha256": {name: digest for name, digest in input_hashes.items() if name not in {"config", "settings"}},
        "git": _git_metadata(),
        "airsim": {
            "python_version": str(airsim.__version__),
            "client_version": client_version,
            "server_version": server_version,
            "mode": config["airsim"]["mode"],
            "host": config["airsim"]["host"],
            "port": config["airsim"]["port"],
        },
        "evidence_classes": {
            "measured": "LOG00106 CSV/meta, BFL, ULog, and contact-anchored joint products",
            "simulated": "AirSim SimpleFlight truth, Actor collision, and simGetDetections bbox",
            "inferred": "finite-difference specific force and trend comparisons",
        },
    }
    if args.skip_throttle_calibration:
        calibration, calibration_metadata = _fallback_throttle_calibration()
    else:
        calibration, calibration_metadata = _calibrate_throttle(client, airsim, config)
    actor_scale, actor_metadata = _calibrate_actor_scale(client, airsim, config)
    provenance["throttle_calibration"] = calibration_metadata
    provenance["actor_visual_scale_calibration"] = actor_metadata
    cases = list(config["cases"])
    if args.case_id:
        requested = set(args.case_id)
        cases = [case for case in cases if case["case_id"] in requested]
        missing = sorted(requested - {case["case_id"] for case in cases})
        if missing:
            raise ValueError(f"unknown case IDs: {', '.join(missing)}")
    if args.smoke:
        cases = [next(case for case in cases if case["case_id"] == "ideal_5p06_early")]
    metrics = [
        _run_case(
            client,
            airsim,
            config,
            paths,
            case,
            run_id,
            run_dir,
            actor_scale,
            calibration,
            provenance,
            smoke=args.smoke,
        )
        for case in cases
    ]
    summary = {
        **provenance,
        "run_id": run_id,
        "case_count": len(metrics),
        "smoke": bool(args.smoke),
        "cases": metrics,
        "conclusion_limit": (
            "LOG00106 contains one physical contact sample. These AirSim runs do not establish "
            "an 80 percent real-world hit rate."
        ),
    }
    (run_dir / "metrics.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8"
    )
    (Path(args.output_root).expanduser().resolve() / "latest_run.txt").write_text(
        run_id + "\n", encoding="utf-8"
    )
    try:
        client.moveByAngleRatesThrottleAsync(
            0.0,
            0.0,
            0.0,
            calibration.command_for_load(model_load_factor_from_pwm(1303.0))[0],
            0.2,
            vehicle_name=str(config["airsim"]["vehicle_name"]),
        )
    except Exception:
        pass
    print(f"output={run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
