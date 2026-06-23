from __future__ import annotations

import argparse
import atexit
import csv
import json
import os
import site
import sys
import time
import types
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision_guidance.airsim_adapter import (  # noqa: E402
    AirSimDetectionConfig,
    airsim_orientation_to_R_IB,
    choose_detection,
    configure_detection_filter,
    detection_to_frame_detection,
    get_vehicle_object_collision,
    get_vehicle_pair_collision,
    get_detections,
    infer_intrinsics_from_fov,
)
from vision_guidance.attitude_buffer import AttitudeHistoryBuffer  # noqa: E402
from vision_guidance.geometry import (  # noqa: E402
    airsim_gimbal_camera_to_body,
    camera_ray_from_pixel,
    los_camera_to_inertial,
    normalize,
    project_perpendicular,
)
from vision_guidance.los_filter import LOSKalmanFilter6D  # noqa: E402
from vision_guidance.png_eval import TTCGainSchedule  # noqa: E402
from vision_guidance.terminal_image_kf import (  # noqa: E402
    IMAGE_KF_PREDICT,
    TerminalImageEstimate,
    TerminalImageKF,
    TerminalImageKFConfig,
    center_from_angle_error,
)
from vision_guidance.terminal_extrapolation import (  # noqa: E402
    ABORT_HOLD,
    BLIND_PUSH,
    COMPLETE,
    TERMINAL_VISUAL,
    TerminalConfig,
    TerminalExtrapolator,
)
from vision_guidance.ttc import ScaleExpansionTTC, TTCConfig  # noqa: E402
from vision_guidance.types import AttitudeSample  # noqa: E402
from vision_guidance.yolo_bytetrack_detector import (  # noqa: E402
    add_detector_args,
    create_detection_provider,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
AIRSIM_SETTINGS_PATH = Path.home() / "Documents" / "AirSim" / "settings.json"
SETTINGS_EXAMPLE_PATH = PROJECT_ROOT / "config" / "airsim_blocks_settings.json"
GRAVITY_MPS2 = 9.80665


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run AirSim gimbal-camera pure-vision PNG validation.")
    parser.add_argument("--interceptor", default="Interceptor")
    parser.add_argument("--intruder", default="Intruder")
    parser.add_argument(
        "--intruder-actor",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Use a spawned scene Actor as the intruder target instead of a second AirSim vehicle.",
    )
    parser.add_argument("--intruder-actor-name", default="IntruderActor")
    parser.add_argument("--intruder-actor-asset", default="1M_Cube_Chamfer")
    parser.add_argument("--intruder-actor-scale", type=float, default=2.0)
    parser.add_argument("--intruder-actor-physics", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--intruder-actor-blueprint", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--intruder-actor-respawn", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--collision-interceptor-pattern", action="append", default=None)
    parser.add_argument("--collision-intruder-pattern", action="append", default=None)
    parser.add_argument("--camera", default="0")
    parser.add_argument("--mesh", default="Intruder*")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fov-deg", type=float, default=120.0)
    parser.add_argument("--camera-x", type=float, default=0.0)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=-0.5)
    parser.add_argument("--intruder-speed", type=float, default=5.0)
    parser.add_argument("--intruder-vx", type=float, default=0.0)
    parser.add_argument("--intruder-vy", type=float, default=None)
    parser.add_argument("--intruder-vz", type=float, default=0.0)
    parser.add_argument("--speed-ratio", type=float, default=2.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--min-command-duration-s", type=float, default=0.25)
    parser.add_argument("--command-duration-margin-s", type=float, default=0.20)
    parser.add_argument("--max-command-duration-s", type=float, default=1.00)
    parser.add_argument("--enable-motion", action="store_true", help="Apply AirSim velocity commands.")
    parser.add_argument("--reset", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--px4-interceptor",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Treat the interceptor as PX4 SITL and use PX4-friendly prep/state estimates.",
    )
    parser.add_argument(
        "--px4-intruder",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Treat the intruder as PX4 SITL for dual-SITL tests.",
    )
    parser.add_argument(
        "--px4-max-vertical-speed",
        type=float,
        default=2.0,
        help="PX4 SITL vertical speed clamp for commanded velocities.",
    )
    parser.add_argument(
        "--px4-command-join",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Join PX4 interceptor velocity futures. Disable for high-rate Offboard setpoint streaming.",
    )
    parser.add_argument(
        "--px4-command-mode",
        choices=("velocity_simple", "velocity_yaw_rate"),
        default="velocity_simple",
        help="PX4 SITL velocity command mapping.",
    )
    parser.add_argument("--climb-to-altitude", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--intercept-altitude-m", type=float, default=50.0)
    parser.add_argument(
        "--intruder-altitude-offset-m",
        type=float,
        default=30.0,
        help="Intruder starts this many meters above the interceptor before interception.",
    )
    parser.add_argument("--climb-speed", type=float, default=5.0)
    parser.add_argument("--climb-timeout-s", type=float, default=60.0)
    parser.add_argument("--settle-s", type=float, default=2.0)
    parser.add_argument("--settle-speed", type=float, default=0.5)
    parser.add_argument("--settle-timeout-s", type=float, default=8.0)
    parser.add_argument("--start-horizontal-range-m", type=float, default=None)
    parser.add_argument("--start-forward-offset-m", type=float, default=None)
    parser.add_argument("--start-lateral-offset-m", type=float, default=-20.0)
    parser.add_argument("--start-interceptor-x-m", type=float, default=0.0)
    parser.add_argument("--start-interceptor-y-m", type=float, default=0.0)
    parser.add_argument("--start-geometry-settle-s", type=float, default=0.5)
    parser.add_argument("--hit-radius-m", type=float, default=1.0, help="Deprecated; AirSim collision is the success criterion.")
    parser.add_argument("--gimbal-yaw-limit-deg", type=float, default=80.0)
    parser.add_argument("--gimbal-pitch-limit-deg", type=float, default=45.0)
    parser.add_argument("--gimbal-rate-limit-deg", type=float, default=90.0)
    parser.add_argument("--gimbal-yaw-gain", type=float, default=0.85)
    parser.add_argument("--gimbal-pitch-gain", type=float, default=0.85)
    parser.add_argument("--max-vision-lateral-speed", type=float, default=4.0)
    parser.add_argument("--max-vision-vertical-speed", type=float, default=3.0)
    parser.add_argument("--coast-timeout-s", type=float, default=0.5)
    parser.add_argument("--blind-push-timeout-s", type=float, default=1.0)
    parser.add_argument("--terminal-bbox-area-ratio", type=float, default=0.25)
    parser.add_argument("--terminal-extrapolation", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--terminal-enter-area-ratio", type=float, default=0.20)
    parser.add_argument("--terminal-soft-enter-area-ratio", type=float, default=0.05)
    parser.add_argument("--terminal-cutoff-area-ratio", type=float, default=0.60)
    parser.add_argument("--terminal-gimbal-limit-area-ratio", type=float, default=0.05)
    parser.add_argument("--terminal-cutoff-miss-count", type=int, default=3)
    parser.add_argument("--terminal-min-tracking-time-s", type=float, default=0.20)
    parser.add_argument("--terminal-confidence-min-score", type=float, default=0.35)
    parser.add_argument("--terminal-max-measurement-age-s", type=float, default=0.12)
    parser.add_argument("--terminal-blind-duration-s", type=float, default=0.30)
    parser.add_argument("--terminal-command-average-window-s", type=float, default=0.10)
    parser.add_argument("--terminal-command-decay-tau-s", type=float, default=0.18)
    parser.add_argument("--terminal-trend-bias-gain", type=float, default=0.10)
    parser.add_argument("--terminal-trend-bias-max-mps", type=float, default=1.5)
    parser.add_argument("--terminal-pitch-up-bias-mps", type=float, default=0.8)
    parser.add_argument("--terminal-abort-on-tilt-hardcap", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--terminal-image-kf", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--terminal-image-kf-max-predict-s", type=float, default=0.35)
    parser.add_argument("--terminal-image-kf-meas-noise-rad", type=float, default=0.006)
    parser.add_argument("--terminal-image-kf-accel-noise-rad-s2", type=float, default=8.0)
    parser.add_argument("--terminal-image-kf-innovation-reject-rad", type=float, default=0.20)
    parser.add_argument("--terminal-image-kf-max-angle-rad", type=float, default=1.0)
    parser.add_argument("--terminal-image-kf-max-rate-rad-s", type=float, default=8.0)
    parser.add_argument(
        "--terminal-gimbal-gain-scale",
        type=float,
        default=0.35,
        help="Scale gimbal centering gain while terminal visual state is active.",
    )
    parser.add_argument(
        "--terminal-freeze-gimbal-on-blind",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Hold the last reliable gimbal pose during terminal blind push.",
    )
    parser.add_argument("--gimbal-limit-margin-deg", type=float, default=3.0)
    parser.add_argument("--yaw-control", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--detection-radius-cm", type=float, default=50000.0)
    parser.add_argument("--initial-truth-align", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--diagnostic-truth",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Continuously log intruder truth for offline evaluation only; guidance logic does not consume it.",
    )
    parser.add_argument("--ttc-min-area", type=float, default=0.0)
    parser.add_argument("--ttc-max-s", type=float, default=120.0)
    parser.add_argument(
        "--bbox-noise",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Inject repeatable synthetic noise into AirSim detection bbox before LOS/TTC processing.",
    )
    parser.add_argument("--bbox-center-noise-px", type=float, default=3.0)
    parser.add_argument("--bbox-area-noise-ratio", type=float, default=0.08)
    parser.add_argument("--bbox-noise-seed", type=int, default=20260617)
    parser.add_argument(
        "--los-filter",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use the 6D LOS Kalman filter. Disable it for noiseless AirSim detection/pose analysis.",
    )
    parser.add_argument("--allow-los-fallback", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--los-fallback-gain", type=float, default=0.5)
    add_detector_args(parser)
    parser.add_argument("--detection-warmup-s", type=float, default=1.0)
    parser.add_argument("--show-window", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--window-scale", type=float, default=0.75)
    parser.add_argument("--record-preview", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--preview-dir", default="")
    parser.add_argument(
        "--preview-every-s",
        type=float,
        default=0.0,
        help="When positive, save one annotated recognition frame per this many simulation seconds.",
    )
    parser.add_argument("--preview-every-n", type=int, default=20)
    parser.add_argument("--preview-max-frames", type=int, default=12)
    parser.add_argument("--preview-near-terminal-only", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--print-every-n", type=int, default=10)
    parser.add_argument("--settings-path", default=str(SETTINGS_EXAMPLE_PATH))
    parser.add_argument("--list-vehicles", action="store_true")
    parser.add_argument("--trajectory-dir", default=str(PROJECT_ROOT / "logs"))
    parser.add_argument("--trajectory-prefix", default="")
    parser.add_argument("--no-plot", action="store_true")
    return parser.parse_args()


def _format_names(names: Sequence[str]) -> str:
    if not names:
        return "(none)"
    return ", ".join(repr(name) for name in names)


def _require_vehicles(client, required: Sequence[str]) -> list[str]:
    available = list(client.listVehicles())
    missing = [name for name in required if name not in available]
    if missing:
        raise SystemExit(
            "AirSim vehicle configuration does not match this example.\n"
            f"Required vehicles: {_format_names(required)}\n"
            f"Available vehicles: {_format_names(available)}\n\n"
            f"Use {SETTINGS_EXAMPLE_PATH} or pass matching --interceptor/--intruder names."
        )
    return available


def _vector_xyz(vector) -> np.ndarray:
    return np.array([float(vector.x_val), float(vector.y_val), float(vector.z_val)], dtype=float)


def _load_vehicle_origins(settings_path: str, vehicles: Sequence[str]) -> dict[str, np.ndarray]:
    origins = {vehicle: np.zeros(3, dtype=float) for vehicle in vehicles}
    path = Path(settings_path).expanduser()
    if not path.exists():
        print(f"settings_path_not_found={path}; using zero vehicle origins")
        return origins
    try:
        with path.open("r", encoding="utf-8") as stream:
            settings = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"settings_path_unreadable={path}: {exc}; using zero vehicle origins")
        return origins
    vehicle_settings = settings.get("Vehicles", {})
    for vehicle in vehicles:
        item = vehicle_settings.get(vehicle)
        if isinstance(item, dict):
            origins[vehicle] = np.array(
                [float(item.get("X", 0.0)), float(item.get("Y", 0.0)), float(item.get("Z", 0.0))],
                dtype=float,
            )
    return origins


def _truth_kinematics(client, vehicle_name: str):
    return client.simGetGroundTruthKinematics(vehicle_name=vehicle_name)


def _is_px4_vehicle(vehicle_name: str, args) -> bool:
    if getattr(args, "intruder_actor", False) and vehicle_name == getattr(args, "intruder", ""):
        return False
    return (getattr(args, "px4_interceptor", False) and vehicle_name == args.interceptor) or (
        getattr(args, "px4_intruder", False) and vehicle_name == args.intruder
    )


def _guidance_kinematics(client, vehicle_name: str, args):
    if _is_px4_vehicle(vehicle_name, args):
        return client.getMultirotorState(vehicle_name=vehicle_name).kinematics_estimated
    return _truth_kinematics(client, vehicle_name)


def _world_position(kinematics, vehicle_name: str, origins: dict[str, np.ndarray]) -> np.ndarray:
    return origins.get(vehicle_name, np.zeros(3, dtype=float)) + _vector_xyz(kinematics.position)


def _actor_name(args) -> str:
    return str(getattr(args, "intruder_actor_name", "") or args.intruder)


def _actor_pose(airsim_module, position: np.ndarray, yaw_deg: float = 0.0):
    return airsim_module.Pose(
        airsim_module.Vector3r(float(position[0]), float(position[1]), float(position[2])),
        airsim_module.to_quaternion(0.0, 0.0, np.deg2rad(float(yaw_deg))),
    )


def _actor_position_from_pose(pose) -> np.ndarray:
    return _vector_xyz(pose.position)


def _intruder_truth_position(client, args, origins: dict[str, np.ndarray]) -> np.ndarray:
    if getattr(args, "intruder_actor", False):
        return _actor_position_from_pose(client.simGetObjectPose(_actor_name(args)))
    intruder_kin = _guidance_kinematics(client, args.intruder, args)
    return _world_position(intruder_kin, args.intruder, origins)


def _spawn_or_move_intruder_actor(client, airsim_module, args, world_position: np.ndarray, yaw_deg: float) -> None:
    object_name = _actor_name(args)
    pose = _actor_pose(airsim_module, world_position, yaw_deg)
    if args.intruder_actor_respawn:
        try:
            client.simDestroyObject(object_name)
        except Exception:
            pass
    else:
        try:
            scene_objects = client.simListSceneObjects(f"{object_name}.*")
        except Exception:
            scene_objects = []
        if object_name in scene_objects:
            client.simSetObjectPose(object_name, pose, teleport=True)
            return
    spawned = False
    try:
        scale = airsim_module.Vector3r(
            float(args.intruder_actor_scale),
            float(args.intruder_actor_scale),
            float(args.intruder_actor_scale),
        )
        spawned = bool(
            client.simSpawnObject(
                object_name,
                args.intruder_actor_asset,
                pose,
                scale,
                bool(args.intruder_actor_physics),
                bool(args.intruder_actor_blueprint),
            )
        )
    except Exception as exc:
        print(f"intruder_actor_spawn_warning={exc}; trying simSetObjectPose on existing object")
    if not spawned:
        try:
            client.simSetObjectPose(object_name, pose, teleport=True)
        except Exception as exc:
            raise SystemExit(
                f"Failed to spawn or move intruder actor '{object_name}' with asset "
                f"'{args.intruder_actor_asset}': {exc}"
            ) from exc


def _move_intruder_actor(client, airsim_module, args, world_position: np.ndarray, yaw_deg: float) -> None:
    client.simSetObjectPose(_actor_name(args), _actor_pose(airsim_module, world_position, yaw_deg), teleport=True)


def _actor_collision_patterns(args) -> tuple[str, ...]:
    patterns = list(args.collision_intruder_pattern or [])
    object_name = _actor_name(args)
    asset = str(args.intruder_actor_asset or "")
    patterns.extend([object_name, f"{object_name}*", asset, f"{asset}*"])
    return tuple(pattern for pattern in patterns if pattern)


def _configure_actor_detection_aliases(client, airsim_module, config: AirSimDetectionConfig, args) -> None:
    if not args.intruder_actor:
        return
    image_type = getattr(airsim_module.ImageType, config.image_type_name)
    aliases = {
        str(args.mesh or ""),
        _actor_name(args),
    }
    for alias in sorted(pattern for pattern in aliases if pattern):
        try:
            client.simAddDetectionFilterMeshName(
                config.camera_name,
                image_type,
                alias,
                vehicle_name=config.vehicle_name,
            )
        except Exception as exc:
            print(f"actor_detection_alias_warning alias={alias}: {exc}")


def _px4_local_target_z(vehicle: str, world_target_z: float, origins: dict[str, np.ndarray]) -> float:
    return float(world_target_z - origins.get(vehicle, np.zeros(3, dtype=float))[2])


def _px4_limited_velocity(v_cmd: np.ndarray, args) -> np.ndarray:
    command = np.array(v_cmd, dtype=float)
    vertical_limit = max(0.0, float(args.px4_max_vertical_speed))
    if vertical_limit > 0.0:
        command[2] = float(np.clip(command[2], -vertical_limit, vertical_limit))
    return command


def _ensure_px4_api_control(client, args, vehicle_name: str) -> bool:
    if not _is_px4_vehicle(vehicle_name, args):
        return True
    try:
        if not client.isApiControlEnabled(vehicle_name=vehicle_name):
            client.enableApiControl(True, vehicle_name=vehicle_name)
        armed_key = f"_px4_armed_once_{vehicle_name}"
        if not getattr(args, armed_key, False):
            client.armDisarm(True, vehicle_name=vehicle_name)
            setattr(args, armed_key, True)
        return True
    except Exception as exc:
        print(f"px4_api_control_warning vehicle={vehicle_name}: {exc}")
        return False


def _command_vehicle_velocity(
    client,
    airsim_module,
    vehicle_name: str,
    velocity: np.ndarray,
    yaw_rate_deg_s: float,
    command_duration: float,
    args,
):
    command = _px4_limited_velocity(velocity, args) if _is_px4_vehicle(vehicle_name, args) else np.asarray(velocity, dtype=float)
    if _is_px4_vehicle(vehicle_name, args):
        _ensure_px4_api_control(client, args, vehicle_name)
        if args.px4_command_mode == "velocity_simple":
            return client.moveByVelocityAsync(
                float(command[0]),
                float(command[1]),
                float(command[2]),
                duration=command_duration,
                vehicle_name=vehicle_name,
            )
    return client.moveByVelocityAsync(
        float(command[0]),
        float(command[1]),
        float(command[2]),
        duration=command_duration,
        drivetrain=airsim_module.DrivetrainType.MaxDegreeOfFreedom,
        yaw_mode=airsim_module.YawMode(is_rate=True, yaw_or_rate=yaw_rate_deg_s),
        vehicle_name=vehicle_name,
    )


def _command_interceptor_velocity(client, airsim_module, v_cmd: np.ndarray, command_yaw_deg: float, command_duration: float, args):
    yaw_rate_deg_s = 0.0
    if args.yaw_control and not _is_px4_vehicle(args.interceptor, args):
        drivetrain, yaw_mode = _air_sim_yaw_mode(airsim_module, v_cmd, args)
        future = client.moveByVelocityAsync(
            float(v_cmd[0]),
            float(v_cmd[1]),
            float(v_cmd[2]),
            duration=command_duration,
            drivetrain=drivetrain,
            yaw_mode=yaw_mode,
            vehicle_name=args.interceptor,
        )
    else:
        future = _command_vehicle_velocity(client, airsim_module, args.interceptor, v_cmd, yaw_rate_deg_s, command_duration, args)
    if args.px4_interceptor and args.px4_command_join:
        future.join()
    return future


def _px4_keepalive(client, airsim_module, vehicles: Sequence[str], args, duration_s: float = 1.0) -> None:
    px4_vehicles = [vehicle for vehicle in vehicles if _is_px4_vehicle(vehicle, args)]
    if not px4_vehicles:
        return
    command_dt = 0.20
    deadline = time.monotonic() + max(command_dt, float(duration_s))
    while time.monotonic() < deadline:
        for vehicle in px4_vehicles:
            _command_vehicle_velocity(client, airsim_module, vehicle, np.zeros(3, dtype=float), 0.0, command_dt, args)
        time.sleep(command_dt)


def _register_px4_shutdown_stop(client, args) -> None:
    if not (getattr(args, "px4_interceptor", False) or getattr(args, "px4_intruder", False)):
        return

    vehicles = [args.interceptor]
    if getattr(args, "px4_intruder", False) and not getattr(args, "intruder_actor", False):
        vehicles.append(args.intruder)

    def stop_px4_vehicles() -> None:
        for _ in range(10):
            for vehicle in vehicles:
                try:
                    client.moveByVelocityAsync(0.0, 0.0, 0.0, duration=0.2, vehicle_name=vehicle)
                except Exception:
                    pass
            time.sleep(0.05)

    atexit.register(stop_px4_vehicles)


def _kinematics_timestamp_s(kinematics) -> Optional[float]:
    for name in ("time_stamp", "timestamp"):
        value = getattr(kinematics, name, None)
        if value is None:
            continue
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if not np.isfinite(number):
            continue
        return number * 1.0e-9 if number > 1.0e6 else number
    return None


def _linear_acceleration_mps2(kinematics) -> np.ndarray:
    acceleration = getattr(kinematics, "linear_acceleration", None)
    if acceleration is None:
        return np.zeros(3, dtype=float)
    return _vector_xyz(acceleration)


def _intruder_velocity(args) -> np.ndarray:
    vy = args.intruder_speed if args.intruder_vy is None else args.intruder_vy
    return np.array([args.intruder_vx, vy, args.intruder_vz], dtype=float)


def _apply_bbox_noise(frame_detection, intrinsics, rng: np.random.Generator, args):
    if not args.bbox_noise:
        return frame_detection, {
            "dx": 0.0,
            "dy": 0.0,
            "area_scale": 1.0,
            "raw_bbox": frame_detection.bbox_xyxy,
            "noisy_bbox": frame_detection.bbox_xyxy,
        }

    x1, y1, x2, y2 = frame_detection.bbox_xyxy
    width = max(1.0e-6, x2 - x1)
    height = max(1.0e-6, y2 - y1)
    cx = 0.5 * (x1 + x2)
    cy = 0.5 * (y1 + y2)

    center_sigma = max(0.0, float(args.bbox_center_noise_px))
    area_sigma = max(0.0, float(args.bbox_area_noise_ratio))
    dx = float(rng.normal(0.0, center_sigma)) if center_sigma > 0.0 else 0.0
    dy = float(rng.normal(0.0, center_sigma)) if center_sigma > 0.0 else 0.0
    area_scale = 1.0 + (float(rng.normal(0.0, area_sigma)) if area_sigma > 0.0 else 0.0)
    area_scale = max(0.05, area_scale)
    side_scale = float(np.sqrt(area_scale))

    noisy_cx = cx + dx
    noisy_cy = cy + dy
    noisy_w = width * side_scale
    noisy_h = height * side_scale
    nx1 = max(0.0, noisy_cx - 0.5 * noisy_w)
    ny1 = max(0.0, noisy_cy - 0.5 * noisy_h)
    nx2 = min(float(intrinsics.width), noisy_cx + 0.5 * noisy_w)
    ny2 = min(float(intrinsics.height), noisy_cy + 0.5 * noisy_h)
    if nx2 <= nx1:
        nx2 = min(float(intrinsics.width), nx1 + 1.0)
    if ny2 <= ny1:
        ny2 = min(float(intrinsics.height), ny1 + 1.0)

    noisy = type(frame_detection)(
        frame_id=frame_detection.frame_id,
        exposure_ts=frame_detection.exposure_ts,
        bbox_xyxy=(float(nx1), float(ny1), float(nx2), float(ny2)),
        track_id=frame_detection.track_id,
        score=frame_detection.score,
    )
    return noisy, {
        "dx": dx,
        "dy": dy,
        "area_scale": area_scale,
        "raw_bbox": frame_detection.bbox_xyxy,
        "noisy_bbox": noisy.bbox_xyxy,
    }


def _target_altitude_m(vehicle: str, args) -> float:
    base_altitude = abs(float(args.intercept_altitude_m))
    if vehicle == args.intruder:
        return base_altitude + float(args.intruder_altitude_offset_m)
    return base_altitude


def _target_z_ned(vehicle: str, args) -> float:
    return -_target_altitude_m(vehicle, args)


def _move_px4_vehicle_to_local(
    client,
    airsim_module,
    vehicle_name: str,
    local_position: np.ndarray,
    args,
    *,
    label: str,
) -> None:
    target = np.asarray(local_position, dtype=float)
    speed = max(0.5, float(args.climb_speed))
    deadline = time.monotonic() + max(5.0, float(args.climb_timeout_s))
    command_dt = 0.50
    reached = False
    last_print = 0.0
    while time.monotonic() < deadline:
        _ensure_px4_api_control(client, args, vehicle_name)
        kin = _guidance_kinematics(client, vehicle_name, args)
        position = _vector_xyz(kin.position)
        velocity = _vector_xyz(kin.linear_velocity)
        error = target - position
        distance = float(np.linalg.norm(error))
        if distance <= 1.0 and float(np.linalg.norm(velocity)) <= max(0.2, float(args.settle_speed)):
            reached = True
            break
        direction = error / max(1.0e-6, distance)
        command = direction * min(speed, 1.5 * distance)
        command = _px4_limited_velocity(command, args)
        client.moveByVelocityAsync(
            float(command[0]),
            float(command[1]),
            float(command[2]),
            duration=command_dt,
            drivetrain=airsim_module.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim_module.YawMode(is_rate=True, yaw_or_rate=0.0),
            vehicle_name=vehicle_name,
        )
        for keepalive_vehicle in (args.interceptor, args.intruder):
            if keepalive_vehicle != vehicle_name and _is_px4_vehicle(keepalive_vehicle, args):
                client.moveByVelocityAsync(0.0, 0.0, 0.0, duration=command_dt, vehicle_name=keepalive_vehicle)
        now = time.monotonic()
        if now - last_print >= 2.0:
            print(
                f"px4_{label}_status vehicle={vehicle_name} pos={np.array2string(position, precision=2)} "
                f"target={np.array2string(target, precision=2)} dist={distance:.2f}"
            )
            last_print = now
        time.sleep(command_dt)
    if not reached:
        kin = _guidance_kinematics(client, vehicle_name, args)
        position = _vector_xyz(kin.position)
        raise SystemExit(
            f"PX4 vehicle failed to reach {label}: vehicle={vehicle_name}, "
            f"pos={np.array2string(position, precision=2)}, target={np.array2string(target, precision=2)}."
        )


def _prepare_px4_mixed_intercept_altitude(client, vehicles, target_z: dict[str, float], args, origins: dict[str, np.ndarray]) -> None:
    px4_vehicles = [vehicle for vehicle in vehicles if _is_px4_vehicle(vehicle, args)]
    simple_vehicles = [vehicle for vehicle in vehicles if not _is_px4_vehicle(vehicle, args)]
    print(f"Using PX4-friendly altitude preparation for: {_format_names(px4_vehicles)}")
    for vehicle in simple_vehicles:
        client.takeoffAsync(timeout_sec=args.climb_timeout_s, vehicle_name=vehicle).join()
        client.moveToZAsync(
            target_z[vehicle],
            velocity=args.climb_speed,
            timeout_sec=args.climb_timeout_s,
            vehicle_name=vehicle,
        ).join()
        client.hoverAsync(vehicle_name=vehicle).join()

    for vehicle in px4_vehicles:
        for _ in range(6):
            _ensure_px4_api_control(client, args, vehicle)
            client.moveByVelocityAsync(0.0, 0.0, 0.0, duration=0.2, vehicle_name=vehicle)
            time.sleep(0.1)

    climb_speed = max(0.3, abs(float(args.climb_speed)))
    settle_speed = max(0.1, float(args.settle_speed))
    deadline = time.monotonic() + max(5.0, float(args.climb_timeout_s))
    command_dt = 0.25
    last_print = 0.0
    reached: set[str] = set()
    while time.monotonic() < deadline:
        statuses = []
        for vehicle in px4_vehicles:
            if vehicle in reached:
                continue
            local_target_z = _px4_local_target_z(vehicle, target_z[vehicle], origins)
            kin = _guidance_kinematics(client, vehicle, args)
            position = _vector_xyz(kin.position)
            velocity = _vector_xyz(kin.linear_velocity)
            z_error = local_target_z - float(position[2])
            if abs(z_error) <= 1.0 and abs(float(velocity[2])) <= settle_speed:
                reached.add(vehicle)
                continue
            vz_cmd = float(np.clip(1.2 * z_error, -climb_speed, climb_speed))
            client.moveByVelocityAsync(
                0.0,
                0.0,
                vz_cmd,
                duration=command_dt,
                vehicle_name=vehicle,
            )
            statuses.append(
                f"{vehicle}: z={position[2]:.2f} target={local_target_z:.2f} "
                f"err={z_error:.2f} vz={velocity[2]:.2f} cmd={vz_cmd:.2f}"
            )
        if len(reached) == len(px4_vehicles):
            break
        now = time.monotonic()
        if statuses and now - last_print >= 2.0:
            print("px4_climb_status " + " | ".join(statuses))
            last_print = now
        time.sleep(command_dt)
    missing = [vehicle for vehicle in px4_vehicles if vehicle not in reached]
    if missing:
        details = []
        for vehicle in missing:
            kin = _guidance_kinematics(client, vehicle, args)
            position = _vector_xyz(kin.position)
            details.append(f"{vehicle}: z={position[2]:.2f}, target={_px4_local_target_z(vehicle, target_z[vehicle], origins):.2f}")
        raise SystemExit("PX4 vehicle failed to reach intercept altitude: " + "; ".join(details))

    for vehicle in simple_vehicles:
        client.hoverAsync(vehicle_name=vehicle)
    if args.settle_s > 0.0:
        time.sleep(args.settle_s)
    for _ in range(4):
        for vehicle in px4_vehicles:
            client.moveByVelocityAsync(0.0, 0.0, 0.0, duration=0.25, vehicle_name=vehicle)
        time.sleep(0.1)


def _prepare_intercept_altitude(client, vehicles: Sequence[str], args, origins: dict[str, np.ndarray]) -> None:
    if not args.climb_to_altitude:
        return
    target_z = {
        args.interceptor: -abs(float(args.intercept_altitude_m)),
        args.intruder: -(abs(float(args.intercept_altitude_m)) + float(args.intruder_altitude_offset_m)),
    }
    if args.intruder_actor:
        target_z.pop(args.intruder, None)
    target_text = ", ".join(
        f"{vehicle}: altitude={-target_z[vehicle]:.1f}m, NED_Z={target_z[vehicle]:.1f}"
        for vehicle in vehicles
    )
    print(f"Climbing vehicles to intercept start altitudes: {target_text}")
    if args.px4_interceptor or args.px4_intruder:
        _prepare_px4_mixed_intercept_altitude(client, vehicles, target_z, args, origins)
    else:
        for future in [client.takeoffAsync(timeout_sec=args.climb_timeout_s, vehicle_name=v) for v in vehicles]:
            future.join()
        for future in [
            client.moveToZAsync(target_z[v], velocity=args.climb_speed, timeout_sec=args.climb_timeout_s, vehicle_name=v)
            for v in vehicles
        ]:
            future.join()
        for vehicle in vehicles:
            client.hoverAsync(vehicle_name=vehicle).join()
        if args.settle_s > 0.0:
            time.sleep(args.settle_s)
        settle_start = time.monotonic()
        while time.monotonic() - settle_start < args.settle_timeout_s:
            speeds = [
                float(np.linalg.norm(_vector_xyz(_guidance_kinematics(client, vehicle, args).linear_velocity)))
                for vehicle in vehicles
            ]
            if speeds and max(speeds) <= args.settle_speed:
                break
            for future in [client.hoverAsync(vehicle_name=vehicle) for vehicle in vehicles]:
                future.join()
            time.sleep(0.2)
    print("Altitude preparation complete; starting gimbal vision loop.")


def _start_geometry_enabled(args) -> bool:
    return args.start_horizontal_range_m is not None or args.start_forward_offset_m is not None


def _local_start_z(vehicle: str, args) -> float:
    altitude = abs(float(args.intercept_altitude_m))
    if vehicle == args.intruder:
        altitude += float(args.intruder_altitude_offset_m)
    return -altitude


def _start_geometry_offsets(args) -> tuple[float, float, float]:
    lateral = float(args.start_lateral_offset_m)
    if args.start_horizontal_range_m is not None and args.start_forward_offset_m is not None:
        raise SystemExit("Use either --start-horizontal-range-m or --start-forward-offset-m, not both.")
    if args.start_horizontal_range_m is not None:
        horizontal_range = float(args.start_horizontal_range_m)
        if horizontal_range <= 0.0:
            raise SystemExit("--start-horizontal-range-m must be positive")
        if abs(lateral) > horizontal_range:
            raise SystemExit("--start-lateral-offset-m magnitude cannot exceed --start-horizontal-range-m")
        forward = float(np.sqrt(max(0.0, horizontal_range * horizontal_range - lateral * lateral)))
    elif args.start_forward_offset_m is not None:
        forward = float(args.start_forward_offset_m)
        horizontal_range = float(np.hypot(forward, lateral))
    else:
        forward = 0.0
        horizontal_range = 0.0
    return forward, lateral, horizontal_range


def _interceptor_start_local(args, origins: dict[str, np.ndarray]) -> np.ndarray:
    local = np.array(
        [float(args.start_interceptor_x_m), float(args.start_interceptor_y_m), _local_start_z(args.interceptor, args)],
        dtype=float,
    )
    if args.px4_interceptor:
        local[2] = -abs(float(args.intercept_altitude_m)) - origins.get(args.interceptor, np.zeros(3, dtype=float))[2]
    return local


def _apply_start_geometry(client, airsim_module, args, origins: dict[str, np.ndarray]) -> None:
    if not _start_geometry_enabled(args):
        return

    forward, lateral, horizontal_range = _start_geometry_offsets(args)
    if args.px4_interceptor:
        interceptor_kin = _guidance_kinematics(client, args.interceptor, args)
        interceptor_local = _vector_xyz(interceptor_kin.position)
        interceptor_world = _world_position(interceptor_kin, args.interceptor, origins)
        state = client.getMultirotorState(vehicle_name=args.interceptor)
        R_IB = airsim_orientation_to_R_IB(state.kinematics_estimated.orientation)
        forward_axis = np.array([R_IB[0, 0], R_IB[1, 0], 0.0], dtype=float)
        right_axis = np.array([R_IB[0, 1], R_IB[1, 1], 0.0], dtype=float)
        forward_norm = float(np.linalg.norm(forward_axis))
        right_norm = float(np.linalg.norm(right_axis))
        forward_axis = forward_axis / forward_norm if forward_norm > 1.0e-6 else np.array([1.0, 0.0, 0.0])
        right_axis = right_axis / right_norm if right_norm > 1.0e-6 else np.array([0.0, 1.0, 0.0])
        intruder_world = interceptor_world + forward * forward_axis + lateral * right_axis
        intruder_world[2] = interceptor_world[2] - float(args.intruder_altitude_offset_m)
    else:
        interceptor_local = _interceptor_start_local(args, origins)
        interceptor_world = origins.get(args.interceptor, np.zeros(3, dtype=float)) + interceptor_local
        intruder_world = interceptor_world + np.array([forward, lateral, -float(args.intruder_altitude_offset_m)], dtype=float)
    intruder_local = intruder_world - origins.get(args.intruder, np.zeros(3, dtype=float))

    if args.intruder_actor:
        _spawn_or_move_intruder_actor(client, airsim_module, args, intruder_world, _yaw_deg_from_velocity(_intruder_velocity(args)))
    else:
        if _is_px4_vehicle(args.intruder, args):
            _move_px4_vehicle_to_local(client, airsim_module, args.intruder, intruder_local, args, label="start")
        else:
            client.moveToPositionAsync(
                float(intruder_local[0]),
                float(intruder_local[1]),
                float(intruder_local[2]),
                velocity=max(0.5, float(args.climb_speed)),
                timeout_sec=max(1.0, float(args.climb_timeout_s)),
                vehicle_name=args.intruder,
            ).join()
            client.hoverAsync(vehicle_name=args.intruder).join()

    if not args.px4_interceptor:
        client.moveToPositionAsync(
            float(interceptor_local[0]),
            float(interceptor_local[1]),
            float(interceptor_local[2]),
            velocity=max(0.5, float(args.climb_speed)),
            timeout_sec=max(1.0, float(args.climb_timeout_s)),
            vehicle_name=args.interceptor,
        ).join()
        client.hoverAsync(vehicle_name=args.interceptor).join()
    intruder_yaw_deg = _yaw_deg_from_velocity(_intruder_velocity(args))
    if args.intruder_actor:
        pass
    elif _is_px4_vehicle(args.intruder, args):
        _command_vehicle_velocity(client, airsim_module, args.intruder, np.zeros(3), 0.0, 0.5, args)
    else:
        client.rotateToYawAsync(
            intruder_yaw_deg,
            timeout_sec=max(1.0, float(args.climb_timeout_s)),
            margin=3.0,
            vehicle_name=args.intruder,
        ).join()

    if args.start_geometry_settle_s > 0.0:
        time.sleep(float(args.start_geometry_settle_s))
    print(
        "Applied gimbal start geometry: "
        f"horizontal_range={horizontal_range:.2f}m, forward={forward:.2f}m, lateral={lateral:.2f}m, "
        f"altitude_offset={float(args.intruder_altitude_offset_m):.2f}m, "
        f"interceptor_local={np.array2string(interceptor_local, precision=2)}, "
        f"intruder_local={np.array2string(intruder_local, precision=2)}"
    )


def _camera_offset_body(args) -> np.ndarray:
    return np.array([args.camera_x, args.camera_y, args.camera_z], dtype=float)


def _camera_pose(airsim_module, yaw_rad: float, pitch_rad: float, args):
    return airsim_module.Pose(
        airsim_module.Vector3r(args.camera_x, args.camera_y, args.camera_z),
        airsim_module.to_quaternion(-pitch_rad, 0.0, yaw_rad),
    )


def _update_gimbal_from_pixel(
    yaw_rad: float,
    pitch_rad: float,
    center: tuple[float, float],
    intrinsics,
    dt: float,
    args,
    gain_scale: float = 1.0,
) -> tuple[float, float, float, float]:
    u, v = center
    yaw_error = float(np.arctan2(u - intrinsics.cx, intrinsics.fx))
    pitch_error = float(np.arctan2(v - intrinsics.cy, intrinsics.fy))
    max_step = np.deg2rad(args.gimbal_rate_limit_deg) * dt
    scale = max(0.0, float(gain_scale))
    yaw_step = float(np.clip(scale * args.gimbal_yaw_gain * yaw_error, -max_step, max_step))
    pitch_step = float(np.clip(scale * args.gimbal_pitch_gain * pitch_error, -max_step, max_step))
    yaw_limit = np.deg2rad(args.gimbal_yaw_limit_deg)
    pitch_limit = np.deg2rad(args.gimbal_pitch_limit_deg)
    yaw_rad = float(np.clip(yaw_rad + yaw_step, -yaw_limit, yaw_limit))
    pitch_rad = float(np.clip(pitch_rad + pitch_step, -pitch_limit, pitch_limit))
    return yaw_rad, pitch_rad, yaw_error, pitch_error


def _gimbal_update_profile(
    *,
    detected: bool,
    terminal_state: str,
    terminal_reason: str,
    using_blind_push: bool,
    args,
) -> tuple[bool, float, str]:
    if not detected:
        return False, 0.0, "no_detection"
    if _is_bbox_clipped_reason(terminal_reason):
        return False, 0.0, f"{terminal_reason}_hold"
    if using_blind_push and args.terminal_freeze_gimbal_on_blind:
        return False, 0.0, "blind_push_hold"
    if terminal_state == TERMINAL_VISUAL or using_blind_push:
        return True, max(0.0, float(args.terminal_gimbal_gain_scale)), "terminal_scaled"
    return True, 1.0, "tracking"


def _terminal_image_kf_config_from_args(args) -> TerminalImageKFConfig:
    return TerminalImageKFConfig(
        enable=bool(args.terminal_image_kf),
        max_predict_s=max(0.0, float(args.terminal_image_kf_max_predict_s)),
        measurement_noise_rad=max(1.0e-6, float(args.terminal_image_kf_meas_noise_rad)),
        accel_noise_rad_s2=max(0.0, float(args.terminal_image_kf_accel_noise_rad_s2)),
        innovation_reject_rad=max(1.0e-6, float(args.terminal_image_kf_innovation_reject_rad)),
        max_angle_rad=max(1.0e-6, float(args.terminal_image_kf_max_angle_rad)),
        max_rate_rad_s=max(1.0e-6, float(args.terminal_image_kf_max_rate_rad_s)),
    )


def _image_kf_takeover_allowed(
    image_kf: TerminalImageEstimate,
    terminal_result,
    reason: str,
    detected: bool,
    area_ratio: float,
    args,
    *,
    profile: str,
) -> tuple[bool, str]:
    if not image_kf.valid:
        return False, "image_kf_invalid"
    if image_kf.mode != IMAGE_KF_PREDICT:
        return False, "image_kf_not_predict"

    terminal_reasons = {"bbox_area_large", "terminal_lost", "gimbal_limit"}
    if terminal_result.using_blind_push:
        return True, "blind_push"
    if terminal_result.state in {TERMINAL_VISUAL, BLIND_PUSH}:
        return True, "terminal_state"
    if terminal_result.reason in terminal_reasons or _is_bbox_clipped_reason(terminal_result.reason):
        return True, terminal_result.reason
    if _is_bbox_clipped_reason(reason) or _is_short_visual_loss_reason(reason):
        return True, reason
    if profile == "strapdown" and reason == "los_innovation_reject":
        if area_ratio >= max(0.0, float(args.terminal_soft_enter_area_ratio)):
            return True, "strapdown_los_reject"
    if not detected and area_ratio >= max(0.0, float(args.terminal_soft_enter_area_ratio)):
        return True, "soft_terminal_loss"
    return False, "not_terminal"


def _is_short_visual_loss_reason(reason: str) -> bool:
    text = str(reason or "")
    return (
        text == "no_detection"
        or text == "los_innovation_reject"
        or text.endswith("_missing")
        or text.startswith("yolo_")
    )


def _empty_image_kf_estimate(timestamp: float) -> TerminalImageEstimate:
    return TerminalImageEstimate(
        timestamp=timestamp,
        theta_x=0.0,
        theta_y=0.0,
        theta_dot_x=0.0,
        theta_dot_y=0.0,
        valid=False,
        mode="unavailable",
        age_s=0.0,
        quality=0.0,
        reject_reason="",
    )


def _gimbal_from_relative_body(relative_body: np.ndarray, args) -> tuple[float, float]:
    rel = np.asarray(relative_body, dtype=float)
    horizontal = float(np.hypot(rel[0], rel[1]))
    yaw = float(np.arctan2(rel[1], rel[0]))
    pitch = float(np.arctan2(rel[2], max(horizontal, 1.0e-6)))
    yaw = float(np.clip(yaw, -np.deg2rad(args.gimbal_yaw_limit_deg), np.deg2rad(args.gimbal_yaw_limit_deg)))
    pitch = float(np.clip(pitch, -np.deg2rad(args.gimbal_pitch_limit_deg), np.deg2rad(args.gimbal_pitch_limit_deg)))
    return yaw, pitch


def _initial_truth_align_gimbal(client, args, origins: dict[str, np.ndarray]) -> tuple[float, float]:
    interceptor_kin = _guidance_kinematics(client, args.interceptor, args)
    interceptor_pos = _world_position(interceptor_kin, args.interceptor, origins)
    intruder_pos = _intruder_truth_position(client, args, origins)
    state = client.getMultirotorState(vehicle_name=args.interceptor)
    R_IB = airsim_orientation_to_R_IB(state.kinematics_estimated.orientation)
    camera_pos = interceptor_pos + R_IB @ _camera_offset_body(args)
    relative_body = R_IB.T @ (intruder_pos - camera_pos)
    yaw_rad, pitch_rad = _gimbal_from_relative_body(relative_body, args)
    print(
        "Initial truth gimbal alignment: "
        f"camera_offset_body={np.array2string(_camera_offset_body(args), precision=2)}, "
        f"relative_body={np.array2string(relative_body, precision=2)}, "
        f"yaw={np.rad2deg(yaw_rad):.2f}deg, pitch={np.rad2deg(pitch_rad):.2f}deg"
    )
    return yaw_rad, pitch_rad


def _guidance_velocity(
    own_velocity: np.ndarray,
    lambda_I: Optional[np.ndarray],
    omega_los: Optional[np.ndarray],
    gain: float,
    speed_cap: float,
    args,
) -> np.ndarray:
    if lambda_I is None:
        forward = np.array([1.0, 0.0, 0.0], dtype=float)
    else:
        los = np.asarray(lambda_I, dtype=float)
        if float(np.linalg.norm(los)) <= 1.0e-6:
            forward = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            forward = normalize(los)
    base = speed_cap * forward
    if omega_los is None:
        return base

    correction = gain * np.cross(omega_los, lambda_I)
    lateral = np.array([correction[0], correction[1], 0.0], dtype=float)
    lateral_norm = float(np.linalg.norm(lateral))
    if lateral_norm > args.max_vision_lateral_speed:
        lateral *= args.max_vision_lateral_speed / lateral_norm
    vertical = float(np.clip(correction[2], -args.max_vision_vertical_speed, args.max_vision_vertical_speed))
    command = base + lateral
    command[2] = float(np.clip(command[2] + vertical, -speed_cap, speed_cap))
    norm = float(np.linalg.norm(command))
    if norm > speed_cap and norm > 1.0e-6:
        command *= speed_cap / norm
    return command


def _yaw_deg_from_velocity(velocity: np.ndarray) -> float:
    horizontal_speed = float(np.hypot(velocity[0], velocity[1]))
    if horizontal_speed <= 1.0e-6:
        return 0.0
    return float(np.rad2deg(np.arctan2(velocity[1], velocity[0])))


def _wrap_angle_deg(angle_deg: float) -> float:
    return float((angle_deg + 180.0) % 360.0 - 180.0)


def _body_yaw_deg(airsim_module, orientation) -> float:
    return float(np.rad2deg(airsim_module.to_eularian_angles(orientation)[2]))


def _bearing_deg_from_xy(vector_xy: np.ndarray) -> float:
    return float(np.rad2deg(np.arctan2(float(vector_xy[1]), float(vector_xy[0]))))


def _air_sim_yaw_mode(airsim_module, velocity: np.ndarray, args):
    if not args.yaw_control:
        return airsim_module.DrivetrainType.MaxDegreeOfFreedom, airsim_module.YawMode(is_rate=True, yaw_or_rate=0.0)
    return airsim_module.DrivetrainType.ForwardOnly, airsim_module.YawMode(
        is_rate=False,
        yaw_or_rate=_yaw_deg_from_velocity(velocity),
    )


def _airsim_safety_ok(client, vehicle_name: str) -> bool:
    try:
        return bool(client.isApiControlEnabled(vehicle_name=vehicle_name))
    except Exception:
        return True


def _near_gimbal_limit(yaw_rad: float, pitch_rad: float, args) -> bool:
    margin = np.deg2rad(max(0.0, args.gimbal_limit_margin_deg))
    yaw_limit = np.deg2rad(args.gimbal_yaw_limit_deg)
    pitch_limit = np.deg2rad(args.gimbal_pitch_limit_deg)
    return abs(yaw_rad) >= yaw_limit - margin or abs(pitch_rad) >= pitch_limit - margin


def _terminal_area(frame_area: float, intrinsics, args) -> bool:
    image_area = max(1.0, float(intrinsics.width * intrinsics.height))
    return frame_area >= max(0.0, args.terminal_bbox_area_ratio) * image_area


def _terminal_trigger(reason: str, detected: bool, bbox_area: float, yaw_rad: float, pitch_rad: float, intrinsics, args) -> str:
    if _is_bbox_clipped_reason(reason):
        return reason
    if detected and _terminal_area(bbox_area, intrinsics, args):
        return "bbox_area_large"
    if detected and _near_gimbal_limit(yaw_rad, pitch_rad, args) and _terminal_area(
        2.0 * bbox_area,
        intrinsics,
        args,
    ):
        return "gimbal_limit"
    return ""


def _is_bbox_clipped_reason(reason: str) -> bool:
    return str(reason or "").startswith("bbox_") and str(reason or "").endswith("_clipped")


def _command_duration(loop_dt: float, target_dt: float, args) -> float:
    requested = max(loop_dt + args.command_duration_margin_s, target_dt, args.min_command_duration_s)
    return float(np.clip(requested, args.min_command_duration_s, args.max_command_duration_s))


def _los_fallback_allowed(reason: str, args) -> bool:
    if not args.allow_los_fallback:
        return False
    return reason in {"ttc_out_of_range", "area_not_expanding", "ttc_invalid"}


def _terminal_config_from_args(args) -> TerminalConfig:
    return TerminalConfig(
        enable=bool(args.terminal_extrapolation),
        terminal_enter_area_ratio=max(0.0, float(args.terminal_enter_area_ratio)),
        soft_enter_area_ratio=max(0.0, float(args.terminal_soft_enter_area_ratio)),
        cutoff_area_ratio=max(0.0, float(args.terminal_cutoff_area_ratio)),
        terminal_gimbal_limit_area_ratio=max(0.0, float(args.terminal_gimbal_limit_area_ratio)),
        cutoff_miss_count=max(1, int(args.terminal_cutoff_miss_count)),
        min_tracking_time_s=max(0.0, float(args.terminal_min_tracking_time_s)),
        confidence_min_score=max(0.0, float(args.terminal_confidence_min_score)),
        max_measurement_age_s=max(0.0, float(args.terminal_max_measurement_age_s)),
        blind_duration_s=max(0.0, float(args.terminal_blind_duration_s)),
        command_average_window_s=max(0.01, float(args.terminal_command_average_window_s)),
        command_decay_tau_s=max(1.0e-6, float(args.terminal_command_decay_tau_s)),
        trend_bias_gain=max(0.0, float(args.terminal_trend_bias_gain)),
        trend_bias_max_mps=max(0.0, float(args.terminal_trend_bias_max_mps)),
        pitch_up_bias_mps=max(0.0, float(args.terminal_pitch_up_bias_mps)),
        abort_on_tilt_hardcap=bool(args.terminal_abort_on_tilt_hardcap),
    )


def _experiment_fields(
    *,
    args,
    experiment_type: str,
    speed_cap: float,
    intruder_velocity_cmd: np.ndarray,
    intrinsics,
) -> dict[str, float | int | str]:
    return {
        "experiment_type": experiment_type,
        "speed_ratio": float(args.speed_ratio),
        "intruder_speed": float(args.intruder_speed),
        "intruder_speed_arg": float(args.intruder_speed),
        "intruder_vx": float(intruder_velocity_cmd[0]),
        "intruder_vy": float(intruder_velocity_cmd[1]),
        "intruder_vz": float(intruder_velocity_cmd[2]),
        "speed_cap": float(speed_cap),
        "intercept_altitude_m": float(args.intercept_altitude_m),
        "intruder_altitude_offset_m": float(args.intruder_altitude_offset_m),
        "start_horizontal_range_m": "" if getattr(args, "start_horizontal_range_m", None) is None else float(args.start_horizontal_range_m),
        "start_forward_offset_m": "" if getattr(args, "start_forward_offset_m", None) is None else float(args.start_forward_offset_m),
        "start_lateral_offset_m": float(getattr(args, "start_lateral_offset_m", 0.0)),
        "camera_name": str(args.camera),
        "camera_x": float(args.camera_x),
        "camera_y": float(args.camera_y),
        "camera_z": float(args.camera_z),
        "camera_pitch_deg": float(getattr(args, "camera_pitch_deg", 0.0)),
        "camera_roll_deg": float(getattr(args, "camera_roll_deg", 0.0)),
        "camera_yaw_deg": float(getattr(args, "camera_yaw_deg", 0.0)),
        "fov_deg": float(args.fov_deg),
        "image_width_config": int(args.width),
        "image_height_config": int(args.height),
        "image_width_runtime": int(intrinsics.width),
        "image_height_runtime": int(intrinsics.height),
        "rate_hz": float(args.rate_hz),
        "duration_s": float(args.duration_s),
        "terminal_enter_area_ratio": float(args.terminal_enter_area_ratio),
        "terminal_soft_enter_area_ratio": float(args.terminal_soft_enter_area_ratio),
        "terminal_cutoff_area_ratio": float(args.terminal_cutoff_area_ratio),
        "terminal_gimbal_limit_area_ratio": float(args.terminal_gimbal_limit_area_ratio),
        "terminal_image_kf_max_predict_s": float(args.terminal_image_kf_max_predict_s),
        "terminal_blind_duration_s": float(args.terminal_blind_duration_s),
        "terminal_command_average_window_s": float(args.terminal_command_average_window_s),
        "terminal_pitch_up_bias_mps": float(args.terminal_pitch_up_bias_mps),
        "reject_top_clipped_pitch": int(bool(getattr(args, "reject_top_clipped_pitch", False))),
        "intruder_actor": int(bool(getattr(args, "intruder_actor", False))),
        "intruder_actor_name": str(getattr(args, "intruder_actor_name", "")),
        "intruder_actor_asset": str(getattr(args, "intruder_actor_asset", "")),
        "intruder_actor_scale": float(getattr(args, "intruder_actor_scale", 0.0)),
        "px4_interceptor": int(bool(getattr(args, "px4_interceptor", False))),
        "px4_intruder": int(bool(getattr(args, "px4_intruder", False))),
        "px4_max_vertical_speed": float(getattr(args, "px4_max_vertical_speed", 0.0)),
        "px4_command_join": int(bool(getattr(args, "px4_command_join", False))),
        "px4_command_mode": str(getattr(args, "px4_command_mode", "")),
        "bbox_noise_enabled": int(bool(getattr(args, "bbox_noise", False))),
        "bbox_noise_center_sigma_px": float(getattr(args, "bbox_center_noise_px", 0.0)),
        "bbox_noise_area_sigma_ratio": float(getattr(args, "bbox_area_noise_ratio", 0.0)),
        "bbox_noise_seed": int(getattr(args, "bbox_noise_seed", 0)),
        "los_filter_enabled": int(bool(getattr(args, "los_filter", True))),
        "guidance_law": str(getattr(args, "guidance_law", "ttc_png")),
        "guidance_output_mode": str(getattr(args, "guidance_output_mode", "velocity_bias")),
        "navigation_constant": float(getattr(args, "navigation_constant", 0.0)),
        "max_guidance_accel_mps2": float(getattr(args, "max_guidance_accel_mps2", 0.0)),
        "min_speed_ratio": float(getattr(args, "min_speed_ratio", 0.0)),
        "accel_integral_reset_on_invalid": int(bool(getattr(args, "accel_integral_reset_on_invalid", False))),
        "body_rate_max_tilt_deg": float(getattr(args, "body_rate_max_tilt_deg", 0.0)),
        "body_rate_roll_gain": float(getattr(args, "body_rate_roll_gain", 0.0)),
        "body_rate_pitch_gain": float(getattr(args, "body_rate_pitch_gain", 0.0)),
        "body_rate_attitude_p": float(getattr(args, "body_rate_attitude_p", 0.0)),
        "body_rate_max_roll_rate_deg": float(getattr(args, "body_rate_max_roll_rate_deg", 0.0)),
        "body_rate_max_pitch_rate_deg": float(getattr(args, "body_rate_max_pitch_rate_deg", 0.0)),
        "body_rate_hover_thrust": float(getattr(args, "body_rate_hover_thrust", 0.0)),
        "body_rate_thrust_gain": float(getattr(args, "body_rate_thrust_gain", 0.0)),
        "body_rate_min_thrust": float(getattr(args, "body_rate_min_thrust", 0.0)),
        "body_rate_max_thrust": float(getattr(args, "body_rate_max_thrust", 0.0)),
        "body_rate_speed_hold_gain": float(getattr(args, "body_rate_speed_hold_gain", 0.0)),
        "body_rate_speed_hold_max_accel_mps2": float(getattr(args, "body_rate_speed_hold_max_accel_mps2", 0.0)),
        "body_rate_total_accel_limit_mps2": float(getattr(args, "body_rate_total_accel_limit_mps2", 0.0)),
        "detector_source": str(getattr(args, "detector_source", "airsim")),
        "yolo_model": str(getattr(args, "yolo_model", "")),
        "yolo_class_id": "" if getattr(args, "yolo_class_id", None) is None else int(args.yolo_class_id),
        "yolo_conf": float(getattr(args, "yolo_conf", 0.0)),
        "yolo_iou": float(getattr(args, "yolo_iou", 0.0)),
        "yolo_imgsz": int(getattr(args, "yolo_imgsz", 0)),
        "yolo_device": str(getattr(args, "yolo_device", "")),
        "yolo_tracker": str(getattr(args, "yolo_tracker", "")),
    }


def _json_safe(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    try:
        json.dumps(value)
    except TypeError:
        return str(value)
    return value


def _finite_row_float(row: dict[str, float | int | str], key: str) -> Optional[float]:
    try:
        value = row.get(key, "")
        if value == "":
            return None
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if np.isfinite(number) else None


def _write_run_metadata(
    *,
    args,
    csv_path: Path,
    script_name: str,
    experiment_type: str,
    intrinsics,
    speed_cap: float,
    intruder_velocity_cmd: np.ndarray,
    rows: Sequence[dict[str, float | int | str]],
    hit: bool,
) -> Path:
    ranges = [value for row in rows if (value := _finite_row_float(row, "range")) is not None]
    wall_fps_values = [value for row in rows if (value := _finite_row_float(row, "wall_fps")) is not None and value > 0.0]
    sim_fps_values = [value for row in rows if (value := _finite_row_float(row, "sim_sample_fps")) is not None and value > 0.0]
    sim_clock_ratios = [value for row in rows if (value := _finite_row_float(row, "sim_clock_ratio")) is not None and value > 0.0]
    detector_fps_values = [value for row in rows if (value := _finite_row_float(row, "detector_fps")) is not None and value > 0.0]
    load_factors = [value for row in rows if (value := _finite_row_float(row, "load_factor_g")) is not None]
    load_factors_fd = [value for row in rows if (value := _finite_row_float(row, "load_factor_fd_g")) is not None]
    meta = {
        "script_name": script_name,
        "experiment_type": experiment_type,
        "created_local_time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "csv_path": str(csv_path),
        "settings_path": str(args.settings_path),
        "vehicle_names": {"interceptor": args.interceptor, "intruder": args.intruder},
        "args": {key: value for key, value in vars(args).items() if not str(key).startswith("_")},
        "intrinsics": {
            "fx": float(intrinsics.fx),
            "fy": float(intrinsics.fy),
            "cx": float(intrinsics.cx),
            "cy": float(intrinsics.cy),
            "width": int(intrinsics.width),
            "height": int(intrinsics.height),
        },
        "derived": {
            "speed_cap": float(speed_cap),
            "intruder_velocity_cmd": [float(value) for value in intruder_velocity_cmd],
            "frame_count": len(rows),
            "hit": bool(hit),
            "min_range_m": min(ranges) if ranges else None,
            "final_range_m": ranges[-1] if ranges else None,
            "avg_wall_fps": float(np.mean(wall_fps_values)) if wall_fps_values else None,
            "avg_sim_sample_fps": float(np.mean(sim_fps_values)) if sim_fps_values else None,
            "avg_sim_clock_ratio": float(np.mean(sim_clock_ratios)) if sim_clock_ratios else None,
            "avg_detector_fps": float(np.mean(detector_fps_values)) if detector_fps_values else None,
            "p50_detector_fps": float(np.percentile(detector_fps_values, 50)) if detector_fps_values else None,
            "avg_load_factor_g": float(np.mean(load_factors)) if load_factors else None,
            "max_load_factor_g": max(load_factors) if load_factors else None,
            "avg_load_factor_fd_g": float(np.mean(load_factors_fd)) if load_factors_fd else None,
            "max_load_factor_fd_g": max(load_factors_fd) if load_factors_fd else None,
        },
    }
    metadata_path = csv_path.with_name(f"{csv_path.stem}_meta.json")
    with metadata_path.open("w", encoding="utf-8") as stream:
        json.dump(_json_safe(meta), stream, ensure_ascii=False, indent=2, sort_keys=True)
    return metadata_path


def _summarize_run(rows: Sequence[dict[str, float | int | str]]) -> None:
    if len(rows) < 2:
        return

    def finite_float(row, key: str) -> Optional[float]:
        try:
            value = float(row.get(key, ""))
        except (TypeError, ValueError):
            return None
        return value if np.isfinite(value) else None

    elapsed = finite_float(rows[-1], "t")
    if elapsed is None or elapsed <= 0.0:
        return
    avg_hz = (len(rows) - 1) / elapsed
    detected = sum(int(row.get("detected", 0)) == 1 for row in rows)
    valid = sum(int(row.get("valid", 0)) == 1 for row in rows)
    fallback = sum(row.get("guidance_mode") == "los_fallback" for row in rows)
    ranges = [value for row in rows if (value := finite_float(row, "range")) is not None]
    wall_fps_values = [value for row in rows if (value := finite_float(row, "wall_fps")) is not None and value > 0.0]
    sim_fps_values = [value for row in rows if (value := finite_float(row, "sim_sample_fps")) is not None and value > 0.0]
    detector_fps_values = [value for row in rows if (value := finite_float(row, "detector_fps")) is not None and value > 0.0]
    load_factors = [value for row in rows if (value := finite_float(row, "load_factor_g")) is not None]
    load_factors_fd = [value for row in rows if (value := finite_float(row, "load_factor_fd_g")) is not None]
    final_range = ranges[-1] if ranges else None
    min_range = min(ranges) if ranges else None

    def average_speed(prefix: str) -> Optional[float]:
        coords = []
        for key in ("x", "y", "z"):
            start_value = finite_float(rows[0], f"{prefix}_{key}")
            end_value = finite_float(rows[-1], f"{prefix}_{key}")
            if start_value is None or end_value is None:
                return None
            coords.append(end_value - start_value)
        return float(np.linalg.norm(np.asarray(coords, dtype=float)) / elapsed)

    interceptor_speed = average_speed("interceptor")
    intruder_speed = average_speed("intruder")
    print(
        "run_summary="
        f"frames={len(rows)}, avg_hz={avg_hz:.2f}, detected={detected}/{len(rows)}, "
        f"valid={valid}/{len(rows)}, los_fallback={fallback}/{len(rows)}, "
        f"min_range={min_range if min_range is not None else float('nan'):.3f}, "
        f"final_range={final_range if final_range is not None else float('nan'):.3f}, "
        f"interceptor_avg_speed={interceptor_speed if interceptor_speed is not None else float('nan'):.3f}, "
        f"intruder_avg_speed={intruder_speed if intruder_speed is not None else float('nan'):.3f}, "
        f"avg_wall_fps={float(np.mean(wall_fps_values)) if wall_fps_values else float('nan'):.2f}, "
        f"avg_sim_fps={float(np.mean(sim_fps_values)) if sim_fps_values else float('nan'):.2f}, "
        f"avg_detector_fps={float(np.mean(detector_fps_values)) if detector_fps_values else float('nan'):.2f}, "
        f"p50_detector_fps={float(np.percentile(detector_fps_values, 50)) if detector_fps_values else float('nan'):.2f}, "
        f"max_load_g={max(load_factors) if load_factors else float('nan'):.2f}, "
        f"max_load_fd_g={max(load_factors_fd) if load_factors_fd else float('nan'):.2f}"
    )


def _make_display(airsim_module, enabled: bool):
    if not enabled:
        return None
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("No DISPLAY/WAYLAND_DISPLAY found; continuing without preview window.")
        return None
    try:
        import cv2
    except Exception as exc:
        print(f"OpenCV display unavailable ({exc}); continuing without preview window.")
        return None
    try:
        cv2.namedWindow("Gimbal Vision PNG", cv2.WINDOW_NORMAL)
    except Exception as exc:
        print(f"OpenCV window unavailable ({exc}); continuing without preview window.")
        return None
    print("OpenCV preview window enabled: Gimbal Vision PNG")
    return {"cv2": cv2, "airsim": airsim_module, "failed": False, "image_failures": 0}


def _make_preview_recorder(airsim_module, args, default_dir: Path):
    if not getattr(args, "record_preview", False):
        return None
    try:
        import cv2
    except Exception as exc:
        print(f"OpenCV preview recorder unavailable ({exc}); continuing without saved preview frames.")
        return None
    preview_dir_arg = str(getattr(args, "preview_dir", "") or "").strip()
    preview_dir = Path(preview_dir_arg).expanduser() if preview_dir_arg else default_dir
    preview_dir.mkdir(parents=True, exist_ok=True)
    print(f"preview_recording_dir={preview_dir}")
    return {
        "cv2": cv2,
        "airsim": airsim_module,
        "dir": preview_dir,
        "saved": 0,
        "failed": False,
        "image_failures": 0,
    }


def _decode_airsim_image(cv2, raw_image):
    if raw_image is None:
        return None
    if isinstance(raw_image, str):
        raw_image = raw_image.encode("latin1")
    return cv2.imdecode(np.frombuffer(raw_image, dtype=np.uint8), cv2.IMREAD_UNCHANGED)


def _annotated_detection_image(
    *,
    cv2,
    airsim_module,
    client,
    config: AirSimDetectionConfig,
    detections,
    selected,
    intrinsics,
    lines: Sequence[str],
    window_scale: float,
    image_bgr=None,
):
    try:
        if image_bgr is None:
            raw_image = client.simGetImage(
                config.camera_name,
                getattr(airsim_module.ImageType, config.image_type_name),
                vehicle_name=config.vehicle_name,
            )
            if raw_image is None:
                return None
            image = _decode_airsim_image(cv2, raw_image)
            if image is None:
                return None
        else:
            image = np.array(image_bgr, copy=True)
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        cx = int(round(intrinsics.cx))
        cy = int(round(intrinsics.cy))
        cv2.line(image, (cx - 16, cy), (cx + 16, cy), (255, 255, 255), 1)
        cv2.line(image, (cx, cy - 16), (cx, cy + 16), (255, 255, 255), 1)

        for detection in detections:
            box = detection.box2D
            color = (0, 255, 0) if detection is selected else (255, 0, 0)
            p1 = (int(box.min.x_val), int(box.min.y_val))
            p2 = (int(box.max.x_val), int(box.max.y_val))
            cv2.rectangle(image, p1, p2, color, 2)
            cv2.putText(image, getattr(detection, "name", "target"), (p1[0], max(15, p1[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        if lines:
            overlay_height = min(image.shape[0] - 2, 12 + 20 * len(lines))
            overlay_width = min(image.shape[1] - 2, 630)
            cv2.rectangle(image, (4, 4), (overlay_width, overlay_height), (0, 0, 0), -1)
        for i, line in enumerate(lines):
            cv2.putText(image, line, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (36, 255, 12), 1)

        if window_scale > 0.0 and abs(window_scale - 1.0) > 1.0e-6:
            image = cv2.resize(image, None, fx=window_scale, fy=window_scale)
        return image
    except Exception:
        return None


def _preview_lines(
    *,
    profile: str,
    detections,
    selected,
    valid: bool,
    reason: str,
    ttc_text: str,
    terminal_state: str,
    image_kf_mode: str,
    range_m: float,
    control_line: str,
    hit: bool,
    include_quit: bool = False,
) -> list[str]:
    lines = [
        control_line,
        f"detections={len(detections)} selected={getattr(selected, 'name', '-') if selected is not None else '-'}",
        f"valid={valid} reason={reason or 'ok'} ttc={ttc_text or '-'}",
        f"terminal={terminal_state or '-'} image_kf={image_kf_mode or '-'} hit={int(hit)}",
        f"profile={profile} range={range_m:.2f}m" if np.isfinite(range_m) else f"profile={profile} range=-",
    ]
    if include_quit:
        lines.append("q: quit")
    return lines


def _sanitize_preview_token(value: str) -> str:
    token = "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in str(value or ""))
    return token.strip("_") or "none"


def _record_detection_preview(
    recorder,
    client,
    config: AirSimDetectionConfig,
    detections,
    selected,
    intrinsics,
    lines: Sequence[str],
    args,
    *,
    frame_id: int,
    terminal_state: str,
    guidance_mode: str,
    reason: str,
    hit: bool,
    image_bgr=None,
    timestamp: Optional[float] = None,
):
    if recorder is None or recorder.get("failed"):
        return None
    max_frames = max(0, int(getattr(args, "preview_max_frames", 0)))
    if max_frames <= 0 or int(recorder.get("saved", 0)) >= max_frames:
        return None

    near_terminal = bool(
        hit
        or terminal_state in {TERMINAL_VISUAL, BLIND_PUSH, COMPLETE, ABORT_HOLD}
        or _is_bbox_clipped_reason(reason)
        or reason in {"bbox_area_large", "gimbal_limit", "terminal_lost"}
    )
    preview_every_s = max(0.0, float(getattr(args, "preview_every_s", 0.0) or 0.0))
    if preview_every_s > 0.0 and timestamp is not None:
        preview_bucket = int(np.floor(float(timestamp) / preview_every_s))
        if not hit and recorder.get("last_preview_bucket") == preview_bucket:
            return None
    else:
        if getattr(args, "preview_near_terminal_only", True) and not near_terminal:
            return None
        every_n = max(1, int(getattr(args, "preview_every_n", 1)))
        if not near_terminal and frame_id % every_n != 0:
            return None

    cv2 = recorder["cv2"]
    image = _annotated_detection_image(
        cv2=cv2,
        airsim_module=recorder["airsim"],
        client=client,
        config=config,
        detections=detections,
        selected=selected,
        intrinsics=intrinsics,
        lines=lines,
        window_scale=1.0,
        image_bgr=image_bgr,
    )
    if image is None:
        recorder["image_failures"] += 1
        if recorder["image_failures"] <= 3:
            print("Preview recorder: failed to capture or decode AirSim image.")
        return None

    name = (
        f"{frame_id:05d}_"
        f"{_sanitize_preview_token(terminal_state)}_"
        f"{_sanitize_preview_token(guidance_mode)}_"
        f"{_sanitize_preview_token(reason)}.png"
    )
    output_path = Path(recorder["dir"]) / name
    ok = cv2.imwrite(str(output_path), image)
    if not ok:
        recorder["failed"] = True
        print(f"Preview recorder failed to write {output_path}; disabling preview recording.")
        return None
    recorder["saved"] = int(recorder.get("saved", 0)) + 1
    if preview_every_s > 0.0 and timestamp is not None:
        recorder["last_preview_bucket"] = int(np.floor(float(timestamp) / preview_every_s))
    return output_path


def _draw_detection_window(
    display,
    client,
    config: AirSimDetectionConfig,
    detections,
    selected,
    intrinsics,
    yaw_rad: float,
    pitch_rad: float,
    valid: bool,
    reason: str,
    ttc_text: str,
    args,
    *,
    terminal_state: str = "",
    image_kf_mode: str = "",
    range_m: float = float("nan"),
    hit: bool = False,
    image_bgr=None,
) -> bool:
    if display is None or display.get("failed"):
        return False
    cv2 = display["cv2"]
    lines = _preview_lines(
        profile="gimbal",
        detections=detections,
        selected=selected,
        valid=valid,
        reason=reason,
        ttc_text=ttc_text,
        terminal_state=terminal_state,
        image_kf_mode=image_kf_mode,
        range_m=range_m,
        control_line=f"yaw={np.rad2deg(yaw_rad):.1f} pitch={np.rad2deg(pitch_rad):.1f}",
        hit=hit,
        include_quit=True,
    )
    try:
        image = _annotated_detection_image(
            cv2=cv2,
            airsim_module=display["airsim"],
            client=client,
            config=config,
            detections=detections,
            selected=selected,
            intrinsics=intrinsics,
            lines=lines,
            window_scale=args.window_scale,
            image_bgr=image_bgr,
        )
        if image is None:
            display["image_failures"] += 1
            if display["image_failures"] <= 3:
                print("OpenCV display: failed to capture or decode AirSim image.")
            return False
        cv2.imshow("Gimbal Vision PNG", image)
        key = cv2.waitKey(1) & 0xFF
        return key == ord("q")
    except Exception as exc:
        display["failed"] = True
        print(f"OpenCV display failed ({exc}); continuing without preview window.")
        return False


def _probe_camera_intrinsics(client, airsim_module, config: AirSimDetectionConfig, args):
    intrinsics = infer_intrinsics_from_fov(args.width, args.height, args.fov_deg)
    try:
        import cv2
    except Exception as exc:
        print(
            f"OpenCV image probe unavailable ({exc}); "
            f"using CLI camera size {args.width}x{args.height}."
        )
        return intrinsics
    try:
        raw_image = client.simGetImage(
            config.camera_name,
            getattr(airsim_module.ImageType, config.image_type_name),
            vehicle_name=config.vehicle_name,
        )
    except Exception as exc:
        print(
            f"Camera image probe failed ({exc}); "
            f"using CLI camera size {args.width}x{args.height}."
        )
        return intrinsics
    if raw_image is None:
        print(f"Camera image probe returned no image; using CLI camera size {args.width}x{args.height}.")
        return intrinsics
    image = _decode_airsim_image(cv2, raw_image)
    if image is None:
        print(f"Camera image probe decode failed; using CLI camera size {args.width}x{args.height}.")
        return intrinsics
    height, width = image.shape[:2]
    if width <= 0 or height <= 0:
        print(f"Camera image probe invalid shape={image.shape}; using CLI camera size {args.width}x{args.height}.")
        return intrinsics
    probed = infer_intrinsics_from_fov(width, height, args.fov_deg)
    if width != args.width or height != args.height:
        print(
            "Camera image size differs from CLI defaults: "
            f"actual={width}x{height}, cli={args.width}x{args.height}; using actual size."
        )
    else:
        print(f"Camera image size: {width}x{height}")
    return probed


def _detection_names(detections) -> str:
    names = [getattr(detection, "name", "") or "(unnamed)" for detection in detections]
    if not names:
        return ""
    return "|".join(names)


def _warmup_detection(client, config: AirSimDetectionConfig, args) -> None:
    if args.detection_warmup_s <= 0.0:
        return
    start = time.monotonic()
    best_count = 0
    best_names = ""
    best_area = 0.0
    while time.monotonic() - start < args.detection_warmup_s:
        detections = list(get_detections(client, config))
        if len(detections) > best_count:
            best_count = len(detections)
            best_names = _detection_names(detections)
        for detection in detections:
            frame_detection = detection_to_frame_detection(detection, 0, 0.0, 1)
            best_area = max(best_area, frame_detection.area)
        if detections:
            break
        time.sleep(0.1)
    print(
        "Initial detection warmup: "
        f"count={best_count}, names={best_names or '(none)'}, max_bbox_area={best_area:.3f}, "
        f"mesh={config.mesh_name_pattern!r}, radius_cm={config.detection_radius_cm:.1f}"
    )


def _prefer_user_mpl_toolkits() -> None:
    user_site = Path(site.getusersitepackages())
    user_toolkits = user_site / "mpl_toolkits"
    if not user_toolkits.exists():
        return
    user_site_text = str(user_site)
    if user_site_text in sys.path:
        sys.path.remove(user_site_text)
    sys.path.insert(0, user_site_text)
    for name in list(sys.modules):
        if name == "mpl_toolkits" or name.startswith("mpl_toolkits."):
            del sys.modules[name]
    module = types.ModuleType("mpl_toolkits")
    module.__path__ = [str(user_toolkits)]
    module.__file__ = str(user_toolkits)
    sys.modules["mpl_toolkits"] = module


def _output_paths(args) -> tuple[Path, Path]:
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    prefix = args.trajectory_prefix or f"gimbal_vision_png_{timestamp}"
    base = Path(args.trajectory_dir)
    return base / f"{prefix}.csv", base / f"{prefix}.png"


def _write_csv(rows: Sequence[dict[str, float | int | str]], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys()) if rows else []
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        if not fields:
            return
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _rows_until_first_hit(rows: Sequence[dict[str, float | int | str]]) -> Sequence[dict[str, float | int | str]]:
    for index, row in enumerate(rows):
        try:
            hit = int(row.get("hit", 0)) == 1
        except (TypeError, ValueError):
            hit = False
        if hit:
            return rows[: index + 1]
    return rows


def _plot(rows: Sequence[dict[str, float | int | str]], plot_path: Path) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        _prefer_user_mpl_toolkits()
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except Exception as exc:
        print(f"matplotlib 3D plot unavailable ({exc}); CSV was still saved.")
        return False
    rows = _rows_until_first_hit(rows)
    if not rows:
        return False

    has_truth = all(str(row.get("intruder_x", "")) != "" for row in rows)
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(15, 8))
    ax3d = fig.add_subplot(221, projection="3d")
    ax_range = fig.add_subplot(222)
    ax_pixel = fig.add_subplot(223)
    ax_gimbal = fig.add_subplot(224)

    trajectory_specs = [("interceptor", "tab:blue", "Interceptor")]
    if has_truth:
        trajectory_specs.append(("intruder", "tab:red", "Intruder"))
    for prefix, color, label in trajectory_specs:
        xs = [float(row[f"{prefix}_x"]) for row in rows]
        ys = [float(row[f"{prefix}_y"]) for row in rows]
        alts = [-float(row[f"{prefix}_z"]) for row in rows]
        ax3d.plot(xs, ys, alts, color=color, linewidth=2, label=label)
        ax3d.scatter(xs[0], ys[0], alts[0], color=color, marker="o", s=28)
        ax3d.scatter(xs[-1], ys[-1], alts[-1], color=color, marker="x", s=52)
    ax3d.set_xlabel("NED X / m")
    ax3d.set_ylabel("NED Y / m")
    ax3d.set_zlabel("Altitude / m")
    ax3d.legend()
    ax3d.grid(True)

    t = [float(row["t"]) for row in rows]
    if has_truth:
        ax_range.plot(t, [float(row["range"]) for row in rows], color="tab:green")
    else:
        ax_range.text(0.5, 0.5, "diagnostic truth disabled", transform=ax_range.transAxes, ha="center", va="center")
    ax_range.set_xlabel("Time / s")
    ax_range.set_ylabel("Range / m")
    ax_range.grid(True)

    ax_pixel.plot(t, [float(row["pixel_error_x"]) for row in rows], label="u error")
    ax_pixel.plot(t, [float(row["pixel_error_y"]) for row in rows], label="v error")
    ax_pixel.set_xlabel("Time / s")
    ax_pixel.set_ylabel("Pixel error / px")
    ax_pixel.legend()
    ax_pixel.grid(True)

    ax_gimbal.plot(t, [float(row["gimbal_yaw_deg"]) for row in rows], label="yaw")
    ax_gimbal.plot(t, [float(row["gimbal_pitch_deg"]) for row in rows], label="pitch")
    if all("body_yaw_deg" in row and "cmd_yaw_deg" in row for row in rows):
        ax_gimbal.plot(t, [float(row["body_yaw_deg"]) for row in rows], label="body yaw", linestyle="--")
        ax_gimbal.plot(t, [float(row["cmd_yaw_deg"]) for row in rows], label="cmd yaw", linestyle=":")
    ax_gimbal.set_xlabel("Time / s")
    ax_gimbal.set_ylabel("Yaw/Pitch / deg")
    ax_gimbal.legend()
    ax_gimbal.grid(True)

    fig.subplots_adjust(left=0.05, right=0.98, bottom=0.08, top=0.94, hspace=0.32, wspace=0.25)
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    return True


def main() -> None:
    args = parse_args()
    try:
        import airsim
    except ImportError as exc:
        raise SystemExit("Install the AirSim Python package before running this example.") from exc
    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive")
    if args.speed_ratio <= 0.0:
        raise SystemExit("--speed-ratio must be positive")

    client = airsim.MultirotorClient()
    _register_px4_shutdown_stop(client, args)
    try:
        client.confirmConnection()
    except Exception as exc:
        raise SystemExit("Failed to connect to AirSim RPC. Start Blocks first.") from exc
    required_vehicles = [args.interceptor] if args.intruder_actor else [args.interceptor, args.intruder]
    available = _require_vehicles(client, required_vehicles)
    print(f"AirSim vehicles: {_format_names(available)}")
    if args.list_vehicles:
        return
    detector = create_detection_provider(args, airsim_module=airsim)

    if args.intruder_actor:
        args.px4_intruder = False
        if args.mesh == "Intruder*":
            args.mesh = _actor_name(args)
    origins = _load_vehicle_origins(args.settings_path, required_vehicles)
    if args.reset and (args.px4_interceptor or args.px4_intruder):
        print("PX4 SITL mode: ignoring AirSim client.reset(); restart PX4/Blocks for a clean SITL session.")
    elif args.reset:
        client.reset()
        time.sleep(1.0)
    for vehicle in required_vehicles:
        client.enableApiControl(True, vehicle_name=vehicle)
        if _is_px4_vehicle(vehicle, args):
            _ensure_px4_api_control(client, args, vehicle)
        else:
            client.armDisarm(True, vehicle_name=vehicle)
    _prepare_intercept_altitude(client, required_vehicles, args, origins)
    _px4_keepalive(client, airsim, required_vehicles, args, duration_s=0.8)
    _apply_start_geometry(client, airsim, args, origins)
    _px4_keepalive(client, airsim, required_vehicles, args, duration_s=0.8)
    if args.intruder_actor and not _start_geometry_enabled(args):
        interceptor_kin = _guidance_kinematics(client, args.interceptor, args)
        interceptor_pos = _world_position(interceptor_kin, args.interceptor, origins)
        actor_pos = interceptor_pos + np.array(
            [
                float(args.start_forward_offset_m or args.start_horizontal_range_m or 100.0),
                float(args.start_lateral_offset_m),
                -float(args.intruder_altitude_offset_m),
            ],
            dtype=float,
        )
        _spawn_or_move_intruder_actor(client, airsim, args, actor_pos, _yaw_deg_from_velocity(_intruder_velocity(args)))

    config = AirSimDetectionConfig(
        camera_name=args.camera,
        detection_radius_cm=args.detection_radius_cm,
        mesh_name_pattern=args.mesh,
        vehicle_name=args.interceptor,
    )
    if args.detector_source == "airsim":
        configure_detection_filter(client, config)
        _configure_actor_detection_aliases(client, airsim, config, args)

    attitude_buffer = AttitudeHistoryBuffer(duration_s=2.0)
    los_filter = LOSKalmanFilter6D()
    ttc_filter = ScaleExpansionTTC(
        TTCConfig(min_area=max(0.0, args.ttc_min_area), max_ttc_s=max(0.1, args.ttc_max_s))
    )
    gain_schedule = TTCGainSchedule()
    terminal_extrapolator = TerminalExtrapolator(_terminal_config_from_args(args))
    terminal_image_kf = TerminalImageKF(_terminal_image_kf_config_from_args(args))

    target_dt = 1.0 / args.rate_hz
    intruder_velocity_cmd = _intruder_velocity(args)
    intruder_speed = max(float(np.linalg.norm(intruder_velocity_cmd)), args.intruder_speed)
    speed_cap = args.speed_ratio * intruder_speed
    actor_initial_pos: Optional[np.ndarray] = None
    if args.intruder_actor:
        actor_initial_pos = _intruder_truth_position(client, args, origins)
    if args.initial_truth_align:
        yaw_rad, pitch_rad = _initial_truth_align_gimbal(client, args, origins)
    else:
        yaw_rad = 0.0
        pitch_rad = 0.0
    client.simSetCameraPose(args.camera, _camera_pose(airsim, yaw_rad, pitch_rad, args), vehicle_name=args.interceptor)
    intrinsics = _probe_camera_intrinsics(client, airsim, config, args)
    experiment_fields = _experiment_fields(
        args=args,
        experiment_type="gimbal_vision_png",
        speed_cap=speed_cap,
        intruder_velocity_cmd=intruder_velocity_cmd,
        intrinsics=intrinsics,
    )
    if args.detector_source == "airsim":
        _warmup_detection(client, config, args)
    else:
        print(
            "YOLO ByteTrack detector active: "
            f"model={args.yolo_model}, class_id={args.yolo_class_id}, tracker={args.yolo_tracker}"
        )
    display = _make_display(airsim, args.show_window)
    active_name: Optional[str] = None
    active_track_id: Optional[int] = None
    last_valid_ts: Optional[float] = None
    last_lambda_I: Optional[np.ndarray] = None
    last_omega_los: Optional[np.ndarray] = None
    last_v_cmd: Optional[np.ndarray] = None
    rows: list[dict[str, float | int | str]] = []
    csv_path, plot_path = _output_paths(args)
    preview_recorder = _make_preview_recorder(airsim, args, csv_path.with_name(f"{csv_path.stem}_preview"))
    start = time.monotonic()
    last_loop_start = start
    last_wall_t: Optional[float] = None
    last_kin_t: Optional[float] = None
    last_interceptor_vel: Optional[np.ndarray] = None
    last_raw_lambda_I: Optional[np.ndarray] = None
    last_raw_lambda_ts: Optional[float] = None
    sim_start_t: Optional[float] = None
    frame_id = 0
    hit = False
    bbox_noise_rng = np.random.default_rng(int(args.bbox_noise_seed))

    print(
        "frame,t,loop_dt,command_duration,detection_count,detected,range,px_err_x,px_err_y,bbox_area,"
        "ttc,valid,guidance_mode,reason,yaw_deg,pitch_deg,v_cmd,hit"
    )
    while True:
        loop_start = time.monotonic()
        loop_dt = target_dt if frame_id == 0 else max(1.0e-6, loop_start - last_loop_start)
        last_loop_start = loop_start
        command_duration = _command_duration(loop_dt, target_dt, args)
        if args.enable_motion and not hit and not args.intruder_actor:
            _command_vehicle_velocity(client, airsim, args.intruder, intruder_velocity_cmd, 0.0, command_duration, args)
        capture_yaw_rad = yaw_rad
        capture_pitch_rad = pitch_rad
        client.simSetCameraPose(
            args.camera,
            _camera_pose(airsim, capture_yaw_rad, capture_pitch_rad, args),
            vehicle_name=args.interceptor,
        )

        state = client.getMultirotorState(vehicle_name=args.interceptor)
        kin_t_abs = _kinematics_timestamp_s(state)
        if kin_t_abs is not None and sim_start_t is None:
            sim_start_t = kin_t_abs
        sim_t = (
            kin_t_abs - sim_start_t
            if kin_t_abs is not None and sim_start_t is not None
            else loop_start - start
        )
        if sim_t >= args.duration_s:
            break

        if args.enable_motion and args.intruder_actor and actor_initial_pos is not None and not hit:
            actor_pos = actor_initial_pos + intruder_velocity_cmd * float(sim_t)
            _move_intruder_actor(client, airsim, args, actor_pos, _yaw_deg_from_velocity(intruder_velocity_cmd))

        interceptor_kin = _guidance_kinematics(client, args.interceptor, args)
        interceptor_pos = _world_position(interceptor_kin, args.interceptor, origins)
        interceptor_vel = _vector_xyz(interceptor_kin.linear_velocity)
        interceptor_accel = _linear_acceleration_mps2(interceptor_kin)
        interceptor_accel_norm = float(np.linalg.norm(interceptor_accel))
        load_factor_g = interceptor_accel_norm / GRAVITY_MPS2
        kin_t = kin_t_abs or _kinematics_timestamp_s(interceptor_kin)
        wall_fps = 0.0 if last_wall_t is None else 1.0 / max(1.0e-6, loop_start - last_wall_t)
        sim_sample_fps = ""
        sim_clock_ratio = ""
        accel_fd_norm = 0.0
        load_factor_fd_g = 0.0
        if kin_t is not None and last_kin_t is not None:
            kin_dt = max(1.0e-6, kin_t - last_kin_t)
            sim_sample_fps = 1.0 / kin_dt
            sim_clock_ratio = kin_dt / max(1.0e-6, loop_start - float(last_wall_t or loop_start))
            if last_interceptor_vel is not None:
                accel_fd = (interceptor_vel - last_interceptor_vel) / kin_dt
                accel_fd_norm = float(np.linalg.norm(accel_fd))
                load_factor_fd_g = accel_fd_norm / GRAVITY_MPS2
        last_wall_t = loop_start
        if kin_t is not None:
            last_kin_t = kin_t
        last_interceptor_vel = np.array(interceptor_vel, dtype=float)
        intruder_pos = np.full(3, np.nan)
        rel = np.full(3, np.nan)
        range_m = float("nan")
        if args.diagnostic_truth:
            intruder_pos = _intruder_truth_position(client, args, origins)
            rel = intruder_pos - interceptor_pos
            range_m = float(np.linalg.norm(rel))
        if args.intruder_actor:
            pair_collision = get_vehicle_object_collision(
                client,
                args.interceptor,
                _actor_collision_patterns(args),
            )
        else:
            pair_collision = get_vehicle_pair_collision(
                client,
                args.interceptor,
                args.intruder,
                interceptor_object_patterns=args.collision_interceptor_pattern,
                intruder_object_patterns=args.collision_intruder_pattern,
            )
        hit = pair_collision.collided

        R_IB = airsim_orientation_to_R_IB(state.kinematics_estimated.orientation)
        attitude_buffer.push(AttitudeSample(timestamp=sim_t, R_IB=R_IB))
        body_yaw_deg = _body_yaw_deg(airsim, state.kinematics_estimated.orientation)

        detector_frame = detector.detect(
            client=client,
            config=config,
            frame_id=frame_id,
            exposure_ts=sim_t,
            active_name=active_name,
            active_track_id=active_track_id,
        )
        detections = detector_frame.detections
        detection_count = len(detections)
        detection_names = _detection_names(detections)
        detection = detector_frame.selected
        detected = detection is not None
        detector_stats = detector_frame.stats
        detector_image = detector_frame.image_bgr
        px_err_x = 0.0
        px_err_y = 0.0
        ttc_value = ""
        reason = ""
        valid = False
        g_eval = np.zeros(3)
        los_quality = 0.0
        ttc_quality = 0.0
        ttc_area = 0.0
        ttc_area_dot = 0.0
        lambda_I = last_lambda_I
        omega_los = last_omega_los
        guidance_gain = 0.0
        bbox_area = 0.0
        guidance_mode = "invalid"
        detection_score = 0.0
        center: Optional[tuple[float, float]] = None
        frame_detection = None
        bbox_noise_dx_px = 0.0
        bbox_noise_dy_px = 0.0
        bbox_noise_area_scale = 1.0
        bbox_raw_xyxy = (0.0, 0.0, 0.0, 0.0)
        bbox_noisy_xyxy = (0.0, 0.0, 0.0, 0.0)
        detection_clipped = False
        bbox_left_clipped = False
        bbox_right_clipped = False
        bbox_top_clipped = False
        bbox_bottom_clipped = False
        lambda_I_measured: Optional[np.ndarray] = None
        lambda_raw: Optional[np.ndarray] = None
        omega_raw: Optional[np.ndarray] = None
        los_dt_s: float | str = ""
        los_angle_step_deg: float | str = ""
        los_source = "none"
        track_switched = False

        if detection is None:
            reason = str(detector_stats.get("detector_reject_reason") or "no_detection")
        else:
            active_name = getattr(detection, "name", None) or active_name
            frame_detection = detector_frame.frame_detection
            if frame_detection is None:
                reason = str(detector_stats.get("detector_reject_reason") or "no_detection")
            else:
                track_switched = active_track_id is not None and int(frame_detection.track_id) != int(active_track_id)
                if track_switched:
                    los_filter.reset()
                    ttc_filter.reset()
                    terminal_image_kf.reset()
                    terminal_extrapolator = TerminalExtrapolator(_terminal_config_from_args(args))
                    last_lambda_I = None
                    last_omega_los = None
                    last_raw_lambda_I = None
                    last_raw_lambda_ts = None
                    last_valid_ts = None
                    lambda_I = None
                    omega_los = None
                active_track_id = int(frame_detection.track_id)
        if frame_detection is not None:
            frame_detection, bbox_noise = _apply_bbox_noise(frame_detection, intrinsics, bbox_noise_rng, args)
            bbox_noise_dx_px = float(bbox_noise["dx"])
            bbox_noise_dy_px = float(bbox_noise["dy"])
            bbox_noise_area_scale = float(bbox_noise["area_scale"])
            bbox_raw_xyxy = tuple(float(value) for value in bbox_noise["raw_bbox"])
            bbox_noisy_xyxy = tuple(float(value) for value in bbox_noise["noisy_bbox"])
            detection_score = frame_detection.score
            bbox_area = frame_detection.area
            center = frame_detection.center
            px_err_x = center[0] - intrinsics.cx
            px_err_y = center[1] - intrinsics.cy
            clip_flags = frame_detection.clip_flags(intrinsics.width, intrinsics.height)
            bbox_left_clipped = bool(clip_flags["left"])
            bbox_right_clipped = bool(clip_flags["right"])
            bbox_top_clipped = bool(clip_flags["top"])
            bbox_bottom_clipped = bool(clip_flags["bottom"])
            detection_clipped = any(clip_flags.values())

            lookup = attitude_buffer.lookup(frame_detection.exposure_ts)
            if not lookup.valid or lookup.sample is None:
                reason = lookup.reason or "attitude_lookup_failed"
            else:
                R_BC = airsim_gimbal_camera_to_body(capture_yaw_rad, capture_pitch_rad)
                los_C = camera_ray_from_pixel(*center, intrinsics)
                lambda_I_measured = los_camera_to_inertial(los_C, R_BC, lookup.sample.R_IB)
                lambda_raw = normalize(lambda_I_measured)
                if last_raw_lambda_I is not None and last_raw_lambda_ts is not None:
                    raw_dt = max(1.0e-3, frame_detection.exposure_ts - last_raw_lambda_ts)
                    los_dt_s = raw_dt
                    raw_delta = project_perpendicular((lambda_raw - last_raw_lambda_I) / raw_dt, lambda_raw)
                    omega_raw = np.cross(lambda_raw, raw_delta)
                    dot_raw = float(np.clip(np.dot(last_raw_lambda_I, lambda_raw), -1.0, 1.0))
                    los_angle_step_deg = float(np.degrees(np.arccos(dot_raw)))
                else:
                    omega_raw = np.zeros(3, dtype=float)
                last_raw_lambda_I = np.array(lambda_raw, dtype=float)
                last_raw_lambda_ts = frame_detection.exposure_ts

                if args.los_filter:
                    los = los_filter.update(frame_detection.exposure_ts, lambda_I_measured)
                    los_source = "kalman"
                    los_valid = los.valid
                    los_lambda = los.lambda_I
                    los_omega = los.omega_los
                    los_quality = los.quality
                    los_reject_reason = los.reject_reason
                else:
                    los_source = "raw_fd"
                    los_valid = True
                    los_lambda = lambda_raw
                    los_omega = omega_raw
                    los_quality = 1.0
                    los_reject_reason = None
                ttc = ttc_filter.update(frame_detection, intrinsics.width, intrinsics.height)
                ttc_quality = ttc.quality
                ttc_area = ttc.area_filtered
                ttc_area_dot = ttc.area_dot_filtered
                if ttc.ttc is not None:
                    ttc_value = f"{ttc.ttc:.3f}"
                if not los_valid:
                    reason = los_reject_reason or "los_invalid"
                elif not ttc.valid or ttc.ttc is None:
                    lambda_I = los_lambda
                    omega_los = los_omega
                    reason = ttc.reject_reason or "ttc_invalid"
                    if _los_fallback_allowed(reason, args):
                        guidance_gain = max(0.0, args.los_fallback_gain)
                        g_eval = guidance_gain * omega_los
                        valid = True
                        guidance_mode = "los_fallback"
                        last_valid_ts = sim_t
                        last_lambda_I = lambda_I
                        last_omega_los = omega_los
                else:
                    gain = gain_schedule.gain(ttc.ttc)
                    guidance_gain = gain
                    omega_los = los_omega
                    lambda_I = los_lambda
                    g_eval = gain * omega_los
                    valid = True
                    guidance_mode = "ttc_png"
                    last_valid_ts = sim_t
                    last_lambda_I = lambda_I
                    last_omega_los = omega_los

        image_kf = terminal_image_kf.update(
            timestamp=sim_t,
            center=center,
            intrinsics=intrinsics,
            detected=detected,
            measurement_valid=detected,
            clipped=detection_clipped,
            track_id=active_track_id if detected else None,
        )

        if not valid and last_valid_ts is not None and sim_t - last_valid_ts <= args.coast_timeout_s:
            lambda_I = last_lambda_I
            omega_los = last_omega_los

        candidate_v_cmd = _guidance_velocity(interceptor_vel, lambda_I, omega_los if valid else None, guidance_gain, speed_cap, args)
        terminal_result = terminal_extrapolator.update(
            timestamp=sim_t,
            detected=detected,
            measurement_valid=valid,
            measurement_score=detection_score,
            bbox_area=bbox_area,
            image_width=intrinsics.width,
            image_height=intrinsics.height,
            reject_reason=reason,
            v_cmd=candidate_v_cmd,
            lambda_I=lambda_I,
            omega_los=omega_los if valid else None,
            speed_cap=speed_cap,
            max_vertical_speed=args.max_vision_vertical_speed,
            gimbal_at_limit=_near_gimbal_limit(yaw_rad, pitch_rad, args),
            safety_ok=_airsim_safety_ok(client, args.interceptor),
            soft_measurement_valid=image_kf.valid,
        )
        v_cmd = terminal_result.v_cmd
        if terminal_result.using_blind_push:
            guidance_mode = "blind_push"
            reason = terminal_result.reason
        elif valid:
            last_v_cmd = np.array(v_cmd, dtype=float)
        gimbal_update_enabled, gimbal_gain_scale, gimbal_hold_reason = _gimbal_update_profile(
            detected=detected,
            terminal_state=terminal_result.state,
            terminal_reason=terminal_result.reason,
            using_blind_push=terminal_result.using_blind_push,
            args=args,
        )
        gimbal_update_source = "hold"
        gimbal_update_center = center
        image_kf_takeover_allowed, image_kf_takeover_reason = _image_kf_takeover_allowed(
            image_kf,
            terminal_result,
            reason,
            detected,
            terminal_result.area_ratio,
            args,
            profile="gimbal",
        )
        if image_kf_takeover_allowed:
            gimbal_update_enabled = True
            gimbal_gain_scale = max(0.0, float(args.terminal_gimbal_gain_scale))
            gimbal_hold_reason = "image_kf_predict"
            gimbal_update_center = center_from_angle_error(image_kf.theta, intrinsics)
            gimbal_update_source = "kf"
        elif center is not None and gimbal_update_enabled:
            gimbal_update_source = "measurement"

        if gimbal_update_center is not None and gimbal_update_enabled:
            yaw_rad, pitch_rad, _, _ = _update_gimbal_from_pixel(
                yaw_rad,
                pitch_rad,
                gimbal_update_center,
                intrinsics,
                loop_dt,
                args,
                gain_scale=gimbal_gain_scale,
            )

        stop_requested = _draw_detection_window(
            display,
            client,
            config,
            detections,
            detection,
            intrinsics,
            yaw_rad,
            pitch_rad,
            valid,
            reason,
            ttc_value,
            args,
            terminal_state=terminal_result.state,
            image_kf_mode=image_kf.mode,
            range_m=range_m,
            hit=hit,
            image_bgr=detector_image,
        )
        preview_lines = _preview_lines(
            profile="gimbal",
            detections=detections,
            selected=detection,
            valid=valid,
            reason=reason,
            ttc_text=ttc_value,
            terminal_state=terminal_result.state,
            image_kf_mode=image_kf.mode,
            range_m=range_m,
            control_line=f"yaw={np.rad2deg(yaw_rad):.1f} pitch={np.rad2deg(pitch_rad):.1f}",
            hit=hit,
        )
        _record_detection_preview(
            preview_recorder,
            client,
            config,
            detections,
            detection,
            intrinsics,
            preview_lines,
            args,
            frame_id=frame_id,
            terminal_state=terminal_result.state,
            guidance_mode=guidance_mode,
            reason=reason,
            hit=hit,
            image_bgr=detector_image,
        )

        cmd_yaw_deg = _yaw_deg_from_velocity(v_cmd)
        yaw_error_deg = _wrap_angle_deg(cmd_yaw_deg - body_yaw_deg)
        target_bearing_deg = ""
        target_body_bearing_deg = ""
        gimbal_body_bearing_error_deg = ""
        if args.diagnostic_truth and np.all(np.isfinite(rel[:2])) and float(np.linalg.norm(rel[:2])) > 1.0e-6:
            target_bearing = _bearing_deg_from_xy(rel[:2])
            target_bearing_deg = target_bearing
            target_body_bearing = _wrap_angle_deg(target_bearing - body_yaw_deg)
            target_body_bearing_deg = target_body_bearing
            gimbal_body_bearing_error_deg = _wrap_angle_deg(float(np.rad2deg(yaw_rad)) - target_body_bearing)
        if args.enable_motion and not hit:
            _command_interceptor_velocity(client, airsim, v_cmd, cmd_yaw_deg, command_duration, args)

        frame_elapsed = time.monotonic() - loop_start
        vertical_error_sign = ""
        vertical_command_consistent = ""
        if args.diagnostic_truth and np.all(np.isfinite(rel)):
            vertical_error_sign = float(np.sign(rel[2]))
            if abs(float(rel[2])) <= 1.0e-6 or abs(float(v_cmd[2])) <= 1.0e-6:
                vertical_command_consistent = 1
            else:
                vertical_command_consistent = int(np.sign(rel[2]) == np.sign(v_cmd[2]))

        rows.append(
            {
                "frame": frame_id,
                **experiment_fields,
                "t": sim_t,
                "loop_dt": loop_dt,
                "wall_t": loop_start - start,
                "frame_elapsed": frame_elapsed,
                "wall_fps": wall_fps,
                "sim_time_s": "" if kin_t is None or sim_start_t is None else kin_t - sim_start_t,
                "sim_sample_fps": sim_sample_fps,
                "sim_clock_ratio": sim_clock_ratio,
                "command_duration": command_duration,
                "target_hz": args.rate_hz,
                "detection_count": detection_count,
                "detection_names": detection_names,
                "detected": int(detected),
                "target_name": active_name or "",
                "target_track_id": "" if active_track_id is None else int(active_track_id),
                "track_switched": int(track_switched),
                "detector_source": str(detector_stats.get("detector_source", args.detector_source)),
                "detector_reject_reason": str(detector_stats.get("detector_reject_reason", "")),
                "detector_raw_count": detector_stats.get("detector_raw_count", ""),
                "detector_class_filtered_count": detector_stats.get("detector_class_filtered_count", ""),
                "detector_track_filtered_count": detector_stats.get("detector_track_filtered_count", ""),
                "yolo_raw_count": detector_stats.get("yolo_raw_count", ""),
                "yolo_class_filtered_count": detector_stats.get("yolo_class_filtered_count", ""),
                "yolo_track_filtered_count": detector_stats.get("yolo_track_filtered_count", ""),
                "yolo_track_missing_count": detector_stats.get("yolo_track_missing_count", ""),
                "yolo_selected_track_id": detector_stats.get("yolo_selected_track_id", ""),
                "yolo_selected_class_id": detector_stats.get("yolo_selected_class_id", ""),
                "yolo_selected_score": detector_stats.get("yolo_selected_score", ""),
                "image_width": intrinsics.width,
                "image_height": intrinsics.height,
                "range": range_m,
                "horizontal_range": float(np.linalg.norm(rel[:2])) if args.diagnostic_truth else "",
                "vertical_error": float(rel[2]) if args.diagnostic_truth else "",
                "vertical_error_sign": vertical_error_sign,
                "v_cmd_z_sign": float(np.sign(v_cmd[2])),
                "vertical_command_consistent": vertical_command_consistent,
                "hit": int(hit),
                "collision_reason": pair_collision.reason,
                "interceptor_collision_object": pair_collision.interceptor_object_name,
                "intruder_collision_object": pair_collision.intruder_object_name,
                "pixel_error_x": px_err_x,
                "pixel_error_y": px_err_y,
                "bbox_area": bbox_area,
                "bbox_clipped": int(detection_clipped),
                "bbox_left_clipped": int(bbox_left_clipped),
                "bbox_right_clipped": int(bbox_right_clipped),
                "bbox_top_clipped": int(bbox_top_clipped),
                "bbox_bottom_clipped": int(bbox_bottom_clipped),
                "bbox_noise_enabled": int(args.bbox_noise),
                "bbox_noise_center_sigma_px": float(args.bbox_center_noise_px),
                "bbox_noise_area_sigma_ratio": float(args.bbox_area_noise_ratio),
                "bbox_noise_seed": int(args.bbox_noise_seed),
                "bbox_noise_dx_px": bbox_noise_dx_px,
                "bbox_noise_dy_px": bbox_noise_dy_px,
                "bbox_noise_area_scale": bbox_noise_area_scale,
                "bbox_raw_x1": bbox_raw_xyxy[0],
                "bbox_raw_y1": bbox_raw_xyxy[1],
                "bbox_raw_x2": bbox_raw_xyxy[2],
                "bbox_raw_y2": bbox_raw_xyxy[3],
                "bbox_noisy_x1": bbox_noisy_xyxy[0],
                "bbox_noisy_y1": bbox_noisy_xyxy[1],
                "bbox_noisy_x2": bbox_noisy_xyxy[2],
                "bbox_noisy_y2": bbox_noisy_xyxy[3],
                "gimbal_yaw_deg": float(np.rad2deg(yaw_rad)),
                "gimbal_pitch_deg": float(np.rad2deg(pitch_rad)),
                "body_yaw_deg": body_yaw_deg,
                "cmd_yaw_deg": cmd_yaw_deg,
                "yaw_error_deg": yaw_error_deg,
                "target_bearing_deg": target_bearing_deg,
                "target_body_bearing_deg": target_body_bearing_deg,
                "gimbal_body_bearing_error_deg": gimbal_body_bearing_error_deg,
                "los_quality": los_quality,
                "ttc_quality": ttc_quality,
                "ttc": "" if ttc_value == "" else float(ttc_value),
                "ttc_area": ttc_area,
                "ttc_area_dot": ttc_area_dot,
                "los_source": los_source,
                "los_filter_enabled": int(args.los_filter),
                "lambda_meas_x": "" if lambda_I_measured is None else float(lambda_I_measured[0]),
                "lambda_meas_y": "" if lambda_I_measured is None else float(lambda_I_measured[1]),
                "lambda_meas_z": "" if lambda_I_measured is None else float(lambda_I_measured[2]),
                "lambda_raw_x": "" if lambda_raw is None else float(lambda_raw[0]),
                "lambda_raw_y": "" if lambda_raw is None else float(lambda_raw[1]),
                "lambda_raw_z": "" if lambda_raw is None else float(lambda_raw[2]),
                "omega_raw_x": "" if omega_raw is None else float(omega_raw[0]),
                "omega_raw_y": "" if omega_raw is None else float(omega_raw[1]),
                "omega_raw_z": "" if omega_raw is None else float(omega_raw[2]),
                "los_dt_s": los_dt_s,
                "los_angle_step_deg": los_angle_step_deg,
                "valid": int(valid),
                "guidance_mode": guidance_mode,
                "reject_reason": reason,
                "blind_push_reason": terminal_result.reason if terminal_result.using_blind_push else "",
                "terminal_state": terminal_result.state,
                "terminal_reason": terminal_result.reason,
                "terminal_arm_source": terminal_result.terminal_arm_source,
                "terminal_cutoff_source": terminal_result.terminal_cutoff_source,
                "terminal_area_ratio": terminal_result.area_ratio,
                "terminal_miss_count": terminal_result.miss_count,
                "terminal_profile": "gimbal",
                "gimbal_update_enabled": int(gimbal_update_enabled),
                "gimbal_gain_scale": gimbal_gain_scale,
                "gimbal_hold_reason": gimbal_hold_reason,
                "gimbal_update_source": gimbal_update_source,
                "image_kf_takeover_allowed": int(image_kf_takeover_allowed),
                "image_kf_takeover_reason": image_kf_takeover_reason,
                "image_kf_valid": int(image_kf.valid),
                "image_kf_mode": image_kf.mode,
                "image_kf_theta_x": image_kf.theta_x,
                "image_kf_theta_y": image_kf.theta_y,
                "image_kf_theta_dot_x": image_kf.theta_dot_x,
                "image_kf_theta_dot_y": image_kf.theta_dot_y,
                "image_kf_age_s": image_kf.age_s,
                "image_kf_quality": image_kf.quality,
                "image_kf_reject_reason": image_kf.reject_reason,
                "blind_elapsed_s": terminal_result.blind_elapsed_s,
                "blind_decay": terminal_result.blind_decay,
                "blind_sample_count": terminal_result.blind_sample_count,
                "lambda_x": 0.0 if lambda_I is None else float(lambda_I[0]),
                "lambda_y": 0.0 if lambda_I is None else float(lambda_I[1]),
                "lambda_z": 0.0 if lambda_I is None else float(lambda_I[2]),
                "omega_x": 0.0 if omega_los is None else float(omega_los[0]),
                "omega_y": 0.0 if omega_los is None else float(omega_los[1]),
                "omega_z": 0.0 if omega_los is None else float(omega_los[2]),
                "g_eval_x": float(g_eval[0]),
                "g_eval_y": float(g_eval[1]),
                "g_eval_z": float(g_eval[2]),
                "v_cmd_base_x": float(terminal_result.v_cmd_base[0]),
                "v_cmd_base_y": float(terminal_result.v_cmd_base[1]),
                "v_cmd_base_z": float(terminal_result.v_cmd_base[2]),
                "v_cmd_trend_bias_x": float(terminal_result.v_cmd_trend_bias[0]),
                "v_cmd_trend_bias_y": float(terminal_result.v_cmd_trend_bias[1]),
                "v_cmd_trend_bias_z": float(terminal_result.v_cmd_trend_bias[2]),
                "v_cmd_pitch_up_bias_x": float(terminal_result.v_cmd_pitch_up_bias[0]),
                "v_cmd_pitch_up_bias_y": float(terminal_result.v_cmd_pitch_up_bias[1]),
                "v_cmd_pitch_up_bias_z": float(terminal_result.v_cmd_pitch_up_bias[2]),
                "v_cmd_x": float(v_cmd[0]),
                "v_cmd_y": float(v_cmd[1]),
                "v_cmd_z": float(v_cmd[2]),
                "interceptor_accel_x": float(interceptor_accel[0]),
                "interceptor_accel_y": float(interceptor_accel[1]),
                "interceptor_accel_z": float(interceptor_accel[2]),
                "interceptor_accel_norm_mps2": interceptor_accel_norm,
                "load_factor_g": load_factor_g,
                "interceptor_accel_fd_norm_mps2": accel_fd_norm,
                "load_factor_fd_g": load_factor_fd_g,
                "interceptor_x": float(interceptor_pos[0]),
                "interceptor_y": float(interceptor_pos[1]),
                "interceptor_z": float(interceptor_pos[2]),
                "intruder_x": float(intruder_pos[0]) if args.diagnostic_truth else "",
                "intruder_y": float(intruder_pos[1]) if args.diagnostic_truth else "",
                "intruder_z": float(intruder_pos[2]) if args.diagnostic_truth else "",
                "intruder_actor": int(args.intruder_actor),
                "intruder_actor_name": _actor_name(args) if args.intruder_actor else "",
                "px4_interceptor": int(args.px4_interceptor),
                "px4_intruder": int(args.px4_intruder),
                "px4_command_mode": args.px4_command_mode,
                "px4_command_join": int(args.px4_command_join),
            }
        )
        if args.print_every_n > 0 and (frame_id % args.print_every_n == 0 or hit):
            print(
                f"{frame_id},{sim_t:.3f},{loop_dt:.3f},{command_duration:.3f},"
                f"{detection_count},{int(detected)},{range_m:.3f},"
                f"{px_err_x:.1f},{px_err_y:.1f},{bbox_area:.2f},"
                f"{ttc_value},{valid},{guidance_mode},{reason},"
                f"{np.rad2deg(yaw_rad):.1f},{np.rad2deg(pitch_rad):.1f},"
                f"body_yaw={body_yaw_deg:.1f},cmd_yaw={cmd_yaw_deg:.1f},yaw_err={yaw_error_deg:.1f},"
                f"{np.array2string(v_cmd, precision=2)},{int(hit)}"
            )
        if hit:
            print(
                f"hit=True collision=True reason={pair_collision.reason} "
                f"range={range_m:.3f}m t={sim_t:.3f}s"
            )
            break
        if stop_requested:
            print("preview_window_quit=True")
            break
        sleep_s = max(0.0, target_dt - frame_elapsed)
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        frame_id += 1

    _write_csv(rows, csv_path)
    print(f"gimbal_vision_csv={csv_path}")
    metadata_path = _write_run_metadata(
        args=args,
        csv_path=csv_path,
        script_name=Path(__file__).name,
        experiment_type="gimbal_vision_png",
        intrinsics=intrinsics,
        speed_cap=speed_cap,
        intruder_velocity_cmd=intruder_velocity_cmd,
        rows=rows,
        hit=hit,
    )
    print(f"gimbal_vision_meta={metadata_path}")
    if not args.no_plot and _plot(rows, plot_path):
        print(f"gimbal_vision_plot={plot_path}")
    _summarize_run(rows)
    if not hit and rows:
        print(f"hit=False final_range={rows[-1]['range']:.3f}m")
    if display is not None:
        display["cv2"].destroyAllWindows()


if __name__ == "__main__":
    main()
