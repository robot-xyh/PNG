from __future__ import annotations

import argparse
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
    get_detections,
    infer_intrinsics_from_fov,
)
from vision_guidance.attitude_buffer import AttitudeHistoryBuffer  # noqa: E402
from vision_guidance.geometry import (  # noqa: E402
    airsim_gimbal_camera_to_body,
    camera_ray_from_pixel,
    los_camera_to_inertial,
    normalize,
)
from vision_guidance.los_filter import LOSKalmanFilter6D  # noqa: E402
from vision_guidance.png_eval import TTCGainSchedule  # noqa: E402
from vision_guidance.ttc import ScaleExpansionTTC, TTCConfig  # noqa: E402
from vision_guidance.types import AttitudeSample  # noqa: E402


PROJECT_ROOT = Path(__file__).resolve().parents[1]
AIRSIM_SETTINGS_PATH = Path.home() / "Documents" / "AirSim" / "settings.json"
SETTINGS_EXAMPLE_PATH = PROJECT_ROOT / "config" / "airsim_blocks_settings.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run AirSim gimbal-camera pure-vision PNG validation.")
    parser.add_argument("--interceptor", default="Interceptor")
    parser.add_argument("--intruder", default="Intruder")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--mesh", default="Intruder*")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fov-deg", type=float, default=90.0)
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
    parser.add_argument("--hit-radius-m", type=float, default=1.0)
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
    parser.add_argument("--allow-los-fallback", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--los-fallback-gain", type=float, default=0.5)
    parser.add_argument("--detection-warmup-s", type=float, default=1.0)
    parser.add_argument("--show-window", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--window-scale", type=float, default=0.75)
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


def _world_position(kinematics, vehicle_name: str, origins: dict[str, np.ndarray]) -> np.ndarray:
    return origins.get(vehicle_name, np.zeros(3, dtype=float)) + _vector_xyz(kinematics.position)


def _intruder_velocity(args) -> np.ndarray:
    vy = args.intruder_speed if args.intruder_vy is None else args.intruder_vy
    return np.array([args.intruder_vx, vy, args.intruder_vz], dtype=float)


def _target_altitude_m(vehicle: str, args) -> float:
    base_altitude = abs(float(args.intercept_altitude_m))
    if vehicle == args.intruder:
        return base_altitude + float(args.intruder_altitude_offset_m)
    return base_altitude


def _target_z_ned(vehicle: str, args) -> float:
    return -_target_altitude_m(vehicle, args)


def _prepare_intercept_altitude(client, vehicles: Sequence[str], args) -> None:
    if not args.climb_to_altitude:
        return
    target_text = ", ".join(
        f"{vehicle}: altitude={_target_altitude_m(vehicle, args):.1f}m, NED_Z={_target_z_ned(vehicle, args):.1f}"
        for vehicle in vehicles
    )
    print(f"Climbing vehicles to intercept start altitudes: {target_text}")
    for future in [client.takeoffAsync(timeout_sec=args.climb_timeout_s, vehicle_name=v) for v in vehicles]:
        future.join()
    for future in [
        client.moveToZAsync(_target_z_ned(v, args), velocity=args.climb_speed, timeout_sec=args.climb_timeout_s, vehicle_name=v)
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
            float(np.linalg.norm(_vector_xyz(_truth_kinematics(client, vehicle).linear_velocity)))
            for vehicle in vehicles
        ]
        if speeds and max(speeds) <= args.settle_speed:
            break
        for vehicle in vehicles:
            client.hoverAsync(vehicle_name=vehicle)
        time.sleep(0.2)
    print("Altitude preparation complete; starting gimbal vision loop.")


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
) -> tuple[float, float, float, float]:
    u, v = center
    yaw_error = float(np.arctan2(u - intrinsics.cx, intrinsics.fx))
    pitch_error = float(np.arctan2(v - intrinsics.cy, intrinsics.fy))
    max_step = np.deg2rad(args.gimbal_rate_limit_deg) * dt
    yaw_step = float(np.clip(args.gimbal_yaw_gain * yaw_error, -max_step, max_step))
    pitch_step = float(np.clip(args.gimbal_pitch_gain * pitch_error, -max_step, max_step))
    yaw_limit = np.deg2rad(args.gimbal_yaw_limit_deg)
    pitch_limit = np.deg2rad(args.gimbal_pitch_limit_deg)
    yaw_rad = float(np.clip(yaw_rad + yaw_step, -yaw_limit, yaw_limit))
    pitch_rad = float(np.clip(pitch_rad + pitch_step, -pitch_limit, pitch_limit))
    return yaw_rad, pitch_rad, yaw_error, pitch_error


def _gimbal_from_relative_body(relative_body: np.ndarray, args) -> tuple[float, float]:
    rel = np.asarray(relative_body, dtype=float)
    horizontal = float(np.hypot(rel[0], rel[1]))
    yaw = float(np.arctan2(rel[1], rel[0]))
    pitch = float(np.arctan2(rel[2], max(horizontal, 1.0e-6)))
    yaw = float(np.clip(yaw, -np.deg2rad(args.gimbal_yaw_limit_deg), np.deg2rad(args.gimbal_yaw_limit_deg)))
    pitch = float(np.clip(pitch, -np.deg2rad(args.gimbal_pitch_limit_deg), np.deg2rad(args.gimbal_pitch_limit_deg)))
    return yaw, pitch


def _initial_truth_align_gimbal(client, args, origins: dict[str, np.ndarray]) -> tuple[float, float]:
    interceptor_kin = _truth_kinematics(client, args.interceptor)
    intruder_kin = _truth_kinematics(client, args.intruder)
    interceptor_pos = _world_position(interceptor_kin, args.interceptor, origins)
    intruder_pos = _world_position(intruder_kin, args.intruder, origins)
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


def _near_gimbal_limit(yaw_rad: float, pitch_rad: float, args) -> bool:
    margin = np.deg2rad(max(0.0, args.gimbal_limit_margin_deg))
    yaw_limit = np.deg2rad(args.gimbal_yaw_limit_deg)
    pitch_limit = np.deg2rad(args.gimbal_pitch_limit_deg)
    return abs(yaw_rad) >= yaw_limit - margin or abs(pitch_rad) >= pitch_limit - margin


def _terminal_area(frame_area: float, intrinsics, args) -> bool:
    image_area = max(1.0, float(intrinsics.width * intrinsics.height))
    return frame_area >= max(0.0, args.terminal_bbox_area_ratio) * image_area


def _terminal_trigger(reason: str, detected: bool, bbox_area: float, yaw_rad: float, pitch_rad: float, intrinsics, args) -> str:
    if reason == "bbox_clipped":
        return "bbox_clipped"
    if detected and _terminal_area(bbox_area, intrinsics, args):
        return "bbox_area_large"
    if detected and _near_gimbal_limit(yaw_rad, pitch_rad, args) and _terminal_area(
        2.0 * bbox_area,
        intrinsics,
        args,
    ):
        return "gimbal_limit"
    return ""


def _command_duration(loop_dt: float, target_dt: float, args) -> float:
    requested = max(loop_dt + args.command_duration_margin_s, target_dt, args.min_command_duration_s)
    return float(np.clip(requested, args.min_command_duration_s, args.max_command_duration_s))


def _los_fallback_allowed(reason: str, args) -> bool:
    if not args.allow_los_fallback:
        return False
    return reason in {"ttc_out_of_range", "area_not_expanding", "ttc_invalid"}


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
        f"intruder_avg_speed={intruder_speed if intruder_speed is not None else float('nan'):.3f}"
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


def _decode_airsim_image(cv2, raw_image):
    if raw_image is None:
        return None
    if isinstance(raw_image, str):
        raw_image = raw_image.encode("latin1")
    return cv2.imdecode(np.frombuffer(raw_image, dtype=np.uint8), cv2.IMREAD_UNCHANGED)


def _draw_detection_window(display, client, config: AirSimDetectionConfig, detections, selected, intrinsics, yaw_rad: float, pitch_rad: float, valid: bool, reason: str, ttc_text: str, args) -> bool:
    if display is None or display.get("failed"):
        return False
    cv2 = display["cv2"]
    airsim_module = display["airsim"]
    try:
        raw_image = client.simGetImage(
            config.camera_name,
            getattr(airsim_module.ImageType, config.image_type_name),
            vehicle_name=config.vehicle_name,
        )
        if raw_image is None:
            display["image_failures"] += 1
            if display["image_failures"] <= 3:
                print("OpenCV display: simGetImage returned no image.")
            return False
        image = _decode_airsim_image(cv2, raw_image)
        if image is None:
            display["image_failures"] += 1
            if display["image_failures"] <= 3:
                print("OpenCV display: failed to decode AirSim image.")
            return False
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

        lines = [
            f"yaw={np.rad2deg(yaw_rad):.1f} pitch={np.rad2deg(pitch_rad):.1f}",
            f"detections={len(detections)} selected={getattr(selected, 'name', '-') if selected is not None else '-'}",
            f"valid={valid} reason={reason or 'ok'} ttc={ttc_text or '-'}",
            "q: quit",
        ]
        for i, line in enumerate(lines):
            cv2.putText(image, line, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (36, 255, 12), 1)

        if args.window_scale > 0.0 and abs(args.window_scale - 1.0) > 1.0e-6:
            image = cv2.resize(image, None, fx=args.window_scale, fy=args.window_scale)
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
    try:
        client.confirmConnection()
    except Exception as exc:
        raise SystemExit("Failed to connect to AirSim RPC. Start Blocks first.") from exc
    available = _require_vehicles(client, [args.interceptor, args.intruder])
    print(f"AirSim vehicles: {_format_names(available)}")
    if args.list_vehicles:
        return

    origins = _load_vehicle_origins(args.settings_path, [args.interceptor, args.intruder])
    if args.reset:
        client.reset()
        time.sleep(1.0)
    for vehicle in [args.interceptor, args.intruder]:
        client.enableApiControl(True, vehicle_name=vehicle)
        client.armDisarm(True, vehicle_name=vehicle)
    _prepare_intercept_altitude(client, [args.interceptor, args.intruder], args)

    config = AirSimDetectionConfig(
        camera_name=args.camera,
        detection_radius_cm=args.detection_radius_cm,
        mesh_name_pattern=args.mesh,
        vehicle_name=args.interceptor,
    )
    configure_detection_filter(client, config)

    attitude_buffer = AttitudeHistoryBuffer(duration_s=2.0)
    los_filter = LOSKalmanFilter6D()
    ttc_filter = ScaleExpansionTTC(
        TTCConfig(min_area=max(0.0, args.ttc_min_area), max_ttc_s=max(0.1, args.ttc_max_s))
    )
    gain_schedule = TTCGainSchedule()

    target_dt = 1.0 / args.rate_hz
    intruder_velocity_cmd = _intruder_velocity(args)
    intruder_speed = max(float(np.linalg.norm(intruder_velocity_cmd)), args.intruder_speed)
    speed_cap = args.speed_ratio * intruder_speed
    if args.initial_truth_align:
        yaw_rad, pitch_rad = _initial_truth_align_gimbal(client, args, origins)
    else:
        yaw_rad = 0.0
        pitch_rad = 0.0
    client.simSetCameraPose(args.camera, _camera_pose(airsim, yaw_rad, pitch_rad, args), vehicle_name=args.interceptor)
    intrinsics = _probe_camera_intrinsics(client, airsim, config, args)
    _warmup_detection(client, config, args)
    display = _make_display(airsim, args.show_window)
    active_name: Optional[str] = None
    last_valid_ts: Optional[float] = None
    last_lambda_I: Optional[np.ndarray] = None
    last_omega_los: Optional[np.ndarray] = None
    last_v_cmd: Optional[np.ndarray] = None
    last_detection_ts: Optional[float] = None
    blind_push_until: Optional[float] = None
    blind_push_reason = ""
    terminal_armed = False
    rows: list[dict[str, float | int | str]] = []
    csv_path, plot_path = _output_paths(args)
    start = time.monotonic()
    last_loop_start = start
    frame_id = 0
    hit = False

    print(
        "frame,t,loop_dt,command_duration,detection_count,detected,range,px_err_x,px_err_y,bbox_area,"
        "ttc,valid,guidance_mode,reason,yaw_deg,pitch_deg,v_cmd,hit"
    )
    while time.monotonic() - start < args.duration_s:
        loop_start = time.monotonic()
        sim_t = loop_start - start
        loop_dt = target_dt if frame_id == 0 else max(1.0e-6, loop_start - last_loop_start)
        last_loop_start = loop_start
        command_duration = _command_duration(loop_dt, target_dt, args)
        if args.enable_motion:
            client.moveByVelocityAsync(
                float(intruder_velocity_cmd[0]),
                float(intruder_velocity_cmd[1]),
                float(intruder_velocity_cmd[2]),
                duration=command_duration,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0.0),
                vehicle_name=args.intruder,
            )
        capture_yaw_rad = yaw_rad
        capture_pitch_rad = pitch_rad
        client.simSetCameraPose(
            args.camera,
            _camera_pose(airsim, capture_yaw_rad, capture_pitch_rad, args),
            vehicle_name=args.interceptor,
        )

        interceptor_kin = _truth_kinematics(client, args.interceptor)
        interceptor_pos = _world_position(interceptor_kin, args.interceptor, origins)
        interceptor_vel = _vector_xyz(interceptor_kin.linear_velocity)
        intruder_pos = np.full(3, np.nan)
        rel = np.full(3, np.nan)
        range_m = float("nan")
        hit = False
        if args.diagnostic_truth:
            intruder_kin = _truth_kinematics(client, args.intruder)
            intruder_pos = _world_position(intruder_kin, args.intruder, origins)
            rel = intruder_pos - interceptor_pos
            range_m = float(np.linalg.norm(rel))
            hit = range_m <= args.hit_radius_m

        state = client.getMultirotorState(vehicle_name=args.interceptor)
        R_IB = airsim_orientation_to_R_IB(state.kinematics_estimated.orientation)
        attitude_buffer.push(AttitudeSample(timestamp=sim_t, R_IB=R_IB))
        body_yaw_deg = _body_yaw_deg(airsim, state.kinematics_estimated.orientation)

        detections = list(get_detections(client, config))
        detection_count = len(detections)
        detection_names = _detection_names(detections)
        detection = choose_detection(detections, preferred_name=active_name)
        detected = detection is not None
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

        if detection is None:
            reason = "no_detection"
        else:
            last_detection_ts = sim_t
            active_name = getattr(detection, "name", None) or active_name
            frame_detection = detection_to_frame_detection(
                detection=detection,
                frame_id=frame_id,
                exposure_ts=sim_t,
                track_id=1,
                score=1.0,
            )
            bbox_area = frame_detection.area
            center = frame_detection.center
            px_err_x = center[0] - intrinsics.cx
            px_err_y = center[1] - intrinsics.cy

            lookup = attitude_buffer.lookup(frame_detection.exposure_ts)
            if not lookup.valid or lookup.sample is None:
                reason = lookup.reason or "attitude_lookup_failed"
            else:
                R_BC = airsim_gimbal_camera_to_body(capture_yaw_rad, capture_pitch_rad)
                los_C = camera_ray_from_pixel(*center, intrinsics)
                lambda_I_measured = los_camera_to_inertial(los_C, R_BC, lookup.sample.R_IB)
                los = los_filter.update(frame_detection.exposure_ts, lambda_I_measured)
                ttc = ttc_filter.update(frame_detection, intrinsics.width, intrinsics.height)
                los_quality = los.quality
                ttc_quality = ttc.quality
                ttc_area = ttc.area_filtered
                ttc_area_dot = ttc.area_dot_filtered
                if ttc.ttc is not None:
                    ttc_value = f"{ttc.ttc:.3f}"
                if not los.valid:
                    reason = los.reject_reason or "los_invalid"
                elif not ttc.valid or ttc.ttc is None:
                    lambda_I = los.lambda_I
                    omega_los = los.omega_los
                    reason = ttc.reject_reason or "ttc_invalid"
                    terminal_reason = _terminal_trigger(
                        reason,
                        detected,
                        bbox_area,
                        yaw_rad,
                        pitch_rad,
                        intrinsics,
                        args,
                    )
                    if terminal_reason and last_v_cmd is not None:
                        blind_push_until = sim_t + args.blind_push_timeout_s
                        blind_push_reason = terminal_reason
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
                    omega_los = los.omega_los
                    lambda_I = los.lambda_I
                    g_eval = gain * omega_los
                    valid = True
                    guidance_mode = "ttc_png"
                    last_valid_ts = sim_t
                    last_lambda_I = lambda_I
                    last_omega_los = omega_los

            yaw_rad, pitch_rad, _, _ = _update_gimbal_from_pixel(
                yaw_rad,
                pitch_rad,
                center,
                intrinsics,
                loop_dt,
                args,
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
        )

        terminal_reason = _terminal_trigger(reason, detected, bbox_area, yaw_rad, pitch_rad, intrinsics, args)
        if terminal_reason and last_v_cmd is not None:
            blind_push_until = sim_t + args.blind_push_timeout_s
            blind_push_reason = terminal_reason
            terminal_armed = True
        elif detected and blind_push_reason == "lost_after_track":
            blind_push_until = None
            blind_push_reason = ""
        if (
            not detected
            and last_detection_ts is not None
            and sim_t - last_detection_ts <= args.coast_timeout_s
            and last_v_cmd is not None
            and terminal_armed
        ):
            blind_push_until = max(blind_push_until or sim_t, sim_t + args.blind_push_timeout_s)
            blind_push_reason = "lost_after_track"

        using_blind_push = blind_push_until is not None and sim_t <= blind_push_until and last_v_cmd is not None
        if not valid and last_valid_ts is not None and sim_t - last_valid_ts <= args.coast_timeout_s:
            lambda_I = last_lambda_I
            omega_los = last_omega_los

        if using_blind_push:
            v_cmd = np.array(last_v_cmd, dtype=float)
            guidance_mode = "blind_push"
            reason = blind_push_reason
        else:
            v_cmd = _guidance_velocity(interceptor_vel, lambda_I, omega_los if valid else None, guidance_gain, speed_cap, args)
            if valid:
                last_v_cmd = np.array(v_cmd, dtype=float)
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
        if args.enable_motion:
            drivetrain, yaw_mode = _air_sim_yaw_mode(airsim, v_cmd, args)
            client.moveByVelocityAsync(
                float(v_cmd[0]),
                float(v_cmd[1]),
                float(v_cmd[2]),
                duration=command_duration,
                drivetrain=drivetrain,
                yaw_mode=yaw_mode,
                vehicle_name=args.interceptor,
            )

        frame_elapsed = time.monotonic() - loop_start

        rows.append(
            {
                "frame": frame_id,
                "t": sim_t,
                "loop_dt": loop_dt,
                "frame_elapsed": frame_elapsed,
                "command_duration": command_duration,
                "target_hz": args.rate_hz,
                "detection_count": detection_count,
                "detection_names": detection_names,
                "detected": int(detected),
                "target_name": active_name or "",
                "image_width": intrinsics.width,
                "image_height": intrinsics.height,
                "range": range_m,
                "horizontal_range": float(np.linalg.norm(rel[:2])) if args.diagnostic_truth else "",
                "vertical_error": float(rel[2]) if args.diagnostic_truth else "",
                "hit": int(hit),
                "pixel_error_x": px_err_x,
                "pixel_error_y": px_err_y,
                "bbox_area": bbox_area,
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
                "valid": int(valid),
                "guidance_mode": guidance_mode,
                "reject_reason": reason,
                "blind_push_reason": blind_push_reason if using_blind_push else "",
                "lambda_x": 0.0 if lambda_I is None else float(lambda_I[0]),
                "lambda_y": 0.0 if lambda_I is None else float(lambda_I[1]),
                "lambda_z": 0.0 if lambda_I is None else float(lambda_I[2]),
                "omega_x": 0.0 if omega_los is None else float(omega_los[0]),
                "omega_y": 0.0 if omega_los is None else float(omega_los[1]),
                "omega_z": 0.0 if omega_los is None else float(omega_los[2]),
                "g_eval_x": float(g_eval[0]),
                "g_eval_y": float(g_eval[1]),
                "g_eval_z": float(g_eval[2]),
                "v_cmd_x": float(v_cmd[0]),
                "v_cmd_y": float(v_cmd[1]),
                "v_cmd_z": float(v_cmd[2]),
                "interceptor_x": float(interceptor_pos[0]),
                "interceptor_y": float(interceptor_pos[1]),
                "interceptor_z": float(interceptor_pos[2]),
                "intruder_x": float(intruder_pos[0]) if args.diagnostic_truth else "",
                "intruder_y": float(intruder_pos[1]) if args.diagnostic_truth else "",
                "intruder_z": float(intruder_pos[2]) if args.diagnostic_truth else "",
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
            print(f"hit=True range={range_m:.3f}m t={sim_t:.3f}s")
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
    if not args.no_plot and _plot(rows, plot_path):
        print(f"gimbal_vision_plot={plot_path}")
    _summarize_run(rows)
    if not hit and rows:
        print(f"hit=False final_range={rows[-1]['range']:.3f}m")
    if display is not None:
        display["cv2"].destroyAllWindows()


if __name__ == "__main__":
    main()
