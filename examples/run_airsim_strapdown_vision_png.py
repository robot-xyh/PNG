from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from examples.run_airsim_gimbal_vision_png import (
    PROJECT_ROOT,
    SETTINGS_EXAMPLE_PATH,
    _bearing_deg_from_xy,
    _body_yaw_deg,
    _camera_offset_body,
    _command_duration,
    _decode_airsim_image,
    _detection_names,
    _format_names,
    _guidance_velocity,
    _intruder_velocity,
    _load_vehicle_origins,
    _los_fallback_allowed,
    _make_display,
    _output_paths,
    _prefer_user_mpl_toolkits,
    _prepare_intercept_altitude,
    _probe_camera_intrinsics,
    _require_vehicles,
    _summarize_run,
    _terminal_area,
    _truth_kinematics,
    _vector_xyz,
    _warmup_detection,
    _world_position,
    _wrap_angle_deg,
    _write_csv,
    _yaw_deg_from_velocity,
)
from vision_guidance.airsim_adapter import (
    AirSimDetectionConfig,
    airsim_orientation_to_R_IB,
    choose_detection,
    configure_detection_filter,
    detection_to_frame_detection,
    get_vehicle_pair_collision,
    get_detections,
)
from vision_guidance.attitude_buffer import AttitudeHistoryBuffer
from vision_guidance.geometry import airsim_camera_zero_to_body, camera_ray_from_pixel, los_camera_to_inertial
from vision_guidance.los_filter import LOSKalmanFilter6D
from vision_guidance.png_eval import TTCGainSchedule
from vision_guidance.ttc import ScaleExpansionTTC, TTCConfig
from vision_guidance.types import AttitudeSample


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run AirSim strapdown-camera pure-vision PNG validation.")
    parser.add_argument("--interceptor", default="Interceptor")
    parser.add_argument("--intruder", default="Intruder")
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
    parser.add_argument("--hit-radius-m", type=float, default=1.0, help="Deprecated; AirSim collision is the success criterion.")
    parser.add_argument("--max-vision-lateral-speed", type=float, default=4.0)
    parser.add_argument("--max-vision-vertical-speed", type=float, default=3.0)
    parser.add_argument("--coast-timeout-s", type=float, default=0.5)
    parser.add_argument("--blind-push-timeout-s", type=float, default=1.0)
    parser.add_argument("--terminal-bbox-area-ratio", type=float, default=0.25)
    parser.add_argument("--yaw-control", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--yaw-error-gain", type=float, default=2.5)
    parser.add_argument("--max-yaw-rate-deg", type=float, default=90.0)
    parser.add_argument("--initial-truth-align", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--initial-align-timeout-s", type=float, default=8.0)
    parser.add_argument("--initial-align-margin-deg", type=float, default=3.0)
    parser.add_argument("--detection-radius-cm", type=float, default=50000.0)
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


def _output_paths_strapdown(args) -> tuple[Path, Path]:
    if args.trajectory_prefix:
        return _output_paths(args)
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    base = Path(args.trajectory_dir)
    return base / f"strapdown_vision_png_{timestamp}.csv", base / f"strapdown_vision_png_{timestamp}.png"


def _fixed_camera_pose(airsim_module, args):
    return airsim_module.Pose(
        airsim_module.Vector3r(args.camera_x, args.camera_y, args.camera_z),
        airsim_module.to_quaternion(0.0, 0.0, 0.0),
    )


def _initial_truth_yaw_deg(client, args, origins: dict[str, np.ndarray]) -> float:
    interceptor_kin = _truth_kinematics(client, args.interceptor)
    intruder_kin = _truth_kinematics(client, args.intruder)
    interceptor_pos = _world_position(interceptor_kin, args.interceptor, origins)
    intruder_pos = _world_position(intruder_kin, args.intruder, origins)
    state = client.getMultirotorState(vehicle_name=args.interceptor)
    R_IB = airsim_orientation_to_R_IB(state.kinematics_estimated.orientation)
    camera_pos = interceptor_pos + R_IB @ _camera_offset_body(args)
    relative = intruder_pos - camera_pos
    yaw_deg = _bearing_deg_from_xy(relative[:2])
    print(
        "Initial truth strapdown yaw alignment: "
        f"camera_offset_body={np.array2string(_camera_offset_body(args), precision=2)}, "
        f"relative_I={np.array2string(relative, precision=2)}, yaw={yaw_deg:.2f}deg"
    )
    return yaw_deg


def _align_body_yaw_to_target(client, airsim_module, args, origins: dict[str, np.ndarray]) -> float:
    yaw_deg = _initial_truth_yaw_deg(client, args, origins)
    client.rotateToYawAsync(
        yaw_deg,
        timeout_sec=args.initial_align_timeout_s,
        margin=args.initial_align_margin_deg,
        vehicle_name=args.interceptor,
    ).join()
    state = client.getMultirotorState(vehicle_name=args.interceptor)
    actual_yaw_deg = _body_yaw_deg(airsim_module, state.kinematics_estimated.orientation)
    print(f"Initial body yaw after alignment: commanded={yaw_deg:.2f}deg, actual={actual_yaw_deg:.2f}deg")
    return yaw_deg


def _yaw_rate_from_pixel_error(pixel_error_x: float, intrinsics, args) -> float:
    if not args.yaw_control:
        return 0.0
    yaw_error_rad = float(np.arctan2(pixel_error_x, intrinsics.fx))
    yaw_rate = float(np.rad2deg(args.yaw_error_gain * yaw_error_rad))
    return float(np.clip(yaw_rate, -args.max_yaw_rate_deg, args.max_yaw_rate_deg))


def _terminal_trigger_strapdown(reason: str, detected: bool, bbox_area: float, intrinsics, args) -> str:
    if reason == "bbox_clipped":
        return "bbox_clipped"
    if detected and _terminal_area(bbox_area, intrinsics, args):
        return "bbox_area_large"
    return ""


def _row_float(row: dict[str, float | int | str], key: str, default: float = 0.0) -> float:
    try:
        value = row.get(key, default)
        if value == "":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _rows_until_first_hit(rows: list[dict[str, float | int | str]]) -> list[dict[str, float | int | str]]:
    for index, row in enumerate(rows):
        try:
            hit = int(row.get("hit", 0)) == 1
        except (TypeError, ValueError):
            hit = False
        if hit:
            return rows[: index + 1]
    return rows


def _plot_strapdown(rows: list[dict[str, float | int | str]], plot_path: Path) -> bool:
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
    ax_yaw = fig.add_subplot(224)

    trajectory_specs = [("interceptor", "tab:blue", "Interceptor")]
    if has_truth:
        trajectory_specs.append(("intruder", "tab:red", "Intruder"))
    for prefix, color, label in trajectory_specs:
        xs = [_row_float(row, f"{prefix}_x") for row in rows]
        ys = [_row_float(row, f"{prefix}_y") for row in rows]
        alts = [-_row_float(row, f"{prefix}_z") for row in rows]
        ax3d.plot(xs, ys, alts, color=color, linewidth=2, label=label)
        ax3d.scatter(xs[0], ys[0], alts[0], color=color, marker="o", s=28)
        ax3d.scatter(xs[-1], ys[-1], alts[-1], color=color, marker="x", s=52)
    ax3d.set_xlabel("NED X / m")
    ax3d.set_ylabel("NED Y / m")
    ax3d.set_zlabel("Altitude / m")
    ax3d.legend()
    ax3d.grid(True)

    t = [_row_float(row, "t") for row in rows]
    if has_truth:
        ax_range.plot(t, [_row_float(row, "range", float("nan")) for row in rows], color="tab:green")
    else:
        ax_range.text(0.5, 0.5, "diagnostic truth disabled", transform=ax_range.transAxes, ha="center", va="center")
    ax_range.set_xlabel("Time / s")
    ax_range.set_ylabel("Range / m")
    ax_range.grid(True)

    ax_pixel.plot(t, [_row_float(row, "pixel_error_x") for row in rows], label="u error")
    ax_pixel.plot(t, [_row_float(row, "pixel_error_y") for row in rows], label="v error")
    ax_pixel.set_xlabel("Time / s")
    ax_pixel.set_ylabel("Pixel error / px")
    ax_pixel.legend()
    ax_pixel.grid(True)

    ax_yaw.plot(t, [_row_float(row, "body_yaw_deg") for row in rows], label="body yaw")
    ax_yaw.plot(t, [_row_float(row, "cmd_yaw_deg") for row in rows], label="cmd yaw", linestyle=":")
    ax_yaw.plot(t, [_row_float(row, "target_body_bearing_deg") for row in rows], label="target body bearing", linestyle="--")
    ax_yaw.plot(t, [_row_float(row, "yaw_rate_cmd_deg_s") for row in rows], label="yaw rate cmd", alpha=0.75)
    ax_yaw.set_xlabel("Time / s")
    ax_yaw.set_ylabel("Yaw / deg, rate / deg/s")
    ax_yaw.legend()
    ax_yaw.grid(True)

    fig.subplots_adjust(left=0.05, right=0.98, bottom=0.08, top=0.94, hspace=0.32, wspace=0.25)
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    return True


def _draw_detection_window_strapdown(
    display,
    client,
    config: AirSimDetectionConfig,
    detections,
    selected,
    intrinsics,
    yaw_rate_deg_s: float,
    valid: bool,
    reason: str,
    ttc_text: str,
    args,
) -> bool:
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
            f"yaw_rate={yaw_rate_deg_s:.1f} deg/s",
            f"detections={len(detections)} selected={getattr(selected, 'name', '-') if selected is not None else '-'}",
            f"valid={valid} reason={reason or 'ok'} ttc={ttc_text or '-'}",
            "q: quit",
        ]
        for i, line in enumerate(lines):
            cv2.putText(image, line, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (36, 255, 12), 1)
        if args.window_scale > 0.0 and abs(args.window_scale - 1.0) > 1.0e-6:
            image = cv2.resize(image, None, fx=args.window_scale, fy=args.window_scale)
        cv2.imshow("Strapdown Vision PNG", image)
        key = cv2.waitKey(1) & 0xFF
        return key == ord("q")
    except Exception as exc:
        display["failed"] = True
        print(f"OpenCV display failed ({exc}); continuing without preview window.")
        return False


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
    client.simSetCameraPose(args.camera, _fixed_camera_pose(airsim, args), vehicle_name=args.interceptor)
    print("Fixed strapdown camera pose set once; subsequent alignment rotates interceptor body yaw only.")
    if args.initial_truth_align:
        _align_body_yaw_to_target(client, airsim, args, origins)

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
    intrinsics = _probe_camera_intrinsics(client, airsim, config, args)
    _warmup_detection(client, config, args)
    display = _make_display(airsim, args.show_window)

    target_dt = 1.0 / args.rate_hz
    intruder_velocity_cmd = _intruder_velocity(args)
    intruder_speed = max(float(np.linalg.norm(intruder_velocity_cmd)), args.intruder_speed)
    speed_cap = args.speed_ratio * intruder_speed
    R_BC = airsim_camera_zero_to_body()

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
    csv_path, plot_path = _output_paths_strapdown(args)
    start = time.monotonic()
    last_loop_start = start
    frame_id = 0
    hit = False

    print(
        "frame,t,loop_dt,command_duration,detection_count,detected,range,px_err_x,px_err_y,bbox_area,"
        "ttc,valid,guidance_mode,reason,yaw_rate,body_yaw,cmd_yaw,v_cmd,hit"
    )
    while time.monotonic() - start < args.duration_s:
        loop_start = time.monotonic()
        sim_t = loop_start - start
        loop_dt = target_dt if frame_id == 0 else max(1.0e-6, loop_start - last_loop_start)
        last_loop_start = loop_start
        command_duration = _command_duration(loop_dt, target_dt, args)
        if args.enable_motion and not hit:
            client.moveByVelocityAsync(
                float(intruder_velocity_cmd[0]),
                float(intruder_velocity_cmd[1]),
                float(intruder_velocity_cmd[2]),
                duration=command_duration,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0.0),
                vehicle_name=args.intruder,
            )

        interceptor_kin = _truth_kinematics(client, args.interceptor)
        interceptor_pos = _world_position(interceptor_kin, args.interceptor, origins)
        interceptor_vel = _vector_xyz(interceptor_kin.linear_velocity)
        intruder_pos = np.full(3, np.nan)
        rel = np.full(3, np.nan)
        range_m = float("nan")
        if args.diagnostic_truth:
            intruder_kin = _truth_kinematics(client, args.intruder)
            intruder_pos = _world_position(intruder_kin, args.intruder, origins)
            rel = intruder_pos - interceptor_pos
            range_m = float(np.linalg.norm(rel))
        pair_collision = get_vehicle_pair_collision(
            client,
            args.interceptor,
            args.intruder,
            interceptor_object_patterns=args.collision_interceptor_pattern,
            intruder_object_patterns=args.collision_intruder_pattern,
        )
        hit = pair_collision.collided

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
        yaw_rate_cmd_deg_s = 0.0
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
            yaw_rate_cmd_deg_s = _yaw_rate_from_pixel_error(px_err_x, intrinsics, args)

            lookup = attitude_buffer.lookup(frame_detection.exposure_ts)
            if not lookup.valid or lookup.sample is None:
                reason = lookup.reason or "attitude_lookup_failed"
            else:
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
                    terminal_reason = _terminal_trigger_strapdown(
                        reason,
                        detected,
                        bbox_area,
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

        stop_requested = _draw_detection_window_strapdown(
            display,
            client,
            config,
            detections,
            detection,
            intrinsics,
            yaw_rate_cmd_deg_s,
            valid,
            reason,
            ttc_value,
            args,
        )

        terminal_reason = _terminal_trigger_strapdown(reason, detected, bbox_area, intrinsics, args)
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
        if args.diagnostic_truth and np.all(np.isfinite(rel[:2])) and float(np.linalg.norm(rel[:2])) > 1.0e-6:
            target_bearing = _bearing_deg_from_xy(rel[:2])
            target_bearing_deg = target_bearing
            target_body_bearing_deg = _wrap_angle_deg(target_bearing - body_yaw_deg)
        if args.enable_motion:
            client.moveByVelocityAsync(
                float(v_cmd[0]),
                float(v_cmd[1]),
                float(v_cmd[2]),
                duration=command_duration,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate_cmd_deg_s),
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
                "collision_reason": pair_collision.reason,
                "interceptor_collision_object": pair_collision.interceptor_object_name,
                "intruder_collision_object": pair_collision.intruder_object_name,
                "pixel_error_x": px_err_x,
                "pixel_error_y": px_err_y,
                "bbox_area": bbox_area,
                "yaw_rate_cmd_deg_s": yaw_rate_cmd_deg_s,
                "body_yaw_deg": body_yaw_deg,
                "cmd_yaw_deg": cmd_yaw_deg,
                "yaw_error_deg": yaw_error_deg,
                "target_bearing_deg": target_bearing_deg,
                "target_body_bearing_deg": target_body_bearing_deg,
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
                f"{yaw_rate_cmd_deg_s:.1f},{body_yaw_deg:.1f},{cmd_yaw_deg:.1f},"
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
    print(f"strapdown_vision_csv={csv_path}")
    if not args.no_plot and _plot_strapdown(rows, plot_path):
        print(f"strapdown_vision_plot={plot_path}")
    _summarize_run(rows)
    if not hit and rows:
        print(f"hit=False final_range={rows[-1]['range']:.3f}m")
    if display is not None:
        display["cv2"].destroyAllWindows()


if __name__ == "__main__":
    main()
