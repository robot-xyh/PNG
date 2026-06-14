from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
import site
import types
from typing import Sequence

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
from vision_guidance.fusion import PureVisionGuidancePipeline  # noqa: E402
from vision_guidance.geometry import camera_to_body_mount  # noqa: E402
from vision_guidance.types import AttitudeSample  # noqa: E402


PROJECT_ROOT = Path(__file__).resolve().parents[1]
AIRSIM_SETTINGS_PATH = Path.home() / "Documents" / "AirSim" / "settings.json"
SETTINGS_EXAMPLE_PATH = PROJECT_ROOT / "config" / "airsim_blocks_settings.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run pure-vision PNG evaluation in AirSim Blocks.")
    parser.add_argument("--interceptor", default="Interceptor")
    parser.add_argument("--intruder", default="Intruder")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--mesh", default="Intruder*")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fov-deg", type=float, default=120.0)
    parser.add_argument("--camera-pitch-up-deg", type=float, default=0.0)
    parser.add_argument("--intruder-speed", type=float, default=5.0)
    parser.add_argument("--speed-ratio", type=float, default=2.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--enable-motion", action="store_true", help="Apply bounded AirSim velocity commands.")
    parser.add_argument("--max-lateral-speed", type=float, default=5.0)
    parser.add_argument("--list-vehicles", action="store_true", help="Print AirSim vehicle names and exit.")
    parser.add_argument("--spawn-intruder", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--climb-to-altitude", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--intercept-altitude-m", type=float, default=50.0)
    parser.add_argument("--climb-speed", type=float, default=5.0)
    parser.add_argument("--climb-timeout-s", type=float, default=60.0)
    parser.add_argument("--takeoff", action="store_true", help="Run AirSim takeoffAsync after arming.")
    parser.add_argument("--intruder-x", type=float, default=35.0)
    parser.add_argument("--intruder-y", type=float, default=-20.0)
    parser.add_argument("--intruder-z", type=float, default=-2.0)
    parser.add_argument("--intruder-yaw-deg", type=float, default=90.0)
    parser.add_argument("--trajectory-dir", default=str(PROJECT_ROOT / "logs"))
    parser.add_argument("--trajectory-prefix", default="")
    parser.add_argument("--no-plot", action="store_true", help="Disable matplotlib 3D trajectory output.")
    return parser.parse_args()


def _format_names(names: Sequence[str]) -> str:
    if not names:
        return "(none)"
    return ", ".join(repr(name) for name in names)


def _require_vehicles(client, required: Sequence[str]) -> list[str]:
    try:
        available = list(client.listVehicles())
    except Exception as exc:
        raise SystemExit(f"Failed to list AirSim vehicles: {exc}") from exc

    missing = [name for name in required if name not in available]
    if missing:
        raise SystemExit(
            "AirSim vehicle configuration does not match this example.\n"
            f"Required vehicles: {_format_names(required)}\n"
            f"Available vehicles: {_format_names(available)}\n\n"
            "Fix one of these:\n"
            f"1. Copy {SETTINGS_EXAMPLE_PATH} to {AIRSIM_SETTINGS_PATH}, then restart Blocks.\n"
            "2. Or pass the actual vehicle names with --interceptor and --intruder.\n\n"
            "The current AirSim settings must define the required Multirotor/SimpleFlight vehicles."
        )
    return available


def _spawn_intruder_if_needed(client, args, airsim_module) -> list[str]:
    available = list(client.listVehicles())
    if args.intruder in available:
        return available
    if not args.spawn_intruder:
        return _require_vehicles(client, [args.interceptor, args.intruder])

    pose = airsim_module.Pose(
        airsim_module.Vector3r(args.intruder_x, args.intruder_y, args.intruder_z),
        airsim_module.to_quaternion(0.0, 0.0, np.deg2rad(args.intruder_yaw_deg)),
    )
    created = client.simAddVehicle(args.intruder, "simpleflight", pose)
    if not created:
        raise SystemExit(
            f"Failed to spawn intruder vehicle {args.intruder!r} with simAddVehicle.\n"
            f"Available vehicles: {_format_names(available)}\n"
            "Try restarting Blocks, or define the intruder explicitly in AirSim settings.json."
        )
    time.sleep(0.5)
    available = list(client.listVehicles())
    print(f"Spawned intruder vehicle {args.intruder!r}.")
    return _require_vehicles(client, [args.interceptor, args.intruder])


def _resolve_interceptor_name(client, requested: str) -> str:
    available = list(client.listVehicles())
    if requested != "auto":
        _require_vehicles(client, [requested])
        return requested
    if not available:
        raise SystemExit(
            "No AirSim multirotor vehicles are available.\n"
            "Start Blocks with ./run_blocks_nvidia.sh and wait until the scene finishes loading."
        )
    if "Interceptor" in available:
        return "Interceptor"
    if "SimpleFlight" in available:
        return "SimpleFlight"
    if "" in available:
        return ""
    return available[0]


def _vehicle_kinematics(client, vehicle_name: str):
    return client.getMultirotorState(vehicle_name=vehicle_name).kinematics_estimated


def _vector_xyz(vector) -> tuple[float, float, float]:
    return float(vector.x_val), float(vector.y_val), float(vector.z_val)


def _record_trajectory_sample(rows: list[dict[str, float | int | str]], client, vehicles: Sequence[str], timestamp: float, phase: str) -> None:
    for vehicle in vehicles:
        kinematics = _vehicle_kinematics(client, vehicle)
        x, y, z = _vector_xyz(kinematics.position)
        vx, vy, vz = _vector_xyz(kinematics.linear_velocity)
        rows.append(
            {
                "t": timestamp,
                "phase": phase,
                "vehicle": vehicle,
                "x": x,
                "y": y,
                "z": z,
                "altitude": -z,
                "vx": vx,
                "vy": vy,
                "vz": vz,
            }
        )


def _write_trajectory_csv(rows: Sequence[dict[str, float | int | str]], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["t", "phase", "vehicle", "x", "y", "z", "altitude", "vx", "vy", "vz"]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


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


def _plot_trajectory(rows: Sequence[dict[str, float | int | str]], plot_path: Path, interceptor: str, intruder: str) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        _prefer_user_mpl_toolkits()
        import matplotlib.pyplot as plt
        try:
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

            has_3d = True
        except Exception:
            has_3d = False
    except Exception as exc:
        print(f"matplotlib plot unavailable ({exc}); trajectory CSV was still saved.")
        return False

    if not rows:
        return False

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(10, 7))
    if has_3d:
        ax = fig.add_subplot(111, projection="3d")
    else:
        print("matplotlib 3D projection unavailable; writing top-down 2D trajectory plot.")
        ax = fig.add_subplot(111)

    for vehicle, color in [(interceptor, "tab:blue"), (intruder, "tab:red")]:
        vehicle_rows = [row for row in rows if row["vehicle"] == vehicle]
        if not vehicle_rows:
            continue
        xs = [float(row["x"]) for row in vehicle_rows]
        ys = [float(row["y"]) for row in vehicle_rows]
        alts = [float(row["altitude"]) for row in vehicle_rows]
        if has_3d:
            ax.plot(xs, ys, alts, color=color, linewidth=2, label=vehicle)
            ax.scatter(xs[0], ys[0], alts[0], color=color, marker="o", s=32)
            ax.scatter(xs[-1], ys[-1], alts[-1], color=color, marker="x", s=48)
        else:
            ax.plot(xs, ys, color=color, linewidth=2, label=vehicle)
            ax.scatter(xs[0], ys[0], color=color, marker="o", s=32)
            ax.scatter(xs[-1], ys[-1], color=color, marker="x", s=48)

    ax.set_xlabel("NED X / m")
    ax.set_ylabel("NED Y / m")
    if has_3d:
        ax.set_zlabel("Altitude / m")
        ax.set_title("AirSim Intercept Trajectory")
    else:
        ax.set_aspect("equal", adjustable="box")
        ax.set_title("AirSim Intercept Top-Down Trajectory")
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    return True


def _trajectory_paths(args) -> tuple[Path, Path]:
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    prefix = args.trajectory_prefix or f"airsim_trajectory_{timestamp}"
    base = Path(args.trajectory_dir)
    return base / f"{prefix}.csv", base / f"{prefix}.png"


def _prepare_intercept_altitude(client, vehicles: Sequence[str], args) -> None:
    if not args.climb_to_altitude:
        if args.takeoff:
            for vehicle in vehicles:
                client.takeoffAsync(vehicle_name=vehicle).join()
        return

    target_z = -abs(args.intercept_altitude_m)
    print(
        f"Climbing vehicles to intercept altitude: {args.intercept_altitude_m:.1f} m "
        f"(AirSim NED Z={target_z:.1f})"
    )

    takeoff_futures = [
        client.takeoffAsync(timeout_sec=args.climb_timeout_s, vehicle_name=vehicle)
        for vehicle in vehicles
    ]
    for future in takeoff_futures:
        future.join()

    climb_futures = [
        client.moveToZAsync(
            target_z,
            velocity=args.climb_speed,
            timeout_sec=args.climb_timeout_s,
            vehicle_name=vehicle,
        )
        for vehicle in vehicles
    ]
    for future in climb_futures:
        future.join()

    for vehicle in vehicles:
        client.hoverAsync(vehicle_name=vehicle).join()
    print("Altitude preparation complete; starting intercept loop.")


def main() -> None:
    args = parse_args()
    try:
        import airsim
    except ImportError as exc:
        raise SystemExit("Install the AirSim Python package before running this example.") from exc

    client = airsim.MultirotorClient()
    client.confirmConnection()

    available_vehicles = list(client.listVehicles())
    print(f"AirSim vehicles: {_format_names(available_vehicles)}")
    if args.list_vehicles:
        return

    args.interceptor = _resolve_interceptor_name(client, args.interceptor)
    print(f"Using interceptor vehicle: {args.interceptor!r}")
    available_vehicles = _spawn_intruder_if_needed(client, args, airsim)
    print(f"Using AirSim vehicles: {_format_names(available_vehicles)}")
    active_vehicles = [args.interceptor, args.intruder]
    trajectory_rows: list[dict[str, float | int | str]] = []
    csv_path, plot_path = _trajectory_paths(args)

    for vehicle in [args.interceptor, args.intruder]:
        try:
            client.enableApiControl(True, vehicle_name=vehicle)
            client.armDisarm(True, vehicle_name=vehicle)
        except Exception as exc:
            raise SystemExit(
                f"Failed to initialize vehicle {vehicle!r}: {exc}\n"
                f"Available vehicles: {_format_names(available_vehicles)}\n"
                "If you just changed settings.json, restart Blocks before rerunning this script."
            ) from exc

    _record_trajectory_sample(trajectory_rows, client, active_vehicles, 0.0, "spawn")
    _prepare_intercept_altitude(client, active_vehicles, args)
    _record_trajectory_sample(trajectory_rows, client, active_vehicles, 0.0, "ready")

    config = AirSimDetectionConfig(
        camera_name=args.camera,
        mesh_name_pattern=args.mesh,
        vehicle_name=args.interceptor,
    )
    configure_detection_filter(client, config)

    intrinsics = infer_intrinsics_from_fov(args.width, args.height, args.fov_deg)
    attitude_buffer = AttitudeHistoryBuffer(duration_s=2.0)
    pipeline = PureVisionGuidancePipeline(
        intrinsics=intrinsics,
        R_BC=camera_to_body_mount(args.camera_pitch_up_deg),
        attitude_buffer=attitude_buffer,
    )

    dt = 1.0 / args.rate_hz
    interceptor_speed = args.speed_ratio * args.intruder_speed
    start = time.monotonic()
    frame_id = 0

    print("frame,t,detected,ttc,valid,quality,g_eval")
    while time.monotonic() - start < args.duration_s:
        now = time.monotonic()
        sim_t = now - start

        intruder_velocity = (0.0, args.intruder_speed, 0.0)
        client.moveByVelocityAsync(
            *intruder_velocity,
            duration=dt,
            vehicle_name=args.intruder,
        )

        state = client.getMultirotorState(vehicle_name=args.interceptor)
        R_IB = airsim_orientation_to_R_IB(state.kinematics_estimated.orientation)
        attitude_buffer.push(AttitudeSample(timestamp=sim_t, R_IB=R_IB))
        _record_trajectory_sample(trajectory_rows, client, active_vehicles, sim_t, "intercept")

        detections = list(get_detections(client, config))
        detection = choose_detection(detections)
        if detection is None:
            print(f"{frame_id},{sim_t:.3f},0,,,,")
            if args.enable_motion:
                client.moveByVelocityAsync(interceptor_speed, 0.0, 0.0, duration=dt, vehicle_name=args.interceptor)
            time.sleep(dt)
            frame_id += 1
            continue

        frame_detection = detection_to_frame_detection(
            detection=detection,
            frame_id=frame_id,
            exposure_ts=sim_t,
            track_id=1,
            score=1.0,
        )
        result = pipeline.process(frame_detection)
        g_eval = result.guidance.g_eval

        if args.enable_motion and result.guidance.valid:
            lateral = np.clip(g_eval[:2], -args.max_lateral_speed, args.max_lateral_speed)
            client.moveByVelocityAsync(
                interceptor_speed,
                float(lateral[0]),
                float(lateral[1]),
                duration=dt,
                vehicle_name=args.interceptor,
            )
        elif args.enable_motion:
            client.moveByVelocityAsync(interceptor_speed, 0.0, 0.0, duration=dt, vehicle_name=args.interceptor)

        ttc_text = "" if result.ttc is None or result.ttc.ttc is None else f"{result.ttc.ttc:.3f}"
        print(
            f"{frame_id},{sim_t:.3f},1,{ttc_text},{result.guidance.valid},"
            f"{result.guidance.quality:.3f},{np.array2string(g_eval, precision=4)}"
        )

        time.sleep(dt)
        frame_id += 1

    _write_trajectory_csv(trajectory_rows, csv_path)
    print(f"trajectory_csv={csv_path}")
    if not args.no_plot:
        if _plot_trajectory(trajectory_rows, plot_path, args.interceptor, args.intruder):
            print(f"trajectory_plot={plot_path}")


if __name__ == "__main__":
    main()
