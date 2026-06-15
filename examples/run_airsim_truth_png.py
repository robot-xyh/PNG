from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path
import site
import types
from typing import Sequence

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision_guidance.airsim_adapter import get_vehicle_pair_collision  # noqa: E402
from vision_guidance.truth_png import compute_truth_png, integrate_velocity_command  # noqa: E402


PROJECT_ROOT = Path(__file__).resolve().parents[1]
AIRSIM_SETTINGS_PATH = Path.home() / "Documents" / "AirSim" / "settings.json"
SETTINGS_EXAMPLE_PATH = PROJECT_ROOT / "config" / "airsim_blocks_settings.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run truth-state classic PNG baseline in AirSim Blocks.")
    parser.add_argument("--interceptor", default="Interceptor")
    parser.add_argument("--intruder", default="Intruder")
    parser.add_argument("--collision-interceptor-pattern", action="append", default=None)
    parser.add_argument("--collision-intruder-pattern", action="append", default=None)
    parser.add_argument("--intruder-speed", type=float, default=5.0)
    parser.add_argument("--intruder-vx", type=float, default=0.0)
    parser.add_argument("--intruder-vy", type=float, default=None)
    parser.add_argument("--intruder-vz", type=float, default=0.0)
    parser.add_argument("--speed-ratio", type=float, default=2.0)
    parser.add_argument("--navigation-constant", type=float, default=3.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--enable-motion", action="store_true", help="Apply AirSim velocity commands.")
    parser.add_argument("--reset", action=argparse.BooleanOptionalAction, default=True, help="Reset AirSim before the run.")
    parser.add_argument("--hit-radius-m", type=float, default=1.0, help="Deprecated; AirSim collision is the success criterion.")
    parser.add_argument("--max-accel", type=float, default=15.0)
    parser.add_argument("--min-speed-ratio", type=float, default=0.8)
    parser.add_argument(
        "--altitude-correction",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Add a truth-altitude velocity wrapper for AirSim multirotor control.",
    )
    parser.add_argument("--vertical-kp", type=float, default=1.5)
    parser.add_argument("--vertical-speed-limit", type=float, default=3.0)
    parser.add_argument("--climb-to-altitude", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--intercept-altitude-m", type=float, default=50.0)
    parser.add_argument("--climb-speed", type=float, default=5.0)
    parser.add_argument("--climb-timeout-s", type=float, default=60.0)
    parser.add_argument("--settle-s", type=float, default=2.0, help="Hover settle time after altitude preparation.")
    parser.add_argument("--settle-speed", type=float, default=0.5, help="Maximum speed treated as settled after hover.")
    parser.add_argument("--settle-timeout-s", type=float, default=8.0, help="Maximum extra hover-settle wait.")
    parser.add_argument("--list-vehicles", action="store_true", help="Print AirSim vehicle names and exit.")
    parser.add_argument(
        "--settings-path",
        default=str(SETTINGS_EXAMPLE_PATH),
        help="AirSim settings JSON used to recover each vehicle's world-frame start offset.",
    )
    parser.add_argument("--trajectory-dir", default=str(PROJECT_ROOT / "logs"))
    parser.add_argument("--trajectory-prefix", default="")
    parser.add_argument("--no-plot", action="store_true", help="Disable matplotlib trajectory output.")
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


def _vector_xyz(vector) -> np.ndarray:
    return np.array([float(vector.x_val), float(vector.y_val), float(vector.z_val)], dtype=float)


def _vehicle_truth_kinematics(client, vehicle_name: str):
    return client.simGetGroundTruthKinematics(vehicle_name=vehicle_name)


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
        if not isinstance(item, dict):
            print(f"vehicle_origin_missing={vehicle}; using zero origin")
            continue
        origins[vehicle] = np.array(
            [
                float(item.get("X", 0.0)),
                float(item.get("Y", 0.0)),
                float(item.get("Z", 0.0)),
            ],
            dtype=float,
        )
    return origins


def _world_position(kinematics, vehicle_name: str, origins: dict[str, np.ndarray]) -> np.ndarray:
    # AirSim reports vehicle kinematics.position in the vehicle start frame.
    # Add the configured vehicle start offset to compare multiple vehicles.
    return origins.get(vehicle_name, np.zeros(3, dtype=float)) + _vector_xyz(kinematics.position)


def _record_sample(
    rows: list[dict[str, float | int | str | bool]],
    experiment_fields: dict[str, float | int | str],
    t: float,
    interceptor_name: str,
    intruder_name: str,
    interceptor_pos: np.ndarray,
    intruder_pos: np.ndarray,
    interceptor_vel: np.ndarray,
    intruder_vel: np.ndarray,
    range_m: float,
    closing_speed: float,
    los: np.ndarray,
    omega_los: np.ndarray,
    acceleration: np.ndarray,
    v_cmd: np.ndarray,
    guidance_valid: bool,
    reject_reason: str,
    hit: bool,
    collision_reason: str,
    interceptor_collision_object: str,
    intruder_collision_object: str,
) -> None:
    row: dict[str, float | int | str | bool] = {
        **experiment_fields,
        "t": t,
        "range": range_m,
        "horizontal_range": float(np.linalg.norm((intruder_pos - interceptor_pos)[:2])),
        "vertical_error": float(intruder_pos[2] - interceptor_pos[2]),
        "closing_speed": closing_speed,
        "guidance_valid": int(guidance_valid),
        "reject_reason": reject_reason,
        "hit": int(hit),
        "collision_reason": collision_reason,
        "interceptor_collision_object": interceptor_collision_object,
        "intruder_collision_object": intruder_collision_object,
    }
    for prefix, vector in [
        ("los", los),
        ("omega_los", omega_los),
        ("a_cmd", acceleration),
        ("v_cmd", v_cmd),
        ("interceptor_pos", interceptor_pos),
        ("intruder_pos", intruder_pos),
        ("interceptor_vel", interceptor_vel),
        ("intruder_vel", intruder_vel),
    ]:
        row[f"{prefix}_x"] = float(vector[0])
        row[f"{prefix}_y"] = float(vector[1])
        row[f"{prefix}_z"] = float(vector[2])
    row["interceptor"] = interceptor_name
    row["intruder"] = intruder_name
    rows.append(row)


def _truth_experiment_fields(args, speed_cap: float, intruder_velocity: np.ndarray) -> dict[str, float | int | str]:
    return {
        "experiment_type": "truth_png",
        "speed_ratio": float(args.speed_ratio),
        "intruder_speed": float(args.intruder_speed),
        "intruder_speed_arg": float(args.intruder_speed),
        "intruder_vx": float(intruder_velocity[0]),
        "intruder_vy": float(intruder_velocity[1]),
        "intruder_vz": float(intruder_velocity[2]),
        "speed_cap": float(speed_cap),
        "navigation_constant": float(args.navigation_constant),
        "max_accel": float(args.max_accel),
        "min_speed_ratio": float(args.min_speed_ratio),
        "intercept_altitude_m": float(args.intercept_altitude_m),
        "intruder_altitude_offset_m": 0.0,
        "rate_hz": float(args.rate_hz),
        "duration_s": float(args.duration_s),
        "altitude_correction": int(bool(args.altitude_correction)),
        "vertical_kp": float(args.vertical_kp),
        "vertical_speed_limit": float(args.vertical_speed_limit),
    }


def _write_csv(rows: Sequence[dict[str, float | int | str | bool]], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys()) if rows else []
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        if not fields:
            return
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


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
    return value


def _row_float(row: dict[str, float | int | str | bool], key: str) -> float | None:
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
    speed_cap: float,
    intruder_velocity: np.ndarray,
    rows: Sequence[dict[str, float | int | str | bool]],
    hit: bool,
) -> Path:
    ranges = [value for row in rows if (value := _row_float(row, "range")) is not None]
    meta = {
        "script_name": Path(__file__).name,
        "experiment_type": "truth_png",
        "created_local_time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "csv_path": str(csv_path),
        "settings_path": str(args.settings_path),
        "vehicle_names": {"interceptor": args.interceptor, "intruder": args.intruder},
        "args": vars(args),
        "derived": {
            "speed_cap": float(speed_cap),
            "intruder_velocity_cmd": [float(value) for value in intruder_velocity],
            "frame_count": len(rows),
            "hit": bool(hit),
            "min_range_m": min(ranges) if ranges else None,
            "final_range_m": ranges[-1] if ranges else None,
        },
    }
    metadata_path = csv_path.with_name(f"{csv_path.stem}_meta.json")
    with metadata_path.open("w", encoding="utf-8") as stream:
        json.dump(_json_safe(meta), stream, ensure_ascii=False, indent=2, sort_keys=True)
    return metadata_path


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


def _rows_until_first_hit(rows: Sequence[dict[str, float | int | str | bool]]) -> Sequence[dict[str, float | int | str | bool]]:
    for index, row in enumerate(rows):
        try:
            hit = int(row.get("hit", 0)) == 1
        except (TypeError, ValueError):
            hit = False
        if hit:
            return rows[: index + 1]
    return rows


def _plot_truth_png(rows: Sequence[dict[str, float | int | str | bool]], plot_path: Path) -> bool:
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
        print(f"matplotlib plot unavailable ({exc}); CSV was still saved.")
        return False

    rows = _rows_until_first_hit(rows)
    if not rows:
        return False

    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(14, 7))
    if has_3d:
        ax_traj = fig.add_subplot(121, projection="3d")
    else:
        ax_traj = fig.add_subplot(121)
    ax_range = fig.add_subplot(122)

    interceptor_x = [float(row["interceptor_pos_x"]) for row in rows]
    interceptor_y = [float(row["interceptor_pos_y"]) for row in rows]
    interceptor_alt = [-float(row["interceptor_pos_z"]) for row in rows]
    intruder_x = [float(row["intruder_pos_x"]) for row in rows]
    intruder_y = [float(row["intruder_pos_y"]) for row in rows]
    intruder_alt = [-float(row["intruder_pos_z"]) for row in rows]

    if has_3d:
        ax_traj.plot(interceptor_x, interceptor_y, interceptor_alt, color="tab:blue", linewidth=2, label="Interceptor")
        ax_traj.plot(intruder_x, intruder_y, intruder_alt, color="tab:red", linewidth=2, label="Intruder")
        ax_traj.scatter(interceptor_x[0], interceptor_y[0], interceptor_alt[0], color="tab:blue", marker="o", s=32)
        ax_traj.scatter(intruder_x[0], intruder_y[0], intruder_alt[0], color="tab:red", marker="o", s=32)
        ax_traj.scatter(interceptor_x[-1], interceptor_y[-1], interceptor_alt[-1], color="tab:blue", marker="x", s=56)
        ax_traj.scatter(intruder_x[-1], intruder_y[-1], intruder_alt[-1], color="tab:red", marker="x", s=56)
        ax_traj.set_zlabel("Altitude / m")
        ax_traj.set_title("Truth PNG Trajectory")
    else:
        print("matplotlib 3D projection unavailable; writing top-down 2D trajectory plot.")
        ax_traj.plot(interceptor_x, interceptor_y, color="tab:blue", linewidth=2, label="Interceptor")
        ax_traj.plot(intruder_x, intruder_y, color="tab:red", linewidth=2, label="Intruder")
        ax_traj.scatter(interceptor_x[0], interceptor_y[0], color="tab:blue", marker="o", s=32)
        ax_traj.scatter(intruder_x[0], intruder_y[0], color="tab:red", marker="o", s=32)
        ax_traj.scatter(interceptor_x[-1], interceptor_y[-1], color="tab:blue", marker="x", s=56)
        ax_traj.scatter(intruder_x[-1], intruder_y[-1], color="tab:red", marker="x", s=56)
        ax_traj.set_aspect("equal", adjustable="box")
        ax_traj.set_title("Truth PNG Top-Down Trajectory")
    ax_traj.set_xlabel("NED X / m")
    ax_traj.set_ylabel("NED Y / m")
    ax_traj.legend()
    ax_traj.grid(True)

    times = [float(row["t"]) for row in rows]
    ranges = [float(row["range"]) for row in rows]
    ax_range.plot(times, ranges, color="tab:green", linewidth=2)
    ax_range.set_xlabel("Time / s")
    ax_range.set_ylabel("Range / m")
    ax_range.set_title("Range History")
    ax_range.grid(True)

    fig.subplots_adjust(left=0.04, right=0.98, bottom=0.09, top=0.92, wspace=0.28)
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)
    return True


def _output_paths(args) -> tuple[Path, Path]:
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    prefix = args.trajectory_prefix or f"truth_png_trajectory_{timestamp}"
    base = Path(args.trajectory_dir)
    return base / f"{prefix}.csv", base / f"{prefix}.png"


def _prepare_intercept_altitude(client, vehicles: Sequence[str], args) -> None:
    if not args.climb_to_altitude:
        return

    target_z = -abs(args.intercept_altitude_m)
    print(
        f"Climbing vehicles to intercept altitude: {args.intercept_altitude_m:.1f} m "
        f"(AirSim NED Z={target_z:.1f})"
    )
    for future in [
        client.takeoffAsync(timeout_sec=args.climb_timeout_s, vehicle_name=vehicle)
        for vehicle in vehicles
    ]:
        future.join()
    for future in [
        client.moveToZAsync(
            target_z,
            velocity=args.climb_speed,
            timeout_sec=args.climb_timeout_s,
            vehicle_name=vehicle,
        )
        for vehicle in vehicles
    ]:
        future.join()
    for vehicle in vehicles:
        client.hoverAsync(vehicle_name=vehicle).join()
    if args.settle_s > 0.0:
        time.sleep(args.settle_s)
    settle_start = time.monotonic()
    while time.monotonic() - settle_start < args.settle_timeout_s:
        speeds = []
        for vehicle in vehicles:
            velocity = _vector_xyz(_vehicle_truth_kinematics(client, vehicle).linear_velocity)
            speeds.append(float(np.linalg.norm(velocity)))
        if speeds and max(speeds) <= args.settle_speed:
            break
        for vehicle in vehicles:
            client.hoverAsync(vehicle_name=vehicle)
        time.sleep(0.2)
    print("Altitude preparation complete; starting truth-PNG loop.")


def _intruder_velocity(args) -> np.ndarray:
    vy = args.intruder_speed if args.intruder_vy is None else args.intruder_vy
    return np.array([args.intruder_vx, vy, args.intruder_vz], dtype=float)


def _horizontal_direction(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    horizontal = np.array([float(vector[0]), float(vector[1]), 0.0], dtype=float)
    norm = float(np.linalg.norm(horizontal))
    if norm > 1.0e-6:
        return horizontal / norm
    return fallback


def _apply_altitude_correction(
    v_cmd: np.ndarray,
    relative_position: np.ndarray,
    intruder_velocity: np.ndarray,
    speed_cap: float,
    args,
) -> np.ndarray:
    if not args.altitude_correction:
        return v_cmd

    corrected = np.array(v_cmd, dtype=float, copy=True)
    vertical_limit = max(0.0, min(float(args.vertical_speed_limit), speed_cap))
    vertical_cmd = intruder_velocity[2] + float(np.clip(args.vertical_kp * relative_position[2], -vertical_limit, vertical_limit))
    corrected[2] = float(np.clip(vertical_cmd, -vertical_limit, vertical_limit))

    horizontal_norm = float(np.linalg.norm(corrected[:2]))
    max_horizontal = float(np.sqrt(max(0.0, speed_cap * speed_cap - corrected[2] * corrected[2])))
    if horizontal_norm > max_horizontal and horizontal_norm > 1.0e-6:
        corrected[:2] *= max_horizontal / horizontal_norm
    return corrected


def _print_status(
    frame: int,
    t: float,
    result,
    rel_pos: np.ndarray,
    accel: np.ndarray,
    v_cmd: np.ndarray,
    hit: bool,
) -> None:
    print(
        f"{frame},{t:.3f},{result.range_m:.3f},{np.linalg.norm(rel_pos[:2]):.3f},{rel_pos[2]:.3f},"
        f"{result.closing_speed:.3f},"
        f"{result.valid},{result.reject_reason or ''},"
        f"{np.linalg.norm(accel):.3f},{np.linalg.norm(v_cmd):.3f},{int(hit)}"
    )


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
        raise SystemExit(
            "Failed to connect to AirSim RPC. Start Blocks first with ./run_blocks_nvidia.sh, "
            "wait until the scene finishes loading, then rerun this script."
        ) from exc
    available = list(client.listVehicles())
    print(f"AirSim vehicles: {_format_names(available)}")
    if args.list_vehicles:
        return

    _require_vehicles(client, [args.interceptor, args.intruder])
    vehicle_origins = _load_vehicle_origins(args.settings_path, [args.interceptor, args.intruder])
    print(
        "Vehicle world origins from settings: "
        f"{args.interceptor}={vehicle_origins[args.interceptor].tolist()}, "
        f"{args.intruder}={vehicle_origins[args.intruder].tolist()}"
    )

    if args.reset:
        client.reset()
        time.sleep(1.0)

    for vehicle in [args.interceptor, args.intruder]:
        try:
            client.enableApiControl(True, vehicle_name=vehicle)
            client.armDisarm(True, vehicle_name=vehicle)
        except Exception as exc:
            raise SystemExit(
                f"Failed to initialize vehicle {vehicle!r}: {exc}\n"
                f"Available vehicles: {_format_names(available)}\n"
                "If you just changed AirSim settings, restart Blocks before rerunning this script."
            ) from exc

    _prepare_intercept_altitude(client, [args.interceptor, args.intruder], args)

    initial_interceptor_kin = _vehicle_truth_kinematics(client, args.interceptor)
    initial_intruder_kin = _vehicle_truth_kinematics(client, args.intruder)
    initial_interceptor_pos = _world_position(initial_interceptor_kin, args.interceptor, vehicle_origins)
    initial_intruder_pos = _world_position(initial_intruder_kin, args.intruder, vehicle_origins)
    initial_range = float(np.linalg.norm(initial_intruder_pos - initial_interceptor_pos))
    print(
        "Initial world positions after altitude prep: "
        f"{args.interceptor}={initial_interceptor_pos.tolist()}, "
        f"{args.intruder}={initial_intruder_pos.tolist()}, "
        f"range={initial_range:.3f} m"
    )

    dt = 1.0 / args.rate_hz
    intruder_velocity = _intruder_velocity(args)
    intruder_speed_norm = float(np.linalg.norm(intruder_velocity))
    configured_intruder_speed = intruder_speed_norm if intruder_speed_norm > 0.0 else args.intruder_speed
    speed_cap = args.speed_ratio * configured_intruder_speed
    min_speed = max(0.0, min(speed_cap, args.min_speed_ratio * speed_cap))
    default_fallback_direction = np.array([1.0, 0.0, 0.0], dtype=float)
    experiment_fields = _truth_experiment_fields(args, speed_cap, intruder_velocity)

    csv_path, plot_path = _output_paths(args)
    rows: list[dict[str, float | int | str | bool]] = []
    start = time.monotonic()
    frame = 0
    hit = False
    last_range = float("inf")

    print(
        "Truth PNG baseline: "
        f"N={args.navigation_constant:.2f}, intruder_velocity={intruder_velocity.tolist()}, "
        f"speed_cap={speed_cap:.2f} m/s, max_accel={args.max_accel:.2f} m/s^2"
    )
    print("frame,t,range,horizontal_range,vertical_error,closing_speed,valid,reject_reason,a_norm,v_cmd_norm,hit")

    while time.monotonic() - start < args.duration_s:
        now = time.monotonic()
        sim_t = now - start

        if args.enable_motion:
            client.moveByVelocityAsync(
                float(intruder_velocity[0]),
                float(intruder_velocity[1]),
                float(intruder_velocity[2]),
                duration=dt,
                vehicle_name=args.intruder,
            )

        interceptor_kin = _vehicle_truth_kinematics(client, args.interceptor)
        intruder_kin = _vehicle_truth_kinematics(client, args.intruder)
        interceptor_pos = _world_position(interceptor_kin, args.interceptor, vehicle_origins)
        intruder_pos = _world_position(intruder_kin, args.intruder, vehicle_origins)
        interceptor_vel = _vector_xyz(interceptor_kin.linear_velocity)
        intruder_vel = _vector_xyz(intruder_kin.linear_velocity)

        relative_position = intruder_pos - interceptor_pos
        result = compute_truth_png(
            relative_position,
            intruder_vel - interceptor_vel,
            navigation_constant=args.navigation_constant,
            max_accel=args.max_accel,
        )
        accel = result.acceleration
        fallback_direction = _horizontal_direction(relative_position, default_fallback_direction)
        v_cmd = integrate_velocity_command(
            interceptor_vel,
            accel,
            dt,
            speed_cap=speed_cap,
            min_speed=min_speed,
            fallback_direction=fallback_direction,
        )
        v_cmd = _apply_altitude_correction(v_cmd, relative_position, intruder_vel, speed_cap, args)

        pair_collision = get_vehicle_pair_collision(
            client,
            args.interceptor,
            args.intruder,
            interceptor_object_patterns=args.collision_interceptor_pattern,
            intruder_object_patterns=args.collision_intruder_pattern,
        )
        hit = pair_collision.collided
        _record_sample(
            rows,
            experiment_fields,
            sim_t,
            args.interceptor,
            args.intruder,
            interceptor_pos,
            intruder_pos,
            interceptor_vel,
            intruder_vel,
            result.range_m,
            result.closing_speed,
            result.los,
            result.omega_los,
            accel,
            v_cmd,
            result.valid,
            result.reject_reason or "",
            hit,
            pair_collision.reason,
            pair_collision.interceptor_object_name,
            pair_collision.intruder_object_name,
        )
        _print_status(frame, sim_t, result, relative_position, accel, v_cmd, hit)

        if hit:
            print(
                f"hit=True collision=True reason={pair_collision.reason} "
                f"range={result.range_m:.3f}m t={sim_t:.3f}s"
            )
            break

        if args.enable_motion:
            client.moveByVelocityAsync(
                float(v_cmd[0]),
                float(v_cmd[1]),
                float(v_cmd[2]),
                duration=dt,
                vehicle_name=args.interceptor,
            )

        last_range = result.range_m
        time.sleep(dt)
        frame += 1

    _write_csv(rows, csv_path)
    print(f"truth_png_csv={csv_path}")
    metadata_path = _write_run_metadata(
        args=args,
        csv_path=csv_path,
        speed_cap=speed_cap,
        intruder_velocity=intruder_velocity,
        rows=rows,
        hit=hit,
    )
    print(f"truth_png_meta={metadata_path}")
    if not args.no_plot and _plot_truth_png(rows, plot_path):
        print(f"truth_png_plot={plot_path}")
    if not hit:
        print(f"hit=False final_range={last_range:.3f}m")


if __name__ == "__main__":
    main()
