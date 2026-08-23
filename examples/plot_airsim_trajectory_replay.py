from __future__ import annotations

import argparse
import csv
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import airsim


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CSV = (
    PROJECT_ROOT
    / "logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_visual_s_maneuver_35_clock0p3_demo_fg_reboot_20260715_200959_r35_h30.csv"
)


@dataclass(frozen=True)
class TrajectorySample:
    index: int
    t: float
    interceptor: airsim.Vector3r
    actor: airsim.Vector3r
    hit: bool
    near_hit: bool
    range_m: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay a saved AirSim interception CSV as persistent trajectory markers. "
            "This script only draws points/lines; it does not start PX4 or command flight."
        )
    )
    parser.add_argument("--csv", default=str(DEFAULT_CSV), help="Strapdown CSV containing interceptor_* and intruder_* columns.")
    parser.add_argument("--host", default=os.environ.get("AIRSIM_RPC_HOST", "127.0.0.2"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("AIRSIM_RPC_PORT", "41451")))
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument("--wait", action=argparse.BooleanOptionalAction, default=True, help="Wait for AirSim RPC before drawing.")
    parser.add_argument("--wait-timeout-s", type=float, default=90.0)
    parser.add_argument("--flush", action=argparse.BooleanOptionalAction, default=True, help="Clear existing persistent plot markers first.")
    parser.add_argument("--animate", action=argparse.BooleanOptionalAction, default=True, help="Draw trajectory point by point for recording.")
    parser.add_argument("--interval-s", type=float, default=0.04, help="Real-time delay between plotted samples when --animate is used.")
    parser.add_argument("--use-log-timing", action="store_true", help="Use logged dt instead of fixed --interval-s.")
    parser.add_argument("--time-scale", type=float, default=1.0, help="Scale logged dt when --use-log-timing is enabled.")
    parser.add_argument("--stride", type=int, default=1, help="Use every Nth CSV sample.")
    parser.add_argument("--max-points", type=int, default=0, help="Limit samples after striding; 0 keeps all.")
    parser.add_argument("--line-thickness", type=float, default=8.0)
    parser.add_argument("--point-size", type=float, default=14.0)
    parser.add_argument("--marker-size", type=float, default=36.0)
    parser.add_argument("--label-scale", type=float, default=2.5)
    parser.add_argument("--duration-s", type=float, default=-1.0, help="AirSim marker duration; -1 keeps markers persistent.")
    parser.add_argument("--pose-vehicle", default="", help="Optional vehicle to place at the first interceptor point.")
    parser.add_argument("--no-pose-vehicle", dest="pose_vehicle", action="store_const", const="")
    parser.add_argument("--pose-cv-camera", action=argparse.BooleanOptionalAction, default=True, help="Place the ComputerVision camera to view the full trajectory.")
    parser.add_argument("--camera-back-m", type=float, default=70.0, help="Camera offset behind the trajectory center along -X.")
    parser.add_argument("--camera-side-m", type=float, default=55.0, help="Camera offset from the trajectory center along -Y.")
    parser.add_argument("--camera-low-m", type=float, default=28.0, help="Camera offset below the trajectory center in NED +Z.")
    parser.add_argument("--plot-los-every", type=int, default=20, help="Draw gray interceptor-actor connecting lines every N samples; 0 disables.")
    parser.add_argument("--start-delay-s", type=float, default=0.0, help="Delay before plotting starts, useful for screen recording.")
    parser.add_argument("--final-hold-s", type=float, default=0.0, help="Sleep after drawing before exiting.")
    return parser.parse_args()


def _finite_float(value: str) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _truthy(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def _vector_from_row(row: dict[str, str], prefix: str) -> Optional[airsim.Vector3r]:
    x = _finite_float(row.get(f"{prefix}_x", ""))
    y = _finite_float(row.get(f"{prefix}_y", ""))
    z = _finite_float(row.get(f"{prefix}_z", ""))
    if x is None or y is None or z is None:
        return None
    return airsim.Vector3r(x, y, z)


def load_samples(csv_path: Path, stride: int = 1, max_points: int = 0) -> list[TrajectorySample]:
    samples: list[TrajectorySample] = []
    stride = max(1, int(stride))
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row_index, row in enumerate(reader):
            if row_index % stride != 0:
                continue
            interceptor = _vector_from_row(row, "interceptor")
            actor = _vector_from_row(row, "intruder")
            if interceptor is None or actor is None:
                continue
            t = _finite_float(row.get("t", "")) or 0.0
            range_m = _finite_float(row.get("range", "")) or float("nan")
            samples.append(
                TrajectorySample(
                    index=row_index,
                    t=t,
                    interceptor=interceptor,
                    actor=actor,
                    hit=_truthy(row.get("hit", "")),
                    near_hit=_truthy(row.get("near_hit", "")),
                    range_m=range_m,
                )
            )
            if max_points > 0 and len(samples) >= max_points:
                break
    if not samples:
        raise RuntimeError(f"no finite trajectory rows found in {csv_path}")
    return samples


def wait_for_client(host: str, port: int, timeout_s: float, wait_timeout_s: float) -> airsim.VehicleClient:
    deadline = time.monotonic() + max(0.0, wait_timeout_s)
    last_error: Optional[BaseException] = None
    while True:
        client = airsim.VehicleClient(ip=host, port=port, timeout_value=timeout_s)
        try:
            client.confirmConnection()
            return client
        except BaseException as exc:
            last_error = exc
            if time.monotonic() >= deadline:
                raise RuntimeError(f"AirSim RPC not ready at {host}:{port}: {last_error}") from exc
            time.sleep(1.0)


def plot_static(
    client: airsim.VehicleClient,
    samples: list[TrajectorySample],
    *,
    duration_s: float,
    line_thickness: float,
    point_size: float,
    marker_size: float,
    label_scale: float,
    plot_los_every: int,
) -> None:
    interceptor_points = [sample.interceptor for sample in samples]
    actor_points = [sample.actor for sample in samples]
    client.simPlotLineStrip(interceptor_points, [0.0, 0.35, 1.0, 1.0], line_thickness, duration_s, True)
    client.simPlotLineStrip(actor_points, [1.0, 0.1, 0.0, 1.0], line_thickness, duration_s, True)
    client.simPlotPoints(interceptor_points, [0.0, 0.55, 1.0, 1.0], point_size, duration_s, True)
    client.simPlotPoints(actor_points, [1.0, 0.25, 0.0, 1.0], point_size, duration_s, True)
    _plot_key_markers(client, samples, duration_s, marker_size, label_scale)
    _plot_los_samples(client, samples, duration_s, max(0, int(plot_los_every)))


def plot_animated(
    client: airsim.VehicleClient,
    samples: list[TrajectorySample],
    *,
    duration_s: float,
    line_thickness: float,
    point_size: float,
    marker_size: float,
    label_scale: float,
    interval_s: float,
    use_log_timing: bool,
    time_scale: float,
    plot_los_every: int,
) -> None:
    _plot_key_markers(client, samples[:1], duration_s, marker_size, label_scale)
    last: Optional[TrajectorySample] = None
    for index, sample in enumerate(samples):
        if last is not None:
            client.simPlotLineStrip([last.interceptor, sample.interceptor], [0.0, 0.35, 1.0, 1.0], line_thickness, duration_s, True)
            client.simPlotLineStrip([last.actor, sample.actor], [1.0, 0.1, 0.0, 1.0], line_thickness, duration_s, True)
        client.simPlotPoints([sample.interceptor], [0.0, 0.55, 1.0, 1.0], point_size, duration_s, True)
        client.simPlotPoints([sample.actor], [1.0, 0.25, 0.0, 1.0], point_size, duration_s, True)
        if plot_los_every > 0 and index % plot_los_every == 0:
            client.simPlotLineStrip([sample.interceptor, sample.actor], [0.8, 0.8, 0.8, 0.55], 2.0, duration_s, True)
        if sample.hit or (sample.near_hit and index == len(samples) - 1):
            client.simPlotPoints([_midpoint(sample.interceptor, sample.actor)], [1.0, 1.0, 0.0, 1.0], marker_size, duration_s, True)
        if last is not None:
            delay = max(0.0, float(interval_s))
            if use_log_timing:
                delay = max(0.0, (sample.t - last.t) / max(1.0e-6, float(time_scale)))
            if delay > 0.0:
                time.sleep(delay)
        last = sample
    _plot_key_markers(client, samples, duration_s, marker_size, label_scale)


def _plot_key_markers(
    client: airsim.VehicleClient,
    samples: list[TrajectorySample],
    duration_s: float,
    marker_size: float,
    label_scale: float,
) -> None:
    if not samples:
        return
    first = samples[0]
    last = samples[-1]
    hit_sample = next((sample for sample in samples if sample.hit), None)
    client.simPlotPoints([first.interceptor, first.actor], [0.0, 1.0, 0.0, 1.0], marker_size, duration_s, True)
    client.simPlotPoints([last.interceptor, last.actor], [1.0, 1.0, 1.0, 1.0], marker_size, duration_s, True)
    labels = ["Interceptor start", "Actor start", "Interceptor end", "Actor end"]
    positions = [first.interceptor, first.actor, last.interceptor, last.actor]
    if hit_sample is not None:
        hit_point = _midpoint(hit_sample.interceptor, hit_sample.actor)
        client.simPlotPoints([hit_point], [1.0, 1.0, 0.0, 1.0], marker_size * 1.4, duration_s, True)
        labels.append(f"hit range={hit_sample.range_m:.2f}m")
        positions.append(hit_point)
    try:
        client.simPlotStrings(labels, positions, label_scale, [1.0, 1.0, 1.0, 1.0], duration_s)
    except Exception:
        pass


def _plot_los_samples(
    client: airsim.VehicleClient,
    samples: list[TrajectorySample],
    duration_s: float,
    every: int,
) -> None:
    if every <= 0:
        return
    points: list[airsim.Vector3r] = []
    for index, sample in enumerate(samples):
        if index % every == 0 or index == len(samples) - 1 or sample.hit:
            points.extend([sample.interceptor, sample.actor])
    if points:
        client.simPlotLineList(points, [0.8, 0.8, 0.8, 0.45], 2.0, duration_s, True)


def _midpoint(a: airsim.Vector3r, b: airsim.Vector3r) -> airsim.Vector3r:
    return airsim.Vector3r(
        0.5 * (a.x_val + b.x_val),
        0.5 * (a.y_val + b.y_val),
        0.5 * (a.z_val + b.z_val),
    )


def _pose_vehicle_at_start(client: airsim.VehicleClient, samples: list[TrajectorySample], vehicle_name: str) -> None:
    if not vehicle_name:
        return
    start = samples[0].interceptor
    pose = airsim.Pose(start, airsim.to_quaternion(0.0, 0.0, 0.0))
    try:
        client.enableApiControl(True, vehicle_name=vehicle_name)
        client.simSetVehiclePose(pose, True, vehicle_name=vehicle_name)
    except Exception as exc:
        print(f"warning: could not pose vehicle {vehicle_name!r}: {exc}")


def _pose_cv_camera(
    client: airsim.VehicleClient,
    samples: list[TrajectorySample],
    *,
    camera_back_m: float,
    camera_side_m: float,
    camera_low_m: float,
) -> None:
    points = [sample.interceptor for sample in samples] + [sample.actor for sample in samples]
    (min_x, max_x), (min_y, max_y), (min_z, max_z) = _bounds(points)
    center = airsim.Vector3r(
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )
    camera = airsim.Vector3r(
        center.x_val - abs(camera_back_m),
        center.y_val - abs(camera_side_m),
        center.z_val + abs(camera_low_m),
    )
    dx = center.x_val - camera.x_val
    dy = center.y_val - camera.y_val
    dz = center.z_val - camera.z_val
    yaw = math.atan2(dy, dx)
    horizontal = max(1.0e-6, math.hypot(dx, dy))
    pitch = math.atan2(-dz, horizontal)
    pose = airsim.Pose(camera, airsim.to_quaternion(pitch, 0.0, yaw))
    try:
        client.simSetVehiclePose(pose, True)
        print(
            "cv_camera_pose="
            f"({camera.x_val:.2f},{camera.y_val:.2f},{camera.z_val:.2f}) "
            f"pitch_deg={math.degrees(pitch):.1f} yaw_deg={math.degrees(yaw):.1f}"
        )
    except Exception as exc:
        print(f"warning: could not pose ComputerVision camera: {exc}")


def _bounds(points: Iterable[airsim.Vector3r]) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    point_list = list(points)
    xs = [p.x_val for p in point_list]
    ys = [p.y_val for p in point_list]
    zs = [p.z_val for p in point_list]
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv).expanduser().resolve()
    samples = load_samples(csv_path, stride=args.stride, max_points=args.max_points)
    client = wait_for_client(args.host, args.port, args.timeout_s, args.wait_timeout_s) if args.wait else airsim.VehicleClient(ip=args.host, port=args.port, timeout_value=args.timeout_s)

    if args.flush:
        client.simFlushPersistentMarkers()
    if args.pose_cv_camera:
        _pose_cv_camera(
            client,
            samples,
            camera_back_m=args.camera_back_m,
            camera_side_m=args.camera_side_m,
            camera_low_m=args.camera_low_m,
        )
    _pose_vehicle_at_start(client, samples, args.pose_vehicle)

    print(f"trajectory_csv={csv_path}")
    print(f"samples={len(samples)} t={samples[0].t:.3f}..{samples[-1].t:.3f}s")
    print(f"interceptor_bounds={_bounds(sample.interceptor for sample in samples)}")
    print(f"actor_bounds={_bounds(sample.actor for sample in samples)}")
    hit = next((sample for sample in samples if sample.hit), None)
    if hit is not None:
        print(f"hit_index={hit.index} hit_t={hit.t:.3f}s hit_range={hit.range_m:.3f}m")
    if args.start_delay_s > 0.0:
        print(f"plot_start_delay_s={args.start_delay_s:.1f}")
        time.sleep(args.start_delay_s)

    if args.animate:
        plot_animated(
            client,
            samples,
            duration_s=args.duration_s,
            line_thickness=args.line_thickness,
            point_size=args.point_size,
            marker_size=args.marker_size,
            label_scale=args.label_scale,
            interval_s=args.interval_s,
            use_log_timing=args.use_log_timing,
            time_scale=args.time_scale,
            plot_los_every=args.plot_los_every,
        )
    else:
        plot_static(
            client,
            samples,
            duration_s=args.duration_s,
            line_thickness=args.line_thickness,
            point_size=args.point_size,
            marker_size=args.marker_size,
            label_scale=args.label_scale,
            plot_los_every=args.plot_los_every,
        )
    print("plot_complete=1")
    if args.final_hold_s > 0.0:
        time.sleep(args.final_hold_s)


if __name__ == "__main__":
    main()
