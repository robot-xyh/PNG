from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Sequence

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision_guidance.betaflight_sitl import (  # noqa: E402
    BodyRateRCConfig,
    BetaflightMSPClient,
    BetaflightRCCommand,
    RateCommand,
)
from vision_guidance.truth_png import compute_truth_png  # noqa: E402


PROJECT_ROOT = Path(__file__).resolve().parents[1]
GRAVITY_MPS2 = 9.80665


@dataclass(frozen=True)
class GazeboPoseSample:
    wall_time_s: float
    sim_time_s: float | None
    position_enu: np.ndarray
    velocity_enu: np.ndarray
    quat_wxyz: np.ndarray


class GazeboPoseSubscriber:
    def __init__(self, *, world: str, model: str) -> None:
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self.topic = f"/world/{world}/pose/info"
        self.model = model
        self._lock = Lock()
        self._sample: GazeboPoseSample | None = None
        self._last_position: np.ndarray | None = None
        self._last_sim_time_s: float | None = None
        self._node = Node()
        if not self._node.subscribe(Pose_V, self.topic, self._callback):
            raise RuntimeError(f"failed to subscribe Gazebo topic {self.topic}")

    def _callback(self, msg) -> None:
        sim_time = None
        if msg.header.HasField("stamp"):
            sim_time = float(msg.header.stamp.sec) + 1.0e-9 * float(msg.header.stamp.nsec)
        for pose in msg.pose:
            if pose.name != self.model:
                continue
            pos = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            quat = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z], dtype=float)
            vel = np.zeros(3, dtype=float)
            if self._last_position is not None and sim_time is not None and self._last_sim_time_s is not None:
                dt = sim_time - self._last_sim_time_s
                if dt > 1.0e-5:
                    vel = (pos - self._last_position) / dt
            self._last_position = pos
            self._last_sim_time_s = sim_time
            with self._lock:
                self._sample = GazeboPoseSample(time.monotonic(), sim_time, pos, vel, quat)
            return

    def latest(self) -> GazeboPoseSample | None:
        with self._lock:
            return self._sample

    def wait_latest(self, timeout_s: float) -> GazeboPoseSample:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            sample = self.latest()
            if sample is not None:
                return sample
            time.sleep(0.02)
        raise TimeoutError(f"no pose for model {self.model!r} on {self.topic}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run truth PNG directly in Gazebo with Betaflight SITL MSP RC injection.")
    parser.add_argument("--world", default="betaloop_demo")
    parser.add_argument("--model", default="iris")
    parser.add_argument("--betaflight-host", default="127.0.0.1")
    parser.add_argument("--betaflight-msp-port", type=int, default=5761)
    parser.add_argument("--msp-timeout-s", type=float, default=0.2)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--prime-s", type=float, default=2.5)
    parser.add_argument("--arm-s", type=float, default=3.0)
    parser.add_argument("--spool-s", type=float, default=1.0)
    parser.add_argument("--spool-thrust", type=float, default=0.8)
    parser.add_argument("--spool-min-dz", type=float, default=0.2)
    parser.add_argument("--spool-retry-thrust", type=float, default=1.0)
    parser.add_argument("--spool-retry-s", type=float, default=1.0)
    parser.add_argument("--spool-max-retries", type=int, default=2)
    parser.add_argument("--start-range-m", type=float, default=80.0)
    parser.add_argument("--start-lateral-m", type=float, default=0.0)
    parser.add_argument("--start-frame", choices=("body", "world"), default="body")
    parser.add_argument("--target-altitude-offset-m", type=float, default=1.0)
    parser.add_argument("--target-velocity-frame", choices=("body", "world"), default="body")
    parser.add_argument("--target-vx", type=float, default=-5.0)
    parser.add_argument("--target-vy", type=float, default=0.0)
    parser.add_argument("--target-vz", type=float, default=0.0)
    parser.add_argument("--interceptor-speed", type=float, default=4.0)
    parser.add_argument("--navigation-constant", type=float, default=3.0)
    parser.add_argument("--max-png-accel", type=float, default=12.0)
    parser.add_argument("--speed-hold-kp", type=float, default=1.2)
    parser.add_argument("--max-speed-hold-accel", type=float, default=1.2)
    parser.add_argument("--altitude-kp", type=float, default=1.5)
    parser.add_argument("--altitude-kd", type=float, default=2.0)
    parser.add_argument("--max-total-accel", type=float, default=3.0)
    parser.add_argument("--max-tilt-deg", type=float, default=6.0)
    parser.add_argument("--attitude-p", type=float, default=2.5)
    parser.add_argument("--yaw-p", type=float, default=0.8)
    parser.add_argument("--max-yaw-rate-deg-s", type=float, default=35.0)
    parser.add_argument("--hover-thrust", type=float, default=0.76)
    parser.add_argument("--thrust-gain", type=float, default=0.25)
    parser.add_argument("--min-thrust", type=float, default=0.0)
    parser.add_argument("--max-thrust", type=float, default=0.92)
    parser.add_argument("--roll-accel-sign", type=float, default=-1.0)
    parser.add_argument("--pitch-accel-sign", type=float, default=1.0)
    parser.add_argument("--roll-rc-sign", type=float, default=1.0)
    parser.add_argument("--pitch-rc-sign", type=float, default=1.0)
    parser.add_argument("--yaw-rc-sign", type=float, default=1.0)
    parser.add_argument("--max-roll-rate-deg-s", type=float, default=200.0)
    parser.add_argument("--max-pitch-rate-deg-s", type=float, default=200.0)
    parser.add_argument("--max-yaw-rc-rate-deg-s", type=float, default=200.0)
    parser.add_argument("--hit-radius-m", type=float, default=1.0)
    parser.add_argument("--msp-diagnostic-every-n", type=int, default=10)
    parser.add_argument("--trajectory-dir", default=str(PROJECT_ROOT / "logs" / "gazebo_betaflight_truth_png"))
    parser.add_argument("--trajectory-prefix", default="")
    parser.add_argument("--print-every-n", type=int, default=10)
    return parser.parse_args()


def _clip_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if max_norm <= 0.0 or norm <= max_norm or norm <= 1.0e-9:
        return vector
    return vector * (float(max_norm) / norm)


def _wrap_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _euler_from_quat_wxyz(quat_wxyz: Sequence[float]) -> tuple[float, float, float]:
    w, x, y, z = np.asarray(quat_wxyz, dtype=float)
    norm = float(np.linalg.norm([w, x, y, z]))
    if norm <= 1.0e-12:
        return 0.0, 0.0, 0.0
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _rc_config(args: argparse.Namespace) -> BodyRateRCConfig:
    return BodyRateRCConfig(
        max_roll_rate_rad_s=math.radians(float(args.max_roll_rate_deg_s)),
        max_pitch_rate_rad_s=math.radians(float(args.max_pitch_rate_deg_s)),
        max_yaw_rate_rad_s=math.radians(float(args.max_yaw_rc_rate_deg_s)),
        roll_sign=float(args.roll_rc_sign),
        pitch_sign=float(args.pitch_rc_sign),
        yaw_sign=float(args.yaw_rc_sign),
    )


def _command_from_accel(accel_enu: np.ndarray, quat_wxyz: np.ndarray, rel_pos: np.ndarray, args: argparse.Namespace) -> tuple[RateCommand, dict[str, float]]:
    roll, pitch, yaw = _euler_from_quat_wxyz(quat_wxyz)
    forward = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=float)
    left = np.array([-math.sin(yaw), math.cos(yaw), 0.0], dtype=float)
    body_x_accel = float(np.dot(accel_enu, forward))
    body_y_accel = float(np.dot(accel_enu, left))

    max_tilt = math.radians(max(0.0, float(args.max_tilt_deg)))
    pitch_sp = float(args.pitch_accel_sign) * math.atan2(body_x_accel, GRAVITY_MPS2)
    roll_sp = float(args.roll_accel_sign) * math.atan2(body_y_accel, GRAVITY_MPS2)
    tilt = math.hypot(roll_sp, pitch_sp)
    if max_tilt > 0.0 and tilt > max_tilt:
        scale = max_tilt / max(tilt, 1.0e-9)
        roll_sp *= scale
        pitch_sp *= scale

    target_yaw = math.atan2(float(rel_pos[1]), float(rel_pos[0]))
    yaw_error = _wrap_pi(target_yaw - yaw)
    yaw_rate = float(np.clip(float(args.yaw_p) * yaw_error, -math.radians(args.max_yaw_rate_deg_s), math.radians(args.max_yaw_rate_deg_s)))
    roll_rate = float(args.attitude_p) * _wrap_pi(roll_sp - roll)
    pitch_rate = float(args.attitude_p) * _wrap_pi(pitch_sp - pitch)
    roll_rate = float(np.clip(roll_rate, -math.radians(args.max_roll_rate_deg_s), math.radians(args.max_roll_rate_deg_s)))
    pitch_rate = float(np.clip(pitch_rate, -math.radians(args.max_pitch_rate_deg_s), math.radians(args.max_pitch_rate_deg_s)))
    thrust = float(args.hover_thrust) + float(args.thrust_gain) * float(accel_enu[2]) / GRAVITY_MPS2
    thrust = float(np.clip(thrust, float(args.min_thrust), float(args.max_thrust)))
    command = RateCommand(roll_rate_rad_s=roll_rate, pitch_rate_rad_s=pitch_rate, yaw_rate_rad_s=yaw_rate, thrust_z=thrust)
    debug = {
        "roll_deg": math.degrees(roll),
        "pitch_deg": math.degrees(pitch),
        "yaw_deg": math.degrees(yaw),
        "roll_sp_deg": math.degrees(roll_sp),
        "pitch_sp_deg": math.degrees(pitch_sp),
        "yaw_error_deg": math.degrees(yaw_error),
        "body_x_accel": body_x_accel,
        "body_y_accel": body_y_accel,
    }
    return command, debug


def _write_csv(rows: Sequence[dict[str, float | int | str]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.touch()
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _send_idle(client: BetaflightMSPClient, *, armed: bool, duration_s: float, rate_hz: float) -> None:
    aux = (2000, 2000, 1000, 1000) if armed else (1000, 1000, 1000, 1000)
    command = BetaflightRCCommand(throttle=1000, aux=aux)
    dt = 1.0 / max(1.0, float(rate_hz))
    deadline = time.monotonic() + max(0.0, float(duration_s))
    while time.monotonic() < deadline:
        client.send_rc_command(command)
        time.sleep(dt)


def _send_spool(client: BetaflightMSPClient, *, thrust: float, duration_s: float, rate_hz: float) -> None:
    throttle = int(round(1000.0 + float(np.clip(thrust, 0.0, 1.0)) * 1000.0))
    command = BetaflightRCCommand(throttle=throttle, aux=(2000, 1000, 1000, 1000))
    dt = 1.0 / max(1.0, float(rate_hz))
    deadline = time.monotonic() + max(0.0, float(duration_s))
    while time.monotonic() < deadline:
        client.send_rc_command(command)
        time.sleep(dt)


def _output_path(args: argparse.Namespace) -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    prefix = str(args.trajectory_prefix or f"gazebo_betaflight_truth_png_{stamp}")
    return Path(args.trajectory_dir) / f"{prefix}.csv"


def main() -> None:
    args = parse_args()
    if args.rate_hz <= 0.0:
        raise SystemExit("--rate-hz must be positive")
    pose_sub = GazeboPoseSubscriber(world=str(args.world), model=str(args.model))
    first = pose_sub.wait_latest(timeout_s=5.0)
    _first_roll, _first_pitch, first_yaw = _euler_from_quat_wxyz(first.quat_wxyz)
    if str(args.start_frame) == "body":
        forward = np.array([math.cos(first_yaw), math.sin(first_yaw), 0.0], dtype=float)
        left = np.array([-math.sin(first_yaw), math.cos(first_yaw), 0.0], dtype=float)
        target_start = (
            first.position_enu
            + forward * float(args.start_range_m)
            + left * float(args.start_lateral_m)
            + np.array([0.0, 0.0, float(args.target_altitude_offset_m)], dtype=float)
        )
    else:
        target_start = first.position_enu + np.array(
            [float(args.start_range_m), float(args.start_lateral_m), float(args.target_altitude_offset_m)],
            dtype=float,
        )
    target_velocity_raw = np.array([float(args.target_vx), float(args.target_vy), float(args.target_vz)], dtype=float)
    if str(args.target_velocity_frame) == "body":
        forward = np.array([math.cos(first_yaw), math.sin(first_yaw), 0.0], dtype=float)
        left = np.array([-math.sin(first_yaw), math.cos(first_yaw), 0.0], dtype=float)
        target_velocity = (
            forward * float(target_velocity_raw[0])
            + left * float(target_velocity_raw[1])
            + np.array([0.0, 0.0, float(target_velocity_raw[2])], dtype=float)
        )
    else:
        target_velocity = target_velocity_raw
    rc_config = _rc_config(args)
    rows: list[dict[str, float | int | str]] = []
    csv_path = _output_path(args)
    min_range = float("inf")
    hit = False

    with BetaflightMSPClient(
        host=str(args.betaflight_host),
        port=int(args.betaflight_msp_port),
        timeout_s=float(args.msp_timeout_s),
    ) as msp:
        _send_idle(msp, armed=False, duration_s=float(args.prime_s), rate_hz=float(args.rate_hz))
        _send_idle(msp, armed=True, duration_s=float(args.arm_s), rate_hz=float(args.rate_hz))
        _send_spool(
            msp,
            thrust=float(args.spool_thrust),
            duration_s=float(args.spool_s),
            rate_hz=float(args.rate_hz),
        )
        spool_reference_z = float(first.position_enu[2])
        for retry in range(max(0, int(args.spool_max_retries))):
            spool_sample = pose_sub.wait_latest(timeout_s=1.0)
            if abs(float(spool_sample.position_enu[2]) - spool_reference_z) >= max(0.0, float(args.spool_min_dz)):
                break
            print(
                "spool_retry "
                f"attempt={retry + 1} dz={float(spool_sample.position_enu[2]) - spool_reference_z:.3f}m "
                f"thrust={float(args.spool_retry_thrust):.2f}"
            )
            _send_spool(
                msp,
                thrust=float(args.spool_retry_thrust),
                duration_s=float(args.spool_retry_s),
                rate_hz=float(args.rate_hz),
            )
        start = time.monotonic()
        next_tick = start
        index = 0
        while True:
            now = time.monotonic()
            t = now - start
            if t > float(args.duration_s):
                break
            sample = pose_sub.wait_latest(timeout_s=1.0)
            target_pos = target_start + target_velocity * t
            rel_pos = target_pos - sample.position_enu
            rel_vel = target_velocity - sample.velocity_enu
            png = compute_truth_png(
                rel_pos,
                rel_vel,
                navigation_constant=float(args.navigation_constant),
                max_accel=float(args.max_png_accel),
            )
            los = rel_pos / max(float(np.linalg.norm(rel_pos)), 1.0e-9)
            desired_velocity = los * max(0.0, float(args.interceptor_speed))
            speed_hold = _clip_norm(float(args.speed_hold_kp) * (desired_velocity - sample.velocity_enu), float(args.max_speed_hold_accel))
            altitude_accel = np.array(
                [0.0, 0.0, float(args.altitude_kp) * rel_pos[2] - float(args.altitude_kd) * sample.velocity_enu[2]],
                dtype=float,
            )
            png_accel = np.asarray(png.acceleration, dtype=float) if png.valid else np.zeros(3, dtype=float)
            total_accel = _clip_norm(png_accel + speed_hold + altitude_accel, float(args.max_total_accel))
            rate_command, control_debug = _command_from_accel(total_accel, sample.quat_wxyz, rel_pos, args)
            rc_result = msp.send_rate_command(rate_command, rc_config)

            attitude_roll = attitude_pitch = attitude_yaw = float("nan")
            motor0 = motor1 = motor2 = motor3 = float("nan")
            if int(args.msp_diagnostic_every_n) > 0 and index % int(args.msp_diagnostic_every_n) == 0:
                try:
                    attitude = msp.read_attitude(timeout_s=float(args.msp_timeout_s))
                    attitude_roll, attitude_pitch, attitude_yaw = attitude.roll_deg, attitude.pitch_deg, attitude.yaw_deg
                except Exception:
                    pass
                try:
                    motor = msp.read_motor(timeout_s=float(args.msp_timeout_s))
                    motor_values = list(motor.outputs_us[:4])
                    if len(motor_values) >= 4:
                        motor0, motor1, motor2, motor3 = (float(v) for v in motor_values[:4])
                except Exception:
                    pass

            range_m = float(np.linalg.norm(rel_pos))
            min_range = min(min_range, range_m)
            rows.append(
                {
                    "t": t,
                    "range_m": range_m,
                    "min_range_m": min_range,
                    "png_valid": int(bool(png.valid)),
                    "png_reject_reason": str(png.reject_reason or ""),
                    "closing_speed_mps": float(png.closing_speed),
                    "own_x": float(sample.position_enu[0]),
                    "own_y": float(sample.position_enu[1]),
                    "own_z": float(sample.position_enu[2]),
                    "own_vx": float(sample.velocity_enu[0]),
                    "own_vy": float(sample.velocity_enu[1]),
                    "own_vz": float(sample.velocity_enu[2]),
                    "target_x": float(target_pos[0]),
                    "target_y": float(target_pos[1]),
                    "target_z": float(target_pos[2]),
                    "png_ax": float(png_accel[0]),
                    "png_ay": float(png_accel[1]),
                    "png_az": float(png_accel[2]),
                    "speed_hold_ax": float(speed_hold[0]),
                    "speed_hold_ay": float(speed_hold[1]),
                    "speed_hold_az": float(speed_hold[2]),
                    "control_ax": float(total_accel[0]),
                    "control_ay": float(total_accel[1]),
                    "control_az": float(total_accel[2]),
                    "roll_deg": control_debug["roll_deg"],
                    "pitch_deg": control_debug["pitch_deg"],
                    "yaw_deg": control_debug["yaw_deg"],
                    "roll_sp_deg": control_debug["roll_sp_deg"],
                    "pitch_sp_deg": control_debug["pitch_sp_deg"],
                    "yaw_error_deg": control_debug["yaw_error_deg"],
                    "rate_roll_deg_s": math.degrees(rate_command.roll_rate_rad_s),
                    "rate_pitch_deg_s": math.degrees(rate_command.pitch_rate_rad_s),
                    "rate_yaw_deg_s": math.degrees(rate_command.yaw_rate_rad_s),
                    "thrust": float(rate_command.thrust_z),
                    "rc_roll": int(rc_result.command.roll),
                    "rc_pitch": int(rc_result.command.pitch),
                    "rc_throttle": int(rc_result.command.throttle),
                    "rc_yaw": int(rc_result.command.yaw),
                    "msp_roll_deg": attitude_roll,
                    "msp_pitch_deg": attitude_pitch,
                    "msp_yaw_deg": attitude_yaw,
                    "msp_motor0": motor0,
                    "msp_motor1": motor1,
                    "msp_motor2": motor2,
                    "msp_motor3": motor3,
                }
            )
            if args.print_every_n > 0 and index % int(args.print_every_n) == 0:
                print(
                    "gazebo_bf_png "
                    f"t={t:.2f}s range={range_m:.2f}m min={min_range:.2f}m "
                    f"pos=({sample.position_enu[0]:.2f},{sample.position_enu[1]:.2f},{sample.position_enu[2]:.2f}) "
                    f"rc=({rc_result.command.roll},{rc_result.command.pitch},{rc_result.command.throttle},{rc_result.command.yaw}) "
                    f"png={int(bool(png.valid))}:{png.reject_reason or 'ok'}"
                )
            if float(args.hit_radius_m) > 0.0 and range_m <= float(args.hit_radius_m):
                hit = True
                break
            index += 1
            next_tick += 1.0 / float(args.rate_hz)
            time.sleep(max(0.0, next_tick - time.monotonic()))
        _send_idle(msp, armed=False, duration_s=0.5, rate_hz=float(args.rate_hz))

    _write_csv(rows, csv_path)
    print(
        "Gazebo Betaflight truth PNG complete: "
        f"rows={len(rows)} hit={int(hit)} min_range={min_range:.3f}m csv={csv_path}"
    )


if __name__ == "__main__":
    main()
