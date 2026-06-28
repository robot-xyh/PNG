from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np


GRAVITY_MPS2 = 9.80665
AIRSIM_GENERIC_QUAD_MASS_KG = 1.0
AIRSIM_GENERIC_QUAD_ROTOR_COUNT = 4
AIRSIM_GENERIC_QUAD_MAX_THRUST_PER_ROTOR_N = 4.179446268
AIRSIM_GENERIC_QUAD_MAX_TOTAL_THRUST_N = (
    AIRSIM_GENERIC_QUAD_ROTOR_COUNT * AIRSIM_GENERIC_QUAD_MAX_THRUST_PER_ROTOR_N
)
AIRSIM_GENERIC_QUAD_HOVER_THRUST = (
    AIRSIM_GENERIC_QUAD_MASS_KG * GRAVITY_MPS2 / AIRSIM_GENERIC_QUAD_MAX_TOTAL_THRUST_N
)


def add_body_rate_args(parser) -> None:
    parser.add_argument("--px4-mavlink-url", default="udp:127.0.0.1:14550")
    parser.add_argument("--px4-offboard-prime-s", type=float, default=2.5)
    parser.add_argument("--max-yaw-rate-deg", type=float, default=90.0)
    parser.add_argument("--body-rate-max-tilt-deg", type=float, default=20.0)
    parser.add_argument("--body-rate-roll-gain", type=float, default=1.0)
    parser.add_argument("--body-rate-pitch-gain", type=float, default=1.0)
    parser.add_argument("--body-rate-attitude-p", type=float, default=4.0)
    parser.add_argument("--body-rate-max-roll-rate-deg", type=float, default=60.0)
    parser.add_argument("--body-rate-max-pitch-rate-deg", type=float, default=60.0)
    parser.add_argument(
        "--body-rate-control-profile",
        choices=("legacy", "v2", "hybrid_v2"),
        default="legacy",
    )
    parser.add_argument("--body-rate-v2-kp-roll", type=float, default=5.0)
    parser.add_argument("--body-rate-v2-kp-pitch", type=float, default=5.0)
    parser.add_argument("--body-rate-v2-kp-yaw", type=float, default=3.0)
    parser.add_argument("--body-rate-v2-max-pq-rate-deg-s", type=float, default=120.0)
    parser.add_argument("--body-rate-v2-slew-pq-deg-s2", type=float, default=720.0)
    parser.add_argument("--body-rate-v2-slew-r-deg-s2", type=float, default=540.0)
    parser.add_argument("--body-rate-v2-thrust-reserve", type=float, default=0.15)
    parser.add_argument("--body-rate-v2-guard-error-ratio", type=float, default=0.55)
    parser.add_argument("--body-rate-v2-guard-png-scale", type=float, default=0.60)
    parser.add_argument("--body-rate-v2-guard-speed-hold-scale", type=float, default=0.45)
    parser.add_argument("--body-rate-hybrid-terminal-tilt-deg", type=float, default=25.0)
    parser.add_argument("--body-rate-hybrid-terminal-max-pq-rate-deg-s", type=float, default=100.0)
    parser.add_argument("--body-rate-hybrid-terminal-thrust-max", type=float, default=0.85)
    parser.add_argument("--vehicle-mass-kg", type=float, default=AIRSIM_GENERIC_QUAD_MASS_KG)
    parser.add_argument("--vehicle-max-total-thrust-n", type=float, default=AIRSIM_GENERIC_QUAD_MAX_TOTAL_THRUST_N)
    parser.add_argument(
        "--thrust-model",
        choices=("airsim_generic_quad", "empirical"),
        default="airsim_generic_quad",
    )
    parser.add_argument("--body-rate-hover-thrust", type=float, default=AIRSIM_GENERIC_QUAD_HOVER_THRUST)
    parser.add_argument("--body-rate-thrust-gain", type=float, default=AIRSIM_GENERIC_QUAD_HOVER_THRUST)
    parser.add_argument("--body-rate-min-thrust", type=float, default=0.25)
    parser.add_argument("--body-rate-max-thrust", type=float, default=0.95)
    parser.add_argument("--body-rate-speed-hold-gain", type=float, default=1.2)
    parser.add_argument("--body-rate-speed-hold-max-accel-mps2", type=float, default=6.0)
    parser.add_argument("--body-rate-total-accel-limit-mps2", type=float, default=18.0)


def validate_body_rate_args(args) -> None:
    if float(getattr(args, "vehicle_mass_kg", AIRSIM_GENERIC_QUAD_MASS_KG)) <= 0.0:
        raise SystemExit("--vehicle-mass-kg must be positive")
    if float(getattr(args, "vehicle_max_total_thrust_n", AIRSIM_GENERIC_QUAD_MAX_TOTAL_THRUST_N)) <= 0.0:
        raise SystemExit("--vehicle-max-total-thrust-n must be positive")
    if float(args.body_rate_min_thrust) > float(args.body_rate_max_thrust):
        raise SystemExit("--body-rate-min-thrust cannot exceed --body-rate-max-thrust")
    if not 0.0 <= float(args.body_rate_v2_thrust_reserve) < 1.0:
        raise SystemExit("--body-rate-v2-thrust-reserve must be in [0, 1)")
    if not 0.0 <= float(args.body_rate_v2_guard_png_scale) <= 1.0:
        raise SystemExit("--body-rate-v2-guard-png-scale must be in [0, 1]")
    if not 0.0 <= float(args.body_rate_v2_guard_speed_hold_scale) <= 1.0:
        raise SystemExit("--body-rate-v2-guard-speed-hold-scale must be in [0, 1]")
    if not 0.0 <= float(args.body_rate_hybrid_terminal_thrust_max) <= 1.0:
        raise SystemExit("--body-rate-hybrid-terminal-thrust-max must be in [0, 1]")


def clip_vector_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    limit = max(0.0, float(max_norm))
    norm = float(np.linalg.norm(value))
    if norm <= limit or norm <= 1.0e-9:
        return np.array(value, dtype=float)
    return value * (limit / norm)


def normalized_thrust_from_accel(
    acceleration_ned: np.ndarray,
    *,
    roll_rad: float,
    pitch_rad: float,
    min_thrust: float,
    max_thrust: float,
    hover_thrust: float,
    thrust_gain: float,
    args,
) -> tuple[float, float, float]:
    accel = np.asarray(acceleration_ned, dtype=float)
    if accel.shape != (3,) or not np.all(np.isfinite(accel)):
        accel = np.zeros(3, dtype=float)
    min_cmd = max(0.0, float(min_thrust))
    max_cmd = min(1.0, float(max_thrust))
    if min_cmd > max_cmd:
        min_cmd, max_cmd = max_cmd, min_cmd

    if str(getattr(args, "thrust_model", "empirical")) != "airsim_generic_quad":
        raw = float(hover_thrust) + float(thrust_gain) * (-float(accel[2]) / GRAVITY_MPS2)
        return float(np.clip(raw, min_cmd, max_cmd)), float(raw), 1.0

    mass = max(1.0e-6, float(getattr(args, "vehicle_mass_kg", AIRSIM_GENERIC_QUAD_MASS_KG)))
    max_total_thrust = max(
        1.0e-6,
        float(getattr(args, "vehicle_max_total_thrust_n", AIRSIM_GENERIC_QUAD_MAX_TOTAL_THRUST_N)),
    )
    required_up_accel = GRAVITY_MPS2 - float(accel[2])
    cos_tilt = math.cos(float(roll_rad)) * math.cos(float(pitch_rad))
    cos_tilt = max(0.20, abs(cos_tilt))
    raw = mass * required_up_accel / (max_total_thrust * cos_tilt)
    return float(np.clip(raw, min_cmd, max_cmd)), float(raw), float(cos_tilt)


def tilt_from_accel_body(accel_body: np.ndarray, max_tilt: float) -> tuple[float, float, np.ndarray]:
    accel = np.asarray(accel_body, dtype=float)
    if accel.shape != (3,) or not np.all(np.isfinite(accel)):
        accel = np.zeros(3, dtype=float)
    max_tilt = max(0.0, float(max_tilt))
    body_z_specific_force = np.array(
        [-float(accel[0]), -float(accel[1]), GRAVITY_MPS2 - float(accel[2])],
        dtype=float,
    )
    if body_z_specific_force[2] <= 1.0e-6:
        body_z_specific_force[2] = 1.0e-6
    roll_sp = float(np.arctan2(-body_z_specific_force[1], body_z_specific_force[2]))
    pitch_sp = float(np.arctan2(body_z_specific_force[0], body_z_specific_force[2]))
    roll_sp = float(np.clip(roll_sp, -max_tilt, max_tilt))
    pitch_sp = float(np.clip(pitch_sp, -max_tilt, max_tilt))
    return roll_sp, pitch_sp, body_z_specific_force


def body_rate_neutral_thrust(args) -> float:
    thrust, _, _ = normalized_thrust_from_accel(
        np.zeros(3, dtype=float),
        roll_rad=0.0,
        pitch_rad=0.0,
        min_thrust=float(getattr(args, "body_rate_min_thrust", 0.0)),
        max_thrust=float(getattr(args, "body_rate_max_thrust", 1.0)),
        hover_thrust=float(getattr(args, "body_rate_hover_thrust", AIRSIM_GENERIC_QUAD_HOVER_THRUST)),
        thrust_gain=float(getattr(args, "body_rate_thrust_gain", AIRSIM_GENERIC_QUAD_HOVER_THRUST)),
        args=args,
    )
    return thrust


def euler_to_quaternion_wxyz(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    cr = math.cos(0.5 * float(roll_rad))
    sr = math.sin(0.5 * float(roll_rad))
    cp = math.cos(0.5 * float(pitch_rad))
    sp = math.sin(0.5 * float(pitch_rad))
    cy = math.cos(0.5 * float(yaw_rad))
    sy = math.sin(0.5 * float(yaw_rad))
    return normalize_quaternion_wxyz(
        np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ],
            dtype=float,
        )
    )


def normalize_quaternion_wxyz(quat: np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    norm = float(np.linalg.norm(q))
    if norm <= 1.0e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    q = q / norm
    if q[0] < 0.0:
        q = -q
    return q


def quaternion_multiply_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    qa = normalize_quaternion_wxyz(a)
    qb = normalize_quaternion_wxyz(b)
    aw, ax, ay, az = qa
    bw, bx, by, bz = qb
    return normalize_quaternion_wxyz(
        np.array(
            [
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ],
            dtype=float,
        )
    )


def quaternion_conjugate_wxyz(quat: np.ndarray) -> np.ndarray:
    q = normalize_quaternion_wxyz(quat)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quaternion_error_vector_wxyz(q_desired: np.ndarray, q_current: np.ndarray) -> np.ndarray:
    q_err = quaternion_multiply_wxyz(quaternion_conjugate_wxyz(q_current), q_desired)
    if q_err[0] < 0.0:
        q_err = -q_err
    return 2.0 * np.asarray(q_err[1:4], dtype=float)


def body_rate_control_acceleration(
    *,
    png_acceleration_I: np.ndarray,
    current_velocity_I: np.ndarray,
    velocity_reference_I: np.ndarray,
    png_scale: float = 1.0,
    speed_hold_scale: float = 1.0,
    args,
) -> tuple[np.ndarray, np.ndarray]:
    speed_hold = float(args.body_rate_speed_hold_gain) * (
        np.asarray(velocity_reference_I, dtype=float) - np.asarray(current_velocity_I, dtype=float)
    )
    speed_hold *= float(np.clip(float(speed_hold_scale), 0.0, 1.0))
    speed_hold = clip_vector_norm(speed_hold, float(args.body_rate_speed_hold_max_accel_mps2))
    png = np.asarray(png_acceleration_I, dtype=float) * float(np.clip(float(png_scale), 0.0, 1.0))
    total = clip_vector_norm(png + speed_hold, float(args.body_rate_total_accel_limit_mps2))
    return total, speed_hold


def body_rate_command_from_accel(
    acceleration_I: np.ndarray,
    R_IB: np.ndarray,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
    yaw_rate_deg_s: float,
    dt_s: float,
    args,
) -> dict[str, float | np.ndarray]:
    accel_I = np.asarray(acceleration_I, dtype=float)
    if not np.all(np.isfinite(accel_I)):
        accel_I = np.zeros(3, dtype=float)
    accel_B = np.asarray(R_IB, dtype=float).T @ accel_I
    profile = str(getattr(args, "body_rate_control_profile", "legacy"))
    effective_max_tilt_deg = float(
        getattr(args, "_body_rate_effective_max_tilt_deg", args.body_rate_max_tilt_deg)
        if profile == "hybrid_v2"
        else args.body_rate_max_tilt_deg
    )
    max_tilt = np.deg2rad(max(0.0, effective_max_tilt_deg))
    roll_sp, pitch_sp, body_z_specific_force = tilt_from_accel_body(accel_B, max_tilt)
    roll_sp = float(np.clip(roll_sp * float(args.body_rate_roll_gain), -max_tilt, max_tilt))
    pitch_sp = float(np.clip(pitch_sp * float(args.body_rate_pitch_gain), -max_tilt, max_tilt))
    max_yaw_rate = np.deg2rad(max(0.0, float(args.max_yaw_rate_deg)))

    if profile in {"v2", "hybrid_v2"}:
        q_current = euler_to_quaternion_wxyz(float(roll_rad), float(pitch_rad), float(yaw_rad))
        q_desired = euler_to_quaternion_wxyz(roll_sp, pitch_sp, float(yaw_rad))
        q_error = quaternion_error_vector_wxyz(q_desired, q_current)
        max_pq_rate_deg_s = float(
            getattr(args, "_body_rate_effective_max_pq_rate_deg_s", args.body_rate_v2_max_pq_rate_deg_s)
            if profile == "hybrid_v2"
            else args.body_rate_v2_max_pq_rate_deg_s
        )
        max_pq_rate = np.deg2rad(max(0.0, max_pq_rate_deg_s))
        raw_cmd = np.array(
            [
                float(args.body_rate_v2_kp_roll) * float(q_error[0]),
                float(args.body_rate_v2_kp_pitch) * float(q_error[1]),
                float(args.body_rate_v2_kp_yaw) * float(q_error[2]) + np.deg2rad(float(yaw_rate_deg_s)),
            ],
            dtype=float,
        )
        clipped_cmd = np.array(
            [
                float(np.clip(raw_cmd[0], -max_pq_rate, max_pq_rate)),
                float(np.clip(raw_cmd[1], -max_pq_rate, max_pq_rate)),
                float(np.clip(raw_cmd[2], -max_yaw_rate, max_yaw_rate)),
            ],
            dtype=float,
        )
        dt = max(1.0e-4, float(dt_s))
        slew_limits = np.deg2rad(
            np.array(
                [
                    max(0.0, float(args.body_rate_v2_slew_pq_deg_s2)),
                    max(0.0, float(args.body_rate_v2_slew_pq_deg_s2)),
                    max(0.0, float(args.body_rate_v2_slew_r_deg_s2)),
                ],
                dtype=float,
            )
        )
        prev = getattr(args, "_body_rate_v2_prev_cmd_rad_s", None)
        if prev is None:
            cmd = clipped_cmd
            slew_limited = np.zeros(3, dtype=int)
        else:
            prev_cmd = np.asarray(prev, dtype=float)
            if prev_cmd.shape != (3,) or not np.all(np.isfinite(prev_cmd)):
                prev_cmd = clipped_cmd
            delta = clipped_cmd - prev_cmd
            limited_delta = np.clip(delta, -slew_limits * dt, slew_limits * dt)
            cmd = prev_cmd + limited_delta
            slew_limited = (np.abs(delta - limited_delta) > 1.0e-9).astype(int)
        args._body_rate_v2_prev_cmd_rad_s = np.array(cmd, dtype=float)

        min_cmd = max(0.0, float(args.body_rate_min_thrust))
        requested_max_cmd = min(1.0, float(getattr(args, "_body_rate_effective_max_thrust", args.body_rate_max_thrust)))
        reserved_max_cmd = max(min_cmd, min(requested_max_cmd, 1.0 - float(args.body_rate_v2_thrust_reserve)))
        thrust_axis_I = np.asarray(R_IB, dtype=float) @ np.array([0.0, 0.0, 1.0], dtype=float)
        axis_norm = float(np.linalg.norm(thrust_axis_I))
        thrust_axis_I = np.array([0.0, 0.0, 1.0], dtype=float) if axis_norm <= 1.0e-9 else thrust_axis_I / axis_norm
        required_specific_force_I = np.array(
            [-float(accel_I[0]), -float(accel_I[1]), GRAVITY_MPS2 - float(accel_I[2])],
            dtype=float,
        )
        mass = max(1.0e-6, float(getattr(args, "vehicle_mass_kg", AIRSIM_GENERIC_QUAD_MASS_KG)))
        max_total_thrust = max(
            1.0e-6,
            float(getattr(args, "vehicle_max_total_thrust_n", AIRSIM_GENERIC_QUAD_MAX_TOTAL_THRUST_N)),
        )
        thrust_raw = mass * float(np.dot(required_specific_force_I, thrust_axis_I)) / max_total_thrust
        if not math.isfinite(thrust_raw):
            thrust_raw = float(args.body_rate_hover_thrust)
        thrust = float(np.clip(thrust_raw, min_cmd, reserved_max_cmd))
        return {
            "profile": profile,
            "accel_B": accel_B,
            "body_z_specific_force": body_z_specific_force,
            "roll_sp_rad": roll_sp,
            "pitch_sp_rad": pitch_sp,
            "body_rates_rad_s": cmd,
            "body_rates_raw_rad_s": raw_cmd,
            "body_rates_clipped_rad_s": clipped_cmd,
            "body_rate_q_error": q_error,
            "body_rate_slew_limited": slew_limited,
            "thrust": thrust,
            "thrust_raw": float(thrust_raw),
            "thrust_cos_tilt": float(np.dot(thrust_axis_I, np.array([0.0, 0.0, 1.0], dtype=float))),
            "thrust_reserved_max": reserved_max_cmd,
            "thrust_saturated": int(thrust <= min_cmd + 1.0e-9 or thrust >= reserved_max_cmd - 1.0e-9),
        }

    attitude_p = max(0.0, float(args.body_rate_attitude_p))
    max_roll_rate = np.deg2rad(max(0.0, float(args.body_rate_max_roll_rate_deg)))
    max_pitch_rate = np.deg2rad(max(0.0, float(args.body_rate_max_pitch_rate_deg)))
    p_cmd = float(np.clip(attitude_p * (roll_sp - float(roll_rad)), -max_roll_rate, max_roll_rate))
    q_cmd = float(np.clip(attitude_p * (pitch_sp - float(pitch_rad)), -max_pitch_rate, max_pitch_rate))
    r_cmd = float(np.clip(np.deg2rad(float(yaw_rate_deg_s)), -max_yaw_rate, max_yaw_rate))
    thrust, thrust_raw, thrust_cos_tilt = normalized_thrust_from_accel(
        accel_I,
        roll_rad=roll_sp,
        pitch_rad=pitch_sp,
        min_thrust=float(args.body_rate_min_thrust),
        max_thrust=float(args.body_rate_max_thrust),
        hover_thrust=float(args.body_rate_hover_thrust),
        thrust_gain=float(args.body_rate_thrust_gain),
        args=args,
    )
    return {
        "profile": profile,
        "accel_B": accel_B,
        "body_z_specific_force": body_z_specific_force,
        "roll_sp_rad": roll_sp,
        "pitch_sp_rad": pitch_sp,
        "body_rates_rad_s": np.array([p_cmd, q_cmd, r_cmd], dtype=float),
        "body_rates_raw_rad_s": np.array([p_cmd, q_cmd, r_cmd], dtype=float),
        "body_rates_clipped_rad_s": np.array([p_cmd, q_cmd, r_cmd], dtype=float),
        "body_rate_q_error": np.zeros(3, dtype=float),
        "body_rate_slew_limited": np.zeros(3, dtype=int),
        "thrust": thrust,
        "thrust_raw": thrust_raw,
        "thrust_cos_tilt": thrust_cos_tilt,
        "thrust_reserved_max": float(args.body_rate_max_thrust),
        "thrust_saturated": int(
            thrust <= float(args.body_rate_min_thrust) + 1.0e-9
            or thrust >= float(args.body_rate_max_thrust) - 1.0e-9
        ),
    }


class PX4MavlinkOffboard:
    def __init__(self, url: str, *, source_system: int = 134, source_component: int = 1):
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise SystemExit("Install pymavlink before using MAVLink Offboard body-rate control.") from exc
        self.mavutil = mavutil
        self.master = mavutil.mavlink_connection(url, source_system=source_system, source_component=source_component)
        self.target_system = 1
        self.target_component = 1
        self.connected = False
        self.armed = False
        self.offboard_requested = False
        self.last_mode_request_s = 0.0
        self.start_monotonic = time.monotonic()

    def connect(self, timeout_s: float = 12.0) -> bool:
        if self.connected:
            return True
        heartbeat = self.master.wait_heartbeat(timeout=timeout_s)
        if heartbeat is None:
            print(f"px4_mavlink_warning=no_heartbeat url={self.master.address}")
            return False
        self.target_system = int(self.master.target_system or 1)
        self.target_component = int(self.master.target_component or 1)
        self.connected = True
        print(
            "px4_mavlink_connected "
            f"target_system={self.target_system} target_component={self.target_component} heartbeat={heartbeat}"
        )
        return True

    def _time_boot_ms(self) -> int:
        return int(max(0.0, time.monotonic() - self.start_monotonic) * 1000.0) & 0xFFFFFFFF

    def send_body_rate(self, body_rates_rad_s: np.ndarray, thrust: float) -> None:
        if not self.connected and not self.connect(timeout_s=0.2):
            return
        rates = np.asarray(body_rates_rad_s, dtype=float)
        mavlink = self.mavutil.mavlink
        type_mask = int(getattr(mavlink, "ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE", 128))
        self.master.mav.set_attitude_target_send(
            self._time_boot_ms(),
            self.target_system,
            self.target_component,
            type_mask,
            [1.0, 0.0, 0.0, 0.0],
            float(rates[0]),
            float(rates[1]),
            float(rates[2]),
            float(np.clip(float(thrust), 0.0, 1.0)),
        )

    def arm(self) -> None:
        if not self.connected and not self.connect(timeout_s=1.0):
            return
        if self.armed:
            return
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        self.armed = True

    def request_offboard(self) -> None:
        if not self.connected and not self.connect(timeout_s=1.0):
            return
        now = time.monotonic()
        if self.offboard_requested and now - self.last_mode_request_s < 2.0:
            return
        self.master.set_mode("OFFBOARD")
        self.offboard_requested = True
        self.last_mode_request_s = now

    def prime_body_rate_and_request_offboard(self, duration_s: float, hover_thrust: float) -> None:
        if not self.connect(timeout_s=max(2.0, duration_s + 4.0)):
            return
        deadline = time.monotonic() + max(0.0, float(duration_s))
        while time.monotonic() < deadline:
            self.send_body_rate(np.zeros(3, dtype=float), hover_thrust)
            time.sleep(0.05)
        self.arm()
        for _ in range(10):
            self.send_body_rate(np.zeros(3, dtype=float), hover_thrust)
            time.sleep(0.05)
        self.request_offboard()

    def stop_body_rate(self, hover_thrust: float) -> None:
        if not self.connected:
            return
        for _ in range(10):
            self.send_body_rate(np.zeros(3, dtype=float), hover_thrust)
            time.sleep(0.03)


def px4_mavlink_offboard(args) -> Optional[PX4MavlinkOffboard]:
    if getattr(args, "px4_command_mode", "") != "mavlink_body_rate":
        return None
    offboard = getattr(args, "_px4_mavlink_offboard", None)
    if offboard is None:
        offboard = PX4MavlinkOffboard(args.px4_mavlink_url)
        setattr(args, "_px4_mavlink_offboard", offboard)
    return offboard


def prime_body_rate_offboard(args) -> None:
    offboard = px4_mavlink_offboard(args)
    if offboard is not None:
        offboard.prime_body_rate_and_request_offboard(args.px4_offboard_prime_s, body_rate_neutral_thrust(args))


def command_body_rate(body_rates_rad_s: np.ndarray, thrust: float, args) -> None:
    offboard = px4_mavlink_offboard(args)
    if offboard is not None:
        offboard.send_body_rate(body_rates_rad_s, thrust)
        offboard.arm()
        offboard.request_offboard()


def stop_body_rate(args) -> None:
    offboard = px4_mavlink_offboard(args)
    if offboard is not None:
        offboard.stop_body_rate(body_rate_neutral_thrust(args))


def vehicle_euler_rad(airsim_module, orientation) -> tuple[float, float, float]:
    pitch, roll, yaw = airsim_module.to_eularian_angles(orientation)
    return float(roll), float(pitch), float(yaw)
