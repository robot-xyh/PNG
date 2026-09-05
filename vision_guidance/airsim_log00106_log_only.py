from __future__ import annotations

import csv
import hashlib
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np

from .betaflight_intercept_controller import (
    EngagementPolicy,
    FovPriorityConfig,
    VelocityEstablishingPngConfig,
    VelocityEstablishingPngController,
    VelocityEstablishingPngInput,
    _clip_norm,
    _suppress_fov_opposition,
)
from .geometry import camera_ray_from_pixel, los_camera_to_inertial, normalize
from .types import CameraIntrinsics


REAL_INTRINSICS = CameraIntrinsics(
    fx=530.8443137412,
    fy=532.2954942356,
    cx=321.0278689412,
    cy=247.2573194658,
    width=640,
    height=512,
)
R_BC_UPWARD_FRD = np.array(
    [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
    dtype=float,
)
GRAVITY_M_S2 = 9.80665
LOG00106_ALGORITHM_STOP_S = 1.670804


CSV_FIELDS = (
    "case_id",
    "run_id",
    "seed",
    "t_sim_s",
    "t_algorithm_s",
    "t_contact_s",
    "sample_time_s",
    "available_time_s",
    "measurement_age_ms",
    "fusion_wait_ms",
    "timing_replay_extrapolated",
    "bbox_measurement_source",
    "controller_phase",
    "guidance_valid",
    "guidance_reason",
    "algorithm_active",
    "contact_detected",
    "post_contact",
    "interceptor_position_n_m",
    "interceptor_position_e_m",
    "interceptor_position_d_m",
    "interceptor_velocity_n_m_s",
    "interceptor_velocity_e_m_s",
    "interceptor_velocity_d_m_s",
    "interceptor_velocity_observed_n_m_s",
    "interceptor_velocity_observed_e_m_s",
    "interceptor_velocity_observed_d_m_s",
    "target_position_n_m",
    "target_position_e_m",
    "target_position_d_m",
    "target_velocity_n_m_s",
    "target_velocity_e_m_s",
    "target_velocity_d_m_s",
    "relative_position_n_m",
    "relative_position_e_m",
    "relative_position_d_m",
    "relative_range_m",
    "closing_speed_m_s",
    "miss_distance_truth_m",
    "bbox_x1_px",
    "bbox_y1_px",
    "bbox_x2_px",
    "bbox_y2_px",
    "bbox_center_u_px",
    "bbox_center_v_px",
    "bbox_area_ratio",
    "bbox_in_fov",
    "lambda_truth_n",
    "lambda_truth_e",
    "lambda_truth_d",
    "lambda_measured_n",
    "lambda_measured_e",
    "lambda_measured_d",
    "lambda_filtered_n",
    "lambda_filtered_e",
    "lambda_filtered_d",
    "lambda_dot_n_s",
    "lambda_dot_e_s",
    "lambda_dot_d_s",
    "omega_los_n_rad_s",
    "omega_los_e_rad_s",
    "omega_los_d_rad_s",
    "velocity_reference_n_m_s",
    "velocity_reference_e_m_s",
    "velocity_reference_d_m_s",
    "speed_accel_n_m_s2",
    "speed_accel_e_m_s2",
    "speed_accel_d_m_s2",
    "speed_accel_norm_m_s2",
    "png_accel_n_m_s2",
    "png_accel_e_m_s2",
    "png_accel_d_m_s2",
    "png_accel_norm_m_s2",
    "fov_accel_n_m_s2",
    "fov_accel_e_m_s2",
    "fov_accel_d_m_s2",
    "fov_accel_norm_m_s2",
    "total_accel_n_m_s2",
    "total_accel_e_m_s2",
    "total_accel_d_m_s2",
    "total_accel_norm_m_s2",
    "speed_saturated",
    "png_saturated",
    "fov_saturated",
    "fov_priority_active",
    "fov_priority_weight",
    "total_saturated",
    "roll_frd_deg",
    "pitch_frd_deg",
    "yaw_ned_deg",
    "desired_roll_frd_deg",
    "desired_pitch_frd_deg",
    "roll_rate_generated_deg_s",
    "pitch_rate_generated_deg_s",
    "yaw_rate_generated_deg_s",
    "roll_rate_setpoint_deg_s",
    "pitch_rate_setpoint_deg_s",
    "yaw_rate_setpoint_deg_s",
    "roll_rate_actual_deg_s",
    "pitch_rate_actual_deg_s",
    "yaw_rate_actual_deg_s",
    "throttle_model_target_us",
    "throttle_handover_output_us",
    "throttle_applied_us",
    "throttle_handover_alpha",
    "airsim_throttle_command_0_1",
    "thrust_model_load_factor_g",
    "specific_force_actual_g",
    "thrust_model_ratio",
    "specific_force_estimate_source",
    "voltage_label_min_v",
    "voltage_label_max_v",
    "rate_limited",
    "tilt_limited",
    "throttle_limited",
    "algorithm_exit_event",
    "contact_event",
    "closest_point_event",
)


@dataclass(frozen=True)
class ReplayTimingSample:
    index: int
    sample_offset_s: float
    available_offset_s: float
    measurement_age_s: float
    fusion_wait_s: float


@dataclass(frozen=True)
class ScheduledTiming:
    sample_time_s: float
    available_time_s: float
    measurement_age_s: float
    fusion_wait_s: float
    extrapolated: bool


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_log00106_replay_timing(path: str | Path) -> tuple[ReplayTimingSample, ...]:
    """Load the 40 paired perception timings used during algorithm publication."""

    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = [
            row
            for row in csv.DictReader(stream)
            if row.get("msp_publish_mode") == "algorithm"
            and row.get("perception_new_result") == "1"
            and row.get("fusion_status") == "processed"
        ]
    if len(rows) != 40:
        raise ValueError(f"expected 40 LOG00106 perception results, got {len(rows)}")

    first_sample = float(rows[0]["detection_exposure_ts"])
    first_available = float(rows[0]["timestamp"])
    samples: list[ReplayTimingSample] = []
    for index, row in enumerate(rows):
        sample_time = float(row["detection_exposure_ts"])
        available_time = float(row["timestamp"])
        result_age_s = float(row["perception_result_age_ms"]) * 1.0e-3
        fusion_wait_s = float(row["fusion_wait_ms"]) * 1.0e-3
        if not math.isclose(available_time - sample_time, result_age_s, abs_tol=1.5e-3):
            raise ValueError(f"unpaired LOG00106 timing at perception sample {index}")
        samples.append(
            ReplayTimingSample(
                index=index,
                sample_offset_s=sample_time - first_sample,
                available_offset_s=available_time - first_available,
                measurement_age_s=result_age_s,
                fusion_wait_s=fusion_wait_s,
            )
        )
    return tuple(samples)


class ReplayTimingSchedule:
    def __init__(self, samples: Sequence[ReplayTimingSample]):
        if not samples:
            raise ValueError("timing replay requires at least one sample")
        self.samples = tuple(samples)
        sample_steps = np.diff([sample.sample_offset_s for sample in self.samples])
        tail_step = float(np.median(sample_steps)) if len(sample_steps) else 1.0 / 30.0
        self.period_s = float(self.samples[-1].sample_offset_s + max(1.0e-3, tail_step))
        self._next_index = 0

    def pop_due(self, timestamp_s: float) -> list[ScheduledTiming]:
        due: list[ScheduledTiming] = []
        now = float(timestamp_s)
        while True:
            cycle, index = divmod(self._next_index, len(self.samples))
            sample = self.samples[index]
            sample_time = cycle * self.period_s + sample.sample_offset_s
            if sample_time > now + 1.0e-9:
                break
            due.append(
                ScheduledTiming(
                    sample_time_s=sample_time,
                    available_time_s=sample_time + sample.measurement_age_s,
                    measurement_age_s=sample.measurement_age_s,
                    fusion_wait_s=sample.fusion_wait_s,
                    extrapolated=cycle > 0,
                )
            )
            self._next_index += 1
        return due


def render_intrinsics(width: int, height: int, horizontal_fov_deg: float) -> CameraIntrinsics:
    focal = 0.5 * float(width) / math.tan(0.5 * math.radians(float(horizontal_fov_deg)))
    return CameraIntrinsics(
        fx=focal,
        fy=focal,
        cx=0.5 * float(width),
        cy=0.5 * float(height),
        width=int(width),
        height=int(height),
    )


def remap_render_bbox_to_real_intrinsics(
    bbox_xyxy: Sequence[float],
    source: CameraIntrinsics,
    destination: CameraIntrinsics = REAL_INTRINSICS,
) -> tuple[float, float, float, float]:
    if len(bbox_xyxy) != 4:
        raise ValueError("bbox must contain x1, y1, x2, y2")

    def remap(u: float, v: float) -> tuple[float, float]:
        x_normalized = (float(u) - source.cx) / source.fx
        y_normalized = (float(v) - source.cy) / source.fy
        return (
            destination.cx + destination.fx * x_normalized,
            destination.cy + destination.fy * y_normalized,
        )

    x1, y1 = remap(float(bbox_xyxy[0]), float(bbox_xyxy[1]))
    x2, y2 = remap(float(bbox_xyxy[2]), float(bbox_xyxy[3]))
    return (x1, y1, x2, y2)


def bbox_center_and_area(
    bbox_xyxy: Sequence[float], intrinsics: CameraIntrinsics = REAL_INTRINSICS
) -> tuple[float, float, float]:
    x1, y1, x2, y2 = (float(value) for value in bbox_xyxy)
    center_u = 0.5 * (x1 + x2)
    center_v = 0.5 * (y1 + y2)
    area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    return center_u, center_v, area / float(intrinsics.width * intrinsics.height)


def measured_los_ned_from_bbox(
    bbox_xyxy: Sequence[float],
    R_IB: np.ndarray,
    intrinsics: CameraIntrinsics = REAL_INTRINSICS,
    R_BC: np.ndarray = R_BC_UPWARD_FRD,
) -> np.ndarray:
    center_u, center_v, _ = bbox_center_and_area(bbox_xyxy, intrinsics)
    ray_camera = camera_ray_from_pixel(center_u, center_v, intrinsics)
    return los_camera_to_inertial(ray_camera, R_BC, np.asarray(R_IB, dtype=float))


def project_los_to_real_pixel(
    lambda_ned: Sequence[float],
    R_IB: np.ndarray,
    intrinsics: CameraIntrinsics = REAL_INTRINSICS,
    R_BC: np.ndarray = R_BC_UPWARD_FRD,
) -> tuple[float, float]:
    ray_camera = np.asarray(R_BC, dtype=float).T @ np.asarray(R_IB, dtype=float).T @ normalize(
        np.asarray(lambda_ned, dtype=float)
    )
    if ray_camera[2] <= 0.0:
        raise ValueError("LOS is behind the upward camera")
    return (
        intrinsics.cx + intrinsics.fx * float(ray_camera[0] / ray_camera[2]),
        intrinsics.cy + intrinsics.fy * float(ray_camera[1] / ray_camera[2]),
    )


def frd_rates_to_airsim_flu(rates_frd_rad_s: Sequence[float]) -> np.ndarray:
    rates = np.asarray(rates_frd_rad_s, dtype=float)
    if rates.shape != (3,) or not np.all(np.isfinite(rates)):
        raise ValueError("FRD body rates must be a finite 3-vector")
    return np.array([rates[0], -rates[1], -rates[2]], dtype=float)


def airsim_flu_rates_to_frd(rates_flu_rad_s: Sequence[float]) -> np.ndarray:
    return frd_rates_to_airsim_flu(rates_flu_rad_s)


class LowRateVelocityObserver:
    """Sample truth at a low rate and apply the production first-order filter."""

    def __init__(
        self,
        initial_velocity_ned_m_s: Sequence[float],
        *,
        update_rate_hz: float,
        time_constant_s: float,
    ) -> None:
        if update_rate_hz <= 0.0 or time_constant_s <= 0.0:
            raise ValueError("velocity observer rate and time constant must be positive")
        self.value = np.asarray(initial_velocity_ned_m_s, dtype=float).copy()
        if self.value.shape != (3,) or not np.all(np.isfinite(self.value)):
            raise ValueError("initial velocity must be a finite 3-vector")
        self.period_s = 1.0 / float(update_rate_hz)
        self.time_constant_s = float(time_constant_s)
        self.last_update_s = 0.0
        self.next_update_s = self.period_s

    def update(self, timestamp_s: float, truth_velocity_ned_m_s: Sequence[float]) -> tuple[np.ndarray, bool]:
        now = float(timestamp_s)
        updated = False
        truth = np.asarray(truth_velocity_ned_m_s, dtype=float)
        while now + 1.0e-9 >= self.next_update_s:
            dt = self.next_update_s - self.last_update_s
            alpha = 1.0 - math.exp(-dt / self.time_constant_s)
            self.value += alpha * (truth - self.value)
            self.last_update_s = self.next_update_s
            self.next_update_s += self.period_s
            updated = True
        return self.value.copy(), updated


class DelayedVectorQueue:
    def __init__(self, delay_s: float, initial: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        if delay_s < 0.0:
            raise ValueError("delay must be non-negative")
        self.delay_s = float(delay_s)
        self.initial = np.asarray(initial, dtype=float)
        self._values: deque[tuple[float, np.ndarray]] = deque()
        self._output = self.initial.copy()

    def push(self, timestamp_s: float, value: Sequence[float]) -> None:
        array = np.asarray(value, dtype=float)
        if array.shape != self.initial.shape or not np.all(np.isfinite(array)):
            raise ValueError("delayed queue value has invalid shape or values")
        self._values.append((float(timestamp_s) + self.delay_s, array.copy()))

    def output(self, timestamp_s: float) -> np.ndarray:
        now = float(timestamp_s)
        while self._values and self._values[0][0] <= now + 1.0e-9:
            _, self._output = self._values.popleft()
        return self._output.copy()

    def clear(self, value: Sequence[float] | None = None) -> None:
        self._values.clear()
        self._output = self.initial.copy() if value is None else np.asarray(value, dtype=float).copy()


@dataclass(frozen=True)
class ExitDecision:
    algorithm_active: bool
    exit_event: bool
    may_stop_run: bool


class AlgorithmExitStateMachine:
    def __init__(self, *, early_exit: bool, stop_time_s: float, post_exit_min_s: float) -> None:
        self.early_exit = bool(early_exit)
        self.stop_time_s = float(stop_time_s)
        self.post_exit_min_s = float(post_exit_min_s)
        self.exited = False

    def update(self, timestamp_s: float) -> ExitDecision:
        now = float(timestamp_s)
        event = False
        if self.early_exit and not self.exited and now >= self.stop_time_s:
            self.exited = True
            event = True
        may_stop = self.exited and now >= self.stop_time_s + self.post_exit_min_s
        return ExitDecision(not self.exited, event, may_stop)


@dataclass(frozen=True)
class ThrottleHandoverValue:
    target_us: float
    output_us: float
    alpha: float
    limited: bool


class ThrottleHandover:
    def __init__(
        self,
        *,
        source_us: float = 1303.0,
        duration_s: float = 0.8,
        minimum_us: float = 1200.0,
        maximum_us: float = 1500.0,
        slew_limit_us_s: float = 600.0,
    ) -> None:
        self.source_us = float(source_us)
        self.duration_s = float(duration_s)
        self.minimum_us = float(minimum_us)
        self.maximum_us = float(maximum_us)
        self.slew_limit_us_s = float(slew_limit_us_s)
        self.output_us = self.source_us
        self.last_timestamp_s: float | None = None

    def update(self, timestamp_s: float, requested_target_us: float) -> ThrottleHandoverValue:
        now = float(timestamp_s)
        requested = float(requested_target_us)
        target = float(np.clip(requested, self.minimum_us, self.maximum_us))
        alpha = float(np.clip(now / max(self.duration_s, 1.0e-12), 0.0, 1.0))
        handover_target = self.source_us + alpha * (target - self.source_us)
        dt = 0.0 if self.last_timestamp_s is None else max(0.0, now - self.last_timestamp_s)
        self.last_timestamp_s = now
        maximum_delta = self.slew_limit_us_s * dt
        delta = float(np.clip(handover_target - self.output_us, -maximum_delta, maximum_delta))
        next_output = float(np.clip(self.output_us + delta, self.minimum_us, self.maximum_us))
        limited = not math.isclose(next_output, handover_target, abs_tol=1.0e-9)
        self.output_us = next_output
        return ThrottleHandoverValue(target, next_output, alpha, limited)


@dataclass(frozen=True)
class ThrottleCalibrationTable:
    commands: tuple[float, ...]
    load_factors_g: tuple[float, ...]
    source: str

    def __post_init__(self) -> None:
        if len(self.commands) < 2 or len(self.commands) != len(self.load_factors_g):
            raise ValueError("throttle calibration requires paired samples")
        if any(b <= a for a, b in zip(self.commands, self.commands[1:])):
            raise ValueError("throttle calibration commands must be strictly increasing")
        if any(b < a for a, b in zip(self.load_factors_g, self.load_factors_g[1:])):
            raise ValueError("throttle calibration loads must be monotonic")

    def command_for_load(self, load_factor_g: float) -> tuple[float, bool]:
        requested = float(load_factor_g)
        limited = requested < self.load_factors_g[0] or requested > self.load_factors_g[-1]
        value = float(np.interp(requested, self.load_factors_g, self.commands))
        return value, limited

    def load_for_command(self, command: float) -> float:
        return float(np.interp(float(command), self.commands, self.load_factors_g))


@dataclass(frozen=True)
class Log00106GuidanceOutput:
    phase: str
    valid: bool
    reason: str
    acceleration_ned_m_s2: np.ndarray
    velocity_reference_ned_m_s: np.ndarray
    speed_acceleration_ned_m_s2: np.ndarray
    png_acceleration_ned_m_s2: np.ndarray
    fov_acceleration_ned_m_s2: np.ndarray
    speed_saturated: bool
    png_saturated: bool
    fov_saturated: bool
    fov_priority_active: bool
    fov_priority_weight: float
    total_saturated: bool


class Log00106ControllerAdapter:
    """Run the production controller with the fixed-VM semantics flown in LOG00106.

    The current branch has a slewed velocity reference and a dynamic PNG speed.
    LOG00106 predates those changes. The adapter still delegates validation, LOS
    prediction, FOV geometry, and controller state to the production class, then
    applies the archived fixed-VM combination documented by the flight metadata.
    """

    def __init__(self) -> None:
        self.config = VelocityEstablishingPngConfig(
            fixed_vm_m_s=10.0,
            navigation_constant=3.0,
            speed_gain_s_inv=1.2,
            speed_accel_limit_m_s2=7.0,
            png_accel_limit_m_s2=7.0,
            fov_centering_gain_s2=16.0,
            fov_centering_accel_limit_m_s2=7.0,
            total_accel_limit_m_s2=7.0,
            vertical_speed_reference_limit_m_s=6.0,
            velocity_reference_slew_m_s2=1.0e6,
            acquire_consecutive_frames=1,
            detection_timeout_s=0.35,
            velocity_timeout_s=0.5,
            los_prediction_max_s=0.15,
            gravity_m_s2=GRAVITY_M_S2,
            fov_constraint_half_angle_deg=0.0,
            fov_priority=FovPriorityConfig(
                enabled=True,
                start_ratio=0.75,
                full_ratio=0.95,
                horizontal_half_fov_deg=31.000688982474795,
                vertical_half_fov_deg=24.91540513181656,
            ),
            engagement_policy=EngagementPolicy.CONTACT.value,
            contact_bbox_terminal_ratio=2.0,
            contact_bbox_complete_ratio=3.0,
            contact_ttc_terminal_s=1.0e-6,
        )
        self.production = VelocityEstablishingPngController(self.config)

    def reset(self) -> None:
        self.production.reset()

    def update(self, value: VelocityEstablishingPngInput) -> Log00106GuidanceOutput:
        production = self.production.update(value)
        zero = np.zeros(3, dtype=float)
        if not production.valid:
            return Log00106GuidanceOutput(
                phase=production.phase.value,
                valid=False,
                reason=production.reason,
                acceleration_ned_m_s2=zero,
                velocity_reference_ned_m_s=zero,
                speed_acceleration_ned_m_s2=zero,
                png_acceleration_ned_m_s2=zero,
                fov_acceleration_ned_m_s2=zero,
                speed_saturated=False,
                png_saturated=False,
                fov_saturated=False,
                fov_priority_active=False,
                fov_priority_weight=0.0,
                total_saturated=False,
            )

        detection_age = max(0.0, float(value.timestamp_s) - float(value.los_timestamp_s or value.timestamp_s))
        prediction_horizon = min(detection_age, self.config.los_prediction_max_s)
        control_los = normalize(
            np.asarray(value.lambda_ned, dtype=float)
            + prediction_horizon * np.asarray(value.lambda_dot_ned_s, dtype=float)
        )
        velocity_reference = self.config.fixed_vm_m_s * control_los
        velocity_reference[2] = float(
            np.clip(
                velocity_reference[2],
                -self.config.vertical_speed_reference_limit_m_s,
                self.config.vertical_speed_reference_limit_m_s,
            )
        )
        speed_raw = self.config.speed_gain_s_inv * (
            velocity_reference - np.asarray(value.velocity_ned_m_s, dtype=float)
        )
        speed, speed_saturated = _clip_norm(speed_raw, self.config.speed_accel_limit_m_s2)
        png_raw = (
            self.config.navigation_constant
            * self.config.fixed_vm_m_s
            * np.asarray(value.lambda_dot_ned_s, dtype=float)
        )
        png, png_saturated = _clip_norm(png_raw, self.config.png_accel_limit_m_s2)
        fov = np.asarray(production.fov_acceleration_ned_m_s2, dtype=float)
        non_fov = speed + png
        if production.fov_priority_active:
            non_fov = _suppress_fov_opposition(
                non_fov,
                fov,
                production.fov_priority_weight,
            )
        total, total_saturated = _clip_norm(
            non_fov + fov,
            self.config.total_accel_limit_m_s2,
        )
        los_speed = float(np.dot(np.asarray(value.velocity_ned_m_s, dtype=float), control_los))
        phase = "PNG_TRACK" if los_speed >= 0.8 * self.config.fixed_vm_m_s else "ACCELERATE"
        return Log00106GuidanceOutput(
            phase=phase,
            valid=True,
            reason="active",
            acceleration_ned_m_s2=total,
            velocity_reference_ned_m_s=velocity_reference,
            speed_acceleration_ned_m_s2=speed,
            png_acceleration_ned_m_s2=png,
            fov_acceleration_ned_m_s2=fov,
            speed_saturated=speed_saturated,
            png_saturated=png_saturated,
            fov_saturated=production.fov_saturated,
            fov_priority_active=production.fov_priority_active,
            fov_priority_weight=production.fov_priority_weight,
            total_saturated=total_saturated,
        )


def model_load_factor_from_pwm(throttle_us: float) -> float:
    throttle = float(throttle_us)
    if throttle <= 1275.0:
        return max(0.0, throttle / 1275.0)
    return 1.0 + (throttle - 1275.0) / (1500.0 - 1275.0) * (2.37 - 1.0)


def pwm_from_normalized_thrust(thrust: float) -> float:
    value = float(np.clip(thrust, 0.0, 1.0))
    if value <= 0.5:
        return 1200.0 + (value / 0.5) * (1275.0 - 1200.0)
    return 1275.0 + ((value - 0.5) / 0.5) * (1500.0 - 1275.0)


def euler_frd_from_R_IB(R_IB: np.ndarray) -> tuple[float, float, float]:
    rotation = np.asarray(R_IB, dtype=float)
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def closest_point_confirmed(
    closing_speed_history: Iterable[float], *, required_nonclosing_samples: int = 5
) -> bool:
    values = list(closing_speed_history)
    return (
        len(values) >= required_nonclosing_samples + 1
        and any(value > 0.0 for value in values[:-required_nonclosing_samples])
        and all(value <= 0.0 for value in values[-required_nonclosing_samples:])
    )


def validate_csv_row(row: Mapping[str, object]) -> None:
    missing = [field for field in CSV_FIELDS if field not in row]
    if missing:
        raise ValueError(f"LOG00106 CSV row missing fields: {', '.join(missing)}")
