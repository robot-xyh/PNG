from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
from functools import lru_cache
import math
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from .betaflight_intercept_controller import (
    FovPriorityConfig,
    InterceptPhase,
    VelocityEstablishingPngConfig,
    VelocityEstablishingPngController,
    VelocityEstablishingPngInput,
)
from .flight_control import (
    AccelerationTiltRateConfig,
    ThrustFeedforwardConfig,
    guidance_eval_to_setpoint,
)
from .los_filter import LOSFilterConfig, LOSKalmanFilter6D
from .thrust_model import VoltageThrottleThrustModel
from .types import GuidanceEval


CONTROLLER_MODES = (
    "fixed_thrust",
    "ideal_altitude_hold",
    "speed_hold_variable_thrust",
    "candidate_velocity_hold_variable_thrust",
)
START_PROFILES = ("hover", "established_speed")


@dataclass(frozen=True)
class MatrixCase:
    case_id: str
    horizontal_range_m: float
    lateral_offset_m: float
    altitude_offset_m: float
    target_speed_m_s: float
    speed_ratio: float = 2.0
    target_course_deg: float = 90.0


MATRIX15_CASES = (
    MatrixCase("M01", 40.0, -10.0, 30.0, 5.0),
    MatrixCase("M02", 25.0, -10.0, 30.0, 5.0),
    MatrixCase("M03", 55.0, -10.0, 30.0, 5.0),
    MatrixCase("M04", 40.0, 0.0, 30.0, 5.0),
    MatrixCase("M05", 40.0, -20.0, 30.0, 5.0),
    MatrixCase("M06", 40.0, 20.0, 30.0, 5.0),
    MatrixCase("M07", 40.0, -10.0, 20.0, 5.0),
    MatrixCase("M08", 40.0, -10.0, 40.0, 5.0),
    MatrixCase("M09", 30.0, -10.0, 30.0, 3.0),
    MatrixCase("M10", 30.0, -10.0, 30.0, 7.0),
    MatrixCase("M11", 50.0, -10.0, 30.0, 3.0),
    MatrixCase("M12", 50.0, -10.0, 30.0, 7.0),
    MatrixCase("M13", 45.0, -20.0, 40.0, 7.0),
    MatrixCase("M14", 55.0, 15.0, 20.0, 7.0),
    MatrixCase("M15", 25.0, 20.0, 40.0, 3.0),
)


@dataclass(frozen=True)
class ClosedLoopSimulationConfig:
    dt_s: float = 0.01
    duration_s: float = 40.0
    navigation_constant: float = 3.0
    guidance_accel_limit_m_s2: float = 20.0
    gravity_m_s2: float = 9.80665
    upward_centering_gain_s2: float = 8.0
    upward_centering_accel_limit_m_s2: float = 4.0
    speed_hold_gain_s_inv: float = 1.2
    speed_hold_accel_limit_m_s2: float = 8.0
    total_accel_limit_m_s2: float = 28.0
    vertical_speed_reference_limit_m_s: float = 6.0
    max_roll_tilt_deg: float = 35.0
    max_pitch_tilt_deg: float = 35.0
    attitude_kp_s_inv: float = 4.0
    max_roll_rate_deg_s: float = 120.0
    max_pitch_rate_deg_s: float = 120.0
    control_rate_hz: float = 0.0
    body_rate_command_delay_s: float = 0.0
    body_rate_response_tau_s: float = 0.04
    entry_handoff_enabled: bool = False
    entry_handoff_duration_s: float = 0.0
    altitude_hold_position_gain_s2: float = 1.0
    altitude_hold_velocity_gain_s_inv: float = 2.0
    min_thrust_specific_force_m_s2: float = 4.903325
    max_thrust_specific_force_m_s2: float = 16.671305
    throttle_dynamics_enabled: bool = False
    thrust_response_tau_s: float = 0.0
    throttle_handover_duration_s: float = 0.0
    throttle_slew_limit_us_per_s: float = 0.0
    throttle_min_us: float = 1200.0
    throttle_hover_us: float = 1275.0
    throttle_max_us: float = 1500.0
    hover_load_factor_g: float = 1.0
    max_load_factor_g: float = 2.37
    battery_voltage_v: float = 0.0
    thrust_model_path: str = ""
    thrust_model_sha256: str = ""
    thrust_model_calibration_id: str = ""
    collision_radius_m: float = 1.0
    near_hit_radius_m: float = 1.5
    camera_half_fov_deg: float = 60.0
    camera_horizontal_half_fov_deg: float = 0.0
    camera_vertical_half_fov_deg: float = 0.0
    perception_latency_s: float = 0.0
    perception_rate_hz: float = 0.0
    perception_stale_timeout_s: float = 0.35
    perception_fov_gate_enabled: bool = False
    random_seed: int = 0
    measurement_dropout_probability: float = 0.0
    measurement_dropout_burst_start_probability: float = 0.0
    measurement_dropout_burst_lengths: tuple[int, ...] = ()
    los_angle_noise_std_deg: float = 0.0
    relative_velocity_noise_std_m_s: float = 0.0
    wind_accel_std_m_s2: float = 0.0
    wind_time_constant_s: float = 1.0
    kinematic_rate_hz: float = 5.0
    kinematic_latency_s: float = 0.15
    kinematic_stale_timeout_s: float = 0.5
    kinematic_velocity_noise_std_m_s: float = 0.25
    kinematic_dropout_probability: float = 0.05
    los_filter_process_lambda: float = 1.0e-4
    los_filter_process_lambda_dot: float = 5.0e-3
    los_filter_measurement_noise: float = 5.0e-3
    los_filter_innovation_reject: float = 0.25
    candidate_png_track_speed_ratio: float = 0.8
    candidate_velocity_reference_slew_m_s2: float = 3.0
    candidate_acquire_consecutive_frames: int = 5
    candidate_los_prediction_max_s: float = 0.0
    candidate_fixed_vm_m_s: float = 0.0
    candidate_fov_constraint_half_angle_deg: float = 0.0
    candidate_fov_priority_enabled: bool = False
    candidate_fov_priority_start_ratio: float = 0.70
    candidate_fov_priority_full_ratio: float = 0.90
    candidate_engagement_policy: str = "contact"
    candidate_noncollision_bbox_abort_ratio: float = 0.012
    candidate_noncollision_ttc_abort_s: float = 2.0
    candidate_contact_bbox_terminal_ratio: float = 0.05
    candidate_contact_ttc_terminal_s: float = 1.0
    candidate_contact_bbox_complete_ratio: float = 0.25
    candidate_blind_hold_s: float = 0.20

    def __post_init__(self) -> None:
        positive = (
            "dt_s",
            "duration_s",
            "navigation_constant",
            "guidance_accel_limit_m_s2",
            "gravity_m_s2",
            "upward_centering_accel_limit_m_s2",
            "speed_hold_accel_limit_m_s2",
            "total_accel_limit_m_s2",
            "vertical_speed_reference_limit_m_s",
            "max_roll_tilt_deg",
            "max_pitch_tilt_deg",
            "attitude_kp_s_inv",
            "max_roll_rate_deg_s",
            "max_pitch_rate_deg_s",
            "body_rate_response_tau_s",
            "altitude_hold_position_gain_s2",
            "altitude_hold_velocity_gain_s_inv",
            "min_thrust_specific_force_m_s2",
            "max_thrust_specific_force_m_s2",
            "collision_radius_m",
            "near_hit_radius_m",
            "camera_half_fov_deg",
            "perception_stale_timeout_s",
            "wind_time_constant_s",
            "kinematic_rate_hz",
            "kinematic_stale_timeout_s",
            "los_filter_measurement_noise",
            "los_filter_innovation_reject",
            "candidate_velocity_reference_slew_m_s2",
            "candidate_noncollision_bbox_abort_ratio",
            "candidate_noncollision_ttc_abort_s",
            "candidate_contact_bbox_terminal_ratio",
            "candidate_contact_ttc_terminal_s",
            "candidate_contact_bbox_complete_ratio",
            "candidate_blind_hold_s",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        for name in (
            "upward_centering_gain_s2",
            "speed_hold_gain_s_inv",
            "body_rate_command_delay_s",
            "perception_latency_s",
            "perception_rate_hz",
            "los_angle_noise_std_deg",
            "relative_velocity_noise_std_m_s",
            "wind_accel_std_m_s2",
            "kinematic_latency_s",
            "kinematic_velocity_noise_std_m_s",
            "los_filter_process_lambda",
            "los_filter_process_lambda_dot",
            "candidate_los_prediction_max_s",
            "candidate_fixed_vm_m_s",
            "camera_horizontal_half_fov_deg",
            "camera_vertical_half_fov_deg",
            "candidate_fov_constraint_half_angle_deg",
            "control_rate_hz",
            "entry_handoff_duration_s",
            "throttle_handover_duration_s",
            "throttle_slew_limit_us_per_s",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.max_roll_tilt_deg >= 90.0 or self.max_pitch_tilt_deg >= 90.0:
            raise ValueError("tilt limits must be below 90 degrees")
        if self.camera_half_fov_deg >= 180.0:
            raise ValueError("camera half FOV must be below 180 degrees")
        rectangular_fov = (
            float(self.camera_horizontal_half_fov_deg),
            float(self.camera_vertical_half_fov_deg),
        )
        if any(value < 0.0 or value >= 90.0 for value in rectangular_fov):
            raise ValueError("rectangular camera half FOV values must be in [0, 90)")
        if (rectangular_fov[0] > 0.0) != (rectangular_fov[1] > 0.0):
            raise ValueError("both rectangular camera half FOV values must be configured")
        if self.perception_latency_s > self.perception_stale_timeout_s:
            raise ValueError("perception latency cannot exceed the stale timeout")
        if isinstance(self.random_seed, bool) or int(self.random_seed) != self.random_seed:
            raise ValueError("random_seed must be an integer")
        if int(self.random_seed) < 0:
            raise ValueError("random_seed must be non-negative")
        dropout_probability = float(self.measurement_dropout_probability)
        if not math.isfinite(dropout_probability) or not 0.0 <= dropout_probability <= 1.0:
            raise ValueError("measurement_dropout_probability must be in [0, 1]")
        burst_probability = float(self.measurement_dropout_burst_start_probability)
        if not math.isfinite(burst_probability) or not 0.0 <= burst_probability <= 1.0:
            raise ValueError(
                "measurement_dropout_burst_start_probability must be in [0, 1]"
            )
        burst_lengths = tuple(self.measurement_dropout_burst_lengths)
        if any(
            isinstance(length, bool)
            or not isinstance(length, (int, np.integer))
            or int(length) <= 0
            for length in burst_lengths
        ):
            raise ValueError("measurement_dropout_burst_lengths must contain positive integers")
        if burst_probability > 0.0 and not burst_lengths:
            raise ValueError(
                "measurement_dropout_burst_lengths are required when burst dropout is enabled"
            )
        if burst_probability > 0.0 and dropout_probability > 0.0:
            raise ValueError("independent and burst measurement dropout cannot both be enabled")
        kinematic_dropout_probability = float(self.kinematic_dropout_probability)
        if not 0.0 <= kinematic_dropout_probability <= 1.0:
            raise ValueError("kinematic_dropout_probability must be in [0, 1]")
        if self.kinematic_latency_s > self.kinematic_stale_timeout_s:
            raise ValueError("kinematic latency cannot exceed the stale timeout")
        if self.min_thrust_specific_force_m_s2 > self.max_thrust_specific_force_m_s2:
            raise ValueError("minimum thrust cannot exceed maximum thrust")
        if self.entry_handoff_enabled and self.entry_handoff_duration_s <= 0.0:
            raise ValueError("enabled entry handoff requires a positive duration")
        if not (
            self.throttle_min_us
            < self.throttle_hover_us
            < self.throttle_max_us
        ):
            raise ValueError("throttle PWM points must be strictly increasing")
        if not math.isfinite(self.hover_load_factor_g) or self.hover_load_factor_g <= 0.0:
            raise ValueError("hover_load_factor_g must be finite and positive")
        if (
            not math.isfinite(self.max_load_factor_g)
            or self.max_load_factor_g <= self.hover_load_factor_g
        ):
            raise ValueError("max_load_factor_g must exceed hover_load_factor_g")
        if self.throttle_dynamics_enabled and self.throttle_slew_limit_us_per_s <= 0.0:
            raise ValueError(
                "enabled throttle dynamics require a positive throttle slew limit"
            )
        if self.throttle_dynamics_enabled:
            if not math.isfinite(self.thrust_response_tau_s) or self.thrust_response_tau_s <= 0.0:
                raise ValueError(
                    "enabled throttle dynamics require a positive thrust_response_tau_s"
                )
            if not math.isfinite(self.battery_voltage_v) or self.battery_voltage_v <= 0.0:
                raise ValueError(
                    "enabled throttle dynamics require a positive battery_voltage_v"
                )
            if not str(self.thrust_model_path).strip():
                raise ValueError(
                    "enabled throttle dynamics require a thrust_model_path"
                )
            digest = str(self.thrust_model_sha256).strip().lower()
            if len(digest) != 64 or any(
                character not in "0123456789abcdef" for character in digest
            ):
                raise ValueError(
                    "enabled throttle dynamics require a thrust_model_sha256"
                )
            if not str(self.thrust_model_calibration_id).strip():
                raise ValueError(
                    "enabled throttle dynamics require a thrust_model_calibration_id"
                )
        if self.collision_radius_m > self.near_hit_radius_m:
            raise ValueError("collision radius cannot exceed near-hit radius")
        if not 0.0 < float(self.candidate_png_track_speed_ratio) <= 1.0:
            raise ValueError("candidate_png_track_speed_ratio must be in (0, 1]")
        if self.candidate_engagement_policy not in {"noncollision", "contact"}:
            raise ValueError("candidate_engagement_policy must be noncollision or contact")
        if self.candidate_fov_constraint_half_angle_deg >= 90.0:
            raise ValueError("candidate_fov_constraint_half_angle_deg must be below 90")
        for name in (
            "candidate_fov_priority_start_ratio",
            "candidate_fov_priority_full_ratio",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if (
            self.candidate_fov_priority_full_ratio
            <= self.candidate_fov_priority_start_ratio
        ):
            raise ValueError("candidate FOV priority full ratio must exceed start ratio")
        if self.candidate_fov_priority_enabled and not all(
            value > 0.0 for value in rectangular_fov
        ):
            raise ValueError(
                "enabled candidate FOV priority requires rectangular camera FOV"
            )
        if (
            isinstance(self.candidate_acquire_consecutive_frames, bool)
            or int(self.candidate_acquire_consecutive_frames)
            != self.candidate_acquire_consecutive_frames
            or int(self.candidate_acquire_consecutive_frames) < 1
        ):
            raise ValueError("candidate_acquire_consecutive_frames must be a positive integer")


@dataclass(frozen=True)
class ClosedLoopSimulationResult:
    case_id: str
    controller_mode: str
    start_profile: str
    fixed_vm_m_s: float
    hit: bool
    near_hit: bool
    fov_feasible_hit: bool
    target_continuously_in_fov: bool
    minimum_range_m: float
    minimum_range_time_s: float
    final_range_m: float
    elapsed_s: float
    maximum_speed_m_s: float
    maximum_altitude_loss_m: float
    maximum_climb_m: float
    maximum_vertical_displacement_m: float
    maximum_roll_deg: float
    maximum_pitch_deg: float
    maximum_commanded_rate_deg_s: float
    maximum_guidance_accel_m_s2: float
    maximum_control_accel_m_s2: float
    control_update_count: int
    entry_handoff_active_fraction: float
    fov_priority_active_fraction: float
    guidance_accel_saturation_fraction: float
    centering_accel_saturation_fraction: float
    speed_hold_accel_saturation_fraction: float
    total_accel_saturation_fraction: float
    tilt_saturation_fraction: float
    rate_saturation_fraction: float
    thrust_saturation_fraction: float
    throttle_handover_active_fraction: float
    throttle_slew_saturation_fraction: float
    minimum_throttle_us: float | None
    maximum_throttle_us: float | None
    maximum_load_factor_g: float | None
    target_in_fov_fraction: float
    maximum_target_off_up_axis_deg: float
    initial_target_in_fov: bool
    measurement_valid_fraction: float
    measurement_capture_count: int
    measurement_delivered_count: int
    measurement_fov_reject_count: int
    measurement_dropout_count: int
    measurement_dropout_fraction: float
    maximum_measurement_angle_error_deg: float
    maximum_relative_velocity_error_m_s: float
    maximum_wind_accel_m_s2: float
    maximum_measurement_age_s: float | None
    first_measurement_valid_time_s: float | None
    first_measurement_stale_time_s: float | None
    kinematic_valid_fraction: float
    maximum_kinematic_velocity_error_m_s: float
    controller_final_phase: str
    controller_final_reason: str
    controller_abort_time_s: float | None
    outcome_reason: str

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class _DelayedMeasurement:
    exposure_time_s: float
    available_time_s: float
    relative_position_m: np.ndarray
    relative_velocity_m_s: np.ndarray


@dataclass(frozen=True)
class _DelayedKinematicState:
    sample_time_s: float
    available_time_s: float
    velocity_ned_m_s: np.ndarray


def simulate_matrix15(
    *,
    config: ClosedLoopSimulationConfig | None = None,
    controller_modes: Sequence[str] = CONTROLLER_MODES,
    start_profiles: Sequence[str] = START_PROFILES,
    cases: Iterable[MatrixCase] = MATRIX15_CASES,
) -> dict[str, object]:
    cfg = config or ClosedLoopSimulationConfig()
    selected_cases = tuple(cases)
    results = [
        simulate_case(case, controller_mode=mode, start_profile=start, config=cfg)
        for start in start_profiles
        for mode in controller_modes
        for case in selected_cases
    ]
    summaries = []
    for start in start_profiles:
        for mode in controller_modes:
            selected = [
                result
                for result in results
                if result.start_profile == start and result.controller_mode == mode
            ]
            summaries.append(_summarize(selected, controller_mode=mode, start_profile=start))
    return {
        "schema_version": 2,
        "purpose": "deterministic sampled-LOS Betaflight PNG closed-loop interception evaluation",
        "limitations": [
            "This is a point-mass and first-order body-rate model, not a flight approval.",
            "Truth relative state bypasses detector noise and association errors; configured sampling, latency, FOV gating, and staleness are deterministic.",
            "The variable-thrust path is an ideal force-projection reference, not a Betaflight motor model.",
            "Target motion is the straight world-Y motion used by the matrix15 launch scripts.",
        ],
        "config": asdict(cfg),
        "cases": [asdict(case) for case in selected_cases],
        "summaries": summaries,
        "results": [result.to_dict() for result in results],
    }


def simulate_case(
    case: MatrixCase,
    *,
    controller_mode: str,
    start_profile: str,
    config: ClosedLoopSimulationConfig | None = None,
) -> ClosedLoopSimulationResult:
    if controller_mode not in CONTROLLER_MODES:
        raise ValueError(f"unsupported controller mode: {controller_mode}")
    if start_profile not in START_PROFILES:
        raise ValueError(f"unsupported start profile: {start_profile}")
    cfg = config or ClosedLoopSimulationConfig()
    _validate_case(case)
    thrust_model = _configured_thrust_model(cfg)
    dropout_rng, measurement_rng, wind_rng, kinematic_rng = _simulation_rngs(
        cfg.random_seed,
        case_id=case.case_id,
    )

    forward = math.sqrt(
        max(0.0, case.horizontal_range_m**2 - case.lateral_offset_m**2)
    )
    interceptor_position = np.zeros(3, dtype=float)
    target_position = np.array(
        [forward, case.lateral_offset_m, -case.altitude_offset_m], dtype=float
    )
    target_course_rad = math.radians(case.target_course_deg)
    target_velocity = case.target_speed_m_s * np.array(
        [math.cos(target_course_rad), math.sin(target_course_rad), 0.0],
        dtype=float,
    )
    fixed_vm = float(
        cfg.candidate_fixed_vm_m_s
        if cfg.candidate_fixed_vm_m_s > 0.0
        else case.speed_ratio * case.target_speed_m_s
    )
    initial_los = _normalized(target_position - interceptor_position)
    interceptor_velocity = (
        _velocity_reference(initial_los, fixed_vm, cfg)
        if start_profile == "established_speed"
        else np.zeros(3, dtype=float)
    )

    roll_rad = 0.0
    pitch_rad = 0.0
    yaw_rad = math.atan2(float(target_position[1]), float(target_position[0]))
    actual_p_rad_s = 0.0
    actual_q_rad_s = 0.0
    body_rate_command_history: deque[tuple[float, float, float]] = deque()
    mapping_config = AccelerationTiltRateConfig(
        gravity_mps2=cfg.gravity_m_s2,
        roll_attitude_kp_s_inv=cfg.attitude_kp_s_inv,
        pitch_attitude_kp_s_inv=cfg.attitude_kp_s_inv,
        max_roll_tilt_deg=cfg.max_roll_tilt_deg,
        max_pitch_tilt_deg=cfg.max_pitch_tilt_deg,
        max_roll_rate_deg_s=cfg.max_roll_rate_deg_s,
        max_pitch_rate_deg_s=cfg.max_pitch_rate_deg_s,
        thrust_feedforward=ThrustFeedforwardConfig(
            enabled=cfg.throttle_dynamics_enabled,
            model=(
                "voltage_throttle_lut"
                if cfg.throttle_dynamics_enabled
                else "fixed_hover"
            ),
            hover_load_factor_g=cfg.hover_load_factor_g,
            max_load_factor_g=cfg.max_load_factor_g,
            calibration_id=cfg.thrust_model_calibration_id,
            model_path=cfg.thrust_model_path,
            model_sha256=cfg.thrust_model_sha256,
        ),
    )
    candidate_controller = (
        VelocityEstablishingPngController(
            VelocityEstablishingPngConfig(
                fixed_vm_m_s=fixed_vm,
                navigation_constant=cfg.navigation_constant,
                speed_gain_s_inv=cfg.speed_hold_gain_s_inv,
                speed_accel_limit_m_s2=cfg.speed_hold_accel_limit_m_s2,
                png_accel_limit_m_s2=cfg.guidance_accel_limit_m_s2,
                fov_centering_gain_s2=cfg.upward_centering_gain_s2,
                fov_centering_accel_limit_m_s2=cfg.upward_centering_accel_limit_m_s2,
                total_accel_limit_m_s2=cfg.total_accel_limit_m_s2,
                vertical_speed_reference_limit_m_s=cfg.vertical_speed_reference_limit_m_s,
                png_track_speed_ratio=cfg.candidate_png_track_speed_ratio,
                velocity_reference_slew_m_s2=cfg.candidate_velocity_reference_slew_m_s2,
                acquire_consecutive_frames=cfg.candidate_acquire_consecutive_frames,
                detection_timeout_s=cfg.perception_stale_timeout_s,
                velocity_timeout_s=cfg.kinematic_stale_timeout_s,
                los_prediction_max_s=cfg.candidate_los_prediction_max_s,
                gravity_m_s2=cfg.gravity_m_s2,
                fov_constraint_half_angle_deg=cfg.candidate_fov_constraint_half_angle_deg,
                fov_priority=FovPriorityConfig(
                    enabled=cfg.candidate_fov_priority_enabled,
                    start_ratio=cfg.candidate_fov_priority_start_ratio,
                    full_ratio=cfg.candidate_fov_priority_full_ratio,
                    horizontal_half_fov_deg=cfg.camera_horizontal_half_fov_deg,
                    vertical_half_fov_deg=cfg.camera_vertical_half_fov_deg,
                ),
                engagement_policy=cfg.candidate_engagement_policy,
                noncollision_bbox_abort_ratio=cfg.candidate_noncollision_bbox_abort_ratio,
                noncollision_ttc_abort_s=cfg.candidate_noncollision_ttc_abort_s,
                contact_bbox_terminal_ratio=cfg.candidate_contact_bbox_terminal_ratio,
                contact_ttc_terminal_s=cfg.candidate_contact_ttc_terminal_s,
                contact_bbox_complete_ratio=cfg.candidate_contact_bbox_complete_ratio,
                blind_hold_s=cfg.candidate_blind_hold_s,
            )
        )
        if controller_mode == "candidate_velocity_hold_variable_thrust"
        else None
    )
    candidate_los_filter = (
        LOSKalmanFilter6D(
            LOSFilterConfig(
                process_lambda=cfg.los_filter_process_lambda,
                process_lambda_dot=cfg.los_filter_process_lambda_dot,
                measurement_noise=cfg.los_filter_measurement_noise,
                innovation_reject=cfg.los_filter_innovation_reject,
            )
        )
        if candidate_controller is not None
        else None
    )

    minimum_range = float("inf")
    minimum_range_time = 0.0
    maximum_speed = 0.0
    maximum_altitude_loss = 0.0
    maximum_climb = 0.0
    maximum_vertical_displacement = 0.0
    maximum_roll = 0.0
    maximum_pitch = 0.0
    maximum_commanded_rate = 0.0
    maximum_guidance_accel = 0.0
    maximum_control_accel = 0.0
    counters = {
        "guidance": 0,
        "centering": 0,
        "speed_hold": 0,
        "total": 0,
        "tilt": 0,
        "rate": 0,
        "thrust": 0,
        "fov": 0,
        "measurement_valid": 0,
        "kinematic_valid": 0,
        "entry_handoff": 0,
        "fov_priority": 0,
        "throttle_handover": 0,
        "throttle_slew": 0,
    }
    initial_body_down_axis = _rotation_matrix_frd(
        roll_rad, pitch_rad, yaw_rad
    )[:, 2]
    initial_off_axis_deg = math.degrees(
        math.acos(
            float(np.clip(np.dot(initial_los, -initial_body_down_axis), -1.0, 1.0))
        )
    )
    maximum_off_axis_deg = initial_off_axis_deg
    initial_target_in_fov = _target_in_fov(
        initial_los,
        _rotation_matrix_frd(roll_rad, pitch_rad, yaw_rad),
        cfg,
    )
    target_continuously_in_fov = initial_target_in_fov
    pending_measurements: deque[_DelayedMeasurement] = deque()
    last_measurement: _DelayedMeasurement | None = None
    next_measurement_time_s = 0.0
    measurement_capture_count = 0
    measurement_delivered_count = 0
    measurement_fov_reject_count = 0
    measurement_dropout_count = 0
    measurement_dropout_remaining = 0
    maximum_measurement_angle_error_deg = 0.0
    maximum_relative_velocity_error_m_s = 0.0
    wind_accel_m_s2 = np.zeros(3, dtype=float)
    maximum_wind_accel_m_s2 = 0.0
    maximum_measurement_age_s: float | None = None
    first_measurement_valid_time_s: float | None = None
    first_measurement_stale_time_s: float | None = None
    measurement_was_valid = False
    candidate_los_estimate = None
    pending_kinematic_states: deque[_DelayedKinematicState] = deque()
    last_kinematic_state: _DelayedKinematicState | None = None
    next_kinematic_time_s = 0.0
    maximum_kinematic_velocity_error_m_s = 0.0
    candidate_output = None
    controller_abort_time_s: float | None = None
    control_update_count = 0
    next_control_time_s = 0.0
    held_p_command_rad_s = actual_p_rad_s
    held_q_command_rad_s = actual_q_rad_s
    entry_start_p_rad_s = actual_p_rad_s
    entry_start_q_rad_s = actual_q_rad_s
    current_throttle_us = float(cfg.throttle_hover_us)
    held_target_throttle_us = float(cfg.throttle_hover_us)
    minimum_throttle_us: float | None = (
        current_throttle_us if cfg.throttle_dynamics_enabled else None
    )
    maximum_throttle_us: float | None = minimum_throttle_us
    maximum_load_factor_g: float | None = (
        (
            thrust_model.specific_force(
                cfg.battery_voltage_v,
                cfg.throttle_hover_us,
            )
            / cfg.gravity_m_s2
        )
        if thrust_model is not None
        else None
    )
    actual_thrust_specific_force = (
        thrust_model.specific_force(cfg.battery_voltage_v, cfg.throttle_hover_us)
        if thrust_model is not None
        else cfg.gravity_m_s2
    )
    setpoint = None
    samples = 0
    hit = False
    elapsed = 0.0

    for index in range(int(math.ceil(cfg.duration_s / cfg.dt_s)) + 1):
        elapsed = min(cfg.duration_s, index * cfg.dt_s)
        relative_position = target_position - interceptor_position
        range_m = float(np.linalg.norm(relative_position))
        if range_m < minimum_range:
            minimum_range = range_m
            minimum_range_time = elapsed
        if range_m <= cfg.collision_radius_m:
            hit = True
            break
        if elapsed >= cfg.duration_s:
            break

        R_IB = _rotation_matrix_frd(roll_rad, pitch_rad, yaw_rad)
        body_down_axis = R_IB[:, 2]
        truth_los = _normalized(relative_position)
        relative_velocity = target_velocity - interceptor_velocity
        if candidate_controller is not None:
            kinematic_period_s = 1.0 / cfg.kinematic_rate_hz
            while next_kinematic_time_s <= elapsed + 1.0e-12:
                if float(kinematic_rng.random()) >= cfg.kinematic_dropout_probability:
                    measured_own_velocity = np.array(interceptor_velocity, dtype=float)
                    if cfg.kinematic_velocity_noise_std_m_s > 0.0:
                        measured_own_velocity += kinematic_rng.normal(
                            loc=0.0,
                            scale=cfg.kinematic_velocity_noise_std_m_s,
                            size=3,
                        )
                    maximum_kinematic_velocity_error_m_s = max(
                        maximum_kinematic_velocity_error_m_s,
                        float(np.linalg.norm(measured_own_velocity - interceptor_velocity)),
                    )
                    pending_kinematic_states.append(
                        _DelayedKinematicState(
                            sample_time_s=next_kinematic_time_s,
                            available_time_s=next_kinematic_time_s + cfg.kinematic_latency_s,
                            velocity_ned_m_s=measured_own_velocity,
                        )
                    )
                next_kinematic_time_s += kinematic_period_s
            while (
                pending_kinematic_states
                and pending_kinematic_states[0].available_time_s <= elapsed + 1.0e-12
            ):
                last_kinematic_state = pending_kinematic_states.popleft()
        truth_off_axis_deg = math.degrees(
            math.acos(
                float(np.clip(np.dot(truth_los, -body_down_axis), -1.0, 1.0))
            )
        )
        truth_target_in_fov = _target_in_fov(truth_los, R_IB, cfg)

        capture_times: list[float] = []
        if cfg.perception_rate_hz <= 0.0:
            capture_times.append(elapsed)
        else:
            measurement_period_s = 1.0 / cfg.perception_rate_hz
            while next_measurement_time_s <= elapsed + 1.0e-12:
                capture_times.append(next_measurement_time_s)
                next_measurement_time_s += measurement_period_s
        for exposure_time_s in capture_times:
            measurement_capture_count += 1
            if (
                cfg.perception_fov_gate_enabled
                and not truth_target_in_fov
            ):
                measurement_fov_reject_count += 1
                continue
            dropped = False
            if measurement_dropout_remaining > 0:
                measurement_dropout_remaining -= 1
                dropped = True
            elif (
                cfg.measurement_dropout_burst_start_probability > 0.0
                and float(dropout_rng.random())
                < cfg.measurement_dropout_burst_start_probability
            ):
                burst_lengths = tuple(cfg.measurement_dropout_burst_lengths)
                burst_index = int(dropout_rng.integers(0, len(burst_lengths)))
                burst_length = int(burst_lengths[burst_index])
                measurement_dropout_remaining = burst_length - 1
                dropped = True
            elif (
                cfg.measurement_dropout_probability > 0.0
                and float(dropout_rng.random()) < cfg.measurement_dropout_probability
            ):
                dropped = True
            if dropped:
                measurement_dropout_count += 1
                continue
            (
                measured_relative_position,
                measured_relative_velocity,
                measurement_angle_error_deg,
                relative_velocity_error_m_s,
            ) = _perturb_relative_measurement(
                relative_position,
                (
                    np.zeros(3, dtype=float)
                    if candidate_controller is not None
                    else relative_velocity
                ),
                angle_noise_std_deg=cfg.los_angle_noise_std_deg,
                velocity_noise_std_m_s=(
                    0.0
                    if candidate_controller is not None
                    else cfg.relative_velocity_noise_std_m_s
                ),
                rng=measurement_rng,
            )
            maximum_measurement_angle_error_deg = max(
                maximum_measurement_angle_error_deg, measurement_angle_error_deg
            )
            maximum_relative_velocity_error_m_s = max(
                maximum_relative_velocity_error_m_s, relative_velocity_error_m_s
            )
            pending_measurements.append(
                _DelayedMeasurement(
                    exposure_time_s=exposure_time_s,
                    available_time_s=exposure_time_s + cfg.perception_latency_s,
                    relative_position_m=measured_relative_position,
                    relative_velocity_m_s=measured_relative_velocity,
                )
            )
        while (
            pending_measurements
            and pending_measurements[0].available_time_s <= elapsed + 1.0e-12
        ):
            last_measurement = pending_measurements.popleft()
            measurement_delivered_count += 1
            if candidate_los_filter is not None:
                updated_los_estimate = candidate_los_filter.update(
                    last_measurement.exposure_time_s,
                    _normalized(last_measurement.relative_position_m),
                )
                if updated_los_estimate.valid:
                    candidate_los_estimate = updated_los_estimate

        measurement_age_s = (
            elapsed - last_measurement.exposure_time_s
            if last_measurement is not None
            else float("inf")
        )
        measurement_valid = bool(
            last_measurement is not None
            and measurement_age_s <= cfg.perception_stale_timeout_s + 1.0e-12
        )
        if measurement_valid:
            counters["measurement_valid"] += 1
            if first_measurement_valid_time_s is None:
                first_measurement_valid_time_s = elapsed
            maximum_measurement_age_s = max(
                maximum_measurement_age_s or 0.0, measurement_age_s
            )
            measurement_was_valid = True
            if candidate_controller is not None:
                los = (
                    np.zeros(3, dtype=float)
                    if candidate_los_estimate is None
                    else np.array(candidate_los_estimate.lambda_I, dtype=float)
                )
                guidance = np.zeros(3, dtype=float)
                guidance_saturated = False
            else:
                measured_position = last_measurement.relative_position_m
                measured_velocity = last_measurement.relative_velocity_m_s
                measured_range_m = float(np.linalg.norm(measured_position))
                los = _normalized(measured_position)
                omega_los = np.cross(measured_position, measured_velocity) / max(
                    measured_range_m * measured_range_m, 1.0e-12
                )
                raw_guidance = (
                    cfg.navigation_constant * fixed_vm * np.cross(omega_los, los)
                )
                guidance, guidance_saturated = _clip_norm_with_flag(
                    raw_guidance, cfg.guidance_accel_limit_m_s2
                )
        else:
            if (
                measurement_was_valid
                and first_measurement_stale_time_s is None
                and last_measurement is not None
                and measurement_age_s > cfg.perception_stale_timeout_s
            ):
                first_measurement_stale_time_s = elapsed
            los = np.zeros(3, dtype=float)
            guidance = np.zeros(3, dtype=float)
            guidance_saturated = False

        centering = np.zeros(3, dtype=float)
        speed_hold = np.zeros(3, dtype=float)
        centering_saturated = False
        speed_hold_saturated = False
        total_saturated = False
        kinematic_age_s = (
            float("inf")
            if last_kinematic_state is None
            else elapsed - last_kinematic_state.sample_time_s
        )
        kinematic_valid = bool(
            last_kinematic_state is not None
            and kinematic_age_s <= cfg.kinematic_stale_timeout_s + 1.0e-12
        )
        if kinematic_valid:
            counters["kinematic_valid"] += 1
        if candidate_controller is not None:
            candidate_output = candidate_controller.update(
                VelocityEstablishingPngInput(
                    timestamp_s=elapsed,
                    los_timestamp_s=(
                        None
                        if last_measurement is None
                        else last_measurement.exposure_time_s
                    ),
                    lambda_ned=(
                        None
                        if candidate_los_estimate is None
                        else candidate_los_estimate.lambda_I
                    ),
                    lambda_dot_ned_s=(
                        None
                        if candidate_los_estimate is None
                        else candidate_los_estimate.lambda_dot_I
                    ),
                    tracking_valid=bool(
                        candidate_los_estimate is not None
                        and elapsed - candidate_los_estimate.timestamp
                        <= cfg.perception_stale_timeout_s + 1.0e-12
                    ),
                    bbox_area_ratio=(
                        None
                        if last_measurement is None
                        else min(
                            1.0,
                            cfg.collision_radius_m**2
                            / max(
                                float(np.dot(last_measurement.relative_position_m, last_measurement.relative_position_m)),
                                1.0e-12,
                            ),
                        )
                    ),
                    attitude_R_IB=R_IB,
                    attitude_valid=True,
                    velocity_timestamp_s=(
                        None
                        if last_kinematic_state is None
                        else last_kinematic_state.sample_time_s
                    ),
                    velocity_ned_m_s=(
                        None
                        if last_kinematic_state is None
                        else last_kinematic_state.velocity_ned_m_s
                    ),
                    velocity_valid=kinematic_valid,
                    track_id=1,
                )
            )
            if candidate_output.phase == InterceptPhase.ABORT and controller_abort_time_s is None:
                controller_abort_time_s = elapsed
            centering = np.array(candidate_output.fov_acceleration_ned_m_s2, dtype=float)
            speed_hold = np.array(candidate_output.speed_acceleration_ned_m_s2, dtype=float)
            guidance = np.array(candidate_output.png_acceleration_ned_m_s2, dtype=float)
            control_accel = np.array(candidate_output.acceleration_ned_m_s2, dtype=float)
            centering_saturated = candidate_output.fov_saturated
            speed_hold_saturated = candidate_output.speed_saturated
            guidance_saturated = candidate_output.png_saturated
            total_saturated = candidate_output.total_saturated
        elif controller_mode == "speed_hold_variable_thrust" and measurement_valid:
            los_body = R_IB.T @ los
            raw_centering_body = np.array(
                [
                    cfg.upward_centering_gain_s2 * float(los_body[0]),
                    cfg.upward_centering_gain_s2 * float(los_body[1]),
                    0.0,
                ],
                dtype=float,
            )
            centering_body, centering_saturated = _clip_norm_with_flag(
                raw_centering_body, cfg.upward_centering_accel_limit_m_s2
            )
            centering = R_IB @ centering_body
            guidance, second_guidance_clip = _clip_norm_with_flag(
                guidance + centering, cfg.guidance_accel_limit_m_s2
            )
            guidance_saturated = guidance_saturated or second_guidance_clip
            velocity_reference = _velocity_reference(los, fixed_vm, cfg)
            speed_hold, speed_hold_saturated = _clip_norm_with_flag(
                cfg.speed_hold_gain_s_inv
                * (velocity_reference - interceptor_velocity),
                cfg.speed_hold_accel_limit_m_s2,
            )
            control_accel, total_saturated = _clip_norm_with_flag(
                guidance + speed_hold, cfg.total_accel_limit_m_s2
            )
        else:
            control_accel = guidance

        control_update_due = bool(
            cfg.control_rate_hz <= 0.0
            or next_control_time_s <= elapsed + 1.0e-12
        )
        if control_update_due:
            setpoint = guidance_eval_to_setpoint(
                GuidanceEval(elapsed, control_accel, True, 1.0),
                R_IB=R_IB,
                rate_gain_matrix=np.zeros((3, 3), dtype=float),
                hover_thrust=0.5,
                mapping_type="accel_tilt_rate",
                accel_tilt_rate=mapping_config,
                thrust_model=thrust_model,
                battery_voltage_v=(
                    cfg.battery_voltage_v
                    if cfg.throttle_dynamics_enabled
                    else None
                ),
            )
            if not setpoint.valid:
                raise RuntimeError(
                    "thrust-model setpoint generation failed: "
                    f"{setpoint.reject_reason}"
                )
            held_p_command_rad_s = math.radians(setpoint.roll_rate_deg_s)
            held_q_command_rad_s = math.radians(setpoint.pitch_rate_deg_s)
            if cfg.entry_handoff_enabled:
                progress = _smoothstep01(
                    elapsed / max(cfg.entry_handoff_duration_s, 1.0e-12)
                )
                held_p_command_rad_s = _lerp(
                    entry_start_p_rad_s, held_p_command_rad_s, progress
                )
                held_q_command_rad_s = _lerp(
                    entry_start_q_rad_s, held_q_command_rad_s, progress
                )
            held_target_throttle_us = (
                float(setpoint.throttle_target_us)
                if setpoint.throttle_target_us is not None
                else _thrust_to_pwm(setpoint.thrust, cfg)
            )
            body_rate_command_history.append(
                (elapsed, held_p_command_rad_s, held_q_command_rad_s)
            )
            control_update_count += 1
            if cfg.control_rate_hz > 0.0:
                control_period_s = 1.0 / cfg.control_rate_hz
                while next_control_time_s <= elapsed + 1.0e-12:
                    next_control_time_s += control_period_s
        if setpoint is None:
            raise RuntimeError("control setpoint was not initialized")
        if cfg.control_rate_hz > 0.0:
            p_command, q_command = _delayed_held_body_rate_command(
                body_rate_command_history,
                elapsed - cfg.body_rate_command_delay_s,
            )
        else:
            p_command, q_command = _delayed_body_rate_command(
                body_rate_command_history,
                elapsed - cfg.body_rate_command_delay_s,
            )
        response_alpha = 1.0 - math.exp(-cfg.dt_s / cfg.body_rate_response_tau_s)
        actual_p_rad_s += response_alpha * (p_command - actual_p_rad_s)
        actual_q_rad_s += response_alpha * (q_command - actual_q_rad_s)
        roll_rad, pitch_rad, yaw_rad = _integrate_euler_frd(
            roll_rad,
            pitch_rad,
            yaw_rad,
            actual_p_rad_s,
            actual_q_rad_s,
            0.0,
            cfg.dt_s,
        )
        R_IB = _rotation_matrix_frd(roll_rad, pitch_rad, yaw_rad)
        body_down_axis = R_IB[:, 2]

        thrust_saturated = False
        if controller_mode == "fixed_thrust":
            thrust_specific_force = cfg.gravity_m_s2
        elif controller_mode == "ideal_altitude_hold":
            desired_vertical_accel = -(
                cfg.altitude_hold_position_gain_s2 * float(interceptor_position[2])
                + cfg.altitude_hold_velocity_gain_s_inv
                * float(interceptor_velocity[2])
            )
            thrust_specific_force = (
                cfg.gravity_m_s2 - desired_vertical_accel
            ) / max(0.20, float(body_down_axis[2]))
        elif cfg.throttle_dynamics_enabled:
            handover_active = bool(
                cfg.throttle_handover_duration_s > 0.0
                and elapsed < cfg.throttle_handover_duration_s
            )
            handover_alpha = (
                1.0
                if cfg.throttle_handover_duration_s <= 0.0
                else float(
                    np.clip(
                        elapsed / cfg.throttle_handover_duration_s,
                        0.0,
                        1.0,
                    )
                )
            )
            handover_target_us = _lerp(
                cfg.throttle_hover_us,
                held_target_throttle_us,
                handover_alpha,
            )
            maximum_delta_us = cfg.throttle_slew_limit_us_per_s * cfg.dt_s
            throttle_delta_us = float(
                np.clip(
                    handover_target_us - current_throttle_us,
                    -maximum_delta_us,
                    maximum_delta_us,
                )
            )
            next_throttle_us = current_throttle_us + throttle_delta_us
            throttle_slew_saturated = not math.isclose(
                next_throttle_us,
                handover_target_us,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            current_throttle_us = float(
                np.clip(
                    next_throttle_us,
                    cfg.throttle_min_us,
                    cfg.throttle_max_us,
                )
            )
            if thrust_model is None:
                raise RuntimeError("throttle dynamics require a loaded thrust LUT")
            target_thrust_specific_force = thrust_model.specific_force(
                cfg.battery_voltage_v,
                current_throttle_us,
            )
            thrust_alpha = 1.0 - math.exp(-cfg.dt_s / cfg.thrust_response_tau_s)
            actual_thrust_specific_force += thrust_alpha * (
                target_thrust_specific_force - actual_thrust_specific_force
            )
            thrust_specific_force = actual_thrust_specific_force
            load_factor_g = thrust_specific_force / cfg.gravity_m_s2
            thrust_saturated = bool(setpoint.thrust_command_limited)
            counters["throttle_handover"] += int(handover_active)
            counters["throttle_slew"] += int(throttle_slew_saturated)
            minimum_throttle_us = min(
                float(minimum_throttle_us), current_throttle_us
            )
            maximum_throttle_us = max(
                float(maximum_throttle_us), current_throttle_us
            )
            maximum_load_factor_g = max(
                float(maximum_load_factor_g), load_factor_g
            )
        else:
            required_specific_force = (
                np.array([0.0, 0.0, cfg.gravity_m_s2], dtype=float)
                - control_accel
            )
            raw_thrust = float(np.dot(required_specific_force, body_down_axis))
            thrust_specific_force = float(
                np.clip(
                    raw_thrust,
                    cfg.min_thrust_specific_force_m_s2,
                    cfg.max_thrust_specific_force_m_s2,
                )
            )
            thrust_saturated = not math.isclose(
                thrust_specific_force, raw_thrust, rel_tol=0.0, abs_tol=1.0e-9
            )
        actual_acceleration = (
            np.array([0.0, 0.0, cfg.gravity_m_s2], dtype=float)
            - thrust_specific_force * body_down_axis
        )
        if cfg.wind_accel_std_m_s2 > 0.0:
            wind_decay = math.exp(-cfg.dt_s / cfg.wind_time_constant_s)
            wind_noise_scale = cfg.wind_accel_std_m_s2 * math.sqrt(
                max(0.0, 1.0 - wind_decay * wind_decay)
            )
            wind_accel_m_s2 = (
                wind_decay * wind_accel_m_s2
                + wind_noise_scale * wind_rng.normal(size=3)
            )
        actual_acceleration += wind_accel_m_s2
        maximum_wind_accel_m_s2 = max(
            maximum_wind_accel_m_s2, float(np.linalg.norm(wind_accel_m_s2))
        )
        interceptor_velocity += actual_acceleration * cfg.dt_s
        interceptor_position += interceptor_velocity * cfg.dt_s
        target_position += target_velocity * cfg.dt_s

        desired_roll = float(setpoint.desired_roll_angle_deg or 0.0)
        desired_pitch = float(setpoint.desired_pitch_angle_deg or 0.0)
        tilt_saturated = (
            abs(desired_roll) >= cfg.max_roll_tilt_deg - 1.0e-9
            or abs(desired_pitch) >= cfg.max_pitch_tilt_deg - 1.0e-9
        )
        rate_saturated = (
            abs(setpoint.roll_rate_deg_s) >= cfg.max_roll_rate_deg_s - 1.0e-9
            or abs(setpoint.pitch_rate_deg_s) >= cfg.max_pitch_rate_deg_s - 1.0e-9
        )
        updated_los = _normalized(target_position - interceptor_position)
        off_axis_deg = math.degrees(
            math.acos(
                float(
                    np.clip(
                        np.dot(updated_los, -body_down_axis), -1.0, 1.0
                    )
                )
            )
        )

        samples += 1
        counters["guidance"] += int(guidance_saturated)
        counters["centering"] += int(centering_saturated)
        counters["speed_hold"] += int(speed_hold_saturated)
        counters["total"] += int(total_saturated)
        counters["tilt"] += int(tilt_saturated)
        counters["rate"] += int(rate_saturated)
        counters["thrust"] += int(thrust_saturated)
        counters["entry_handoff"] += int(
            cfg.entry_handoff_enabled
            and elapsed < cfg.entry_handoff_duration_s
        )
        counters["fov_priority"] += int(
            candidate_output is not None
            and candidate_output.fov_priority_active
        )
        updated_target_in_fov = _target_in_fov(updated_los, R_IB, cfg)
        counters["fov"] += int(updated_target_in_fov)
        target_continuously_in_fov = bool(
            target_continuously_in_fov and updated_target_in_fov
        )
        maximum_off_axis_deg = max(maximum_off_axis_deg, off_axis_deg)
        maximum_speed = max(maximum_speed, float(np.linalg.norm(interceptor_velocity)))
        maximum_altitude_loss = max(
            maximum_altitude_loss, float(interceptor_position[2])
        )
        maximum_climb = max(maximum_climb, -float(interceptor_position[2]))
        maximum_vertical_displacement = max(
            maximum_vertical_displacement, abs(float(interceptor_position[2]))
        )
        maximum_roll = max(maximum_roll, abs(math.degrees(roll_rad)))
        maximum_pitch = max(maximum_pitch, abs(math.degrees(pitch_rad)))
        maximum_commanded_rate = max(
            maximum_commanded_rate,
            abs(setpoint.roll_rate_deg_s),
            abs(setpoint.pitch_rate_deg_s),
        )
        maximum_guidance_accel = max(
            maximum_guidance_accel, float(np.linalg.norm(guidance))
        )
        maximum_control_accel = max(
            maximum_control_accel, float(np.linalg.norm(control_accel))
        )

    final_range = float(np.linalg.norm(target_position - interceptor_position))
    denominator = max(1, samples)
    eligible_measurement_count = max(
        0, measurement_capture_count - measurement_fov_reject_count
    )
    measurement_dropout_fraction = measurement_dropout_count / max(
        1, eligible_measurement_count
    )
    near_hit = minimum_range <= cfg.near_hit_radius_m
    if hit:
        outcome_reason = "hit"
    elif near_hit:
        outcome_reason = "near_miss"
    elif (
        cfg.perception_fov_gate_enabled
        and not initial_target_in_fov
        and first_measurement_valid_time_s is None
    ):
        outcome_reason = "initial_target_out_of_fov"
    elif candidate_output is not None and candidate_output.phase == InterceptPhase.ABORT:
        outcome_reason = "controller_abort"
    elif first_measurement_stale_time_s is not None:
        outcome_reason = "target_stale"
    else:
        outcome_reason = "timeout"
    return ClosedLoopSimulationResult(
        case_id=case.case_id,
        controller_mode=controller_mode,
        start_profile=start_profile,
        fixed_vm_m_s=fixed_vm,
        hit=hit,
        near_hit=near_hit,
        fov_feasible_hit=bool(hit and target_continuously_in_fov),
        target_continuously_in_fov=target_continuously_in_fov,
        minimum_range_m=minimum_range,
        minimum_range_time_s=minimum_range_time,
        final_range_m=final_range,
        elapsed_s=elapsed,
        maximum_speed_m_s=maximum_speed,
        maximum_altitude_loss_m=max(0.0, maximum_altitude_loss),
        maximum_climb_m=max(0.0, maximum_climb),
        maximum_vertical_displacement_m=maximum_vertical_displacement,
        maximum_roll_deg=maximum_roll,
        maximum_pitch_deg=maximum_pitch,
        maximum_commanded_rate_deg_s=maximum_commanded_rate,
        maximum_guidance_accel_m_s2=maximum_guidance_accel,
        maximum_control_accel_m_s2=maximum_control_accel,
        control_update_count=control_update_count,
        entry_handoff_active_fraction=counters["entry_handoff"] / denominator,
        fov_priority_active_fraction=counters["fov_priority"] / denominator,
        guidance_accel_saturation_fraction=counters["guidance"] / denominator,
        centering_accel_saturation_fraction=counters["centering"] / denominator,
        speed_hold_accel_saturation_fraction=counters["speed_hold"] / denominator,
        total_accel_saturation_fraction=counters["total"] / denominator,
        tilt_saturation_fraction=counters["tilt"] / denominator,
        rate_saturation_fraction=counters["rate"] / denominator,
        thrust_saturation_fraction=counters["thrust"] / denominator,
        throttle_handover_active_fraction=(
            counters["throttle_handover"] / denominator
        ),
        throttle_slew_saturation_fraction=counters["throttle_slew"] / denominator,
        minimum_throttle_us=minimum_throttle_us,
        maximum_throttle_us=maximum_throttle_us,
        maximum_load_factor_g=maximum_load_factor_g,
        target_in_fov_fraction=counters["fov"] / denominator,
        maximum_target_off_up_axis_deg=maximum_off_axis_deg,
        initial_target_in_fov=initial_target_in_fov,
        measurement_valid_fraction=counters["measurement_valid"] / denominator,
        measurement_capture_count=measurement_capture_count,
        measurement_delivered_count=measurement_delivered_count,
        measurement_fov_reject_count=measurement_fov_reject_count,
        measurement_dropout_count=measurement_dropout_count,
        measurement_dropout_fraction=measurement_dropout_fraction,
        maximum_measurement_angle_error_deg=maximum_measurement_angle_error_deg,
        maximum_relative_velocity_error_m_s=maximum_relative_velocity_error_m_s,
        maximum_wind_accel_m_s2=maximum_wind_accel_m_s2,
        maximum_measurement_age_s=maximum_measurement_age_s,
        first_measurement_valid_time_s=first_measurement_valid_time_s,
        first_measurement_stale_time_s=first_measurement_stale_time_s,
        kinematic_valid_fraction=counters["kinematic_valid"] / denominator,
        maximum_kinematic_velocity_error_m_s=maximum_kinematic_velocity_error_m_s,
        controller_final_phase=(
            "diagnostic"
            if candidate_output is None
            else candidate_output.phase.value
        ),
        controller_final_reason=(
            "not_applicable" if candidate_output is None else candidate_output.reason
        ),
        controller_abort_time_s=controller_abort_time_s,
        outcome_reason=outcome_reason,
    )


def _validate_case(case: MatrixCase) -> None:
    for name in (
        "horizontal_range_m",
        "altitude_offset_m",
        "target_speed_m_s",
        "speed_ratio",
    ):
        value = float(getattr(case, name))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"case {name} must be finite and positive")
    if abs(case.lateral_offset_m) > case.horizontal_range_m:
        raise ValueError("lateral offset cannot exceed horizontal range")
    if not math.isfinite(case.lateral_offset_m):
        raise ValueError("lateral offset must be finite")
    if not math.isfinite(case.target_course_deg):
        raise ValueError("target_course_deg must be finite")


def _target_in_fov(
    los_ned: np.ndarray,
    R_IB: np.ndarray,
    config: ClosedLoopSimulationConfig,
) -> bool:
    los_body = np.asarray(R_IB, dtype=float).T @ np.asarray(los_ned, dtype=float)
    if (
        config.camera_horizontal_half_fov_deg > 0.0
        and config.camera_vertical_half_fov_deg > 0.0
    ):
        camera_forward = -float(los_body[2])
        if camera_forward <= 0.0:
            return False
        horizontal_deg = math.degrees(
            math.atan2(abs(float(los_body[1])), camera_forward)
        )
        vertical_deg = math.degrees(
            math.atan2(abs(float(los_body[0])), camera_forward)
        )
        return bool(
            horizontal_deg <= config.camera_horizontal_half_fov_deg
            and vertical_deg <= config.camera_vertical_half_fov_deg
        )
    off_axis_deg = math.degrees(
        math.acos(float(np.clip(np.dot(los_body, [0.0, 0.0, -1.0]), -1.0, 1.0)))
    )
    return off_axis_deg <= config.camera_half_fov_deg


def _simulation_rngs(
    random_seed: int,
    *,
    case_id: str,
) -> tuple[
    np.random.Generator,
    np.random.Generator,
    np.random.Generator,
    np.random.Generator,
]:
    seed = np.random.SeedSequence(
        [
            int(random_seed),
            _stable_text_seed(case_id),
        ]
    )
    children = seed.spawn(4)
    return (
        np.random.default_rng(children[0]),
        np.random.default_rng(children[1]),
        np.random.default_rng(children[2]),
        np.random.default_rng(children[3]),
    )


def _stable_text_seed(value: str) -> int:
    result = 2166136261
    for byte in value.encode("utf-8"):
        result ^= byte
        result = (result * 16777619) & 0xFFFFFFFF
    return result


def _perturb_relative_measurement(
    relative_position_m: np.ndarray,
    relative_velocity_m_s: np.ndarray,
    *,
    angle_noise_std_deg: float,
    velocity_noise_std_m_s: float,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    position = np.asarray(relative_position_m, dtype=float)
    velocity = np.asarray(relative_velocity_m_s, dtype=float)
    range_m = float(np.linalg.norm(position))
    truth_los = _normalized(position)
    measured_los = np.array(truth_los, dtype=float)
    if angle_noise_std_deg > 0.0 and range_m > 1.0e-12:
        tangent_noise = rng.normal(
            loc=0.0, scale=math.radians(angle_noise_std_deg), size=3
        )
        tangent_noise -= truth_los * float(np.dot(tangent_noise, truth_los))
        measured_los = _normalized(truth_los + tangent_noise)
    measured_position = measured_los * range_m
    measured_velocity = np.array(velocity, dtype=float)
    if velocity_noise_std_m_s > 0.0:
        measured_velocity += rng.normal(
            loc=0.0, scale=velocity_noise_std_m_s, size=3
        )
    angle_error_deg = math.degrees(
        math.acos(float(np.clip(np.dot(truth_los, measured_los), -1.0, 1.0)))
    )
    velocity_error_m_s = float(np.linalg.norm(measured_velocity - velocity))
    return (
        measured_position,
        measured_velocity,
        angle_error_deg,
        velocity_error_m_s,
    )


def _summarize(
    results: Sequence[ClosedLoopSimulationResult],
    *,
    controller_mode: str,
    start_profile: str,
) -> dict[str, object]:
    if not results:
        raise ValueError("cannot summarize an empty result set")
    return {
        "controller_mode": controller_mode,
        "start_profile": start_profile,
        "case_count": len(results),
        "hit_count": sum(int(result.hit) for result in results),
        "near_hit_count": sum(int(result.near_hit) for result in results),
        "fov_feasible_hit_count": sum(
            int(result.fov_feasible_hit) for result in results
        ),
        "initially_visible_case_count": sum(
            int(result.initial_target_in_fov) for result in results
        ),
        "initially_visible_hit_count": sum(
            int(result.initial_target_in_fov and result.hit) for result in results
        ),
        "minimum_range_mean_m": float(
            np.mean([result.minimum_range_m for result in results])
        ),
        "minimum_range_worst_m": max(result.minimum_range_m for result in results),
        "maximum_altitude_loss_m": max(
            result.maximum_altitude_loss_m for result in results
        ),
        "maximum_climb_m": max(result.maximum_climb_m for result in results),
        "maximum_vertical_displacement_m": max(
            result.maximum_vertical_displacement_m for result in results
        ),
        "mean_target_in_fov_fraction": float(
            np.mean([result.target_in_fov_fraction for result in results])
        ),
        "mean_measurement_valid_fraction": float(
            np.mean([result.measurement_valid_fraction for result in results])
        ),
        "measurement_fov_reject_count": sum(
            result.measurement_fov_reject_count for result in results
        ),
        "measurement_dropout_count": sum(
            result.measurement_dropout_count for result in results
        ),
        "mean_measurement_dropout_fraction": float(
            np.mean([result.measurement_dropout_fraction for result in results])
        ),
        "maximum_measurement_angle_error_deg": max(
            result.maximum_measurement_angle_error_deg for result in results
        ),
        "maximum_relative_velocity_error_m_s": max(
            result.maximum_relative_velocity_error_m_s for result in results
        ),
        "maximum_wind_accel_m_s2": max(
            result.maximum_wind_accel_m_s2 for result in results
        ),
        "outcome_counts": {
            reason: sum(result.outcome_reason == reason for result in results)
            for reason in sorted({result.outcome_reason for result in results})
        },
        "mean_guidance_accel_saturation_fraction": float(
            np.mean(
                [result.guidance_accel_saturation_fraction for result in results]
            )
        ),
        "mean_tilt_saturation_fraction": float(
            np.mean([result.tilt_saturation_fraction for result in results])
        ),
        "mean_rate_saturation_fraction": float(
            np.mean([result.rate_saturation_fraction for result in results])
        ),
        "mean_thrust_saturation_fraction": float(
            np.mean([result.thrust_saturation_fraction for result in results])
        ),
    }


def _velocity_reference(
    los: np.ndarray, fixed_vm_m_s: float, config: ClosedLoopSimulationConfig
) -> np.ndarray:
    reference = float(fixed_vm_m_s) * np.asarray(los, dtype=float)
    reference[2] = float(
        np.clip(
            reference[2],
            -config.vertical_speed_reference_limit_m_s,
            config.vertical_speed_reference_limit_m_s,
        )
    )
    return reference


def _clip_norm_with_flag(vector: np.ndarray, limit: float) -> tuple[np.ndarray, bool]:
    value = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(value))
    if norm <= float(limit) or norm <= 1.0e-12:
        return np.array(value, dtype=float), False
    return value * (float(limit) / norm), True


def _normalized(vector: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(value))
    if norm <= 1.0e-12:
        return np.zeros(3, dtype=float)
    return value / norm


def _rotation_matrix_frd(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _delayed_body_rate_command(
    history: deque[tuple[float, float, float]],
    query_time_s: float,
) -> tuple[float, float]:
    if not history or query_time_s < history[0][0]:
        return 0.0, 0.0
    while len(history) >= 2 and history[1][0] <= query_time_s:
        history.popleft()
    left = history[0]
    if len(history) == 1 or query_time_s <= left[0]:
        return left[1], left[2]
    right = history[1]
    alpha = (query_time_s - left[0]) / max(1.0e-12, right[0] - left[0])
    return (
        left[1] + alpha * (right[1] - left[1]),
        left[2] + alpha * (right[2] - left[2]),
    )


def _delayed_held_body_rate_command(
    history: deque[tuple[float, float, float]],
    query_time_s: float,
) -> tuple[float, float]:
    if not history or query_time_s < history[0][0]:
        return 0.0, 0.0
    while len(history) >= 2 and history[1][0] <= query_time_s:
        history.popleft()
    return history[0][1], history[0][2]


def _smoothstep01(value: float) -> float:
    bounded = float(np.clip(value, 0.0, 1.0))
    return bounded * bounded * (3.0 - 2.0 * bounded)


def _lerp(start: float, end: float, progress: float) -> float:
    return float(start + (end - start) * progress)


def _thrust_to_pwm(thrust: float, config: ClosedLoopSimulationConfig) -> float:
    value = float(np.clip(thrust, 0.0, 1.0))
    if value <= 0.5:
        return _lerp(
            config.throttle_min_us,
            config.throttle_hover_us,
            value / 0.5,
        )
    return _lerp(
        config.throttle_hover_us,
        config.throttle_max_us,
        (value - 0.5) / 0.5,
    )


def _configured_thrust_model(
    config: ClosedLoopSimulationConfig,
) -> VoltageThrottleThrustModel | None:
    if not config.throttle_dynamics_enabled:
        return None
    model = _load_thrust_model(
        str(Path(config.thrust_model_path).expanduser().resolve()),
        config.thrust_model_sha256,
        config.thrust_model_calibration_id,
    )
    if not model.covers_voltage(config.battery_voltage_v):
        raise ValueError("battery voltage is outside thrust LUT coverage")
    if (
        float(model.throttle_us[0]) > config.throttle_min_us
        or float(model.throttle_us[-1]) < config.throttle_max_us
    ):
        raise ValueError("thrust LUT does not cover the configured throttle envelope")
    return model


@lru_cache(maxsize=8)
def _load_thrust_model(
    path: str,
    sha256: str,
    calibration_id: str,
) -> VoltageThrottleThrustModel:
    return VoltageThrottleThrustModel.from_file(
        path,
        expected_sha256=sha256,
        expected_calibration_id=calibration_id,
    )


def _integrate_euler_frd(
    roll: float,
    pitch: float,
    yaw: float,
    p: float,
    q: float,
    r: float,
    dt: float,
) -> tuple[float, float, float]:
    cos_pitch = max(0.05, abs(math.cos(pitch)))
    if math.cos(pitch) < 0.0:
        cos_pitch = -cos_pitch
    tan_pitch = math.sin(pitch) / cos_pitch
    roll_dot = p + tan_pitch * (q * math.sin(roll) + r * math.cos(roll))
    pitch_dot = q * math.cos(roll) - r * math.sin(roll)
    yaw_dot = (q * math.sin(roll) + r * math.cos(roll)) / cos_pitch
    return (
        roll + roll_dot * dt,
        pitch + pitch_dot * dt,
        yaw + yaw_dot * dt,
    )
