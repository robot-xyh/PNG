from __future__ import annotations

import math
from collections import deque
from dataclasses import asdict, dataclass, field
from enum import Enum

import numpy as np


class InterceptPhase(str, Enum):
    ACQUIRE = "ACQUIRE"
    TRACKING = "TRACKING"
    TERMINAL_VISUAL = "TERMINAL_VISUAL"
    BLIND_HOLD = "BLIND_HOLD"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"


class EngagementPolicy(str, Enum):
    NONCOLLISION = "noncollision"
    CONTACT = "contact"


@dataclass(frozen=True)
class FovPriorityConfig:
    enabled: bool = False
    start_ratio: float = 0.70
    full_ratio: float = 0.90
    horizontal_half_fov_deg: float = 0.0
    vertical_half_fov_deg: float = 0.0

    def __post_init__(self) -> None:
        for name in ("start_ratio", "full_ratio"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"fov priority {name} must be in [0, 1]")
        if self.full_ratio <= self.start_ratio:
            raise ValueError("fov priority full_ratio must exceed start_ratio")
        half_fov = (
            float(self.horizontal_half_fov_deg),
            float(self.vertical_half_fov_deg),
        )
        if any(
            not math.isfinite(value) or not 0.0 <= value < 90.0
            for value in half_fov
        ):
            raise ValueError("fov priority half-FOV values must be in [0, 90)")
        if self.enabled and any(value <= 0.0 for value in half_fov):
            raise ValueError(
                "enabled fov priority requires positive rectangular half-FOV values"
            )


@dataclass(frozen=True)
class VelocityEstablishingPngConfig:
    fixed_vm_m_s: float
    navigation_constant: float = 3.0
    speed_gain_s_inv: float = 1.2
    speed_accel_limit_m_s2: float = 8.0
    png_accel_limit_m_s2: float = 20.0
    fov_centering_gain_s2: float = 8.0
    fov_centering_accel_limit_m_s2: float = 4.0
    total_accel_limit_m_s2: float = 28.0
    vertical_speed_reference_limit_m_s: float = 6.0
    velocity_reference_slew_m_s2: float = 3.0
    png_track_speed_ratio: float = 0.8
    acquire_consecutive_frames: int = 5
    detection_timeout_s: float = 0.15
    detection_result_age_limit_s: float = 0.20
    velocity_timeout_s: float = 0.5
    los_prediction_max_s: float = 0.0
    gravity_m_s2: float = 9.80665
    fov_constraint_half_angle_deg: float = 0.0
    fov_priority: FovPriorityConfig = field(default_factory=FovPriorityConfig)
    engagement_policy: str = EngagementPolicy.NONCOLLISION.value
    noncollision_bbox_abort_ratio: float = 0.012
    noncollision_ttc_abort_s: float = 2.0
    contact_bbox_terminal_ratio: float = 0.05
    contact_ttc_terminal_s: float = 1.0
    contact_bbox_complete_ratio: float = 0.25
    blind_hold_s: float = 0.20
    terminal_reacquire_frames: int = 2
    area_ttc_window_s: float = 0.60
    area_ttc_min_samples: int = 5
    area_ttc_min_span_s: float = 0.10

    def __post_init__(self) -> None:
        positive = (
            self.fixed_vm_m_s,
            self.navigation_constant,
            self.speed_gain_s_inv,
            self.speed_accel_limit_m_s2,
            self.png_accel_limit_m_s2,
            self.fov_centering_accel_limit_m_s2,
            self.total_accel_limit_m_s2,
            self.vertical_speed_reference_limit_m_s,
            self.velocity_reference_slew_m_s2,
            self.detection_timeout_s,
            self.detection_result_age_limit_s,
            self.velocity_timeout_s,
            self.noncollision_bbox_abort_ratio,
            self.noncollision_ttc_abort_s,
            self.contact_bbox_terminal_ratio,
            self.contact_ttc_terminal_s,
            self.contact_bbox_complete_ratio,
            self.blind_hold_s,
            self.area_ttc_window_s,
            self.area_ttc_min_span_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("velocity-establishing PNG limits must be finite and positive")
        if not math.isfinite(self.fov_centering_gain_s2) or self.fov_centering_gain_s2 < 0.0:
            raise ValueError("fov_centering_gain_s2 must be finite and non-negative")
        if self.acquire_consecutive_frames < 1:
            raise ValueError("acquire_consecutive_frames must be positive")
        if not 0.0 < self.png_track_speed_ratio <= 1.0:
            raise ValueError("png_track_speed_ratio must be in (0, 1]")
        if self.terminal_reacquire_frames < 1:
            raise ValueError("terminal_reacquire_frames must be positive")
        if self.area_ttc_min_samples < 5:
            raise ValueError("area_ttc_min_samples must be at least five")
        if self.contact_bbox_complete_ratio <= self.contact_bbox_terminal_ratio:
            raise ValueError("contact complete ratio must exceed terminal ratio")
        try:
            EngagementPolicy(self.engagement_policy)
        except ValueError as exc:
            raise ValueError(
                "engagement_policy must be 'noncollision' or 'contact'"
            ) from exc
        if not math.isfinite(self.los_prediction_max_s) or self.los_prediction_max_s < 0.0:
            raise ValueError("los_prediction_max_s must be finite and non-negative")
        if not math.isfinite(self.gravity_m_s2) or self.gravity_m_s2 <= 0.0:
            raise ValueError("gravity_m_s2 must be finite and positive")
        if (
            not math.isfinite(self.fov_constraint_half_angle_deg)
            or not 0.0 <= self.fov_constraint_half_angle_deg < 90.0
        ):
            raise ValueError("fov_constraint_half_angle_deg must be in [0, 90)")


@dataclass(frozen=True)
class VelocityEstablishingPngInput:
    timestamp_s: float
    los_timestamp_s: float | None
    lambda_ned: np.ndarray | None
    lambda_dot_ned_s: np.ndarray | None
    tracking_valid: bool
    bbox_area_ratio: float | None
    attitude_R_IB: np.ndarray | None
    attitude_valid: bool
    velocity_timestamp_s: float | None
    velocity_ned_m_s: np.ndarray | None
    velocity_valid: bool
    tracking_reason: str | None = None
    ttc_valid: bool = False
    ttc_s: float | None = None
    track_id: int | None = None
    los_update_timestamp_s: float | None = None


@dataclass(frozen=True)
class VelocityEstablishingPngOutput:
    timestamp_s: float
    phase: InterceptPhase
    valid: bool
    reason: str
    acceleration_ned_m_s2: tuple[float, float, float]
    velocity_reference_raw_ned_m_s: tuple[float, float, float]
    speed_acceleration_ned_m_s2: tuple[float, float, float]
    speed_acceleration_raw_ned_m_s2: tuple[float, float, float]
    png_acceleration_ned_m_s2: tuple[float, float, float]
    png_acceleration_raw_ned_m_s2: tuple[float, float, float]
    fov_acceleration_ned_m_s2: tuple[float, float, float]
    fov_acceleration_raw_ned_m_s2: tuple[float, float, float]
    protected_acceleration_raw_ned_m_s2: tuple[float, float, float]
    protected_acceleration_ned_m_s2: tuple[float, float, float]
    velocity_reference_ned_m_s: tuple[float, float, float]
    png_speed_m_s: float
    los_speed_m_s: float | None
    detection_age_s: float | None
    detection_update_age_s: float | None
    velocity_age_s: float | None
    los_prediction_horizon_s: float
    acquire_count: int
    speed_saturated: bool
    png_saturated: bool
    fov_saturated: bool
    fov_constraint_active: bool
    fov_priority_active: bool
    fov_priority_weight: float
    protected_scale: float
    speed_budget_scale: float
    total_saturated: bool
    terminal_trigger: str | None
    area_ttc_s: float | None
    track_id: int | None
    terminal_track_id: int | None
    terminal_reacquire_count: int
    blind_age_s: float | None
    blind_scale: float

    def to_dict(self) -> dict[str, object]:
        values = asdict(self)
        values["phase"] = self.phase.value
        return values


class VelocityEstablishingPngController:
    """Offline controller that emits physical acceleration diagnostics, never RC/PWM."""

    def __init__(self, config: VelocityEstablishingPngConfig):
        self.config = config
        self.phase = InterceptPhase.ACQUIRE
        self._acquire_count = 0
        self._acquire_track_id: int | None = None
        self._last_los_timestamp_s: float | None = None
        self._abort_reason = ""
        self._terminal_trigger: str | None = None
        self._active_track_id: int | None = None
        self._terminal_track_id: int | None = None
        self._terminal_reacquire_count = 0
        self._last_reacquire_timestamp_s: float | None = None
        self._velocity_reference: np.ndarray | None = None
        self._last_control_timestamp_s: float | None = None
        self._area_samples: deque[tuple[float, float]] = deque()
        self._area_track_id: int | None = None
        self._last_reliable_acceleration = np.zeros(3, dtype=float)
        self._blind_started_s: float | None = None

    def reset(self) -> None:
        self.phase = InterceptPhase.ACQUIRE
        self._acquire_count = 0
        self._acquire_track_id = None
        self._last_los_timestamp_s = None
        self._abort_reason = ""
        self._terminal_trigger = None
        self._active_track_id = None
        self._terminal_track_id = None
        self._terminal_reacquire_count = 0
        self._last_reacquire_timestamp_s = None
        self._velocity_reference = None
        self._last_control_timestamp_s = None
        self._area_samples.clear()
        self._area_track_id = None
        self._last_reliable_acceleration = np.zeros(3, dtype=float)
        self._blind_started_s = None

    def update(self, value: VelocityEstablishingPngInput) -> VelocityEstablishingPngOutput:
        now = float(value.timestamp_s)
        if not math.isfinite(now):
            return self._empty(0.0, "nonfinite_input", None, None, None)
        detection_age = _age(now, value.los_timestamp_s)
        detection_update_age = _age(
            now,
            value.los_timestamp_s
            if value.los_update_timestamp_s is None
            else value.los_update_timestamp_s,
        )
        velocity_age = _age(now, value.velocity_timestamp_s)
        invalid_reason = self._invalid_reason(
            value,
            detection_age,
            detection_update_age,
            velocity_age,
        )
        if self.phase in (InterceptPhase.ABORT, InterceptPhase.COMPLETE):
            return self._empty(
                now,
                self._abort_reason or "abort_latched",
                detection_age,
                detection_update_age,
                velocity_age,
            )
        if (
            invalid_reason is None
            and self._active_track_id is not None
            and value.track_id is not None
            and value.track_id != self._active_track_id
        ):
            invalid_reason = "track_id_changed"
        if invalid_reason is not None:
            return self._handle_invalid(
                now,
                invalid_reason,
                detection_age,
                detection_update_age,
                velocity_age,
            )

        if self.phase == InterceptPhase.BLIND_HOLD:
            if value.track_id != self._terminal_track_id:
                return self._blind_hold(
                    now,
                    "terminal_track_mismatch",
                    detection_age,
                    detection_update_age,
                    velocity_age,
                )
            if value.los_timestamp_s != self._last_reacquire_timestamp_s:
                self._last_reacquire_timestamp_s = value.los_timestamp_s
                self._terminal_reacquire_count += 1
            if self._terminal_reacquire_count < self.config.terminal_reacquire_frames:
                return self._blind_hold(
                    now,
                    "terminal_reacquiring",
                    detection_age,
                    detection_update_age,
                    velocity_age,
                )
            self.phase = InterceptPhase.TERMINAL_VISUAL
            self._blind_started_s = None

        los = _unit(value.lambda_ned)
        los_dot = np.asarray(value.lambda_dot_ned_s, dtype=float)
        velocity = np.asarray(value.velocity_ned_m_s, dtype=float)
        R_IB = np.asarray(value.attitude_R_IB, dtype=float)
        prediction_horizon = min(
            detection_age or 0.0,
            self.config.los_prediction_max_s,
        )
        control_los = _unit(los + prediction_horizon * los_dot)
        self._record_area_sample(value)
        area_ttc_s = self._area_ttc_s()
        terminal_trigger = self._terminal_trigger_for(value, area_ttc_s)
        if terminal_trigger is not None:
            self._terminal_trigger = terminal_trigger
            if self.config.engagement_policy == EngagementPolicy.NONCOLLISION.value:
                return self._latch_terminal(
                    InterceptPhase.ABORT,
                    terminal_trigger,
                    now,
                    detection_age,
                    detection_update_age,
                    velocity_age,
                    area_ttc_s,
                    value.track_id,
                )
            if terminal_trigger == "contact_bbox_complete":
                return self._latch_terminal(
                    InterceptPhase.COMPLETE,
                    terminal_trigger,
                    now,
                    detection_age,
                    detection_update_age,
                    velocity_age,
                    area_ttc_s,
                    value.track_id,
                )

        if self.phase == InterceptPhase.ACQUIRE:
            if value.track_id is None:
                self._acquire_count = 0
                self._acquire_track_id = None
                self._last_los_timestamp_s = None
                return self._empty(
                    now,
                    "track_id_missing",
                    detection_age,
                    detection_update_age,
                    velocity_age,
                )
            if value.track_id != self._acquire_track_id:
                self._acquire_track_id = value.track_id
                self._acquire_count = 0
                self._last_los_timestamp_s = None
            if value.los_timestamp_s != self._last_los_timestamp_s:
                self._last_los_timestamp_s = value.los_timestamp_s
                self._acquire_count += 1
            if self._acquire_count < self.config.acquire_consecutive_frames:
                return self._empty(
                    now,
                    "acquiring",
                    detection_age,
                    detection_update_age,
                    velocity_age,
                )
            self.phase = InterceptPhase.TRACKING
            self._active_track_id = value.track_id
            self._velocity_reference = np.array(velocity, dtype=float, copy=True)
            self._last_control_timestamp_s = now
        if (
            self.config.engagement_policy == EngagementPolicy.CONTACT.value
            and terminal_trigger is not None
        ):
            self.phase = InterceptPhase.TERMINAL_VISUAL
            self._terminal_track_id = value.track_id

        velocity_reference_raw = self.config.fixed_vm_m_s * control_los
        velocity_reference_raw[2] = float(
            np.clip(
                velocity_reference_raw[2],
                -self.config.vertical_speed_reference_limit_m_s,
                self.config.vertical_speed_reference_limit_m_s,
            )
        )
        velocity_reference = self._ramp_velocity_reference(
            velocity_reference_raw,
            velocity,
            now,
        )
        speed_accel_raw = self.config.speed_gain_s_inv * (velocity_reference - velocity)
        if self.phase == InterceptPhase.TERMINAL_VISUAL:
            speed_accel_raw = np.zeros(3, dtype=float)
        speed_accel, speed_saturated = _clip_norm(
            speed_accel_raw,
            self.config.speed_accel_limit_m_s2,
        )
        png_speed_m_s = min(
            self.config.fixed_vm_m_s,
            max(float(np.linalg.norm(velocity)), float(np.linalg.norm(velocity_reference))),
        )
        png_accel_raw = self.config.navigation_constant * png_speed_m_s * los_dot
        png_accel, png_saturated = _clip_norm(
            png_accel_raw,
            self.config.png_accel_limit_m_s2,
        )
        los_body = R_IB.T @ control_los
        fov_body_raw = np.array(
            [
                self.config.fov_centering_gain_s2 * los_body[0],
                self.config.fov_centering_gain_s2 * los_body[1],
                0.0,
            ],
            dtype=float,
        )
        fov_body, fov_saturated = _clip_norm(
            fov_body_raw, self.config.fov_centering_accel_limit_m_s2
        )
        fov_accel_raw = R_IB @ fov_body_raw
        fov_accel = R_IB @ fov_body
        protected_raw = png_accel + fov_accel
        protected_accel, protected_saturated = _clip_norm(
            protected_raw,
            self.config.total_accel_limit_m_s2,
        )
        protected_norm = float(np.linalg.norm(protected_raw))
        protected_scale = (
            1.0
            if protected_norm <= 1.0e-12
            else min(1.0, self.config.total_accel_limit_m_s2 / protected_norm)
        )
        priority_weight = _fov_priority_weight(
            control_los,
            R_IB,
            self.config.fov_priority,
        )
        fov_priority_active = priority_weight > 0.0
        if fov_priority_active:
            speed_accel = _suppress_fov_opposition(
                speed_accel,
                fov_accel,
                priority_weight,
            )
        speed_budget_scale = _maximum_vector_budget_scale(
            protected_accel,
            speed_accel,
            self.config.total_accel_limit_m_s2,
        )
        total = protected_accel + speed_budget_scale * speed_accel
        total_saturated = protected_saturated or speed_budget_scale < 1.0 - 1.0e-12
        total, fov_constraint_active = _constrain_acceleration_to_los(
            total,
            control_los,
            gravity_m_s2=self.config.gravity_m_s2,
            half_angle_deg=self.config.fov_constraint_half_angle_deg,
            acceleration_limit_m_s2=self.config.total_accel_limit_m_s2,
        )
        total_saturated = total_saturated or fov_constraint_active
        los_speed = float(np.dot(velocity, los))
        if self.phase == InterceptPhase.TERMINAL_VISUAL:
            self._last_reliable_acceleration = np.array(total, dtype=float, copy=True)
        return VelocityEstablishingPngOutput(
            timestamp_s=now,
            phase=self.phase,
            valid=True,
            reason="active",
            acceleration_ned_m_s2=_tuple3(total),
            velocity_reference_raw_ned_m_s=_tuple3(velocity_reference_raw),
            speed_acceleration_ned_m_s2=_tuple3(speed_accel),
            speed_acceleration_raw_ned_m_s2=_tuple3(speed_accel_raw),
            png_acceleration_ned_m_s2=_tuple3(png_accel),
            png_acceleration_raw_ned_m_s2=_tuple3(png_accel_raw),
            fov_acceleration_ned_m_s2=_tuple3(fov_accel),
            fov_acceleration_raw_ned_m_s2=_tuple3(fov_accel_raw),
            protected_acceleration_raw_ned_m_s2=_tuple3(protected_raw),
            protected_acceleration_ned_m_s2=_tuple3(protected_accel),
            velocity_reference_ned_m_s=_tuple3(velocity_reference),
            png_speed_m_s=png_speed_m_s,
            los_speed_m_s=los_speed,
            detection_age_s=detection_age,
            detection_update_age_s=detection_update_age,
            velocity_age_s=velocity_age,
            los_prediction_horizon_s=prediction_horizon,
            acquire_count=self._acquire_count,
            speed_saturated=speed_saturated,
            png_saturated=png_saturated,
            fov_saturated=fov_saturated,
            fov_constraint_active=fov_constraint_active,
            fov_priority_active=fov_priority_active,
            fov_priority_weight=priority_weight,
            protected_scale=protected_scale,
            speed_budget_scale=speed_budget_scale,
            total_saturated=total_saturated,
            terminal_trigger=self._terminal_trigger,
            area_ttc_s=area_ttc_s,
            track_id=value.track_id,
            terminal_track_id=self._terminal_track_id,
            terminal_reacquire_count=self._terminal_reacquire_count,
            blind_age_s=None,
            blind_scale=0.0,
        )

    def _ramp_velocity_reference(
        self,
        target: np.ndarray,
        velocity: np.ndarray,
        now: float,
    ) -> np.ndarray:
        if self._velocity_reference is None:
            self._velocity_reference = np.array(velocity, dtype=float, copy=True)
        previous_s = self._last_control_timestamp_s
        dt = 0.0 if previous_s is None else max(0.0, now - previous_s)
        self._last_control_timestamp_s = now
        self._velocity_reference = _move_towards(
            self._velocity_reference,
            target,
            self.config.velocity_reference_slew_m_s2 * dt,
        )
        return np.array(self._velocity_reference, dtype=float, copy=True)

    def _handle_invalid(
        self,
        now: float,
        reason: str,
        detection_age_s: float | None,
        detection_update_age_s: float | None,
        velocity_age_s: float | None,
    ) -> VelocityEstablishingPngOutput:
        tracking_loss = reason in {
            "tracking_invalid",
            "detection_stale",
            "detection_result_stale",
            "track_id_changed",
            "no_detection",
        } or reason.startswith("track_")
        if (
            self.config.engagement_policy == EngagementPolicy.CONTACT.value
            and self.phase in (InterceptPhase.TERMINAL_VISUAL, InterceptPhase.BLIND_HOLD)
            and tracking_loss
        ):
            return self._blind_hold(
                now,
                reason,
                detection_age_s,
                detection_update_age_s,
                velocity_age_s,
            )
        if self.phase == InterceptPhase.ACQUIRE:
            self._acquire_count = 0
            self._last_los_timestamp_s = None
            return self._empty(
                now,
                reason,
                detection_age_s,
                detection_update_age_s,
                velocity_age_s,
            )
        self.phase = InterceptPhase.ABORT
        self._abort_reason = reason
        return self._empty(
            now,
            reason,
            detection_age_s,
            detection_update_age_s,
            velocity_age_s,
        )

    def _blind_hold(
        self,
        now: float,
        reason: str,
        detection_age_s: float | None,
        detection_update_age_s: float | None,
        velocity_age_s: float | None,
    ) -> VelocityEstablishingPngOutput:
        if self._blind_started_s is None:
            self._blind_started_s = now
            self._terminal_reacquire_count = 0
            self._last_reacquire_timestamp_s = None
        age_s = max(0.0, now - self._blind_started_s)
        if age_s + 1.0e-12 >= self.config.blind_hold_s:
            self.phase = InterceptPhase.COMPLETE
            return self._empty(
                now,
                "blind_hold_complete",
                detection_age_s,
                detection_update_age_s,
                velocity_age_s,
            )
        self.phase = InterceptPhase.BLIND_HOLD
        scale = max(0.0, 1.0 - age_s / self.config.blind_hold_s)
        output = self._empty(
            now,
            reason,
            detection_age_s,
            detection_update_age_s,
            velocity_age_s,
        )
        return VelocityEstablishingPngOutput(
            **{
                **output.__dict__,
                "valid": True,
                "reason": "blind_hold" if reason != "terminal_reacquiring" else reason,
                "acceleration_ned_m_s2": _tuple3(
                    scale * self._last_reliable_acceleration
                ),
                "velocity_reference_ned_m_s": _tuple3(
                    np.zeros(3) if self._velocity_reference is None else self._velocity_reference
                ),
                "terminal_trigger": self._terminal_trigger,
                "terminal_track_id": self._terminal_track_id,
                "terminal_reacquire_count": self._terminal_reacquire_count,
                "blind_age_s": age_s,
                "blind_scale": scale,
            }
        )

    def _record_area_sample(self, value: VelocityEstablishingPngInput) -> None:
        if value.track_id is None or value.bbox_area_ratio is None or value.los_timestamp_s is None:
            return
        if self._area_track_id != value.track_id:
            self._area_samples.clear()
            self._area_track_id = value.track_id
        timestamp_s = float(value.los_timestamp_s)
        if self._area_samples and timestamp_s == self._area_samples[-1][0]:
            return
        self._area_samples.append((timestamp_s, float(value.bbox_area_ratio)))
        cutoff = timestamp_s - self.config.area_ttc_window_s
        while self._area_samples and self._area_samples[0][0] < cutoff:
            self._area_samples.popleft()

    def _area_ttc_s(self) -> float | None:
        samples = list(self._area_samples)
        if len(samples) < self.config.area_ttc_min_samples:
            return None
        if samples[-1][0] - samples[0][0] < self.config.area_ttc_min_span_s:
            return None
        slopes = [
            (area_b - area_a) / (time_b - time_a)
            for index, (time_a, area_a) in enumerate(samples[:-1])
            for time_b, area_b in samples[index + 1 :]
            if time_b - time_a > 1.0e-9
        ]
        if not slopes:
            return None
        slope = float(np.median(slopes))
        if not math.isfinite(slope) or slope <= 1.0e-9:
            return None
        intercept = float(np.median([area - slope * timestamp for timestamp, area in samples]))
        fitted_area = intercept + slope * samples[-1][0]
        ttc_s = 2.0 * fitted_area / slope
        if not math.isfinite(ttc_s) or ttc_s <= 0.0:
            return None
        return ttc_s

    def _terminal_trigger_for(
        self,
        value: VelocityEstablishingPngInput,
        area_ttc_s: float | None,
    ) -> str | None:
        bbox_ratio = value.bbox_area_ratio
        valid_ttc = value.ttc_s if value.ttc_valid else None
        if self.config.engagement_policy == EngagementPolicy.NONCOLLISION.value:
            if bbox_ratio is not None and bbox_ratio >= self.config.noncollision_bbox_abort_ratio:
                return "noncollision_bbox_abort"
            if valid_ttc is not None and valid_ttc <= self.config.noncollision_ttc_abort_s:
                return "noncollision_ttc_abort"
            if area_ttc_s is not None and area_ttc_s <= self.config.noncollision_ttc_abort_s:
                return "noncollision_area_ttc_abort"
            return None
        if bbox_ratio is not None and bbox_ratio >= self.config.contact_bbox_complete_ratio:
            return "contact_bbox_complete"
        if bbox_ratio is not None and bbox_ratio >= self.config.contact_bbox_terminal_ratio:
            return "contact_bbox_terminal"
        if valid_ttc is not None and valid_ttc <= self.config.contact_ttc_terminal_s:
            return "contact_ttc_terminal"
        if area_ttc_s is not None and area_ttc_s <= self.config.contact_ttc_terminal_s:
            return "contact_area_ttc_terminal"
        return None

    def _latch_terminal(
        self,
        phase: InterceptPhase,
        reason: str,
        timestamp_s: float,
        detection_age_s: float | None,
        detection_update_age_s: float | None,
        velocity_age_s: float | None,
        area_ttc_s: float | None,
        track_id: int | None,
    ) -> VelocityEstablishingPngOutput:
        self.phase = phase
        self._abort_reason = reason
        self._terminal_track_id = track_id
        output = self._empty(
            timestamp_s,
            reason,
            detection_age_s,
            detection_update_age_s,
            velocity_age_s,
        )
        return VelocityEstablishingPngOutput(
            **{
                **output.__dict__,
                "terminal_trigger": reason,
                "area_ttc_s": area_ttc_s,
                "track_id": track_id,
                "terminal_track_id": track_id,
            }
        )

    def _invalid_reason(
        self,
        value: VelocityEstablishingPngInput,
        detection_age: float | None,
        detection_update_age: float | None,
        velocity_age: float | None,
    ) -> str | None:
        for timestamp in (
            value.los_timestamp_s,
            value.los_update_timestamp_s,
            value.velocity_timestamp_s,
        ):
            if timestamp is not None and not math.isfinite(float(timestamp)):
                return "nonfinite_input"
        if (
            value.los_timestamp_s is not None
            and value.los_update_timestamp_s is not None
            and float(value.los_update_timestamp_s) < float(value.los_timestamp_s)
        ):
            return "detection_time_order_invalid"
        if not value.tracking_valid or value.lambda_ned is None or value.lambda_dot_ned_s is None:
            return value.tracking_reason or "tracking_invalid"
        if (
            detection_update_age is None
            or detection_update_age > self.config.detection_timeout_s
        ):
            return "detection_stale"
        result_age_at_delivery = max(
            0.0,
            (detection_age or 0.0) - detection_update_age,
        )
        if result_age_at_delivery > self.config.detection_result_age_limit_s:
            return "detection_result_stale"
        if not value.velocity_valid or value.velocity_ned_m_s is None:
            return "velocity_invalid"
        if velocity_age is None or velocity_age > self.config.velocity_timeout_s:
            return "velocity_stale"
        if not value.attitude_valid or value.attitude_R_IB is None:
            return "attitude_invalid"
        arrays = (value.lambda_ned, value.lambda_dot_ned_s, value.velocity_ned_m_s)
        if any(np.asarray(item).shape != (3,) for item in arrays):
            return "invalid_vector_shape"
        rotation = np.asarray(value.attitude_R_IB)
        if rotation.shape != (3, 3):
            return "invalid_attitude_shape"
        if not all(np.all(np.isfinite(item)) for item in (*arrays, rotation)):
            return "nonfinite_input"
        if float(np.linalg.norm(value.lambda_ned)) <= 1.0e-12:
            return "los_zero"
        if value.bbox_area_ratio is not None and (
            not math.isfinite(value.bbox_area_ratio) or value.bbox_area_ratio < 0.0
        ):
            return "bbox_area_invalid"
        if value.ttc_valid and (
            value.ttc_s is None
            or not math.isfinite(float(value.ttc_s))
            or float(value.ttc_s) <= 0.0
        ):
            return "ttc_invalid"
        return None

    def _empty(
        self,
        timestamp_s: float,
        reason: str,
        detection_age_s: float | None,
        detection_update_age_s: float | None,
        velocity_age_s: float | None,
    ) -> VelocityEstablishingPngOutput:
        zero = (0.0, 0.0, 0.0)
        return VelocityEstablishingPngOutput(
            timestamp_s=timestamp_s,
            phase=self.phase,
            valid=False,
            reason=reason,
            acceleration_ned_m_s2=zero,
            velocity_reference_raw_ned_m_s=zero,
            speed_acceleration_ned_m_s2=zero,
            speed_acceleration_raw_ned_m_s2=zero,
            png_acceleration_ned_m_s2=zero,
            png_acceleration_raw_ned_m_s2=zero,
            fov_acceleration_ned_m_s2=zero,
            fov_acceleration_raw_ned_m_s2=zero,
            protected_acceleration_raw_ned_m_s2=zero,
            protected_acceleration_ned_m_s2=zero,
            velocity_reference_ned_m_s=zero,
            png_speed_m_s=0.0,
            los_speed_m_s=None,
            detection_age_s=detection_age_s,
            detection_update_age_s=detection_update_age_s,
            velocity_age_s=velocity_age_s,
            los_prediction_horizon_s=0.0,
            acquire_count=self._acquire_count,
            speed_saturated=False,
            png_saturated=False,
            fov_saturated=False,
            fov_constraint_active=False,
            fov_priority_active=False,
            fov_priority_weight=0.0,
            protected_scale=1.0,
            speed_budget_scale=0.0,
            total_saturated=False,
            terminal_trigger=self._terminal_trigger,
            area_ttc_s=self._area_ttc_s(),
            track_id=self._active_track_id,
            terminal_track_id=self._terminal_track_id,
            terminal_reacquire_count=self._terminal_reacquire_count,
            blind_age_s=None,
            blind_scale=0.0,
        )


def _fov_priority_weight(
    los_ned: np.ndarray,
    R_IB: np.ndarray,
    config: FovPriorityConfig,
) -> float:
    if not config.enabled:
        return 0.0
    los_body = np.asarray(R_IB, dtype=float).T @ np.asarray(los_ned, dtype=float)
    camera_forward = -float(los_body[2])
    if camera_forward <= 0.0:
        return 1.0
    horizontal_deg = math.degrees(
        math.atan2(abs(float(los_body[1])), camera_forward)
    )
    vertical_deg = math.degrees(
        math.atan2(abs(float(los_body[0])), camera_forward)
    )
    ratio = max(
        horizontal_deg / config.horizontal_half_fov_deg,
        vertical_deg / config.vertical_half_fov_deg,
    )
    linear = float(
        np.clip(
            (ratio - config.start_ratio)
            / (config.full_ratio - config.start_ratio),
            0.0,
            1.0,
        )
    )
    return linear * linear * (3.0 - 2.0 * linear)


def _suppress_fov_opposition(
    non_fov_acceleration: np.ndarray,
    fov_acceleration: np.ndarray,
    weight: float,
) -> np.ndarray:
    non_fov = np.asarray(non_fov_acceleration, dtype=float)
    fov = np.asarray(fov_acceleration, dtype=float)
    fov_norm = float(np.linalg.norm(fov))
    if fov_norm <= 1.0e-12 or weight <= 0.0:
        return np.array(non_fov, dtype=float)
    fov_direction = fov / fov_norm
    opposing_scalar = min(0.0, float(np.dot(non_fov, fov_direction)))
    return non_fov - float(weight) * opposing_scalar * fov_direction


def _constrain_acceleration_to_los(
    acceleration_ned_m_s2: np.ndarray,
    los_ned: np.ndarray,
    *,
    gravity_m_s2: float,
    half_angle_deg: float,
    acceleration_limit_m_s2: float,
) -> tuple[np.ndarray, bool]:
    acceleration = np.asarray(acceleration_ned_m_s2, dtype=float)
    if half_angle_deg <= 0.0:
        return np.array(acceleration, dtype=float), False
    gravity = np.array([0.0, 0.0, gravity_m_s2], dtype=float)
    thrust_vector = acceleration - gravity
    thrust_norm = float(np.linalg.norm(thrust_vector))
    if thrust_norm <= 1.0e-12:
        return np.array(acceleration, dtype=float), False
    desired_up = thrust_vector / thrust_norm
    los = _unit(los_ned)
    cosine = float(np.clip(np.dot(los, desired_up), -1.0, 1.0))
    angle = math.acos(cosine)
    limit = math.radians(half_angle_deg)
    if angle <= limit + 1.0e-12:
        return np.array(acceleration, dtype=float), False

    tangent = desired_up - cosine * los
    tangent_norm = float(np.linalg.norm(tangent))
    if tangent_norm <= 1.0e-12:
        tangent = _orthogonal_unit(los)
    else:
        tangent /= tangent_norm
    constrained_up = math.cos(limit) * los + math.sin(limit) * tangent

    preferred_thrust = max(0.0, float(np.dot(thrust_vector, constrained_up)))
    gravity_projection = float(np.dot(gravity, constrained_up))
    discriminant = gravity_projection**2 - (
        float(np.dot(gravity, gravity)) - acceleration_limit_m_s2**2
    )
    if discriminant < 0.0:
        return np.array(acceleration, dtype=float), False
    root = math.sqrt(max(0.0, discriminant))
    thrust_min = max(0.0, -gravity_projection - root)
    thrust_max = max(thrust_min, -gravity_projection + root)
    thrust = float(np.clip(preferred_thrust, thrust_min, thrust_max))
    return gravity + thrust * constrained_up, True


def _orthogonal_unit(vector: np.ndarray) -> np.ndarray:
    value = _unit(vector)
    axis = np.array([1.0, 0.0, 0.0], dtype=float)
    if abs(float(np.dot(value, axis))) > 0.9:
        axis = np.array([0.0, 1.0, 0.0], dtype=float)
    return _unit(axis - value * float(np.dot(value, axis)))


def _age(now: float, timestamp: float | None) -> float | None:
    if timestamp is None or not math.isfinite(float(timestamp)):
        return None
    return max(0.0, now - float(timestamp))


def _unit(value: np.ndarray | None) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm <= 1.0e-12:
        raise ValueError("LOS vector must be nonzero")
    return vector / norm


def _clip_norm(vector: np.ndarray, limit: float) -> tuple[np.ndarray, bool]:
    value = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(value))
    if norm <= limit:
        return value, False
    return value * (limit / max(norm, 1.0e-12)), True


def _move_towards(current: np.ndarray, target: np.ndarray, max_delta: float) -> np.ndarray:
    current_value = np.asarray(current, dtype=float)
    delta = np.asarray(target, dtype=float) - current_value
    distance = float(np.linalg.norm(delta))
    if distance <= max_delta or distance <= 1.0e-12:
        return np.asarray(target, dtype=float)
    return current_value + delta * (max_delta / distance)


def _maximum_vector_budget_scale(
    protected: np.ndarray,
    candidate: np.ndarray,
    limit: float,
) -> float:
    protected_value = np.asarray(protected, dtype=float)
    candidate_value = np.asarray(candidate, dtype=float)
    if float(np.linalg.norm(protected_value + candidate_value)) <= limit + 1.0e-12:
        return 1.0
    quadratic = float(np.dot(candidate_value, candidate_value))
    if quadratic <= 1.0e-18:
        return 0.0
    linear = 2.0 * float(np.dot(protected_value, candidate_value))
    constant = float(np.dot(protected_value, protected_value)) - limit**2
    discriminant = max(0.0, linear**2 - 4.0 * quadratic * constant)
    upper_root = (-linear + math.sqrt(discriminant)) / (2.0 * quadratic)
    return float(np.clip(upper_root, 0.0, 1.0))


def _tuple3(vector: np.ndarray) -> tuple[float, float, float]:
    return tuple(float(value) for value in np.asarray(vector, dtype=float))  # type: ignore[return-value]
