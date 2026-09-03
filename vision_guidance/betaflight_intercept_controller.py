from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from enum import Enum

import numpy as np


class InterceptPhase(str, Enum):
    ACQUIRE = "ACQUIRE"
    ACCELERATE = "ACCELERATE"
    PNG_TRACK = "PNG_TRACK"
    ABORT = "ABORT"


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
    png_track_speed_ratio: float = 0.8
    acquire_consecutive_frames: int = 5
    detection_timeout_s: float = 0.35
    velocity_timeout_s: float = 0.5
    los_prediction_max_s: float = 0.0
    gravity_m_s2: float = 9.80665
    fov_constraint_half_angle_deg: float = 0.0

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
            self.detection_timeout_s,
            self.velocity_timeout_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("velocity-establishing PNG limits must be finite and positive")
        if not math.isfinite(self.fov_centering_gain_s2) or self.fov_centering_gain_s2 < 0.0:
            raise ValueError("fov_centering_gain_s2 must be finite and non-negative")
        if not 0.0 < self.png_track_speed_ratio <= 1.0:
            raise ValueError("png_track_speed_ratio must be in (0, 1]")
        if self.acquire_consecutive_frames < 1:
            raise ValueError("acquire_consecutive_frames must be positive")
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


@dataclass(frozen=True)
class VelocityEstablishingPngOutput:
    timestamp_s: float
    phase: InterceptPhase
    valid: bool
    reason: str
    acceleration_ned_m_s2: tuple[float, float, float]
    speed_acceleration_ned_m_s2: tuple[float, float, float]
    png_acceleration_ned_m_s2: tuple[float, float, float]
    fov_acceleration_ned_m_s2: tuple[float, float, float]
    velocity_reference_ned_m_s: tuple[float, float, float]
    los_speed_m_s: float | None
    detection_age_s: float | None
    velocity_age_s: float | None
    los_prediction_horizon_s: float
    acquire_count: int
    speed_saturated: bool
    png_saturated: bool
    fov_saturated: bool
    fov_constraint_active: bool
    total_saturated: bool

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
        self._last_los_timestamp_s: float | None = None
        self._has_valid_sample = False
        self._abort_reason = ""

    def reset(self) -> None:
        self.phase = InterceptPhase.ACQUIRE
        self._acquire_count = 0
        self._last_los_timestamp_s = None
        self._has_valid_sample = False
        self._abort_reason = ""

    def update(self, value: VelocityEstablishingPngInput) -> VelocityEstablishingPngOutput:
        now = float(value.timestamp_s)
        if not math.isfinite(now):
            return self._empty(0.0, "nonfinite_input", None, None)
        detection_age = _age(now, value.los_timestamp_s)
        velocity_age = _age(now, value.velocity_timestamp_s)
        invalid_reason = self._invalid_reason(value, detection_age, velocity_age)
        if self.phase == InterceptPhase.ABORT:
            return self._empty(now, self._abort_reason or "abort_latched", detection_age, velocity_age)
        if invalid_reason is not None:
            if self._has_valid_sample:
                self.phase = InterceptPhase.ABORT
                self._abort_reason = invalid_reason
                return self._empty(now, invalid_reason, detection_age, velocity_age)
            self._acquire_count = 0
            return self._empty(now, invalid_reason, detection_age, velocity_age)

        los = _unit(value.lambda_ned)
        los_dot = np.asarray(value.lambda_dot_ned_s, dtype=float)
        velocity = np.asarray(value.velocity_ned_m_s, dtype=float)
        R_IB = np.asarray(value.attitude_R_IB, dtype=float)
        prediction_horizon = min(
            detection_age or 0.0,
            self.config.los_prediction_max_s,
        )
        control_los = _unit(los + prediction_horizon * los_dot)
        self._has_valid_sample = True
        if value.los_timestamp_s != self._last_los_timestamp_s:
            self._last_los_timestamp_s = value.los_timestamp_s
            self._acquire_count += 1
        if self.phase == InterceptPhase.ACQUIRE:
            if self._acquire_count < self.config.acquire_consecutive_frames:
                return self._empty(now, "acquiring", detection_age, velocity_age)
            self.phase = InterceptPhase.ACCELERATE

        velocity_reference = self.config.fixed_vm_m_s * control_los
        velocity_reference[2] = float(
            np.clip(
                velocity_reference[2],
                -self.config.vertical_speed_reference_limit_m_s,
                self.config.vertical_speed_reference_limit_m_s,
            )
        )
        speed_accel, speed_saturated = _clip_norm(
            self.config.speed_gain_s_inv * (velocity_reference - velocity),
            self.config.speed_accel_limit_m_s2,
        )
        png_accel, png_saturated = _clip_norm(
            self.config.navigation_constant * self.config.fixed_vm_m_s * los_dot,
            self.config.png_accel_limit_m_s2,
        )
        los_body = R_IB.T @ control_los
        fov_body = np.array(
            [
                self.config.fov_centering_gain_s2 * los_body[0],
                self.config.fov_centering_gain_s2 * los_body[1],
                0.0,
            ],
            dtype=float,
        )
        fov_body, fov_saturated = _clip_norm(
            fov_body, self.config.fov_centering_accel_limit_m_s2
        )
        fov_accel = R_IB @ fov_body
        total, total_saturated = _clip_norm(
            speed_accel + png_accel + fov_accel,
            self.config.total_accel_limit_m_s2,
        )
        total, fov_constraint_active = _constrain_acceleration_to_los(
            total,
            control_los,
            gravity_m_s2=self.config.gravity_m_s2,
            half_angle_deg=self.config.fov_constraint_half_angle_deg,
            acceleration_limit_m_s2=self.config.total_accel_limit_m_s2,
        )
        total_saturated = total_saturated or fov_constraint_active
        los_speed = float(np.dot(velocity, los))
        if (
            self.phase == InterceptPhase.ACCELERATE
            and los_speed >= self.config.png_track_speed_ratio * self.config.fixed_vm_m_s
        ):
            self.phase = InterceptPhase.PNG_TRACK
        return VelocityEstablishingPngOutput(
            timestamp_s=now,
            phase=self.phase,
            valid=True,
            reason="active",
            acceleration_ned_m_s2=_tuple3(total),
            speed_acceleration_ned_m_s2=_tuple3(speed_accel),
            png_acceleration_ned_m_s2=_tuple3(png_accel),
            fov_acceleration_ned_m_s2=_tuple3(fov_accel),
            velocity_reference_ned_m_s=_tuple3(velocity_reference),
            los_speed_m_s=los_speed,
            detection_age_s=detection_age,
            velocity_age_s=velocity_age,
            los_prediction_horizon_s=prediction_horizon,
            acquire_count=self._acquire_count,
            speed_saturated=speed_saturated,
            png_saturated=png_saturated,
            fov_saturated=fov_saturated,
            fov_constraint_active=fov_constraint_active,
            total_saturated=total_saturated,
        )

    def _invalid_reason(
        self,
        value: VelocityEstablishingPngInput,
        detection_age: float | None,
        velocity_age: float | None,
    ) -> str | None:
        for timestamp in (value.los_timestamp_s, value.velocity_timestamp_s):
            if timestamp is not None and not math.isfinite(float(timestamp)):
                return "nonfinite_input"
        if not value.tracking_valid or value.lambda_ned is None or value.lambda_dot_ned_s is None:
            return value.tracking_reason or "tracking_invalid"
        if detection_age is None or detection_age > self.config.detection_timeout_s:
            return "detection_stale"
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
        return None

    def _empty(
        self,
        timestamp_s: float,
        reason: str,
        detection_age_s: float | None,
        velocity_age_s: float | None,
    ) -> VelocityEstablishingPngOutput:
        zero = (0.0, 0.0, 0.0)
        return VelocityEstablishingPngOutput(
            timestamp_s=timestamp_s,
            phase=self.phase,
            valid=False,
            reason=reason,
            acceleration_ned_m_s2=zero,
            speed_acceleration_ned_m_s2=zero,
            png_acceleration_ned_m_s2=zero,
            fov_acceleration_ned_m_s2=zero,
            velocity_reference_ned_m_s=zero,
            los_speed_m_s=None,
            detection_age_s=detection_age_s,
            velocity_age_s=velocity_age_s,
            los_prediction_horizon_s=0.0,
            acquire_count=self._acquire_count,
            speed_saturated=False,
            png_saturated=False,
            fov_saturated=False,
            fov_constraint_active=False,
            total_saturated=False,
        )


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


def _tuple3(vector: np.ndarray) -> tuple[float, float, float]:
    return tuple(float(value) for value in np.asarray(vector, dtype=float))  # type: ignore[return-value]
