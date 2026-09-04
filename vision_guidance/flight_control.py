from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Mapping, Protocol, Sequence

import numpy as np

from .geometry import validated_rotation_matrix
from .thrust_model import VoltageThrottleThrustModel
from .types import GuidanceEval


class FlightControllerAdapter(Protocol):
    """Minimal adapter contract for real flight-controller backends."""

    def open(self) -> None:
        ...

    def close(self) -> None:
        ...

    def send_rc(self, command: "RcCommand") -> None:
        ...


@dataclass(frozen=True)
class GuidanceSetpoint:
    timestamp: float
    roll_rate_deg_s: float = 0.0
    pitch_rate_deg_s: float = 0.0
    yaw_rate_deg_s: float = 0.0
    thrust: float = 0.5
    valid: bool = True
    source: str = ""
    reject_reason: str = ""
    mapping_type: str = ""
    desired_roll_angle_deg: float | None = None
    desired_pitch_angle_deg: float | None = None
    current_roll_angle_deg: float | None = None
    current_pitch_angle_deg: float | None = None
    roll_attitude_error_deg: float | None = None
    pitch_attitude_error_deg: float | None = None
    thrust_model: str = "fixed_hover"
    thrust_required_specific_force_mps2: float | None = None
    thrust_load_factor_raw_g: float | None = None
    thrust_command_raw: float | None = None
    thrust_command_limited: bool = False
    throttle_target_us: float | None = None
    thrust_model_voltage_v: float | None = None

    @classmethod
    def from_body_rates_rad_s(
        cls,
        timestamp: float,
        body_rates_rad_s: Sequence[float],
        thrust: float,
        *,
        valid: bool = True,
        source: str = "body_rate",
        reject_reason: str = "",
    ) -> "GuidanceSetpoint":
        rates = np.asarray(body_rates_rad_s, dtype=float)
        if rates.shape != (3,) or not np.all(np.isfinite(rates)):
            return cls(timestamp=timestamp, valid=False, source=source, reject_reason="invalid_body_rates")
        return cls(
            timestamp=timestamp,
            roll_rate_deg_s=float(np.rad2deg(rates[0])),
            pitch_rate_deg_s=float(np.rad2deg(rates[1])),
            yaw_rate_deg_s=float(np.rad2deg(rates[2])),
            thrust=float(thrust),
            valid=valid,
            source=source,
            reject_reason=reject_reason,
        )


class GuidanceSetpointHold:
    """Bridge asynchronous perception gaps without masking real rejections."""

    def __init__(self, timeout_s: float):
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = float(timeout_s)
        self._last_valid: GuidanceSetpoint | None = None
        self._last_valid_s: float | None = None

    def reset(self) -> None:
        self._last_valid = None
        self._last_valid_s = None

    def update(
        self,
        setpoint: GuidanceSetpoint,
        *,
        timestamp: float,
        allow_hold: bool,
        gate_open: bool,
    ) -> GuidanceSetpoint:
        now = float(timestamp)
        if not gate_open:
            self.reset()
            return setpoint
        if setpoint.valid:
            self._last_valid = setpoint
            self._last_valid_s = now
            return setpoint
        age_s = None if self._last_valid_s is None else max(0.0, now - self._last_valid_s)
        if allow_hold and self._last_valid is not None and age_s is not None and age_s <= self.timeout_s:
            return replace(
                self._last_valid,
                timestamp=now,
                source="guidance_hold",
                reject_reason="",
            )
        self.reset()
        return setpoint


@dataclass(frozen=True)
class EntryHandoffConfig:
    enabled: bool = False
    duration_s: float = 0.8
    gyro_max_age_s: float = 0.25
    rate_source: str = "zero"

    def __post_init__(self) -> None:
        if not np.isfinite(self.duration_s) or self.duration_s < 0.0:
            raise ValueError("entry handoff duration_s must be finite and non-negative")
        if not np.isfinite(self.gyro_max_age_s) or self.gyro_max_age_s <= 0.0:
            raise ValueError("entry handoff gyro_max_age_s must be finite and positive")
        if self.rate_source not in {"zero", "gyro"}:
            raise ValueError("entry handoff rate_source must be zero or gyro")

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "EntryHandoffConfig":
        return cls(
            enabled=bool(values.get("enabled", False)),
            duration_s=float(values.get("duration_s", 0.8)),
            gyro_max_age_s=float(values.get("gyro_max_age_s", 0.25)),
            rate_source=str(values.get("rate_source", "zero")),
        )


@dataclass(frozen=True)
class TiltEnvelopeConfig:
    enabled: bool = False
    max_roll_angle_deg: float = 35.0
    max_pitch_angle_deg: float = 35.0
    softcap_band_deg: float = 10.0
    hardcap_margin_deg: float = 5.0
    hardcap_level_kp: float = 3.0
    hardcap_max_level_rate_deg_s: float = 60.0

    def __post_init__(self) -> None:
        for name in ("max_roll_angle_deg", "max_pitch_angle_deg"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0 or value > 90.0:
                raise ValueError(f"{name} must be finite and in (0, 90]")
        if (
            not np.isfinite(self.softcap_band_deg)
            or self.softcap_band_deg < 0.0
            or self.softcap_band_deg >= min(self.max_roll_angle_deg, self.max_pitch_angle_deg)
        ):
            raise ValueError("softcap_band_deg must be finite, non-negative, and below both angle limits")
        for name in ("hardcap_margin_deg", "hardcap_level_kp", "hardcap_max_level_rate_deg_s"):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "TiltEnvelopeConfig":
        return cls(
            enabled=bool(values.get("enabled", False)),
            max_roll_angle_deg=float(values.get("max_roll_angle_deg", 35.0)),
            max_pitch_angle_deg=float(values.get("max_pitch_angle_deg", 35.0)),
            softcap_band_deg=float(values.get("softcap_band_deg", 10.0)),
            hardcap_margin_deg=float(values.get("hardcap_margin_deg", 5.0)),
            hardcap_level_kp=float(values.get("hardcap_level_kp", 3.0)),
            hardcap_max_level_rate_deg_s=float(values.get("hardcap_max_level_rate_deg_s", 60.0)),
        )


@dataclass(frozen=True)
class GuidanceCommandShaperConfig:
    entry_handoff: EntryHandoffConfig = field(default_factory=EntryHandoffConfig)
    tilt_envelope: TiltEnvelopeConfig = field(default_factory=TiltEnvelopeConfig)

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "GuidanceCommandShaperConfig":
        entry = values.get("entry_handoff", {})
        tilt = values.get("tilt_envelope", {})
        if not isinstance(entry, Mapping) or not isinstance(tilt, Mapping):
            raise ValueError("entry_handoff and tilt_envelope must be mappings")
        return cls(
            entry_handoff=EntryHandoffConfig.from_mapping(entry),
            tilt_envelope=TiltEnvelopeConfig.from_mapping(tilt),
        )


@dataclass(frozen=True)
class GuidanceCommandShapingDiagnostics:
    valid: bool = True
    reason: str = ""
    input_roll_rate_deg_s: float = 0.0
    input_pitch_rate_deg_s: float = 0.0
    output_roll_rate_deg_s: float = 0.0
    output_pitch_rate_deg_s: float = 0.0
    entry_active: bool = False
    entry_progress: float = 1.0
    entry_source: str = "disabled"
    entry_start_roll_rate_deg_s: float = 0.0
    entry_start_pitch_rate_deg_s: float = 0.0
    roll_attitude_deg: float | None = None
    pitch_attitude_deg: float | None = None
    roll_softcap_factor: float = 1.0
    pitch_softcap_factor: float = 1.0
    roll_level_weight: float = 0.0
    pitch_level_weight: float = 0.0
    hardcap_active: bool = False


@dataclass(frozen=True)
class ThrustFeedforwardConfig:
    enabled: bool = False
    model: str = "fixed_hover"
    hover_load_factor_g: float = 1.0
    max_load_factor_g: float = 2.37
    minimum_tilt_cosine: float = 0.5
    calibration_id: str = ""
    model_path: str = ""
    model_sha256: str = ""

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "ThrustFeedforwardConfig":
        return cls(
            enabled=bool(values.get("enabled", False)),
            model=str(values.get("model", "fixed_hover")).strip().lower(),
            hover_load_factor_g=float(values.get("hover_load_factor_g", 1.0)),
            max_load_factor_g=float(values.get("max_load_factor_g", 2.37)),
            minimum_tilt_cosine=float(values.get("minimum_tilt_cosine", 0.5)),
            calibration_id=str(values.get("calibration_id", "")).strip(),
            model_path=str(values.get("model_path", "")).strip(),
            model_sha256=str(values.get("model_sha256", "")).strip().lower(),
        )

    def __post_init__(self) -> None:
        if self.model not in {
            "fixed_hover",
            "measured_load_factor",
            "voltage_throttle_lut",
        }:
            raise ValueError("unsupported thrust feedforward model")
        if self.enabled and self.model not in {
            "measured_load_factor",
            "voltage_throttle_lut",
        }:
            raise ValueError("enabled thrust feedforward requires a calibrated model")
        if not np.isfinite(self.hover_load_factor_g) or self.hover_load_factor_g <= 0.0:
            raise ValueError("hover_load_factor_g must be finite and positive")
        if (
            not np.isfinite(self.max_load_factor_g)
            or self.max_load_factor_g <= self.hover_load_factor_g
        ):
            raise ValueError("max_load_factor_g must exceed hover_load_factor_g")
        if (
            not np.isfinite(self.minimum_tilt_cosine)
            or not 0.0 < self.minimum_tilt_cosine <= 1.0
        ):
            raise ValueError("minimum_tilt_cosine must be in (0, 1]")
        if self.enabled and not self.calibration_id:
            raise ValueError("enabled thrust feedforward requires calibration_id")
        if self.enabled and self.model == "voltage_throttle_lut":
            if not self.model_path:
                raise ValueError("voltage thrust feedforward requires model_path")
            if len(self.model_sha256) != 64 or any(
                character not in "0123456789abcdef" for character in self.model_sha256
            ):
                raise ValueError("voltage thrust feedforward requires a SHA256 hash")


@dataclass(frozen=True)
class AccelerationTiltRateConfig:
    """Map an inertial acceleration demand through an attitude outer loop."""

    gravity_mps2: float = 9.80665
    roll_attitude_kp_s_inv: float = 4.0
    pitch_attitude_kp_s_inv: float = 4.0
    max_roll_tilt_deg: float = 20.0
    max_pitch_tilt_deg: float = 20.0
    max_roll_rate_deg_s: float = 60.0
    max_pitch_rate_deg_s: float = 60.0
    roll_rate_sign: float = 1.0
    pitch_rate_sign: float = 1.0
    min_vertical_specific_force_mps2: float = 0.5
    thrust_feedforward: ThrustFeedforwardConfig = field(
        default_factory=ThrustFeedforwardConfig
    )

    def __post_init__(self) -> None:
        positive_fields = (
            "gravity_mps2",
            "roll_attitude_kp_s_inv",
            "pitch_attitude_kp_s_inv",
            "max_roll_tilt_deg",
            "max_pitch_tilt_deg",
            "max_roll_rate_deg_s",
            "max_pitch_rate_deg_s",
            "min_vertical_specific_force_mps2",
        )
        for name in positive_fields:
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.max_roll_tilt_deg >= 90.0 or self.max_pitch_tilt_deg >= 90.0:
            raise ValueError("acceleration tilt limits must be below 90 degrees")
        for name in ("roll_rate_sign", "pitch_rate_sign"):
            value = float(getattr(self, name))
            if value not in (-1.0, 1.0):
                raise ValueError(f"{name} must be -1 or 1")

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "AccelerationTiltRateConfig":
        return cls(
            gravity_mps2=float(values.get("gravity_mps2", 9.80665)),
            roll_attitude_kp_s_inv=float(values.get("roll_attitude_kp_s_inv", 4.0)),
            pitch_attitude_kp_s_inv=float(values.get("pitch_attitude_kp_s_inv", 4.0)),
            max_roll_tilt_deg=float(values.get("max_roll_tilt_deg", 20.0)),
            max_pitch_tilt_deg=float(values.get("max_pitch_tilt_deg", 20.0)),
            max_roll_rate_deg_s=float(values.get("max_roll_rate_deg_s", 60.0)),
            max_pitch_rate_deg_s=float(values.get("max_pitch_rate_deg_s", 60.0)),
            roll_rate_sign=float(values.get("roll_rate_sign", 1.0)),
            pitch_rate_sign=float(values.get("pitch_rate_sign", 1.0)),
            min_vertical_specific_force_mps2=float(
                values.get("min_vertical_specific_force_mps2", 0.5)
            ),
            thrust_feedforward=ThrustFeedforwardConfig.from_mapping(
                dict(values.get("thrust_feedforward", {}))
            ),
        )


class GuidanceCommandShaper:
    """Smooth algorithm engagement and constrain outward rates near tilt limits."""

    def __init__(self, config: GuidanceCommandShaperConfig):
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._engaged = False
        self._entry_start_s: float | None = None
        self._entry_start_roll_rate_deg_s = 0.0
        self._entry_start_pitch_rate_deg_s = 0.0
        self._entry_source = "disabled"

    def update(
        self,
        setpoint: GuidanceSetpoint,
        *,
        timestamp: float,
        gate_open: bool,
        attitude_deg: Sequence[float] | None,
        gyro_deg_s: Sequence[float] | None,
        gyro_age_s: float | None,
    ) -> tuple[GuidanceSetpoint, GuidanceCommandShapingDiagnostics]:
        now = float(timestamp)
        input_roll = float(setpoint.roll_rate_deg_s)
        input_pitch = float(setpoint.pitch_rate_deg_s)
        if not gate_open or not setpoint.valid:
            self.reset()
            return setpoint, GuidanceCommandShapingDiagnostics(
                valid=setpoint.valid,
                reason=setpoint.reject_reason or ("gate_closed" if not gate_open else ""),
                input_roll_rate_deg_s=input_roll,
                input_pitch_rate_deg_s=input_pitch,
                output_roll_rate_deg_s=input_roll,
                output_pitch_rate_deg_s=input_pitch,
            )

        if not np.all(
            np.isfinite(
                [
                    now,
                    setpoint.roll_rate_deg_s,
                    setpoint.pitch_rate_deg_s,
                    setpoint.yaw_rate_deg_s,
                    setpoint.thrust,
                ]
            )
        ) or (
            setpoint.throttle_target_us is not None
            and not np.isfinite(setpoint.throttle_target_us)
        ):
            self.reset()
            invalid = replace(setpoint, valid=False, reject_reason="command_shaper_nonfinite_setpoint")
            return invalid, GuidanceCommandShapingDiagnostics(
                valid=False,
                reason=invalid.reject_reason,
                input_roll_rate_deg_s=input_roll,
                input_pitch_rate_deg_s=input_pitch,
                output_roll_rate_deg_s=0.0,
                output_pitch_rate_deg_s=0.0,
            )

        tilt = self.config.tilt_envelope
        attitude = _finite_pair(attitude_deg)
        if tilt.enabled and attitude is None:
            self.reset()
            invalid = replace(setpoint, valid=False, reject_reason="tilt_attitude_unavailable")
            return invalid, GuidanceCommandShapingDiagnostics(
                valid=False,
                reason=invalid.reject_reason,
                input_roll_rate_deg_s=input_roll,
                input_pitch_rate_deg_s=input_pitch,
                output_roll_rate_deg_s=0.0,
                output_pitch_rate_deg_s=0.0,
            )

        if not self._engaged:
            self._start_entry(now, gyro_deg_s, gyro_age_s)

        roll_rate, pitch_rate, entry_active, entry_progress = self._apply_entry(
            input_roll,
            input_pitch,
            now,
        )
        roll_factor = 1.0
        pitch_factor = 1.0
        roll_weight = 0.0
        pitch_weight = 0.0
        if tilt.enabled and attitude is not None:
            roll_rate, roll_factor, roll_weight = _apply_tilt_axis(
                roll_rate,
                attitude[0],
                tilt.max_roll_angle_deg,
                tilt.softcap_band_deg,
                tilt.hardcap_margin_deg,
                tilt.hardcap_level_kp,
                tilt.hardcap_max_level_rate_deg_s,
            )
            pitch_rate, pitch_factor, pitch_weight = _apply_tilt_axis(
                pitch_rate,
                attitude[1],
                tilt.max_pitch_angle_deg,
                tilt.softcap_band_deg,
                tilt.hardcap_margin_deg,
                tilt.hardcap_level_kp,
                tilt.hardcap_max_level_rate_deg_s,
            )

        shaped = replace(
            setpoint,
            roll_rate_deg_s=roll_rate,
            pitch_rate_deg_s=pitch_rate,
        )
        return shaped, GuidanceCommandShapingDiagnostics(
            input_roll_rate_deg_s=input_roll,
            input_pitch_rate_deg_s=input_pitch,
            output_roll_rate_deg_s=roll_rate,
            output_pitch_rate_deg_s=pitch_rate,
            entry_active=entry_active,
            entry_progress=entry_progress,
            entry_source=self._entry_source,
            entry_start_roll_rate_deg_s=self._entry_start_roll_rate_deg_s,
            entry_start_pitch_rate_deg_s=self._entry_start_pitch_rate_deg_s,
            roll_attitude_deg=None if attitude is None else attitude[0],
            pitch_attitude_deg=None if attitude is None else attitude[1],
            roll_softcap_factor=roll_factor,
            pitch_softcap_factor=pitch_factor,
            roll_level_weight=roll_weight,
            pitch_level_weight=pitch_weight,
            hardcap_active=max(roll_weight, pitch_weight) > 0.5,
        )

    def _start_entry(
        self,
        timestamp: float,
        gyro_deg_s: Sequence[float] | None,
        gyro_age_s: float | None,
    ) -> None:
        self._engaged = True
        self._entry_start_s = timestamp
        self._entry_start_roll_rate_deg_s = 0.0
        self._entry_start_pitch_rate_deg_s = 0.0
        self._entry_source = "zero"
        entry = self.config.entry_handoff
        gyro = _finite_pair(gyro_deg_s)
        gyro_fresh = (
            gyro is not None
            and gyro_age_s is not None
            and np.isfinite(gyro_age_s)
            and 0.0 <= float(gyro_age_s) <= entry.gyro_max_age_s
        )
        if entry.enabled and entry.rate_source == "gyro" and gyro_fresh and gyro is not None:
            self._entry_start_roll_rate_deg_s = gyro[0]
            self._entry_start_pitch_rate_deg_s = gyro[1]
            self._entry_source = "gyro"
        elif not entry.enabled:
            self._entry_source = "disabled"

    def _apply_entry(
        self,
        target_roll_rate_deg_s: float,
        target_pitch_rate_deg_s: float,
        timestamp: float,
    ) -> tuple[float, float, bool, float]:
        entry = self.config.entry_handoff
        if not entry.enabled or entry.duration_s <= 0.0 or self._entry_start_s is None:
            return target_roll_rate_deg_s, target_pitch_rate_deg_s, False, 1.0
        elapsed_s = max(0.0, timestamp - self._entry_start_s)
        linear_progress = float(np.clip(elapsed_s / max(1.0e-9, entry.duration_s), 0.0, 1.0))
        progress = linear_progress * linear_progress * (3.0 - 2.0 * linear_progress)
        roll_rate = _lerp(self._entry_start_roll_rate_deg_s, target_roll_rate_deg_s, progress)
        pitch_rate = _lerp(self._entry_start_pitch_rate_deg_s, target_pitch_rate_deg_s, progress)
        return roll_rate, pitch_rate, linear_progress < 1.0, progress


def _finite_pair(values: Sequence[float] | None) -> tuple[float, float] | None:
    if values is None or len(values) < 2:
        return None
    pair = (float(values[0]), float(values[1]))
    return pair if np.all(np.isfinite(pair)) else None


def _lerp(start: float, end: float, progress: float) -> float:
    return float(start + (end - start) * progress)


def _smoothstep01(value: float) -> float:
    x = float(np.clip(value, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


def _apply_tilt_axis(
    command_rate_deg_s: float,
    attitude_deg: float,
    max_angle_deg: float,
    softcap_band_deg: float,
    hardcap_margin_deg: float,
    level_kp: float,
    max_level_rate_deg_s: float,
) -> tuple[float, float, float]:
    command = float(command_rate_deg_s)
    attitude = float(attitude_deg)
    outward = attitude * command > 0.0
    abs_attitude = abs(attitude)
    if not outward or abs_attitude < max_angle_deg - softcap_band_deg:
        softcap_factor = 1.0
    elif abs_attitude >= max_angle_deg:
        softcap_factor = 0.0
    elif softcap_band_deg > 0.0:
        softcap_factor = 1.0 - float(
            np.clip(
                (abs_attitude - (max_angle_deg - softcap_band_deg)) / softcap_band_deg,
                0.0,
                1.0,
            )
        )
    else:
        softcap_factor = 1.0
    soft_command = command * softcap_factor
    level_rate = float(np.clip(-level_kp * attitude, -max_level_rate_deg_s, max_level_rate_deg_s))
    if hardcap_margin_deg > 1.0e-9:
        level_weight = _smoothstep01((abs_attitude - max_angle_deg) / hardcap_margin_deg)
    else:
        level_weight = 1.0 if abs_attitude >= max_angle_deg else 0.0
    output = _lerp(soft_command, level_rate, level_weight)
    return output, softcap_factor, level_weight


@dataclass(frozen=True)
class RcCommand:
    timestamp: float
    channels: tuple[int, ...]
    active: bool
    reason: str = ""
    raw_channels: tuple[int, ...] = ()
    target_channels: tuple[int, ...] = ()
    clipped_flags: tuple[int, ...] = ()
    slew_limited_flags: tuple[int, ...] = ()
    requested_rates_deg_s: tuple[float, ...] = ()
    limited_rates_deg_s: tuple[float, ...] = ()
    stick_deflections: tuple[float, ...] = ()
    requested_thrust: float | None = None
    limited_thrust: float | None = None
    requested_throttle_us: float | None = None

    def __post_init__(self) -> None:
        if not self.channels:
            raise ValueError("channels must not be empty")
        for value in self.channels:
            if int(value) != value:
                raise ValueError("RC channel values must be integers")
        for name in ("raw_channels", "target_channels", "clipped_flags", "slew_limited_flags"):
            values = getattr(self, name)
            if values and len(values) != len(self.channels):
                raise ValueError(f"{name} length must match channels")
        for name in ("requested_rates_deg_s", "limited_rates_deg_s", "stick_deflections"):
            values = getattr(self, name)
            if values and len(values) != 3:
                raise ValueError(f"{name} must contain roll, pitch, and yaw values")


@dataclass(frozen=True)
class RcMappingConfig:
    channel_map: str = "AETR1234"
    channel_count: int = 8
    min_us: int = 1000
    mid_us: int = 1500
    max_us: int = 2000
    roll_rate_limit_deg_s: float = 120.0
    pitch_rate_limit_deg_s: float = 120.0
    yaw_rate_limit_deg_s: float = 90.0
    rate_mapping_type: str = "linear"
    betaflight_rc_rate: tuple[float, float, float] = (1.0, 1.0, 1.0)
    betaflight_super_rate: tuple[float, float, float] = (0.0, 0.0, 0.0)
    betaflight_expo: tuple[float, float, float] = (0.0, 0.0, 0.0)
    roll_command_limit_deg_s: float | None = None
    pitch_command_limit_deg_s: float | None = None
    yaw_command_limit_deg_s: float | None = None
    thrust_min: float = 0.0
    thrust_hover: float = 0.5
    thrust_max: float = 1.0
    throttle_min_us: int = 1000
    throttle_hover_us: int = 1500
    throttle_max_us: int = 2000
    neutral_throttle_us: int = 1000
    max_delta_us_per_s: float = 0.0
    aux_values_us: Mapping[int, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.channel_count < 4:
            raise ValueError("channel_count must be at least 4")
        if len(self.channel_map) != self.channel_count:
            raise ValueError("channel_map length must match channel_count")
        for role in ("A", "E", "T", "R"):
            if self.channel_map.count(role) != 1:
                raise ValueError(f"channel_map must contain exactly one {role}")
        if not (self.min_us < self.mid_us < self.max_us):
            raise ValueError("RC min/mid/max must be strictly increasing")
        if not (self.thrust_min <= self.thrust_hover <= self.thrust_max):
            raise ValueError("thrust_min <= thrust_hover <= thrust_max is required")
        if not (
            self.min_us <= self.throttle_min_us <= self.throttle_hover_us <= self.throttle_max_us <= self.max_us
        ):
            raise ValueError("throttle PWM endpoints must be ordered within RC endpoints")
        if not self.throttle_min_us <= self.neutral_throttle_us <= self.throttle_max_us:
            raise ValueError("neutral throttle must be within throttle PWM endpoints")
        if self.rate_mapping_type not in {"linear", "betaflight"}:
            raise ValueError("rate_mapping_type must be linear or betaflight")
        for name in ("betaflight_rc_rate", "betaflight_super_rate", "betaflight_expo"):
            values = tuple(float(value) for value in getattr(self, name))
            if len(values) != 3 or not np.all(np.isfinite(values)):
                raise ValueError(f"{name} must contain three finite values")
        if any(value <= 0.0 for value in self.betaflight_rc_rate):
            raise ValueError("betaflight_rc_rate values must be positive")
        if any(value < 0.0 or value > 1.0 for value in self.betaflight_super_rate):
            raise ValueError("betaflight_super_rate values must be in [0, 1]")
        if any(value < 0.0 or value > 1.0 for value in self.betaflight_expo):
            raise ValueError("betaflight_expo values must be in [0, 1]")
        for name in ("roll_command_limit_deg_s", "pitch_command_limit_deg_s", "yaw_command_limit_deg_s"):
            value = getattr(self, name)
            if value is not None and (not np.isfinite(value) or value < 0.0):
                raise ValueError(f"{name} must be finite and non-negative")


class RcCommandMapper:
    def __init__(self, config: RcMappingConfig):
        self.config = config
        self._previous: RcCommand | None = None

    def neutral(self, timestamp: float, reason: str = "neutral") -> RcCommand:
        channels, raw_channels, clipped_flags = self._base_channels_with_flags()
        throttle_index = self._role_index("T")
        raw_channels[throttle_index] = int(round(float(self.config.neutral_throttle_us)))
        channels[throttle_index] = self._clip_us(self.config.neutral_throttle_us)
        clipped_flags[throttle_index] = int(raw_channels[throttle_index] != channels[throttle_index])
        command = RcCommand(
            timestamp=timestamp,
            channels=tuple(channels),
            active=False,
            reason=reason,
            raw_channels=tuple(raw_channels),
            target_channels=tuple(channels),
            clipped_flags=tuple(clipped_flags),
            slew_limited_flags=self._zero_flags(),
        )
        self._previous = command
        return command

    def map_setpoint(self, setpoint: GuidanceSetpoint, *, active: bool = True) -> RcCommand:
        timestamp = float(setpoint.timestamp)
        if not active:
            return self._map_neutral_with_slew(timestamp, "inactive")
        if not setpoint.valid:
            return self._map_neutral_with_slew(timestamp, setpoint.reject_reason or "setpoint_invalid")
        values = np.array(
            [
                setpoint.roll_rate_deg_s,
                setpoint.pitch_rate_deg_s,
                setpoint.yaw_rate_deg_s,
                setpoint.thrust,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            return self._map_neutral_with_slew(timestamp, "setpoint_nonfinite")

        channels, raw_channels, clipped_flags = self._base_channels_with_flags()
        requested_rates: list[float] = []
        limited_rates: list[float] = []
        stick_deflections: list[float] = []
        for role, rate, limit in (
            ("A", setpoint.roll_rate_deg_s, self.config.roll_rate_limit_deg_s),
            ("E", setpoint.pitch_rate_deg_s, self.config.pitch_rate_limit_deg_s),
            ("R", setpoint.yaw_rate_deg_s, self.config.yaw_rate_limit_deg_s),
        ):
            index = self._role_index(role)
            raw, clipped, was_clipped = self._rate_to_us_with_clip(role, rate, limit)
            raw_channels[index] = raw
            channels[index] = clipped
            clipped_flags[index] = int(was_clipped)
            requested_rates.append(float(rate))
            limited_rate = self._limit_rate_command(role, float(rate))
            limited_rates.append(limited_rate)
            stick_deflections.append(self._rate_to_stick_deflection(role, limited_rate, limit))
        throttle_index = self._role_index("T")
        if setpoint.throttle_target_us is None:
            raw, clipped, was_clipped = self._thrust_to_us_with_clip(setpoint.thrust)
        else:
            raw = int(round(float(setpoint.throttle_target_us)))
            clipped = self._clip_throttle_us(setpoint.throttle_target_us)
            was_clipped = raw != clipped
        raw_channels[throttle_index] = raw
        channels[throttle_index] = clipped
        clipped_flags[throttle_index] = int(was_clipped)
        target_channels = tuple(channels)
        channels_tuple, slew_flags = self._apply_slew(target_channels, timestamp)
        limited_thrust = float(np.clip(setpoint.thrust, self.config.thrust_min, self.config.thrust_max))
        command = RcCommand(
            timestamp=timestamp,
            channels=channels_tuple,
            active=True,
            reason=setpoint.source,
            raw_channels=tuple(raw_channels),
            target_channels=target_channels,
            clipped_flags=tuple(clipped_flags),
            slew_limited_flags=slew_flags,
            requested_rates_deg_s=tuple(requested_rates),
            limited_rates_deg_s=tuple(limited_rates),
            stick_deflections=tuple(stick_deflections),
            requested_thrust=float(setpoint.thrust),
            limited_thrust=limited_thrust,
            requested_throttle_us=setpoint.throttle_target_us,
        )
        self._previous = command
        return command

    def _map_neutral_with_slew(self, timestamp: float, reason: str) -> RcCommand:
        channels, raw_channels, clipped_flags = self._base_channels_with_flags()
        throttle_index = self._role_index("T")
        raw_channels[throttle_index] = int(round(float(self.config.neutral_throttle_us)))
        channels[throttle_index] = self._clip_us(self.config.neutral_throttle_us)
        clipped_flags[throttle_index] = int(raw_channels[throttle_index] != channels[throttle_index])
        target_channels = tuple(channels)
        channels_tuple, slew_flags = self._apply_slew(target_channels, timestamp)
        command = RcCommand(
            timestamp=timestamp,
            channels=channels_tuple,
            active=False,
            reason=reason,
            raw_channels=tuple(raw_channels),
            target_channels=target_channels,
            clipped_flags=tuple(clipped_flags),
            slew_limited_flags=slew_flags,
        )
        self._previous = command
        return command

    def _base_channels(self) -> list[int]:
        channels, _raw_channels, _clipped_flags = self._base_channels_with_flags()
        return channels

    def _base_channels_with_flags(self) -> tuple[list[int], list[int], list[int]]:
        channels = [int(self.config.mid_us)] * self.config.channel_count
        raw_channels = [int(self.config.mid_us)] * self.config.channel_count
        clipped_flags = [0] * self.config.channel_count
        for aux_number, value in self.config.aux_values_us.items():
            role = str(int(aux_number))
            if role in self.config.channel_map:
                index = self._role_index(role)
                raw_value = int(round(float(value)))
                clipped_value = self._clip_us(value)
                raw_channels[index] = raw_value
                channels[index] = clipped_value
                clipped_flags[index] = int(raw_value != clipped_value)
        return channels, raw_channels, clipped_flags

    def _role_index(self, role: str) -> int:
        return self.config.channel_map.index(role)

    def _rate_to_us(self, role: str, rate_deg_s: float, limit_deg_s: float) -> int:
        _raw, clipped, _was_clipped = self._rate_to_us_with_clip(role, rate_deg_s, limit_deg_s)
        return clipped

    def _rate_to_us_with_clip(self, role: str, rate_deg_s: float, limit_deg_s: float) -> tuple[int, int, bool]:
        requested = float(rate_deg_s)
        bounded = self._limit_rate_command(role, requested)
        if self.config.rate_mapping_type == "betaflight":
            axis = self._rate_axis(role)
            raw_deflection, raw_saturated = self._betaflight_rate_to_deflection(requested, axis)
            deflection, bounded_saturated = self._betaflight_rate_to_deflection(bounded, axis)
            raw = self._deflection_to_us(raw_deflection)
            clipped = self._clip_us(self._deflection_to_us(deflection))
            was_clipped = requested != bounded or raw_saturated or bounded_saturated or raw != clipped
            return raw, clipped, was_clipped

        limit = max(1.0e-6, float(limit_deg_s))
        span = 0.5 * float(self.config.max_us - self.config.min_us)
        raw = int(round(float(self.config.mid_us) + (requested / limit) * span))
        bounded_raw = int(round(float(self.config.mid_us) + (bounded / limit) * span))
        clipped = self._clip_us(bounded_raw)
        return raw, clipped, requested != bounded or bounded_raw != clipped

    def _limit_rate_command(self, role: str, value: float) -> float:
        limit = {
            "A": self.config.roll_command_limit_deg_s,
            "E": self.config.pitch_command_limit_deg_s,
            "R": self.config.yaw_command_limit_deg_s,
        }[role]
        if limit is None:
            return float(value)
        return float(np.clip(value, -float(limit), float(limit)))

    def _rate_to_stick_deflection(self, role: str, rate_deg_s: float, limit_deg_s: float) -> float:
        if self.config.rate_mapping_type == "betaflight":
            deflection, _saturated = self._betaflight_rate_to_deflection(rate_deg_s, self._rate_axis(role))
            return float(deflection)
        limit = max(1.0e-6, float(limit_deg_s))
        return float(np.clip(float(rate_deg_s) / limit, -1.0, 1.0))

    @staticmethod
    def _rate_axis(role: str) -> int:
        return {"A": 0, "E": 1, "R": 2}[role]

    def _betaflight_rate_to_deflection(self, rate_deg_s: float, axis: int) -> tuple[float, bool]:
        requested = float(rate_deg_s)
        sign = -1.0 if requested < 0.0 else 1.0
        target = abs(requested)
        maximum = self._betaflight_rate_from_deflection(1.0, axis)
        if target >= maximum:
            return sign, target > maximum
        low = 0.0
        high = 1.0
        for _ in range(48):
            middle = 0.5 * (low + high)
            if self._betaflight_rate_from_deflection(middle, axis) < target:
                low = middle
            else:
                high = middle
        return sign * (0.5 * (low + high)), False

    def _betaflight_rate_from_deflection(self, deflection: float, axis: int) -> float:
        value = float(np.clip(deflection, 0.0, 1.0))
        expo = float(self.config.betaflight_expo[axis])
        curved = value * (value**3) * expo + value * (1.0 - expo)
        rc_rate = float(self.config.betaflight_rc_rate[axis])
        if rc_rate > 2.0:
            rc_rate += 14.54 * (rc_rate - 2.0)
        angle_rate = 200.0 * rc_rate * curved
        super_rate = float(self.config.betaflight_super_rate[axis])
        if super_rate > 0.0:
            angle_rate /= max(0.01, 1.0 - value * super_rate)
        return angle_rate

    def _deflection_to_us(self, deflection: float) -> int:
        value = float(deflection)
        if value >= 0.0:
            span = float(self.config.max_us - self.config.mid_us)
        else:
            span = float(self.config.mid_us - self.config.min_us)
        return int(round(float(self.config.mid_us) + value * span))

    def _thrust_to_us(self, thrust: float) -> int:
        _raw, clipped, _was_clipped = self._thrust_to_us_with_clip(thrust)
        return clipped

    def _thrust_to_us_with_clip(self, thrust: float) -> tuple[int, int, bool]:
        raw_value = float(thrust)
        bounded_value = float(np.clip(raw_value, self.config.thrust_min, self.config.thrust_max))
        raw_us = self._thrust_value_to_us(raw_value)
        bounded_us = self._thrust_value_to_us(bounded_value)
        raw = int(round(raw_us))
        clipped = self._clip_throttle_us(bounded_us)
        return raw, clipped, raw_value != bounded_value or raw != clipped

    def _thrust_value_to_us(self, value: float) -> float:
        if value <= self.config.thrust_hover:
            denom = max(1.0e-9, self.config.thrust_hover - self.config.thrust_min)
            alpha = (value - self.config.thrust_min) / denom
            return self.config.throttle_min_us + alpha * (self.config.throttle_hover_us - self.config.throttle_min_us)
        denom = max(1.0e-9, self.config.thrust_max - self.config.thrust_hover)
        alpha = (value - self.config.thrust_hover) / denom
        return self.config.throttle_hover_us + alpha * (self.config.throttle_max_us - self.config.throttle_hover_us)

    def _clip_us(self, value: float) -> int:
        return int(np.clip(round(float(value)), self.config.min_us, self.config.max_us))

    def _clip_throttle_us(self, value: float) -> int:
        return int(np.clip(round(float(value)), self.config.throttle_min_us, self.config.throttle_max_us))

    def _apply_slew(self, target: tuple[int, ...], timestamp: float) -> tuple[tuple[int, ...], tuple[int, ...]]:
        limit = float(self.config.max_delta_us_per_s)
        if limit <= 0.0 or self._previous is None:
            return target, self._zero_flags()
        dt = max(0.0, float(timestamp) - float(self._previous.timestamp))
        if dt <= 0.0:
            return target, self._zero_flags()
        max_delta = limit * dt
        previous = np.asarray(self._previous.channels, dtype=float)
        current = np.asarray(target, dtype=float)
        delta = np.clip(current - previous, -max_delta, max_delta)
        result = tuple(self._clip_us(value) for value in previous + delta)
        flags = tuple(int(int(result_value) != int(target_value)) for result_value, target_value in zip(result, target))
        return result, flags

    def _zero_flags(self) -> tuple[int, ...]:
        return tuple(0 for _ in range(self.config.channel_count))


class SafetyState(str, Enum):
    DISABLED = "DISABLED"
    LOG_ONLY = "LOG_ONLY"
    READY = "READY"
    ACTIVE = "ACTIVE"
    DEGRADED = "DEGRADED"
    FAILSAFE = "FAILSAFE"


@dataclass(frozen=True)
class MotorOutputInterlockConfig:
    enabled: bool = False
    channel_count: int = 4
    max_output_us: int = 1200
    max_spread_us: int = 150
    violation_grace_s: float = 0.0
    telemetry_timeout_s: float = 0.75
    latch_until_disarm: bool = True

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "MotorOutputInterlockConfig":
        return cls(
            enabled=bool(values.get("enabled", False)),
            channel_count=int(values.get("channel_count", 4)),
            max_output_us=int(values.get("max_output_us", 1200)),
            max_spread_us=int(values.get("max_spread_us", 150)),
            violation_grace_s=float(values.get("violation_grace_s", 0.0)),
            telemetry_timeout_s=float(values.get("telemetry_timeout_s", 0.75)),
            latch_until_disarm=bool(values.get("latch_until_disarm", True)),
        )

    def __post_init__(self) -> None:
        if not 1 <= self.channel_count <= 8:
            raise ValueError("motor interlock channel_count must be in [1, 8]")
        if not 1000 <= self.max_output_us <= 2000:
            raise ValueError("motor interlock max_output_us must be in [1000, 2000]")
        if not 0 <= self.max_spread_us <= 1000:
            raise ValueError("motor interlock max_spread_us must be in [0, 1000]")
        if not np.isfinite(self.violation_grace_s) or self.violation_grace_s < 0.0:
            raise ValueError("motor interlock violation_grace_s must be finite and non-negative")
        if self.telemetry_timeout_s <= 0.0:
            raise ValueError("motor interlock telemetry_timeout_s must be positive")


@dataclass(frozen=True)
class MotorOutputInterlockState:
    ok: bool
    reason: str
    latched: bool
    output_max_us: float | None = None
    output_spread_us: float | None = None
    telemetry_age_s: float | None = None


class MotorOutputInterlock:
    def __init__(self, config: MotorOutputInterlockConfig):
        self.config = config
        self._latched = False
        self._latched_reason = ""
        self._latched_output_max_us: float | None = None
        self._latched_output_spread_us: float | None = None
        self._spread_violation_started_s: float | None = None

    def update(
        self,
        *,
        armed: bool,
        motor_outputs: Sequence[float] | None,
        telemetry_age_s: float | None,
        timestamp: float | None = None,
    ) -> MotorOutputInterlockState:
        if not self.config.enabled:
            return MotorOutputInterlockState(True, "disabled", False, telemetry_age_s=telemetry_age_s)
        if not armed:
            self._latched = False
            self._latched_reason = ""
            self._latched_output_max_us = None
            self._latched_output_spread_us = None
            self._spread_violation_started_s = None
            return MotorOutputInterlockState(True, "disarmed", False, telemetry_age_s=telemetry_age_s)
        if self._latched:
            return MotorOutputInterlockState(
                False,
                self._latched_reason or "motor_output_fault_latched",
                True,
                self._latched_output_max_us,
                self._latched_output_spread_us,
                telemetry_age_s=telemetry_age_s,
            )
        if telemetry_age_s is None or telemetry_age_s > self.config.telemetry_timeout_s:
            return self._fault(
                "motor_telemetry_stale",
                telemetry_age_s=telemetry_age_s,
            )
        source_outputs = () if motor_outputs is None else motor_outputs
        values = tuple(float(value) for value in source_outputs[: self.config.channel_count])
        if len(values) != self.config.channel_count or not np.all(np.isfinite(values)) or any(
            value <= 0.0 for value in values
        ):
            return self._fault(
                "motor_telemetry_invalid",
                telemetry_age_s=telemetry_age_s,
            )
        output_max = max(values)
        output_spread = output_max - min(values)
        reason = ""
        if output_max > self.config.max_output_us:
            reason = "motor_output_high"
        elif output_spread > self.config.max_spread_us:
            reason = "motor_output_spread_high"
        else:
            self._spread_violation_started_s = None
        if reason == "motor_output_spread_high" and self.config.violation_grace_s > 0.0:
            if timestamp is None or not np.isfinite(timestamp):
                return self._fault(
                    reason,
                    output_max,
                    output_spread,
                    telemetry_age_s,
                )
            now = float(timestamp)
            if self._spread_violation_started_s is None or now < self._spread_violation_started_s:
                self._spread_violation_started_s = now
            if now - self._spread_violation_started_s < self.config.violation_grace_s:
                return MotorOutputInterlockState(
                    True,
                    "motor_output_spread_grace",
                    False,
                    output_max,
                    output_spread,
                    telemetry_age_s,
                )
        if reason:
            return self._fault(
                reason,
                output_max,
                output_spread,
                telemetry_age_s,
            )
        return MotorOutputInterlockState(
            True,
            "ok",
            False,
            output_max,
            output_spread,
            telemetry_age_s,
        )

    def _fault(
        self,
        reason: str,
        output_max_us: float | None = None,
        output_spread_us: float | None = None,
        telemetry_age_s: float | None = None,
    ) -> MotorOutputInterlockState:
        self._latched = bool(self.config.latch_until_disarm)
        self._latched_reason = reason
        self._latched_output_max_us = output_max_us
        self._latched_output_spread_us = output_spread_us
        return MotorOutputInterlockState(
            False,
            reason,
            self._latched,
            output_max_us,
            output_spread_us,
            telemetry_age_s,
        )


@dataclass(frozen=True)
class TakeoverDurationInterlockConfig:
    enabled: bool = False
    max_duration_s: float | None = 3.0
    latch_until_disarm: bool = True
    rearm_release_s: float = 0.0
    max_takeovers_per_arm: int | None = None

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "TakeoverDurationInterlockConfig":
        raw_max_duration = values.get("max_duration_s", 3.0)
        return cls(
            enabled=bool(values.get("enabled", False)),
            max_duration_s=(
                None if raw_max_duration is None else float(raw_max_duration)
            ),
            latch_until_disarm=bool(values.get("latch_until_disarm", True)),
            rearm_release_s=float(values.get("rearm_release_s", 0.0)),
            max_takeovers_per_arm=(
                None
                if values.get("max_takeovers_per_arm") is None
                else int(values["max_takeovers_per_arm"])
            ),
        )

    def __post_init__(self) -> None:
        if self.enabled and (
            self.max_duration_s is None
            or not np.isfinite(self.max_duration_s)
            or self.max_duration_s <= 0.0
        ):
            raise ValueError("takeover duration max_duration_s must be finite and positive")
        if self.max_duration_s is not None and (
            not np.isfinite(self.max_duration_s) or self.max_duration_s <= 0.0
        ):
            raise ValueError("takeover duration max_duration_s must be null or finite and positive")
        if not np.isfinite(self.rearm_release_s) or self.rearm_release_s < 0.0:
            raise ValueError("takeover duration rearm_release_s must be finite and non-negative")
        if self.max_takeovers_per_arm is not None and self.max_takeovers_per_arm < 1:
            raise ValueError("max_takeovers_per_arm must be null or positive")


@dataclass(frozen=True)
class TakeoverDurationInterlockState:
    ok: bool
    reason: str
    latched: bool
    active_duration_s: float = 0.0
    max_duration_s: float | None = None
    remaining_s: float | None = None
    takeover_requested: bool = False
    control_active: bool = False
    release_elapsed_s: float = 0.0
    takeover_count: int = 0
    max_takeovers_per_arm: int | None = None


class TakeoverDurationInterlock:
    """Bound actual algorithm publication within one supervised takeover request."""

    def __init__(self, config: TakeoverDurationInterlockConfig):
        self.config = config
        self._latched = False
        self._active_duration_s = 0.0
        self._rearm_required = False
        self._release_started_s: float | None = None
        self._last_update_s: float | None = None
        self._last_control_active = False
        self._takeover_count = 0
        self._latched_reason = ""

    def update(
        self,
        *,
        timestamp: float,
        armed: bool,
        takeover_requested: bool | None = None,
        control_active: bool | None = None,
        takeover_active: bool | None = None,
    ) -> TakeoverDurationInterlockState:
        now = float(timestamp)
        if not np.isfinite(now):
            raise ValueError("takeover duration timestamp must be finite")
        if takeover_active is not None:
            if takeover_requested is None:
                takeover_requested = bool(takeover_active)
            if control_active is None:
                control_active = bool(takeover_active)
        requested = bool(takeover_requested)
        active = bool(control_active)
        if active and not requested:
            active = False
        if not self.config.enabled:
            self._reset(now)
            return TakeoverDurationInterlockState(
                True,
                "disabled",
                False,
                takeover_requested=requested,
                control_active=active,
                takeover_count=0,
                max_takeovers_per_arm=self.config.max_takeovers_per_arm,
            )
        if not armed:
            self._reset(now)
            return TakeoverDurationInterlockState(
                True,
                "disarmed",
                False,
                max_duration_s=self.config.max_duration_s,
                remaining_s=self.config.max_duration_s,
                takeover_requested=requested,
                control_active=False,
                takeover_count=0,
                max_takeovers_per_arm=self.config.max_takeovers_per_arm,
            )
        previous_update_s = self._last_update_s
        dt = (
            0.0
            if previous_update_s is None or now < previous_update_s
            else max(0.0, now - previous_update_s)
        )
        self._last_update_s = now
        if self._latched:
            return TakeoverDurationInterlockState(
                False,
                self._latched_reason or "takeover_duration_exceeded",
                True,
                self._active_duration_s,
                self.config.max_duration_s,
                0.0,
                requested,
                active,
                takeover_count=self._takeover_count,
                max_takeovers_per_arm=self.config.max_takeovers_per_arm,
            )
        if active and not self._last_control_active:
            maximum = self.config.max_takeovers_per_arm
            if maximum is not None and self._takeover_count >= maximum:
                self._latched = True
                self._latched_reason = "takeover_count_exceeded"
                return TakeoverDurationInterlockState(
                    False,
                    self._latched_reason,
                    True,
                    self._active_duration_s,
                    self.config.max_duration_s,
                    max(0.0, self.config.max_duration_s - self._active_duration_s),
                    requested,
                    False,
                    takeover_count=self._takeover_count,
                    max_takeovers_per_arm=maximum,
                )
            self._takeover_count += 1
        if self._rearm_required:
            if requested:
                self._release_started_s = None
                self._last_control_active = False
                return TakeoverDurationInterlockState(
                    False,
                    "takeover_release_required",
                    False,
                    self._active_duration_s,
                    self.config.max_duration_s,
                    max(0.0, self.config.max_duration_s - self._active_duration_s),
                    True,
                    False,
                )
            if self._release_started_s is None or now < self._release_started_s:
                self._release_started_s = now
            release_duration_s = max(0.0, now - self._release_started_s)
            if release_duration_s < self.config.rearm_release_s:
                self._last_control_active = False
                return TakeoverDurationInterlockState(
                    True,
                    "takeover_rearm_wait",
                    False,
                    self._active_duration_s,
                    self.config.max_duration_s,
                    max(0.0, self.config.max_duration_s - self._active_duration_s),
                    False,
                    False,
                    release_duration_s,
                )
            self._active_duration_s = 0.0
            self._rearm_required = False
            self._release_started_s = None
            self._last_control_active = False
            return TakeoverDurationInterlockState(
                True,
                "inactive",
                False,
                max_duration_s=self.config.max_duration_s,
                remaining_s=self.config.max_duration_s,
            )
        if not requested:
            if self._active_duration_s > 0.0:
                self._rearm_required = True
                self._release_started_s = now
                reason = (
                    "inactive"
                    if self.config.rearm_release_s <= 0.0
                    else "takeover_rearm_wait"
                )
                if self.config.rearm_release_s <= 0.0:
                    self._active_duration_s = 0.0
                    self._rearm_required = False
                    self._release_started_s = None
            else:
                reason = "inactive"
            self._last_control_active = False
            return TakeoverDurationInterlockState(
                True,
                reason,
                False,
                self._active_duration_s,
                max_duration_s=self.config.max_duration_s,
                remaining_s=max(
                    0.0,
                    self.config.max_duration_s - self._active_duration_s,
                ),
                release_elapsed_s=0.0,
                takeover_count=self._takeover_count,
                max_takeovers_per_arm=self.config.max_takeovers_per_arm,
            )
        if active and self._last_control_active:
            self._active_duration_s += dt
        self._last_control_active = active
        if self._active_duration_s >= self.config.max_duration_s:
            self._active_duration_s = max(
                self.config.max_duration_s,
                self._active_duration_s,
            )
            self._latched = bool(self.config.latch_until_disarm)
            if self._latched:
                self._latched_reason = "takeover_duration_exceeded"
            self._rearm_required = not self._latched
            self._release_started_s = None
            return TakeoverDurationInterlockState(
                False,
                "takeover_duration_exceeded",
                self._latched,
                self._active_duration_s,
                self.config.max_duration_s,
                0.0,
                True,
                active,
            )
        return TakeoverDurationInterlockState(
            True,
            "timing" if active else "waiting_for_control",
            False,
            self._active_duration_s,
            self.config.max_duration_s,
            max(0.0, self.config.max_duration_s - self._active_duration_s),
            True,
            active,
            takeover_count=self._takeover_count,
            max_takeovers_per_arm=self.config.max_takeovers_per_arm,
        )

    def _reset(self, timestamp: float | None = None) -> None:
        self._latched = False
        self._active_duration_s = 0.0
        self._rearm_required = False
        self._release_started_s = None
        self._last_update_s = timestamp
        self._last_control_active = False
        self._takeover_count = 0
        self._latched_reason = ""


@dataclass(frozen=True)
class SafetyInputs:
    control_requested: bool = False
    allow_control: bool = False
    target_valid: bool = False
    aux_enabled: bool = False
    telemetry_fresh: bool = False
    attitude_synced: bool = False
    motor_output_ok: bool = True
    takeover_duration_ok: bool = True
    voltage_ok: bool = True
    watchdog_ok: bool = False
    armed: bool = False
    override_available: bool = False
    override_active: bool = False
    prefill_ready: bool = True
    msp_response_fresh: bool = True
    physical_rc_fresh: bool = False
    snapshot_approved: bool = False
    config_conflict_free: bool = False


@dataclass(frozen=True)
class SafetyDecision:
    state: SafetyState
    command_active: bool
    reason: str


class BetaflightSafetyStateMachine:
    def __init__(self) -> None:
        self.state = SafetyState.DISABLED

    def update(self, inputs: SafetyInputs) -> SafetyDecision:
        if not inputs.control_requested:
            return self._set(SafetyState.LOG_ONLY, False, "log_only")
        if not inputs.allow_control:
            return self._set(SafetyState.DISABLED, False, "control_not_allowed")
        if not inputs.snapshot_approved:
            return self._set(SafetyState.DISABLED, False, "snapshot_not_approved")
        if not inputs.config_conflict_free:
            return self._set(SafetyState.DISABLED, False, "config_conflict")
        if not inputs.override_available:
            return self._set(SafetyState.DISABLED, False, "msp_override_unavailable")
        if not inputs.override_active:
            return self._set(SafetyState.READY, False, "msp_override_inactive")
        if not inputs.prefill_ready:
            return self._set(SafetyState.FAILSAFE, False, "msp_prefill_not_ready")
        if not inputs.msp_response_fresh:
            return self._set(SafetyState.FAILSAFE, False, "msp_set_raw_rc_ack_stale")
        if not inputs.armed:
            return self._set(SafetyState.READY, False, "not_armed")
        if not inputs.physical_rc_fresh:
            return self._set(SafetyState.FAILSAFE, False, "physical_rc_stale")
        if not inputs.telemetry_fresh:
            return self._set(SafetyState.FAILSAFE, False, "telemetry_stale")
        if not inputs.attitude_synced:
            return self._set(SafetyState.FAILSAFE, False, "attitude_not_synced")
        if not inputs.motor_output_ok:
            return self._set(SafetyState.FAILSAFE, False, "motor_output_interlock")
        if not inputs.takeover_duration_ok:
            return self._set(SafetyState.FAILSAFE, False, "takeover_duration_interlock")
        if not inputs.voltage_ok:
            return self._set(SafetyState.FAILSAFE, False, "low_voltage")
        if not inputs.watchdog_ok:
            return self._set(SafetyState.FAILSAFE, False, "watchdog_expired")
        if not inputs.aux_enabled:
            return self._set(SafetyState.READY, False, "aux_disabled")
        if not inputs.target_valid:
            return self._set(SafetyState.DEGRADED, False, "target_invalid")
        return self._set(SafetyState.ACTIVE, True, "active")

    def _set(self, state: SafetyState, command_active: bool, reason: str) -> SafetyDecision:
        self.state = state
        return SafetyDecision(state=state, command_active=command_active, reason=reason)


class CommandWatchdog:
    def __init__(self, timeout_s: float):
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = float(timeout_s)
        self._last_kick_s: float | None = None

    def kick(self, timestamp: float) -> None:
        self._last_kick_s = float(timestamp)

    def fresh(self, timestamp: float) -> bool:
        if self._last_kick_s is None:
            return False
        return float(timestamp) - self._last_kick_s <= self.timeout_s

    def age_s(self, timestamp: float) -> float | None:
        if self._last_kick_s is None:
            return None
        return max(0.0, float(timestamp) - self._last_kick_s)


def guidance_eval_to_setpoint(
    guidance: GuidanceEval | None,
    *,
    R_IB: Sequence[Sequence[float]] | None,
    rate_gain_matrix: Sequence[Sequence[float]],
    hover_thrust: float,
    yaw_rate_deg_s: float = 0.0,
    mapping_type: str = "direct_rate_matrix",
    accel_tilt_rate: Mapping[str, object] | AccelerationTiltRateConfig | None = None,
    thrust_model: VoltageThrottleThrustModel | None = None,
    battery_voltage_v: float | None = None,
) -> GuidanceSetpoint:
    mapping = str(mapping_type).strip().lower()
    if guidance is None:
        return GuidanceSetpoint(
            timestamp=0.0,
            valid=False,
            reject_reason="guidance_missing",
            source="guidance_eval",
            mapping_type=mapping,
        )
    if not guidance.valid:
        return GuidanceSetpoint(
            timestamp=float(guidance.timestamp),
            valid=False,
            reject_reason=guidance.reject_reason or "guidance_invalid",
            source="guidance_eval",
            mapping_type=mapping,
        )
    if mapping not in {"direct_rate_matrix", "accel_tilt_rate"}:
        return GuidanceSetpoint(
            timestamp=float(guidance.timestamp),
            valid=False,
            reject_reason="unsupported_guidance_command_mapping",
            source="guidance_eval",
            mapping_type=mapping,
        )
    if R_IB is None:
        return GuidanceSetpoint(
            timestamp=float(guidance.timestamp),
            valid=False,
            reject_reason="guidance_attitude_missing",
            source="guidance_eval",
            mapping_type=mapping,
        )
    try:
        rotation = validated_rotation_matrix(np.asarray(R_IB, dtype=float), name="guidance R_IB")
    except ValueError:
        return GuidanceSetpoint(
            timestamp=float(guidance.timestamp),
            valid=False,
            reject_reason="invalid_guidance_frame_transform",
            source="guidance_eval",
            mapping_type=mapping,
        )
    guidance_vector = np.asarray(guidance.g_eval, dtype=float)
    if guidance_vector.shape != (3,) or not np.all(np.isfinite(guidance_vector)):
        return GuidanceSetpoint(
            timestamp=float(guidance.timestamp),
            valid=False,
            reject_reason="invalid_guidance_vector",
            source="guidance_eval",
            mapping_type=mapping,
        )

    if mapping == "accel_tilt_rate":
        if isinstance(accel_tilt_rate, AccelerationTiltRateConfig):
            config = accel_tilt_rate
        else:
            config = AccelerationTiltRateConfig.from_mapping(accel_tilt_rate or {})
        return _acceleration_tilt_rate_setpoint(
            guidance,
            rotation,
            config,
            hover_thrust=float(hover_thrust),
            yaw_rate_deg_s=float(yaw_rate_deg_s),
            thrust_model=thrust_model,
            battery_voltage_v=battery_voltage_v,
        )

    gain = np.asarray(rate_gain_matrix, dtype=float)
    if gain.shape != (3, 3) or not np.all(np.isfinite(gain)):
        return GuidanceSetpoint(
            timestamp=float(guidance.timestamp),
            valid=False,
            reject_reason="invalid_guidance_rate_mapping",
            source="guidance_eval",
            mapping_type=mapping,
        )
    vector_body_frd = rotation.T @ guidance_vector
    rates = gain @ vector_body_frd
    return GuidanceSetpoint(
        timestamp=float(guidance.timestamp),
        roll_rate_deg_s=float(rates[0]),
        pitch_rate_deg_s=float(rates[1]),
        yaw_rate_deg_s=float(rates[2] + yaw_rate_deg_s),
        thrust=float(hover_thrust),
        valid=True,
        source="guidance_eval",
        mapping_type=mapping,
    )


def _acceleration_tilt_rate_setpoint(
    guidance: GuidanceEval,
    R_IB: np.ndarray,
    config: AccelerationTiltRateConfig,
    *,
    hover_thrust: float,
    yaw_rate_deg_s: float,
    thrust_model: VoltageThrottleThrustModel | None,
    battery_voltage_v: float | None,
) -> GuidanceSetpoint:
    roll_rad, pitch_rad, yaw_rad = _rotation_matrix_to_euler_frd(R_IB)
    cos_yaw = float(np.cos(yaw_rad))
    sin_yaw = float(np.sin(yaw_rad))
    yaw_rotation = np.array(
        [[cos_yaw, -sin_yaw, 0.0], [sin_yaw, cos_yaw, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )
    accel_yaw_frd = yaw_rotation.T @ np.asarray(guidance.g_eval, dtype=float)
    vertical_force = max(
        config.min_vertical_specific_force_mps2,
        config.gravity_mps2 - float(accel_yaw_frd[2]),
    )
    desired_roll_deg = float(
        np.clip(
            np.rad2deg(np.arctan2(float(accel_yaw_frd[1]), vertical_force)),
            -config.max_roll_tilt_deg,
            config.max_roll_tilt_deg,
        )
    )
    desired_pitch_deg = float(
        np.clip(
            np.rad2deg(np.arctan2(-float(accel_yaw_frd[0]), vertical_force)),
            -config.max_pitch_tilt_deg,
            config.max_pitch_tilt_deg,
        )
    )
    current_roll_deg = float(np.rad2deg(roll_rad))
    current_pitch_deg = float(np.rad2deg(pitch_rad))
    roll_error_deg = desired_roll_deg - current_roll_deg
    pitch_error_deg = desired_pitch_deg - current_pitch_deg
    roll_rate = float(
        np.clip(
            config.roll_rate_sign * config.roll_attitude_kp_s_inv * roll_error_deg,
            -config.max_roll_rate_deg_s,
            config.max_roll_rate_deg_s,
        )
    )
    pitch_rate = float(
        np.clip(
            config.pitch_rate_sign * config.pitch_attitude_kp_s_inv * pitch_error_deg,
            -config.max_pitch_rate_deg_s,
            config.max_pitch_rate_deg_s,
        )
    )
    try:
        (
            thrust,
            thrust_model_name,
            required_specific_force,
            load_factor_raw,
            thrust_raw,
            throttle_target_us,
            thrust_model_voltage_v,
            thrust_limited,
        ) = _acceleration_thrust_feedforward(
            vertical_force_mps2=vertical_force,
            roll_rad=roll_rad,
            pitch_rad=pitch_rad,
            hover_thrust=hover_thrust,
            gravity_mps2=config.gravity_mps2,
            config=config.thrust_feedforward,
            thrust_model=thrust_model,
            battery_voltage_v=battery_voltage_v,
        )
    except ValueError as exc:
        return GuidanceSetpoint(
            timestamp=float(guidance.timestamp),
            valid=False,
            reject_reason=str(exc),
            source="guidance_eval",
            mapping_type="accel_tilt_rate",
            desired_roll_angle_deg=desired_roll_deg,
            desired_pitch_angle_deg=desired_pitch_deg,
            current_roll_angle_deg=current_roll_deg,
            current_pitch_angle_deg=current_pitch_deg,
            roll_attitude_error_deg=roll_error_deg,
            pitch_attitude_error_deg=pitch_error_deg,
            thrust_model=config.thrust_feedforward.model,
            thrust_model_voltage_v=battery_voltage_v,
        )
    return GuidanceSetpoint(
        timestamp=float(guidance.timestamp),
        roll_rate_deg_s=roll_rate,
        pitch_rate_deg_s=pitch_rate,
        yaw_rate_deg_s=float(yaw_rate_deg_s),
        thrust=thrust,
        valid=True,
        source="guidance_eval",
        mapping_type="accel_tilt_rate",
        desired_roll_angle_deg=desired_roll_deg,
        desired_pitch_angle_deg=desired_pitch_deg,
        current_roll_angle_deg=current_roll_deg,
        current_pitch_angle_deg=current_pitch_deg,
        roll_attitude_error_deg=roll_error_deg,
        pitch_attitude_error_deg=pitch_error_deg,
        thrust_model=thrust_model_name,
        thrust_required_specific_force_mps2=required_specific_force,
        thrust_load_factor_raw_g=load_factor_raw,
        thrust_command_raw=thrust_raw,
        thrust_command_limited=thrust_limited,
        throttle_target_us=throttle_target_us,
        thrust_model_voltage_v=thrust_model_voltage_v,
    )


def _acceleration_thrust_feedforward(
    *,
    vertical_force_mps2: float,
    roll_rad: float,
    pitch_rad: float,
    hover_thrust: float,
    gravity_mps2: float,
    config: ThrustFeedforwardConfig,
    thrust_model: VoltageThrottleThrustModel | None,
    battery_voltage_v: float | None,
) -> tuple[float, str, float, float, float, float | None, float | None, bool]:
    if not config.enabled:
        hover = float(hover_thrust)
        return (
            hover,
            "fixed_hover",
            float(vertical_force_mps2),
            1.0,
            hover,
            None,
            None,
            False,
        )

    tilt_cosine = max(
        config.minimum_tilt_cosine,
        float(np.cos(roll_rad) * np.cos(pitch_rad)),
    )
    required_specific_force = float(vertical_force_mps2) / tilt_cosine
    load_factor = required_specific_force / float(gravity_mps2)
    if config.model == "voltage_throttle_lut":
        if thrust_model is None:
            raise ValueError("thrust_lut_unavailable")
        if thrust_model.calibration_id != config.calibration_id:
            raise ValueError("thrust_lut_calibration_id_mismatch")
        if battery_voltage_v is None or not np.isfinite(battery_voltage_v):
            raise ValueError("thrust_lut_voltage_missing")
        if not thrust_model.covers_voltage(float(battery_voltage_v)):
            raise ValueError("thrust_lut_voltage_outside_coverage")
        lookup = thrust_model.throttle_for_specific_force(
            float(battery_voltage_v),
            required_specific_force,
        )
        return (
            float(hover_thrust),
            config.model,
            required_specific_force,
            load_factor,
            float(hover_thrust),
            lookup.throttle_us,
            lookup.voltage_v,
            lookup.limited,
        )

    if load_factor <= config.hover_load_factor_g:
        raw_thrust = float(hover_thrust) * (
            load_factor / config.hover_load_factor_g
        )
    else:
        raw_thrust = float(hover_thrust) + (
            (load_factor - config.hover_load_factor_g)
            / (config.max_load_factor_g - config.hover_load_factor_g)
            * (1.0 - float(hover_thrust))
        )
    thrust = float(np.clip(raw_thrust, 0.0, 1.0))
    return (
        thrust,
        config.model,
        required_specific_force,
        load_factor,
        raw_thrust,
        None,
        None,
        not np.isclose(thrust, raw_thrust),
    )


def _rotation_matrix_to_euler_frd(R_IB: np.ndarray) -> tuple[float, float, float]:
    pitch = float(np.arcsin(np.clip(-float(R_IB[2, 0]), -1.0, 1.0)))
    roll = float(np.arctan2(float(R_IB[2, 1]), float(R_IB[2, 2])))
    yaw = float(np.arctan2(float(R_IB[1, 0]), float(R_IB[0, 0])))
    return roll, pitch, yaw


def inertial_vector_to_body_frd(
    vector_inertial_ned: Sequence[float],
    R_IB: Sequence[Sequence[float]],
) -> np.ndarray:
    """Rotate an inertial NED vector into the Betaflight FRD body frame."""

    vector = np.asarray(vector_inertial_ned, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError("inertial guidance vector must be finite with shape (3,)")
    rotation = validated_rotation_matrix(np.asarray(R_IB, dtype=float), name="guidance R_IB")
    return rotation.T @ vector


def aux_range_enabled(channels: Sequence[int], *, channel_index: int, min_us: int, max_us: int) -> bool:
    index = int(channel_index) - 1
    if index < 0 or index >= len(channels):
        return False
    value = int(channels[index])
    return int(min_us) <= value <= int(max_us)
