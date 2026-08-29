from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Mapping, Protocol, Sequence

import numpy as np

from .geometry import validated_rotation_matrix
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
        raw, clipped, was_clipped = self._thrust_to_us_with_clip(setpoint.thrust)
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
    telemetry_timeout_s: float = 0.75
    latch_until_disarm: bool = True

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "MotorOutputInterlockConfig":
        return cls(
            enabled=bool(values.get("enabled", False)),
            channel_count=int(values.get("channel_count", 4)),
            max_output_us=int(values.get("max_output_us", 1200)),
            max_spread_us=int(values.get("max_spread_us", 150)),
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

    def update(
        self,
        *,
        armed: bool,
        motor_outputs: Sequence[float] | None,
        telemetry_age_s: float | None,
    ) -> MotorOutputInterlockState:
        if not self.config.enabled:
            return MotorOutputInterlockState(True, "disabled", False, telemetry_age_s=telemetry_age_s)
        if not armed:
            self._latched = False
            self._latched_reason = ""
            self._latched_output_max_us = None
            self._latched_output_spread_us = None
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
class SafetyInputs:
    control_requested: bool = False
    allow_control: bool = False
    target_valid: bool = False
    aux_enabled: bool = False
    telemetry_fresh: bool = False
    attitude_synced: bool = False
    motor_output_ok: bool = True
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
) -> GuidanceSetpoint:
    if guidance is None:
        return GuidanceSetpoint(timestamp=0.0, valid=False, reject_reason="guidance_missing", source="guidance_eval")
    if not guidance.valid:
        return GuidanceSetpoint(
            timestamp=float(guidance.timestamp),
            valid=False,
            reject_reason=guidance.reject_reason or "guidance_invalid",
            source="guidance_eval",
        )
    gain = np.asarray(rate_gain_matrix, dtype=float)
    if gain.shape != (3, 3) or not np.all(np.isfinite(gain)):
        return GuidanceSetpoint(
            timestamp=float(guidance.timestamp),
            valid=False,
            reject_reason="invalid_guidance_rate_mapping",
            source="guidance_eval",
        )
    if R_IB is None:
        return GuidanceSetpoint(
            timestamp=float(guidance.timestamp),
            valid=False,
            reject_reason="guidance_attitude_missing",
            source="guidance_eval",
        )
    try:
        vector_body_frd = inertial_vector_to_body_frd(guidance.g_eval, R_IB)
    except ValueError:
        return GuidanceSetpoint(
            timestamp=float(guidance.timestamp),
            valid=False,
            reject_reason="invalid_guidance_frame_transform",
            source="guidance_eval",
        )
    rates = gain @ vector_body_frd
    return GuidanceSetpoint(
        timestamp=float(guidance.timestamp),
        roll_rate_deg_s=float(rates[0]),
        pitch_rate_deg_s=float(rates[1]),
        yaw_rate_deg_s=float(rates[2] + yaw_rate_deg_s),
        thrust=float(hover_thrust),
        valid=True,
        source="guidance_eval",
    )


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
