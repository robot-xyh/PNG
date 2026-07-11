from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Mapping, Protocol, Sequence

import numpy as np

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


@dataclass(frozen=True)
class RcCommand:
    timestamp: float
    channels: tuple[int, ...]
    active: bool
    reason: str = ""
    raw_channels: tuple[int, ...] = ()
    clipped_flags: tuple[int, ...] = ()
    slew_limited_flags: tuple[int, ...] = ()

    def __post_init__(self) -> None:
        if not self.channels:
            raise ValueError("channels must not be empty")
        for value in self.channels:
            if int(value) != value:
                raise ValueError("RC channel values must be integers")
        for name in ("raw_channels", "clipped_flags", "slew_limited_flags"):
            values = getattr(self, name)
            if values and len(values) != len(self.channels):
                raise ValueError(f"{name} length must match channels")


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
        for role, rate, limit in (
            ("A", setpoint.roll_rate_deg_s, self.config.roll_rate_limit_deg_s),
            ("E", setpoint.pitch_rate_deg_s, self.config.pitch_rate_limit_deg_s),
            ("R", setpoint.yaw_rate_deg_s, self.config.yaw_rate_limit_deg_s),
        ):
            index = self._role_index(role)
            raw, clipped, was_clipped = self._rate_to_us_with_clip(rate, limit)
            raw_channels[index] = raw
            channels[index] = clipped
            clipped_flags[index] = int(was_clipped)
        throttle_index = self._role_index("T")
        raw, clipped, was_clipped = self._thrust_to_us_with_clip(setpoint.thrust)
        raw_channels[throttle_index] = raw
        channels[throttle_index] = clipped
        clipped_flags[throttle_index] = int(was_clipped)
        channels_tuple, slew_flags = self._apply_slew(tuple(channels), timestamp)
        command = RcCommand(
            timestamp=timestamp,
            channels=channels_tuple,
            active=True,
            reason=setpoint.source,
            raw_channels=tuple(raw_channels),
            clipped_flags=tuple(clipped_flags),
            slew_limited_flags=slew_flags,
        )
        self._previous = command
        return command

    def _map_neutral_with_slew(self, timestamp: float, reason: str) -> RcCommand:
        channels, raw_channels, clipped_flags = self._base_channels_with_flags()
        throttle_index = self._role_index("T")
        raw_channels[throttle_index] = int(round(float(self.config.neutral_throttle_us)))
        channels[throttle_index] = self._clip_us(self.config.neutral_throttle_us)
        clipped_flags[throttle_index] = int(raw_channels[throttle_index] != channels[throttle_index])
        channels_tuple, slew_flags = self._apply_slew(tuple(channels), timestamp)
        command = RcCommand(
            timestamp=timestamp,
            channels=channels_tuple,
            active=False,
            reason=reason,
            raw_channels=tuple(raw_channels),
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

    def _rate_to_us(self, rate_deg_s: float, limit_deg_s: float) -> int:
        _raw, clipped, _was_clipped = self._rate_to_us_with_clip(rate_deg_s, limit_deg_s)
        return clipped

    def _rate_to_us_with_clip(self, rate_deg_s: float, limit_deg_s: float) -> tuple[int, int, bool]:
        limit = max(1.0e-6, float(limit_deg_s))
        normalized_raw = float(rate_deg_s) / limit
        span = 0.5 * float(self.config.max_us - self.config.min_us)
        raw = int(round(float(self.config.mid_us) + normalized_raw * span))
        clipped = self._clip_us(raw)
        return raw, clipped, raw != clipped

    def _thrust_to_us(self, thrust: float) -> int:
        _raw, clipped, _was_clipped = self._thrust_to_us_with_clip(thrust)
        return clipped

    def _thrust_to_us_with_clip(self, thrust: float) -> tuple[int, int, bool]:
        raw_value = float(thrust)
        if raw_value <= self.config.thrust_hover:
            denom = max(1.0e-9, self.config.thrust_hover - self.config.thrust_min)
            alpha = (raw_value - self.config.thrust_min) / denom
            raw_us = self.config.throttle_min_us + alpha * (self.config.throttle_hover_us - self.config.throttle_min_us)
        else:
            denom = max(1.0e-9, self.config.thrust_max - self.config.thrust_hover)
            alpha = (raw_value - self.config.thrust_hover) / denom
            raw_us = self.config.throttle_hover_us + alpha * (self.config.throttle_max_us - self.config.throttle_hover_us)
        raw = int(round(raw_us))
        clipped = self._clip_us(raw)
        return raw, clipped, raw != clipped

    def _clip_us(self, value: float) -> int:
        return int(np.clip(round(float(value)), self.config.min_us, self.config.max_us))

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
class SafetyInputs:
    control_requested: bool = False
    allow_control: bool = False
    target_valid: bool = False
    aux_enabled: bool = False
    telemetry_fresh: bool = False
    attitude_synced: bool = False
    voltage_ok: bool = True
    watchdog_ok: bool = False


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
        if not inputs.telemetry_fresh:
            return self._set(SafetyState.FAILSAFE, False, "telemetry_stale")
        if not inputs.attitude_synced:
            return self._set(SafetyState.FAILSAFE, False, "attitude_not_synced")
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
    vector = np.asarray(guidance.g_eval, dtype=float)
    if gain.shape != (3, 3) or vector.shape != (3,) or not np.all(np.isfinite(gain)) or not np.all(np.isfinite(vector)):
        return GuidanceSetpoint(
            timestamp=float(guidance.timestamp),
            valid=False,
            reject_reason="invalid_guidance_rate_mapping",
            source="guidance_eval",
        )
    rates = gain @ vector
    return GuidanceSetpoint(
        timestamp=float(guidance.timestamp),
        roll_rate_deg_s=float(rates[0]),
        pitch_rate_deg_s=float(rates[1]),
        yaw_rate_deg_s=float(rates[2] + yaw_rate_deg_s),
        thrust=float(hover_thrust),
        valid=True,
        source="guidance_eval",
    )


def aux_range_enabled(channels: Sequence[int], *, channel_index: int, min_us: int, max_us: int) -> bool:
    index = int(channel_index) - 1
    if index < 0 or index >= len(channels):
        return False
    value = int(channels[index])
    return int(min_us) <= value <= int(max_us)
