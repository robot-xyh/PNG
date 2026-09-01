from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any

from .betaflight_msp import AltitudeTelemetry, BetaflightTelemetry, RawGpsTelemetry


EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class KinematicEstimatorConfig:
    gps_timeout_s: float = 0.5
    altitude_timeout_s: float = 0.5
    velocity_filter_tau_s: float = 0.25
    minimum_fix: int = 1
    minimum_satellites: int = 6
    origin_lock_samples: int = 3
    origin_stability_radius_m: float = 5.0

    @classmethod
    def from_mapping(cls, values: dict[str, Any]) -> "KinematicEstimatorConfig":
        return cls(
            gps_timeout_s=float(values.get("gps_timeout_s", 0.5)),
            altitude_timeout_s=float(values.get("altitude_timeout_s", 0.5)),
            velocity_filter_tau_s=float(values.get("velocity_filter_tau_s", 0.25)),
            minimum_fix=int(values.get("minimum_fix", 1)),
            minimum_satellites=int(values.get("minimum_satellites", 6)),
            origin_lock_samples=int(values.get("origin_lock_samples", 3)),
            origin_stability_radius_m=float(values.get("origin_stability_radius_m", 5.0)),
        )

    def __post_init__(self) -> None:
        if self.gps_timeout_s <= 0.0 or self.altitude_timeout_s <= 0.0:
            raise ValueError("kinematic sample timeouts must be positive")
        if self.velocity_filter_tau_s < 0.0:
            raise ValueError("velocity_filter_tau_s must be non-negative")
        if self.minimum_fix < 1 or self.minimum_satellites < 0:
            raise ValueError("GPS fix requirements are invalid")
        if self.origin_lock_samples < 1 or self.origin_stability_radius_m <= 0.0:
            raise ValueError("origin lock requirements are invalid")


@dataclass(frozen=True)
class VehicleKinematicState:
    timestamp_s: float
    valid: bool
    reason: str
    source: str
    horizontal_valid: bool
    vertical_valid: bool
    position_ned_m: tuple[float | None, float | None, float | None]
    velocity_ned_raw_m_s: tuple[float | None, float | None, float | None]
    velocity_ned_filtered_m_s: tuple[float | None, float | None, float | None]
    latitude_deg: float | None
    longitude_deg: float | None
    gps_altitude_m: float | None
    baro_altitude_m: float | None
    ground_speed_m_s: float | None
    ground_course_deg: float | None
    vertical_speed_up_m_s: float | None
    fix: int | None
    satellites: int | None
    hdop: int | None
    gps_age_s: float | None
    altitude_age_s: float | None
    origin_locked: bool
    origin_latitude_deg: float | None
    origin_longitude_deg: float | None
    origin_baro_altitude_m: float | None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


class BetaflightKinematicEstimator:
    """Convert read-only MSP GPS/barometer samples into a fail-closed NED state."""

    def __init__(self, config: KinematicEstimatorConfig | None = None):
        self.config = config or KinematicEstimatorConfig()
        self._origin_latitude_deg: float | None = None
        self._origin_longitude_deg: float | None = None
        self._origin_baro_altitude_m: float | None = None
        self._origin_candidates: list[tuple[float, float]] = []
        self._last_gps_timestamp_s: float | None = None
        self._last_altitude_timestamp_s: float | None = None
        self._velocity_filtered: tuple[float, float, float] | None = None
        self._last_filter_timestamp_s: float | None = None

    def update(
        self,
        telemetry: BetaflightTelemetry | None,
        timestamp_s: float,
    ) -> VehicleKinematicState:
        now = float(timestamp_s)
        gps = None if telemetry is None else telemetry.raw_gps
        altitude = None if telemetry is None else telemetry.altitude
        gps_timestamp = None if telemetry is None else telemetry.raw_gps_timestamp_s
        altitude_timestamp = None if telemetry is None else telemetry.altitude_timestamp_s
        if gps is not None and gps_timestamp is None:
            gps_timestamp = telemetry.timestamp
        if altitude is not None and altitude_timestamp is None:
            altitude_timestamp = telemetry.timestamp

        gps_age = _age(now, gps_timestamp)
        altitude_age = _age(now, altitude_timestamp)
        gps_sample_valid = self._gps_sample_valid(gps)
        gps_fresh = bool(
            gps_sample_valid
            and gps_age is not None
            and gps_age <= self.config.gps_timeout_s
        )
        altitude_sample_valid = bool(
            altitude is not None
            and _finite(altitude.altitude_m, altitude.vertical_speed_m_s)
        )
        altitude_fresh = bool(
            altitude_sample_valid
            and altitude_age is not None
            and altitude_age <= self.config.altitude_timeout_s
        )

        if gps_fresh and gps_timestamp != self._last_gps_timestamp_s:
            assert gps is not None
            self._observe_origin(gps)
            self._last_gps_timestamp_s = gps_timestamp
        if altitude_fresh and altitude_timestamp != self._last_altitude_timestamp_s:
            assert altitude is not None
            if self._origin_baro_altitude_m is None:
                self._origin_baro_altitude_m = float(altitude.altitude_m)
            self._last_altitude_timestamp_s = altitude_timestamp

        north: float | None = None
        east: float | None = None
        down: float | None = None
        velocity_raw: tuple[float | None, float | None, float | None] = (None, None, None)
        horizontal_valid = bool(
            gps_fresh
            and self._origin_latitude_deg is not None
            and self._origin_longitude_deg is not None
        )
        vertical_valid = bool(altitude_fresh and self._origin_baro_altitude_m is not None)
        if horizontal_valid:
            assert gps is not None
            north, east = _wgs84_to_local_ne(
                gps.latitude_deg,
                gps.longitude_deg,
                self._origin_latitude_deg,
                self._origin_longitude_deg,
            )
            course_rad = math.radians(gps.ground_course_deg)
            velocity_raw = (
                gps.ground_speed_m_s * math.cos(course_rad),
                gps.ground_speed_m_s * math.sin(course_rad),
                velocity_raw[2],
            )
        if vertical_valid:
            assert altitude is not None and self._origin_baro_altitude_m is not None
            down = -(altitude.altitude_m - self._origin_baro_altitude_m)
            velocity_raw = (velocity_raw[0], velocity_raw[1], -altitude.vertical_speed_m_s)

        valid = bool(horizontal_valid and vertical_valid and _optional_finite(*velocity_raw))
        velocity_filtered = self._filter_velocity(velocity_raw, now) if valid else (None, None, None)
        reason = self._reason(
            gps,
            gps_sample_valid,
            gps_fresh,
            altitude_sample_valid,
            altitude_fresh,
            horizontal_valid,
            vertical_valid,
            valid,
        )
        return VehicleKinematicState(
            timestamp_s=now,
            valid=valid,
            reason=reason,
            source="msp_raw_gps+msp_altitude",
            horizontal_valid=horizontal_valid,
            vertical_valid=vertical_valid,
            position_ned_m=(north, east, down),
            velocity_ned_raw_m_s=velocity_raw,
            velocity_ned_filtered_m_s=velocity_filtered,
            latitude_deg=None if gps is None else gps.latitude_deg,
            longitude_deg=None if gps is None else gps.longitude_deg,
            gps_altitude_m=None if gps is None else gps.altitude_m,
            baro_altitude_m=None if altitude is None else altitude.altitude_m,
            ground_speed_m_s=None if gps is None else gps.ground_speed_m_s,
            ground_course_deg=None if gps is None else gps.ground_course_deg,
            vertical_speed_up_m_s=None if altitude is None else altitude.vertical_speed_m_s,
            fix=None if gps is None else gps.fix,
            satellites=None if gps is None else gps.satellites,
            hdop=None if gps is None else gps.hdop,
            gps_age_s=gps_age,
            altitude_age_s=altitude_age,
            origin_locked=self._origin_latitude_deg is not None,
            origin_latitude_deg=self._origin_latitude_deg,
            origin_longitude_deg=self._origin_longitude_deg,
            origin_baro_altitude_m=self._origin_baro_altitude_m,
        )

    def _gps_sample_valid(self, gps: RawGpsTelemetry | None) -> bool:
        return bool(
            gps is not None
            and gps.fix >= self.config.minimum_fix
            and gps.satellites >= self.config.minimum_satellites
            and _finite(
                gps.latitude_deg,
                gps.longitude_deg,
                gps.altitude_m,
                gps.ground_speed_m_s,
                gps.ground_course_deg,
            )
            and -90.0 <= gps.latitude_deg <= 90.0
            and -180.0 <= gps.longitude_deg <= 180.0
            and gps.ground_speed_m_s >= 0.0
        )

    def _observe_origin(self, gps: RawGpsTelemetry) -> None:
        if self._origin_latitude_deg is not None:
            return
        candidate = (float(gps.latitude_deg), float(gps.longitude_deg))
        if self._origin_candidates:
            distance = _horizontal_distance_m(candidate, self._origin_candidates[-1])
            if distance > self.config.origin_stability_radius_m:
                self._origin_candidates.clear()
        self._origin_candidates.append(candidate)
        if len(self._origin_candidates) < self.config.origin_lock_samples:
            return
        selected = self._origin_candidates[-self.config.origin_lock_samples :]
        self._origin_latitude_deg = sum(value[0] for value in selected) / len(selected)
        self._origin_longitude_deg = sum(value[1] for value in selected) / len(selected)

    def _filter_velocity(
        self,
        raw: tuple[float | None, float | None, float | None],
        timestamp_s: float,
    ) -> tuple[float, float, float]:
        values = tuple(float(value) for value in raw if value is not None)
        if len(values) != 3:
            raise ValueError("valid kinematic velocity must have three axes")
        sample = (values[0], values[1], values[2])
        if self._velocity_filtered is None or self._last_filter_timestamp_s is None:
            filtered = sample
        else:
            dt = max(0.0, timestamp_s - self._last_filter_timestamp_s)
            tau = self.config.velocity_filter_tau_s
            alpha = 1.0 if tau <= 0.0 else 1.0 - math.exp(-dt / tau)
            filtered = tuple(
                previous + alpha * (current - previous)
                for previous, current in zip(self._velocity_filtered, sample)
            )
        self._velocity_filtered = filtered
        self._last_filter_timestamp_s = timestamp_s
        return filtered

    def _reason(
        self,
        gps: RawGpsTelemetry | None,
        gps_sample_valid: bool,
        gps_fresh: bool,
        altitude_sample_valid: bool,
        altitude_fresh: bool,
        horizontal_valid: bool,
        vertical_valid: bool,
        valid: bool,
    ) -> str:
        if valid:
            return "valid"
        if gps is None:
            return "gps_missing"
        if not gps_sample_valid:
            return "gps_fix_invalid"
        if not gps_fresh:
            return "gps_stale"
        if not horizontal_valid:
            return "origin_pending"
        if not altitude_sample_valid:
            return "altitude_missing_or_invalid"
        if not altitude_fresh:
            return "altitude_stale"
        if not vertical_valid:
            return "altitude_origin_pending"
        return "nonfinite_state"


def _age(now: float, timestamp: float | None) -> float | None:
    return None if timestamp is None else max(0.0, now - float(timestamp))


def _finite(*values: float) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _optional_finite(*values: float | None) -> bool:
    return all(value is not None and math.isfinite(float(value)) for value in values)


def _wgs84_to_local_ne(
    latitude_deg: float,
    longitude_deg: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
) -> tuple[float, float]:
    d_lat = math.radians(latitude_deg - origin_latitude_deg)
    d_lon = math.radians(longitude_deg - origin_longitude_deg)
    mean_lat = math.radians(0.5 * (latitude_deg + origin_latitude_deg))
    return EARTH_RADIUS_M * d_lat, EARTH_RADIUS_M * math.cos(mean_lat) * d_lon


def _horizontal_distance_m(
    first: tuple[float, float],
    second: tuple[float, float],
) -> float:
    north, east = _wgs84_to_local_ne(first[0], first[1], second[0], second[1])
    return math.hypot(north, east)
