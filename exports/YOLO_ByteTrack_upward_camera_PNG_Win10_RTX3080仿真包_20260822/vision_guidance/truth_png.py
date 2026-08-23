from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class TruthPNGResult:
    range_m: float
    closing_speed: float
    los: np.ndarray
    omega_los: np.ndarray
    lambda_dot: np.ndarray
    acceleration: np.ndarray
    valid: bool
    reject_reason: Optional[str] = None


def _as_vector3(value: np.ndarray | tuple[float, float, float] | list[float]) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"expected 3-vector, got shape {vector.shape}")
    return vector


def _clip_norm(vector: np.ndarray, max_norm: Optional[float]) -> np.ndarray:
    if max_norm is None:
        return vector
    if max_norm < 0.0:
        raise ValueError("max_norm must be non-negative")
    norm = float(np.linalg.norm(vector))
    if norm <= max_norm or norm == 0.0:
        return vector
    return vector * (max_norm / norm)


def compute_truth_png(
    relative_position: np.ndarray | tuple[float, float, float] | list[float],
    relative_velocity: np.ndarray | tuple[float, float, float] | list[float],
    navigation_constant: float = 3.0,
    max_accel: Optional[float] = None,
    min_range: float = 1.0e-3,
    min_closing_speed: float = 1.0e-3,
) -> TruthPNGResult:
    """Compute classic 3D proportional navigation from truth relative state.

    The relative state is target minus interceptor in one inertial frame. AirSim
    NED coordinates can be used directly as long as both vehicles share them.
    """

    if navigation_constant <= 0.0:
        raise ValueError("navigation_constant must be positive")
    if min_range <= 0.0:
        raise ValueError("min_range must be positive")

    r = _as_vector3(relative_position)
    v_rel = _as_vector3(relative_velocity)
    range_m = float(np.linalg.norm(r))
    zero = np.zeros(3)
    if range_m < min_range:
        return TruthPNGResult(range_m, 0.0, zero, zero, zero, zero, False, "range_too_small")

    los = r / range_m
    closing_speed = -float(np.dot(v_rel, los))
    omega_los = np.cross(r, v_rel) / max(range_m * range_m, min_range * min_range)
    lambda_dot = np.cross(omega_los, los)

    if closing_speed <= min_closing_speed:
        return TruthPNGResult(
            range_m,
            closing_speed,
            los,
            omega_los,
            lambda_dot,
            zero,
            False,
            "not_closing",
        )

    acceleration = navigation_constant * closing_speed * lambda_dot
    acceleration = _clip_norm(acceleration, max_accel)
    return TruthPNGResult(range_m, closing_speed, los, omega_los, lambda_dot, acceleration, True)


def integrate_velocity_command(
    current_velocity: np.ndarray | tuple[float, float, float] | list[float],
    acceleration: np.ndarray | tuple[float, float, float] | list[float],
    dt: float,
    speed_cap: Optional[float],
    min_speed: float = 0.0,
    fallback_direction: Optional[np.ndarray | tuple[float, float, float] | list[float]] = None,
    eps: float = 1.0e-6,
) -> np.ndarray:
    """Map a PNG acceleration into a bounded velocity command for AirSim.

    A real interceptor normally has propulsion maintaining forward speed. In
    AirSim velocity-control validation, min_speed and fallback_direction provide
    that speed-hold behavior when the multirotor starts from hover.
    """

    if dt <= 0.0:
        raise ValueError("dt must be positive")
    if speed_cap is not None and speed_cap < 0.0:
        raise ValueError("speed_cap must be non-negative")
    if min_speed < 0.0:
        raise ValueError("min_speed must be non-negative")

    current = _as_vector3(current_velocity)
    accel = _as_vector3(acceleration)
    predicted = current + accel * dt
    speed = float(np.linalg.norm(predicted))

    if min_speed > 0.0 and speed < min_speed:
        current_speed = float(np.linalg.norm(current))
        if fallback_direction is not None and current_speed < min_speed:
            fallback = _as_vector3(fallback_direction)
            fallback_norm = float(np.linalg.norm(fallback))
            direction = fallback / fallback_norm if fallback_norm > eps else np.zeros(3)
        elif current_speed > eps:
            direction = current / current_speed
        elif speed > eps:
            direction = predicted / speed
        else:
            direction = np.zeros(3)

        if float(np.linalg.norm(direction)) > eps:
            predicted = direction * min_speed + accel * dt

    return _clip_norm(predicted, speed_cap)
