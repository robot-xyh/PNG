from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .types import CameraIntrinsics


IMAGE_KF_UPDATE = "update"
IMAGE_KF_PREDICT = "predict"
IMAGE_KF_INVALID = "invalid"
IMAGE_KF_DISABLED = "disabled"


@dataclass(frozen=True)
class TerminalImageKFConfig:
    enable: bool = True
    max_predict_s: float = 0.25
    measurement_noise_rad: float = 0.006
    accel_noise_rad_s2: float = 8.0
    innovation_reject_rad: float = 0.20
    soft_reject_predict: bool = False
    max_angle_rad: float = 1.0
    max_rate_rad_s: float = 8.0
    min_dt_s: float = 1.0e-3


@dataclass(frozen=True)
class TerminalImageEstimate:
    timestamp: float
    theta_x: float
    theta_y: float
    theta_dot_x: float
    theta_dot_y: float
    valid: bool
    mode: str
    age_s: float
    quality: float
    reject_reason: str = ""

    @property
    def theta(self) -> np.ndarray:
        return np.array([self.theta_x, self.theta_y], dtype=float)

    @property
    def theta_dot(self) -> np.ndarray:
        return np.array([self.theta_dot_x, self.theta_dot_y], dtype=float)


class TerminalImageKF:
    def __init__(self, config: TerminalImageKFConfig | None = None):
        self.config = config or TerminalImageKFConfig()
        self.x = np.zeros(4, dtype=float)
        self.P = np.diag([0.05, 0.05, 1.0, 1.0]).astype(float)
        self.initialized = False
        self.last_ts: Optional[float] = None
        self.last_measurement_ts: Optional[float] = None
        self.track_id: Optional[int] = None

    def reset(
        self,
        theta: Optional[np.ndarray] = None,
        timestamp: Optional[float] = None,
        track_id: Optional[int] = None,
    ) -> None:
        self.x = np.zeros(4, dtype=float)
        if theta is not None:
            self.x[:2] = _finite_theta(theta, self.config.max_angle_rad)
        self.P = np.diag([0.05, 0.05, 1.0, 1.0]).astype(float)
        self.initialized = theta is not None
        self.last_ts = timestamp
        self.last_measurement_ts = timestamp if theta is not None else None
        self.track_id = track_id

    def update(
        self,
        *,
        timestamp: float,
        center: Optional[tuple[float, float]],
        intrinsics: CameraIntrinsics,
        detected: bool,
        measurement_valid: bool,
        clipped: bool = False,
        track_id: Optional[int] = None,
    ) -> TerminalImageEstimate:
        if not self.config.enable:
            return self._estimate(timestamp, IMAGE_KF_DISABLED, False, "disabled")

        usable_measurement = bool(detected and measurement_valid and not clipped and center is not None)
        theta = None
        if usable_measurement:
            theta = angle_error_from_center(center, intrinsics)
            usable_measurement = bool(np.all(np.isfinite(theta)))

        if usable_measurement and self.initialized and track_id is not None and self.track_id is not None:
            if track_id != self.track_id:
                self.reset(theta, timestamp, track_id)
                return self._estimate(timestamp, IMAGE_KF_UPDATE, True, quality=0.5)

        if not self.initialized:
            if usable_measurement and theta is not None:
                self.reset(theta, timestamp, track_id)
                return self._estimate(timestamp, IMAGE_KF_UPDATE, True, quality=1.0)
            return self._estimate(timestamp, IMAGE_KF_INVALID, False, "uninitialized")

        self._predict_to(timestamp)

        if not usable_measurement or theta is None:
            reason = "bbox_clipped" if clipped else "no_measurement"
            return self._prediction_estimate(timestamp, reason)

        innovation = theta - self.x[:2]
        innovation_norm = float(np.linalg.norm(innovation))
        if innovation_norm > max(1.0e-9, self.config.innovation_reject_rad):
            if self.config.soft_reject_predict:
                return self._prediction_estimate(timestamp, "image_kf_soft_reject")
            self.reset(theta, timestamp, track_id)
            return self._estimate(timestamp, IMAGE_KF_INVALID, False, "image_kf_innovation_reject")

        H = np.zeros((2, 4), dtype=float)
        H[:, :2] = np.eye(2)
        r = max(1.0e-9, self.config.measurement_noise_rad) ** 2
        R = r * np.eye(2)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ H) @ self.P
        self._apply_limits()
        self.last_measurement_ts = timestamp
        self.track_id = track_id
        quality = max(0.0, 1.0 - innovation_norm / max(1.0e-9, self.config.innovation_reject_rad))
        return self._estimate(timestamp, IMAGE_KF_UPDATE, True, quality=quality)

    def _predict_to(self, timestamp: float) -> None:
        dt = self.config.min_dt_s
        if self.last_ts is not None:
            dt = max(self.config.min_dt_s, float(timestamp) - float(self.last_ts))
        self.last_ts = timestamp

        F = np.eye(4, dtype=float)
        F[0, 2] = dt
        F[1, 3] = dt
        q = max(0.0, self.config.accel_noise_rad_s2) ** 2
        q_axis = q * np.array([[0.25 * dt**4, 0.5 * dt**3], [0.5 * dt**3, dt**2]], dtype=float)
        Q = np.zeros((4, 4), dtype=float)
        Q[np.ix_([0, 2], [0, 2])] = q_axis
        Q[np.ix_([1, 3], [1, 3])] = q_axis
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self._apply_limits()

    def _prediction_estimate(self, timestamp: float, reason: str) -> TerminalImageEstimate:
        age = self._age(timestamp)
        if age > max(0.0, self.config.max_predict_s):
            return self._estimate(timestamp, IMAGE_KF_INVALID, False, "image_kf_predict_timeout")
        if not self._within_limits():
            return self._estimate(timestamp, IMAGE_KF_INVALID, False, "image_kf_state_limit")
        quality = max(0.0, 1.0 - age / max(1.0e-9, self.config.max_predict_s))
        return self._estimate(timestamp, IMAGE_KF_PREDICT, True, reason, quality=quality)

    def _apply_limits(self) -> None:
        self.x[0] = float(np.clip(self.x[0], -self.config.max_angle_rad, self.config.max_angle_rad))
        self.x[1] = float(np.clip(self.x[1], -self.config.max_angle_rad, self.config.max_angle_rad))
        self.x[2] = float(np.clip(self.x[2], -self.config.max_rate_rad_s, self.config.max_rate_rad_s))
        self.x[3] = float(np.clip(self.x[3], -self.config.max_rate_rad_s, self.config.max_rate_rad_s))

    def _within_limits(self) -> bool:
        return (
            abs(float(self.x[0])) <= self.config.max_angle_rad
            and abs(float(self.x[1])) <= self.config.max_angle_rad
            and abs(float(self.x[2])) <= self.config.max_rate_rad_s
            and abs(float(self.x[3])) <= self.config.max_rate_rad_s
            and np.all(np.isfinite(self.x))
        )

    def _age(self, timestamp: float) -> float:
        if self.last_measurement_ts is None:
            return 0.0
        return max(0.0, float(timestamp) - float(self.last_measurement_ts))

    def _estimate(
        self,
        timestamp: float,
        mode: str,
        valid: bool,
        reason: str = "",
        quality: float = 0.0,
    ) -> TerminalImageEstimate:
        return TerminalImageEstimate(
            timestamp=float(timestamp),
            theta_x=float(self.x[0]),
            theta_y=float(self.x[1]),
            theta_dot_x=float(self.x[2]),
            theta_dot_y=float(self.x[3]),
            valid=bool(valid),
            mode=mode,
            age_s=self._age(timestamp),
            quality=float(np.clip(quality, 0.0, 1.0)),
            reject_reason=reason,
        )


def angle_error_from_center(center: tuple[float, float], intrinsics: CameraIntrinsics) -> np.ndarray:
    u, v = center
    return np.array(
        [
            np.arctan2(float(u) - float(intrinsics.cx), max(1.0e-9, float(intrinsics.fx))),
            np.arctan2(float(v) - float(intrinsics.cy), max(1.0e-9, float(intrinsics.fy))),
        ],
        dtype=float,
    )


def center_from_angle_error(theta: np.ndarray, intrinsics: CameraIntrinsics) -> tuple[float, float]:
    theta = np.asarray(theta, dtype=float).reshape(2)
    return (
        float(intrinsics.cx + intrinsics.fx * np.tan(theta[0])),
        float(intrinsics.cy + intrinsics.fy * np.tan(theta[1])),
    )


def _finite_theta(theta: np.ndarray, limit: float) -> np.ndarray:
    value = np.asarray(theta, dtype=float).reshape(2)
    value = np.nan_to_num(value, nan=0.0, posinf=limit, neginf=-limit)
    return np.clip(value, -limit, limit)
