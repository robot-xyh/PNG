from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .geometry import normalize, project_perpendicular
from .types import LOSEstimate


@dataclass(frozen=True)
class LOSFilterConfig:
    process_lambda: float = 1e-4
    process_lambda_dot: float = 5e-3
    measurement_noise: float = 5e-3
    innovation_reject: float = 0.25


class LOSKalmanFilter6D:
    def __init__(self, config: LOSFilterConfig | None = None):
        self.config = config or LOSFilterConfig()
        self.x = np.zeros(6, dtype=float)
        self.x[2] = 1.0
        self.P = np.eye(6, dtype=float)
        self.initialized = False
        self.last_ts: Optional[float] = None

    def reset(self, lambda_I: Optional[np.ndarray] = None, timestamp: Optional[float] = None) -> None:
        self.x = np.zeros(6, dtype=float)
        self.x[:3] = normalize(np.asarray(lambda_I if lambda_I is not None else [0.0, 0.0, 1.0], dtype=float))
        self.P = np.eye(6, dtype=float)
        self.initialized = lambda_I is not None
        self.last_ts = timestamp

    def update(self, timestamp: float, lambda_measured: np.ndarray) -> LOSEstimate:
        z = normalize(np.asarray(lambda_measured, dtype=float))
        if not self.initialized:
            self.reset(z, timestamp)
            return self._estimate(timestamp, 0.0, valid=True)

        dt = max(1e-3, timestamp - float(self.last_ts)) if self.last_ts is not None else 1e-2
        self.last_ts = timestamp

        F = np.eye(6, dtype=float)
        F[:3, 3:] = dt * np.eye(3)
        Q = np.diag(
            [
                self.config.process_lambda,
                self.config.process_lambda,
                self.config.process_lambda,
                self.config.process_lambda_dot,
                self.config.process_lambda_dot,
                self.config.process_lambda_dot,
            ]
        )
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self._apply_constraints()

        H = np.zeros((3, 6), dtype=float)
        H[:, :3] = np.eye(3)
        R = self.config.measurement_noise * np.eye(3)
        y = z - H @ self.x
        innovation_norm = float(np.linalg.norm(y))
        if innovation_norm > self.config.innovation_reject:
            return self._estimate(timestamp, innovation_norm, valid=False, reason="los_innovation_reject")

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
        self._apply_constraints()
        quality = max(0.0, 1.0 - innovation_norm / self.config.innovation_reject)
        return self._estimate(timestamp, innovation_norm, valid=True, quality=quality)

    def _apply_constraints(self) -> None:
        lam = normalize(self.x[:3])
        lam_dot = project_perpendicular(self.x[3:], lam)
        self.x[:3] = lam
        self.x[3:] = lam_dot

    def _estimate(
        self,
        timestamp: float,
        innovation_norm: float,
        valid: bool,
        quality: float = 1.0,
        reason: Optional[str] = None,
    ) -> LOSEstimate:
        lam = normalize(self.x[:3])
        lam_dot = project_perpendicular(self.x[3:], lam)
        omega = np.cross(lam, lam_dot)
        return LOSEstimate(timestamp, lam, lam_dot, omega, innovation_norm, quality, valid, reason)
