from __future__ import annotations

import math

import numpy as np

from .types import CameraIntrinsics


def normalize(vec: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < eps:
        raise ValueError("cannot normalize near-zero vector")
    return vec / norm


def project_perpendicular(vec: np.ndarray, unit_axis: np.ndarray) -> np.ndarray:
    axis = normalize(unit_axis)
    return vec - float(np.dot(vec, axis)) * axis


def camera_ray_from_pixel(u: float, v: float, intrinsics: CameraIntrinsics) -> np.ndarray:
    x_n = (u - intrinsics.cx) / intrinsics.fx
    y_n = (v - intrinsics.cy) / intrinsics.fy
    return normalize(np.array([x_n, y_n, 1.0], dtype=float))


def rotation_y(theta_rad: float) -> np.ndarray:
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=float,
    )


def camera_to_body_mount(pitch_up_deg: float = 45.0) -> np.ndarray:
    return rotation_y(math.radians(pitch_up_deg))


def los_camera_to_inertial(los_C: np.ndarray, R_BC: np.ndarray, R_IB: np.ndarray) -> np.ndarray:
    return normalize(R_IB @ R_BC @ normalize(los_C))
