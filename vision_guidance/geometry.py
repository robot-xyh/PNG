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


def rotation_z(theta_rad: float) -> np.ndarray:
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def rotation_x(theta_rad: float) -> np.ndarray:
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=float,
    )


def camera_to_body_mount(pitch_up_deg: float = 45.0) -> np.ndarray:
    return rotation_y(math.radians(pitch_up_deg))


def validated_rotation_matrix(
    matrix: np.ndarray,
    *,
    name: str = "rotation matrix",
    atol: float = 1.0e-6,
) -> np.ndarray:
    """Return a finite, proper orthonormal rotation matrix or raise."""

    value = np.asarray(matrix, dtype=float)
    if value.shape != (3, 3):
        raise ValueError(f"{name} must be 3x3")
    if not np.all(np.isfinite(value)):
        raise ValueError(f"{name} must contain only finite values")
    orthonormal_error = float(np.linalg.norm(value.T @ value - np.eye(3), ord="fro"))
    determinant = float(np.linalg.det(value))
    if orthonormal_error > float(atol):
        raise ValueError(f"{name} must be orthonormal")
    if abs(determinant - 1.0) > float(atol):
        raise ValueError(f"{name} must be a proper rotation with determinant +1")
    return value


def camera_mount_diagnostics(
    R_BC: np.ndarray,
    *,
    expected_optical_axis_body: np.ndarray | tuple[float, float, float] = (0.0, 0.0, -1.0),
) -> dict[str, float | list[float]]:
    """Describe how OpenCV camera axes map into the Betaflight FRD body frame."""

    rotation = validated_rotation_matrix(R_BC, name="camera.R_BC")
    expected = normalize(np.asarray(expected_optical_axis_body, dtype=float))
    optical_axis_body = normalize(rotation @ np.array([0.0, 0.0, 1.0], dtype=float))
    cosine = float(np.clip(np.dot(optical_axis_body, expected), -1.0, 1.0))
    return {
        "determinant": float(np.linalg.det(rotation)),
        "orthonormal_error": float(np.linalg.norm(rotation.T @ rotation - np.eye(3), ord="fro")),
        "camera_x_axis_body": [float(value) for value in rotation[:, 0]],
        "camera_y_axis_body": [float(value) for value in rotation[:, 1]],
        "optical_axis_body": [float(value) for value in optical_axis_body],
        "expected_optical_axis_body": [float(value) for value in expected],
        "optical_axis_error_deg": math.degrees(math.acos(cosine)),
    }


def airsim_camera_zero_to_body() -> np.ndarray:
    """Map AirSim camera ray coordinates to body NED axes at zero gimbal angle.

    AirSim image rays use camera x right, y down, z forward. For a forward
    mounted multirotor camera, body x is forward, body y is right, body z is
    down.
    """

    return np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=float,
    )


def airsim_gimbal_camera_to_body(yaw_rad: float, pitch_rad: float) -> np.ndarray:
    """Camera-to-body rotation for a yaw/pitch AirSim gimbal.

    Positive yaw turns the camera to body-right. Positive pitch points the
    camera downward in body NED coordinates.
    """

    return rotation_z(yaw_rad) @ rotation_y(-pitch_rad) @ airsim_camera_zero_to_body()


def airsim_fixed_camera_to_body(
    yaw_rad: float = 0.0,
    pitch_rad: float = 0.0,
    roll_rad: float = 0.0,
) -> np.ndarray:
    """Camera-to-body rotation for a fixed AirSim camera mount.

    The pitch convention matches :func:`airsim_gimbal_camera_to_body`: positive
    pitch points the camera down in body NED coordinates, so a camera pitched
    upward by 15 degrees should use ``pitch_rad=-deg2rad(15)``.
    """

    return rotation_z(yaw_rad) @ rotation_y(-pitch_rad) @ rotation_x(roll_rad) @ airsim_camera_zero_to_body()


def los_camera_to_inertial(los_C: np.ndarray, R_BC: np.ndarray, R_IB: np.ndarray) -> np.ndarray:
    return normalize(R_IB @ R_BC @ normalize(los_C))
