from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Optional, Sequence

import numpy as np

from .types import CameraIntrinsics, FrameDetection


@dataclass(frozen=True)
class AirSimDetectionConfig:
    camera_name: str = "0"
    image_type_name: str = "Scene"
    detection_radius_cm: float = 20000.0
    mesh_name_pattern: str = "Intruder*"
    vehicle_name: str = "Interceptor"


def quaternion_to_rotation_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    norm = (w * w + x * x + y * y + z * z) ** 0.5
    if norm <= 1e-12:
        raise ValueError("invalid near-zero quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def airsim_orientation_to_R_IB(orientation: Any) -> np.ndarray:
    return quaternion_to_rotation_matrix(
        float(orientation.w_val),
        float(orientation.x_val),
        float(orientation.y_val),
        float(orientation.z_val),
    )


def detection_to_frame_detection(
    detection: Any,
    frame_id: int,
    exposure_ts: float,
    track_id: int,
    score: float = 1.0,
) -> FrameDetection:
    box = detection.box2D
    return FrameDetection(
        frame_id=frame_id,
        exposure_ts=exposure_ts,
        bbox_xyxy=(
            float(box.min.x_val),
            float(box.min.y_val),
            float(box.max.x_val),
            float(box.max.y_val),
        ),
        track_id=track_id,
        score=score,
    )


def choose_detection(detections: Sequence[Any], preferred_name: Optional[str] = None) -> Optional[Any]:
    if not detections:
        return None
    if preferred_name is not None:
        for detection in detections:
            if getattr(detection, "name", None) == preferred_name:
                return detection
    # AirSim built-in detections do not provide a conventional detector score.
    # Use largest visible area as a deterministic proxy.
    return max(detections, key=_box_area)


def _box_area(detection: Any) -> float:
    box = detection.box2D
    w = max(0.0, float(box.max.x_val) - float(box.min.x_val))
    h = max(0.0, float(box.max.y_val) - float(box.min.y_val))
    return w * h


def configure_detection_filter(client: Any, config: AirSimDetectionConfig) -> None:
    image_type = _image_type(client, config.image_type_name)
    clear = getattr(client, "simClearDetectionMeshNames", None)
    if callable(clear):
        clear(
            config.camera_name,
            image_type,
            vehicle_name=config.vehicle_name,
        )
    client.simSetDetectionFilterRadius(
        config.camera_name,
        image_type,
        config.detection_radius_cm,
        vehicle_name=config.vehicle_name,
    )
    client.simAddDetectionFilterMeshName(
        config.camera_name,
        image_type,
        config.mesh_name_pattern,
        vehicle_name=config.vehicle_name,
    )


def get_detections(client: Any, config: AirSimDetectionConfig) -> Iterable[Any]:
    image_type = _image_type(client, config.image_type_name)
    return client.simGetDetections(
        config.camera_name,
        image_type,
        vehicle_name=config.vehicle_name,
    )


def _image_type(client: Any, name: str) -> Any:
    # Use the imported airsim module if available; otherwise client-side tests
    # can pass a simple object exposing ImageType.
    image_type_container = getattr(client, "ImageType", None)
    if image_type_container is not None:
        return getattr(image_type_container, name)
    try:
        import airsim  # type: ignore
    except ImportError as exc:
        raise RuntimeError("airsim package is required to resolve ImageType") from exc
    return getattr(airsim.ImageType, name)


def infer_intrinsics_from_fov(width: int, height: int, fov_degrees: float) -> CameraIntrinsics:
    fov = np.deg2rad(float(fov_degrees))
    fx = 0.5 * width / np.tan(0.5 * fov)
    fy = fx
    return CameraIntrinsics(fx=fx, fy=fy, cx=0.5 * width, cy=0.5 * height, width=width, height=height)
