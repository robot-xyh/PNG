from __future__ import annotations

from dataclasses import dataclass
from fnmatch import fnmatchcase
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


@dataclass(frozen=True)
class AirSimPairCollision:
    collided: bool
    reason: str
    interceptor_has_collided: bool
    intruder_has_collided: bool
    interceptor_object_name: str
    intruder_object_name: str


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


def get_vehicle_pair_collision(
    client: Any,
    interceptor: str,
    intruder: str,
    interceptor_object_patterns: Sequence[str] | None = None,
    intruder_object_patterns: Sequence[str] | None = None,
) -> AirSimPairCollision:
    """Return true only when AirSim object names identify a vehicle-to-vehicle collision."""

    interceptor_info = client.simGetCollisionInfo(vehicle_name=interceptor)
    intruder_info = client.simGetCollisionInfo(vehicle_name=intruder)
    interceptor_hit = bool(getattr(interceptor_info, "has_collided", False))
    intruder_hit = bool(getattr(intruder_info, "has_collided", False))
    interceptor_object = str(getattr(interceptor_info, "object_name", "") or "")
    intruder_object = str(getattr(intruder_info, "object_name", "") or "")
    interceptor_patterns = tuple(interceptor_object_patterns or _default_collision_patterns(interceptor))
    intruder_patterns = tuple(intruder_object_patterns or _default_collision_patterns(intruder))

    interceptor_names_intruder = interceptor_hit and _collision_object_matches(interceptor_object, intruder_patterns)
    intruder_names_interceptor = intruder_hit and _collision_object_matches(intruder_object, interceptor_patterns)
    if interceptor_names_intruder or intruder_names_interceptor:
        return AirSimPairCollision(
            True,
            "object_name_pattern_match",
            interceptor_hit,
            intruder_hit,
            interceptor_object,
            intruder_object,
        )

    return AirSimPairCollision(
        False,
        "",
        interceptor_hit,
        intruder_hit,
        interceptor_object,
        intruder_object,
    )


def get_vehicle_object_collision(
    client: Any,
    vehicle_name: str,
    object_patterns: Sequence[str],
) -> AirSimPairCollision:
    """Return true when a vehicle collision names a non-vehicle scene object."""

    collision_info = client.simGetCollisionInfo(vehicle_name=vehicle_name)
    vehicle_hit = bool(getattr(collision_info, "has_collided", False))
    object_name = str(getattr(collision_info, "object_name", "") or "")
    matched = vehicle_hit and _collision_object_matches(object_name, object_patterns)
    return AirSimPairCollision(
        matched,
        "interceptor_object_pattern_match" if matched else "",
        vehicle_hit,
        False,
        object_name,
        "",
    )


def _default_collision_patterns(vehicle_name: str) -> tuple[str, ...]:
    return (vehicle_name, f"{vehicle_name}*")


def _collision_object_matches(object_name: str, patterns: Sequence[str]) -> bool:
    normalized_object = object_name.strip().lower()
    if not normalized_object:
        return False
    for pattern in patterns:
        normalized_pattern = str(pattern or "").strip().lower()
        if normalized_pattern and fnmatchcase(normalized_object, normalized_pattern):
            return True
    return False


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
