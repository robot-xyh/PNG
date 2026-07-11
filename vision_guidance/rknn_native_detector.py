from __future__ import annotations

import ctypes
import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .types import FrameDetection


_ABI_VERSION = 2
_ERROR_BUFFER_SIZE = 1024
_REJECT_REASONS = {
    0: "",
    1: "rknn_no_candidates",
    2: "rknn_candidates_filtered",
}


@dataclass(frozen=True)
class RknnDetectorConfig:
    conf_threshold: float = 0.20
    iou_threshold: float = 0.45
    min_score: float = 0.25
    min_bbox_area: float = 0.0
    max_bbox_aspect_ratio: float = 3.0
    max_det: int = 300
    core_mask: int = 7
    temporal_gating_enabled: bool = False
    gate_radius_px: float = 160.0
    reacquire_area_ratio: float = 0.4
    track_hint_max_misses: int = 30

    @classmethod
    def from_mapping(cls, values: dict[str, Any]) -> "RknnDetectorConfig":
        return cls(
            conf_threshold=float(values.get("conf_threshold", 0.20)),
            iou_threshold=float(values.get("iou_threshold", 0.45)),
            min_score=float(values.get("min_score", 0.25)),
            min_bbox_area=float(values.get("min_bbox_area", 0.0)),
            max_bbox_aspect_ratio=float(values.get("max_bbox_aspect_ratio", 3.0)),
            max_det=int(values.get("max_det", 300)),
            core_mask=int(values.get("core_mask", 7)),
            temporal_gating_enabled=bool(values.get("temporal_gating_enabled", False)),
            gate_radius_px=float(values.get("gate_radius_px", 160.0)),
            reacquire_area_ratio=float(values.get("reacquire_area_ratio", 0.4)),
            track_hint_max_misses=int(values.get("track_hint_max_misses", 30)),
        )


class _NativeConfig(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("conf_threshold", ctypes.c_float),
        ("iou_threshold", ctypes.c_float),
        ("min_score", ctypes.c_float),
        ("min_bbox_area", ctypes.c_float),
        ("max_bbox_aspect_ratio", ctypes.c_float),
        ("max_det", ctypes.c_int32),
        ("core_mask", ctypes.c_int32),
        ("temporal_gating_enabled", ctypes.c_int32),
        ("gate_radius_px", ctypes.c_float),
        ("reacquire_area_ratio", ctypes.c_float),
        ("track_hint_max_misses", ctypes.c_int32),
    ]


class _NativeResult(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("valid", ctypes.c_int32),
        ("x1", ctypes.c_float),
        ("y1", ctypes.c_float),
        ("x2", ctypes.c_float),
        ("y2", ctypes.c_float),
        ("score", ctypes.c_float),
        ("class_id", ctypes.c_int32),
        ("track_id", ctypes.c_int32),
        ("raw_count", ctypes.c_int32),
        ("accepted_count", ctypes.c_int32),
        ("selected_index", ctypes.c_int32),
        ("reject_code", ctypes.c_int32),
        ("preprocess_ms", ctypes.c_float),
        ("inference_ms", ctypes.c_float),
        ("postprocess_ms", ctypes.c_float),
        ("total_ms", ctypes.c_float),
    ]


class _NativeDetection(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("x1", ctypes.c_float),
        ("y1", ctypes.c_float),
        ("x2", ctypes.c_float),
        ("y2", ctypes.c_float),
        ("score", ctypes.c_float),
        ("class_id", ctypes.c_int32),
        ("candidate_index", ctypes.c_int32),
    ]


class _NativeBatchResult(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("count", ctypes.c_int32),
        ("total_count", ctypes.c_int32),
        ("truncated", ctypes.c_int32),
        ("preprocess_ms", ctypes.c_float),
        ("inference_ms", ctypes.c_float),
        ("postprocess_ms", ctypes.c_float),
        ("total_ms", ctypes.c_float),
    ]


@dataclass(frozen=True)
class RknnDetection:
    bbox_xyxy: tuple[float, float, float, float]
    score: float
    class_id: int
    candidate_index: int


@dataclass(frozen=True)
class RknnDetectionBatch:
    detections: tuple[RknnDetection, ...]
    total_count: int
    truncated: bool
    preprocess_ms: float
    inference_ms: float
    postprocess_ms: float
    total_ms: float


class NativeRknnBridge:
    def __init__(self, library_path: str | Path, model_path: str | Path, config: RknnDetectorConfig):
        self.library_path = Path(library_path).expanduser().resolve()
        self.model_path = Path(model_path).expanduser().resolve()
        if not self.library_path.is_file():
            raise RuntimeError(f"RKNN bridge library not found: {self.library_path}")
        if not self.model_path.is_file():
            raise RuntimeError(f"RKNN model not found: {self.model_path}")

        self._library = ctypes.CDLL(str(self.library_path))
        self._configure_functions()
        abi_version = int(self._library.circle_rknn_abi_version())
        if abi_version != _ABI_VERSION:
            raise RuntimeError(f"RKNN bridge ABI mismatch: Python={_ABI_VERSION}, native={abi_version}")

        native_config = _NativeConfig(
            struct_size=ctypes.sizeof(_NativeConfig),
            conf_threshold=config.conf_threshold,
            iou_threshold=config.iou_threshold,
            min_score=config.min_score,
            min_bbox_area=config.min_bbox_area,
            max_bbox_aspect_ratio=config.max_bbox_aspect_ratio,
            max_det=config.max_det,
            core_mask=config.core_mask,
            temporal_gating_enabled=int(config.temporal_gating_enabled),
            gate_radius_px=config.gate_radius_px,
            reacquire_area_ratio=config.reacquire_area_ratio,
            track_hint_max_misses=config.track_hint_max_misses,
        )
        error = ctypes.create_string_buffer(_ERROR_BUFFER_SIZE)
        self._handle = self._library.circle_rknn_create(
            str(self.model_path).encode(), ctypes.byref(native_config), error, len(error)
        )
        if not self._handle:
            raise RuntimeError(f"failed to create RKNN detector: {_decode_error(error)}")
        schema_bytes = self._library.circle_rknn_output_schema(self._handle)
        schema_text = schema_bytes.decode("utf-8") if schema_bytes else "{}"
        try:
            self.output_schema = json.loads(schema_text)
        except json.JSONDecodeError as exc:
            self.close()
            raise RuntimeError(f"invalid RKNN output schema from native bridge: {schema_text}") from exc
        self.abi_version = abi_version

    def _configure_functions(self) -> None:
        self._library.circle_rknn_abi_version.argtypes = []
        self._library.circle_rknn_abi_version.restype = ctypes.c_uint32
        self._library.circle_rknn_create.argtypes = [
            ctypes.c_char_p,
            ctypes.POINTER(_NativeConfig),
            ctypes.c_char_p,
            ctypes.c_size_t,
        ]
        self._library.circle_rknn_create.restype = ctypes.c_void_p
        self._library.circle_rknn_infer.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int32,
            ctypes.c_int32,
            ctypes.c_int32,
            ctypes.POINTER(_NativeResult),
            ctypes.c_char_p,
            ctypes.c_size_t,
        ]
        self._library.circle_rknn_infer.restype = ctypes.c_int
        self._library.circle_rknn_infer_all.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int32,
            ctypes.c_int32,
            ctypes.c_int32,
            ctypes.POINTER(_NativeDetection),
            ctypes.c_int32,
            ctypes.POINTER(_NativeBatchResult),
            ctypes.c_char_p,
            ctypes.c_size_t,
        ]
        self._library.circle_rknn_infer_all.restype = ctypes.c_int
        self._library.circle_rknn_output_schema.argtypes = [ctypes.c_void_p]
        self._library.circle_rknn_output_schema.restype = ctypes.c_char_p
        self._library.circle_rknn_destroy.argtypes = [ctypes.c_void_p]
        self._library.circle_rknn_destroy.restype = None

    def infer(self, image_rgb: np.ndarray) -> _NativeResult:
        if self._handle is None:
            raise RuntimeError("RKNN detector is closed")
        image = _packed_rgb(image_rgb)
        result = _NativeResult(struct_size=ctypes.sizeof(_NativeResult))
        error = ctypes.create_string_buffer(_ERROR_BUFFER_SIZE)
        status = self._library.circle_rknn_infer(
            self._handle,
            image.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
            image.shape[1],
            image.shape[0],
            image.strides[0],
            ctypes.byref(result),
            error,
            len(error),
        )
        if status != 0:
            raise RuntimeError(f"RKNN inference failed ({status}): {_decode_error(error)}")
        return result

    def infer_all(self, image_rgb: np.ndarray, *, capacity: int = 300) -> RknnDetectionBatch:
        if self._handle is None:
            raise RuntimeError("RKNN detector is closed")
        if capacity <= 0:
            raise ValueError("RKNN detection capacity must be positive")
        image = _packed_rgb(image_rgb)
        native_detections = (_NativeDetection * int(capacity))()
        result = _NativeBatchResult(struct_size=ctypes.sizeof(_NativeBatchResult))
        error = ctypes.create_string_buffer(_ERROR_BUFFER_SIZE)
        status = self._library.circle_rknn_infer_all(
            self._handle,
            image.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
            image.shape[1],
            image.shape[0],
            image.strides[0],
            native_detections,
            capacity,
            ctypes.byref(result),
            error,
            len(error),
        )
        if status != 0:
            raise RuntimeError(f"RKNN batch inference failed ({status}): {_decode_error(error)}")
        detections = tuple(
            RknnDetection(
                bbox_xyxy=(float(item.x1), float(item.y1), float(item.x2), float(item.y2)),
                score=float(item.score),
                class_id=int(item.class_id),
                candidate_index=int(item.candidate_index),
            )
            for item in native_detections[: result.count]
        )
        return RknnDetectionBatch(
            detections=detections,
            total_count=int(result.total_count),
            truncated=bool(result.truncated),
            preprocess_ms=float(result.preprocess_ms),
            inference_ms=float(result.inference_ms),
            postprocess_ms=float(result.postprocess_ms),
            total_ms=float(result.total_ms),
        )

    def close(self) -> None:
        handle = getattr(self, "_handle", None)
        if handle:
            self._library.circle_rknn_destroy(handle)
            self._handle = None


class RknnNativeDetector:
    source = "rknn_native"

    def __init__(
        self,
        library_path: str | Path,
        model_path: str | Path,
        config: RknnDetectorConfig,
        *,
        bridge: Any = None,
    ) -> None:
        self.config = config
        self.bridge = bridge if bridge is not None else NativeRknnBridge(library_path, model_path, config)
        self.model_path = Path(model_path).expanduser().resolve()
        self.library_path = Path(library_path).expanduser().resolve()
        self.model_sha256 = _sha256(self.model_path) if self.model_path.is_file() else ""

    def detect(
        self,
        image_rgb: np.ndarray,
        *,
        frame_id: int,
        exposure_ts: float,
    ) -> tuple[FrameDetection | None, dict[str, Any]]:
        native = self.bridge.infer(_packed_rgb(image_rgb))
        reject_reason = _REJECT_REASONS.get(int(native.reject_code), f"rknn_reject_{native.reject_code}")
        stats = {
            "detector_source": self.source,
            "detector_reject_reason": reject_reason,
            "detector_raw_count": int(native.raw_count),
            "detector_class_filtered_count": int(native.accepted_count),
            "detector_track_filtered_count": int(bool(native.valid)),
            "rknn_selected_index": int(native.selected_index),
            "rknn_preprocess_ms": float(native.preprocess_ms),
            "rknn_inference_ms": float(native.inference_ms),
            "rknn_postprocess_ms": float(native.postprocess_ms),
            "rknn_total_ms": float(native.total_ms),
        }
        if not native.valid:
            return None, stats
        detection = FrameDetection(
            frame_id=int(frame_id),
            exposure_ts=float(exposure_ts),
            bbox_xyxy=(float(native.x1), float(native.y1), float(native.x2), float(native.y2)),
            track_id=int(native.track_id),
            score=float(native.score),
        )
        return detection, stats

    def metadata(self) -> dict[str, Any]:
        return {
            "backend": self.source,
            "abi_version": int(getattr(self.bridge, "abi_version", _ABI_VERSION)),
            "library_path": str(self.library_path),
            "model_path": str(self.model_path),
            "model_sha256": self.model_sha256,
            "output_schema": getattr(self.bridge, "output_schema", {}),
            "config": asdict(self.config),
        }

    def close(self) -> None:
        self.bridge.close()


def _packed_rgb(image: np.ndarray) -> np.ndarray:
    array = np.asarray(image)
    if array.dtype != np.uint8 or array.ndim != 3 or array.shape[2] != 3:
        raise ValueError("RKNN input must be an HxWx3 uint8 RGB image")
    return np.ascontiguousarray(array)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _decode_error(buffer: ctypes.Array) -> str:
    return bytes(buffer.value).decode("utf-8", errors="replace") or "unknown error"
