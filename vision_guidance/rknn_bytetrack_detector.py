from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

from .bytetrack_adapter import ByteTrackAdapter, ByteTrackConfig
from .rknn_native_detector import NativeRknnBridge, RknnDetectorConfig, _packed_rgb, _sha256
from .types import FrameDetection


class RknnByteTrackDetector:
    source = "rknn_bytetrack"

    def __init__(
        self,
        library_path: str | Path,
        model_path: str | Path,
        rknn_config: RknnDetectorConfig,
        tracker_config: ByteTrackConfig,
        *,
        bridge: Any = None,
        tracker: ByteTrackAdapter | None = None,
    ) -> None:
        self.rknn_config = rknn_config
        self.tracker_config = tracker_config
        self.bridge = bridge if bridge is not None else NativeRknnBridge(library_path, model_path, rknn_config)
        self.tracker = tracker if tracker is not None else ByteTrackAdapter(tracker_config)
        self.library_path = Path(library_path).expanduser().resolve()
        self.model_path = Path(model_path).expanduser().resolve()
        self.model_sha256 = _sha256(self.model_path) if self.model_path.is_file() else ""

    def detect(
        self,
        image_rgb: np.ndarray,
        *,
        frame_id: int,
        exposure_ts: float,
    ) -> tuple[FrameDetection | None, dict[str, Any]]:
        batch = self.bridge.infer_all(_packed_rgb(image_rgb), capacity=self.rknn_config.max_det)
        update = self.tracker.update(batch.detections, timestamp=exposure_ts)
        best_score = max((float(item.score) for item in batch.detections), default=None)
        stats = {
            "detector_source": self.source,
            "detector_reject_reason": "",
            "detector_raw_count": batch.total_count,
            "detector_class_filtered_count": len(batch.detections),
            "detector_track_filtered_count": int(update.selected is not None),
            "detector_best_score": best_score,
            "rknn_batch_truncated": int(batch.truncated),
            "rknn_preprocess_ms": batch.preprocess_ms,
            "rknn_inference_ms": batch.inference_ms,
            "rknn_postprocess_ms": batch.postprocess_ms,
            "rknn_total_ms": batch.total_ms,
        }
        stats.update(update.stats)
        if update.selected is None:
            stats["detector_reject_reason"] = str(update.stats["target_selector_reason"])
            return None, stats
        selected = update.selected
        detection = FrameDetection(
            frame_id=int(frame_id),
            exposure_ts=float(exposure_ts),
            bbox_xyxy=selected.bbox_xyxy,
            track_id=selected.track_id,
            score=selected.score,
        )
        return detection, stats

    def metadata(self) -> dict[str, Any]:
        return {
            "backend": self.source,
            "abi_version": int(getattr(self.bridge, "abi_version", 2)),
            "library_path": str(self.library_path),
            "model_path": str(self.model_path),
            "model_sha256": self.model_sha256,
            "output_schema": getattr(self.bridge, "output_schema", {}),
            "rknn_config": asdict(self.rknn_config),
            "tracker": self.tracker.metadata(),
        }

    def close(self) -> None:
        self.bridge.close()
