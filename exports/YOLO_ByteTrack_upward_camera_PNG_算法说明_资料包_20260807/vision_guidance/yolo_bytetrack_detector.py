from __future__ import annotations

import importlib
import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np

from .airsim_adapter import (
    AirSimDetectionConfig,
    choose_detection,
    detection_to_frame_detection,
    get_detections,
)
from .types import FrameDetection


@dataclass(frozen=True)
class _Point2D:
    x_val: float
    y_val: float


@dataclass(frozen=True)
class _Box2D:
    min: _Point2D
    max: _Point2D


@dataclass(frozen=True)
class TrackedBoxDetection:
    name: str
    box2D: _Box2D
    score: float
    track_id: Optional[int]
    class_id: int
    source: str


@dataclass
class DetectorFrame:
    detections: list[Any]
    selected: Optional[Any]
    frame_detection: Optional[FrameDetection]
    stats: dict[str, Any]
    image_bgr: Optional[np.ndarray] = None


def add_detector_args(parser: Any) -> None:
    parser.add_argument(
        "--detector-source",
        choices=("airsim", "yolo_bytetrack", "yolo_kcf"),
        default="airsim",
        help=(
            "Detection source. AirSim detect remains the default; YOLO modes fail fast on "
            "missing model/dependencies. yolo_kcf uses YOLO for init/correction and KCF "
            "for frame-to-frame tracking."
        ),
    )
    parser.add_argument("--yolo-model", default="", help="YOLOv8 model path used when --detector-source yolo_bytetrack.")
    parser.add_argument("--yolo-class-id", type=int, default=None, help="Target class id used to filter YOLO detections.")
    parser.add_argument("--yolo-conf", type=float, default=0.25)
    parser.add_argument("--yolo-iou", type=float, default=0.70)
    parser.add_argument("--yolo-imgsz", type=int, default=640)
    parser.add_argument("--yolo-device", default="", help="Ultralytics device string, for example '0', 'cpu', or empty for auto.")
    parser.add_argument("--yolo-tracker", default="bytetrack.yaml")
    parser.add_argument(
        "--yolo-allow-untracked-fallback",
        action="store_true",
        help=(
            "When ByteTrack has not assigned an id, use the highest-confidence YOLO box "
            "as a temporary pseudo track instead of rejecting the frame."
        ),
    )
    parser.add_argument(
        "--yolo-single-target-mode",
        action="store_true",
        help=(
            "Use a single-target continuity selector. This is useful when the scene has only one "
            "real target and ByteTrack ids are intermittent."
        ),
    )
    parser.add_argument(
        "--yolo-single-target-max-center-jump-px",
        type=float,
        default=220.0,
        help="Maximum bbox center jump accepted by --yolo-single-target-mode.",
    )
    parser.add_argument("--kcf-yolo-period-n", type=int, default=8, help="Run YOLO correction every N frames in yolo_kcf mode.")
    parser.add_argument("--kcf-yolo-period-s", type=float, default=0.5, help="Run YOLO correction after this many seconds in yolo_kcf mode.")
    parser.add_argument("--kcf-max-coast-s", type=float, default=0.8, help="Maximum KCF-only tracking time after the last YOLO correction.")
    parser.add_argument("--kcf-min-yolo-iou", type=float, default=0.25, help="Minimum YOLO/KCF IoU accepted for correction.")
    parser.add_argument("--kcf-max-center-jump-px", type=float, default=180.0, help="Maximum YOLO/KCF center jump accepted for correction.")
    parser.add_argument("--kcf-area-ratio-min", type=float, default=0.35, help="Minimum YOLO/KCF area ratio accepted for correction.")
    parser.add_argument("--kcf-area-ratio-max", type=float, default=2.8, help="Maximum YOLO/KCF area ratio accepted for correction.")
    parser.add_argument(
        "--kcf-reset-on-yolo-drift",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="When YOLO and KCF disagree, reset/reinitialize KCF from the YOLO box instead of coasting.",
    )


def create_detection_provider(args: Any, airsim_module: Any = None):
    source = str(getattr(args, "detector_source", "airsim") or "airsim")
    if source == "airsim":
        return AirSimBuiltinDetector()
    if source == "yolo_bytetrack":
        return YoloByteTrackDetector(
            model_path=str(getattr(args, "yolo_model", "") or ""),
            class_id=getattr(args, "yolo_class_id", None),
            conf=float(getattr(args, "yolo_conf", 0.25)),
            iou=float(getattr(args, "yolo_iou", 0.70)),
            imgsz=int(getattr(args, "yolo_imgsz", 640)),
            device=str(getattr(args, "yolo_device", "") or ""),
            tracker=str(getattr(args, "yolo_tracker", "bytetrack.yaml") or "bytetrack.yaml"),
            allow_untracked_fallback=bool(getattr(args, "yolo_allow_untracked_fallback", False)),
            single_target_mode=bool(getattr(args, "yolo_single_target_mode", False)),
            single_target_max_center_jump_px=float(getattr(args, "yolo_single_target_max_center_jump_px", 220.0)),
            airsim_module=airsim_module,
        )
    if source == "yolo_kcf":
        return YoloKcfDetector(
            model_path=str(getattr(args, "yolo_model", "") or ""),
            class_id=getattr(args, "yolo_class_id", None),
            conf=float(getattr(args, "yolo_conf", 0.25)),
            iou=float(getattr(args, "yolo_iou", 0.70)),
            imgsz=int(getattr(args, "yolo_imgsz", 640)),
            device=str(getattr(args, "yolo_device", "") or ""),
            single_target_mode=bool(getattr(args, "yolo_single_target_mode", False)),
            single_target_max_center_jump_px=float(getattr(args, "yolo_single_target_max_center_jump_px", 220.0)),
            yolo_period_n=int(getattr(args, "kcf_yolo_period_n", 8)),
            yolo_period_s=float(getattr(args, "kcf_yolo_period_s", 0.5)),
            max_coast_s=float(getattr(args, "kcf_max_coast_s", 0.8)),
            min_yolo_iou=float(getattr(args, "kcf_min_yolo_iou", 0.25)),
            max_center_jump_px=float(getattr(args, "kcf_max_center_jump_px", 180.0)),
            area_ratio_min=float(getattr(args, "kcf_area_ratio_min", 0.35)),
            area_ratio_max=float(getattr(args, "kcf_area_ratio_max", 2.8)),
            reset_on_yolo_drift=bool(getattr(args, "kcf_reset_on_yolo_drift", True)),
            airsim_module=airsim_module,
        )
    raise ValueError(f"unsupported detector source: {source}")


class AirSimBuiltinDetector:
    source = "airsim"

    def detect(
        self,
        *,
        client: Any,
        config: AirSimDetectionConfig,
        frame_id: int,
        exposure_ts: float,
        active_name: Optional[str] = None,
        active_track_id: Optional[int] = None,
    ) -> DetectorFrame:
        del active_track_id
        detections = list(get_detections(client, config))
        selected = choose_detection(detections, preferred_name=active_name)
        frame_detection = None
        if selected is not None:
            frame_detection = detection_to_frame_detection(
                selected,
                frame_id=frame_id,
                exposure_ts=exposure_ts,
                track_id=int(getattr(selected, "track_id", 1) or 1),
                score=float(getattr(selected, "score", 1.0) or 1.0),
            )
        return DetectorFrame(
            detections=detections,
            selected=selected,
            frame_detection=frame_detection,
            stats={
                "detector_source": self.source,
                "detector_reject_reason": "" if selected is not None else "no_detection",
                "detector_raw_count": len(detections),
                "detector_class_filtered_count": len(detections),
                "detector_track_filtered_count": 1 if frame_detection is not None else 0,
                "yolo_raw_count": "",
                "yolo_class_filtered_count": "",
                "yolo_track_filtered_count": "",
                "yolo_track_missing_count": "",
                "yolo_selected_track_id": "",
                "yolo_selected_class_id": "",
                "yolo_selected_score": "",
                "yolo_selected_source": "",
                "yolo_requested_device": "",
                "yolo_runtime_device": "",
                "yolo_cuda_available": "",
                "yolo_gpu_name": "",
                "yolo_allow_untracked_fallback": "",
                "yolo_used_untracked_fallback": "",
                "yolo_single_target_mode": "",
                "yolo_single_target_selected": "",
                "yolo_single_target_distance_px": "",
            },
        )


class YoloByteTrackDetector:
    source = "yolo_bytetrack"

    def __init__(
        self,
        *,
        model_path: str,
        class_id: Optional[int],
        conf: float = 0.25,
        iou: float = 0.70,
        imgsz: int = 640,
        device: str = "",
        tracker: str = "bytetrack.yaml",
        allow_untracked_fallback: bool = False,
        single_target_mode: bool = False,
        single_target_max_center_jump_px: float = 220.0,
        airsim_module: Any = None,
        model_factory: Optional[Callable[[str], Any]] = None,
        image_reader: Optional[Callable[[Any, AirSimDetectionConfig], np.ndarray]] = None,
        cv2_module: Any = None,
    ) -> None:
        if class_id is None:
            raise RuntimeError("--yolo-class-id is required when --detector-source yolo_bytetrack.")
        self.class_id = int(class_id)
        self.conf = max(0.0, float(conf))
        self.iou = max(0.0, float(iou))
        self.imgsz = max(1, int(imgsz))
        self.device = str(device or "")
        self.tracker = str(tracker or "bytetrack.yaml")
        self.allow_untracked_fallback = bool(allow_untracked_fallback)
        self.single_target_mode = bool(single_target_mode)
        self.single_target_max_center_jump_px = max(0.0, float(single_target_max_center_jump_px))
        self.airsim_module = airsim_module
        self.image_reader = image_reader
        self.runtime_info = _torch_runtime_info()
        self._last_selected_center: Optional[tuple[float, float]] = None
        self._last_selected_area: Optional[float] = None

        path = Path(str(model_path or "")).expanduser()
        if not path.exists():
            raise RuntimeError(f"YOLO model file not found: {path}. Pass --yolo-model <path>.")
        self.model_path = path

        if model_factory is None:
            self.cv2 = cv2_module if cv2_module is not None else _require_cv2()
            yolo_cls = _require_ultralytics_yolo()
            _require_tracker_dependency()
            self.model = yolo_cls(str(path))
        else:
            self.cv2 = cv2_module
            self.model = model_factory(str(path))

    def detect(
        self,
        *,
        client: Any,
        config: AirSimDetectionConfig,
        frame_id: int,
        exposure_ts: float,
        active_name: Optional[str] = None,
        active_track_id: Optional[int] = None,
    ) -> DetectorFrame:
        del active_name
        stats: dict[str, Any] = {
            "detector_source": self.source,
            "detector_reject_reason": "",
            "detector_raw_count": 0,
            "detector_class_filtered_count": 0,
            "detector_track_filtered_count": 0,
            "yolo_raw_count": 0,
            "yolo_class_filtered_count": 0,
            "yolo_track_filtered_count": 0,
            "yolo_track_missing_count": 0,
            "yolo_selected_track_id": "",
            "yolo_selected_class_id": "",
            "yolo_selected_score": "",
            "yolo_selected_source": "",
            "yolo_requested_device": self.device or "auto",
            "yolo_runtime_device": self._runtime_device(),
            "yolo_cuda_available": int(bool(self.runtime_info.get("cuda_available", False))),
            "yolo_gpu_name": str(self.runtime_info.get("gpu_name", "")),
            "yolo_allow_untracked_fallback": int(self.allow_untracked_fallback),
            "yolo_used_untracked_fallback": 0,
            "yolo_single_target_mode": int(self.single_target_mode),
            "yolo_single_target_selected": 0,
            "yolo_single_target_distance_px": "",
        }
        image_bgr = self._read_scene_image(client, config)
        if image_bgr is None:
            stats["detector_reject_reason"] = "yolo_image_unavailable"
            return DetectorFrame([], None, None, stats, image_bgr=None)

        kwargs: dict[str, Any] = {
            "persist": True,
            "tracker": self.tracker,
            "conf": self.conf,
            "iou": self.iou,
            "imgsz": self.imgsz,
            "verbose": False,
        }
        if self.device:
            kwargs["device"] = self.device
        results = self.model.track(image_bgr, **kwargs)
        detections, parse_stats = self._detections_from_results(results, image_bgr.shape)
        stats.update(parse_stats)
        stats["detector_raw_count"] = stats["yolo_raw_count"]
        stats["detector_class_filtered_count"] = stats["yolo_class_filtered_count"]
        stats["detector_track_filtered_count"] = stats["yolo_track_filtered_count"]

        selected, selector_source, selector_distance_px = _select_yolo_detection(
            detections,
            active_track_id,
            allow_untracked_fallback=self.allow_untracked_fallback,
            single_target_mode=self.single_target_mode,
            last_center=self._last_selected_center,
            last_area=self._last_selected_area,
            max_center_jump_px=self.single_target_max_center_jump_px,
        )
        frame_detection = None
        if selected is not None and (selected.track_id is not None or self.allow_untracked_fallback):
            frame_track_id = int(selected.track_id) if selected.track_id is not None else -1
            selected_center = _bbox_center(selected)
            selected_area = _bbox_area(selected)
            frame_detection = detection_to_frame_detection(
                selected,
                frame_id=frame_id,
                exposure_ts=exposure_ts,
                track_id=frame_track_id,
                score=float(selected.score),
            )
            self._last_selected_center = selected_center
            self._last_selected_area = selected_area
            stats["yolo_selected_track_id"] = frame_track_id
            stats["yolo_selected_class_id"] = int(selected.class_id)
            stats["yolo_selected_score"] = float(selected.score)
            if selector_source == "single_target":
                stats["yolo_selected_source"] = "single_target"
                stats["yolo_single_target_selected"] = 1
                if selector_distance_px is not None:
                    stats["yolo_single_target_distance_px"] = float(selector_distance_px)
            else:
                stats["yolo_selected_source"] = "bytetrack" if selected.track_id is not None else "untracked_fallback"
            stats["yolo_used_untracked_fallback"] = int(selected.track_id is None)
        else:
            stats["detector_reject_reason"] = _yolo_reject_reason(stats)
        stats["yolo_runtime_device"] = self._runtime_device()

        return DetectorFrame(detections, selected, frame_detection, stats, image_bgr=image_bgr)

    def _runtime_device(self) -> str:
        try:
            model = getattr(self.model, "model", None)
            device = getattr(model, "device", None)
            if device is not None:
                return str(device)
            parameters = getattr(model, "parameters", None)
            if callable(parameters):
                return str(next(parameters()).device)
        except Exception:
            pass
        return str(self.device or "auto")

    def _read_scene_image(self, client: Any, config: AirSimDetectionConfig) -> Optional[np.ndarray]:
        if self.image_reader is not None:
            image = self.image_reader(client, config)
            return None if image is None else _ensure_bgr(image, self.cv2)
        if self.airsim_module is None:
            raise RuntimeError("airsim_module is required for YOLO image capture.")
        raw_image = client.simGetImage(
            config.camera_name,
            getattr(self.airsim_module.ImageType, config.image_type_name),
            vehicle_name=config.vehicle_name,
        )
        if raw_image is None:
            return None
        if isinstance(raw_image, str):
            raw_image = raw_image.encode("latin1")
        image = self.cv2.imdecode(np.frombuffer(raw_image, dtype=np.uint8), self.cv2.IMREAD_UNCHANGED)
        if image is None:
            return None
        return _ensure_bgr(image, self.cv2)

    def _detections_from_results(self, results: Any, image_shape: tuple[int, ...]) -> tuple[list[TrackedBoxDetection], dict[str, Any]]:
        return _detections_from_yolo_results(results, image_shape, self.class_id, self.source)


class YoloKcfDetector:
    source = "yolo_kcf"

    def __init__(
        self,
        *,
        model_path: str,
        class_id: Optional[int],
        conf: float = 0.25,
        iou: float = 0.70,
        imgsz: int = 640,
        device: str = "",
        single_target_mode: bool = False,
        single_target_max_center_jump_px: float = 220.0,
        yolo_period_n: int = 8,
        yolo_period_s: float = 0.5,
        max_coast_s: float = 0.8,
        min_yolo_iou: float = 0.25,
        max_center_jump_px: float = 180.0,
        area_ratio_min: float = 0.35,
        area_ratio_max: float = 2.8,
        reset_on_yolo_drift: bool = True,
        airsim_module: Any = None,
        model_factory: Optional[Callable[[str], Any]] = None,
        image_reader: Optional[Callable[[Any, AirSimDetectionConfig], np.ndarray]] = None,
        cv2_module: Any = None,
        tracker_factory: Optional[Callable[[], Any]] = None,
    ) -> None:
        if class_id is None:
            raise RuntimeError("--yolo-class-id is required when --detector-source yolo_kcf.")
        self.class_id = int(class_id)
        self.conf = max(0.0, float(conf))
        self.iou = max(0.0, float(iou))
        self.imgsz = max(1, int(imgsz))
        self.device = str(device or "")
        self.single_target_mode = bool(single_target_mode)
        self.single_target_max_center_jump_px = max(0.0, float(single_target_max_center_jump_px))
        self.yolo_period_n = max(1, int(yolo_period_n))
        self.yolo_period_s = max(0.0, float(yolo_period_s))
        self.max_coast_s = max(0.0, float(max_coast_s))
        self.min_yolo_iou = max(0.0, min(1.0, float(min_yolo_iou)))
        self.max_center_jump_px = max(0.0, float(max_center_jump_px))
        self.area_ratio_min = max(0.0, float(area_ratio_min))
        self.area_ratio_max = max(self.area_ratio_min, float(area_ratio_max))
        self.reset_on_yolo_drift = bool(reset_on_yolo_drift)
        self.airsim_module = airsim_module
        self.image_reader = image_reader
        self.runtime_info = _torch_runtime_info()
        self._last_selected_center: Optional[tuple[float, float]] = None
        self._last_selected_area: Optional[float] = None
        self._tracker: Optional[Any] = None
        self._tracker_factory = tracker_factory
        self._tracker_bbox_xyxy: Optional[tuple[float, float, float, float]] = None
        self._track_id: Optional[int] = None
        self._last_yolo_frame_id: Optional[int] = None
        self._last_yolo_ts: Optional[float] = None
        self._last_kcf_ts: Optional[float] = None

        path = Path(str(model_path or "")).expanduser()
        if not path.exists():
            raise RuntimeError(f"YOLO model file not found: {path}. Pass --yolo-model <path>.")
        self.model_path = path

        if model_factory is None:
            self.cv2 = cv2_module if cv2_module is not None else _require_cv2()
            yolo_cls = _require_ultralytics_yolo()
            if self._tracker_factory is None:
                _require_kcf_tracker(self.cv2)
            self.model = yolo_cls(str(path))
        else:
            self.cv2 = cv2_module
            self.model = model_factory(str(path))

    def detect(
        self,
        *,
        client: Any,
        config: AirSimDetectionConfig,
        frame_id: int,
        exposure_ts: float,
        active_name: Optional[str] = None,
        active_track_id: Optional[int] = None,
    ) -> DetectorFrame:
        del active_name
        stats = _base_yolo_stats(
            source=self.source,
            device=self.device,
            runtime_device=self._runtime_device(),
            runtime_info=self.runtime_info,
            allow_untracked_fallback=True,
            single_target_mode=self.single_target_mode,
        )
        stats.update(
            {
                "kcf_state": "uninitialized" if self._tracker is None else "tracking",
                "kcf_source": "",
                "kcf_update_ok": 0,
                "kcf_age_s": "",
                "kcf_since_yolo_s": "",
                "kcf_yolo_iou": "",
                "kcf_reject_reason": "",
                "kcf_yolo_period_n": self.yolo_period_n,
                "kcf_yolo_period_s": self.yolo_period_s,
                "kcf_max_coast_s": self.max_coast_s,
                "kcf_min_yolo_iou": self.min_yolo_iou,
                "kcf_max_center_jump_px": self.max_center_jump_px,
                "kcf_area_ratio_min": self.area_ratio_min,
                "kcf_area_ratio_max": self.area_ratio_max,
                "kcf_reset_on_yolo_drift": int(self.reset_on_yolo_drift),
            }
        )
        image_bgr = self._read_scene_image(client, config)
        if image_bgr is None:
            stats["detector_reject_reason"] = "yolo_image_unavailable"
            self._reset_tracker()
            return DetectorFrame([], None, None, stats, image_bgr=None)

        kcf_bbox, kcf_ok = self._update_tracker(image_bgr, image_bgr.shape)
        stats["kcf_update_ok"] = int(kcf_ok)
        if kcf_ok and kcf_bbox is not None:
            self._tracker_bbox_xyxy = kcf_bbox
            self._last_kcf_ts = exposure_ts
        elif self._tracker is not None:
            stats["kcf_reject_reason"] = "kcf_update_failed"
            self._reset_tracker()

        if self._last_kcf_ts is not None:
            stats["kcf_age_s"] = max(0.0, float(exposure_ts - self._last_kcf_ts))
        if self._last_yolo_ts is not None:
            stats["kcf_since_yolo_s"] = max(0.0, float(exposure_ts - self._last_yolo_ts))

        run_yolo = self._should_run_yolo(frame_id, exposure_ts)
        detections: list[TrackedBoxDetection] = []
        selected: Optional[TrackedBoxDetection] = None
        selector_source = ""
        selector_distance_px: Optional[float] = None
        yolo_frame_detection: Optional[FrameDetection] = None
        if run_yolo:
            detections, parse_stats = self._run_yolo(image_bgr)
            stats.update(parse_stats)
            stats["detector_raw_count"] = stats["yolo_raw_count"]
            stats["detector_class_filtered_count"] = stats["yolo_class_filtered_count"]
            selected, selector_source, selector_distance_px = _select_yolo_detection(
                detections,
                active_track_id if active_track_id is not None and active_track_id >= 0 else None,
                allow_untracked_fallback=True,
                single_target_mode=self.single_target_mode,
                last_center=self._last_selected_center,
                last_area=self._last_selected_area,
                max_center_jump_px=self.single_target_max_center_jump_px,
            )
            if selected is not None:
                yolo_frame_detection = self._frame_detection_from_selection(selected, frame_id, exposure_ts)
                self._log_yolo_selection(stats, selected, yolo_frame_detection.track_id, selector_source, selector_distance_px)
                self._last_yolo_frame_id = frame_id
                self._last_yolo_ts = exposure_ts

        frame_detection: Optional[FrameDetection] = None
        output_detection: Optional[TrackedBoxDetection] = None
        if yolo_frame_detection is not None and selected is not None:
            accept_yolo, reject_reason, yolo_iou = self._accept_yolo_correction(selected, kcf_bbox)
            stats["kcf_yolo_iou"] = "" if yolo_iou is None else yolo_iou
            if accept_yolo or self._tracker is None or not kcf_ok or self.reset_on_yolo_drift:
                frame_detection = yolo_frame_detection
                output_detection = selected
                self._init_tracker(image_bgr, frame_detection.bbox_xyxy)
                self._track_id = int(frame_detection.track_id)
                self._last_selected_center = frame_detection.center
                self._last_selected_area = frame_detection.area
                stats["kcf_state"] = "correction" if kcf_ok and accept_yolo else "tracking"
                stats["kcf_source"] = "yolo_correct" if kcf_ok and accept_yolo else "yolo_reinit"
                if reject_reason and self.reset_on_yolo_drift:
                    stats["kcf_reject_reason"] = reject_reason
                    stats["kcf_state"] = "reset"
            else:
                stats["kcf_reject_reason"] = reject_reason
                frame_detection, output_detection = self._kcf_frame_detection(frame_id, exposure_ts, image_bgr.shape)
                stats["kcf_state"] = "coast" if frame_detection is not None else "lost"
                stats["kcf_source"] = "kcf_track" if frame_detection is not None else ""
        elif kcf_ok:
            frame_detection, output_detection = self._kcf_frame_detection(frame_id, exposure_ts, image_bgr.shape)
            stats["kcf_state"] = "tracking"
            stats["kcf_source"] = "kcf_track" if frame_detection is not None else ""
        elif yolo_frame_detection is None:
            stats["detector_reject_reason"] = _yolo_reject_reason(stats) if run_yolo else "kcf_update_failed"

        if frame_detection is not None and output_detection is not None:
            stats["detector_reject_reason"] = ""
            stats["detector_track_filtered_count"] = 1
            if not stats["yolo_selected_source"] and stats["kcf_source"] == "kcf_track":
                stats["yolo_selected_source"] = "kcf_track"
            if output_detection not in detections:
                detections = [output_detection, *detections]
            stats["detector_raw_count"] = max(int(stats.get("detector_raw_count") or 0), len(detections))
            stats["detector_class_filtered_count"] = max(int(stats.get("detector_class_filtered_count") or 0), len(detections))
            return DetectorFrame(detections, output_detection, frame_detection, stats, image_bgr=image_bgr)

        if self._last_yolo_ts is not None and exposure_ts - self._last_yolo_ts > self.max_coast_s:
            stats["kcf_state"] = "lost"
            stats["kcf_reject_reason"] = stats["kcf_reject_reason"] or "kcf_coast_timeout"
            self._reset_tracker()
        stats["detector_reject_reason"] = stats["detector_reject_reason"] or stats["kcf_reject_reason"] or "no_detection"
        return DetectorFrame(detections, None, None, stats, image_bgr=image_bgr)

    def _runtime_device(self) -> str:
        try:
            model = getattr(self.model, "model", None)
            device = getattr(model, "device", None)
            if device is not None:
                return str(device)
            parameters = getattr(model, "parameters", None)
            if callable(parameters):
                return str(next(parameters()).device)
        except Exception:
            pass
        return str(self.device or "auto")

    def _read_scene_image(self, client: Any, config: AirSimDetectionConfig) -> Optional[np.ndarray]:
        if self.image_reader is not None:
            image = self.image_reader(client, config)
            return None if image is None else _ensure_bgr(image, self.cv2)
        if self.airsim_module is None:
            raise RuntimeError("airsim_module is required for YOLO image capture.")
        raw_image = client.simGetImage(
            config.camera_name,
            getattr(self.airsim_module.ImageType, config.image_type_name),
            vehicle_name=config.vehicle_name,
        )
        if raw_image is None:
            return None
        if isinstance(raw_image, str):
            raw_image = raw_image.encode("latin1")
        image = self.cv2.imdecode(np.frombuffer(raw_image, dtype=np.uint8), self.cv2.IMREAD_UNCHANGED)
        if image is None:
            return None
        return _ensure_bgr(image, self.cv2)

    def _run_yolo(self, image_bgr: np.ndarray) -> tuple[list[TrackedBoxDetection], dict[str, Any]]:
        kwargs: dict[str, Any] = {
            "conf": self.conf,
            "iou": self.iou,
            "imgsz": self.imgsz,
            "verbose": False,
        }
        if self.device:
            kwargs["device"] = self.device
        predict = getattr(self.model, "predict", None)
        if callable(predict):
            results = predict(image_bgr, **kwargs)
        else:
            results = self.model(image_bgr, **kwargs)
        return _detections_from_yolo_results(results, image_bgr.shape, self.class_id, self.source)

    def _should_run_yolo(self, frame_id: int, exposure_ts: float) -> bool:
        if self._tracker is None or self._tracker_bbox_xyxy is None:
            return True
        if self._last_yolo_frame_id is None or frame_id - self._last_yolo_frame_id >= self.yolo_period_n:
            return True
        if self.yolo_period_s > 0.0 and self._last_yolo_ts is not None and exposure_ts - self._last_yolo_ts >= self.yolo_period_s:
            return True
        if self._last_yolo_ts is not None and exposure_ts - self._last_yolo_ts >= self.max_coast_s:
            return True
        return False

    def _update_tracker(self, image_bgr: np.ndarray, image_shape: tuple[int, ...]) -> tuple[Optional[tuple[float, float, float, float]], bool]:
        if self._tracker is None:
            return None, False
        try:
            ok, bbox_xywh = self._tracker.update(image_bgr)
        except Exception:
            return None, False
        if not ok:
            return None, False
        bbox_xyxy = _xywh_to_xyxy(bbox_xywh)
        bbox_xyxy = _clamp_bbox(bbox_xyxy, image_shape)
        if _bbox_area_xyxy(bbox_xyxy) <= 1.0:
            return None, False
        return bbox_xyxy, True

    def _init_tracker(self, image_bgr: np.ndarray, bbox_xyxy: tuple[float, float, float, float]) -> None:
        tracker = self._new_tracker()
        tracker.init(image_bgr, _xyxy_to_xywh(bbox_xyxy))
        self._tracker = tracker
        self._tracker_bbox_xyxy = tuple(float(value) for value in bbox_xyxy)

    def _new_tracker(self) -> Any:
        if self._tracker_factory is not None:
            return self._tracker_factory()
        return _create_kcf_tracker(self.cv2)

    def _reset_tracker(self) -> None:
        self._tracker = None
        self._tracker_bbox_xyxy = None
        self._last_kcf_ts = None

    def _accept_yolo_correction(
        self,
        selected: TrackedBoxDetection,
        kcf_bbox: Optional[tuple[float, float, float, float]],
    ) -> tuple[bool, str, Optional[float]]:
        if kcf_bbox is None:
            return True, "", None
        yolo_bbox = _detection_bbox_xyxy(selected)
        iou = _bbox_iou_xyxy(yolo_bbox, kcf_bbox)
        if iou < self.min_yolo_iou:
            return False, "kcf_yolo_iou_low", iou
        yolo_center = _bbox_center_xyxy(yolo_bbox)
        kcf_center = _bbox_center_xyxy(kcf_bbox)
        center_jump = float(np.hypot(yolo_center[0] - kcf_center[0], yolo_center[1] - kcf_center[1]))
        if self.max_center_jump_px > 0.0 and center_jump > self.max_center_jump_px:
            return False, "kcf_yolo_center_jump"
        kcf_area = max(1.0e-6, _bbox_area_xyxy(kcf_bbox))
        yolo_area = max(1.0e-6, _bbox_area_xyxy(yolo_bbox))
        area_ratio = yolo_area / kcf_area
        if area_ratio < self.area_ratio_min or area_ratio > self.area_ratio_max:
            return False, "kcf_yolo_area_jump"
        return True, "", iou

    def _frame_detection_from_selection(
        self,
        selected: TrackedBoxDetection,
        frame_id: int,
        exposure_ts: float,
    ) -> FrameDetection:
        track_id = int(selected.track_id) if selected.track_id is not None else -2
        return detection_to_frame_detection(
            selected,
            frame_id=frame_id,
            exposure_ts=exposure_ts,
            track_id=track_id,
            score=float(selected.score),
        )

    def _kcf_frame_detection(
        self,
        frame_id: int,
        exposure_ts: float,
        image_shape: tuple[int, ...],
    ) -> tuple[Optional[FrameDetection], Optional[TrackedBoxDetection]]:
        if self._tracker_bbox_xyxy is None:
            return None, None
        x1, y1, x2, y2 = _clamp_bbox(self._tracker_bbox_xyxy, image_shape)
        track_id = int(self._track_id) if self._track_id is not None else -2
        detection = TrackedBoxDetection(
            name=f"kcf_track{track_id}",
            box2D=_Box2D(_Point2D(x1, y1), _Point2D(x2, y2)),
            score=0.50,
            track_id=track_id,
            class_id=self.class_id,
            source=self.source,
        )
        frame_detection = detection_to_frame_detection(
            detection,
            frame_id=frame_id,
            exposure_ts=exposure_ts,
            track_id=track_id,
            score=float(detection.score),
        )
        self._last_selected_center = frame_detection.center
        self._last_selected_area = frame_detection.area
        return frame_detection, detection

    @staticmethod
    def _log_yolo_selection(
        stats: dict[str, Any],
        selected: TrackedBoxDetection,
        frame_track_id: int,
        selector_source: str,
        selector_distance_px: Optional[float],
    ) -> None:
        stats["yolo_selected_track_id"] = frame_track_id
        stats["yolo_selected_class_id"] = int(selected.class_id)
        stats["yolo_selected_score"] = float(selected.score)
        stats["yolo_selected_source"] = selector_source or "yolo"
        stats["yolo_used_untracked_fallback"] = int(selected.track_id is None)
        if selector_source == "single_target":
            stats["yolo_single_target_selected"] = 1
            if selector_distance_px is not None:
                stats["yolo_single_target_distance_px"] = float(selector_distance_px)


def _select_yolo_detection(
    detections: list[TrackedBoxDetection],
    active_track_id: Optional[int],
    *,
    allow_untracked_fallback: bool = False,
    single_target_mode: bool = False,
    last_center: Optional[tuple[float, float]] = None,
    last_area: Optional[float] = None,
    max_center_jump_px: float = 220.0,
) -> tuple[Optional[TrackedBoxDetection], str, Optional[float]]:
    tracked = [detection for detection in detections if detection.track_id is not None]
    if active_track_id is not None:
        for detection in tracked:
            if int(detection.track_id) == int(active_track_id):
                return detection, "bytetrack", None
    if single_target_mode and detections:
        selected, distance_px = _select_single_target_continuity(
            detections,
            last_center=last_center,
            last_area=last_area,
            max_center_jump_px=max_center_jump_px,
        )
        if selected is not None:
            return selected, "single_target", distance_px
    if tracked:
        return max(tracked, key=lambda detection: float(detection.score)), "bytetrack", None
    if not allow_untracked_fallback:
        return None, "", None
    return max(detections, key=lambda detection: float(detection.score), default=None), "untracked_fallback", None


def _base_yolo_stats(
    *,
    source: str,
    device: str,
    runtime_device: str,
    runtime_info: dict[str, Any],
    allow_untracked_fallback: bool,
    single_target_mode: bool,
) -> dict[str, Any]:
    return {
        "detector_source": source,
        "detector_reject_reason": "",
        "detector_raw_count": 0,
        "detector_class_filtered_count": 0,
        "detector_track_filtered_count": 0,
        "yolo_raw_count": 0,
        "yolo_class_filtered_count": 0,
        "yolo_track_filtered_count": 0,
        "yolo_track_missing_count": 0,
        "yolo_selected_track_id": "",
        "yolo_selected_class_id": "",
        "yolo_selected_score": "",
        "yolo_selected_source": "",
        "yolo_requested_device": device or "auto",
        "yolo_runtime_device": runtime_device,
        "yolo_cuda_available": int(bool(runtime_info.get("cuda_available", False))),
        "yolo_gpu_name": str(runtime_info.get("gpu_name", "")),
        "yolo_allow_untracked_fallback": int(allow_untracked_fallback),
        "yolo_used_untracked_fallback": 0,
        "yolo_single_target_mode": int(single_target_mode),
        "yolo_single_target_selected": 0,
        "yolo_single_target_distance_px": "",
    }


def _detections_from_yolo_results(
    results: Any,
    image_shape: tuple[int, ...],
    class_id_filter: int,
    source: str,
) -> tuple[list[TrackedBoxDetection], dict[str, Any]]:
    stats: dict[str, Any] = {
        "yolo_raw_count": 0,
        "yolo_class_filtered_count": 0,
        "yolo_track_filtered_count": 0,
        "yolo_track_missing_count": 0,
    }
    if results is None:
        return [], stats
    result_items = list(results) if isinstance(results, (list, tuple)) else [results]
    detections: list[TrackedBoxDetection] = []
    for result in result_items:
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            continue
        xyxy = _as_numpy(getattr(boxes, "xyxy", None))
        if xyxy.size == 0:
            continue
        if xyxy.ndim == 1:
            xyxy = xyxy.reshape(1, -1)
        conf = _as_numpy(getattr(boxes, "conf", None))
        cls = _as_numpy(getattr(boxes, "cls", None))
        track_ids = _as_optional_numpy(getattr(boxes, "id", None))
        stats["yolo_raw_count"] += int(xyxy.shape[0])
        for index, bbox in enumerate(xyxy):
            class_id = _array_int(cls, index, default=-1)
            if class_id != class_id_filter:
                continue
            stats["yolo_class_filtered_count"] += 1
            score = _array_float(conf, index, default=0.0)
            track_id = None if track_ids is None else _array_int(track_ids, index, default=None)
            if track_id is None:
                stats["yolo_track_missing_count"] += 1
            else:
                stats["yolo_track_filtered_count"] += 1
            x1, y1, x2, y2 = _clamp_bbox(bbox, image_shape)
            name = (
                f"yolo_cls{class_id}_track{track_id}"
                if track_id is not None
                else f"yolo_cls{class_id}_no_track"
            )
            detections.append(
                TrackedBoxDetection(
                    name=name,
                    box2D=_Box2D(_Point2D(x1, y1), _Point2D(x2, y2)),
                    score=score,
                    track_id=track_id,
                    class_id=class_id,
                    source=source,
                )
            )
    return detections, stats


def _select_single_target_continuity(
    detections: list[TrackedBoxDetection],
    *,
    last_center: Optional[tuple[float, float]],
    last_area: Optional[float],
    max_center_jump_px: float,
) -> tuple[Optional[TrackedBoxDetection], Optional[float]]:
    if not detections:
        return None, None
    if last_center is None:
        return max(detections, key=lambda detection: float(detection.score), default=None), None

    best_detection: Optional[TrackedBoxDetection] = None
    best_cost = float("inf")
    best_distance_px: Optional[float] = None
    max_jump = max(0.0, float(max_center_jump_px))
    for detection in detections:
        center = _bbox_center(detection)
        center_dist = float(np.hypot(center[0] - last_center[0], center[1] - last_center[1]))
        if max_jump > 0.0 and center_dist > max_jump:
            continue
        area_cost = 0.0
        if last_area is not None and last_area > 1.0e-6:
            area_ratio = max(1.0e-6, _bbox_area(detection)) / max(1.0e-6, last_area)
            area_cost = abs(float(np.log(area_ratio)))
        score_bonus = 30.0 * float(detection.score)
        cost = center_dist + 40.0 * area_cost - score_bonus
        if cost < best_cost:
            best_cost = cost
            best_detection = detection
            best_distance_px = center_dist
    return best_detection, best_distance_px


def _bbox_center(detection: TrackedBoxDetection) -> tuple[float, float]:
    return (
        0.5 * (float(detection.box2D.min.x_val) + float(detection.box2D.max.x_val)),
        0.5 * (float(detection.box2D.min.y_val) + float(detection.box2D.max.y_val)),
    )


def _bbox_area(detection: TrackedBoxDetection) -> float:
    width = max(0.0, float(detection.box2D.max.x_val) - float(detection.box2D.min.x_val))
    height = max(0.0, float(detection.box2D.max.y_val) - float(detection.box2D.min.y_val))
    return width * height


def _detection_bbox_xyxy(detection: TrackedBoxDetection) -> tuple[float, float, float, float]:
    return (
        float(detection.box2D.min.x_val),
        float(detection.box2D.min.y_val),
        float(detection.box2D.max.x_val),
        float(detection.box2D.max.y_val),
    )


def _xywh_to_xyxy(bbox_xywh: Any) -> tuple[float, float, float, float]:
    values = np.ravel(np.asarray(bbox_xywh, dtype=float))
    if values.size < 4:
        return 0.0, 0.0, 1.0, 1.0
    x, y, width, height = [float(value) for value in values[:4]]
    return x, y, x + max(0.0, width), y + max(0.0, height)


def _xyxy_to_xywh(bbox_xyxy: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = [float(value) for value in bbox_xyxy]
    return x1, y1, max(1.0, x2 - x1), max(1.0, y2 - y1)


def _bbox_center_xyxy(bbox_xyxy: tuple[float, float, float, float]) -> tuple[float, float]:
    x1, y1, x2, y2 = [float(value) for value in bbox_xyxy]
    return 0.5 * (x1 + x2), 0.5 * (y1 + y2)


def _bbox_area_xyxy(bbox_xyxy: tuple[float, float, float, float]) -> float:
    x1, y1, x2, y2 = [float(value) for value in bbox_xyxy]
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def _bbox_iou_xyxy(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> float:
    ax1, ay1, ax2, ay2 = [float(value) for value in a]
    bx1, by1, bx2, by2 = [float(value) for value in b]
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    union = _bbox_area_xyxy(a) + _bbox_area_xyxy(b) - intersection
    if union <= 1.0e-9:
        return 0.0
    return float(intersection / union)


def _yolo_reject_reason(stats: dict[str, Any]) -> str:
    if int(stats.get("yolo_raw_count") or 0) <= 0:
        return "no_detection"
    if int(stats.get("yolo_class_filtered_count") or 0) <= 0:
        return "yolo_class_missing"
    if int(stats.get("yolo_track_filtered_count") or 0) <= 0:
        return "yolo_track_id_missing"
    return "no_detection"


def _as_numpy(value: Any) -> np.ndarray:
    if value is None:
        return np.asarray([])
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def _as_optional_numpy(value: Any) -> Optional[np.ndarray]:
    if value is None:
        return None
    array = _as_numpy(value)
    if array.size == 0:
        return None
    return array


def _array_float(array: np.ndarray, index: int, default: float) -> float:
    try:
        return float(np.ravel(array)[index])
    except (IndexError, TypeError, ValueError):
        return float(default)


def _array_int(array: np.ndarray, index: int, default: Optional[int]) -> Optional[int]:
    try:
        value = float(np.ravel(array)[index])
    except (IndexError, TypeError, ValueError):
        return default
    if not np.isfinite(value):
        return default
    return int(round(value))


def _clamp_bbox(bbox: Any, image_shape: tuple[int, ...]) -> tuple[float, float, float, float]:
    height = float(image_shape[0]) if len(image_shape) >= 1 else 0.0
    width = float(image_shape[1]) if len(image_shape) >= 2 else 0.0
    values = np.ravel(np.asarray(bbox, dtype=float))
    if values.size < 4:
        return 0.0, 0.0, 1.0, 1.0
    x1, y1, x2, y2 = [float(value) for value in values[:4]]
    if width > 0.0:
        x1 = float(np.clip(x1, 0.0, width))
        x2 = float(np.clip(x2, 0.0, width))
    if height > 0.0:
        y1 = float(np.clip(y1, 0.0, height))
        y2 = float(np.clip(y2, 0.0, height))
    if x2 <= x1:
        x2 = min(width if width > 0.0 else x1 + 1.0, x1 + 1.0)
    if y2 <= y1:
        y2 = min(height if height > 0.0 else y1 + 1.0, y1 + 1.0)
    return x1, y1, x2, y2


def _ensure_bgr(image: np.ndarray, cv2: Any) -> np.ndarray:
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if image.ndim == 3 and image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return np.asarray(image)


def _require_cv2() -> Any:
    try:
        return importlib.import_module("cv2")
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV is required for YOLO detector sources. Install dependencies with: "
            "python3 -m pip install torch ultralytics lap opencv-contrib-python"
        ) from exc


def _create_kcf_tracker(cv2: Any) -> Any:
    legacy = getattr(cv2, "legacy", None)
    factory = getattr(legacy, "TrackerKCF_create", None) if legacy is not None else None
    if factory is None:
        factory = getattr(cv2, "TrackerKCF_create", None)
    if factory is None:
        raise RuntimeError(
            "KCF tracker is unavailable in this OpenCV build. Install opencv-contrib-python, "
            "for example: python3 -m pip install opencv-contrib-python --break-system-packages"
        )
    return factory()


def _require_kcf_tracker(cv2: Any) -> None:
    legacy = getattr(cv2, "legacy", None)
    if legacy is not None and getattr(legacy, "TrackerKCF_create", None) is not None:
        return
    if getattr(cv2, "TrackerKCF_create", None) is not None:
        return
    raise RuntimeError(
        "KCF tracker is unavailable in this OpenCV build. Install opencv-contrib-python, "
        "for example: python3 -m pip install opencv-contrib-python --break-system-packages"
    )


def _require_ultralytics_yolo() -> Any:
    missing: list[str] = []
    for module_name, package_name in (("torch", "torch"), ("ultralytics", "ultralytics")):
        try:
            importlib.import_module(module_name)
        except ImportError:
            missing.append(package_name)
    if missing:
        raise RuntimeError(
            "Missing YOLO/ByteTrack dependencies: "
            + ", ".join(missing)
            + ". Install with: python3 -m pip install torch ultralytics lap"
        )
    from ultralytics import YOLO  # type: ignore

    return YOLO


def _torch_runtime_info() -> dict[str, Any]:
    try:
        torch = importlib.import_module("torch")
        cuda_available = bool(torch.cuda.is_available())
        gpu_name = str(torch.cuda.get_device_name(0)) if cuda_available and torch.cuda.device_count() > 0 else ""
        return {
            "version": str(getattr(torch, "__version__", "")),
            "cuda_available": cuda_available,
            "device_count": int(torch.cuda.device_count()) if hasattr(torch, "cuda") else 0,
            "gpu_name": gpu_name,
        }
    except Exception:
        return {"version": "", "cuda_available": False, "device_count": 0, "gpu_name": ""}


def _require_tracker_dependency() -> None:
    for module_name in ("lap", "lapx"):
        try:
            importlib.import_module(module_name)
            return
        except ImportError:
            continue
    raise RuntimeError(
        "Missing ByteTrack assignment dependency: install lap or lapx, for example "
        "python3 -m pip install lap"
    )
