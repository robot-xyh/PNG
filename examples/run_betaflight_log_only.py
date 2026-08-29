from __future__ import annotations

import argparse
import csv
import gc
import hashlib
import importlib
import json
import multiprocessing
import os
import platform
import queue
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vision_guidance.airsim_adapter import AirSimDetectionConfig  # noqa: E402
from vision_guidance.attitude_buffer import AttitudeHistoryBuffer  # noqa: E402
from vision_guidance.betaflight_msp import (  # noqa: E402
    MSP_ANALOG,
    MSP_ATTITUDE,
    MSP_MOTOR,
    MSP_RAW_IMU,
    MSP_RC,
    MSP_SET_RAW_RC,
    MSP_STATUS,
    BetaflightMSPAdapter,
    BetaflightTelemetry,
)
from vision_guidance.betaflight_runtime import (  # noqa: E402
    MSP_OVERRIDE_PERMANENT_ID,
    BetaflightMspIoWorker,
    MspRuntimeConfig,
    armed_from_telemetry,
    box_mode_active,
    box_mode_index,
    resolve_control_authorization,
)
from vision_guidance.betaflight_web import (  # noqa: E402
    TelemetryWebConfig,
    TelemetryWebService,
    telemetry_payload_from_log_row,
)
from vision_guidance.flight_control import (  # noqa: E402
    BetaflightSafetyStateMachine,
    CommandWatchdog,
    GuidanceCommandShaper,
    GuidanceCommandShaperConfig,
    GuidanceCommandShapingDiagnostics,
    GuidanceSetpoint,
    GuidanceSetpointHold,
    MotorOutputInterlock,
    MotorOutputInterlockConfig,
    RcCommand,
    RcCommandMapper,
    RcMappingConfig,
    SafetyInputs,
    aux_range_enabled,
    guidance_eval_to_setpoint,
    inertial_vector_to_body_frd,
)
from vision_guidance.fusion import (  # noqa: E402
    DeferredAttitudeFusion,
    PureVisionGuidancePipeline,
    VisionGuidanceResult,
)
from vision_guidance.geometry import (  # noqa: E402
    camera_mount_diagnostics,
    camera_to_body_mount,
    validated_rotation_matrix,
)
from vision_guidance.platform_health import PlatformHealthSampler  # noqa: E402
from vision_guidance.png_eval import FixedVmGuidanceEvaluator, GuidanceEvaluator  # noqa: E402
from vision_guidance.rknn_native_detector import RknnDetectorConfig, RknnNativeDetector  # noqa: E402
from vision_guidance.rknn_bytetrack_detector import RknnByteTrackDetector  # noqa: E402
from vision_guidance.bytetrack_adapter import ByteTrackConfig  # noqa: E402
from vision_guidance.types import AttitudeSample, CameraIntrinsics, FrameDetection  # noqa: E402
from vision_guidance.yolo_bytetrack_detector import YoloByteTrackDetector  # noqa: E402


LOG_SCHEMA_VERSION = 13
GUIDANCE_EVAL_FRAME = "inertial_ned"
RATE_GAIN_INPUT_FRAME = "body_frd"
MSP_COMMAND_LOG_SPECS = (
    ("status", MSP_STATUS),
    ("raw_imu", MSP_RAW_IMU),
    ("motor", MSP_MOTOR),
    ("rc", MSP_RC),
    ("attitude", MSP_ATTITUDE),
    ("analog", MSP_ANALOG),
    ("set_raw_rc", MSP_SET_RAW_RC),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Betaflight MSP telemetry logging with supervised PNG RC candidates.")
    parser.add_argument("--config", default="config/betaflight.example.json")
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--log-dir", default="logs")
    parser.add_argument("--log-prefix", default="betaflight_log")
    parser.add_argument("--serial-port", default="", help="Override config serial.port.")
    parser.add_argument("--msp-baud", type=int, default=0, help="Override config serial.baud.")
    parser.add_argument("--control-mode", choices=("log_only", "msp_raw_rc"), default="log_only")
    parser.add_argument("--allow-control", action="store_true", help="Required before MSP_SET_RAW_RC is sent.")
    parser.add_argument(
        "--detector-source",
        choices=("none", "csv", "camera_only", "yolo_bytetrack", "rknn_native", "rknn_bytetrack"),
        default="none",
    )
    parser.add_argument("--detections-csv", default="", help="CSV with x1,y1,x2,y2 and optional exposure_ts,track_id,score.")
    parser.add_argument("--camera-device", default="", help="Camera index or device path; overrides camera.device.")
    parser.add_argument("--yolo-model", default="")
    parser.add_argument("--yolo-class-id", type=int, default=None)
    parser.add_argument("--yolo-conf", type=float, default=0.25)
    parser.add_argument("--yolo-iou", type=float, default=0.70)
    parser.add_argument("--yolo-imgsz", type=int, default=640)
    parser.add_argument("--yolo-device", default="")
    parser.add_argument("--yolo-tracker", default="bytetrack.yaml")
    parser.add_argument("--yolo-allow-untracked-fallback", action="store_true")
    parser.add_argument("--rknn-library", default="", help="Override rknn_detector.library.")
    parser.add_argument("--rknn-model", default="", help="Override rknn_detector.model.")
    parser.add_argument(
        "--rknn-perception-rate-hz",
        type=float,
        default=0.0,
        help="Override rknn_bytetrack.perception_rate_hz; 0 uses the config value.",
    )
    parser.add_argument(
        "--disable-web-preview",
        action="store_true",
        help="Disable MJPEG encoding while retaining JSON/SSE telemetry.",
    )
    parser.add_argument(
        "--isolate-rknn-process",
        action="store_true",
        help="Run RKNN+ByteTrack in a spawned process so perception cannot hold the MSP worker GIL.",
    )
    parser.add_argument(
        "--main-cpu-affinity",
        default="",
        help="Linux CPU list for the main process and MSP/Web threads, for example 6,7.",
    )
    parser.add_argument(
        "--rknn-cpu-affinity",
        default="",
        help="Linux CPU list for the isolated RKNN process, for example 4,5.",
    )
    return parser.parse_args()


class EdgeEventLogger:
    def __init__(self, path: Path, *, start_s: float):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.start_s = float(start_s)
        self._stream = path.open("w", encoding="utf-8")
        self._previous: dict[str, Any] = {}

    def write(self, event: str, *, timestamp_s: float, old: Any = None, new: Any = None, context=None) -> None:
        record = {
            "schema_version": LOG_SCHEMA_VERSION,
            "monotonic_s": float(timestamp_s),
            "elapsed_s": max(0.0, float(timestamp_s) - self.start_s),
            "event": str(event),
            "old": old,
            "new": new,
            "context": dict(context or {}),
        }
        self._stream.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")
        self._stream.flush()

    def update(self, values: dict[str, Any], *, timestamp_s: float, context=None) -> None:
        for event, value in values.items():
            if event not in self._previous or self._previous[event] != value:
                self.write(
                    event,
                    timestamp_s=timestamp_s,
                    old=self._previous.get(event),
                    new=value,
                    context=context,
                )
                self._previous[event] = value

    def close(self) -> None:
        self._stream.close()


class PythonGcPauseMonitor:
    def __init__(self, *, clock=time.perf_counter):
        self._clock = clock
        self._lock = threading.RLock()
        self._starts: dict[int, float] = {}
        self._collection_count = 0
        self._last_generation: int | None = None
        self._last_pause_ms: float | None = None
        self._max_pause_ms: float | None = None
        self._total_pause_ms = 0.0
        self._callback_ref = self._callback
        self._registered = False

    def start(self) -> None:
        with self._lock:
            if self._registered:
                return
            gc.callbacks.append(self._callback_ref)
            self._registered = True

    def close(self) -> None:
        with self._lock:
            if not self._registered:
                return
            try:
                gc.callbacks.remove(self._callback_ref)
            except ValueError:
                pass
            self._registered = False
            self._starts.clear()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "python_gc_collection_count": self._collection_count,
                "python_gc_last_generation": (
                    "" if self._last_generation is None else self._last_generation
                ),
                "python_gc_last_pause_ms": (
                    "" if self._last_pause_ms is None else self._last_pause_ms
                ),
                "python_gc_max_pause_ms": "" if self._max_pause_ms is None else self._max_pause_ms,
                "python_gc_total_pause_ms": self._total_pause_ms,
            }

    @staticmethod
    def metadata() -> dict[str, Any]:
        return {"python_gc_pause_monitor": True, "clock": "time.perf_counter"}

    def _callback(self, phase: str, info: dict[str, Any]) -> None:
        generation = int(info.get("generation", -1))
        timestamp = float(self._clock())
        with self._lock:
            if phase == "start":
                self._starts[generation] = timestamp
                return
            if phase != "stop":
                return
            started = self._starts.pop(generation, None)
            if started is None:
                return
            pause_ms = 1000.0 * max(0.0, timestamp - started)
            self._collection_count += 1
            self._last_generation = generation
            self._last_pause_ms = pause_ms
            self._max_pause_ms = pause_ms if self._max_pause_ms is None else max(self._max_pause_ms, pause_ms)
            self._total_pause_ms += pause_ms


class DetectionCsvSource:
    def __init__(self, path: str):
        csv_path = Path(path).expanduser()
        if not csv_path.exists():
            raise RuntimeError(f"detections CSV not found: {csv_path}")
        with csv_path.open("r", newline="") as stream:
            self.rows = list(csv.DictReader(stream))
        self.index = 0

    def detect(self, *, elapsed_s: float, timestamp: float, frame_id: int) -> tuple[FrameDetection | None, dict[str, Any]]:
        if self.index >= len(self.rows):
            return None, {"detector_source": "csv", "detector_reject_reason": "csv_exhausted"}
        row = self.rows[self.index]
        row_ts = _float_or_none(row.get("exposure_ts"))
        if row_ts is not None and row_ts > elapsed_s:
            return None, {"detector_source": "csv", "detector_reject_reason": "csv_waiting"}
        self.index += 1
        detection = FrameDetection(
            frame_id=int(float(row.get("frame_id") or frame_id)),
            exposure_ts=float(timestamp),
            bbox_xyxy=(
                float(row["x1"]),
                float(row["y1"]),
                float(row["x2"]),
                float(row["y2"]),
            ),
            track_id=int(float(row.get("track_id") or 1)),
            score=float(row.get("score") or 1.0),
        )
        return detection, {"detector_source": "csv", "detector_reject_reason": ""}


class OpenCvCameraSource:
    def __init__(
        self,
        args: argparse.Namespace,
        config: dict[str, Any],
        *,
        cv2_module: Any = None,
        capture_factory: Any = None,
        preview_sink: Any = None,
    ) -> None:
        self.cv2 = cv2_module if cv2_module is not None else importlib.import_module("cv2")
        self.camera_config = dict(config.get("camera", {}))
        configured_device = self.camera_config.get("device", 0)
        requested_device = getattr(args, "camera_device", "")
        self.device = _camera_device_value(requested_device if requested_device not in (None, "") else configured_device)
        factory = capture_factory if capture_factory is not None else self.cv2.VideoCapture
        self.capture = factory(self.device)
        if not self.capture.isOpened():
            raise RuntimeError(f"failed to open camera device {self.device}")

        self.capture_width = int(self.camera_config.get("capture_width", self.camera_config.get("width", 640)))
        self.capture_height = int(self.camera_config.get("capture_height", self.camera_config.get("height", 480)))
        self.output_width = int(self.camera_config.get("width", self.capture_width))
        self.output_height = int(self.camera_config.get("height", self.capture_height))
        self.requested_fps = float(self.camera_config.get("fps", 0.0))
        self.requested_fourcc = str(self.camera_config.get("fourcc", "") or "").upper()
        self.buffer_size = int(self.camera_config.get("buffer_size", 1))
        self.failed_frames = 0
        self.last_image = None
        self.preview_sink = preview_sink
        self.last_stats = self._empty_stats()
        self._configure_capture()

    def _configure_capture(self) -> None:
        self.capture.set(self.cv2.CAP_PROP_FRAME_WIDTH, float(self.capture_width))
        self.capture.set(self.cv2.CAP_PROP_FRAME_HEIGHT, float(self.capture_height))
        if self.requested_fps > 0.0:
            self.capture.set(self.cv2.CAP_PROP_FPS, self.requested_fps)
        if self.requested_fourcc:
            if len(self.requested_fourcc) != 4:
                raise ValueError("camera.fourcc must contain exactly four characters")
            fourcc = self.cv2.VideoWriter_fourcc(*self.requested_fourcc)
            self.capture.set(self.cv2.CAP_PROP_FOURCC, float(fourcc))
        if self.buffer_size > 0 and hasattr(self.cv2, "CAP_PROP_BUFFERSIZE"):
            self.capture.set(self.cv2.CAP_PROP_BUFFERSIZE, float(self.buffer_size))

        actual_width = int(round(float(self.capture.get(self.cv2.CAP_PROP_FRAME_WIDTH))))
        actual_height = int(round(float(self.capture.get(self.cv2.CAP_PROP_FRAME_HEIGHT))))
        if actual_width > 0 and actual_width != self.capture_width:
            raise RuntimeError(f"camera width mismatch: requested {self.capture_width}, got {actual_width}")
        if actual_height > 0 and actual_height != self.capture_height:
            raise RuntimeError(f"camera height mismatch: requested {self.capture_height}, got {actual_height}")

    def close(self) -> None:
        self.capture.release()

    def detect(
        self,
        *,
        timestamp: float,
        frame_id: int,
        active_track_id: int | None,
    ) -> tuple[FrameDetection | None, dict[str, Any]]:
        del timestamp, frame_id, active_track_id
        image = self.read_image()
        stats = dict(self.last_stats)
        stats["detector_source"] = "camera_only"
        stats["detector_reject_reason"] = "" if image is not None else "camera_frame_unavailable"
        _publish_preview(self.preview_sink, image, None, stats)
        return None, stats

    def read_image(self):
        read_start = time.monotonic()
        ok, image = self.capture.read()
        capture_ts = time.monotonic()
        read_ms = 1000.0 * (capture_ts - read_start)
        if not ok or image is None:
            self.failed_frames += 1
            self.last_image = None
            self.last_stats = self._empty_stats(
                frame_ok=False,
                capture_ts=capture_ts,
                read_ms=read_ms,
            )
            return None

        input_height, input_width = image.shape[:2]
        if input_width != self.output_width or input_height != self.output_height:
            image = self.cv2.resize(image, (self.output_width, self.output_height), interpolation=self.cv2.INTER_LINEAR)
        image = self._undistort(image)
        self.last_image = image
        self.last_stats = self._empty_stats(
            frame_ok=True,
            capture_ts=capture_ts,
            read_ms=read_ms,
            input_width=input_width,
            input_height=input_height,
        )
        return image

    def _undistort(self, image):
        coefficients = self.camera_config.get("distortion_coefficients", [])
        if not coefficients:
            return image
        distortion = np.asarray(coefficients, dtype=float)
        if distortion.shape not in {(4,), (5,), (8,), (12,), (14,)} or not np.all(np.isfinite(distortion)):
            raise ValueError("camera.distortion_coefficients must contain 4, 5, 8, 12, or 14 finite values")
        matrix = np.array(
            [
                [float(self.camera_config["fx"]), 0.0, float(self.camera_config["cx"])],
                [0.0, float(self.camera_config["fy"]), float(self.camera_config["cy"])],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        return self.cv2.undistort(image, matrix, distortion, None, matrix)

    def _empty_stats(
        self,
        *,
        frame_ok: bool = False,
        capture_ts: float | None = None,
        read_ms: float | None = None,
        input_width: int | None = None,
        input_height: int | None = None,
    ) -> dict[str, Any]:
        reported_fps = float(self.capture.get(self.cv2.CAP_PROP_FPS))
        reported_fourcc = int(round(float(self.capture.get(self.cv2.CAP_PROP_FOURCC))))
        return {
            "camera_device": str(self.device),
            "camera_frame_ok": int(frame_ok),
            "camera_capture_ts": "" if capture_ts is None else float(capture_ts),
            "camera_read_ms": "" if read_ms is None else float(read_ms),
            "camera_input_width": "" if input_width is None else int(input_width),
            "camera_input_height": "" if input_height is None else int(input_height),
            "camera_output_width": self.output_width,
            "camera_output_height": self.output_height,
            "camera_requested_fps": self.requested_fps,
            "camera_reported_fps": reported_fps,
            "camera_reported_fourcc": _decode_fourcc(reported_fourcc),
            "camera_failed_frames": self.failed_frames,
        }


class OpenCvYoloSource:
    def __init__(
        self,
        args: argparse.Namespace,
        config: dict[str, Any],
        *,
        camera_source: OpenCvCameraSource | None = None,
        preview_sink: Any = None,
    ):
        _validate_yolo_runtime(config, str(args.yolo_device or ""))
        self.camera = (
            camera_source
            if camera_source is not None
            else OpenCvCameraSource(args, config, preview_sink=preview_sink)
        )
        self.preview_sink = preview_sink
        self.cv2 = self.camera.cv2
        _configure_torch_runtime(config)
        self.detector = YoloByteTrackDetector(
            model_path=str(args.yolo_model),
            class_id=args.yolo_class_id,
            conf=float(args.yolo_conf),
            iou=float(args.yolo_iou),
            imgsz=int(args.yolo_imgsz),
            device=str(args.yolo_device or ""),
            tracker=str(args.yolo_tracker or "bytetrack.yaml"),
            allow_untracked_fallback=bool(args.yolo_allow_untracked_fallback),
            image_reader=self._read_image,
            cv2_module=self.cv2,
        )
        self.config = AirSimDetectionConfig(camera_name="opencv", vehicle_name="Betaflight")

    def close(self) -> None:
        self.camera.close()

    def detect(self, *, timestamp: float, frame_id: int, active_track_id: int | None) -> tuple[FrameDetection | None, dict[str, Any]]:
        frame = self.detector.detect(
            client=None,
            config=self.config,
            frame_id=frame_id,
            exposure_ts=timestamp,
            active_track_id=active_track_id,
        )
        stats = dict(frame.stats)
        stats.update(self.camera.last_stats)
        _publish_preview(self.preview_sink, self.camera.last_image, frame.frame_detection, stats)
        return frame.frame_detection, stats

    def _read_image(self, _client: Any, _config: AirSimDetectionConfig):
        return self.camera.read_image()


class OpenCvRknnSource:
    def __init__(
        self,
        args: argparse.Namespace,
        config: dict[str, Any],
        *,
        camera_source: OpenCvCameraSource | None = None,
        detector: RknnNativeDetector | None = None,
        preview_sink: Any = None,
    ) -> None:
        self.camera = (
            camera_source
            if camera_source is not None
            else OpenCvCameraSource(args, config, preview_sink=preview_sink)
        )
        self.preview_sink = preview_sink
        rknn = dict(config.get("rknn_detector", {}))
        library_path = str(getattr(args, "rknn_library", "") or rknn.get("library", ""))
        model_path = str(getattr(args, "rknn_model", "") or rknn.get("model", ""))
        if not library_path:
            raise RuntimeError("rknn_detector.library or --rknn-library is required")
        if not model_path:
            raise RuntimeError("rknn_detector.model or --rknn-model is required")
        self.detector = detector if detector is not None else RknnNativeDetector(
            library_path=library_path,
            model_path=model_path,
            config=RknnDetectorConfig.from_mapping(rknn),
        )

    def close(self) -> None:
        self.detector.close()
        self.camera.close()

    def metadata(self) -> dict[str, Any]:
        return self.detector.metadata()

    def detect(
        self,
        *,
        timestamp: float,
        frame_id: int,
        active_track_id: int | None,
    ) -> tuple[FrameDetection | None, dict[str, Any]]:
        del active_track_id
        image_bgr = self.camera.read_image()
        camera_stats = dict(self.camera.last_stats)
        if image_bgr is None:
            camera_stats.update(
                detector_source="rknn_native",
                detector_reject_reason="camera_frame_unavailable",
            )
            return None, camera_stats
        image_rgb = np.ascontiguousarray(image_bgr[:, :, ::-1])
        exposure_ts = float(camera_stats.get("camera_capture_ts") or timestamp)
        detection, detector_stats = self.detector.detect(
            image_rgb,
            frame_id=frame_id,
            exposure_ts=exposure_ts,
        )
        detector_stats.update(camera_stats)
        _publish_preview(self.preview_sink, image_bgr, detection, detector_stats)
        return detection, detector_stats


class OpenCvRknnByteTrackSource:
    def __init__(
        self,
        args: argparse.Namespace,
        config: dict[str, Any],
        *,
        camera_source: OpenCvCameraSource | None = None,
        detector: RknnByteTrackDetector | None = None,
        preview_sink: Any = None,
    ) -> None:
        self.camera = (
            camera_source
            if camera_source is not None
            else OpenCvCameraSource(args, config, preview_sink=preview_sink)
        )
        self.preview_sink = preview_sink
        rknn = dict(config.get("rknn_detector", {}))
        tracker_values = dict(config.get("rknn_bytetrack", {}))
        library_path = str(getattr(args, "rknn_library", "") or rknn.get("library", ""))
        model_path = str(getattr(args, "rknn_model", "") or rknn.get("model", ""))
        if not library_path:
            raise RuntimeError("rknn_detector.library or --rknn-library is required")
        if not model_path:
            raise RuntimeError("rknn_detector.model or --rknn-model is required")
        detector_values = dict(rknn)
        detector_values["conf_threshold"] = float(tracker_values.get("detector_conf_threshold", 0.05))
        detector_values["iou_threshold"] = float(tracker_values.get("detector_iou_threshold", 0.70))
        self.detector = detector if detector is not None else RknnByteTrackDetector(
            library_path=library_path,
            model_path=model_path,
            rknn_config=RknnDetectorConfig.from_mapping(detector_values),
            tracker_config=ByteTrackConfig.from_mapping(tracker_values),
        )
        configured_rate_hz = float(tracker_values.get("perception_rate_hz", 0.0))
        override_rate_hz = float(getattr(args, "rknn_perception_rate_hz", 0.0) or 0.0)
        if override_rate_hz < 0.0:
            raise ValueError("--rknn-perception-rate-hz must be non-negative")
        self.perception_rate_hz = override_rate_hz or configured_rate_hz
        self._stop_event = threading.Event()
        self._result_lock = threading.Lock()
        self._latest_result: tuple[int, FrameDetection | None, dict[str, Any]] | None = None
        self._latest_consumed_seq = 0
        self._perception_seq = 0
        self._queue_dropped = 0
        self._worker_error = ""
        self._worker: threading.Thread | None = None
        if self.perception_rate_hz > 0.0:
            self._worker = threading.Thread(target=self._worker_loop, name="rknn-bytetrack", daemon=True)
            self._worker.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._worker is not None:
            self._worker.join(timeout=2.0)
        self.detector.close()
        self.camera.close()

    def metadata(self) -> dict[str, Any]:
        metadata = dict(self.detector.metadata())
        metadata["perception_rate_hz"] = self.perception_rate_hz
        return metadata

    def detect(
        self,
        *,
        timestamp: float,
        frame_id: int,
        active_track_id: int | None,
    ) -> tuple[FrameDetection | None, dict[str, Any]]:
        del active_track_id
        if self._worker is None:
            return self._detect_once(timestamp=timestamp, frame_id=frame_id)
        with self._result_lock:
            worker_error = self._worker_error
            latest = self._latest_result
            if latest is None or latest[0] == self._latest_consumed_seq:
                stats = {
                    "detector_source": "rknn_bytetrack",
                    "detector_reject_reason": "perception_no_new_result",
                    "bbox_measurement_source": "none",
                    "perception_new_result": 0,
                    "perception_worker_error": worker_error,
                    "perception_queue_dropped": self._queue_dropped,
                }
                if worker_error:
                    raise RuntimeError(f"RKNN ByteTrack perception worker failed: {worker_error}")
                return None, stats
            self._latest_consumed_seq = latest[0]
            _sequence, detection, stored_stats = latest
            stats = dict(stored_stats)
            stats["perception_new_result"] = 1
        if worker_error:
            raise RuntimeError(f"RKNN ByteTrack perception worker failed: {worker_error}")
        capture_ts = stats.get("camera_capture_ts")
        stats["perception_result_age_ms"] = (
            "" if capture_ts in (None, "") else max(0.0, 1000.0 * (float(timestamp) - float(capture_ts)))
        )
        return detection, stats

    def _detect_once(self, *, timestamp: float, frame_id: int) -> tuple[FrameDetection | None, dict[str, Any]]:
        image_bgr = self.camera.read_image()
        camera_stats = dict(self.camera.last_stats)
        if image_bgr is None:
            camera_stats.update(
                detector_source="rknn_bytetrack",
                detector_reject_reason="camera_frame_unavailable",
                bbox_measurement_source="none",
            )
            return None, camera_stats
        image_rgb = np.ascontiguousarray(image_bgr[:, :, ::-1])
        exposure_ts = float(camera_stats.get("camera_capture_ts") or timestamp)
        detection, detector_stats = self.detector.detect(
            image_rgb,
            frame_id=frame_id,
            exposure_ts=exposure_ts,
        )
        detector_stats.update(camera_stats)
        _publish_preview(self.preview_sink, image_bgr, detection, detector_stats)
        return detection, detector_stats

    def _worker_loop(self) -> None:
        period_s = 1.0 / max(1.0, self.perception_rate_hz)
        try:
            while not self._stop_event.is_set():
                loop_start = time.monotonic()
                self._perception_seq += 1
                detection, stats = self._detect_once(timestamp=loop_start, frame_id=self._perception_seq)
                stats["perception_seq"] = self._perception_seq
                stats["perception_worker_rate_hz"] = self.perception_rate_hz
                with self._result_lock:
                    if self._latest_result is not None and self._latest_result[0] != self._latest_consumed_seq:
                        self._queue_dropped += 1
                    stats["perception_queue_dropped"] = self._queue_dropped
                    stats["perception_worker_error"] = ""
                    self._latest_result = (self._perception_seq, detection, stats)
                self._stop_event.wait(max(0.0, period_s - (time.monotonic() - loop_start)))
        except Exception as exc:
            with self._result_lock:
                self._worker_error = str(exc)


class IsolatedRknnByteTrackSource:
    def __init__(
        self,
        args: argparse.Namespace,
        config: dict[str, Any],
        *,
        preview_sink: Any = None,
    ) -> None:
        tracker_values = dict(config.get("rknn_bytetrack", {}))
        override_rate_hz = float(getattr(args, "rknn_perception_rate_hz", 0.0) or 0.0)
        self.perception_rate_hz = override_rate_hz or float(tracker_values.get("perception_rate_hz", 0.0))
        if self.perception_rate_hz <= 0.0:
            raise ValueError("isolated RKNN perception requires a positive perception rate")
        context = multiprocessing.get_context("spawn")
        self._stop_event = context.Event()
        self._result_queue = context.Queue(maxsize=1)
        self._startup_queue = context.Queue(maxsize=1)
        self._error_queue = context.Queue(maxsize=1)
        preview_config = getattr(getattr(preview_sink, "config", None), "preview", None)
        preview_supported = bool(
            preview_sink is not None
            and preview_config is not None
            and bool(getattr(preview_config, "enabled", False))
            and callable(getattr(preview_sink, "wants_preview", None))
            and callable(getattr(preview_sink, "offer_encoded_preview", None))
        )
        self._preview_sink = preview_sink if preview_supported else None
        self._preview_request_event = context.Event() if preview_supported else None
        self._preview_queue = context.Queue(maxsize=1) if preview_supported else None
        preview_values = None if not preview_supported else {
            "max_fps": float(preview_config.max_fps),
            "jpeg_quality": int(preview_config.jpeg_quality),
        }
        self._metadata: dict[str, Any] = {}
        self._process = context.Process(
            target=_isolated_rknn_bytetrack_main,
            args=(
                dict(vars(args)),
                config,
                self.perception_rate_hz,
                self._stop_event,
                self._result_queue,
                self._startup_queue,
                self._error_queue,
                self._preview_queue,
                self._preview_request_event,
                preview_values,
            ),
            name="rknn-bytetrack-process",
            daemon=True,
        )
        self._process.start()
        try:
            status, payload = self._startup_queue.get(timeout=20.0)
        except queue.Empty as exc:
            self.close()
            raise RuntimeError("isolated RKNN process startup timed out") from exc
        if status != "ready":
            self.close()
            raise RuntimeError(f"isolated RKNN process failed to start: {payload}")
        self._metadata = dict(payload)
        self._metadata.update(
            process_isolation=True,
            perception_rate_hz=self.perception_rate_hz,
            preview_transport="jpeg_latest_queue" if preview_supported else "disabled",
        )

    def close(self) -> None:
        self._stop_event.set()
        process = getattr(self, "_process", None)
        if process is not None:
            process.join(timeout=5.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=2.0)
        for channel_name in ("_result_queue", "_startup_queue", "_error_queue", "_preview_queue"):
            channel = getattr(self, channel_name, None)
            if channel is not None:
                channel.close()
                channel.join_thread()

    def metadata(self) -> dict[str, Any]:
        return dict(self._metadata)

    def detect(
        self,
        *,
        timestamp: float,
        frame_id: int,
        active_track_id: int | None,
    ) -> tuple[FrameDetection | None, dict[str, Any]]:
        del frame_id, active_track_id
        self._relay_preview()
        error = _queue_latest(self._error_queue)
        if error is not None:
            raise RuntimeError(f"isolated RKNN process failed: {error}")
        latest = _queue_latest(self._result_queue)
        if latest is None:
            if not self._process.is_alive():
                raise RuntimeError("isolated RKNN process exited unexpectedly")
            return None, {
                "detector_source": "rknn_bytetrack",
                "detector_reject_reason": "perception_no_new_result",
                "bbox_measurement_source": "none",
                "perception_new_result": 0,
                "perception_worker_rate_hz": self.perception_rate_hz,
                "perception_worker_error": "",
            }
        detection, stats = latest
        stats = dict(stats)
        capture_ts = stats.get("camera_capture_ts")
        stats["perception_result_age_ms"] = (
            "" if capture_ts in (None, "") else max(0.0, 1000.0 * (float(timestamp) - float(capture_ts)))
        )
        return detection, stats

    def _relay_preview(self) -> None:
        if self._preview_sink is None:
            return
        requested = bool(self._preview_sink.wants_preview())
        if requested:
            self._preview_request_event.set()
        else:
            self._preview_request_event.clear()
        jpeg = _queue_latest(self._preview_queue)
        if requested and jpeg is not None:
            self._preview_sink.offer_encoded_preview(jpeg)


class _IsolatedPreviewEncoder:
    """Encode latest-only debug frames inside the isolated perception process."""

    def __init__(
        self,
        output_queue: Any,
        request_event: Any,
        *,
        max_fps: float,
        jpeg_quality: int,
        cv2_module: Any = None,
    ) -> None:
        self.output_queue = output_queue
        self.request_event = request_event
        self.max_fps = float(max_fps)
        self.jpeg_quality = int(jpeg_quality)
        self.cv2 = cv2_module
        self._next_encode_s = 0.0
        self._offer_count = 0
        self._encode_count = 0
        self._drop_count = 0
        self._error_count = 0
        self._last_error = ""

    def offer_preview(self, image_bgr: Any, overlay: dict[str, Any] | None = None) -> None:
        self._offer_count += 1
        if image_bgr is None or not self.request_event.is_set():
            return
        now = time.monotonic()
        if now < self._next_encode_s:
            return
        try:
            if self.cv2 is None:
                self.cv2 = importlib.import_module("cv2")
            canvas = _draw_isolated_preview_overlay(self.cv2, image_bgr, overlay or {})
            ok, encoded = self.cv2.imencode(
                ".jpg",
                canvas,
                [int(self.cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )
            if not ok:
                raise RuntimeError("cv2.imencode returned false")
            if _queue_replace(self.output_queue, encoded.tobytes()):
                self._drop_count += 1
            self._encode_count += 1
            self._next_encode_s = now + 1.0 / max(1.0, self.max_fps)
        except Exception as exc:
            self._error_count += 1
            self._last_error = f"{type(exc).__name__}:{exc}"[:500]

    def stats(self) -> dict[str, Any]:
        return {
            "perception_preview_offer_count": self._offer_count,
            "perception_preview_encode_count": self._encode_count,
            "perception_preview_drop_count": self._drop_count,
            "perception_preview_error_count": self._error_count,
            "perception_preview_last_error": self._last_error,
        }


def _draw_isolated_preview_overlay(cv2: Any, image_bgr: Any, overlay: dict[str, Any]) -> Any:
    canvas = np.ascontiguousarray(image_bgr).copy()
    bbox = overlay.get("bbox_xyxy")
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return canvas
    height, width = canvas.shape[:2]
    x1, y1, x2, y2 = (int(round(float(value))) for value in bbox)
    x1 = min(max(0, x1), max(0, width - 1))
    x2 = min(max(0, x2), max(0, width - 1))
    y1 = min(max(0, y1), max(0, height - 1))
    y2 = min(max(0, y2), max(0, height - 1))
    cv2.rectangle(canvas, (x1, y1), (x2, y2), (40, 220, 80), 2)
    label_parts = []
    if overlay.get("track_id") is not None:
        label_parts.append(f"ID {overlay['track_id']}")
    if overlay.get("score") is not None:
        label_parts.append(f"{float(overlay['score']):.2f}")
    if label_parts and callable(getattr(cv2, "putText", None)):
        cv2.putText(
            canvas,
            " ".join(label_parts),
            (x1, max(14, y1 - 6)),
            int(getattr(cv2, "FONT_HERSHEY_SIMPLEX", 0)),
            0.45,
            (40, 220, 80),
            1,
            int(getattr(cv2, "LINE_AA", 8)),
        )
    return canvas


def _isolated_rknn_bytetrack_main(
    args_values: dict[str, Any],
    config: dict[str, Any],
    perception_rate_hz: float,
    stop_event: Any,
    result_queue: Any,
    startup_queue: Any,
    error_queue: Any,
    preview_queue: Any,
    preview_request_event: Any,
    preview_values: dict[str, Any] | None,
) -> None:
    source = None
    try:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        child_args = argparse.Namespace(**args_values)
        child_cpu_affinity = _apply_cpu_affinity(
            str(getattr(child_args, "rknn_cpu_affinity", "")),
            label="isolated RKNN process",
        )
        child_args.rknn_perception_rate_hz = 0.0
        child_config = dict(config)
        tracker_values = dict(child_config.get("rknn_bytetrack", {}))
        tracker_values["perception_rate_hz"] = 0.0
        child_config["rknn_bytetrack"] = tracker_values
        preview_encoder = None
        if preview_queue is not None and preview_request_event is not None and preview_values is not None:
            preview_encoder = _IsolatedPreviewEncoder(
                preview_queue,
                preview_request_event,
                max_fps=float(preview_values["max_fps"]),
                jpeg_quality=int(preview_values["jpeg_quality"]),
            )
        source = OpenCvRknnByteTrackSource(child_args, child_config, preview_sink=preview_encoder)
        child_metadata = dict(source.metadata())
        child_metadata["cpu_affinity"] = list(child_cpu_affinity)
        startup_queue.put(("ready", child_metadata))
        period_s = 1.0 / float(perception_rate_hz)
        sequence = 0
        dropped_count = 0
        while not stop_event.is_set():
            loop_start = time.monotonic()
            sequence += 1
            detection, stats = source.detect(
                timestamp=loop_start,
                frame_id=sequence,
                active_track_id=None,
            )
            stats = dict(stats)
            stats.update(
                perception_seq=sequence,
                perception_new_result=1,
                perception_worker_rate_hz=perception_rate_hz,
                perception_worker_error="",
            )
            if preview_encoder is not None:
                stats.update(preview_encoder.stats())
            if _queue_replace(result_queue, (detection, stats)):
                dropped_count += 1
            stats["perception_queue_dropped"] = dropped_count
            stop_event.wait(max(0.0, period_s - (time.monotonic() - loop_start)))
    except Exception as exc:
        message = f"{type(exc).__name__}: {exc}"
        if source is None:
            _queue_replace(startup_queue, ("error", message))
        _queue_replace(error_queue, message)
    finally:
        if source is not None:
            source.close()


def _queue_latest(channel: Any) -> Any:
    latest = None
    while True:
        try:
            latest = channel.get_nowait()
        except queue.Empty:
            return latest


def _queue_replace(channel: Any, value: Any) -> bool:
    dropped = False
    while True:
        try:
            channel.get_nowait()
            dropped = True
        except queue.Empty:
            break
    try:
        channel.put_nowait(value)
    except queue.Full:
        try:
            channel.get(timeout=0.01)
            dropped = True
        except queue.Empty:
            return True
        try:
            channel.put_nowait(value)
        except queue.Full:
            return True
    return dropped


def _publish_preview(
    preview_sink: Any,
    image_bgr: Any,
    detection: FrameDetection | None,
    stats: dict[str, Any],
) -> None:
    if preview_sink is None or image_bgr is None:
        return
    preview_sink.offer_preview(
        image_bgr,
        {
            "bbox_xyxy": None if detection is None else list(detection.bbox_xyxy),
            "track_id": None if detection is None else detection.track_id,
            "score": None if detection is None else detection.score,
            "detector_source": stats.get("detector_source", ""),
            "tracker_state": stats.get("tracker_state", ""),
        },
    )


def main() -> None:
    args = parse_args()
    main_cpu_affinity = _parse_cpu_affinity(args.main_cpu_affinity)
    rknn_cpu_affinity = _parse_cpu_affinity(args.rknn_cpu_affinity)
    _validate_cpu_affinity_plan(
        main_cpu_affinity,
        rknn_cpu_affinity,
        isolate_rknn_process=bool(args.isolate_rknn_process),
    )
    args.main_cpu_affinity_applied = list(
        _apply_cpu_affinity(args.main_cpu_affinity, label="main process")
    )
    if main_cpu_affinity or rknn_cpu_affinity:
        print(
            "CPU affinity: "
            f"main={','.join(str(cpu) for cpu in args.main_cpu_affinity_applied) or '-'} "
            f"isolated_rknn={','.join(str(cpu) for cpu in rknn_cpu_affinity) or '-'}"
        )
    config = _load_config(args.config)
    web_values = dict(config.get("telemetry_web", {}))
    if args.disable_web_preview:
        preview_values = dict(web_values.get("preview", {}))
        preview_values["enabled"] = False
        web_values["preview"] = preview_values
    web_config = TelemetryWebConfig.from_mapping(web_values)
    web_service = TelemetryWebService(web_config)
    web_service.start()
    if web_config.enabled:
        print(f"Browser telemetry: {web_service.url}")
    try:
        _run(args, config, web_service)
    except KeyboardInterrupt:
        print("Shutdown requested; closing Betaflight runtime.")
    finally:
        web_service.close()


def _run(args: argparse.Namespace, config: dict[str, Any], web_service: TelemetryWebService) -> None:
    serial_cfg = dict(config.get("serial", {}))
    port = args.serial_port or str(serial_cfg.get("port", ""))
    if not port:
        raise RuntimeError("serial.port is required in config or via --serial-port")
    baudrate = int(args.msp_baud or serial_cfg.get("baud", 115200))
    timeout_s = float(serial_cfg.get("timeout_s", 0.2))

    control_output_requested = args.control_mode == "msp_raw_rc" and bool(args.allow_control)
    intrinsics = _camera_intrinsics(config)
    camera_R_BC = _camera_mount(config, require_control_ready=control_output_requested)
    camera_calibration = _camera_calibration_metadata(config, camera_R_BC)
    rc_mapper = RcCommandMapper(_rc_mapping_config(config))
    command_shaper_config = GuidanceCommandShaperConfig.from_mapping(
        dict(config.get("guidance_command", {}))
    )
    entry_handoff_values = dict(dict(config.get("guidance_command", {})).get("entry_handoff", {}))
    if control_output_requested and entry_handoff_values.get("rate_source") != "zero":
        raise RuntimeError("control requires explicit guidance_command.entry_handoff.rate_source=zero")
    if command_shaper_config.entry_handoff.rate_source != "zero":
        raise RuntimeError(
            "Betaflight entry_handoff.rate_source must remain zero until MSP_RAW_IMU units are verified"
        )
    command_shaper = GuidanceCommandShaper(command_shaper_config)
    safety_cfg = dict(config.get("safety", {}))
    motor_interlock_config = MotorOutputInterlockConfig.from_mapping(
        dict(safety_cfg.get("motor_output_interlock", {}))
    )
    motor_interlock = MotorOutputInterlock(motor_interlock_config)
    msp_runtime_config = MspRuntimeConfig.from_mapping(dict(config.get("msp_runtime", {})))
    bench_scope = str(dict(config.get("bench_profile", {})).get("scope", ""))
    if control_output_requested and bench_scope == "noprop_bench":
        if not motor_interlock_config.enabled or not motor_interlock_config.latch_until_disarm:
            raise RuntimeError(
                "noprop_bench control requires a latched safety.motor_output_interlock"
            )
        if msp_runtime_config.motor_poll_hz <= 0.0:
            raise RuntimeError("noprop_bench motor interlock requires msp_runtime.motor_poll_hz")
    watchdog_timeout_s = float(safety_cfg.get("watchdog_timeout_s", 0.25))
    watchdog = CommandWatchdog(watchdog_timeout_s)
    setpoint_hold = GuidanceSetpointHold(watchdog_timeout_s)
    safety = BetaflightSafetyStateMachine()
    adapter = BetaflightMSPAdapter(port, baudrate, timeout_s=timeout_s)
    adapter.open()
    fc_identity = _read_fc_identity(adapter)
    box_ids, box_ids_error = _read_box_ids(adapter)
    authorization = resolve_control_authorization(
        dict(config.get("control_authorization", {})),
        fc_identity=fc_identity,
        box_ids=box_ids,
        parameters_path=args.config,
    )
    if args.control_mode == "msp_raw_rc" and args.allow_control and not msp_runtime_config.io_worker_enabled:
        raise RuntimeError("msp_runtime.io_worker_enabled=true is required for any RC output")
    if (
        args.control_mode == "msp_raw_rc"
        and args.allow_control
        and msp_runtime_config.transport_mode != "async_pipeline"
    ):
        raise RuntimeError("msp_runtime.transport_mode=async_pipeline is required for any RC output")
    msp_worker = None
    if msp_runtime_config.io_worker_enabled:
        msp_worker = BetaflightMspIoWorker(adapter, msp_runtime_config, box_ids=box_ids)
        msp_worker.start()

    detection_source = _create_detection_source(args, config, preview_sink=web_service)
    attitude_buffer = AttitudeHistoryBuffer(duration_s=float(config.get("attitude_buffer_s", 2.0)))
    guidance_evaluator, guidance_metadata = _guidance_evaluator(config)
    guidance_metadata.update(_guidance_command_frame_metadata(config))
    pipeline = PureVisionGuidancePipeline(
        intrinsics=intrinsics,
        R_BC=camera_R_BC,
        attitude_buffer=attitude_buffer,
        evaluator=guidance_evaluator,
    )
    fusion_cfg = dict(config.get("attitude_fusion", {}))
    deferred_fusion = DeferredAttitudeFusion(
        pipeline,
        max_wait_s=float(fusion_cfg.get("max_wait_s", 0.20)),
        max_pending=int(fusion_cfg.get("max_pending", 8)),
    )

    log_path = _log_path(args.log_dir, args.log_prefix)
    events_path = _events_path(log_path)
    fields = _log_fields(rc_mapper.config.channel_count)
    meta_path = _meta_path(log_path)
    logging_cfg = dict(config.get("logging", {}))
    platform_health_hz = float(logging_cfg.get("platform_health_hz", 1.0))
    platform_health = (
        None
        if platform_health_hz <= 0.0
        else PlatformHealthSampler(sample_hz=platform_health_hz, log_directory=log_path.parent)
    )
    gc_pause_monitor = PythonGcPauseMonitor()
    start = time.monotonic()
    _write_run_meta(
        meta_path,
        args=args,
        config=config,
        log_path=log_path,
        events_path=events_path,
        fields=fields,
        fc_identity=fc_identity,
        detector_metadata=_detector_metadata(detection_source),
        fc_configuration={
            "box_ids": list(box_ids),
            "box_ids_error": box_ids_error,
            "msp_override_permanent_id": MSP_OVERRIDE_PERMANENT_ID,
            "msp_override_available": MSP_OVERRIDE_PERMANENT_ID in box_ids,
            "msp_override_mode_index": box_mode_index(box_ids, MSP_OVERRIDE_PERMANENT_ID),
        },
        control_authorization=asdict(authorization),
        msp_runtime={"config": asdict(msp_runtime_config)},
        attitude_fusion={
            "max_wait_s": deferred_fusion.max_wait_s,
            "max_pending": deferred_fusion.max_pending,
        },
        guidance=guidance_metadata,
        camera_calibration=camera_calibration,
        platform_health={} if platform_health is None else platform_health.metadata(),
        runtime_diagnostics=gc_pause_monitor.metadata(),
        web_telemetry=web_service.metadata(),
    )
    frame_id = 0
    last_telemetry_s: float | None = None
    last_attitude_s: float | None = None
    last_attitude_buffer_sample_s: float | None = None
    last_loop_start_s: float | None = None
    last_runtime_status: tuple[Any, ...] | None = None

    print(f"Logging Betaflight MSP telemetry to: {log_path}")
    print(f"Control mode: {args.control_mode}; allow_control={int(args.allow_control)}")
    print(
        "Guidance law: "
        f"{guidance_metadata['law']}; "
        f"N={guidance_metadata['navigation_constant'] or '-'} "
        f"Vm={guidance_metadata['fixed_vm_m_s'] or '-'} "
        f"N*Vm={guidance_metadata['fixed_gain'] or '-'} "
        f"limit={guidance_metadata['max_guidance_accel_mps2']} "
        f"frame={guidance_metadata['guidance_eval_frame']}"
    )
    print(
        "Guidance command mapping: "
        f"input_frame={guidance_metadata['rate_gain_input_frame']}"
    )
    print(
        f"Authorization: approved={int(authorization.approved)} reason={authorization.reason} "
        f"scope={authorization.scope or '-'}"
    )
    extrinsic = camera_calibration["extrinsic"]
    print(
        "Camera extrinsic: "
        f"source={extrinsic['source']} verified={int(extrinsic['verified'])} "
        f"up_axis_error_deg={extrinsic['optical_axis_error_deg']:.3f} "
        f"control_ready={int(extrinsic['control_ready'])}"
    )
    print(
        "Camera timestamp: "
        f"source={camera_calibration['timestamp']['source']} "
        f"hardware_exposure={int(camera_calibration['timestamp']['hardware_exposure'])}"
    )
    event_logger = EdgeEventLogger(events_path, start_s=start)
    event_logger.write(
        "run_start",
        timestamp_s=start,
        new={
            "control_mode": args.control_mode,
            "allow_control": bool(args.allow_control),
            "guidance": guidance_metadata,
        },
    )
    if platform_health is not None:
        platform_health.start()
    gc_pause_monitor.start()
    try:
        with log_path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            while float(args.duration_s) <= 0.0 or time.monotonic() - start < float(args.duration_s):
                loop_start = time.monotonic()
                elapsed = loop_start - start
                loop_period_s = None if last_loop_start_s is None else max(0.0, loop_start - last_loop_start_s)
                last_loop_start_s = loop_start
                frame_id += 1
                worker_snapshot = None
                if msp_worker is None:
                    telemetry, telemetry_error = _read_telemetry(adapter)
                else:
                    worker_snapshot = msp_worker.snapshot(loop_start)
                    telemetry = worker_snapshot.telemetry
                    telemetry_error = worker_snapshot.telemetry_error
                if telemetry is not None:
                    status_sample_s = telemetry.status_timestamp_s
                    if status_sample_s is None and telemetry.status is not None:
                        status_sample_s = telemetry.timestamp
                    if status_sample_s is not None:
                        last_telemetry_s = status_sample_s
                    if telemetry.attitude is not None:
                        attitude_sample_s = telemetry.attitude_timestamp_s or telemetry.timestamp
                        if attitude_sample_s != last_attitude_buffer_sample_s:
                            attitude_buffer.push(
                                AttitudeSample(timestamp=attitude_sample_s, R_IB=telemetry.attitude.R_IB)
                            )
                            last_attitude_buffer_sample_s = attitude_sample_s
                        last_attitude_s = attitude_sample_s

                raw_detection, detector_stats = _read_detection(
                    detection_source,
                    args,
                    elapsed_s=elapsed,
                    timestamp=loop_start,
                    frame_id=frame_id,
                    active_track_id=pipeline.active_track_id,
                )
                detector_stats.setdefault(
                    "perception_new_result",
                    int(detector_stats.get("detector_reject_reason") != "perception_no_new_result"),
                )
                fusion_state = deferred_fusion.update(
                    raw_detection,
                    timestamp=loop_start,
                    context=detector_stats,
                    perception_new_result=bool(detector_stats["perception_new_result"]),
                )
                detection = fusion_state.detection
                if fusion_state.context is not None:
                    detector_stats = dict(fusion_state.context)
                detector_stats.update(
                    fusion_status=fusion_state.status,
                    fusion_pending_count=fusion_state.pending_count,
                    fusion_dropped_count=fusion_state.dropped_count,
                    fusion_wait_ms="" if fusion_state.wait_ms is None else fusion_state.wait_ms,
                )
                if fusion_state.status == "waiting_for_attitude":
                    detector_stats["detector_reject_reason"] = "fusion_waiting_for_attitude"
                    detector_stats["perception_new_result"] = 0
                capture_ts = detector_stats.get("camera_capture_ts")
                if detection is not None and capture_ts not in (None, ""):
                    detector_stats["perception_result_age_ms"] = max(
                        0.0,
                        1000.0 * (loop_start - float(capture_ts)),
                    )
                detector_stats["detection_attitude_offset_ms"] = (
                    ""
                    if detection is None or last_attitude_buffer_sample_s is None
                    else 1000.0 * (detection.exposure_ts - last_attitude_buffer_sample_s)
                )
                detector_stats["loop_period_s"] = "" if loop_period_s is None else loop_period_s
                detector_stats.update(_guidance_log_stats(guidance_metadata))
                result = fusion_state.result
                guidance = None if result is None else result.guidance
                if guidance is not None and guidance.valid:
                    watchdog.kick(loop_start)
                raw_setpoint, guidance_body_frd = _guidance_setpoint(
                    config,
                    result,
                    loop_start,
                )

                telemetry_age_s = (
                    worker_snapshot.telemetry_age_s
                    if worker_snapshot is not None
                    else None if last_telemetry_s is None else max(0.0, loop_start - last_telemetry_s)
                )
                attitude_age_s = (
                    worker_snapshot.attitude_age_s
                    if worker_snapshot is not None
                    else None if last_attitude_s is None else max(0.0, loop_start - last_attitude_s)
                )
                watchdog_age_s = watchdog.age_s(loop_start)
                telemetry_fresh = telemetry_age_s is not None and telemetry_age_s <= float(safety_cfg.get("telemetry_timeout_s", 0.5))
                attitude_synced = attitude_age_s is not None and attitude_age_s <= float(safety_cfg.get("attitude_timeout_s", 0.5))
                voltage_ok = _voltage_ok(telemetry, safety_cfg)
                watchdog_ok = watchdog.fresh(loop_start)
                physical_rc_age_s = (
                    worker_snapshot.physical_rc_age_s
                    if worker_snapshot is not None
                    else telemetry_age_s if telemetry is not None and telemetry.rc_channels else None
                )
                physical_rc_fresh = (
                    worker_snapshot.physical_rc_fresh
                    if worker_snapshot is not None
                    else physical_rc_age_s is not None
                    and physical_rc_age_s <= msp_runtime_config.physical_rc_timeout_s
                )
                armed = armed_from_telemetry(telemetry, box_ids)
                motor_age_s = (
                    worker_snapshot.motor_age_s
                    if worker_snapshot is not None
                    else None
                    if telemetry is None or telemetry.motor_timestamp_s is None
                    else max(0.0, loop_start - telemetry.motor_timestamp_s)
                )
                motor_interlock_state = motor_interlock.update(
                    armed=armed,
                    motor_outputs=None if telemetry is None else telemetry.motor_outputs,
                    telemetry_age_s=motor_age_s,
                )
                detector_stats.update(
                    motor_interlock_ok=int(motor_interlock_state.ok),
                    motor_interlock_reason=motor_interlock_state.reason,
                    motor_interlock_latched=int(motor_interlock_state.latched),
                    motor_interlock_output_max_us=(
                        ""
                        if motor_interlock_state.output_max_us is None
                        else motor_interlock_state.output_max_us
                    ),
                    motor_interlock_output_spread_us=(
                        ""
                        if motor_interlock_state.output_spread_us is None
                        else motor_interlock_state.output_spread_us
                    ),
                )
                override_available = MSP_OVERRIDE_PERMANENT_ID in box_ids
                override_active = bool(
                    telemetry is not None
                    and telemetry.status is not None
                    and box_mode_active(telemetry.status.mode_flags, box_ids, MSP_OVERRIDE_PERMANENT_ID)
                )
                aux_enabled = _aux_enabled(telemetry, safety_cfg, override_active=override_active)
                control_requested = args.control_mode == "msp_raw_rc"
                allow_control = bool(args.allow_control)
                prefill_ready = bool(
                    worker_snapshot.prefill_ready
                    if worker_snapshot is not None
                    else not msp_runtime_config.prefill_enabled
                )
                msp_response_fresh = bool(
                    worker_snapshot.set_raw_rc_ack_fresh
                    if worker_snapshot is not None
                    else True
                )
                command_gate_open = bool(
                    control_requested
                    and allow_control
                    and authorization.approved
                    and authorization.config_conflict_free
                    and override_available
                    and override_active
                    and prefill_ready
                    and msp_response_fresh
                    and armed
                    and physical_rc_fresh
                    and telemetry_fresh
                    and attitude_synced
                    and motor_interlock_state.ok
                    and voltage_ok
                    and aux_enabled
                    and watchdog_ok
                )
                held_setpoint = setpoint_hold.update(
                    raw_setpoint,
                    timestamp=loop_start,
                    allow_hold=detector_stats.get("detector_reject_reason")
                    in {"perception_no_new_result", "fusion_waiting_for_attitude"},
                    gate_open=command_gate_open,
                )
                gyro_age_s = (
                    worker_snapshot.raw_imu_age_s
                    if worker_snapshot is not None
                    else None
                    if telemetry is None or telemetry.raw_imu_timestamp_s is None
                    else max(0.0, loop_start - telemetry.raw_imu_timestamp_s)
                )
                setpoint, shaping = command_shaper.update(
                    held_setpoint,
                    timestamp=loop_start,
                    gate_open=command_gate_open,
                    attitude_deg=(
                        None
                        if telemetry is None or telemetry.attitude is None
                        else telemetry.attitude.euler_frd_deg[:2]
                    ),
                    gyro_deg_s=None,
                    gyro_age_s=gyro_age_s,
                )
                target_valid = bool(setpoint.valid)
                decision = safety.update(
                    SafetyInputs(
                        control_requested=control_requested,
                        allow_control=allow_control,
                        target_valid=target_valid,
                        aux_enabled=aux_enabled,
                        telemetry_fresh=telemetry_fresh,
                        attitude_synced=attitude_synced,
                        motor_output_ok=motor_interlock_state.ok,
                        voltage_ok=voltage_ok,
                        watchdog_ok=watchdog_ok,
                        armed=armed,
                        override_available=override_available,
                        override_active=override_active,
                        prefill_ready=prefill_ready,
                        msp_response_fresh=msp_response_fresh,
                        physical_rc_fresh=physical_rc_fresh,
                        snapshot_approved=authorization.approved,
                        config_conflict_free=(
                            authorization.config_conflict_free and msp_runtime_config.io_worker_enabled
                        ),
                    )
                )
                rc_command = rc_mapper.map_setpoint(setpoint, active=decision.command_active)
                if msp_worker is None:
                    send_error = _maybe_send_rc(adapter, args, rc_command, decision.command_active, config)
                else:
                    output_enabled = bool(
                        control_requested
                        and allow_control
                        and authorization.approved
                        and authorization.config_conflict_free
                        and override_available
                    )
                    msp_worker.stage(
                        rc_command,
                        output_enabled=output_enabled,
                        algorithm_authorized=decision.command_active,
                        override_active=override_active,
                    )
                    send_error = worker_snapshot.worker_error if worker_snapshot is not None else ""
                detector_stats.update(
                    _msp_log_stats(
                        adapter,
                        worker_snapshot,
                        armed=armed,
                        override_available=override_available,
                        override_active=override_active,
                        override_index=box_mode_index(box_ids, MSP_OVERRIDE_PERMANENT_ID),
                        physical_rc_age_s=physical_rc_age_s,
                        physical_rc_fresh=physical_rc_fresh,
                        authorization=authorization,
                        runtime_config=msp_runtime_config,
                        timestamp=loop_start,
                    )
                )
                if platform_health is not None:
                    detector_stats.update(_platform_health_log_stats(platform_health.snapshot(), loop_start))
                detector_stats.update(gc_pause_monitor.snapshot())
                detector_stats.update(web_service.log_stats())
                event_logger.update(
                    {
                        "armed": int(armed),
                        "msp_override_active": int(override_active),
                        "prefill_ready": int(prefill_ready),
                        "set_raw_rc_ack_fresh": int(msp_response_fresh),
                        "safety_state": str(decision.state.value),
                        "publish_mode": "" if worker_snapshot is None else worker_snapshot.publish_mode,
                        "target_valid": int(target_valid),
                        "entry_handoff_active": int(shaping.entry_active),
                        "tilt_hardcap_active": int(shaping.hardcap_active),
                        "track_id": None if detection is None else detection.track_id,
                        "telemetry_fresh": int(telemetry_fresh),
                        "attitude_synced": int(attitude_synced),
                        "motor_interlock_ok": int(motor_interlock_state.ok),
                        "motor_interlock_reason": motor_interlock_state.reason,
                        "physical_rc_fresh": int(physical_rc_fresh),
                        "watchdog_ok": int(watchdog_ok),
                        "msp_telemetry_error": telemetry_error,
                        "msp_worker_error": "" if worker_snapshot is None else worker_snapshot.worker_error,
                        "web_error": detector_stats.get("web_last_error", ""),
                    },
                    timestamp_s=loop_start,
                    context={
                        "safety_reason": decision.reason,
                        "rc_in": [] if telemetry is None else list(telemetry.rc_channels),
                        "rc_sent": [] if worker_snapshot is None else list(worker_snapshot.last_sent_channels),
                    },
                )
                runtime_status = (
                    decision.state.value,
                    decision.reason,
                    int(armed),
                    int(override_active),
                    int(prefill_ready),
                    "" if worker_snapshot is None else worker_snapshot.publish_mode,
                )
                if runtime_status != last_runtime_status:
                    sent = () if worker_snapshot is None else worker_snapshot.last_sent_channels[:4]
                    sent_text = "-" if not sent else "/".join(str(value) for value in sent)
                    print(
                        f"BF state={runtime_status[0]} reason={runtime_status[1]} "
                        f"armed={runtime_status[2]} override={runtime_status[3]} "
                        f"prefill={runtime_status[4]} target={int(target_valid)} "
                        f"publish={runtime_status[5] or '-'} sent_aetr={sent_text}",
                        flush=True,
                    )
                    last_runtime_status = runtime_status
                row = _log_row(
                    timestamp=loop_start,
                    elapsed_s=elapsed,
                    telemetry=telemetry,
                    telemetry_error=telemetry_error,
                    detector_stats=detector_stats,
                    detection=detection,
                    result=result,
                    pre_shape_setpoint=held_setpoint,
                    setpoint=setpoint,
                    shaping=shaping,
                    rc_command=rc_command,
                    safety_state=str(decision.state.value),
                    safety_reason=decision.reason,
                    send_error=send_error,
                    telemetry_age_s=telemetry_age_s,
                    attitude_age_s=attitude_age_s,
                    watchdog_age_s=watchdog_age_s,
                    telemetry_fresh=telemetry_fresh,
                    attitude_synced=attitude_synced,
                    watchdog_ok=watchdog_ok,
                    voltage_ok=voltage_ok,
                    aux_enabled=aux_enabled,
                    control_requested=control_requested,
                    allow_control=allow_control,
                    intrinsics=intrinsics,
                    channel_count=rc_mapper.config.channel_count,
                    guidance_body_frd=guidance_body_frd,
                )
                writer.writerow(row)
                web_service.publish(
                    telemetry_payload_from_log_row(
                        row,
                        channel_count=rc_mapper.config.channel_count,
                        channel_map=msp_runtime_config.set_raw_rc_channel_map,
                    ),
                    timestamp_s=loop_start,
                )

                sleep_s = max(0.0, (1.0 / max(1.0, float(args.rate_hz))) - (time.monotonic() - loop_start))
                time.sleep(sleep_s)
    finally:
        try:
            if msp_worker is not None:
                msp_worker.close()
        finally:
            try:
                close = getattr(detection_source, "close", None)
                if callable(close):
                    close()
            finally:
                try:
                    adapter.close()
                finally:
                    try:
                        if platform_health is not None:
                            platform_health.close()
                    finally:
                        gc_pause_monitor.close()
                        stopped_s = time.monotonic()
                        event_logger.write("run_stop", timestamp_s=stopped_s, new="normal_or_exception")
                        event_logger.close()


def _load_config(path: str) -> dict[str, Any]:
    config_path = Path(path).expanduser()
    with config_path.open("r") as stream:
        return json.load(stream)


def _guidance_evaluator(
    config: dict[str, Any],
) -> tuple[GuidanceEvaluator | FixedVmGuidanceEvaluator, dict[str, Any]]:
    values = dict(config.get("guidance", {}))
    law = str(values.get("law", "ttc_png")).strip().lower()
    max_norm = _positive_guidance_value(
        values,
        "max_guidance_accel_mps2",
        default=10.0,
    )
    if law == "ttc_png":
        return GuidanceEvaluator(max_norm=max_norm), {
            "law": law,
            "navigation_constant": None,
            "fixed_vm_m_s": None,
            "fixed_gain": None,
            "max_guidance_accel_mps2": max_norm,
            "ttc_required": True,
        }
    if law == "fixed_vm_png":
        navigation_constant = _positive_guidance_value(values, "navigation_constant")
        fixed_vm_m_s = _positive_guidance_value(values, "fixed_vm_m_s")
        evaluator = FixedVmGuidanceEvaluator(
            navigation_constant=navigation_constant,
            fixed_vm_m_s=fixed_vm_m_s,
            max_norm=max_norm,
        )
        return evaluator, {
            "law": law,
            "navigation_constant": navigation_constant,
            "fixed_vm_m_s": fixed_vm_m_s,
            "fixed_gain": evaluator.fixed_gain,
            "max_guidance_accel_mps2": max_norm,
            "ttc_required": False,
        }
    raise RuntimeError(
        f"unsupported guidance.law={law!r}; expected 'ttc_png' or 'fixed_vm_png'"
    )


def _positive_guidance_value(
    values: dict[str, Any],
    key: str,
    *,
    default: float | None = None,
) -> float:
    if key not in values:
        if default is None:
            raise RuntimeError(f"guidance.{key} is required")
        value = float(default)
    else:
        try:
            value = float(values[key])
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"guidance.{key} must be numeric") from exc
    if not np.isfinite(value) or value <= 0.0:
        raise RuntimeError(f"guidance.{key} must be finite and positive")
    return value


def _guidance_log_stats(metadata: dict[str, Any]) -> dict[str, Any]:
    return {
        "guidance_law": metadata["law"],
        "guidance_navigation_constant": metadata["navigation_constant"],
        "guidance_fixed_vm_m_s": metadata["fixed_vm_m_s"],
        "guidance_fixed_gain": metadata["fixed_gain"],
        "guidance_max_accel_mps2": metadata["max_guidance_accel_mps2"],
        "guidance_ttc_required": int(bool(metadata["ttc_required"])),
        "guidance_eval_frame": metadata["guidance_eval_frame"],
        "rate_gain_input_frame": metadata["rate_gain_input_frame"],
    }


def _guidance_command_frame_metadata(config: dict[str, Any]) -> dict[str, str]:
    values = dict(config.get("guidance_command", {}))
    guidance_eval_frame = str(values.get("guidance_eval_frame", "")).strip().lower()
    rate_gain_input_frame = str(values.get("rate_gain_input_frame", "")).strip().lower()
    if guidance_eval_frame != GUIDANCE_EVAL_FRAME:
        raise RuntimeError(
            f"guidance_command.guidance_eval_frame must be {GUIDANCE_EVAL_FRAME!r}"
        )
    if rate_gain_input_frame != RATE_GAIN_INPUT_FRAME:
        raise RuntimeError(
            f"guidance_command.rate_gain_input_frame must be {RATE_GAIN_INPUT_FRAME!r}"
        )
    return {
        "guidance_eval_frame": guidance_eval_frame,
        "rate_gain_input_frame": rate_gain_input_frame,
    }


def _rc_mapping_config(config: dict[str, Any]) -> RcMappingConfig:
    rc = dict(config.get("rc_mapping", {}))
    aux_values = {int(key): int(value) for key, value in dict(rc.get("aux_values_us", {})).items()}
    rc_rate = tuple(float(value) for value in rc.get("betaflight_rc_rate", [1.0, 1.0, 1.0]))
    super_rate = tuple(float(value) for value in rc.get("betaflight_super_rate", [0.0, 0.0, 0.0]))
    expo = tuple(float(value) for value in rc.get("betaflight_expo", [0.0, 0.0, 0.0]))
    return RcMappingConfig(
        channel_map=str(rc.get("channel_map", "AETR1234")),
        channel_count=int(rc.get("channel_count", 8)),
        roll_rate_limit_deg_s=float(rc.get("roll_rate_limit_deg_s", 120.0)),
        pitch_rate_limit_deg_s=float(rc.get("pitch_rate_limit_deg_s", 120.0)),
        yaw_rate_limit_deg_s=float(rc.get("yaw_rate_limit_deg_s", 90.0)),
        rate_mapping_type=str(rc.get("rate_mapping_type", "linear")),
        betaflight_rc_rate=rc_rate,
        betaflight_super_rate=super_rate,
        betaflight_expo=expo,
        roll_command_limit_deg_s=_optional_mapping_float(rc, "roll_command_limit_deg_s"),
        pitch_command_limit_deg_s=_optional_mapping_float(rc, "pitch_command_limit_deg_s"),
        yaw_command_limit_deg_s=_optional_mapping_float(rc, "yaw_command_limit_deg_s"),
        thrust_min=float(rc.get("thrust_min", 0.0)),
        thrust_hover=float(rc.get("thrust_hover", 0.5)),
        thrust_max=float(rc.get("thrust_max", 1.0)),
        throttle_min_us=int(rc.get("throttle_min_us", 1000)),
        throttle_hover_us=int(rc.get("throttle_hover_us", 1500)),
        throttle_max_us=int(rc.get("throttle_max_us", 2000)),
        neutral_throttle_us=int(rc.get("neutral_throttle_us", 1000)),
        max_delta_us_per_s=float(rc.get("max_delta_us_per_s", 0.0)),
        aux_values_us=aux_values,
    )


def _optional_mapping_float(values: dict[str, Any], key: str) -> float | None:
    value = values.get(key)
    return None if value is None else float(value)


def _camera_intrinsics(config: dict[str, Any]) -> CameraIntrinsics:
    camera = dict(config.get("camera", {}))
    width = int(camera.get("width", 640))
    height = int(camera.get("height", 480))
    return CameraIntrinsics(
        fx=float(camera.get("fx", 500.0)),
        fy=float(camera.get("fy", 500.0)),
        cx=float(camera.get("cx", width / 2.0)),
        cy=float(camera.get("cy", height / 2.0)),
        width=width,
        height=height,
    )


def _camera_mount(config: dict[str, Any], *, require_control_ready: bool = False) -> np.ndarray:
    camera = dict(config.get("camera", {}))
    explicit = camera.get("R_BC") is not None
    if explicit:
        matrix = np.asarray(camera["R_BC"], dtype=float)
    else:
        matrix = camera_to_body_mount(float(camera.get("pitch_up_deg", 90.0)))
    matrix = validated_rotation_matrix(matrix, name="camera.R_BC")
    validation = dict(camera.get("extrinsic_validation", {}))
    expected_axis = validation.get("expected_optical_axis_body", [0.0, 0.0, -1.0])
    diagnostics = camera_mount_diagnostics(matrix, expected_optical_axis_body=expected_axis)
    max_error_deg = float(validation.get("max_optical_axis_error_deg", 5.0))
    if require_control_ready:
        if not explicit:
            raise RuntimeError("camera.R_BC must be explicit before Betaflight RC output is allowed")
        if validation.get("verified") is not True:
            raise RuntimeError("camera.extrinsic_validation.verified=true is required for RC output")
        if str(validation.get("body_frame", "")).upper() != "FRD":
            raise RuntimeError("camera extrinsic body_frame must be FRD for Betaflight RC output")
        if str(validation.get("camera_frame", "")) != "opencv_x_right_y_down_z_forward":
            raise RuntimeError("camera extrinsic camera_frame must use the OpenCV ray convention")
        if diagnostics["optical_axis_error_deg"] > max_error_deg:
            raise RuntimeError(
                "camera optical axis does not point body-up within the configured tolerance: "
                f"{diagnostics['optical_axis_error_deg']:.3f} deg"
            )
    return matrix


def _camera_calibration_metadata(config: dict[str, Any], R_BC: np.ndarray) -> dict[str, Any]:
    camera = dict(config.get("camera", {}))
    validation = dict(camera.get("extrinsic_validation", {}))
    expected_axis = validation.get("expected_optical_axis_body", [0.0, 0.0, -1.0])
    diagnostics = camera_mount_diagnostics(R_BC, expected_optical_axis_body=expected_axis)
    max_error_deg = float(validation.get("max_optical_axis_error_deg", 5.0))
    explicit = camera.get("R_BC") is not None
    verified = validation.get("verified") is True
    frame_conventions_valid = bool(
        str(validation.get("body_frame", "")).upper() == "FRD"
        and str(validation.get("camera_frame", "")) == "opencv_x_right_y_down_z_forward"
    )
    timestamp_source = str(camera.get("timestamp_source", "capture_return_monotonic"))
    return {
        "intrinsics": {
            "width": int(camera.get("width", 640)),
            "height": int(camera.get("height", 480)),
            "fx": float(camera.get("fx", 500.0)),
            "fy": float(camera.get("fy", 500.0)),
            "cx": float(camera.get("cx", float(camera.get("width", 640)) / 2.0)),
            "cy": float(camera.get("cy", float(camera.get("height", 480)) / 2.0)),
            "distortion_coefficients": [
                float(value) for value in camera.get("distortion_coefficients", [])
            ],
            "calibration_id": str(camera.get("intrinsic_calibration_id", "")),
        },
        "extrinsic": {
            "source": "R_BC" if explicit else "legacy_pitch_up_deg",
            "R_BC": [[float(value) for value in row] for row in np.asarray(R_BC)],
            "verified": verified,
            "body_frame": str(validation.get("body_frame", "")),
            "camera_frame": str(validation.get("camera_frame", "")),
            "max_optical_axis_error_deg": max_error_deg,
            **diagnostics,
            "control_ready": bool(
                explicit
                and verified
                and frame_conventions_valid
                and diagnostics["optical_axis_error_deg"] <= max_error_deg
            ),
        },
        "timestamp": {
            "source": timestamp_source,
            "hardware_exposure": timestamp_source == "v4l2_hardware_exposure_monotonic",
            "note": (
                "exposure timestamp"
                if timestamp_source == "v4l2_hardware_exposure_monotonic"
                else "software approximation; not the hardware exposure instant"
            ),
        },
    }


def _configure_torch_runtime(config: dict[str, Any], *, torch_module: Any = None) -> None:
    runtime = dict(config.get("torch_runtime", {}))
    num_threads = int(runtime.get("num_threads", 0))
    disable_mkldnn = bool(runtime.get("disable_mkldnn", False))
    if num_threads <= 0 and not disable_mkldnn:
        return
    torch = torch_module if torch_module is not None else importlib.import_module("torch")
    if num_threads > 0:
        torch.set_num_threads(num_threads)
    if disable_mkldnn:
        torch.backends.mkldnn.enabled = False


def _validate_yolo_runtime(config: dict[str, Any], requested_device: str) -> None:
    runtime = dict(config.get("torch_runtime", {}))
    allow_cpu_inference = bool(runtime.get("allow_cpu_inference", True))
    normalized_device = str(requested_device or "").strip().lower()
    if not allow_cpu_inference and normalized_device in {"", "cpu"}:
        raise RuntimeError(
            "CPU YOLO inference is disabled by torch_runtime.allow_cpu_inference; "
            "the RK3588 bench rebooted under sustained PyTorch CPU inference"
        )


def _create_detection_source(
    args: argparse.Namespace,
    config: dict[str, Any],
    *,
    preview_sink: Any = None,
):
    if args.isolate_rknn_process and args.detector_source != "rknn_bytetrack":
        raise RuntimeError("--isolate-rknn-process requires --detector-source rknn_bytetrack")
    if args.detector_source == "none":
        return None
    if args.detector_source == "csv":
        if not args.detections_csv:
            raise RuntimeError("--detections-csv is required for --detector-source csv")
        return DetectionCsvSource(args.detections_csv)
    if args.detector_source == "camera_only":
        return OpenCvCameraSource(args, config, preview_sink=preview_sink)
    if args.detector_source == "yolo_bytetrack":
        return OpenCvYoloSource(args, config, preview_sink=preview_sink)
    if args.detector_source == "rknn_native":
        return OpenCvRknnSource(args, config, preview_sink=preview_sink)
    if args.detector_source == "rknn_bytetrack":
        if args.isolate_rknn_process:
            return IsolatedRknnByteTrackSource(args, config, preview_sink=preview_sink)
        return OpenCvRknnByteTrackSource(args, config, preview_sink=preview_sink)
    raise ValueError(f"unsupported detector source: {args.detector_source}")


def _detector_metadata(detection_source: Any) -> dict[str, Any]:
    metadata = getattr(detection_source, "metadata", None)
    return metadata() if callable(metadata) else {}


def _read_fc_identity(adapter: BetaflightMSPAdapter) -> dict[str, Any]:
    identity: dict[str, Any] = {}
    try:
        api = adapter.read_api_version()
        identity["api_protocol_version"] = api.protocol_version
        identity["api_major"] = api.api_major
        identity["api_minor"] = api.api_minor
        identity["fc_variant"] = adapter.read_fc_variant()
        version = adapter.read_fc_version()
        identity["fc_version_major"] = version.major
        identity["fc_version_minor"] = version.minor
        identity["fc_version_patch"] = version.patch
    except Exception as exc:
        identity["fc_identity_error"] = str(exc)
    return identity


def _read_box_ids(adapter: BetaflightMSPAdapter) -> tuple[tuple[int, ...], str]:
    try:
        return adapter.read_box_ids(), ""
    except Exception as exc:
        return (), str(exc)


def _read_telemetry(adapter: BetaflightMSPAdapter) -> tuple[BetaflightTelemetry | None, str]:
    try:
        return adapter.read_telemetry(), ""
    except Exception as exc:
        return None, str(exc)


def _msp_log_stats(
    adapter: BetaflightMSPAdapter,
    worker_snapshot,
    *,
    armed: bool,
    override_available: bool,
    override_active: bool,
    override_index: int | None,
    physical_rc_age_s: float | None,
    physical_rc_fresh: bool,
    authorization,
    runtime_config: MspRuntimeConfig,
    timestamp: float,
) -> dict[str, Any]:
    stats = adapter.snapshot_stats() if worker_snapshot is None else worker_snapshot.adapter_stats
    values = {
        "armed": int(armed),
        "msp_override_available": int(override_available),
        "msp_override_active": int(override_active),
        "msp_override_mode_index": "" if override_index is None else override_index,
        "physical_rc_age_s": "" if physical_rc_age_s is None else physical_rc_age_s,
        "physical_rc_fresh": int(physical_rc_fresh),
        "control_snapshot_approved": int(authorization.approved),
        "control_authorization_reason": authorization.reason,
        "config_conflict_free": int(authorization.config_conflict_free),
        "msp_io_worker_enabled": int(runtime_config.io_worker_enabled),
        "msp_transport_mode": runtime_config.transport_mode,
        "msp_request_count": stats.request_count,
        "msp_request_error_count": stats.request_error_count,
        "msp_tx_bytes": stats.tx_bytes,
        "msp_rx_bytes": stats.rx_bytes,
        "msp_set_raw_rc_attempt_count": stats.set_raw_rc_attempt_count,
        "msp_set_raw_rc_success_count": stats.set_raw_rc_success_count,
        "msp_set_raw_rc_write_attempt_count": stats.set_raw_rc_write_attempt_count,
        "msp_set_raw_rc_write_success_count": stats.set_raw_rc_write_success_count,
        "msp_set_raw_rc_write_error_count": stats.set_raw_rc_write_error_count,
        "msp_set_raw_rc_ack_count": stats.set_raw_rc_ack_count,
        "msp_set_raw_rc_pending_depth": stats.set_raw_rc_pending_depth,
        "msp_set_raw_rc_last_write_age_s": (
            ""
            if stats.set_raw_rc_last_write_monotonic_s is None
            else max(0.0, timestamp - stats.set_raw_rc_last_write_monotonic_s)
        ),
        "msp_set_raw_rc_ack_age_s": (
            ""
            if stats.set_raw_rc_last_ack_monotonic_s is None
            else max(0.0, timestamp - stats.set_raw_rc_last_ack_monotonic_s)
        ),
        "msp_set_raw_rc_ack_fresh": "",
        "msp_set_raw_rc_write_interval_s": stats.set_raw_rc_write_interval_s,
        "msp_set_raw_rc_write_max_interval_s": stats.set_raw_rc_write_max_interval_s,
        "msp_set_raw_rc_write_rate_hz": stats.set_raw_rc_write_rate_hz,
        "msp_set_raw_rc_write_p50_interval_s": stats.set_raw_rc_write_p50_interval_s,
        "msp_set_raw_rc_write_p95_interval_s": stats.set_raw_rc_write_p95_interval_s,
        "msp_set_raw_rc_write_p99_interval_s": stats.set_raw_rc_write_p99_interval_s,
        "msp_set_raw_rc_write_p999_interval_s": stats.set_raw_rc_write_p999_interval_s,
        "msp_async_pending_telemetry_count": stats.async_pending_telemetry_count,
        "msp_rx_discarded_bytes": stats.rx_discarded_bytes,
        "msp_rx_checksum_error_count": stats.rx_checksum_error_count,
        "msp_rx_parser_error_count": stats.rx_parser_error_count,
        "msp_worker_poll_count": "",
        "msp_worker_poll_error_count": "",
        "msp_worker_staged_count": "",
        "msp_worker_send_skip_count": "",
        "msp_worker_send_error_count": "",
        "msp_worker_error": "",
        "msp_output_enabled": "",
        "msp_algorithm_authorized": "",
        "msp_worker_override_active": "",
        "msp_prefill_ready": "",
        "msp_prefill_success_count": "",
        "msp_passthrough_send_count": "",
        "msp_algorithm_send_count": "",
        "msp_stale_command_count": "",
        "msp_staged_command_age_s": "",
        "msp_publish_mode": "",
        "msp_last_publish_output_enabled": "",
        "msp_last_publish_algorithm_authorized": "",
        "msp_last_publish_override_active": "",
        "msp_last_publish_override_release_hold_active": "",
        "msp_last_publish_prefill_ready": "",
        "msp_last_publish_physical_rc_fresh": "",
        "msp_last_publish_command_fresh": "",
        "msp_last_publish_command_active": "",
        "msp_last_publish_command_reason": "",
        "msp_last_publish_set_raw_rc_ack_fresh": "",
        "msp_override_release_hold_active": "",
        "msp_rc_poll_suspended": "",
        "msp_last_sent_channels": (),
        "msp_status_age_s": "",
        "msp_attitude_age_s": "",
        "msp_analog_age_s": "",
        "msp_raw_imu_age_s": "",
        "msp_motor_age_s": "",
        "msp_publish_tick_interval_s": "",
        "msp_publish_tick_max_interval_s": "",
        "msp_publish_deadline_miss_count": "",
        "msp_send_success_interval_s": "",
        "msp_send_success_max_interval_s": "",
        "msp_last_send_success_age_s": "",
        "msp_consecutive_send_error_count": "",
        "throttle_handover_source_us": "",
        "throttle_handover_target_us": "",
        "throttle_handover_alpha": "",
        "throttle_handover_output_us": "",
        "throttle_handover_active": "",
    }
    for label, command in MSP_COMMAND_LOG_SPECS:
        command_stats = stats.for_command(command)
        prefix = f"msp_cmd_{label}"
        values[f"{prefix}_attempt_count"] = 0 if command_stats is None else command_stats.attempt_count
        values[f"{prefix}_success_count"] = 0 if command_stats is None else command_stats.success_count
        values[f"{prefix}_error_count"] = 0 if command_stats is None else command_stats.error_count
        values[f"{prefix}_last_rtt_ms"] = "" if command_stats is None else command_stats.last_rtt_ms
        values[f"{prefix}_max_rtt_ms"] = "" if command_stats is None else command_stats.max_rtt_ms
        values[f"{prefix}_last_success_age_s"] = (
            ""
            if command_stats is None or command_stats.last_success_monotonic_s is None
            else max(0.0, timestamp - command_stats.last_success_monotonic_s)
        )
        values[f"{prefix}_last_error"] = "" if command_stats is None else command_stats.last_error
    if worker_snapshot is not None:
        handover = worker_snapshot.throttle_handover
        values.update(
            msp_worker_poll_count=worker_snapshot.poll_count,
            msp_worker_poll_error_count=worker_snapshot.poll_error_count,
            msp_worker_staged_count=worker_snapshot.staged_count,
            msp_worker_send_skip_count=worker_snapshot.send_skip_count,
            msp_worker_send_error_count=worker_snapshot.send_error_count,
            msp_worker_error=worker_snapshot.worker_error,
            msp_output_enabled=int(worker_snapshot.output_enabled),
            msp_algorithm_authorized=int(worker_snapshot.algorithm_authorized),
            msp_worker_override_active=int(worker_snapshot.override_active),
            msp_prefill_ready=int(worker_snapshot.prefill_ready),
            msp_prefill_success_count=worker_snapshot.prefill_success_count,
            msp_passthrough_send_count=worker_snapshot.passthrough_send_count,
            msp_algorithm_send_count=worker_snapshot.algorithm_send_count,
            msp_stale_command_count=worker_snapshot.stale_command_count,
            msp_staged_command_age_s=(
                "" if worker_snapshot.staged_command_age_s is None else worker_snapshot.staged_command_age_s
            ),
            msp_publish_mode=worker_snapshot.publish_mode,
            msp_last_publish_output_enabled=int(worker_snapshot.last_publish_output_enabled),
            msp_last_publish_algorithm_authorized=int(
                worker_snapshot.last_publish_algorithm_authorized
            ),
            msp_last_publish_override_active=int(worker_snapshot.last_publish_override_active),
            msp_last_publish_override_release_hold_active=int(
                worker_snapshot.last_publish_override_release_hold_active
            ),
            msp_last_publish_prefill_ready=int(worker_snapshot.last_publish_prefill_ready),
            msp_last_publish_physical_rc_fresh=int(
                worker_snapshot.last_publish_physical_rc_fresh
            ),
            msp_last_publish_command_fresh=int(worker_snapshot.last_publish_command_fresh),
            msp_last_publish_command_active=int(worker_snapshot.last_publish_command_active),
            msp_last_publish_command_reason=worker_snapshot.last_publish_command_reason,
            msp_last_publish_set_raw_rc_ack_fresh=int(
                worker_snapshot.last_publish_set_raw_rc_ack_fresh
            ),
            msp_set_raw_rc_ack_fresh=int(worker_snapshot.set_raw_rc_ack_fresh),
            msp_override_release_hold_active=int(worker_snapshot.override_release_hold_active),
            msp_rc_poll_suspended=int(worker_snapshot.rc_poll_suspended),
            msp_last_sent_channels=worker_snapshot.last_sent_channels,
            msp_status_age_s=worker_snapshot.status_age_s,
            msp_attitude_age_s=worker_snapshot.attitude_age_s,
            msp_analog_age_s=worker_snapshot.analog_age_s,
            msp_raw_imu_age_s=worker_snapshot.raw_imu_age_s,
            msp_motor_age_s=worker_snapshot.motor_age_s,
            msp_publish_tick_interval_s=worker_snapshot.publish_tick_interval_s,
            msp_publish_tick_max_interval_s=worker_snapshot.publish_tick_max_interval_s,
            msp_publish_deadline_miss_count=worker_snapshot.publish_deadline_miss_count,
            msp_send_success_interval_s=worker_snapshot.send_success_interval_s,
            msp_send_success_max_interval_s=worker_snapshot.send_success_max_interval_s,
            msp_last_send_success_age_s=worker_snapshot.last_send_success_age_s,
            msp_consecutive_send_error_count=worker_snapshot.consecutive_send_error_count,
            throttle_handover_source_us=handover.source_us,
            throttle_handover_target_us=handover.target_us,
            throttle_handover_alpha=handover.alpha,
            throttle_handover_output_us=handover.output_us,
            throttle_handover_active=int(handover.active),
        )
    return values


def _platform_health_log_stats(snapshot, timestamp: float) -> dict[str, Any]:
    age_s = None if snapshot.timestamp_s is None else max(0.0, timestamp - snapshot.timestamp_s)
    return {
        "host_sample_age_s": age_s,
        "host_load_1m": snapshot.load_1m,
        "host_process_rss_mb": snapshot.process_rss_mb,
        "host_mem_available_mb": snapshot.mem_available_mb,
        "host_disk_free_gb": snapshot.disk_free_gb,
        "host_thermal_max_c": snapshot.thermal_max_c,
        "host_soc_temp_c": snapshot.soc_temp_c,
        "host_npu_temp_c": snapshot.npu_temp_c,
        "host_cpu_freq_min_mhz": snapshot.cpu_freq_min_mhz,
        "host_cpu_freq_max_mhz": snapshot.cpu_freq_max_mhz,
        "host_npu_freq_mhz": snapshot.npu_freq_mhz,
        "host_health_error": snapshot.error,
    }


def _read_detection(
    detection_source,
    args: argparse.Namespace,
    *,
    elapsed_s: float,
    timestamp: float,
    frame_id: int,
    active_track_id: int | None,
) -> tuple[FrameDetection | None, dict[str, Any]]:
    if detection_source is None:
        return None, {"detector_source": "none", "detector_reject_reason": "detector_disabled"}
    if args.detector_source == "csv":
        return detection_source.detect(elapsed_s=elapsed_s, timestamp=timestamp, frame_id=frame_id)
    return detection_source.detect(timestamp=timestamp, frame_id=frame_id, active_track_id=active_track_id)


def _process_detection(pipeline: PureVisionGuidancePipeline, detection: FrameDetection | None) -> VisionGuidanceResult | None:
    if detection is None:
        return None
    return pipeline.process(detection)


def _guidance_setpoint(
    config: dict[str, Any],
    result: VisionGuidanceResult | None,
    timestamp: float,
) -> tuple[GuidanceSetpoint, np.ndarray | None]:
    guidance = None if result is None else result.guidance
    if guidance is None:
        return (
            GuidanceSetpoint(
                timestamp=timestamp,
                valid=False,
                source="guidance_eval",
                reject_reason="guidance_missing",
            ),
            None,
        )
    command_cfg = dict(config.get("guidance_command", {}))
    setpoint = guidance_eval_to_setpoint(
        guidance,
        R_IB=None if result is None else result.R_IB,
        rate_gain_matrix=command_cfg.get("rate_gain_matrix", [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        hover_thrust=float(command_cfg.get("hover_thrust", config.get("rc_mapping", {}).get("thrust_hover", 0.5))),
        yaw_rate_deg_s=float(command_cfg.get("yaw_rate_bias_deg_s", 0.0)),
    )
    body_vector = None
    if guidance.valid and result is not None and result.R_IB is not None:
        try:
            body_vector = inertial_vector_to_body_frd(guidance.g_eval, result.R_IB)
        except ValueError:
            body_vector = None
    if setpoint.timestamp == 0.0:
        setpoint = GuidanceSetpoint(
            timestamp=timestamp,
            valid=setpoint.valid,
            source=setpoint.source,
            reject_reason=setpoint.reject_reason,
        )
    return setpoint, body_vector


def _voltage_ok(telemetry: BetaflightTelemetry | None, safety_cfg: dict[str, Any]) -> bool:
    threshold = float(safety_cfg.get("min_vbat_v", 0.0))
    if threshold <= 0.0:
        return True
    return bool(telemetry is not None and telemetry.analog is not None and telemetry.analog.vbat_v >= threshold)


def _aux_enabled(
    telemetry: BetaflightTelemetry | None,
    safety_cfg: dict[str, Any],
    *,
    override_active: bool = False,
) -> bool:
    if not bool(safety_cfg.get("require_aux_enable", True)):
        return True
    aux = dict(safety_cfg.get("aux_enable", {}))
    if bool(aux.get("satisfied_by_override_mode", False)) and override_active:
        return True
    if telemetry is None or not telemetry.rc_channels:
        return False
    return aux_range_enabled(
        telemetry.rc_channels,
        channel_index=int(aux.get("channel_index", 5)),
        min_us=int(aux.get("min_us", 1700)),
        max_us=int(aux.get("max_us", 2100)),
    )


def _maybe_send_rc(
    adapter: BetaflightMSPAdapter,
    args: argparse.Namespace,
    command: RcCommand,
    command_active: bool,
    config: dict[str, Any],
) -> str:
    del adapter, command, config
    if args.control_mode != "msp_raw_rc" or not args.allow_control:
        return ""
    if not command_active:
        return ""
    return "msp_io_worker_required"


def _log_path(log_dir: str, prefix: str) -> Path:
    directory = Path(log_dir)
    directory.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return directory / f"{prefix}_{stamp}.csv"


def _meta_path(log_path: Path) -> Path:
    return log_path.with_name(f"{log_path.stem}_meta.json")


def _events_path(log_path: Path) -> Path:
    return log_path.with_name(f"{log_path.stem}_events.jsonl")


def _write_run_meta(
    path: Path,
    *,
    args: argparse.Namespace,
    config: dict[str, Any],
    log_path: Path,
    fields: list[str],
    fc_identity: dict[str, Any],
    events_path: Path | None = None,
    detector_metadata: dict[str, Any] | None = None,
    fc_configuration: dict[str, Any] | None = None,
    control_authorization: dict[str, Any] | None = None,
    msp_runtime: dict[str, Any] | None = None,
    attitude_fusion: dict[str, Any] | None = None,
    guidance: dict[str, Any] | None = None,
    camera_calibration: dict[str, Any] | None = None,
    platform_health: dict[str, Any] | None = None,
    runtime_diagnostics: dict[str, Any] | None = None,
    web_telemetry: dict[str, Any] | None = None,
) -> None:
    config_path = Path(str(getattr(args, "config", ""))).expanduser()
    source_reference_path = Path(str(config.get("source_reference", {}).get("manifest", ""))).expanduser()
    meta = {
        "log_schema_version": LOG_SCHEMA_VERSION,
        "created_unix_s": time.time(),
        "created_local": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "log_csv": str(log_path),
        "log_events_jsonl": "" if events_path is None else str(events_path),
        "args": vars(args),
        "config": config,
        "fields": fields,
        "control_mode": args.control_mode,
        "allow_control": bool(args.allow_control),
        "repository_commit": _git_commit(),
        "config_path": str(config_path),
        "config_sha256": _sha256_path(config_path),
        "source_reference": {
            "path": str(source_reference_path),
            "sha256": _sha256_path(source_reference_path),
        },
        "runtime_platform": {
            "python": platform.python_version(),
            "machine": platform.machine(),
            "platform": platform.platform(),
        },
        "fc_identity": fc_identity,
        "fc_configuration": fc_configuration or {},
        "control_authorization": control_authorization or {},
        "msp_runtime": msp_runtime or {},
        "attitude_fusion": attitude_fusion or {},
        "guidance": guidance or {},
        "camera_calibration": camera_calibration or {},
        "platform_health": platform_health or {},
        "runtime_diagnostics": runtime_diagnostics or {},
        "web_telemetry": web_telemetry or {},
        "detector": detector_metadata or {},
    }
    with path.open("w") as stream:
        json.dump(meta, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _sha256_path(path: Path) -> str:
    if not path.is_file():
        return ""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def _log_fields(channel_count: int) -> list[str]:
    fields = [
        "timestamp",
        "elapsed_s",
        "safety_state",
        "safety_reason",
        "control_requested",
        "allow_control",
        "armed",
        "msp_override_available",
        "msp_override_active",
        "msp_override_mode_index",
        "physical_rc_age_s",
        "physical_rc_fresh",
        "control_snapshot_approved",
        "control_authorization_reason",
        "config_conflict_free",
        "msp_io_worker_enabled",
        "msp_transport_mode",
        "msp_request_count",
        "msp_request_error_count",
        "msp_tx_bytes",
        "msp_rx_bytes",
        "msp_set_raw_rc_attempt_count",
        "msp_set_raw_rc_success_count",
        "msp_set_raw_rc_write_attempt_count",
        "msp_set_raw_rc_write_success_count",
        "msp_set_raw_rc_write_error_count",
        "msp_set_raw_rc_ack_count",
        "msp_set_raw_rc_pending_depth",
        "msp_set_raw_rc_last_write_age_s",
        "msp_set_raw_rc_ack_age_s",
        "msp_set_raw_rc_ack_fresh",
        "msp_set_raw_rc_write_interval_s",
        "msp_set_raw_rc_write_max_interval_s",
        "msp_set_raw_rc_write_rate_hz",
        "msp_set_raw_rc_write_p50_interval_s",
        "msp_set_raw_rc_write_p95_interval_s",
        "msp_set_raw_rc_write_p99_interval_s",
        "msp_set_raw_rc_write_p999_interval_s",
        "msp_async_pending_telemetry_count",
        "msp_rx_discarded_bytes",
        "msp_rx_checksum_error_count",
        "msp_rx_parser_error_count",
        "msp_worker_poll_count",
        "msp_worker_poll_error_count",
        "msp_worker_staged_count",
        "msp_worker_send_skip_count",
        "msp_worker_send_error_count",
        "msp_worker_error",
        "msp_output_enabled",
        "msp_algorithm_authorized",
        "msp_worker_override_active",
        "msp_prefill_ready",
        "msp_prefill_success_count",
        "msp_passthrough_send_count",
        "msp_algorithm_send_count",
        "msp_stale_command_count",
        "msp_staged_command_age_s",
        "msp_publish_mode",
        "msp_last_publish_output_enabled",
        "msp_last_publish_algorithm_authorized",
        "msp_last_publish_override_active",
        "msp_last_publish_override_release_hold_active",
        "msp_last_publish_prefill_ready",
        "msp_last_publish_physical_rc_fresh",
        "msp_last_publish_command_fresh",
        "msp_last_publish_command_active",
        "msp_last_publish_command_reason",
        "msp_last_publish_set_raw_rc_ack_fresh",
        "msp_override_release_hold_active",
        "msp_rc_poll_suspended",
        "msp_status_age_s",
        "msp_attitude_age_s",
        "msp_analog_age_s",
        "msp_raw_imu_age_s",
        "msp_motor_age_s",
        "msp_publish_tick_interval_s",
        "msp_publish_tick_max_interval_s",
        "msp_publish_deadline_miss_count",
        "msp_send_success_interval_s",
        "msp_send_success_max_interval_s",
        "msp_last_send_success_age_s",
        "msp_consecutive_send_error_count",
        "throttle_handover_source_us",
        "throttle_handover_target_us",
        "throttle_handover_alpha",
        "throttle_handover_output_us",
        "throttle_handover_active",
        "rc_sent_all",
        "telemetry_fresh",
        "attitude_synced",
        "watchdog_ok",
        "voltage_ok",
        "aux_enabled",
        "telemetry_age_s",
        "attitude_age_s",
        "watchdog_age_s",
        "telemetry_error",
        "send_error",
        "cycle_time_us",
        "i2c_error_count",
        "sensor_flags",
        "mode_flags",
        "profile",
        "vbat_v",
        "mah_drawn",
        "rssi",
        "amperage_a",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "acc_raw_x",
        "acc_raw_y",
        "acc_raw_z",
        "gyro_msp_raw_x",
        "gyro_msp_raw_y",
        "gyro_msp_raw_z",
        "gyro_roll_deg_s",
        "gyro_pitch_deg_s",
        "gyro_yaw_deg_s",
        "mag_raw_x",
        "mag_raw_y",
        "mag_raw_z",
        "motor_output_count",
        "motor_output_all",
        "motor_interlock_ok",
        "motor_interlock_reason",
        "motor_interlock_latched",
        "motor_interlock_output_max_us",
        "motor_interlock_output_spread_us",
        "rc_in_count",
        "rc_in_all",
        "detector_source",
        "detector_reject_reason",
        "detector_raw_count",
        "detector_class_filtered_count",
        "detector_track_filtered_count",
        "detector_best_score",
        "rknn_selected_index",
        "rknn_preprocess_ms",
        "rknn_inference_ms",
        "rknn_postprocess_ms",
        "rknn_total_ms",
        "rknn_batch_truncated",
        "tracker_state",
        "tracker_track_id",
        "tracker_age_frames",
        "tracker_hits",
        "tracker_lost_frames",
        "tracker_confirmed",
        "tracker_high_count",
        "tracker_low_count",
        "tracker_output_count",
        "tracker_match_count",
        "tracker_match_iou",
        "tracker_association_stage",
        "tracker_switch_count",
        "tracker_fragment_count",
        "tracker_update_ms",
        "tracker_actual_fps",
        "target_selector_reason",
        "bbox_measurement_source",
        "perception_seq",
        "perception_new_result",
        "perception_worker_rate_hz",
        "perception_result_age_ms",
        "perception_queue_dropped",
        "perception_worker_error",
        "detection_attitude_offset_ms",
        "fusion_status",
        "fusion_pending_count",
        "fusion_dropped_count",
        "fusion_wait_ms",
        "loop_period_s",
        "camera_device",
        "camera_frame_ok",
        "camera_capture_ts",
        "camera_read_ms",
        "camera_input_width",
        "camera_input_height",
        "camera_output_width",
        "camera_output_height",
        "camera_requested_fps",
        "camera_reported_fps",
        "camera_reported_fourcc",
        "camera_failed_frames",
        "frame_id",
        "detection_exposure_ts",
        "detection_score",
        "bbox_x1",
        "bbox_y1",
        "bbox_x2",
        "bbox_y2",
        "bbox_width",
        "bbox_height",
        "bbox_area",
        "bbox_area_ratio",
        "bbox_clip_left",
        "bbox_clip_top",
        "bbox_clip_right",
        "bbox_clip_bottom",
        "bbox_clipped",
        "track_id",
        "los_valid",
        "los_reject_reason",
        "los_quality",
        "los_innovation_norm",
        "lambda_I_x",
        "lambda_I_y",
        "lambda_I_z",
        "lambda_dot_I_x",
        "lambda_dot_I_y",
        "lambda_dot_I_z",
        "omega_los_x",
        "omega_los_y",
        "omega_los_z",
        "guidance_law",
        "guidance_navigation_constant",
        "guidance_fixed_vm_m_s",
        "guidance_fixed_gain",
        "guidance_max_accel_mps2",
        "guidance_ttc_required",
        "guidance_eval_frame",
        "rate_gain_input_frame",
        "ttc_valid",
        "ttc_reject_reason",
        "ttc_quality",
        "ttc_s",
        "ttc_area_filtered",
        "ttc_area_dot_filtered",
        "guidance_valid",
        "guidance_reject_reason",
        "guidance_quality",
        "g_eval_x",
        "g_eval_y",
        "g_eval_z",
        "g_eval_body_frd_x",
        "g_eval_body_frd_y",
        "g_eval_body_frd_z",
        "pre_shape_sp_valid",
        "pre_shape_sp_source",
        "pre_shape_sp_reject_reason",
        "pre_shape_sp_roll_rate_deg_s",
        "pre_shape_sp_pitch_rate_deg_s",
        "shaping_valid",
        "shaping_reason",
        "entry_handoff_active",
        "entry_handoff_progress",
        "entry_handoff_source",
        "entry_handoff_start_roll_rate_deg_s",
        "entry_handoff_start_pitch_rate_deg_s",
        "tilt_roll_attitude_deg",
        "tilt_pitch_attitude_deg",
        "tilt_roll_softcap_factor",
        "tilt_pitch_softcap_factor",
        "tilt_roll_level_weight",
        "tilt_pitch_level_weight",
        "tilt_hardcap_active",
        "sp_valid",
        "sp_source",
        "sp_reject_reason",
        "sp_roll_rate_deg_s",
        "sp_pitch_rate_deg_s",
        "sp_yaw_rate_deg_s",
        "sp_thrust",
        "rc_active",
        "rc_reason",
        "map_requested_roll_rate_deg_s",
        "map_requested_pitch_rate_deg_s",
        "map_requested_yaw_rate_deg_s",
        "map_limited_roll_rate_deg_s",
        "map_limited_pitch_rate_deg_s",
        "map_limited_yaw_rate_deg_s",
        "map_roll_stick",
        "map_pitch_stick",
        "map_yaw_stick",
        "map_requested_thrust",
        "map_limited_thrust",
        "python_gc_collection_count",
        "python_gc_last_generation",
        "python_gc_last_pause_ms",
        "python_gc_max_pause_ms",
        "python_gc_total_pause_ms",
        "host_sample_age_s",
        "host_load_1m",
        "host_process_rss_mb",
        "host_mem_available_mb",
        "host_disk_free_gb",
        "host_thermal_max_c",
        "host_soc_temp_c",
        "host_npu_temp_c",
        "host_cpu_freq_min_mhz",
        "host_cpu_freq_max_mhz",
        "host_npu_freq_mhz",
        "host_health_error",
        "web_running",
        "web_sse_clients",
        "web_mjpeg_clients",
        "web_publish_count",
        "web_preview_offer_count",
        "web_preview_encode_count",
        "web_preview_drop_count",
        "web_request_count",
        "web_denied_count",
        "web_error_count",
        "web_last_error",
    ]
    for label, _command in MSP_COMMAND_LOG_SPECS:
        fields.extend(
            (
                f"msp_cmd_{label}_attempt_count",
                f"msp_cmd_{label}_success_count",
                f"msp_cmd_{label}_error_count",
                f"msp_cmd_{label}_last_rtt_ms",
                f"msp_cmd_{label}_max_rtt_ms",
                f"msp_cmd_{label}_last_success_age_s",
                f"msp_cmd_{label}_last_error",
            )
        )
    fields.extend(f"motor_output_ch{i}" for i in range(1, 9))
    fields.extend(f"rc_in_ch{i}" for i in range(1, channel_count + 1))
    fields.extend(f"rc_raw_ch{i}" for i in range(1, channel_count + 1))
    fields.extend(f"rc_target_ch{i}" for i in range(1, channel_count + 1))
    fields.extend(f"rc_ch{i}" for i in range(1, channel_count + 1))
    fields.extend(f"rc_sent_ch{i}" for i in range(1, channel_count + 1))
    fields.extend(f"rc_clipped_ch{i}" for i in range(1, channel_count + 1))
    fields.extend(f"rc_slew_limited_ch{i}" for i in range(1, channel_count + 1))
    return fields


def _log_row(
    *,
    timestamp: float,
    elapsed_s: float,
    telemetry: BetaflightTelemetry | None,
    telemetry_error: str,
    detector_stats: dict[str, Any],
    detection: FrameDetection | None,
    result: VisionGuidanceResult | None,
    pre_shape_setpoint: GuidanceSetpoint,
    setpoint: GuidanceSetpoint,
    shaping: GuidanceCommandShapingDiagnostics,
    rc_command: RcCommand,
    safety_state: str,
    safety_reason: str,
    send_error: str,
    telemetry_age_s: float | None,
    attitude_age_s: float | None,
    watchdog_age_s: float | None,
    telemetry_fresh: bool,
    attitude_synced: bool,
    watchdog_ok: bool,
    voltage_ok: bool,
    aux_enabled: bool,
    control_requested: bool,
    allow_control: bool,
    intrinsics: CameraIntrinsics,
    channel_count: int,
    guidance_body_frd: np.ndarray | None = None,
) -> dict[str, Any]:
    guidance = None if result is None else result.guidance
    los = None if result is None else result.los
    ttc = None if result is None else result.ttc
    status = None if telemetry is None else telemetry.status
    analog = None if telemetry is None else telemetry.analog
    attitude = None if telemetry is None else telemetry.attitude
    raw_imu = None if telemetry is None else telemetry.raw_imu
    motor_outputs = () if telemetry is None else telemetry.motor_outputs
    clip_flags = (
        {"left": "", "top": "", "right": "", "bottom": ""}
        if detection is None
        else detection.clip_flags(intrinsics.width, intrinsics.height)
    )
    bbox_area_ratio = "" if detection is None else detection.area / max(1.0, float(intrinsics.width * intrinsics.height))
    row: dict[str, Any] = {
        "timestamp": f"{timestamp:.6f}",
        "elapsed_s": f"{elapsed_s:.6f}",
        "safety_state": safety_state,
        "safety_reason": safety_reason,
        "control_requested": int(control_requested),
        "allow_control": int(allow_control),
        "armed": detector_stats.get("armed", ""),
        "msp_override_available": detector_stats.get("msp_override_available", ""),
        "msp_override_active": detector_stats.get("msp_override_active", ""),
        "msp_override_mode_index": detector_stats.get("msp_override_mode_index", ""),
        "physical_rc_age_s": _stats_float(detector_stats, "physical_rc_age_s", precision=6),
        "physical_rc_fresh": detector_stats.get("physical_rc_fresh", ""),
        "control_snapshot_approved": detector_stats.get("control_snapshot_approved", ""),
        "control_authorization_reason": detector_stats.get("control_authorization_reason", ""),
        "config_conflict_free": detector_stats.get("config_conflict_free", ""),
        "msp_io_worker_enabled": detector_stats.get("msp_io_worker_enabled", ""),
        "msp_transport_mode": detector_stats.get("msp_transport_mode", ""),
        "msp_request_count": detector_stats.get("msp_request_count", ""),
        "msp_request_error_count": detector_stats.get("msp_request_error_count", ""),
        "msp_tx_bytes": detector_stats.get("msp_tx_bytes", ""),
        "msp_rx_bytes": detector_stats.get("msp_rx_bytes", ""),
        "msp_set_raw_rc_attempt_count": detector_stats.get("msp_set_raw_rc_attempt_count", ""),
        "msp_set_raw_rc_success_count": detector_stats.get("msp_set_raw_rc_success_count", ""),
        "msp_set_raw_rc_write_attempt_count": detector_stats.get(
            "msp_set_raw_rc_write_attempt_count", ""
        ),
        "msp_set_raw_rc_write_success_count": detector_stats.get(
            "msp_set_raw_rc_write_success_count", ""
        ),
        "msp_set_raw_rc_write_error_count": detector_stats.get(
            "msp_set_raw_rc_write_error_count", ""
        ),
        "msp_set_raw_rc_ack_count": detector_stats.get("msp_set_raw_rc_ack_count", ""),
        "msp_set_raw_rc_pending_depth": detector_stats.get("msp_set_raw_rc_pending_depth", ""),
        "msp_set_raw_rc_last_write_age_s": _stats_float(
            detector_stats, "msp_set_raw_rc_last_write_age_s", precision=6
        ),
        "msp_set_raw_rc_ack_age_s": _stats_float(
            detector_stats, "msp_set_raw_rc_ack_age_s", precision=6
        ),
        "msp_set_raw_rc_ack_fresh": detector_stats.get("msp_set_raw_rc_ack_fresh", ""),
        "msp_set_raw_rc_write_interval_s": _stats_float(
            detector_stats, "msp_set_raw_rc_write_interval_s", precision=6
        ),
        "msp_set_raw_rc_write_max_interval_s": _stats_float(
            detector_stats, "msp_set_raw_rc_write_max_interval_s", precision=6
        ),
        "msp_set_raw_rc_write_rate_hz": _stats_float(
            detector_stats, "msp_set_raw_rc_write_rate_hz", precision=3
        ),
        "msp_set_raw_rc_write_p50_interval_s": _stats_float(
            detector_stats, "msp_set_raw_rc_write_p50_interval_s", precision=6
        ),
        "msp_set_raw_rc_write_p95_interval_s": _stats_float(
            detector_stats, "msp_set_raw_rc_write_p95_interval_s", precision=6
        ),
        "msp_set_raw_rc_write_p99_interval_s": _stats_float(
            detector_stats, "msp_set_raw_rc_write_p99_interval_s", precision=6
        ),
        "msp_set_raw_rc_write_p999_interval_s": _stats_float(
            detector_stats, "msp_set_raw_rc_write_p999_interval_s", precision=6
        ),
        "msp_async_pending_telemetry_count": detector_stats.get(
            "msp_async_pending_telemetry_count", ""
        ),
        "msp_rx_discarded_bytes": detector_stats.get("msp_rx_discarded_bytes", ""),
        "msp_rx_checksum_error_count": detector_stats.get("msp_rx_checksum_error_count", ""),
        "msp_rx_parser_error_count": detector_stats.get("msp_rx_parser_error_count", ""),
        "msp_worker_poll_count": detector_stats.get("msp_worker_poll_count", ""),
        "msp_worker_poll_error_count": detector_stats.get("msp_worker_poll_error_count", ""),
        "msp_worker_staged_count": detector_stats.get("msp_worker_staged_count", ""),
        "msp_worker_send_skip_count": detector_stats.get("msp_worker_send_skip_count", ""),
        "msp_worker_send_error_count": detector_stats.get("msp_worker_send_error_count", ""),
        "msp_worker_error": detector_stats.get("msp_worker_error", ""),
        "msp_output_enabled": detector_stats.get("msp_output_enabled", ""),
        "msp_algorithm_authorized": detector_stats.get("msp_algorithm_authorized", ""),
        "msp_worker_override_active": detector_stats.get("msp_worker_override_active", ""),
        "msp_prefill_ready": detector_stats.get("msp_prefill_ready", ""),
        "msp_prefill_success_count": detector_stats.get("msp_prefill_success_count", ""),
        "msp_passthrough_send_count": detector_stats.get("msp_passthrough_send_count", ""),
        "msp_algorithm_send_count": detector_stats.get("msp_algorithm_send_count", ""),
        "msp_stale_command_count": detector_stats.get("msp_stale_command_count", ""),
        "msp_staged_command_age_s": _stats_float(detector_stats, "msp_staged_command_age_s", precision=6),
        "msp_publish_mode": detector_stats.get("msp_publish_mode", ""),
        "msp_last_publish_output_enabled": detector_stats.get(
            "msp_last_publish_output_enabled", ""
        ),
        "msp_last_publish_algorithm_authorized": detector_stats.get(
            "msp_last_publish_algorithm_authorized", ""
        ),
        "msp_last_publish_override_active": detector_stats.get(
            "msp_last_publish_override_active", ""
        ),
        "msp_last_publish_override_release_hold_active": detector_stats.get(
            "msp_last_publish_override_release_hold_active", ""
        ),
        "msp_last_publish_prefill_ready": detector_stats.get(
            "msp_last_publish_prefill_ready", ""
        ),
        "msp_last_publish_physical_rc_fresh": detector_stats.get(
            "msp_last_publish_physical_rc_fresh", ""
        ),
        "msp_last_publish_command_fresh": detector_stats.get(
            "msp_last_publish_command_fresh", ""
        ),
        "msp_last_publish_command_active": detector_stats.get(
            "msp_last_publish_command_active", ""
        ),
        "msp_last_publish_command_reason": detector_stats.get(
            "msp_last_publish_command_reason", ""
        ),
        "msp_last_publish_set_raw_rc_ack_fresh": detector_stats.get(
            "msp_last_publish_set_raw_rc_ack_fresh", ""
        ),
        "msp_override_release_hold_active": detector_stats.get(
            "msp_override_release_hold_active", ""
        ),
        "msp_rc_poll_suspended": detector_stats.get("msp_rc_poll_suspended", ""),
        "msp_status_age_s": _stats_float(detector_stats, "msp_status_age_s", precision=6),
        "msp_attitude_age_s": _stats_float(detector_stats, "msp_attitude_age_s", precision=6),
        "msp_analog_age_s": _stats_float(detector_stats, "msp_analog_age_s", precision=6),
        "msp_raw_imu_age_s": _stats_float(detector_stats, "msp_raw_imu_age_s", precision=6),
        "msp_motor_age_s": _stats_float(detector_stats, "msp_motor_age_s", precision=6),
        "msp_publish_tick_interval_s": _stats_float(
            detector_stats, "msp_publish_tick_interval_s", precision=6
        ),
        "msp_publish_tick_max_interval_s": _stats_float(
            detector_stats, "msp_publish_tick_max_interval_s", precision=6
        ),
        "msp_publish_deadline_miss_count": detector_stats.get("msp_publish_deadline_miss_count", ""),
        "msp_send_success_interval_s": _stats_float(
            detector_stats, "msp_send_success_interval_s", precision=6
        ),
        "msp_send_success_max_interval_s": _stats_float(
            detector_stats, "msp_send_success_max_interval_s", precision=6
        ),
        "msp_last_send_success_age_s": _stats_float(
            detector_stats, "msp_last_send_success_age_s", precision=6
        ),
        "msp_consecutive_send_error_count": detector_stats.get("msp_consecutive_send_error_count", ""),
        "throttle_handover_source_us": detector_stats.get("throttle_handover_source_us", ""),
        "throttle_handover_target_us": detector_stats.get("throttle_handover_target_us", ""),
        "throttle_handover_alpha": _stats_float(detector_stats, "throttle_handover_alpha", precision=6),
        "throttle_handover_output_us": detector_stats.get("throttle_handover_output_us", ""),
        "throttle_handover_active": detector_stats.get("throttle_handover_active", ""),
        "rc_sent_all": _channels_field(detector_stats.get("msp_last_sent_channels", ())),
        "telemetry_fresh": int(telemetry_fresh),
        "attitude_synced": int(attitude_synced),
        "watchdog_ok": int(watchdog_ok),
        "voltage_ok": int(voltage_ok),
        "aux_enabled": int(aux_enabled),
        "telemetry_age_s": _format_optional_float(telemetry_age_s, precision=6),
        "attitude_age_s": _format_optional_float(attitude_age_s, precision=6),
        "watchdog_age_s": _format_optional_float(watchdog_age_s, precision=6),
        "telemetry_error": telemetry_error,
        "send_error": send_error,
        "cycle_time_us": "" if status is None else status.cycle_time_us,
        "i2c_error_count": "" if status is None else status.i2c_error_count,
        "sensor_flags": "" if status is None else status.sensor_flags,
        "mode_flags": "" if status is None else status.mode_flags,
        "profile": "" if status is None or status.profile is None else status.profile,
        "vbat_v": "" if analog is None else f"{analog.vbat_v:.2f}",
        "mah_drawn": "" if analog is None or analog.mah_drawn is None else analog.mah_drawn,
        "rssi": "" if analog is None or analog.rssi is None else analog.rssi,
        "amperage_a": "" if analog is None or analog.amperage_a is None else f"{analog.amperage_a:.3f}",
        "roll_deg": "" if attitude is None else f"{attitude.roll_deg:.3f}",
        "pitch_deg": "" if attitude is None else f"{attitude.pitch_deg:.3f}",
        "yaw_deg": "" if attitude is None else f"{attitude.yaw_deg:.3f}",
        "acc_raw_x": _sequence_field(None if raw_imu is None else raw_imu.acc_raw, 0),
        "acc_raw_y": _sequence_field(None if raw_imu is None else raw_imu.acc_raw, 1),
        "acc_raw_z": _sequence_field(None if raw_imu is None else raw_imu.acc_raw, 2),
        "gyro_msp_raw_x": _sequence_field(None if raw_imu is None else raw_imu.gyro_msp_raw, 0),
        "gyro_msp_raw_y": _sequence_field(None if raw_imu is None else raw_imu.gyro_msp_raw, 1),
        "gyro_msp_raw_z": _sequence_field(None if raw_imu is None else raw_imu.gyro_msp_raw, 2),
        "gyro_roll_deg_s": "",
        "gyro_pitch_deg_s": "",
        "gyro_yaw_deg_s": "",
        "mag_raw_x": _sequence_field(None if raw_imu is None else raw_imu.mag_raw, 0),
        "mag_raw_y": _sequence_field(None if raw_imu is None else raw_imu.mag_raw, 1),
        "mag_raw_z": _sequence_field(None if raw_imu is None else raw_imu.mag_raw, 2),
        "motor_output_count": len(motor_outputs),
        "motor_output_all": _channels_field(motor_outputs),
        "motor_interlock_ok": detector_stats.get("motor_interlock_ok", ""),
        "motor_interlock_reason": detector_stats.get("motor_interlock_reason", ""),
        "motor_interlock_latched": detector_stats.get("motor_interlock_latched", ""),
        "motor_interlock_output_max_us": _stats_float(
            detector_stats, "motor_interlock_output_max_us", precision=3
        ),
        "motor_interlock_output_spread_us": _stats_float(
            detector_stats, "motor_interlock_output_spread_us", precision=3
        ),
        "rc_in_count": "" if telemetry is None else len(telemetry.rc_channels),
        "rc_in_all": "" if telemetry is None else _channels_field(telemetry.rc_channels),
        "detector_source": detector_stats.get("detector_source", ""),
        "detector_reject_reason": detector_stats.get("detector_reject_reason", ""),
        "detector_raw_count": detector_stats.get("detector_raw_count", ""),
        "detector_class_filtered_count": detector_stats.get("detector_class_filtered_count", ""),
        "detector_track_filtered_count": detector_stats.get("detector_track_filtered_count", ""),
        "detector_best_score": _stats_float(detector_stats, "detector_best_score", precision=6),
        "rknn_selected_index": detector_stats.get("rknn_selected_index", ""),
        "rknn_preprocess_ms": _stats_float(detector_stats, "rknn_preprocess_ms", precision=3),
        "rknn_inference_ms": _stats_float(detector_stats, "rknn_inference_ms", precision=3),
        "rknn_postprocess_ms": _stats_float(detector_stats, "rknn_postprocess_ms", precision=3),
        "rknn_total_ms": _stats_float(detector_stats, "rknn_total_ms", precision=3),
        "rknn_batch_truncated": detector_stats.get("rknn_batch_truncated", ""),
        "tracker_state": detector_stats.get("tracker_state", ""),
        "tracker_track_id": detector_stats.get("tracker_track_id", ""),
        "tracker_age_frames": detector_stats.get("tracker_age_frames", ""),
        "tracker_hits": detector_stats.get("tracker_hits", ""),
        "tracker_lost_frames": detector_stats.get("tracker_lost_frames", ""),
        "tracker_confirmed": detector_stats.get("tracker_confirmed", ""),
        "tracker_high_count": detector_stats.get("tracker_high_count", ""),
        "tracker_low_count": detector_stats.get("tracker_low_count", ""),
        "tracker_output_count": detector_stats.get("tracker_output_count", ""),
        "tracker_match_count": detector_stats.get("tracker_match_count", ""),
        "tracker_match_iou": _stats_float(detector_stats, "tracker_match_iou", precision=6),
        "tracker_association_stage": detector_stats.get("tracker_association_stage", ""),
        "tracker_switch_count": detector_stats.get("tracker_switch_count", ""),
        "tracker_fragment_count": detector_stats.get("tracker_fragment_count", ""),
        "tracker_update_ms": _stats_float(detector_stats, "tracker_update_ms", precision=3),
        "tracker_actual_fps": _stats_float(detector_stats, "tracker_actual_fps", precision=3),
        "target_selector_reason": detector_stats.get("target_selector_reason", ""),
        "bbox_measurement_source": detector_stats.get("bbox_measurement_source", ""),
        "perception_seq": detector_stats.get("perception_seq", ""),
        "perception_new_result": detector_stats.get("perception_new_result", ""),
        "perception_worker_rate_hz": _stats_float(detector_stats, "perception_worker_rate_hz", precision=3),
        "perception_result_age_ms": _stats_float(detector_stats, "perception_result_age_ms", precision=3),
        "perception_queue_dropped": detector_stats.get("perception_queue_dropped", ""),
        "perception_worker_error": detector_stats.get("perception_worker_error", ""),
        "detection_attitude_offset_ms": _stats_float(
            detector_stats, "detection_attitude_offset_ms", precision=3
        ),
        "fusion_status": detector_stats.get("fusion_status", ""),
        "fusion_pending_count": detector_stats.get("fusion_pending_count", ""),
        "fusion_dropped_count": detector_stats.get("fusion_dropped_count", ""),
        "fusion_wait_ms": _stats_float(detector_stats, "fusion_wait_ms", precision=3),
        "loop_period_s": _stats_float(detector_stats, "loop_period_s", precision=6),
        "camera_device": detector_stats.get("camera_device", ""),
        "camera_frame_ok": detector_stats.get("camera_frame_ok", ""),
        "camera_capture_ts": _stats_float(detector_stats, "camera_capture_ts", precision=6),
        "camera_read_ms": _stats_float(detector_stats, "camera_read_ms", precision=3),
        "camera_input_width": detector_stats.get("camera_input_width", ""),
        "camera_input_height": detector_stats.get("camera_input_height", ""),
        "camera_output_width": detector_stats.get("camera_output_width", ""),
        "camera_output_height": detector_stats.get("camera_output_height", ""),
        "camera_requested_fps": _stats_float(detector_stats, "camera_requested_fps", precision=3),
        "camera_reported_fps": _stats_float(detector_stats, "camera_reported_fps", precision=3),
        "camera_reported_fourcc": detector_stats.get("camera_reported_fourcc", ""),
        "camera_failed_frames": detector_stats.get("camera_failed_frames", ""),
        "frame_id": "" if detection is None else detection.frame_id,
        "detection_exposure_ts": "" if detection is None else f"{detection.exposure_ts:.6f}",
        "detection_score": "" if detection is None else f"{detection.score:.6f}",
        "bbox_x1": "" if detection is None else f"{detection.bbox_xyxy[0]:.3f}",
        "bbox_y1": "" if detection is None else f"{detection.bbox_xyxy[1]:.3f}",
        "bbox_x2": "" if detection is None else f"{detection.bbox_xyxy[2]:.3f}",
        "bbox_y2": "" if detection is None else f"{detection.bbox_xyxy[3]:.3f}",
        "bbox_width": "" if detection is None else f"{detection.width:.3f}",
        "bbox_height": "" if detection is None else f"{detection.height:.3f}",
        "bbox_area": "" if detection is None else f"{detection.area:.3f}",
        "bbox_area_ratio": "" if detection is None else f"{bbox_area_ratio:.9f}",
        "bbox_clip_left": "" if detection is None else int(bool(clip_flags["left"])),
        "bbox_clip_top": "" if detection is None else int(bool(clip_flags["top"])),
        "bbox_clip_right": "" if detection is None else int(bool(clip_flags["right"])),
        "bbox_clip_bottom": "" if detection is None else int(bool(clip_flags["bottom"])),
        "bbox_clipped": "" if detection is None else int(detection.is_clipped(intrinsics.width, intrinsics.height)),
        "track_id": "" if detection is None else detection.track_id,
        "los_valid": "" if los is None else int(los.valid),
        "los_reject_reason": "" if los is None else los.reject_reason or "",
        "los_quality": "" if los is None else f"{los.quality:.6f}",
        "los_innovation_norm": "" if los is None else f"{los.innovation_norm:.9f}",
        "lambda_I_x": _vector_field(los.lambda_I if los is not None else None, 0),
        "lambda_I_y": _vector_field(los.lambda_I if los is not None else None, 1),
        "lambda_I_z": _vector_field(los.lambda_I if los is not None else None, 2),
        "lambda_dot_I_x": _vector_field(los.lambda_dot_I if los is not None else None, 0),
        "lambda_dot_I_y": _vector_field(los.lambda_dot_I if los is not None else None, 1),
        "lambda_dot_I_z": _vector_field(los.lambda_dot_I if los is not None else None, 2),
        "omega_los_x": _vector_field(los.omega_los if los is not None else None, 0),
        "omega_los_y": _vector_field(los.omega_los if los is not None else None, 1),
        "omega_los_z": _vector_field(los.omega_los if los is not None else None, 2),
        "guidance_law": detector_stats.get("guidance_law", ""),
        "guidance_navigation_constant": _stats_float(
            detector_stats, "guidance_navigation_constant", precision=6
        ),
        "guidance_fixed_vm_m_s": _stats_float(
            detector_stats, "guidance_fixed_vm_m_s", precision=6
        ),
        "guidance_fixed_gain": _stats_float(
            detector_stats, "guidance_fixed_gain", precision=6
        ),
        "guidance_max_accel_mps2": _stats_float(
            detector_stats, "guidance_max_accel_mps2", precision=6
        ),
        "guidance_ttc_required": detector_stats.get("guidance_ttc_required", ""),
        "guidance_eval_frame": detector_stats.get("guidance_eval_frame", ""),
        "rate_gain_input_frame": detector_stats.get("rate_gain_input_frame", ""),
        "ttc_valid": "" if ttc is None else int(ttc.valid),
        "ttc_reject_reason": "" if ttc is None else ttc.reject_reason or "",
        "ttc_quality": "" if ttc is None else f"{ttc.quality:.6f}",
        "ttc_s": "" if ttc is None or ttc.ttc is None else f"{ttc.ttc:.9f}",
        "ttc_area_filtered": "" if ttc is None else f"{ttc.area_filtered:.9f}",
        "ttc_area_dot_filtered": "" if ttc is None else f"{ttc.area_dot_filtered:.9f}",
        "guidance_valid": "" if guidance is None else int(guidance.valid),
        "guidance_reject_reason": "" if guidance is None else guidance.reject_reason or "",
        "guidance_quality": "" if guidance is None else f"{guidance.quality:.6f}",
        "g_eval_x": "" if guidance is None else f"{guidance.g_eval[0]:.9f}",
        "g_eval_y": "" if guidance is None else f"{guidance.g_eval[1]:.9f}",
        "g_eval_z": "" if guidance is None else f"{guidance.g_eval[2]:.9f}",
        "g_eval_body_frd_x": _vector_field(guidance_body_frd, 0),
        "g_eval_body_frd_y": _vector_field(guidance_body_frd, 1),
        "g_eval_body_frd_z": _vector_field(guidance_body_frd, 2),
        "pre_shape_sp_valid": int(pre_shape_setpoint.valid),
        "pre_shape_sp_source": pre_shape_setpoint.source,
        "pre_shape_sp_reject_reason": pre_shape_setpoint.reject_reason,
        "pre_shape_sp_roll_rate_deg_s": f"{pre_shape_setpoint.roll_rate_deg_s:.6f}",
        "pre_shape_sp_pitch_rate_deg_s": f"{pre_shape_setpoint.pitch_rate_deg_s:.6f}",
        "shaping_valid": int(shaping.valid),
        "shaping_reason": shaping.reason,
        "entry_handoff_active": int(shaping.entry_active),
        "entry_handoff_progress": f"{shaping.entry_progress:.6f}",
        "entry_handoff_source": shaping.entry_source,
        "entry_handoff_start_roll_rate_deg_s": (
            f"{shaping.entry_start_roll_rate_deg_s:.6f}"
        ),
        "entry_handoff_start_pitch_rate_deg_s": (
            f"{shaping.entry_start_pitch_rate_deg_s:.6f}"
        ),
        "tilt_roll_attitude_deg": _format_optional_float(
            shaping.roll_attitude_deg, precision=6
        ),
        "tilt_pitch_attitude_deg": _format_optional_float(
            shaping.pitch_attitude_deg, precision=6
        ),
        "tilt_roll_softcap_factor": f"{shaping.roll_softcap_factor:.6f}",
        "tilt_pitch_softcap_factor": f"{shaping.pitch_softcap_factor:.6f}",
        "tilt_roll_level_weight": f"{shaping.roll_level_weight:.6f}",
        "tilt_pitch_level_weight": f"{shaping.pitch_level_weight:.6f}",
        "tilt_hardcap_active": int(shaping.hardcap_active),
        "sp_valid": int(setpoint.valid),
        "sp_source": setpoint.source,
        "sp_reject_reason": setpoint.reject_reason,
        "sp_roll_rate_deg_s": f"{setpoint.roll_rate_deg_s:.6f}",
        "sp_pitch_rate_deg_s": f"{setpoint.pitch_rate_deg_s:.6f}",
        "sp_yaw_rate_deg_s": f"{setpoint.yaw_rate_deg_s:.6f}",
        "sp_thrust": f"{setpoint.thrust:.6f}",
        "rc_active": int(rc_command.active),
        "rc_reason": rc_command.reason,
        "map_requested_roll_rate_deg_s": _sequence_field(rc_command.requested_rates_deg_s, 0),
        "map_requested_pitch_rate_deg_s": _sequence_field(rc_command.requested_rates_deg_s, 1),
        "map_requested_yaw_rate_deg_s": _sequence_field(rc_command.requested_rates_deg_s, 2),
        "map_limited_roll_rate_deg_s": _sequence_field(rc_command.limited_rates_deg_s, 0),
        "map_limited_pitch_rate_deg_s": _sequence_field(rc_command.limited_rates_deg_s, 1),
        "map_limited_yaw_rate_deg_s": _sequence_field(rc_command.limited_rates_deg_s, 2),
        "map_roll_stick": _sequence_field(rc_command.stick_deflections, 0),
        "map_pitch_stick": _sequence_field(rc_command.stick_deflections, 1),
        "map_yaw_stick": _sequence_field(rc_command.stick_deflections, 2),
        "map_requested_thrust": "" if rc_command.requested_thrust is None else rc_command.requested_thrust,
        "map_limited_thrust": "" if rc_command.limited_thrust is None else rc_command.limited_thrust,
        "python_gc_collection_count": detector_stats.get("python_gc_collection_count", ""),
        "python_gc_last_generation": detector_stats.get("python_gc_last_generation", ""),
        "python_gc_last_pause_ms": _stats_float(
            detector_stats, "python_gc_last_pause_ms", precision=6
        ),
        "python_gc_max_pause_ms": _stats_float(
            detector_stats, "python_gc_max_pause_ms", precision=6
        ),
        "python_gc_total_pause_ms": _stats_float(
            detector_stats, "python_gc_total_pause_ms", precision=6
        ),
        "host_sample_age_s": _stats_float(detector_stats, "host_sample_age_s", precision=6),
        "host_load_1m": _stats_float(detector_stats, "host_load_1m", precision=3),
        "host_process_rss_mb": _stats_float(detector_stats, "host_process_rss_mb", precision=3),
        "host_mem_available_mb": _stats_float(detector_stats, "host_mem_available_mb", precision=3),
        "host_disk_free_gb": _stats_float(detector_stats, "host_disk_free_gb", precision=3),
        "host_thermal_max_c": _stats_float(detector_stats, "host_thermal_max_c", precision=3),
        "host_soc_temp_c": _stats_float(detector_stats, "host_soc_temp_c", precision=3),
        "host_npu_temp_c": _stats_float(detector_stats, "host_npu_temp_c", precision=3),
        "host_cpu_freq_min_mhz": _stats_float(detector_stats, "host_cpu_freq_min_mhz", precision=3),
        "host_cpu_freq_max_mhz": _stats_float(detector_stats, "host_cpu_freq_max_mhz", precision=3),
        "host_npu_freq_mhz": _stats_float(detector_stats, "host_npu_freq_mhz", precision=3),
        "host_health_error": detector_stats.get("host_health_error", ""),
        "web_running": detector_stats.get("web_running", ""),
        "web_sse_clients": detector_stats.get("web_sse_clients", ""),
        "web_mjpeg_clients": detector_stats.get("web_mjpeg_clients", ""),
        "web_publish_count": detector_stats.get("web_publish_count", ""),
        "web_preview_offer_count": detector_stats.get("web_preview_offer_count", ""),
        "web_preview_encode_count": detector_stats.get("web_preview_encode_count", ""),
        "web_preview_drop_count": detector_stats.get("web_preview_drop_count", ""),
        "web_request_count": detector_stats.get("web_request_count", ""),
        "web_denied_count": detector_stats.get("web_denied_count", ""),
        "web_error_count": detector_stats.get("web_error_count", ""),
        "web_last_error": detector_stats.get("web_last_error", ""),
    }
    for label, _command in MSP_COMMAND_LOG_SPECS:
        prefix = f"msp_cmd_{label}"
        row[f"{prefix}_attempt_count"] = detector_stats.get(f"{prefix}_attempt_count", "")
        row[f"{prefix}_success_count"] = detector_stats.get(f"{prefix}_success_count", "")
        row[f"{prefix}_error_count"] = detector_stats.get(f"{prefix}_error_count", "")
        row[f"{prefix}_last_rtt_ms"] = _stats_float(
            detector_stats, f"{prefix}_last_rtt_ms", precision=3
        )
        row[f"{prefix}_max_rtt_ms"] = _stats_float(detector_stats, f"{prefix}_max_rtt_ms", precision=3)
        row[f"{prefix}_last_success_age_s"] = _stats_float(
            detector_stats, f"{prefix}_last_success_age_s", precision=6
        )
        row[f"{prefix}_last_error"] = detector_stats.get(f"{prefix}_last_error", "")
    row.update(
        {f"motor_output_ch{i}": _sequence_field(motor_outputs, i - 1) for i in range(1, 9)}
    )
    rc_input = () if telemetry is None else telemetry.rc_channels
    row.update({f"rc_in_ch{i}": _sequence_field(rc_input, i - 1) for i in range(1, channel_count + 1)})
    row.update({f"rc_raw_ch{i}": _sequence_field(rc_command.raw_channels, i - 1) for i in range(1, channel_count + 1)})
    row.update(
        {f"rc_target_ch{i}": _sequence_field(rc_command.target_channels, i - 1) for i in range(1, channel_count + 1)}
    )
    row.update({f"rc_ch{i}": value for i, value in enumerate(rc_command.channels, start=1)})
    sent_channels = detector_stats.get("msp_last_sent_channels", ())
    row.update({f"rc_sent_ch{i}": _sequence_field(sent_channels, i - 1) for i in range(1, channel_count + 1)})
    row.update({f"rc_clipped_ch{i}": _sequence_field(rc_command.clipped_flags, i - 1) for i in range(1, channel_count + 1)})
    row.update({f"rc_slew_limited_ch{i}": _sequence_field(rc_command.slew_limited_flags, i - 1) for i in range(1, channel_count + 1)})
    return row


def _format_optional_float(value: float | None, *, precision: int) -> str:
    if value is None:
        return ""
    return f"{float(value):.{precision}f}"


def _stats_float(stats: dict[str, Any], key: str, *, precision: int) -> str:
    value = stats.get(key)
    if value in (None, ""):
        return ""
    return f"{float(value):.{precision}f}"


def _vector_field(vector: Any, index: int) -> str:
    if vector is None:
        return ""
    value = np.asarray(vector, dtype=float)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        return ""
    return f"{float(value[index]):.9f}"


def _sequence_field(values: Any, index: int) -> Any:
    if values is None or index >= len(values):
        return ""
    return values[index]


def _channels_field(values: Any) -> str:
    if values is None:
        return ""
    return ",".join(str(int(value)) for value in values)


def _float_or_none(value: Any) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


def _camera_device_value(value: Any) -> int | str:
    if isinstance(value, int):
        return value
    text = str(value).strip()
    if text.isdigit():
        return int(text)
    if not text:
        return 0
    return str(Path(text).expanduser())


def _parse_cpu_affinity(value: str | None) -> tuple[int, ...]:
    text = str(value or "").strip()
    if not text:
        return ()
    cpus: set[int] = set()
    for item in text.split(","):
        token = item.strip()
        if not token:
            raise ValueError("CPU affinity contains an empty item")
        if "-" in token:
            start_text, end_text = token.split("-", 1)
            start, end = int(start_text), int(end_text)
            if start < 0 or end < start:
                raise ValueError(f"invalid CPU affinity range: {token}")
            cpus.update(range(start, end + 1))
        else:
            cpu = int(token)
            if cpu < 0:
                raise ValueError(f"invalid CPU affinity index: {token}")
            cpus.add(cpu)
    return tuple(sorted(cpus))


def _validate_cpu_affinity_plan(
    main_cpus: tuple[int, ...],
    rknn_cpus: tuple[int, ...],
    *,
    isolate_rknn_process: bool,
) -> None:
    if rknn_cpus and not isolate_rknn_process:
        raise ValueError("--rknn-cpu-affinity requires --isolate-rknn-process")
    overlap = sorted(set(main_cpus).intersection(rknn_cpus))
    if overlap:
        raise ValueError(f"main and RKNN CPU affinity sets overlap: {overlap}")


def _apply_cpu_affinity(value: str | None, *, label: str) -> tuple[int, ...]:
    requested = _parse_cpu_affinity(value)
    if not requested:
        if hasattr(os, "sched_getaffinity"):
            return tuple(sorted(os.sched_getaffinity(0)))
        return ()
    if not hasattr(os, "sched_setaffinity") or not hasattr(os, "sched_getaffinity"):
        raise RuntimeError(f"{label} CPU affinity is unsupported on this platform")
    cpu_count = os.cpu_count()
    if cpu_count is not None and requested[-1] >= cpu_count:
        raise ValueError(f"{label} CPU affinity exceeds available CPU count {cpu_count}: {requested}")
    try:
        os.sched_setaffinity(0, set(requested))
    except OSError as exc:
        raise RuntimeError(f"failed to apply {label} CPU affinity {requested}: {exc}") from exc
    applied = tuple(sorted(os.sched_getaffinity(0)))
    if applied != requested:
        raise RuntimeError(f"{label} CPU affinity mismatch: requested={requested}, applied={applied}")
    return applied


def _decode_fourcc(value: int) -> str:
    if value <= 0:
        return ""
    return "".join(chr((int(value) >> (8 * index)) & 0xFF) for index in range(4)).rstrip("\x00")


if __name__ == "__main__":
    main()
