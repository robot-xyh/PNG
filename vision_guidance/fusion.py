from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Optional

import numpy as np

from .attitude_buffer import AttitudeHistoryBuffer
from .geometry import camera_ray_from_pixel, los_camera_to_inertial
from .los_filter import LOSKalmanFilter6D
from .png_eval import FixedVmGuidanceEvaluator, GuidanceEvaluator
from .ttc import ScaleExpansionTTC
from .types import CameraIntrinsics, FrameDetection, GuidanceEval, LOSEstimate, TTCState


@dataclass(frozen=True)
class VisionGuidanceResult:
    detection: FrameDetection
    los: Optional[LOSEstimate]
    ttc: Optional[TTCState]
    guidance: GuidanceEval
    R_IB: Optional[np.ndarray] = None


@dataclass(frozen=True)
class DeferredFusionResult:
    detection: FrameDetection | None
    context: Any
    result: VisionGuidanceResult | None
    status: str
    pending_count: int
    dropped_count: int
    wait_ms: float | None = None


@dataclass(frozen=True)
class _PendingDetection:
    detection: FrameDetection
    context: Any
    queued_at: float


class PureVisionGuidancePipeline:
    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        R_BC: np.ndarray,
        attitude_buffer: AttitudeHistoryBuffer,
        los_filter: LOSKalmanFilter6D | None = None,
        ttc_filter: ScaleExpansionTTC | None = None,
        evaluator: GuidanceEvaluator | FixedVmGuidanceEvaluator | None = None,
    ):
        self.intrinsics = intrinsics
        self.R_BC = np.asarray(R_BC, dtype=float)
        self.attitude_buffer = attitude_buffer
        self.los_filter = los_filter or LOSKalmanFilter6D()
        self.ttc_filter = ttc_filter or ScaleExpansionTTC()
        self.evaluator = evaluator or GuidanceEvaluator()
        self.active_track_id: Optional[int] = None

    def process(self, detection: FrameDetection) -> VisionGuidanceResult:
        if self.active_track_id is None:
            self.active_track_id = detection.track_id
        elif detection.track_id != self.active_track_id:
            self.active_track_id = detection.track_id
            self.los_filter.reset()
            self.ttc_filter.reset()
            return self._reject(detection, "track_id_changed")

        lookup = self.attitude_buffer.lookup(detection.exposure_ts)
        if not lookup.valid or lookup.sample is None:
            return self._reject(detection, lookup.reason or "attitude_lookup_failed")

        los_C = camera_ray_from_pixel(*detection.center, self.intrinsics)
        lambda_measured = los_camera_to_inertial(los_C, self.R_BC, lookup.sample.R_IB)
        los = self.los_filter.update(detection.exposure_ts, lambda_measured)
        ttc = self.ttc_filter.update(detection, self.intrinsics.width, self.intrinsics.height)
        guidance = self.evaluator.evaluate(los, ttc)
        return VisionGuidanceResult(
            detection,
            los,
            ttc,
            guidance,
            np.array(lookup.sample.R_IB, dtype=float, copy=True),
        )

    def _reject(self, detection: FrameDetection, reason: str) -> VisionGuidanceResult:
        guidance = GuidanceEval(detection.exposure_ts, np.zeros(3), False, 0.0, reason)
        return VisionGuidanceResult(detection, None, None, guidance, None)


class DeferredAttitudeFusion:
    """Wait briefly for attitude samples that bracket asynchronous detections."""

    def __init__(
        self,
        pipeline: PureVisionGuidancePipeline,
        *,
        max_wait_s: float = 0.20,
        max_pending: int = 8,
    ) -> None:
        if max_wait_s <= 0.0:
            raise ValueError("max_wait_s must be positive")
        if max_pending <= 0:
            raise ValueError("max_pending must be positive")
        self.pipeline = pipeline
        self.max_wait_s = float(max_wait_s)
        self.max_pending = int(max_pending)
        self._pending: Deque[_PendingDetection] = deque()
        self._dropped_count = 0

    def update(
        self,
        detection: FrameDetection | None,
        *,
        timestamp: float,
        context: Any = None,
        perception_new_result: bool = True,
    ) -> DeferredFusionResult:
        now = float(timestamp)
        if perception_new_result and detection is None:
            self._dropped_count += len(self._pending)
            self._pending.clear()
            return self._state(None, context, None, "no_detection")

        if detection is not None:
            self._pending.append(_PendingDetection(detection, context, now))
            while len(self._pending) > self.max_pending:
                self._pending.popleft()
                self._dropped_count += 1

        if not self._pending:
            status = "no_new_result" if not perception_new_result else "idle"
            return self._state(None, context, None, status)

        pending = self._pending[0]
        latest_attitude = self.pipeline.attitude_buffer.latest_timestamp
        wait_s = max(0.0, now - pending.queued_at)
        if latest_attitude is not None and pending.detection.exposure_ts <= latest_attitude:
            self._pending.popleft()
            result = self.pipeline.process(pending.detection)
            return self._state(
                pending.detection,
                pending.context,
                result,
                "processed",
                wait_ms=1000.0 * wait_s,
            )

        if wait_s >= self.max_wait_s:
            self._pending.popleft()
            result = self.pipeline.process(pending.detection)
            return self._state(
                pending.detection,
                pending.context,
                result,
                "attitude_wait_timeout",
                wait_ms=1000.0 * wait_s,
            )

        return self._state(
            None,
            context,
            None,
            "waiting_for_attitude",
            wait_ms=1000.0 * wait_s,
        )

    def _state(
        self,
        detection: FrameDetection | None,
        context: Any,
        result: VisionGuidanceResult | None,
        status: str,
        *,
        wait_ms: float | None = None,
    ) -> DeferredFusionResult:
        return DeferredFusionResult(
            detection=detection,
            context=context,
            result=result,
            status=status,
            pending_count=len(self._pending),
            dropped_count=self._dropped_count,
            wait_ms=wait_ms,
        )
