from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .attitude_buffer import AttitudeHistoryBuffer
from .geometry import camera_ray_from_pixel, los_camera_to_inertial
from .los_filter import LOSKalmanFilter6D
from .png_eval import GuidanceEvaluator
from .ttc import ScaleExpansionTTC
from .types import CameraIntrinsics, FrameDetection, GuidanceEval, LOSEstimate, TTCState


@dataclass(frozen=True)
class VisionGuidanceResult:
    detection: FrameDetection
    los: Optional[LOSEstimate]
    ttc: Optional[TTCState]
    guidance: GuidanceEval


class PureVisionGuidancePipeline:
    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        R_BC: np.ndarray,
        attitude_buffer: AttitudeHistoryBuffer,
        los_filter: LOSKalmanFilter6D | None = None,
        ttc_filter: ScaleExpansionTTC | None = None,
        evaluator: GuidanceEvaluator | None = None,
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
        return VisionGuidanceResult(detection, los, ttc, guidance)

    def _reject(self, detection: FrameDetection, reason: str) -> VisionGuidanceResult:
        guidance = GuidanceEval(detection.exposure_ts, np.zeros(3), False, 0.0, reason)
        return VisionGuidanceResult(detection, None, None, guidance)
