from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .betaflight_intercept_controller import (
    InterceptPhase,
    VelocityEstablishingPngController,
    VelocityEstablishingPngInput,
    VelocityEstablishingPngOutput,
)
from .betaflight_kinematics import VehicleKinematicState
from .fusion import VisionGuidanceResult
from .types import GuidanceEval


VELOCITY_SOURCES = ("msp_kinematics", "bench_zero_velocity")


@dataclass(frozen=True)
class VelocityEstablishingRuntimeResult:
    result: VisionGuidanceResult
    controller: VelocityEstablishingPngOutput
    velocity_source: str
    velocity_reason: str


class VelocityEstablishingPngRuntime:
    """Connect filtered vision and explicit vehicle velocity to the intercept controller."""

    def __init__(
        self,
        controller: VelocityEstablishingPngController,
        *,
        velocity_source: str,
        image_width: int,
        image_height: int,
    ) -> None:
        source = str(velocity_source).strip().lower()
        if source not in VELOCITY_SOURCES:
            raise ValueError(
                f"unsupported velocity source {source!r}; expected one of {VELOCITY_SOURCES}"
            )
        if image_width <= 0 or image_height <= 0:
            raise ValueError("image dimensions must be positive")
        self.controller = controller
        self.velocity_source = source
        self.image_area = float(image_width * image_height)
        self._last_valid_vision: VisionGuidanceResult | None = None

    def update(
        self,
        *,
        timestamp_s: float,
        vision_result: VisionGuidanceResult | None,
        attitude_R_IB: np.ndarray | None,
        attitude_valid: bool,
        kinematics: VehicleKinematicState,
        engagement_active: bool = True,
    ) -> VelocityEstablishingRuntimeResult:
        if (
            not engagement_active
            and self.controller.phase in (InterceptPhase.ABORT, InterceptPhase.COMPLETE)
            and vision_result is not None
            and vision_result.los is not None
            and vision_result.los.valid
        ):
            self.controller.reset()
        source_result = vision_result
        if (
            vision_result is not None
            and vision_result.los is not None
            and vision_result.los.valid
        ):
            self._last_valid_vision = vision_result
        elif vision_result is None:
            source_result = self._last_valid_vision

        los = None if source_result is None else source_result.los
        tracking_valid = bool(los is not None and los.valid)
        tracking_reason = None
        if source_result is not None and not tracking_valid:
            tracking_reason = (
                source_result.guidance.reject_reason
                or (None if los is None else los.reject_reason)
                or "tracking_invalid"
            )

        velocity, velocity_timestamp_s, velocity_valid, velocity_reason = self._velocity(
            timestamp_s,
            kinematics,
        )
        detection = None if source_result is None else source_result.detection
        ttc = None if source_result is None else source_result.ttc
        bbox_area_ratio = (
            None if detection is None else float(detection.area) / self.image_area
        )
        controller_output = self.controller.update(
            VelocityEstablishingPngInput(
                timestamp_s=timestamp_s,
                los_timestamp_s=None if los is None else los.timestamp,
                lambda_ned=None if los is None else los.lambda_I,
                lambda_dot_ned_s=None if los is None else los.lambda_dot_I,
                tracking_valid=tracking_valid,
                bbox_area_ratio=bbox_area_ratio,
                attitude_R_IB=attitude_R_IB,
                attitude_valid=attitude_valid,
                velocity_timestamp_s=velocity_timestamp_s,
                velocity_ned_m_s=velocity,
                velocity_valid=velocity_valid,
                tracking_reason=tracking_reason,
                ttc_valid=bool(ttc is not None and ttc.valid),
                ttc_s=None if ttc is None else ttc.ttc,
                track_id=None if detection is None else detection.track_id,
            )
        )
        quality = 0.0 if los is None else float(los.quality)
        guidance = GuidanceEval(
            timestamp=float(timestamp_s),
            g_eval=np.asarray(controller_output.acceleration_ned_m_s2, dtype=float),
            valid=controller_output.valid,
            quality=quality if controller_output.valid else 0.0,
            reject_reason=None if controller_output.valid else controller_output.reason,
        )
        result = VisionGuidanceResult(
            detection=(
                source_result.detection
                if source_result is not None
                else _empty_detection(timestamp_s)
            ),
            los=los,
            ttc=ttc,
            guidance=guidance,
            R_IB=(
                None
                if attitude_R_IB is None
                else np.array(attitude_R_IB, dtype=float, copy=True)
            ),
        )
        return VelocityEstablishingRuntimeResult(
            result=result,
            controller=controller_output,
            velocity_source=self.velocity_source,
            velocity_reason=velocity_reason,
        )

    def _velocity(
        self,
        timestamp_s: float,
        kinematics: VehicleKinematicState,
    ) -> tuple[np.ndarray | None, float | None, bool, str]:
        if self.velocity_source == "bench_zero_velocity":
            return np.zeros(3, dtype=float), float(timestamp_s), True, "bench_zero_velocity"

        values = kinematics.velocity_ned_filtered_m_s
        valid = bool(
            kinematics.valid
            and all(value is not None and np.isfinite(value) for value in values)
        )
        if not valid:
            return None, None, False, kinematics.reason
        ages = (kinematics.gps_age_s, kinematics.altitude_age_s)
        if any(value is None or not np.isfinite(value) for value in ages):
            return None, None, False, "kinematics_age_missing"
        oldest_age_s = max(float(value) for value in ages if value is not None)
        return (
            np.asarray(values, dtype=float),
            float(timestamp_s) - max(0.0, oldest_age_s),
            True,
            "msp_kinematics",
        )


def _empty_detection(timestamp_s: float):
    # VisionGuidanceResult retains a concrete detection for legacy logging APIs.
    from .types import FrameDetection

    return FrameDetection(
        frame_id=-1,
        exposure_ts=float(timestamp_s),
        bbox_xyxy=(0.0, 0.0, 0.0, 0.0),
        track_id=-1,
        score=0.0,
    )
