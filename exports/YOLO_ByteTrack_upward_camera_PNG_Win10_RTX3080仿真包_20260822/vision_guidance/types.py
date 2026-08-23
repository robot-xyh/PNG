from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


Array3 = np.ndarray
Matrix3 = np.ndarray


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass(frozen=True)
class FrameDetection:
    frame_id: int
    exposure_ts: float
    bbox_xyxy: Tuple[float, float, float, float]
    track_id: int
    score: float = 1.0

    @property
    def center(self) -> Tuple[float, float]:
        x1, y1, x2, y2 = self.bbox_xyxy
        return (0.5 * (x1 + x2), 0.5 * (y1 + y2))

    @property
    def width(self) -> float:
        x1, _, x2, _ = self.bbox_xyxy
        return max(0.0, x2 - x1)

    @property
    def height(self) -> float:
        _, y1, _, y2 = self.bbox_xyxy
        return max(0.0, y2 - y1)

    @property
    def area(self) -> float:
        return self.width * self.height

    def clip_flags(self, image_width: int, image_height: int, margin_px: float = 1.0) -> dict[str, bool]:
        x1, y1, x2, y2 = self.bbox_xyxy
        return {
            "left": x1 <= margin_px,
            "top": y1 <= margin_px,
            "right": x2 >= image_width - margin_px,
            "bottom": y2 >= image_height - margin_px,
        }

    def is_clipped(self, image_width: int, image_height: int, margin_px: float = 1.0) -> bool:
        return any(self.clip_flags(image_width, image_height, margin_px).values())


@dataclass(frozen=True)
class AttitudeSample:
    timestamp: float
    R_IB: Matrix3
    quality: float = 1.0


@dataclass(frozen=True)
class LOSEstimate:
    timestamp: float
    lambda_I: Array3
    lambda_dot_I: Array3
    omega_los: Array3
    innovation_norm: float
    quality: float
    valid: bool
    reject_reason: Optional[str] = None


@dataclass(frozen=True)
class TTCState:
    timestamp: float
    ttc: Optional[float]
    quality: float
    area_filtered: float
    area_dot_filtered: float
    valid: bool
    reject_reason: Optional[str] = None


@dataclass(frozen=True)
class GuidanceEval:
    timestamp: float
    g_eval: Array3
    valid: bool
    quality: float
    reject_reason: Optional[str] = None
