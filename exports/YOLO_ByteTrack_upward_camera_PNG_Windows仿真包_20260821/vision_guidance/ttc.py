from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np

from .types import FrameDetection, TTCState


@dataclass(frozen=True)
class TTCConfig:
    alpha_area: float = 0.25
    window_size: int = 5
    min_area: float = 16.0
    max_area_jump_ratio: float = 2.5
    min_area_dot: float = 1e-6
    max_ttc_s: float = 20.0


class ScaleExpansionTTC:
    def __init__(self, config: TTCConfig | None = None):
        self.config = config or TTCConfig()
        self.area_filtered: Optional[float] = None
        self.prev_raw_area: Optional[float] = None
        self.window: Deque[Tuple[float, float]] = deque(maxlen=self.config.window_size)

    def reset(self) -> None:
        self.area_filtered = None
        self.prev_raw_area = None
        self.window.clear()

    def update(self, detection: FrameDetection, image_width: int, image_height: int) -> TTCState:
        area = detection.area
        ts = detection.exposure_ts
        if area < self.config.min_area:
            return self._state(ts, None, 0.0, 0.0, False, "bbox_area_too_small")
        clip_reason = _bbox_clip_reason(detection, image_width, image_height)
        if clip_reason:
            return self._state(ts, None, 0.0, 0.0, False, clip_reason)
        if self.prev_raw_area is not None:
            ratio = max(area, self.prev_raw_area) / max(1e-9, min(area, self.prev_raw_area))
            if ratio > self.config.max_area_jump_ratio:
                self.prev_raw_area = area
                return self._state(ts, None, 0.0, 0.0, False, "bbox_area_jump")
        self.prev_raw_area = area

        if self.area_filtered is None:
            self.area_filtered = area
        else:
            self.area_filtered = (
                self.config.alpha_area * area + (1.0 - self.config.alpha_area) * self.area_filtered
            )
        self.window.append((ts, self.area_filtered))

        area_dot = self._slope()
        if area_dot is None or area_dot <= self.config.min_area_dot:
            return self._state(ts, None, 0.2, 0.0 if area_dot is None else area_dot, False, "area_not_expanding")

        ttc = 2.0 * self.area_filtered / area_dot
        if not np.isfinite(ttc) or ttc <= 0.0 or ttc > self.config.max_ttc_s:
            return self._state(ts, ttc if np.isfinite(ttc) else None, 0.3, area_dot, False, "ttc_out_of_range")
        quality = max(0.0, min(1.0, 1.0 - ttc / self.config.max_ttc_s))
        return self._state(ts, ttc, quality, area_dot, True)

    def _slope(self) -> Optional[float]:
        if len(self.window) < 2:
            return None
        t = np.array([p[0] for p in self.window], dtype=float)
        a = np.array([p[1] for p in self.window], dtype=float)
        t = t - t.mean()
        denom = float(np.dot(t, t))
        if denom <= 1e-12:
            return None
        return float(np.dot(t, a - a.mean()) / denom)

    def _state(
        self,
        timestamp: float,
        ttc: Optional[float],
        quality: float,
        area_dot: float,
        valid: bool,
        reason: Optional[str] = None,
    ) -> TTCState:
        return TTCState(
            timestamp=timestamp,
            ttc=ttc,
            quality=quality,
            area_filtered=0.0 if self.area_filtered is None else self.area_filtered,
            area_dot_filtered=area_dot,
            valid=valid,
            reject_reason=reason,
        )


def _bbox_clip_reason(detection: FrameDetection, image_width: int, image_height: int) -> str:
    flags = detection.clip_flags(image_width, image_height)
    for name in ("top", "bottom", "left", "right"):
        if flags.get(name, False):
            return f"bbox_{name}_clipped"
    return ""
