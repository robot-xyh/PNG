from __future__ import annotations

import math

import numpy as np

from .types import GuidanceEval, LOSEstimate, TTCState


class TTCGainSchedule:
    def __init__(self, min_gain: float = 0.5, max_gain: float = 5.0, ttc_fast_s: float = 1.0, ttc_slow_s: float = 6.0):
        if min_gain < 0 or max_gain < min_gain:
            raise ValueError("invalid gain range")
        self.min_gain = min_gain
        self.max_gain = max_gain
        self.ttc_fast_s = ttc_fast_s
        self.ttc_slow_s = ttc_slow_s

    def gain(self, ttc: float) -> float:
        if ttc <= self.ttc_fast_s:
            return self.max_gain
        if ttc >= self.ttc_slow_s:
            return self.min_gain
        span = self.ttc_slow_s - self.ttc_fast_s
        x = (self.ttc_slow_s - ttc) / span
        smooth = 0.5 - 0.5 * math.cos(math.pi * x)
        return self.min_gain + (self.max_gain - self.min_gain) * smooth


class GuidanceEvaluator:
    def __init__(self, gain_schedule: TTCGainSchedule | None = None, max_norm: float = 10.0):
        self.gain_schedule = gain_schedule or TTCGainSchedule()
        self.max_norm = max_norm

    def evaluate(self, los: LOSEstimate, ttc: TTCState) -> GuidanceEval:
        if not los.valid:
            return GuidanceEval(los.timestamp, np.zeros(3), False, 0.0, los.reject_reason or "los_invalid")
        if not ttc.valid or ttc.ttc is None:
            return GuidanceEval(los.timestamp, np.zeros(3), False, 0.0, ttc.reject_reason or "ttc_invalid")
        gain = self.gain_schedule.gain(ttc.ttc)
        g_eval = gain * los.lambda_dot_I
        norm = float(np.linalg.norm(g_eval))
        if norm > self.max_norm:
            g_eval = g_eval * (self.max_norm / norm)
        quality = min(los.quality, ttc.quality)
        return GuidanceEval(los.timestamp, g_eval, True, quality)
