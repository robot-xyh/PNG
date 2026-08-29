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
        if not math.isfinite(max_norm) or max_norm <= 0.0:
            raise ValueError("max_norm must be finite and positive")
        self.gain_schedule = gain_schedule or TTCGainSchedule()
        self.max_norm = float(max_norm)

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


class FixedVmGuidanceEvaluator:
    """Evaluate PNG using an explicitly configured constant missile speed."""

    def __init__(
        self,
        navigation_constant: float,
        fixed_vm_m_s: float,
        max_norm: float,
    ) -> None:
        for name, value in (
            ("navigation_constant", navigation_constant),
            ("fixed_vm_m_s", fixed_vm_m_s),
            ("max_norm", max_norm),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self.navigation_constant = float(navigation_constant)
        self.fixed_vm_m_s = float(fixed_vm_m_s)
        self.fixed_gain = self.navigation_constant * self.fixed_vm_m_s
        self.max_norm = float(max_norm)

    def evaluate(self, los: LOSEstimate, ttc: TTCState) -> GuidanceEval:
        del ttc  # Fixed-VM PNG depends on LOS kinematics, not scale-expansion TTC.
        if not los.valid:
            return GuidanceEval(
                los.timestamp,
                np.zeros(3),
                False,
                0.0,
                los.reject_reason or "los_invalid",
            )

        g_eval = self.fixed_gain * np.cross(los.omega_los, los.lambda_I)
        if not np.all(np.isfinite(g_eval)):
            return GuidanceEval(los.timestamp, np.zeros(3), False, 0.0, "los_nonfinite")
        norm = float(np.linalg.norm(g_eval))
        if norm > self.max_norm:
            g_eval = g_eval * (self.max_norm / norm)
        return GuidanceEval(los.timestamp, g_eval, True, los.quality)
