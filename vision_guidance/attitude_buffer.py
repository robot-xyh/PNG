from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import numpy as np

from .types import AttitudeSample


@dataclass(frozen=True)
class AttitudeLookup:
    sample: Optional[AttitudeSample]
    valid: bool
    reason: Optional[str] = None


class AttitudeHistoryBuffer:
    def __init__(self, duration_s: float = 1.0):
        if duration_s <= 0:
            raise ValueError("duration_s must be positive")
        self.duration_s = duration_s
        self._samples: Deque[AttitudeSample] = deque()

    def push(self, sample: AttitudeSample) -> None:
        R = np.asarray(sample.R_IB, dtype=float)
        if R.shape != (3, 3):
            raise ValueError("R_IB must be a 3x3 matrix")
        if self._samples and sample.timestamp < self._samples[-1].timestamp:
            raise ValueError("attitude timestamps must be monotonic")
        self._samples.append(AttitudeSample(sample.timestamp, R, sample.quality))
        newest = self._samples[-1].timestamp
        while self._samples and newest - self._samples[0].timestamp > self.duration_s:
            self._samples.popleft()

    def lookup(self, timestamp: float) -> AttitudeLookup:
        if not self._samples:
            return AttitudeLookup(None, False, "attitude_buffer_empty")
        if timestamp < self._samples[0].timestamp:
            return AttitudeLookup(None, False, "timestamp_before_buffer")
        if timestamp > self._samples[-1].timestamp:
            return AttitudeLookup(None, False, "timestamp_after_buffer")

        samples = list(self._samples)
        for idx, sample in enumerate(samples):
            if abs(sample.timestamp - timestamp) <= 1e-9:
                return AttitudeLookup(sample, True)
            if sample.timestamp > timestamp:
                prev = samples[idx - 1]
                curr = sample
                span = curr.timestamp - prev.timestamp
                if span <= 0:
                    return AttitudeLookup(None, False, "invalid_attitude_span")
                alpha = (timestamp - prev.timestamp) / span
                # Matrix linear interpolation is acceptable for small dt in this
                # evaluation path. A production Pixhawk path should replace this
                # with quaternion slerp.
                R = (1.0 - alpha) * prev.R_IB + alpha * curr.R_IB
                u, _, vh = np.linalg.svd(R)
                R_orth = u @ vh
                quality = min(prev.quality, curr.quality)
                return AttitudeLookup(AttitudeSample(timestamp, R_orth, quality), True)

        return AttitudeLookup(self._samples[-1], True)

    def __len__(self) -> int:
        return len(self._samples)
