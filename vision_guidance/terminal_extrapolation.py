from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


WAITING = "Waiting"
TRACKING = "Tracking"
TERMINAL_VISUAL = "TerminalVisual"
BLIND_PUSH = "BlindPush"
LOSS_HOLD = "LossHold"
COMPLETE = "Complete"
ABORT_HOLD = "AbortHold"
DISABLED = "Disabled"


@dataclass(frozen=True)
class TerminalConfig:
    enable: bool = True
    terminal_enter_area_ratio: float = 0.20
    soft_enter_area_ratio: float = 0.05
    cutoff_area_ratio: float = 0.60
    terminal_gimbal_limit_area_ratio: float = 0.05
    cutoff_miss_count: int = 3
    min_tracking_time_s: float = 0.20
    confidence_min_score: float = 0.35
    max_measurement_age_s: float = 0.12
    blind_duration_s: float = 0.25
    command_average_window_s: float = 0.10
    command_decay_tau_s: float = 0.18
    trend_bias_gain: float = 0.10
    trend_bias_max_mps: float = 1.5
    pitch_up_bias_mps: float = 0.8
    abort_on_tilt_hardcap: bool = True


@dataclass(frozen=True)
class TerminalSample:
    timestamp: float
    v_cmd: np.ndarray
    lambda_I: Optional[np.ndarray]
    omega_los: Optional[np.ndarray]
    area_ratio: float
    score: float


@dataclass(frozen=True)
class TerminalResult:
    v_cmd: np.ndarray
    state: str
    reason: str
    area_ratio: float
    miss_count: int
    using_blind_push: bool
    blind_elapsed_s: float
    blind_decay: float
    blind_sample_count: int
    v_cmd_base: np.ndarray
    v_cmd_trend_bias: np.ndarray
    v_cmd_pitch_up_bias: np.ndarray
    terminal_arm_source: str
    terminal_cutoff_source: str


class TerminalExtrapolator:
    def __init__(self, config: TerminalConfig | None = None):
        self.config = config or TerminalConfig()
        self.state = WAITING
        self.tracking_start_ts: Optional[float] = None
        self.last_valid_detection_ts: Optional[float] = None
        self.miss_count = 0
        self.terminal_armed = False
        self.blind_start_ts: Optional[float] = None
        self.blind_reason = ""
        self.terminal_arm_source = ""
        self.terminal_cutoff_source = ""
        self.samples: list[TerminalSample] = []
        self.blind_base_v_cmd = np.zeros(3, dtype=float)
        self.blind_trend_bias = np.zeros(3, dtype=float)
        self.blind_pitch_bias = np.zeros(3, dtype=float)
        self.blind_sample_count = 0

    def update(
        self,
        *,
        timestamp: float,
        detected: bool,
        measurement_valid: bool,
        measurement_score: float,
        bbox_area: float,
        image_width: int,
        image_height: int,
        reject_reason: str,
        v_cmd: np.ndarray,
        lambda_I: Optional[np.ndarray],
        omega_los: Optional[np.ndarray],
        speed_cap: float,
        max_vertical_speed: float,
        gimbal_at_limit: bool = False,
        safety_ok: bool = True,
        soft_measurement_valid: bool = False,
    ) -> TerminalResult:
        command = _as_vector(v_cmd)
        area_ratio = self._area_ratio(bbox_area, image_width, image_height)
        if not self.config.enable:
            self.state = DISABLED
            return self._passthrough(command, area_ratio)

        if not safety_ok:
            self.state = ABORT_HOLD
            self.blind_start_ts = None
            self.terminal_cutoff_source = "safety_abort"
            return self._passthrough(command, area_ratio, reason="safety_abort", cutoff_source="safety_abort")

        self._record_if_valid(
            timestamp=timestamp,
            detected=detected,
            measurement_valid=measurement_valid,
            measurement_score=measurement_score,
            v_cmd=command,
            lambda_I=lambda_I,
            omega_los=omega_los,
            area_ratio=area_ratio,
        )

        if self.state == BLIND_PUSH:
            elapsed = max(0.0, timestamp - float(self.blind_start_ts or timestamp))
            if elapsed > max(0.0, self.config.blind_duration_s):
                self.state = COMPLETE
                self.blind_start_ts = None
                return self._passthrough(command, area_ratio, reason="blind_complete")
            return self._blind_result(timestamp, area_ratio, speed_cap, max_vertical_speed)

        if self.state in {COMPLETE, ABORT_HOLD} and detected and measurement_valid:
            self._reset_terminal_flags()

        terminal_confident = self._terminal_confident(timestamp)
        cutoff_reason, cutoff_source = self._cutoff_reason(detected, area_ratio, reject_reason, gimbal_at_limit)
        score_valid = measurement_score >= self.config.confidence_min_score
        normal_measurement = bool(detected and measurement_valid and score_valid)
        soft_terminal_measurement = bool(
            detected
            and soft_measurement_valid
            and area_ratio >= max(0.0, self.config.soft_enter_area_ratio)
        )

        if normal_measurement:
            self.miss_count = 0
            if area_ratio >= max(0.0, self.config.terminal_enter_area_ratio):
                self.state = TERMINAL_VISUAL
                self.terminal_armed = True
                if not self.terminal_arm_source:
                    self.terminal_arm_source = "valid_guidance"
            else:
                self.state = TRACKING
        elif soft_terminal_measurement:
            self.miss_count = 0
            self.state = TERMINAL_VISUAL
            self.terminal_armed = True
            if not self.terminal_arm_source:
                self.terminal_arm_source = "image_kf_soft"
        elif self.terminal_armed:
            self.miss_count += 1
            self.state = TERMINAL_VISUAL
        else:
            self.state = LOSS_HOLD if not detected else TRACKING

        if not cutoff_reason and self.terminal_armed and not detected and self.miss_count >= self.config.cutoff_miss_count:
            cutoff_reason = "terminal_lost"
            cutoff_source = "terminal_lost"

        cutoff_allowed = bool(terminal_confident or soft_measurement_valid or self._has_recent_samples(timestamp))
        if cutoff_reason and cutoff_allowed and self._has_recent_samples(timestamp):
            self._enter_blind(timestamp, cutoff_reason, speed_cap, cutoff_source)
            return self._blind_result(timestamp, area_ratio, speed_cap, max_vertical_speed)

        return self._passthrough(command, area_ratio, reason=cutoff_reason, cutoff_source=cutoff_source)

    def _record_if_valid(
        self,
        *,
        timestamp: float,
        detected: bool,
        measurement_valid: bool,
        measurement_score: float,
        v_cmd: np.ndarray,
        lambda_I: Optional[np.ndarray],
        omega_los: Optional[np.ndarray],
        area_ratio: float,
    ) -> None:
        if detected and measurement_valid and measurement_score >= self.config.confidence_min_score:
            if self.tracking_start_ts is None:
                self.tracking_start_ts = timestamp
            self.last_valid_detection_ts = timestamp
            if np.all(np.isfinite(v_cmd)):
                self.samples.append(
                    TerminalSample(
                        timestamp=timestamp,
                        v_cmd=np.array(v_cmd, dtype=float),
                        lambda_I=None if lambda_I is None else np.array(lambda_I, dtype=float),
                        omega_los=None if omega_los is None else np.array(omega_los, dtype=float),
                        area_ratio=area_ratio,
                        score=measurement_score,
                    )
                )
        self._prune_samples(timestamp)

    def _prune_samples(self, timestamp: float) -> None:
        keep_s = max(1.0, 4.0 * max(0.01, self.config.command_average_window_s), self.config.blind_duration_s)
        self.samples = [sample for sample in self.samples if timestamp - sample.timestamp <= keep_s]

    def _terminal_confident(self, timestamp: float) -> bool:
        if self.tracking_start_ts is None or self.last_valid_detection_ts is None:
            return False
        if timestamp - self.tracking_start_ts < self.config.min_tracking_time_s:
            return False
        return timestamp - self.last_valid_detection_ts <= self.config.max_measurement_age_s

    def _has_recent_samples(self, timestamp: float) -> bool:
        return bool(self._window_samples(timestamp))

    def _window_samples(self, timestamp: float) -> list[TerminalSample]:
        window = max(0.01, self.config.command_average_window_s)
        samples = [sample for sample in self.samples if timestamp - sample.timestamp <= window]
        if samples:
            return samples
        return self.samples[-1:] if self.samples else []

    def _cutoff_reason(self, detected: bool, area_ratio: float, reject_reason: str, gimbal_at_limit: bool) -> tuple[str, str]:
        if reject_reason == "bbox_clipped":
            return "bbox_clipped", "bbox_clipped"
        if detected and area_ratio >= max(0.0, self.config.cutoff_area_ratio):
            return "bbox_area_large", "bbox_area_large"
        if detected and gimbal_at_limit and area_ratio >= max(0.0, self.config.terminal_gimbal_limit_area_ratio):
            return "gimbal_limit", "gimbal_limit"
        return "", ""

    def _enter_blind(self, timestamp: float, reason: str, speed_cap: float, cutoff_source: str = "") -> None:
        samples = self._window_samples(timestamp)
        self.blind_start_ts = timestamp
        self.blind_reason = reason
        self.terminal_cutoff_source = cutoff_source or reason
        self.state = BLIND_PUSH
        self.blind_sample_count = len(samples)
        self.blind_base_v_cmd = np.mean([sample.v_cmd for sample in samples], axis=0)
        omega_samples = [sample.omega_los for sample in samples if sample.omega_los is not None]
        if omega_samples:
            trend = speed_cap * self.config.trend_bias_gain * np.mean(omega_samples, axis=0)
            self.blind_trend_bias = _clamp_norm(trend, max(0.0, self.config.trend_bias_max_mps))
        else:
            self.blind_trend_bias = np.zeros(3, dtype=float)
        self.blind_pitch_bias = np.array([0.0, 0.0, -max(0.0, self.config.pitch_up_bias_mps)], dtype=float)

    def _blind_result(
        self,
        timestamp: float,
        area_ratio: float,
        speed_cap: float,
        max_vertical_speed: float,
    ) -> TerminalResult:
        elapsed = max(0.0, timestamp - float(self.blind_start_ts or timestamp))
        tau = max(1.0e-6, self.config.command_decay_tau_s)
        decay = float(np.exp(-elapsed / tau))
        command = self.blind_base_v_cmd + decay * (self.blind_trend_bias + self.blind_pitch_bias)
        if max_vertical_speed > 0.0:
            command[2] = float(np.clip(command[2], -max_vertical_speed, max_vertical_speed))
        command = _clamp_norm(command, speed_cap)
        return TerminalResult(
            v_cmd=command,
            state=self.state,
            reason=self.blind_reason,
            area_ratio=area_ratio,
            miss_count=self.miss_count,
            using_blind_push=True,
            blind_elapsed_s=elapsed,
            blind_decay=decay,
            blind_sample_count=self.blind_sample_count,
            v_cmd_base=np.array(self.blind_base_v_cmd, dtype=float),
            v_cmd_trend_bias=decay * np.array(self.blind_trend_bias, dtype=float),
            v_cmd_pitch_up_bias=decay * np.array(self.blind_pitch_bias, dtype=float),
            terminal_arm_source=self.terminal_arm_source,
            terminal_cutoff_source=self.terminal_cutoff_source,
        )

    def _passthrough(
        self,
        command: np.ndarray,
        area_ratio: float,
        reason: str = "",
        cutoff_source: str = "",
    ) -> TerminalResult:
        if cutoff_source:
            self.terminal_cutoff_source = cutoff_source
        return TerminalResult(
            v_cmd=np.array(command, dtype=float),
            state=self.state,
            reason=reason,
            area_ratio=area_ratio,
            miss_count=self.miss_count,
            using_blind_push=False,
            blind_elapsed_s=0.0,
            blind_decay=0.0,
            blind_sample_count=0,
            v_cmd_base=np.zeros(3, dtype=float),
            v_cmd_trend_bias=np.zeros(3, dtype=float),
            v_cmd_pitch_up_bias=np.zeros(3, dtype=float),
            terminal_arm_source=self.terminal_arm_source,
            terminal_cutoff_source=self.terminal_cutoff_source if reason else "",
        )

    def _reset_terminal_flags(self) -> None:
        self.state = TRACKING
        self.terminal_armed = False
        self.blind_start_ts = None
        self.blind_reason = ""
        self.terminal_arm_source = ""
        self.terminal_cutoff_source = ""
        self.miss_count = 0

    @staticmethod
    def _area_ratio(bbox_area: float, image_width: int, image_height: int) -> float:
        image_area = max(1.0, float(image_width) * float(image_height))
        return max(0.0, float(bbox_area)) / image_area


def _as_vector(value: np.ndarray) -> np.ndarray:
    vector = np.asarray(value, dtype=float).reshape(3)
    return np.array(vector, dtype=float)


def _clamp_norm(vector: np.ndarray, limit: float) -> np.ndarray:
    vector = np.array(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if limit > 0.0 and norm > limit and norm > 1.0e-9:
        vector *= limit / norm
    return vector
