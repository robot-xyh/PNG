"""Pure helpers for screen-to-camera latency measurements."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class BrightnessSample:
    frame_id: int
    capture_monotonic_s: float
    capture_unix_s: float
    roi_mean: float


@dataclass(frozen=True)
class DisplayTransition:
    sequence: int
    state: int
    display_monotonic_s: float
    display_unix_s: float


@dataclass(frozen=True)
class CameraEdge:
    state: int
    frame_id: int
    capture_monotonic_s: float
    capture_unix_s: float


@dataclass(frozen=True)
class LatencyMatch:
    display_sequence: int
    state: int
    display_unix_s: float
    camera_frame_id: int
    camera_unix_s: float
    latency_ms: float


def central_roi(width: int, height: int, fraction: float) -> tuple[int, int, int, int]:
    """Return a centered x/y/width/height ROI."""
    if width <= 0 or height <= 0:
        raise ValueError("image width and height must be positive")
    if not math.isfinite(fraction) or not 0.0 < fraction <= 1.0:
        raise ValueError("ROI fraction must be finite and within (0, 1]")
    roi_width = max(1, int(round(width * fraction)))
    roi_height = max(1, int(round(height * fraction)))
    return (
        (width - roi_width) // 2,
        (height - roi_height) // 2,
        roi_width,
        roi_height,
    )


def validate_roi(roi: Sequence[int], *, width: int, height: int) -> tuple[int, int, int, int]:
    if len(roi) != 4:
        raise ValueError("ROI must contain x,y,width,height")
    x, y, roi_width, roi_height = (int(value) for value in roi)
    if x < 0 or y < 0 or roi_width <= 0 or roi_height <= 0:
        raise ValueError("ROI coordinates must be non-negative with positive dimensions")
    if x + roi_width > width or y + roi_height > height:
        raise ValueError("ROI exceeds image bounds")
    return x, y, roi_width, roi_height


def roi_mean_luma(image_bgr: np.ndarray, roi: Sequence[int]) -> float:
    if image_bgr.ndim != 3 or image_bgr.shape[2] < 3:
        raise ValueError("camera image must have at least three BGR channels")
    x, y, width, height = validate_roi(
        roi,
        width=int(image_bgr.shape[1]),
        height=int(image_bgr.shape[0]),
    )
    pixels = image_bgr[y : y + height, x : x + width, :3].astype(np.float64)
    luma = 0.114 * pixels[:, :, 0] + 0.587 * pixels[:, :, 1] + 0.299 * pixels[:, :, 2]
    return float(np.mean(luma))


def detect_brightness_edges(
    samples: Sequence[BrightnessSample],
    *,
    debounce_frames: int = 2,
    low_percentile: float = 10.0,
    high_percentile: float = 90.0,
    minimum_contrast: float = 20.0,
) -> tuple[list[CameraEdge], dict[str, float]]:
    """Detect debounced binary edges and timestamp the first frame in each new state."""
    if debounce_frames <= 0:
        raise ValueError("debounce_frames must be positive")
    if not 0.0 <= low_percentile < high_percentile <= 100.0:
        raise ValueError("brightness percentiles must be ordered within [0, 100]")
    if not math.isfinite(minimum_contrast) or minimum_contrast <= 0.0:
        raise ValueError("minimum_contrast must be positive and finite")
    if len(samples) < max(4, 2 * debounce_frames):
        raise ValueError("not enough camera samples for edge detection")
    _require_monotonic(
        (sample.capture_monotonic_s for sample in samples),
        "camera monotonic time",
    )

    brightness = np.asarray([sample.roi_mean for sample in samples], dtype=float)
    if not np.all(np.isfinite(brightness)):
        raise ValueError("camera brightness samples must be finite")
    low_level, high_level = np.percentile(brightness, [low_percentile, high_percentile])
    contrast = float(high_level - low_level)
    if contrast < minimum_contrast:
        raise ValueError(
            f"screen contrast is too low: {contrast:.3f} < {minimum_contrast:.3f}"
        )
    low_threshold = float(low_level + 0.40 * contrast)
    high_threshold = float(low_level + 0.60 * contrast)
    midpoint = 0.5 * (low_threshold + high_threshold)

    current_state = 1 if brightness[0] >= midpoint else 0
    candidate_state: int | None = None
    candidate_count = 0
    candidate_sample: BrightnessSample | None = None
    edges: list[CameraEdge] = []
    for sample in samples[1:]:
        observed_state: int | None
        if sample.roi_mean >= high_threshold:
            observed_state = 1
        elif sample.roi_mean <= low_threshold:
            observed_state = 0
        else:
            observed_state = None

        if observed_state is None or observed_state == current_state:
            candidate_state = None
            candidate_count = 0
            candidate_sample = None
            continue
        if observed_state != candidate_state:
            candidate_state = observed_state
            candidate_count = 1
            candidate_sample = sample
        else:
            candidate_count += 1
        if candidate_count < debounce_frames:
            continue

        assert candidate_sample is not None
        edges.append(
            CameraEdge(
                state=int(candidate_state),
                frame_id=candidate_sample.frame_id,
                capture_monotonic_s=candidate_sample.capture_monotonic_s,
                capture_unix_s=candidate_sample.capture_unix_s,
            )
        )
        current_state = int(candidate_state)
        candidate_state = None
        candidate_count = 0
        candidate_sample = None

    return edges, {
        "low_level": float(low_level),
        "high_level": float(high_level),
        "contrast": contrast,
        "low_threshold": low_threshold,
        "high_threshold": high_threshold,
    }


def match_display_to_camera_edges(
    transitions: Sequence[DisplayTransition],
    edges: Sequence[CameraEdge],
    *,
    camera_minus_display_clock_s: float = 0.0,
    minimum_latency_s: float = -0.05,
    maximum_latency_s: float = 0.35,
) -> tuple[list[LatencyMatch], list[int]]:
    """Greedily match ordered same-state edges inside a bounded latency window."""
    if maximum_latency_s <= minimum_latency_s:
        raise ValueError("maximum latency must exceed minimum latency")
    if not all(
        math.isfinite(value)
        for value in (camera_minus_display_clock_s, minimum_latency_s, maximum_latency_s)
    ):
        raise ValueError("clock offset and latency bounds must be finite")
    _require_monotonic(
        (transition.display_unix_s for transition in transitions),
        "display Unix time",
    )
    _require_monotonic((edge.capture_unix_s for edge in edges), "camera Unix time")

    matches: list[LatencyMatch] = []
    unmatched: list[int] = []
    edge_index = 0
    for transition in transitions:
        earliest = transition.display_unix_s + minimum_latency_s
        latest = transition.display_unix_s + maximum_latency_s
        selected_index: int | None = None
        while edge_index < len(edges):
            corrected_edge_time = edges[edge_index].capture_unix_s - camera_minus_display_clock_s
            if corrected_edge_time >= earliest:
                break
            edge_index += 1
        for candidate_index in range(edge_index, len(edges)):
            edge = edges[candidate_index]
            corrected_edge_time = edge.capture_unix_s - camera_minus_display_clock_s
            if corrected_edge_time > latest:
                break
            if edge.state == transition.state:
                selected_index = candidate_index
                break
        if selected_index is None:
            unmatched.append(transition.sequence)
            continue
        edge = edges[selected_index]
        corrected_camera_time = edge.capture_unix_s - camera_minus_display_clock_s
        matches.append(
            LatencyMatch(
                display_sequence=transition.sequence,
                state=transition.state,
                display_unix_s=transition.display_unix_s,
                camera_frame_id=edge.frame_id,
                camera_unix_s=edge.capture_unix_s,
                latency_ms=1000.0 * (corrected_camera_time - transition.display_unix_s),
            )
        )
        edge_index = selected_index + 1
    return matches, unmatched


def summarize_latency(
    matches: Sequence[LatencyMatch],
    *,
    camera_samples: Sequence[BrightnessSample],
    transition_count: int,
) -> dict[str, Any]:
    if not matches:
        raise ValueError("at least one latency match is required")
    if transition_count <= 0:
        raise ValueError("transition_count must be positive")
    if len(matches) > transition_count:
        raise ValueError("matched transitions cannot exceed transition_count")
    _require_monotonic(
        (sample.capture_monotonic_s for sample in camera_samples),
        "camera monotonic time",
    )
    frame_periods_ms = 1000.0 * np.diff(
        np.asarray([sample.capture_monotonic_s for sample in camera_samples], dtype=float)
    )
    frame_periods_ms = frame_periods_ms[np.isfinite(frame_periods_ms) & (frame_periods_ms > 0.0)]
    if not len(frame_periods_ms):
        raise ValueError("camera samples do not contain a positive frame period")
    median_frame_period_ms = float(np.median(frame_periods_ms))
    all_latencies = np.asarray([match.latency_ms for match in matches], dtype=float)

    by_state: dict[str, Any] = {}
    for state in (0, 1):
        selected = np.asarray(
            [match.latency_ms for match in matches if match.state == state],
            dtype=float,
        )
        if len(selected):
            by_state[str(state)] = _distribution(selected)

    return {
        "transition_count": int(transition_count),
        "matched_count": len(matches),
        "matched_ratio": len(matches) / transition_count,
        "latency_ms": _distribution(all_latencies),
        "latency_ms_by_state": by_state,
        "camera_frame_period_ms": _distribution(frame_periods_ms),
        "median_camera_fps": 1000.0 / median_frame_period_ms,
        "equivalent_buffer_frames": _distribution(all_latencies / median_frame_period_ms),
    }


def dataclass_rows(values: Iterable[Any]) -> list[dict[str, Any]]:
    return [asdict(value) for value in values]


def ntp_clock_offset_and_delay_s(
    client_send_unix_s: float,
    server_receive_unix_s: float,
    server_send_unix_s: float,
    client_receive_unix_s: float,
) -> tuple[float, float]:
    """Return server-minus-client clock offset and network delay."""
    values = (
        client_send_unix_s,
        server_receive_unix_s,
        server_send_unix_s,
        client_receive_unix_s,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("clock probe timestamps must be finite")
    if client_receive_unix_s < client_send_unix_s:
        raise ValueError("client receive time precedes client send time")
    if server_send_unix_s < server_receive_unix_s:
        raise ValueError("server send time precedes server receive time")
    offset_s = 0.5 * (
        (server_receive_unix_s - client_send_unix_s)
        + (server_send_unix_s - client_receive_unix_s)
    )
    delay_s = (client_receive_unix_s - client_send_unix_s) - (
        server_send_unix_s - server_receive_unix_s
    )
    if delay_s < -1e-9:
        raise ValueError("clock probe computed a negative network delay")
    return float(offset_s), float(max(0.0, delay_s))


def _distribution(values: np.ndarray) -> dict[str, float]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if not len(finite):
        raise ValueError("distribution is empty")
    p50, p95, p99, maximum = np.percentile(finite, [50.0, 95.0, 99.0, 100.0])
    median = float(np.median(finite))
    mad = float(np.median(np.abs(finite - median)))
    return {
        "count": int(len(finite)),
        "p50": float(p50),
        "p95": float(p95),
        "p99": float(p99),
        "max": float(maximum),
        "min": float(np.min(finite)),
        "mean": float(np.mean(finite)),
        "robust_sigma_mad": 1.4826 * mad,
        "p95_minus_p50": float(p95 - p50),
    }


def _require_monotonic(values: Iterable[float], label: str) -> None:
    previous: float | None = None
    for value in values:
        if not math.isfinite(value):
            raise ValueError(f"{label} must be finite")
        if previous is not None and value <= previous:
            raise ValueError(f"{label} must be strictly increasing")
        previous = value
