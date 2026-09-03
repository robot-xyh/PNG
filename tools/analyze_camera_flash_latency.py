#!/usr/bin/env python3
"""Analyze screen transitions against production-camera ROI brightness."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision_guidance.camera_latency_probe import (  # noqa: E402
    BrightnessSample,
    DisplayTransition,
    dataclass_rows,
    detect_brightness_edges,
    match_display_to_camera_edges,
    summarize_latency,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--display-csv", type=Path, required=True)
    parser.add_argument("--camera-csv", type=Path, required=True)
    parser.add_argument("--display-metadata", type=Path, default=None)
    parser.add_argument("--camera-metadata", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--clock-probe-json", type=Path, default=None)
    parser.add_argument("--camera-minus-display-clock-ms", type=float, default=None)
    parser.add_argument(
        "--clock-uncertainty-ms",
        type=float,
        default=None,
        help="Bound on residual cross-device clock error; required for a passing quality gate.",
    )
    parser.add_argument("--max-clock-uncertainty-ms", type=float, default=5.0)
    parser.add_argument("--minimum-latency-ms", type=float, default=-20.0)
    parser.add_argument("--maximum-latency-ms", type=float, default=350.0)
    parser.add_argument("--debounce-frames", type=int, default=2)
    parser.add_argument("--minimum-contrast", type=float, default=20.0)
    parser.add_argument("--low-percentile", type=float, default=10.0)
    parser.add_argument("--high-percentile", type=float, default=90.0)
    parser.add_argument("--minimum-match-ratio", type=float, default=0.99)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _validate_args(args)
    display_csv = args.display_csv.expanduser().resolve()
    camera_csv = args.camera_csv.expanduser().resolve()
    display_meta_path = args.display_metadata or display_csv.with_name("display_metadata.json")
    camera_meta_path = args.camera_metadata or camera_csv.with_name("camera_metadata.json")
    display_meta = _read_json(display_meta_path)
    camera_meta = _read_json(camera_meta_path)
    clock_probe = _read_json(args.clock_probe_json) if args.clock_probe_json else {}
    clock_offset_ms = (
        args.camera_minus_display_clock_ms
        if args.camera_minus_display_clock_ms is not None
        else _float_or_none(clock_probe.get("camera_minus_display_clock_ms"))
    )
    clock_uncertainty_ms = (
        args.clock_uncertainty_ms
        if args.clock_uncertainty_ms is not None
        else _float_or_none(clock_probe.get("clock_uncertainty_ms"))
    )
    if clock_uncertainty_ms is not None and clock_uncertainty_ms < 0.0:
        raise ValueError("clock uncertainty from all sources must be non-negative")
    effective_clock_offset_ms = 0.0 if clock_offset_ms is None else clock_offset_ms
    clock_probe_offset_span_ms = _float_or_none(
        clock_probe.get("best_20_percent_offset_span_ms")
    )

    display_rows = _read_csv(display_csv)
    camera_rows = _read_csv(camera_csv)
    transitions = [
        DisplayTransition(
            sequence=int(row["sequence"]),
            state=int(row["state"]),
            display_monotonic_s=float(row["display_monotonic_s"]),
            display_unix_s=float(row["display_unix_s"]),
        )
        for row in display_rows
    ]
    samples = [
        BrightnessSample(
            frame_id=int(row["frame_id"]),
            capture_monotonic_s=float(row["capture_monotonic_s"]),
            capture_unix_s=float(row["capture_unix_s"]),
            roi_mean=float(row["roi_mean"]),
        )
        for row in camera_rows
    ]
    if not transitions:
        raise ValueError("display CSV contains no transitions")

    edges, edge_levels = detect_brightness_edges(
        samples,
        debounce_frames=args.debounce_frames,
        low_percentile=args.low_percentile,
        high_percentile=args.high_percentile,
        minimum_contrast=args.minimum_contrast,
    )
    matches, unmatched = match_display_to_camera_edges(
        transitions,
        edges,
        camera_minus_display_clock_s=effective_clock_offset_ms / 1000.0,
        minimum_latency_s=args.minimum_latency_ms / 1000.0,
        maximum_latency_s=args.maximum_latency_ms / 1000.0,
    )
    statistics = summarize_latency(
        matches,
        camera_samples=samples,
        transition_count=len(transitions),
    )

    failed_frames = max(int(float(row.get("camera_failed_frames") or 0)) for row in camera_rows)
    refresh_hz = _nested_float(display_meta, "window", "operator_declared_refresh_hz")
    refresh_period_ms = None if not refresh_hz or refresh_hz <= 0.0 else 1000.0 / refresh_hz
    event_pump_ms = np.asarray(
        [float(row["event_pump_ms"]) for row in display_rows if row.get("event_pump_ms")],
        dtype=float,
    )
    issues: list[str] = []
    if statistics["matched_ratio"] < args.minimum_match_ratio:
        issues.append(
            f"matched_ratio_below_{args.minimum_match_ratio:.3f}:"
            f"{statistics['matched_ratio']:.6f}"
        )
    if failed_frames:
        issues.append(f"camera_failed_frames:{failed_frames}")
    if clock_offset_ms is None:
        issues.append("clock_offset_not_supplied")
    if clock_uncertainty_ms is None:
        issues.append("clock_uncertainty_not_supplied")
    elif clock_uncertainty_ms > args.max_clock_uncertainty_ms:
        issues.append(
            f"clock_uncertainty_above_{args.max_clock_uncertainty_ms:.3f}_ms:"
            f"{clock_uncertainty_ms:.3f}"
        )
    if (
        clock_probe_offset_span_ms is not None
        and clock_probe_offset_span_ms > args.max_clock_uncertainty_ms
    ):
        issues.append(
            f"clock_probe_offset_span_above_{args.max_clock_uncertainty_ms:.3f}_ms:"
            f"{clock_probe_offset_span_ms:.3f}"
        )
    if not display_meta:
        issues.append("display_metadata_missing_or_invalid")
    if not camera_meta:
        issues.append("camera_metadata_missing_or_invalid")
    if bool(display_meta.get("aborted", False)):
        issues.append("display_run_aborted")
    if bool(camera_meta.get("interrupted", False)):
        issues.append("camera_run_interrupted")

    display_clock_drift_ms = _nested_float(
        display_meta,
        "clock",
        "unix_minus_monotonic_drift_ms",
    )
    camera_clock_drift_ms = _nested_float(camera_meta, "unix_minus_monotonic_drift_ms")
    if clock_uncertainty_ms is not None:
        for name, drift_ms in (
            ("display", display_clock_drift_ms),
            ("camera", camera_clock_drift_ms),
        ):
            if drift_ms is not None and abs(drift_ms) > clock_uncertainty_ms:
                issues.append(
                    f"{name}_clock_drift_exceeds_uncertainty:"
                    f"{drift_ms:.3f}_ms"
                )

    timing_floor_ms = None
    if refresh_period_ms is not None and clock_uncertainty_ms is not None:
        timing_floor_ms = (
            refresh_period_ms
            + statistics["camera_frame_period_ms"]["p50"]
            + clock_uncertainty_ms
        )
    summary: dict[str, Any] = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": {
            "display_csv": str(display_csv),
            "camera_csv": str(camera_csv),
            "display_metadata": str(display_meta_path.expanduser().resolve()),
            "camera_metadata": str(camera_meta_path.expanduser().resolve()),
            "clock_probe_json": (
                None
                if args.clock_probe_json is None
                else str(args.clock_probe_json.expanduser().resolve())
            ),
        },
        "edge_detection": {
            **edge_levels,
            "edge_count": len(edges),
            "debounce_frames": args.debounce_frames,
        },
        "matching": {
            "camera_minus_display_clock_ms": clock_offset_ms,
            "clock_uncertainty_ms": clock_uncertainty_ms,
            "clock_probe_best_offset_span_ms": clock_probe_offset_span_ms,
            "clock_source": (
                "manual_override"
                if (
                    args.camera_minus_display_clock_ms is not None
                    or args.clock_uncertainty_ms is not None
                )
                else "clock_probe_json" if clock_probe else "missing_assumed_zero"
            ),
            "latency_window_ms": [args.minimum_latency_ms, args.maximum_latency_ms],
            "unmatched_sequences": unmatched,
        },
        "statistics": statistics,
        "display": {
            "refresh_hz": refresh_hz,
            "refresh_period_ms": refresh_period_ms,
            "event_pump_ms": _distribution(event_pump_ms),
            "metadata_clock_drift_ms": display_clock_drift_ms,
        },
        "camera": {
            "failed_frames": failed_frames,
            "metadata_clock_drift_ms": camera_clock_drift_ms,
        },
        "quality": {
            "passed": not issues,
            "issues": issues,
            "minimum_match_ratio": args.minimum_match_ratio,
            "max_clock_uncertainty_ms": args.max_clock_uncertainty_ms,
            "nominal_timing_resolution_floor_ms": timing_floor_ms,
        },
        "scope": {
            "measured": (
                "screen update event-pump return to production OpenCV capture.read return, "
                "after applying the declared cross-device clock correction"
            ),
            "not_measured": [
                "exact physical display pixel transition time",
                "camera sensor photon exposure timestamp",
                "web MJPEG preview latency",
                "YOLO inference, ByteTrack, guidance, MSP, or motor response latency",
            ],
        },
    }

    output_dir = args.output_dir or camera_csv.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    edges_path = output_dir / "screen_camera_latency_edges.csv"
    matches_path = output_dir / "screen_camera_latency_matches.csv"
    summary_path = output_dir / "screen_camera_latency_summary.json"
    _write_rows(edges_path, dataclass_rows(edges))
    _write_rows(matches_path, dataclass_rows(matches))
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    latency = statistics["latency_ms"]
    print(f"quality_passed={int(not issues)} issues={','.join(issues) or 'none'}")
    print(
        f"matched={statistics['matched_count']}/{statistics['transition_count']} "
        f"contrast={edge_levels['contrast']:.2f} failed_frames={failed_frames}"
    )
    print(
        "latency_ms "
        f"p50={latency['p50']:.3f} p95={latency['p95']:.3f} "
        f"p99={latency['p99']:.3f} max={latency['max']:.3f}"
    )
    print(f"matches_csv={matches_path}")
    print(f"summary_json={summary_path}")


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.expanduser().read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _write_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _nested_float(value: dict[str, Any], *keys: str) -> float | None:
    current: Any = value
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    try:
        result = float(current)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _float_or_none(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _distribution(values: np.ndarray) -> dict[str, float] | None:
    finite = values[np.isfinite(values)]
    if not len(finite):
        return None
    return {
        "count": int(len(finite)),
        "p50": float(np.percentile(finite, 50.0)),
        "p95": float(np.percentile(finite, 95.0)),
        "p99": float(np.percentile(finite, 99.0)),
        "max": float(np.max(finite)),
    }


def _validate_args(args: argparse.Namespace) -> None:
    finite_values = (
        args.max_clock_uncertainty_ms,
        args.minimum_latency_ms,
        args.maximum_latency_ms,
        args.minimum_contrast,
        args.low_percentile,
        args.high_percentile,
        args.minimum_match_ratio,
    )
    if not all(math.isfinite(value) for value in finite_values):
        raise ValueError("numeric analysis arguments must be finite")
    if args.camera_minus_display_clock_ms is not None and not math.isfinite(
        args.camera_minus_display_clock_ms
    ):
        raise ValueError("clock offset must be finite")
    if args.clock_uncertainty_ms is not None and (
        not math.isfinite(args.clock_uncertainty_ms) or args.clock_uncertainty_ms < 0.0
    ):
        raise ValueError("clock uncertainty must be finite and non-negative")
    if args.max_clock_uncertainty_ms < 0.0:
        raise ValueError("maximum clock uncertainty must be non-negative")
    if args.maximum_latency_ms <= args.minimum_latency_ms:
        raise ValueError("maximum latency must exceed minimum latency")
    if args.debounce_frames <= 0:
        raise ValueError("debounce frames must be positive")
    if not 0.0 <= args.low_percentile < args.high_percentile <= 100.0:
        raise ValueError("brightness percentiles must be ordered within [0, 100]")
    if not 0.0 < args.minimum_match_ratio <= 1.0:
        raise ValueError("minimum match ratio must be within (0, 1]")


if __name__ == "__main__":
    main()
