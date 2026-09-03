#!/usr/bin/env python3
"""Generate deterministic virtual detections for VM load-chain validation."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=512)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--bbox-width-px", type=float, default=96.0)
    parser.add_argument("--bbox-height-px", type=float, default=72.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = generate_rows(
        width=args.width,
        height=args.height,
        fps=args.fps,
        duration_s=args.duration_s,
        bbox_width_px=args.bbox_width_px,
        bbox_height_px=args.bbox_height_px,
    )
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = (
        "frame_id",
        "exposure_ts",
        "x1",
        "y1",
        "x2",
        "y2",
        "track_id",
        "score",
        "motion_phase",
    )
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"output={output}")
    print(f"frames={len(rows)} fps={args.fps:.3f} duration_s={args.duration_s:.3f}")


def generate_rows(
    *,
    width: int = 640,
    height: int = 512,
    fps: float = 30.0,
    duration_s: float = 60.0,
    bbox_width_px: float = 96.0,
    bbox_height_px: float = 72.0,
) -> list[dict[str, Any]]:
    _validate_dimensions(
        width=width,
        height=height,
        fps=fps,
        duration_s=duration_s,
        bbox_width_px=bbox_width_px,
        bbox_height_px=bbox_height_px,
    )
    rows: list[dict[str, Any]] = []
    frame_count = int(round(duration_s * fps))
    for index in range(frame_count):
        timestamp_s = index / fps
        center_x, center_y, phase = motion_at(
            timestamp_s,
            width=width,
            height=height,
            duration_s=duration_s,
        )
        x1 = center_x - 0.5 * bbox_width_px
        y1 = center_y - 0.5 * bbox_height_px
        x2 = center_x + 0.5 * bbox_width_px
        y2 = center_y + 0.5 * bbox_height_px
        if x1 < 0.0 or y1 < 0.0 or x2 > width or y2 > height:
            raise ValueError(f"virtual bbox leaves the image at t={timestamp_s:.3f}s")
        rows.append(
            {
                "frame_id": index + 1,
                "exposure_ts": f"{timestamp_s:.6f}",
                "x1": f"{x1:.3f}",
                "y1": f"{y1:.3f}",
                "x2": f"{x2:.3f}",
                "y2": f"{y2:.3f}",
                "track_id": 1,
                "score": "0.990000",
                "motion_phase": phase,
            }
        )
    return rows


def motion_at(
    timestamp_s: float,
    *,
    width: int = 640,
    height: int = 512,
    duration_s: float = 60.0,
) -> tuple[float, float, str]:
    """Return a continuous center/static/crossing/stress trajectory."""
    if duration_s <= 0.0:
        raise ValueError("duration_s must be positive")
    scale = 60.0 / duration_s
    t = max(0.0, float(timestamp_s)) * scale
    center_x = 0.5 * width
    center_y = 0.5 * height
    amplitude_x = min(190.0, 0.30 * width)
    amplitude_y = min(150.0, 0.29 * height)

    if t < 8.0:
        return center_x, center_y, "center_start"
    if t < 20.0:
        phase = 2.0 * math.pi * (t - 8.0) / 4.0
        return center_x + amplitude_x * math.sin(phase), center_y, "horizontal_crossing"
    if t < 32.0:
        phase = 2.0 * math.pi * (t - 20.0) / 4.0
        return center_x, center_y + amplitude_y * math.sin(phase), "vertical_crossing"
    if t < 44.0:
        phase = 2.0 * math.pi * (t - 32.0) / 4.0
        return (
            center_x + amplitude_x * math.sin(phase),
            center_y + amplitude_y * math.sin(phase),
            "diagonal_crossing",
        )
    if t < 54.0:
        phase = 2.0 * math.pi * (t - 44.0) / 2.5
        return (
            center_x + amplitude_x * math.sin(phase),
            center_y + amplitude_y * math.sin(2.0 * phase),
            "high_rate_stress",
        )
    return center_x, center_y, "center_end"


def _validate_dimensions(
    *,
    width: int,
    height: int,
    fps: float,
    duration_s: float,
    bbox_width_px: float,
    bbox_height_px: float,
) -> None:
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    for name, value in (("fps", fps), ("duration_s", duration_s)):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if not 0.0 < bbox_width_px < width or not 0.0 < bbox_height_px < height:
        raise ValueError("bbox dimensions must be positive and smaller than the image")


if __name__ == "__main__":
    main()
