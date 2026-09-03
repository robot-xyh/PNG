#!/usr/bin/env python3
"""Record production-camera ROI brightness for a screen latency probe."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time
from typing import Any

import cv2


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from examples.run_betaflight_log_only import OpenCvCameraSource  # noqa: E402
from vision_guidance.camera_latency_probe import central_roi, roi_mean_luma, validate_roi  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/betaflight.rk3588.kinematics_log_only.example.json"),
    )
    parser.add_argument("--camera-device", default="", help="Override camera.device from config.")
    parser.add_argument("--duration-s", type=float, default=40.0)
    parser.add_argument("--roi", default="", help="Output-image ROI as x,y,width,height.")
    parser.add_argument("--roi-fraction", type=float, default=0.35)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--progress-interval-s", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _validate_args(args)
    config_path = args.config.expanduser().resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(config.get("camera"), dict):
        raise ValueError(f"config does not contain a camera object: {config_path}")

    output_dir = args.output_dir or _default_output_dir("camera")
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "camera_brightness.csv"
    metadata_path = output_dir / "camera_metadata.json"
    preview_path = output_dir / "camera_roi_preview.jpg"

    anchor_start = _clock_anchor()
    source = OpenCvCameraSource(args, config)
    rows: list[dict[str, Any]] = []
    roi: tuple[int, int, int, int] | None = None
    preview_image = None
    preview_luma = -1.0
    interrupted = False
    start_s = time.monotonic()
    next_progress_s = start_s
    last_stats: dict[str, Any] = {}
    try:
        while time.monotonic() - start_s < args.duration_s:
            image = source.read_image()
            stats = dict(source.last_stats)
            last_stats = stats
            if image is None:
                time.sleep(0.005)
                continue
            height, width = image.shape[:2]
            if roi is None:
                roi = (
                    _parse_roi(args.roi, width=width, height=height)
                    if args.roi
                    else central_roi(width, height, args.roi_fraction)
                )
            luma = roi_mean_luma(image, roi)
            capture_monotonic_s = float(stats["camera_capture_ts"])
            rows.append(
                {
                    "frame_id": len(rows) + 1,
                    "capture_monotonic_s": capture_monotonic_s,
                    "capture_unix_s": capture_monotonic_s + anchor_start["unix_minus_monotonic_s"],
                    "elapsed_s": capture_monotonic_s - start_s,
                    "roi_mean": luma,
                    "camera_read_ms": stats.get("camera_read_ms", ""),
                    "image_width": width,
                    "image_height": height,
                    "input_width": stats.get("camera_input_width", ""),
                    "input_height": stats.get("camera_input_height", ""),
                    "camera_failed_frames": source.failed_frames,
                }
            )
            if luma > preview_luma:
                preview_image = image.copy()
                preview_luma = luma
            now_s = time.monotonic()
            if now_s >= next_progress_s:
                print(
                    f"elapsed_s={now_s - start_s:.1f} frames={len(rows)} "
                    f"failed={source.failed_frames} roi_mean={luma:.1f}",
                    flush=True,
                )
                next_progress_s = now_s + args.progress_interval_s
    except KeyboardInterrupt:
        interrupted = True
    finally:
        source.close()

    anchor_end = _clock_anchor()
    if not rows or roi is None or preview_image is None:
        raise RuntimeError("camera probe did not capture any valid frame")
    _write_csv(csv_path, rows)
    x, y, width, height = roi
    cv2.rectangle(preview_image, (x, y), (x + width - 1, y + height - 1), (0, 255, 0), 2)
    if not cv2.imwrite(str(preview_path), preview_image):
        raise RuntimeError(f"failed to write ROI preview: {preview_path}")

    config_bytes = config_path.read_bytes()
    metadata = {
        "schema_version": 1,
        "tool": str(Path(__file__).resolve()),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "csv": str(csv_path.resolve()),
        "preview": str(preview_path.resolve()),
        "config": str(config_path),
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "duration_requested_s": args.duration_s,
        "duration_recorded_s": rows[-1]["capture_monotonic_s"] - rows[0]["capture_monotonic_s"],
        "interrupted": interrupted,
        "valid_frame_count": len(rows),
        "failed_frame_count": source.failed_frames,
        "roi_xywh": list(roi),
        "clock_anchor_start": anchor_start,
        "clock_anchor_end": anchor_end,
        "unix_minus_monotonic_drift_ms": 1000.0
        * (anchor_end["unix_minus_monotonic_s"] - anchor_start["unix_minus_monotonic_s"]),
        "camera_last_stats": last_stats,
        "timestamp_semantics": (
            "capture_monotonic_s is OpenCvCameraSource camera_capture_ts recorded immediately "
            "after capture.read returns and before resize/undistortion"
        ),
    }
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"camera_csv={csv_path}")
    print(f"camera_metadata={metadata_path}")
    print(f"roi_preview={preview_path}")
    print(f"valid_frames={len(rows)} failed_frames={source.failed_frames}")


def _parse_roi(value: str, *, width: int, height: int) -> tuple[int, int, int, int]:
    try:
        parsed = tuple(int(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise ValueError("ROI must use integer x,y,width,height values") from exc
    return validate_roi(parsed, width=width, height=height)


def _clock_anchor() -> dict[str, float]:
    before_s = time.monotonic()
    unix_s = time.time()
    after_s = time.monotonic()
    monotonic_s = 0.5 * (before_s + after_s)
    return {
        "monotonic_s": monotonic_s,
        "unix_s": unix_s,
        "unix_minus_monotonic_s": unix_s - monotonic_s,
        "call_span_ms": 1000.0 * (after_s - before_s),
    }


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _default_output_dir(prefix: str) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path("logs") / "camera_latency" / f"{prefix}_{stamp}"


def _validate_args(args: argparse.Namespace) -> None:
    if args.duration_s <= 0.0:
        raise ValueError("duration must be positive")
    if not 0.0 < args.roi_fraction <= 1.0:
        raise ValueError("ROI fraction must be within (0, 1]")
    if args.progress_interval_s <= 0.0:
        raise ValueError("progress interval must be positive")


if __name__ == "__main__":
    main()
