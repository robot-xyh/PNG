#!/usr/bin/env python3
"""Generate a deterministic hand-held UAV video for RKNN/ByteTrack testing."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ASSET_DIR = ROOT / "tools" / "assets" / "betaflight_synthetic_handheld"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--foreground",
        type=Path,
        default=DEFAULT_ASSET_DIR / "handheld_drone_bottom_rgba.png",
    )
    parser.add_argument(
        "--background",
        type=Path,
        default=DEFAULT_ASSET_DIR / "workshop_ceiling.png",
    )
    parser.add_argument("--output-video", type=Path, required=True)
    parser.add_argument("--ground-truth-csv", type=Path, required=True)
    parser.add_argument("--metadata-json", type=Path, default=None)
    parser.add_argument("--preview-jpg", type=Path, default=None)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=512)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--target-width-px", type=float, default=180.0)
    parser.add_argument("--horizontal-amplitude-px", type=float, default=190.0)
    parser.add_argument(
        "--foreground-opacity",
        type=float,
        default=1.0,
        help="Scale the foreground alpha channel within (0, 1] for contrast stress tests.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _validate_args(args)
    foreground = cv2.imread(str(args.foreground), cv2.IMREAD_UNCHANGED)
    background = cv2.imread(str(args.background), cv2.IMREAD_COLOR)
    if foreground is None or foreground.ndim != 3 or foreground.shape[2] != 4:
        raise RuntimeError(f"foreground must be a readable BGRA image: {args.foreground}")
    if background is None:
        raise RuntimeError(f"failed to read background image: {args.background}")

    foreground = _trim_transparent(foreground)
    background = _cover_resize(background, args.width, args.height)
    args.output_video.parent.mkdir(parents=True, exist_ok=True)
    args.ground_truth_csv.parent.mkdir(parents=True, exist_ok=True)

    writer = cv2.VideoWriter(
        str(args.output_video),
        cv2.VideoWriter_fourcc(*"MJPG"),
        float(args.fps),
        (int(args.width), int(args.height)),
    )
    if not writer.isOpened():
        raise RuntimeError(f"failed to open video writer: {args.output_video}")

    frame_count = int(round(args.duration_s * args.fps))
    preview_indices = {0, frame_count // 4, frame_count // 2, 3 * frame_count // 4, frame_count - 1}
    preview_frames: list[np.ndarray] = []
    with args.ground_truth_csv.open("w", newline="", encoding="utf-8") as stream:
        fields = [
            "frame_id",
            "exposure_ts",
            "x1",
            "y1",
            "x2",
            "y2",
            "center_x",
            "center_y",
            "target_width_px",
            "target_height_px",
            "motion_phase",
        ]
        csv_writer = csv.DictWriter(stream, fieldnames=fields)
        csv_writer.writeheader()
        try:
            for frame_index in range(frame_count):
                timestamp_s = frame_index / args.fps
                center_x, center_y, scale, phase = _motion(timestamp_s, args)
                frame, bbox = _compose(
                    background,
                    foreground,
                    center_x=center_x,
                    center_y=center_y,
                    width_px=args.target_width_px * scale,
                    foreground_opacity=args.foreground_opacity,
                )
                writer.write(frame)
                x1, y1, x2, y2 = bbox
                csv_writer.writerow(
                    {
                        "frame_id": frame_index + 1,
                        "exposure_ts": f"{timestamp_s:.6f}",
                        "x1": x1,
                        "y1": y1,
                        "x2": x2,
                        "y2": y2,
                        "center_x": f"{0.5 * (x1 + x2):.3f}",
                        "center_y": f"{0.5 * (y1 + y2):.3f}",
                        "target_width_px": x2 - x1,
                        "target_height_px": y2 - y1,
                        "motion_phase": phase,
                    }
                )
                if frame_index in preview_indices:
                    preview_frames.append(frame)
        finally:
            writer.release()

    preview_path = args.preview_jpg or args.output_video.with_name(f"{args.output_video.stem}_preview.jpg")
    preview_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(preview_path), _preview_strip(preview_frames))
    metadata_path = args.metadata_json or args.output_video.with_suffix(".json")
    metadata = {
        "schema_version": 1,
        "generator": str(Path(__file__).resolve()),
        "video": str(args.output_video.resolve()),
        "ground_truth_csv": str(args.ground_truth_csv.resolve()),
        "preview_jpg": str(preview_path.resolve()),
        "width": args.width,
        "height": args.height,
        "fps": args.fps,
        "duration_s": args.duration_s,
        "frame_count": frame_count,
        "target_width_px": args.target_width_px,
        "horizontal_amplitude_px": args.horizontal_amplitude_px,
        "foreground_opacity": args.foreground_opacity,
        "foreground": _asset_metadata(args.foreground),
        "background": _asset_metadata(args.background),
        "motion": {
            "static_start_s": 3.0,
            "horizontal_end_s": 23.0,
            "horizontal_period_s": 10.0,
            "vertical_bob_px": 6.0,
            "scale_variation": 0.08,
        },
    }
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"video={args.output_video}")
    print(f"ground_truth={args.ground_truth_csv}")
    print(f"metadata={metadata_path}")
    print(f"preview={preview_path}")
    print(f"frames={frame_count} fps={args.fps:.3f} duration_s={args.duration_s:.3f}")


def _validate_args(args: argparse.Namespace) -> None:
    if args.width <= 0 or args.height <= 0:
        raise ValueError("width and height must be positive")
    if args.fps <= 0.0 or args.duration_s <= 0.0:
        raise ValueError("fps and duration must be positive")
    if args.target_width_px <= 0.0 or args.target_width_px >= args.width:
        raise ValueError("target width must be positive and smaller than the frame")
    if not math.isfinite(args.foreground_opacity) or not 0.0 < args.foreground_opacity <= 1.0:
        raise ValueError("foreground opacity must be finite and within (0, 1]")
    maximum_amplitude = 0.5 * (args.width - args.target_width_px) - 4.0
    if args.horizontal_amplitude_px <= 0.0 or args.horizontal_amplitude_px > maximum_amplitude:
        raise ValueError(f"horizontal amplitude must be in (0, {maximum_amplitude:.1f}]")


def _trim_transparent(image: np.ndarray) -> np.ndarray:
    alpha = image[:, :, 3]
    points = cv2.findNonZero((alpha > 1).astype(np.uint8))
    if points is None:
        raise ValueError("foreground alpha channel is empty")
    x, y, width, height = cv2.boundingRect(points)
    return image[y : y + height, x : x + width]


def _cover_resize(image: np.ndarray, width: int, height: int) -> np.ndarray:
    scale = max(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(
        image,
        (int(math.ceil(image.shape[1] * scale)), int(math.ceil(image.shape[0] * scale))),
        interpolation=cv2.INTER_AREA,
    )
    x = (resized.shape[1] - width) // 2
    y = (resized.shape[0] - height) // 2
    return np.ascontiguousarray(resized[y : y + height, x : x + width])


def _motion(timestamp_s: float, args: argparse.Namespace) -> tuple[float, float, float, str]:
    center_x = 0.5 * args.width
    center_y = 0.48 * args.height
    if timestamp_s < 3.0:
        return center_x, center_y, 1.0, "static_start"
    if timestamp_s < 23.0:
        phase = 2.0 * math.pi * (timestamp_s - 3.0) / 10.0
        return (
            center_x + args.horizontal_amplitude_px * math.sin(phase),
            center_y + 6.0 * math.sin(2.0 * phase + 0.35),
            1.0 + 0.08 * math.sin(0.5 * phase),
            "handheld_horizontal",
        )
    return center_x, center_y, 1.0, "static_end"


def _compose(
    background: np.ndarray,
    foreground: np.ndarray,
    *,
    center_x: float,
    center_y: float,
    width_px: float,
    foreground_opacity: float = 1.0,
) -> tuple[np.ndarray, tuple[int, int, int, int]]:
    scale = width_px / foreground.shape[1]
    target_width = max(1, int(round(foreground.shape[1] * scale)))
    target_height = max(1, int(round(foreground.shape[0] * scale)))
    resized = cv2.resize(foreground, (target_width, target_height), interpolation=cv2.INTER_AREA)
    x1 = int(round(center_x - target_width / 2.0))
    y1 = int(round(center_y - target_height / 2.0))
    x2 = x1 + target_width
    y2 = y1 + target_height
    if x1 < 0 or y1 < 0 or x2 > background.shape[1] or y2 > background.shape[0]:
        raise ValueError(f"composited target would leave frame: {(x1, y1, x2, y2)}")
    frame = background.copy()
    alpha = resized[:, :, 3:4].astype(np.float32) / 255.0
    alpha *= float(foreground_opacity)
    foreground_bgr = resized[:, :, :3].astype(np.float32)
    region = frame[y1:y2, x1:x2].astype(np.float32)
    frame[y1:y2, x1:x2] = np.clip(foreground_bgr * alpha + region * (1.0 - alpha), 0, 255).astype(np.uint8)
    return frame, (x1, y1, x2, y2)


def _preview_strip(frames: list[np.ndarray]) -> np.ndarray:
    if not frames:
        raise ValueError("preview frame list is empty")
    thumbnail_width = 320
    thumbnails = [
        cv2.resize(frame, (thumbnail_width, int(round(frame.shape[0] * thumbnail_width / frame.shape[1]))))
        for frame in frames
    ]
    return cv2.hconcat(thumbnails)


def _asset_metadata(path: Path) -> dict[str, Any]:
    data = path.read_bytes()
    return {
        "path": str(path.resolve()),
        "sha256": hashlib.sha256(data).hexdigest(),
        "bytes": len(data),
    }


if __name__ == "__main__":
    main()
