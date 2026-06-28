#!/usr/bin/env python3
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_guidance.airsim_adapter import AirSimDetectionConfig
from vision_guidance.yolo_bytetrack_detector import YoloByteTrackDetector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark the project YOLO + ByteTrack detector without AirSim.")
    parser.add_argument("--model", default="vision_guidance/best.pt", help="YOLO .pt/.engine path.")
    parser.add_argument("--class-id", type=int, default=0)
    parser.add_argument("--image", default="", help="Optional image path. If omitted, a blank synthetic frame is used.")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--conf", type=float, default=0.1)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--tracker", default="bytetrack.yaml")
    parser.add_argument("--half", action=argparse.BooleanOptionalAction, default=False)
    return parser.parse_args()


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(fraction * (len(ordered) - 1)))))
    return float(ordered[index])


def _mean_stat(rows: list[dict[str, Any]], key: str) -> float:
    values = []
    for row in rows:
        try:
            value = float(row.get(key, ""))
        except (TypeError, ValueError):
            continue
        if np.isfinite(value):
            values.append(value)
    return float(statistics.fmean(values)) if values else float("nan")


def _load_image(args: argparse.Namespace):
    import cv2

    if args.image:
        path = Path(args.image).expanduser()
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise SystemExit(f"failed to read image: {path}")
        return image
    return np.zeros((max(1, args.height), max(1, args.width), 3), dtype=np.uint8)


def main() -> None:
    args = parse_args()
    image = _load_image(args)

    detector = YoloByteTrackDetector(
        model_path=str(Path(args.model).expanduser()),
        class_id=args.class_id,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        device=args.device,
        tracker=args.tracker,
        half=args.half,
        allow_untracked_fallback=True,
        single_target_mode=True,
        image_reader=lambda _client, _config: image,
    )
    config = AirSimDetectionConfig(camera_name="0", vehicle_name="Interceptor")
    rows: list[dict[str, Any]] = []
    elapsed: list[float] = []
    total = max(1, int(args.frames)) + max(0, int(args.warmup))
    for index in range(total):
        start = time.perf_counter()
        frame = detector.detect(client=object(), config=config, frame_id=index, exposure_ts=float(index) / 30.0)
        dt = time.perf_counter() - start
        if index >= args.warmup:
            elapsed.append(dt)
            rows.append(frame.stats)

    fps = [1.0 / max(1.0e-9, value) for value in elapsed]
    print(f"frames={len(elapsed)} warmup={args.warmup}")
    print(f"model={args.model} device_arg={args.device} runtime={rows[-1].get('yolo_runtime_device', '') if rows else ''}")
    print(f"imgsz={args.imgsz} half={int(args.half)} image_shape={image.shape[1]}x{image.shape[0]}")
    print(f"fps_mean={statistics.fmean(fps):.3f} fps_p50={_percentile(fps, 0.50):.3f} fps_p95={_percentile(fps, 0.95):.3f}")
    print(f"elapsed_mean_s={statistics.fmean(elapsed):.6f} elapsed_p95_s={_percentile(elapsed, 0.95):.6f}")
    print(f"capture_mean_s={_mean_stat(rows, 'image_capture_elapsed_s'):.6f}")
    print(f"inference_mean_s={_mean_stat(rows, 'yolo_inference_elapsed_s'):.6f}")
    print(f"postprocess_mean_s={_mean_stat(rows, 'yolo_postprocess_elapsed_s'):.6f}")


if __name__ == "__main__":
    main()
