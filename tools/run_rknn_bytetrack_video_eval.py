#!/usr/bin/env python3
"""Evaluate the production RKNN YOLO + ByteTrack stack on a video file."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision_guidance.bytetrack_adapter import ByteTrackConfig  # noqa: E402
from vision_guidance.rknn_bytetrack_detector import RknnByteTrackDetector  # noqa: E402
from vision_guidance.rknn_native_detector import RknnDetectorConfig  # noqa: E402


CSV_FIELDS = [
    "frame_id",
    "exposure_ts",
    "detection_valid",
    "x1",
    "y1",
    "x2",
    "y2",
    "track_id",
    "score",
    "detector_raw_count",
    "detector_class_filtered_count",
    "detector_best_score",
    "detector_reject_reason",
    "tracker_state",
    "tracker_confirmed",
    "tracker_hits",
    "tracker_match_iou",
    "tracker_association_stage",
    "tracker_switch_count",
    "tracker_fragment_count",
    "target_selector_reason",
    "rknn_preprocess_ms",
    "rknn_inference_ms",
    "rknn_postprocess_ms",
    "rknn_total_ms",
    "truth_center_x",
    "truth_center_y",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--replay-csv", type=Path, required=True)
    parser.add_argument("--summary-json", type=Path, required=True)
    parser.add_argument("--ground-truth-csv", type=Path, default=None)
    parser.add_argument("--max-frames", type=int, default=0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise RuntimeError(f"failed to open video: {args.video}")
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if fps <= 0.0:
        raise RuntimeError("video FPS is unavailable")

    detector_cfg = dict(config.get("rknn_detector", {}))
    tracker_cfg = dict(config.get("rknn_bytetrack", {}))
    detector_values = dict(detector_cfg)
    detector_values["conf_threshold"] = float(tracker_cfg.get("detector_conf_threshold", 0.05))
    detector_values["iou_threshold"] = float(tracker_cfg.get("detector_iou_threshold", 0.70))
    detector = RknnByteTrackDetector(
        library_path=_resolve_repo_path(detector_cfg.get("library", "")),
        model_path=_resolve_repo_path(detector_cfg.get("model", "")),
        rknn_config=RknnDetectorConfig.from_mapping(detector_values),
        tracker_config=ByteTrackConfig.from_mapping(tracker_cfg),
    )
    truth = _load_truth(args.ground_truth_csv)
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    args.replay_csv.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, Any]] = []
    replay_rows: list[dict[str, Any]] = []
    frame_id = 0
    try:
        while True:
            ok, image_bgr = capture.read()
            if not ok or image_bgr is None:
                break
            frame_id += 1
            if args.max_frames > 0 and frame_id > args.max_frames:
                break
            exposure_ts = (frame_id - 1) / fps
            image_rgb = np.ascontiguousarray(image_bgr[:, :, ::-1])
            detection, stats = detector.detect(image_rgb, frame_id=frame_id, exposure_ts=exposure_ts)
            row = _row(frame_id, exposure_ts, detection, stats, truth.get(frame_id))
            rows.append(row)
            if detection is not None:
                replay_rows.append(
                    {
                        "frame_id": frame_id,
                        "exposure_ts": f"{exposure_ts:.6f}",
                        "x1": f"{detection.bbox_xyxy[0]:.6f}",
                        "y1": f"{detection.bbox_xyxy[1]:.6f}",
                        "x2": f"{detection.bbox_xyxy[2]:.6f}",
                        "y2": f"{detection.bbox_xyxy[3]:.6f}",
                        "track_id": detection.track_id,
                        "score": f"{detection.score:.6f}",
                    }
                )
    finally:
        capture.release()
        metadata = detector.metadata()
        detector.close()

    _write_csv(args.output_csv, CSV_FIELDS, rows)
    _write_csv(
        args.replay_csv,
        ["frame_id", "exposure_ts", "x1", "y1", "x2", "y2", "track_id", "score"],
        replay_rows,
    )
    summary = _summary(rows, fps=fps, metadata=metadata, video=args.video)
    args.summary_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"frames={summary['frames']}")
    print(f"candidate_rate={summary['candidate_rate']:.6f}")
    print(f"selected_rate={summary['selected_rate']:.6f}")
    print(f"track_ids={summary['track_ids']}")
    print(f"switches={summary['tracker_switch_count']} fragments={summary['tracker_fragment_count']}")
    print(f"center_x_correlation={summary['truth_center_x_correlation']}")
    print(f"csv={args.output_csv}")
    print(f"replay_csv={args.replay_csv}")
    print(f"summary={args.summary_json}")


def _resolve_repo_path(value: Any) -> Path:
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else ROOT / path


def _load_truth(path: Path | None) -> dict[int, dict[str, str]]:
    if path is None:
        return {}
    with path.open(newline="", encoding="utf-8") as stream:
        return {int(row["frame_id"]): row for row in csv.DictReader(stream)}


def _row(frame_id: int, exposure_ts: float, detection: Any, stats: dict[str, Any], truth: Any) -> dict[str, Any]:
    bbox = ("", "", "", "") if detection is None else detection.bbox_xyxy
    return {
        "frame_id": frame_id,
        "exposure_ts": f"{exposure_ts:.6f}",
        "detection_valid": int(detection is not None),
        "x1": bbox[0],
        "y1": bbox[1],
        "x2": bbox[2],
        "y2": bbox[3],
        "track_id": "" if detection is None else detection.track_id,
        "score": "" if detection is None else detection.score,
        **{name: stats.get(name, "") for name in CSV_FIELDS if name in stats},
        "truth_center_x": "" if truth is None else truth.get("center_x", ""),
        "truth_center_y": "" if truth is None else truth.get("center_y", ""),
    }


def _write_csv(path: Path, fields: list[str], rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _summary(rows: list[dict[str, Any]], *, fps: float, metadata: dict[str, Any], video: Path) -> dict[str, Any]:
    if not rows:
        raise RuntimeError("video produced no frames")
    selected = [row for row in rows if int(row["detection_valid"])]
    candidates = [row for row in rows if int(row.get("detector_class_filtered_count") or 0) > 0]
    ids = sorted({int(row["track_id"]) for row in selected})
    scores = [float(row["score"]) for row in selected]
    inference_ms = [float(row["rknn_inference_ms"]) for row in rows if row.get("rknn_inference_ms") != ""]
    matched = [
        row
        for row in selected
        if row.get("truth_center_x") not in (None, "") and row.get("x1") not in (None, "")
    ]
    detected_x = [0.5 * (float(row["x1"]) + float(row["x2"])) for row in matched]
    truth_x = [float(row["truth_center_x"]) for row in matched]
    correlation = None
    if len(matched) >= 2 and statistics.pstdev(detected_x) > 0.0 and statistics.pstdev(truth_x) > 0.0:
        correlation = float(np.corrcoef(detected_x, truth_x)[0, 1])
    last = rows[-1]
    return {
        "schema_version": 1,
        "video": str(video.resolve()),
        "frames": len(rows),
        "fps": fps,
        "duration_s": len(rows) / fps,
        "candidate_frames": len(candidates),
        "candidate_rate": len(candidates) / len(rows),
        "selected_frames": len(selected),
        "selected_rate": len(selected) / len(rows),
        "track_ids": ids,
        "tracker_switch_count": int(last.get("tracker_switch_count") or 0),
        "tracker_fragment_count": int(last.get("tracker_fragment_count") or 0),
        "score_mean": None if not scores else statistics.fmean(scores),
        "score_min": None if not scores else min(scores),
        "inference_ms_mean": None if not inference_ms else statistics.fmean(inference_ms),
        "inference_ms_max": None if not inference_ms else max(inference_ms),
        "truth_center_x_matched_frames": len(matched),
        "truth_center_x_correlation": correlation,
        "detector_metadata": metadata,
    }


if __name__ == "__main__":
    main()
