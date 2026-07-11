from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import asdict, dataclass
from types import SimpleNamespace
from typing import Any, Iterable

import numpy as np

from ._vendor.ultralytics_tracker import BYTETracker
from ._vendor.ultralytics_tracker.basetrack import TrackState
from .rknn_native_detector import RknnDetection


@dataclass(frozen=True)
class ByteTrackConfig:
    track_high_thresh: float = 0.25
    track_low_thresh: float = 0.10
    new_track_thresh: float = 0.25
    match_thresh: float = 0.80
    fuse_score: bool = True
    track_buffer: int = 30
    track_buffer_s: float = 0.50
    frame_rate: float = 5.0
    minimum_confirmed_frames: int = 3
    final_min_score: float = 0.25
    final_min_bbox_area: float = 0.0
    final_max_bbox_aspect_ratio: float = 3.0

    @classmethod
    def from_mapping(cls, values: dict[str, Any]) -> "ByteTrackConfig":
        return cls(
            track_high_thresh=float(values.get("track_high_thresh", 0.25)),
            track_low_thresh=float(values.get("track_low_thresh", 0.10)),
            new_track_thresh=float(values.get("new_track_thresh", 0.25)),
            match_thresh=float(values.get("match_thresh", 0.80)),
            fuse_score=bool(values.get("fuse_score", True)),
            track_buffer=int(values.get("track_buffer", 30)),
            track_buffer_s=float(values.get("track_buffer_s", 0.50)),
            frame_rate=float(values.get("frame_rate", 5.0)),
            minimum_confirmed_frames=int(values.get("minimum_confirmed_frames", 3)),
            final_min_score=float(values.get("final_min_score", 0.25)),
            final_min_bbox_area=float(values.get("final_min_bbox_area", 0.0)),
            final_max_bbox_aspect_ratio=float(values.get("final_max_bbox_aspect_ratio", 3.0)),
        )

    @property
    def effective_track_buffer(self) -> int:
        if self.track_buffer_s > 0.0:
            return max(1, int(math.ceil(self.track_buffer_s * self.frame_rate)))
        return max(1, self.track_buffer)

    def validate(self) -> None:
        if not 0.0 <= self.track_low_thresh < self.track_high_thresh <= 1.0:
            raise ValueError("ByteTrack thresholds must satisfy 0 <= low < high <= 1")
        if not self.track_high_thresh <= self.new_track_thresh <= 1.0:
            raise ValueError("new_track_thresh must be at least track_high_thresh")
        if not 0.0 < self.match_thresh <= 1.0:
            raise ValueError("match_thresh must be in (0, 1]")
        if self.frame_rate <= 0.0 or self.minimum_confirmed_frames <= 0:
            raise ValueError("frame_rate and minimum_confirmed_frames must be positive")
        if self.final_min_bbox_area < 0.0 or self.final_max_bbox_aspect_ratio < 1.0:
            raise ValueError("final bbox filter parameters are invalid")


@dataclass(frozen=True)
class TrackedDetection:
    bbox_xyxy: tuple[float, float, float, float]
    track_id: int
    score: float
    class_id: int
    age_frames: int
    hits: int
    confirmed: bool


@dataclass(frozen=True)
class ByteTrackUpdate:
    selected: TrackedDetection | None
    stats: dict[str, Any]


class _DetectionArray:
    def __init__(self, xyxy: np.ndarray, conf: np.ndarray, cls: np.ndarray):
        self.xyxy = np.asarray(xyxy, dtype=np.float32).reshape(-1, 4)
        self.conf = np.asarray(conf, dtype=np.float32).reshape(-1)
        self.cls = np.asarray(cls, dtype=np.float32).reshape(-1)

    def __len__(self) -> int:
        return len(self.conf)

    def __getitem__(self, index) -> "_DetectionArray":
        return _DetectionArray(self.xyxy[index], self.conf[index], self.cls[index])

    @property
    def xywh(self) -> np.ndarray:
        result = self.xyxy.copy()
        result[:, 2:] -= result[:, :2]
        result[:, :2] += result[:, 2:] / 2.0
        return result


class ByteTrackAdapter:
    implementation = "ultralytics-8.4.71-vendored"

    def __init__(self, config: ByteTrackConfig, *, tracker: Any = None):
        config.validate()
        self.config = config
        args = SimpleNamespace(
            track_high_thresh=config.track_high_thresh,
            track_low_thresh=config.track_low_thresh,
            new_track_thresh=config.new_track_thresh,
            match_thresh=config.match_thresh,
            fuse_score=config.fuse_score,
            track_buffer=config.effective_track_buffer,
        )
        self.tracker = tracker if tracker is not None else BYTETracker(args)
        self.active_track_id: int | None = None
        self._hits: dict[int, int] = {}
        self._lost_seen: set[int] = set()
        self._released_track_id: int | None = None
        self._switch_count = 0
        self._fragment_count = 0
        self._last_timestamp: float | None = None
        self._fps_ewma: float | None = None

    def update(self, detections: Iterable[RknnDetection], *, timestamp: float) -> ByteTrackUpdate:
        start = time.monotonic()
        detection_list = tuple(detections)
        results = _detection_array(detection_list)
        tracked_rows = np.asarray(self.tracker.update(results), dtype=np.float32)
        if tracked_rows.size == 0:
            tracked_rows = np.empty((0, 8), dtype=np.float32)
        else:
            tracked_rows = tracked_rows.reshape(-1, 8)
        current = self._current_tracks(tracked_rows)
        current_ids = set(current)
        for track_id, candidate in current.items():
            self._hits[track_id] = candidate.hits

        lost_ids = {int(track.track_id) for track in self.tracker.lost_stracks}
        removed_ids = {int(track.track_id) for track in self.tracker.removed_stracks}
        if self.active_track_id in lost_ids:
            self._lost_seen.add(int(self.active_track_id))
        if self.active_track_id in current_ids and self.active_track_id in self._lost_seen:
            self._fragment_count += 1
            self._lost_seen.discard(int(self.active_track_id))
        if self.active_track_id in removed_ids:
            self._released_track_id = self.active_track_id
            self.active_track_id = None

        selected: TrackedDetection | None = None
        selector_reason = "track_unconfirmed"
        if self.active_track_id is not None:
            candidate = current.get(self.active_track_id)
            if candidate is not None:
                selected, selector_reason = self._eligible(candidate)
            elif self.active_track_id in lost_ids:
                selector_reason = "active_track_lost"
            else:
                selector_reason = "active_track_missing"
        else:
            eligible = []
            for candidate in current.values():
                accepted, _reason = self._eligible(candidate)
                if accepted is not None:
                    eligible.append(accepted)
            if eligible:
                selected = max(
                    eligible,
                    key=lambda item: (
                        (item.bbox_xyxy[2] - item.bbox_xyxy[0])
                        * (item.bbox_xyxy[3] - item.bbox_xyxy[1]),
                        item.score,
                    ),
                )
                self.active_track_id = selected.track_id
                if self._released_track_id is not None and self._released_track_id != selected.track_id:
                    self._switch_count += 1
                self._released_track_id = None
                selector_reason = "new_track_locked"
            elif not detection_list:
                selector_reason = "no_detection_candidates"
            elif not current:
                selector_reason = "no_tracked_output"

        self._update_fps(timestamp)
        elapsed_ms = 1000.0 * (time.monotonic() - start)
        high_count = sum(item.score >= self.config.track_high_thresh for item in detection_list)
        low_count = sum(
            self.config.track_low_thresh < item.score < self.config.track_high_thresh
            for item in detection_list
        )
        selected_track = self._find_track(self.active_track_id)
        measured_candidate = current.get(self.active_track_id)
        match_iou = "" if measured_candidate is None else _best_iou(measured_candidate.bbox_xyxy, detection_list)
        association_stage = ""
        if measured_candidate is not None:
            association_stage = "high" if measured_candidate.score >= self.config.track_high_thresh else "low"
        stats = {
            "tracker_state": _track_state(selected_track, lost_ids, removed_ids),
            "tracker_track_id": "" if self.active_track_id is None else self.active_track_id,
            "tracker_age_frames": "" if measured_candidate is None else measured_candidate.age_frames,
            "tracker_hits": "" if self.active_track_id is None else self._hits.get(self.active_track_id, 0),
            "tracker_lost_frames": _lost_frames(selected_track, self.tracker.frame_id),
            "tracker_confirmed": int(bool(measured_candidate is not None and measured_candidate.confirmed)),
            "tracker_high_count": high_count,
            "tracker_low_count": low_count,
            "tracker_output_count": len(current),
            "tracker_match_count": len(current),
            "tracker_match_iou": match_iou,
            "tracker_association_stage": association_stage,
            "tracker_switch_count": self._switch_count,
            "tracker_fragment_count": self._fragment_count,
            "tracker_update_ms": elapsed_ms,
            "tracker_actual_fps": "" if self._fps_ewma is None else self._fps_ewma,
            "target_selector_reason": selector_reason,
            "bbox_measurement_source": "detector_update" if selected is not None else "none",
        }
        return ByteTrackUpdate(selected=selected, stats=stats)

    def metadata(self) -> dict[str, Any]:
        config_values = asdict(self.config)
        config_json = json.dumps(config_values, sort_keys=True, separators=(",", ":"))
        return {
            "implementation": self.implementation,
            "config": config_values,
            "config_sha256": hashlib.sha256(config_json.encode("utf-8")).hexdigest(),
            "effective_track_buffer": self.config.effective_track_buffer,
        }

    def _current_tracks(self, rows: np.ndarray) -> dict[int, TrackedDetection]:
        tracks: dict[int, TrackedDetection] = {}
        for row in rows:
            track_id = int(row[4])
            source = self._find_track(track_id)
            age = 0 if source is None else int(source.frame_id - source.start_frame + 1)
            source_hits = 0 if source is None else int(source.tracklet_len + 1)
            hits = max(self._hits.get(track_id, 0) + 1, source_hits)
            tracks[track_id] = TrackedDetection(
                bbox_xyxy=tuple(float(value) for value in row[:4]),
                track_id=track_id,
                score=float(row[5]),
                class_id=int(row[6]),
                age_frames=age,
                hits=hits,
                confirmed=hits >= self.config.minimum_confirmed_frames,
            )
        return tracks

    def _eligible(self, candidate: TrackedDetection) -> tuple[TrackedDetection | None, str]:
        if not candidate.confirmed:
            return None, "track_unconfirmed"
        if candidate.score < self.config.final_min_score:
            return None, "track_score_below_final"
        x1, y1, x2, y2 = candidate.bbox_xyxy
        width = max(0.0, x2 - x1)
        height = max(0.0, y2 - y1)
        if width * height < self.config.final_min_bbox_area:
            return None, "track_area_below_final"
        aspect = max(width, height) / max(1.0, min(width, height))
        if aspect > self.config.final_max_bbox_aspect_ratio:
            return None, "track_aspect_above_final"
        return candidate, "active_track_measured"

    def _find_track(self, track_id: int | None):
        if track_id is None:
            return None
        for pool in (self.tracker.tracked_stracks, self.tracker.lost_stracks, self.tracker.removed_stracks):
            for track in pool:
                if int(track.track_id) == int(track_id):
                    return track
        return None

    def _update_fps(self, timestamp: float) -> None:
        if self._last_timestamp is not None:
            delta = float(timestamp) - self._last_timestamp
            if delta > 1.0e-6:
                instantaneous = 1.0 / delta
                self._fps_ewma = instantaneous if self._fps_ewma is None else 0.9 * self._fps_ewma + 0.1 * instantaneous
        self._last_timestamp = float(timestamp)


def _detection_array(detections: tuple[RknnDetection, ...]) -> _DetectionArray:
    if not detections:
        return _DetectionArray(np.empty((0, 4)), np.empty(0), np.empty(0))
    return _DetectionArray(
        np.asarray([item.bbox_xyxy for item in detections], dtype=np.float32),
        np.asarray([item.score for item in detections], dtype=np.float32),
        np.asarray([item.class_id for item in detections], dtype=np.float32),
    )


def _track_state(track, lost_ids: set[int], removed_ids: set[int]) -> str:
    if track is None:
        return "none"
    track_id = int(track.track_id)
    if track_id in removed_ids or track.state == TrackState.Removed:
        return "removed"
    if track_id in lost_ids or track.state == TrackState.Lost:
        return "lost"
    return "tracked" if track.is_activated else "unconfirmed"


def _lost_frames(track, frame_id: int) -> int | str:
    if track is None or track.state != TrackState.Lost:
        return 0 if track is not None else ""
    return max(0, int(frame_id - track.end_frame))


def _best_iou(bbox: tuple[float, float, float, float], detections: tuple[RknnDetection, ...]) -> float | str:
    if not detections:
        return ""
    x1, y1, x2, y2 = bbox
    area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    best = 0.0
    for detection in detections:
        dx1, dy1, dx2, dy2 = detection.bbox_xyxy
        intersection = max(0.0, min(x2, dx2) - max(x1, dx1)) * max(0.0, min(y2, dy2) - max(y1, dy1))
        other_area = max(0.0, dx2 - dx1) * max(0.0, dy2 - dy1)
        best = max(best, intersection / max(1.0e-7, area + other_area - intersection))
    return best
