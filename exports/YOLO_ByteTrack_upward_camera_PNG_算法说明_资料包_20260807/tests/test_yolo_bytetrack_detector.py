import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from vision_guidance.airsim_adapter import AirSimDetectionConfig
from vision_guidance.yolo_bytetrack_detector import YoloByteTrackDetector, YoloKcfDetector


class FakeBoxes:
    def __init__(self, xyxy, conf, cls, track_ids):
        self.xyxy = np.asarray(xyxy, dtype=float)
        self.conf = np.asarray(conf, dtype=float)
        self.cls = np.asarray(cls, dtype=float)
        self.id = None if track_ids is None else np.asarray(track_ids, dtype=float)


class FakeModel:
    def __init__(self, results):
        self.results = results
        self.calls = []

    def track(self, image, **kwargs):
        self.calls.append((image, kwargs))
        return self.results


class FakePredictModel(FakeModel):
    def predict(self, image, **kwargs):
        self.calls.append((image, kwargs))
        return self.results


class FakeTracker:
    def __init__(self, updates=None):
        self.inits = []
        self.updates = list(updates or [])

    def init(self, image, bbox_xywh):
        self.inits.append((image.shape, tuple(float(v) for v in bbox_xywh)))
        return True

    def update(self, image):
        if self.updates:
            item = self.updates.pop(0)
            if isinstance(item, tuple) and len(item) == 2 and isinstance(item[0], bool):
                return item
            return True, item
        return True, (100.0, 100.0, 40.0, 40.0)


class FakeCv2:
    COLOR_GRAY2BGR = 1
    COLOR_BGRA2BGR = 2

    @staticmethod
    def cvtColor(image, _code):
        return image


class YoloByteTrackDetectorTest(unittest.TestCase):
    def make_detector(self, results, class_id=2, allow_untracked_fallback=False, single_target_mode=False):
        tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(tempdir.cleanup)
        model_path = Path(tempdir.name) / "target.pt"
        model_path.write_bytes(b"fake")
        model = FakeModel(results)
        detector = YoloByteTrackDetector(
            model_path=str(model_path),
            class_id=class_id,
            conf=0.33,
            iou=0.55,
            imgsz=512,
            device="cpu",
            tracker="bytetrack.yaml",
            allow_untracked_fallback=allow_untracked_fallback,
            single_target_mode=single_target_mode,
            single_target_max_center_jump_px=120.0,
            model_factory=lambda _: model,
            image_reader=lambda _client, _config: np.zeros((480, 640, 3), dtype=np.uint8),
        )
        return detector, model

    def detect_once(self, detector, active_track_id=None):
        return detector.detect(
            client=object(),
            config=AirSimDetectionConfig(camera_name="0", vehicle_name="Interceptor"),
            frame_id=7,
            exposure_ts=1.25,
            active_track_id=active_track_id,
        )

    def test_selects_highest_confidence_matching_class_with_track_id(self):
        result = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(10, 20, 30, 40), (50, 60, 120, 160), (200, 210, 240, 250)],
                conf=[0.90, 0.40, 0.95],
                cls=[1, 2, 2],
                track_ids=[9, 3, 4],
            )
        )
        detector, model = self.make_detector([result], class_id=2)

        frame = self.detect_once(detector)

        self.assertIsNotNone(frame.selected)
        self.assertEqual(frame.frame_detection.track_id, 4)
        self.assertEqual(frame.frame_detection.bbox_xyxy, (200.0, 210.0, 240.0, 250.0))
        self.assertEqual(frame.stats["yolo_raw_count"], 3)
        self.assertEqual(frame.stats["yolo_class_filtered_count"], 2)
        self.assertEqual(frame.stats["yolo_track_filtered_count"], 2)
        self.assertEqual(model.calls[0][1]["conf"], 0.33)
        self.assertEqual(model.calls[0][1]["iou"], 0.55)
        self.assertEqual(model.calls[0][1]["imgsz"], 512)
        self.assertEqual(model.calls[0][1]["device"], "cpu")

    def test_prefers_active_track_id_over_higher_confidence(self):
        result = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(10, 20, 30, 40), (50, 60, 120, 160)],
                conf=[0.95, 0.50],
                cls=[2, 2],
                track_ids=[8, 5],
            )
        )
        detector, _model = self.make_detector([result], class_id=2)

        frame = self.detect_once(detector, active_track_id=5)

        self.assertEqual(frame.frame_detection.track_id, 5)
        self.assertEqual(frame.frame_detection.bbox_xyxy, (50.0, 60.0, 120.0, 160.0))

    def test_rejects_matching_class_when_bytetrack_id_missing(self):
        result = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(10, 20, 30, 40)],
                conf=[0.80],
                cls=[2],
                track_ids=None,
            )
        )
        detector, _model = self.make_detector([result], class_id=2)

        frame = self.detect_once(detector)

        self.assertIsNone(frame.selected)
        self.assertIsNone(frame.frame_detection)
        self.assertEqual(frame.stats["detector_reject_reason"], "yolo_track_id_missing")
        self.assertEqual(frame.stats["yolo_track_missing_count"], 1)

    def test_untracked_fallback_uses_highest_confidence_matching_class(self):
        result = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(10, 20, 30, 40), (50, 60, 120, 160)],
                conf=[0.80, 0.95],
                cls=[2, 2],
                track_ids=None,
            )
        )
        detector, _model = self.make_detector([result], class_id=2, allow_untracked_fallback=True)

        frame = self.detect_once(detector)

        self.assertIsNotNone(frame.selected)
        self.assertIsNotNone(frame.frame_detection)
        self.assertEqual(frame.frame_detection.track_id, -1)
        self.assertEqual(frame.frame_detection.bbox_xyxy, (50.0, 60.0, 120.0, 160.0))
        self.assertEqual(frame.stats["yolo_selected_source"], "untracked_fallback")
        self.assertEqual(frame.stats["yolo_used_untracked_fallback"], 1)

    def test_single_target_mode_prefers_continuity_over_confidence(self):
        first = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(100, 100, 140, 140)],
                conf=[0.70],
                cls=[2],
                track_ids=None,
            )
        )
        second = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(106, 104, 146, 144), (420, 300, 460, 340)],
                conf=[0.60, 0.99],
                cls=[2, 2],
                track_ids=None,
            )
        )
        detector, model = self.make_detector(
            [first],
            class_id=2,
            allow_untracked_fallback=True,
            single_target_mode=True,
        )

        frame1 = self.detect_once(detector)
        model.results = [second]
        frame2 = self.detect_once(detector)

        self.assertEqual(frame1.frame_detection.bbox_xyxy, (100.0, 100.0, 140.0, 140.0))
        self.assertEqual(frame2.frame_detection.bbox_xyxy, (106.0, 104.0, 146.0, 144.0))
        self.assertEqual(frame2.stats["yolo_selected_source"], "single_target")
        self.assertEqual(frame2.stats["yolo_single_target_selected"], 1)

    def test_rejects_when_target_class_missing(self):
        result = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(10, 20, 30, 40)],
                conf=[0.80],
                cls=[1],
                track_ids=[3],
            )
        )
        detector, _model = self.make_detector([result], class_id=2)

        frame = self.detect_once(detector)

        self.assertIsNone(frame.frame_detection)
        self.assertEqual(frame.stats["detector_reject_reason"], "yolo_class_missing")

    def test_requires_existing_model_file(self):
        with self.assertRaisesRegex(RuntimeError, "YOLO model file not found"):
            YoloByteTrackDetector(
                model_path="/tmp/definitely_missing_yolo_model.pt",
                class_id=0,
                model_factory=lambda _: FakeModel([]),
                image_reader=lambda _client, _config: np.zeros((10, 10, 3), dtype=np.uint8),
            )

    def test_requires_class_id(self):
        with tempfile.TemporaryDirectory() as tempdir:
            model_path = Path(tempdir) / "target.pt"
            model_path.write_bytes(b"fake")
            with self.assertRaisesRegex(RuntimeError, "--yolo-class-id"):
                YoloByteTrackDetector(
                    model_path=str(model_path),
                    class_id=None,
                    model_factory=lambda _: FakeModel([]),
                    image_reader=lambda _client, _config: np.zeros((10, 10, 3), dtype=np.uint8),
                )


class YoloKcfDetectorTest(unittest.TestCase):
    def make_detector(self, results, tracker_updates=None, **kwargs):
        tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(tempdir.cleanup)
        model_path = Path(tempdir.name) / "target.pt"
        model_path.write_bytes(b"fake")
        model = FakePredictModel(results)
        trackers = []

        def tracker_factory():
            tracker = FakeTracker(tracker_updates)
            trackers.append(tracker)
            return tracker

        detector = YoloKcfDetector(
            model_path=str(model_path),
            class_id=2,
            conf=0.33,
            iou=0.55,
            imgsz=512,
            device="cpu",
            single_target_mode=True,
            single_target_max_center_jump_px=120.0,
            yolo_period_n=3,
            yolo_period_s=10.0,
            max_coast_s=0.8,
            min_yolo_iou=0.20,
            max_center_jump_px=120.0,
            area_ratio_min=0.3,
            area_ratio_max=3.0,
            model_factory=lambda _: model,
            image_reader=lambda _client, _config: np.zeros((480, 640, 3), dtype=np.uint8),
            cv2_module=FakeCv2(),
            tracker_factory=tracker_factory,
            **kwargs,
        )
        return detector, model, trackers

    def detect_once(self, detector, frame_id=1, ts=1.0, active_track_id=None):
        return detector.detect(
            client=object(),
            config=AirSimDetectionConfig(camera_name="0", vehicle_name="Interceptor"),
            frame_id=frame_id,
            exposure_ts=ts,
            active_track_id=active_track_id,
        )

    def test_yolo_initializes_kcf(self):
        result = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(100, 100, 140, 140)],
                conf=[0.90],
                cls=[2],
                track_ids=None,
            )
        )
        detector, model, trackers = self.make_detector([result])

        frame = self.detect_once(detector)

        self.assertIsNotNone(frame.frame_detection)
        self.assertEqual(frame.frame_detection.track_id, -2)
        self.assertEqual(frame.frame_detection.bbox_xyxy, (100.0, 100.0, 140.0, 140.0))
        self.assertEqual(frame.stats["kcf_source"], "yolo_reinit")
        self.assertEqual(len(model.calls), 1)
        self.assertEqual(trackers[0].inits[0][1], (100.0, 100.0, 40.0, 40.0))

    def test_kcf_tracks_between_yolo_corrections(self):
        result = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(100, 100, 140, 140)],
                conf=[0.90],
                cls=[2],
                track_ids=None,
            )
        )
        detector, model, _trackers = self.make_detector([result], tracker_updates=[(106.0, 104.0, 40.0, 40.0)])

        frame1 = self.detect_once(detector, frame_id=1, ts=1.0)
        frame2 = self.detect_once(detector, frame_id=2, ts=1.1, active_track_id=frame1.frame_detection.track_id)

        self.assertEqual(len(model.calls), 1)
        self.assertEqual(frame2.stats["kcf_source"], "kcf_track")
        self.assertEqual(frame2.frame_detection.bbox_xyxy, (106.0, 104.0, 146.0, 144.0))

    def test_yolo_correction_reinitializes_kcf_when_consistent(self):
        first = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(100, 100, 140, 140)],
                conf=[0.90],
                cls=[2],
                track_ids=None,
            )
        )
        second = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(108, 106, 148, 146)],
                conf=[0.92],
                cls=[2],
                track_ids=None,
            )
        )
        detector, model, trackers = self.make_detector([first], tracker_updates=[(106.0, 104.0, 40.0, 40.0)])

        frame1 = self.detect_once(detector, frame_id=1, ts=1.0)
        model.results = [second]
        frame2 = self.detect_once(detector, frame_id=4, ts=1.3, active_track_id=frame1.frame_detection.track_id)

        self.assertEqual(len(model.calls), 2)
        self.assertEqual(frame2.stats["kcf_source"], "yolo_correct")
        self.assertGreater(float(frame2.stats["kcf_yolo_iou"]), 0.2)
        self.assertEqual(trackers[-1].inits[-1][1], (108.0, 106.0, 40.0, 40.0))

    def test_yolo_drift_reinitializes_when_enabled(self):
        first = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(100, 100, 140, 140)],
                conf=[0.90],
                cls=[2],
                track_ids=None,
            )
        )
        drift = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(400, 300, 450, 350)],
                conf=[0.95],
                cls=[2],
                track_ids=None,
            )
        )
        detector, model, _trackers = self.make_detector([first], tracker_updates=[(102.0, 102.0, 40.0, 40.0)])

        frame1 = self.detect_once(detector, frame_id=1, ts=1.0)
        model.results = [drift]
        frame2 = self.detect_once(detector, frame_id=4, ts=1.3, active_track_id=frame1.frame_detection.track_id)

        self.assertEqual(frame2.stats["kcf_state"], "reset")
        self.assertEqual(frame2.stats["kcf_reject_reason"], "kcf_yolo_iou_low")
        self.assertEqual(frame2.frame_detection.bbox_xyxy, (400.0, 300.0, 450.0, 350.0))

    def test_kcf_update_failure_reports_no_detection(self):
        result = SimpleNamespace(
            boxes=FakeBoxes(
                xyxy=[(100, 100, 140, 140)],
                conf=[0.90],
                cls=[2],
                track_ids=None,
            )
        )
        detector, model, _trackers = self.make_detector([result], tracker_updates=[(False, (0.0, 0.0, 0.0, 0.0))])

        frame1 = self.detect_once(detector, frame_id=1, ts=1.0)
        model.results = []
        frame2 = self.detect_once(detector, frame_id=2, ts=1.1, active_track_id=frame1.frame_detection.track_id)

        self.assertIsNone(frame2.frame_detection)
        self.assertEqual(frame2.stats["kcf_reject_reason"], "kcf_update_failed")


if __name__ == "__main__":
    unittest.main()
