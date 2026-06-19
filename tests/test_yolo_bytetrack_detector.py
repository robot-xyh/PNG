import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from vision_guidance.airsim_adapter import AirSimDetectionConfig
from vision_guidance.yolo_bytetrack_detector import YoloByteTrackDetector


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


class YoloByteTrackDetectorTest(unittest.TestCase):
    def make_detector(self, results, class_id=2):
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


if __name__ == "__main__":
    unittest.main()
