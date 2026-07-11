import unittest
from types import SimpleNamespace

import numpy as np

from vision_guidance.bytetrack_adapter import ByteTrackUpdate, TrackedDetection
from vision_guidance.rknn_bytetrack_detector import RknnByteTrackDetector
from vision_guidance.rknn_native_detector import RknnDetection, RknnDetectionBatch, RknnDetectorConfig


class _FakeBridge:
    abi_version = 2
    output_schema = {"input": [640, 640, 3], "outputs": []}

    def infer_all(self, image, *, capacity):
        self.call = (image.copy(), capacity)
        return RknnDetectionBatch(
            detections=(RknnDetection((10.0, 20.0, 30.0, 50.0), 0.8, 0, 0),),
            total_count=1,
            truncated=False,
            preprocess_ms=1.0,
            inference_ms=5.0,
            postprocess_ms=0.5,
            total_ms=6.5,
        )

    def close(self):
        self.closed = True


class _FakeTracker:
    def update(self, detections, *, timestamp):
        self.call = (tuple(detections), timestamp)
        return ByteTrackUpdate(
            selected=TrackedDetection((10.0, 20.0, 30.0, 50.0), 7, 0.8, 0, 3, 3, True),
            stats={
                "tracker_state": "tracked",
                "target_selector_reason": "new_track_locked",
                "bbox_measurement_source": "detector_update",
            },
        )

    def metadata(self):
        return {"implementation": "fake"}


class RknnByteTrackDetectorTest(unittest.TestCase):
    def test_maps_batch_through_tracker_to_frame_detection(self):
        bridge = _FakeBridge()
        tracker = _FakeTracker()
        detector = RknnByteTrackDetector(
            "missing.so",
            "missing.rknn",
            RknnDetectorConfig(max_det=300),
            SimpleNamespace(),
            bridge=bridge,
            tracker=tracker,
        )

        detection, stats = detector.detect(
            np.zeros((512, 640, 3), dtype=np.uint8), frame_id=5, exposure_ts=2.0
        )

        self.assertEqual(detection.track_id, 7)
        self.assertEqual(detection.bbox_xyxy, (10.0, 20.0, 30.0, 50.0))
        self.assertEqual(bridge.call[1], 300)
        self.assertEqual(stats["detector_source"], "rknn_bytetrack")
        self.assertEqual(stats["rknn_total_ms"], 6.5)
        self.assertEqual(stats["bbox_measurement_source"], "detector_update")


if __name__ == "__main__":
    unittest.main()
