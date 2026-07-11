import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from vision_guidance.rknn_native_detector import RknnDetectorConfig, RknnNativeDetector


class _FakeBridge:
    abi_version = 1
    output_schema = {"input": [640, 640, 3], "outputs": [{"shape": [1, 64, 80, 80]}]}

    def __init__(self, results):
        self.results = list(results)
        self.images = []
        self.closed = False

    def infer(self, image):
        self.images.append(image.copy())
        return self.results.pop(0)

    def close(self):
        self.closed = True


def _result(**overrides):
    values = {
        "valid": 1,
        "x1": 10.0,
        "y1": 20.0,
        "x2": 50.0,
        "y2": 70.0,
        "score": 0.8,
        "class_id": 0,
        "track_id": 3,
        "raw_count": 2,
        "accepted_count": 1,
        "selected_index": 0,
        "reject_code": 0,
        "preprocess_ms": 1.0,
        "inference_ms": 4.0,
        "postprocess_ms": 0.5,
        "total_ms": 5.5,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class RknnNativeDetectorTest(unittest.TestCase):
    def test_maps_modified_native_detector_result_to_frame_detection(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            model = Path(tmpdir) / "modified.rknn"
            model.write_bytes(b"modified-model")
            bridge = _FakeBridge([_result()])
            detector = RknnNativeDetector(
                library_path=Path(tmpdir) / "bridge.so",
                model_path=model,
                config=RknnDetectorConfig(),
                bridge=bridge,
            )

            detection, stats = detector.detect(
                np.zeros((512, 640, 3), dtype=np.uint8),
                frame_id=8,
                exposure_ts=12.5,
            )

            self.assertIsNotNone(detection)
            self.assertEqual(detection.bbox_xyxy, (10.0, 20.0, 50.0, 70.0))
            self.assertEqual(detection.track_id, 3)
            self.assertEqual(stats["detector_source"], "rknn_native")
            self.assertEqual(stats["detector_raw_count"], 2)
            self.assertEqual(stats["rknn_inference_ms"], 4.0)
            self.assertEqual(detector.metadata()["output_schema"]["input"], [640, 640, 3])
            self.assertEqual(len(detector.metadata()["model_sha256"]), 64)

    def test_reports_native_filter_rejection_without_fabricating_detection(self):
        bridge = _FakeBridge([_result(valid=0, reject_code=2, accepted_count=0, selected_index=-1)])
        detector = RknnNativeDetector("missing.so", "missing.rknn", RknnDetectorConfig(), bridge=bridge)

        detection, stats = detector.detect(
            np.zeros((10, 10, 3), dtype=np.uint8), frame_id=1, exposure_ts=1.0
        )

        self.assertIsNone(detection)
        self.assertEqual(stats["detector_reject_reason"], "rknn_candidates_filtered")

    def test_requires_packed_uint8_rgb_shape(self):
        bridge = _FakeBridge([_result()])
        detector = RknnNativeDetector("missing.so", "missing.rknn", RknnDetectorConfig(), bridge=bridge)

        with self.assertRaisesRegex(ValueError, "HxWx3 uint8 RGB"):
            detector.detect(np.zeros((10, 10), dtype=np.uint8), frame_id=1, exposure_ts=1.0)


if __name__ == "__main__":
    unittest.main()
