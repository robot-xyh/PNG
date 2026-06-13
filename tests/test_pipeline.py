import numpy as np
import unittest

from vision_guidance.attitude_buffer import AttitudeHistoryBuffer
from vision_guidance.fusion import PureVisionGuidancePipeline
from vision_guidance.geometry import camera_to_body_mount
from vision_guidance.types import AttitudeSample, CameraIntrinsics, FrameDetection


class PipelineTest(unittest.TestCase):
    def test_pipeline_produces_guidance_eval_after_ttc_valid(self):
        intr = CameraIntrinsics(500, 500, 320, 240, 640, 480)
        attitudes = AttitudeHistoryBuffer(duration_s=1.0)
        for i in range(20):
            attitudes.push(AttitudeSample(i * 0.05, np.eye(3)))
        pipe = PureVisionGuidancePipeline(intr, camera_to_body_mount(0.0), attitudes)

        result = None
        for i in range(2, 10):
            ts = i * 0.05
            size = 15 + i
            det = FrameDetection(i, ts, (300, 220, 300 + size, 220 + size), 1)
            result = pipe.process(det)
        self.assertIsNotNone(result)
        self.assertIsNotNone(result.los)
        self.assertIsNotNone(result.ttc)
        self.assertTrue(result.guidance.valid)

    def test_pipeline_rejects_track_change(self):
        intr = CameraIntrinsics(500, 500, 320, 240, 640, 480)
        attitudes = AttitudeHistoryBuffer(duration_s=1.0)
        attitudes.push(AttitudeSample(0.0, np.eye(3)))
        attitudes.push(AttitudeSample(0.1, np.eye(3)))
        pipe = PureVisionGuidancePipeline(intr, camera_to_body_mount(0.0), attitudes)

        pipe.process(FrameDetection(1, 0.05, (300, 220, 320, 240), 1))
        result = pipe.process(FrameDetection(2, 0.06, (300, 220, 321, 241), 2))
        self.assertFalse(result.guidance.valid)
        self.assertEqual(result.guidance.reject_reason, "track_id_changed")


if __name__ == "__main__":
    unittest.main()
