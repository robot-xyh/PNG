import unittest
from types import SimpleNamespace

import numpy as np

from vision_guidance.airsim_adapter import (
    airsim_orientation_to_R_IB,
    choose_detection,
    detection_to_frame_detection,
    infer_intrinsics_from_fov,
)


def make_detection(name, x1, y1, x2, y2):
    return SimpleNamespace(
        name=name,
        box2D=SimpleNamespace(
            min=SimpleNamespace(x_val=x1, y_val=y1),
            max=SimpleNamespace(x_val=x2, y_val=y2),
        ),
        # Intentionally include fields that the adapter must not use.
        relative_pose=SimpleNamespace(position=SimpleNamespace(x_val=999)),
        geo_point=SimpleNamespace(latitude=1, longitude=2),
    )


class AirSimAdapterTest(unittest.TestCase):
    def test_detection_to_frame_detection_uses_only_box(self):
        det = make_detection("Intruder_01", 10, 20, 30, 50)
        frame = detection_to_frame_detection(det, frame_id=7, exposure_ts=1.5, track_id=3)
        self.assertEqual(frame.frame_id, 7)
        self.assertEqual(frame.exposure_ts, 1.5)
        self.assertEqual(frame.track_id, 3)
        self.assertEqual(frame.bbox_xyxy, (10.0, 20.0, 30.0, 50.0))

    def test_choose_detection_prefers_name_or_largest_area(self):
        small = make_detection("small", 0, 0, 10, 10)
        large = make_detection("large", 0, 0, 20, 20)
        self.assertIs(choose_detection([small, large]), large)
        self.assertIs(choose_detection([small, large], preferred_name="small"), small)

    def test_quaternion_identity(self):
        orientation = SimpleNamespace(w_val=1.0, x_val=0.0, y_val=0.0, z_val=0.0)
        self.assertTrue(np.allclose(airsim_orientation_to_R_IB(orientation), np.eye(3)))

    def test_intrinsics_from_fov(self):
        intr = infer_intrinsics_from_fov(640, 480, 90.0)
        self.assertAlmostEqual(intr.fx, 320.0)
        self.assertAlmostEqual(intr.fy, 320.0)
        self.assertEqual(intr.cx, 320.0)
        self.assertEqual(intr.cy, 240.0)


if __name__ == "__main__":
    unittest.main()
