import numpy as np
import unittest

from vision_guidance.geometry import camera_ray_from_pixel, normalize, project_perpendicular
from vision_guidance.types import CameraIntrinsics


class GeometryTest(unittest.TestCase):
    def test_camera_center_ray_points_forward(self):
        intr = CameraIntrinsics(500, 500, 320, 240, 640, 480)
        ray = camera_ray_from_pixel(320, 240, intr)
        self.assertTrue(np.allclose(ray, [0, 0, 1]))

    def test_project_perpendicular_removes_axis_component(self):
        axis = normalize(np.array([0.0, 0.0, 1.0]))
        out = project_perpendicular(np.array([1.0, 2.0, 3.0]), axis)
        self.assertLess(abs(np.dot(out, axis)), 1e-9)


if __name__ == "__main__":
    unittest.main()
