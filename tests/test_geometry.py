import unittest

import numpy as np

from vision_guidance.geometry import (
    airsim_fixed_camera_to_body,
    airsim_camera_zero_to_body,
    airsim_gimbal_camera_to_body,
    camera_mount_diagnostics,
    camera_ray_from_pixel,
    normalize,
    project_perpendicular,
    validated_rotation_matrix,
)
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

    def test_airsim_camera_zero_points_body_forward(self):
        R_BC = airsim_camera_zero_to_body()
        self.assertTrue(np.allclose(R_BC @ np.array([0.0, 0.0, 1.0]), [1.0, 0.0, 0.0]))
        self.assertTrue(np.allclose(R_BC @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]))
        self.assertTrue(np.allclose(R_BC @ np.array([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0]))

    def test_airsim_gimbal_positive_yaw_points_right(self):
        R_BC = airsim_gimbal_camera_to_body(np.deg2rad(30.0), 0.0)
        forward_B = R_BC @ np.array([0.0, 0.0, 1.0])
        self.assertGreater(forward_B[1], 0.0)
        self.assertAlmostEqual(np.linalg.norm(forward_B), 1.0)

    def test_airsim_gimbal_positive_pitch_points_down(self):
        R_BC = airsim_gimbal_camera_to_body(0.0, np.deg2rad(20.0))
        forward_B = R_BC @ np.array([0.0, 0.0, 1.0])
        self.assertGreater(forward_B[2], 0.0)
        self.assertAlmostEqual(np.linalg.norm(forward_B), 1.0)

    def test_airsim_fixed_negative_pitch_points_up(self):
        R_BC = airsim_fixed_camera_to_body(pitch_rad=np.deg2rad(-15.0))
        forward_B = R_BC @ np.array([0.0, 0.0, 1.0])
        self.assertLess(forward_B[2], 0.0)
        self.assertAlmostEqual(np.linalg.norm(forward_B), 1.0)

    def test_upward_camera_mount_diagnostics_use_frd_body_up(self):
        R_BC = np.array(
            [
                [0.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0],
            ]
        )
        diagnostics = camera_mount_diagnostics(R_BC)

        self.assertEqual(diagnostics["optical_axis_body"], [0.0, 0.0, -1.0])
        self.assertAlmostEqual(diagnostics["optical_axis_error_deg"], 0.0)
        self.assertAlmostEqual(diagnostics["determinant"], 1.0)

    def test_rotation_validation_rejects_scale_and_reflection(self):
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            validated_rotation_matrix(np.diag([1.0, 1.0, 2.0]))
        with self.assertRaisesRegex(ValueError, "determinant"):
            validated_rotation_matrix(np.diag([1.0, 1.0, -1.0]))


if __name__ == "__main__":
    unittest.main()
