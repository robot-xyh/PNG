import unittest
from types import SimpleNamespace

import numpy as np

from examples.run_airsim_gimbal_vision_png import (
    _bearing_deg_from_xy,
    _camera_offset_body,
    _command_duration,
    _gimbal_from_relative_body,
    _guidance_velocity,
    _los_fallback_allowed,
    _terminal_trigger,
    _update_gimbal_from_pixel,
    _wrap_angle_deg,
    _yaw_deg_from_velocity,
)
from vision_guidance.types import CameraIntrinsics


class GimbalVisionPNGTest(unittest.TestCase):
    def test_gimbal_update_moves_right_and_down_for_positive_pixel_errors(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        args = SimpleNamespace(
            gimbal_rate_limit_deg=90.0,
            gimbal_yaw_gain=1.0,
            gimbal_pitch_gain=1.0,
            gimbal_yaw_limit_deg=80.0,
            gimbal_pitch_limit_deg=45.0,
        )

        yaw, pitch, yaw_error, pitch_error = _update_gimbal_from_pixel(
            0.0,
            0.0,
            (400.0, 280.0),
            intrinsics,
            0.1,
            args,
        )

        self.assertGreater(yaw_error, 0.0)
        self.assertGreater(pitch_error, 0.0)
        self.assertGreater(yaw, 0.0)
        self.assertGreater(pitch, 0.0)

    def test_guidance_velocity_uses_3d_los_and_speed_cap(self):
        args = SimpleNamespace(max_vision_lateral_speed=4.0, max_vision_vertical_speed=3.0)

        command = _guidance_velocity(
            own_velocity=np.zeros(3),
            lambda_I=np.array([0.6, 0.8, 0.4]),
            omega_los=None,
            gain=0.0,
            speed_cap=10.0,
            args=args,
        )

        self.assertLessEqual(np.linalg.norm(command), 10.0 + 1.0e-9)
        self.assertGreater(command[0], 0.0)
        self.assertGreater(command[1], 0.0)
        self.assertGreater(command[2], 0.0)

    def test_initial_truth_alignment_relative_body_to_gimbal(self):
        args = SimpleNamespace(gimbal_yaw_limit_deg=80.0, gimbal_pitch_limit_deg=45.0)

        yaw, pitch = _gimbal_from_relative_body(np.array([10.0, 10.0, 5.0]), args)

        self.assertGreater(yaw, 0.0)
        self.assertGreater(pitch, 0.0)

    def test_camera_offset_defaults_to_air_sim_ned_upward_mount(self):
        args = SimpleNamespace(camera_x=0.0, camera_y=0.0, camera_z=-0.5)

        self.assertTrue(np.allclose(_camera_offset_body(args), np.array([0.0, 0.0, -0.5])))

    def test_command_duration_covers_slow_air_sim_loop(self):
        args = SimpleNamespace(
            command_duration_margin_s=0.2,
            min_command_duration_s=0.25,
            max_command_duration_s=1.0,
        )

        self.assertAlmostEqual(_command_duration(0.58, 0.05, args), 0.78)

    def test_command_duration_has_minimum_for_fast_loop(self):
        args = SimpleNamespace(
            command_duration_margin_s=0.2,
            min_command_duration_s=0.25,
            max_command_duration_s=1.0,
        )

        self.assertAlmostEqual(_command_duration(0.01, 0.05, args), 0.25)

    def test_los_fallback_is_limited_to_soft_ttc_failures(self):
        args = SimpleNamespace(allow_los_fallback=True)

        self.assertTrue(_los_fallback_allowed("ttc_out_of_range", args))
        self.assertTrue(_los_fallback_allowed("area_not_expanding", args))
        self.assertFalse(_los_fallback_allowed("no_detection", args))
        self.assertFalse(_los_fallback_allowed("los_invalid", args))

    def test_yaw_from_velocity_uses_horizontal_heading_degrees(self):
        self.assertAlmostEqual(_yaw_deg_from_velocity(np.array([0.0, 2.0, 1.0])), 90.0)
        self.assertAlmostEqual(_yaw_deg_from_velocity(np.array([1.0, 1.0, 0.0])), 45.0)

    def test_angle_diagnostics_wrap_and_bearing(self):
        self.assertAlmostEqual(_wrap_angle_deg(190.0), -170.0)
        self.assertAlmostEqual(_wrap_angle_deg(-190.0), 170.0)
        self.assertAlmostEqual(_bearing_deg_from_xy(np.array([0.0, 1.0])), 90.0)

    def test_terminal_trigger_conditions(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        args = SimpleNamespace(
            terminal_bbox_area_ratio=0.25,
            gimbal_limit_margin_deg=3.0,
            gimbal_yaw_limit_deg=80.0,
            gimbal_pitch_limit_deg=45.0,
        )

        self.assertEqual(_terminal_trigger("bbox_clipped", True, 1.0, 0.0, 0.0, intrinsics, args), "bbox_clipped")
        self.assertEqual(_terminal_trigger("", True, 90000.0, 0.0, 0.0, intrinsics, args), "bbox_area_large")
        self.assertEqual(
            _terminal_trigger("", True, 40000.0, np.deg2rad(78.0), 0.0, intrinsics, args),
            "gimbal_limit",
        )
        self.assertEqual(_terminal_trigger("no_detection", False, 0.0, 0.0, 0.0, intrinsics, args), "")


if __name__ == "__main__":
    unittest.main()
