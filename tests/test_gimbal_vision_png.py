import unittest
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from examples.run_airsim_gimbal_vision_png import (
    _bearing_deg_from_xy,
    _camera_offset_body,
    _command_duration,
    _gimbal_from_relative_body,
    _gimbal_update_profile,
    _guidance_velocity,
    _image_kf_takeover_allowed,
    _los_fallback_allowed,
    _rows_until_first_hit,
    _terminal_trigger,
    _terminal_image_kf_config_from_args,
    _update_gimbal_from_pixel,
    _wrap_angle_deg,
    _write_run_metadata,
    _yaw_deg_from_velocity,
)
from vision_guidance.terminal_extrapolation import BLIND_PUSH, TERMINAL_VISUAL
from vision_guidance.terminal_image_kf import TerminalImageEstimate
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

    def test_gimbal_update_gain_scale_can_freeze_motion(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        args = SimpleNamespace(
            gimbal_rate_limit_deg=90.0,
            gimbal_yaw_gain=1.0,
            gimbal_pitch_gain=1.0,
            gimbal_yaw_limit_deg=80.0,
            gimbal_pitch_limit_deg=45.0,
        )

        yaw, pitch, yaw_error, pitch_error = _update_gimbal_from_pixel(
            0.25,
            -0.15,
            (500.0, 300.0),
            intrinsics,
            0.1,
            args,
            gain_scale=0.0,
        )

        self.assertGreater(yaw_error, 0.0)
        self.assertGreater(pitch_error, 0.0)
        self.assertAlmostEqual(yaw, 0.25)
        self.assertAlmostEqual(pitch, -0.15)

    def test_gimbal_terminal_profile_holds_or_scales_updates(self):
        args = SimpleNamespace(terminal_freeze_gimbal_on_blind=True, terminal_gimbal_gain_scale=0.35)

        enabled, scale, reason = _gimbal_update_profile(
            detected=True,
            terminal_state=TERMINAL_VISUAL,
            terminal_reason="",
            using_blind_push=False,
            args=args,
        )
        self.assertTrue(enabled)
        self.assertAlmostEqual(scale, 0.35)
        self.assertEqual(reason, "terminal_scaled")

        enabled, scale, reason = _gimbal_update_profile(
            detected=True,
            terminal_state=TERMINAL_VISUAL,
            terminal_reason="bbox_top_clipped",
            using_blind_push=False,
            args=args,
        )
        self.assertFalse(enabled)
        self.assertEqual(scale, 0.0)
        self.assertEqual(reason, "bbox_top_clipped_hold")

        enabled, scale, reason = _gimbal_update_profile(
            detected=True,
            terminal_state=BLIND_PUSH,
            terminal_reason="terminal_lost",
            using_blind_push=True,
            args=args,
        )
        self.assertFalse(enabled)
        self.assertEqual(scale, 0.0)
        self.assertEqual(reason, "blind_push_hold")

    def test_terminal_image_kf_config_from_args(self):
        args = SimpleNamespace(
            terminal_image_kf=True,
            terminal_image_kf_max_predict_s=0.22,
            terminal_image_kf_meas_noise_rad=0.007,
            terminal_image_kf_accel_noise_rad_s2=9.0,
            terminal_image_kf_innovation_reject_rad=0.18,
            terminal_image_kf_max_angle_rad=0.9,
            terminal_image_kf_max_rate_rad_s=7.0,
        )

        config = _terminal_image_kf_config_from_args(args)

        self.assertTrue(config.enable)
        self.assertAlmostEqual(config.max_predict_s, 0.22)
        self.assertAlmostEqual(config.measurement_noise_rad, 0.007)
        self.assertAlmostEqual(config.accel_noise_rad_s2, 9.0)
        self.assertAlmostEqual(config.innovation_reject_rad, 0.18)
        self.assertAlmostEqual(config.max_angle_rad, 0.9)
        self.assertAlmostEqual(config.max_rate_rad_s, 7.0)

    def test_image_kf_takeover_allows_terminal_prediction(self):
        image_kf = TerminalImageEstimate(
            timestamp=1.0,
            theta_x=0.1,
            theta_y=-0.1,
            theta_dot_x=0.2,
            theta_dot_y=0.0,
            valid=True,
            mode="predict",
            age_s=0.08,
            quality=0.7,
            reject_reason="bbox_clipped",
        )
        terminal_result = SimpleNamespace(
            state=TERMINAL_VISUAL,
            reason="",
            using_blind_push=False,
        )
        args = SimpleNamespace(terminal_soft_enter_area_ratio=0.05)

        allowed, reason = _image_kf_takeover_allowed(
            image_kf,
            terminal_result,
            "bbox_clipped",
            True,
            0.12,
            args,
            profile="gimbal",
        )

        self.assertTrue(allowed)
        self.assertIn(reason, {"terminal_state", "bbox_clipped"})

    def test_write_run_metadata_records_experiment_parameters(self):
        args = SimpleNamespace(
            settings_path="/tmp/settings.json",
            interceptor="Interceptor",
            intruder="Intruder",
            speed_ratio=2.0,
            intruder_altitude_offset_m=30.0,
        )
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        rows = [{"range": 3.0}, {"range": 1.0}]

        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "run.csv"
            metadata_path = _write_run_metadata(
                args=args,
                csv_path=csv_path,
                script_name="example.py",
                experiment_type="gimbal_vision_png",
                intrinsics=intrinsics,
                speed_cap=10.0,
                intruder_velocity_cmd=np.array([0.0, 5.0, 0.0]),
                rows=rows,
                hit=True,
            )

            data = json.loads(metadata_path.read_text(encoding="utf-8"))

        self.assertEqual(data["experiment_type"], "gimbal_vision_png")
        self.assertEqual(data["args"]["speed_ratio"], 2.0)
        self.assertEqual(data["args"]["intruder_altitude_offset_m"], 30.0)
        self.assertEqual(data["derived"]["speed_cap"], 10.0)
        self.assertEqual(data["derived"]["min_range_m"], 1.0)

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
        self.assertEqual(_terminal_trigger("bbox_top_clipped", True, 1.0, 0.0, 0.0, intrinsics, args), "bbox_top_clipped")
        self.assertEqual(_terminal_trigger("", True, 90000.0, 0.0, 0.0, intrinsics, args), "bbox_area_large")
        self.assertEqual(
            _terminal_trigger("", True, 40000.0, np.deg2rad(78.0), 0.0, intrinsics, args),
            "gimbal_limit",
        )
        self.assertEqual(_terminal_trigger("no_detection", False, 0.0, 0.0, 0.0, intrinsics, args), "")

    def test_plot_rows_stop_at_first_hit(self):
        rows = [{"t": 0.0, "hit": 0}, {"t": 1.0, "hit": 1}, {"t": 2.0, "hit": 0}]

        clipped = _rows_until_first_hit(rows)

        self.assertEqual([row["t"] for row in clipped], [0.0, 1.0])


if __name__ == "__main__":
    unittest.main()
