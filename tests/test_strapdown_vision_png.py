import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from examples.run_airsim_strapdown_vision_png import (
    _append_yaw_rate_sample,
    _apply_bbox_noise,
    _fixed_camera_pose,
    _image_kf_takeover_allowed,
    _kf_yaw_rate_command,
    _output_paths_strapdown,
    _plot_strapdown,
    _rows_until_first_hit,
    _start_geometry_offsets,
    _terminal_trigger_strapdown,
    _terminal_yaw_rate_command,
    _yaw_rate_from_angle_error,
    _yaw_rate_from_pixel_error,
)
from vision_guidance.terminal_image_kf import TerminalImageEstimate
from vision_guidance.terminal_extrapolation import BLIND_PUSH, TERMINAL_VISUAL
from vision_guidance.types import CameraIntrinsics, FrameDetection


class StrapdownVisionPNGTest(unittest.TestCase):
    def test_yaw_rate_from_pixel_error_turns_toward_right_side_target(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        args = SimpleNamespace(yaw_control=True, yaw_error_gain=2.0, max_yaw_rate_deg=90.0)

        self.assertGreater(_yaw_rate_from_pixel_error(80.0, intrinsics, args), 0.0)
        self.assertLess(_yaw_rate_from_pixel_error(-80.0, intrinsics, args), 0.0)

    def test_yaw_rate_respects_limit_and_disable_switch(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        args = SimpleNamespace(yaw_control=True, yaw_error_gain=20.0, max_yaw_rate_deg=30.0)

        self.assertAlmostEqual(_yaw_rate_from_pixel_error(640.0, intrinsics, args), 30.0)

        args.yaw_control = False
        self.assertAlmostEqual(_yaw_rate_from_pixel_error(640.0, intrinsics, args), 0.0)

    def test_yaw_rate_from_angle_error_and_kf_feedforward(self):
        args = SimpleNamespace(yaw_control=True, yaw_error_gain=2.0, max_yaw_rate_deg=30.0)

        self.assertAlmostEqual(_yaw_rate_from_angle_error(0.10, args), np.rad2deg(0.20))

        image_kf = TerminalImageEstimate(
            timestamp=0.0,
            theta_x=0.10,
            theta_y=0.0,
            theta_dot_x=0.20,
            theta_dot_y=0.0,
            valid=True,
            mode="predict",
            age_s=0.05,
            quality=0.8,
        )

        command = _kf_yaw_rate_command(image_kf, args)

        self.assertAlmostEqual(command, np.rad2deg(0.40))

        args.max_yaw_rate_deg = 10.0
        self.assertAlmostEqual(_kf_yaw_rate_command(image_kf, args), 10.0)

    def test_strapdown_image_kf_can_take_over_on_terminal_los_reject(self):
        image_kf = TerminalImageEstimate(
            timestamp=1.0,
            theta_x=0.05,
            theta_y=0.02,
            theta_dot_x=0.10,
            theta_dot_y=0.0,
            valid=True,
            mode="predict",
            age_s=0.06,
            quality=0.8,
        )
        terminal_result = SimpleNamespace(
            state="Tracking",
            reason="",
            using_blind_push=False,
        )
        args = SimpleNamespace(terminal_soft_enter_area_ratio=0.05)

        allowed, reason = _image_kf_takeover_allowed(
            image_kf,
            terminal_result,
            "los_innovation_reject",
            True,
            0.08,
            args,
            profile="strapdown",
        )

        self.assertTrue(allowed)
        self.assertEqual(reason, "strapdown_los_reject")

    def test_terminal_yaw_rate_scales_terminal_visual_and_extrapolates_blind_push(self):
        args = SimpleNamespace(
            terminal_yaw_rate_extrapolation=True,
            terminal_yaw_rate_average_window_s=0.10,
            terminal_yaw_rate_decay_tau_s=0.20,
            terminal_yaw_rate_scale=0.50,
            max_yaw_rate_deg=90.0,
        )
        samples: list[tuple[float, float]] = []
        for ts, value in [(0.00, 20.0), (0.04, 30.0), (0.08, 40.0)]:
            _append_yaw_rate_sample(samples, ts, value, True, args)

        command, base, count, decay = _terminal_yaw_rate_command(
            current_yaw_rate_deg_s=30.0,
            samples=samples,
            timestamp=0.10,
            terminal_state=TERMINAL_VISUAL,
            using_blind_push=False,
            blind_elapsed_s=0.0,
            args=args,
        )
        self.assertAlmostEqual(command, 15.0)
        self.assertAlmostEqual(base, 0.0)
        self.assertEqual(count, 0)
        self.assertAlmostEqual(decay, 1.0)

        command, base, count, decay = _terminal_yaw_rate_command(
            current_yaw_rate_deg_s=0.0,
            samples=samples,
            timestamp=0.10,
            terminal_state=BLIND_PUSH,
            using_blind_push=True,
            blind_elapsed_s=0.10,
            args=args,
        )
        self.assertAlmostEqual(base, 30.0)
        self.assertEqual(count, 3)
        self.assertGreater(decay, 0.0)
        self.assertLess(command, base)
        self.assertGreater(command, 0.0)

    def test_terminal_yaw_rate_extrapolation_can_be_disabled(self):
        args = SimpleNamespace(
            terminal_yaw_rate_extrapolation=False,
            terminal_yaw_rate_average_window_s=0.10,
            terminal_yaw_rate_decay_tau_s=0.20,
            terminal_yaw_rate_scale=0.50,
            max_yaw_rate_deg=90.0,
        )

        command, base, count, decay = _terminal_yaw_rate_command(
            current_yaw_rate_deg_s=0.0,
            samples=[(0.0, 40.0)],
            timestamp=0.10,
            terminal_state=BLIND_PUSH,
            using_blind_push=True,
            blind_elapsed_s=0.10,
            args=args,
        )

        self.assertAlmostEqual(command, 0.0)
        self.assertAlmostEqual(base, 0.0)
        self.assertEqual(count, 0)
        self.assertAlmostEqual(decay, 0.0)

    def test_terminal_trigger_strapdown_conditions(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        args = SimpleNamespace(terminal_bbox_area_ratio=0.25)

        self.assertEqual(_terminal_trigger_strapdown("bbox_clipped", True, 1.0, intrinsics, args), "bbox_clipped")
        self.assertEqual(_terminal_trigger_strapdown("bbox_top_clipped", True, 1.0, intrinsics, args), "bbox_top_clipped")
        self.assertEqual(_terminal_trigger_strapdown("", True, 90000.0, intrinsics, args), "bbox_area_large")
        self.assertEqual(_terminal_trigger_strapdown("", True, 1000.0, intrinsics, args), "")

    def test_bbox_noise_disabled_preserves_detection(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        detection = FrameDetection(1, 1.0, (100.0, 120.0, 180.0, 220.0), 1)
        args = SimpleNamespace(bbox_noise=False, bbox_center_noise_px=3.0, bbox_area_noise_ratio=0.08)

        noisy, info = _apply_bbox_noise(detection, intrinsics, np.random.default_rng(7), args)

        self.assertEqual(noisy.bbox_xyxy, detection.bbox_xyxy)
        self.assertEqual(info["raw_bbox"], detection.bbox_xyxy)
        self.assertEqual(info["noisy_bbox"], detection.bbox_xyxy)
        self.assertAlmostEqual(info["area_scale"], 1.0)

    def test_bbox_noise_is_reproducible_with_seed(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        detection = FrameDetection(1, 1.0, (100.0, 120.0, 180.0, 220.0), 1)
        args = SimpleNamespace(bbox_noise=True, bbox_center_noise_px=3.0, bbox_area_noise_ratio=0.08)

        noisy_a, info_a = _apply_bbox_noise(detection, intrinsics, np.random.default_rng(123), args)
        noisy_b, info_b = _apply_bbox_noise(detection, intrinsics, np.random.default_rng(123), args)

        self.assertEqual(noisy_a.bbox_xyxy, noisy_b.bbox_xyxy)
        self.assertAlmostEqual(info_a["dx"], info_b["dx"])
        self.assertAlmostEqual(info_a["dy"], info_b["dy"])
        self.assertAlmostEqual(info_a["area_scale"], info_b["area_scale"])

    def test_bbox_noise_preserves_aspect_ratio_when_unclipped(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        detection = FrameDetection(1, 1.0, (200.0, 180.0, 300.0, 230.0), 1)
        args = SimpleNamespace(bbox_noise=True, bbox_center_noise_px=0.0, bbox_area_noise_ratio=0.30)

        noisy, info = _apply_bbox_noise(detection, intrinsics, np.random.default_rng(42), args)

        self.assertAlmostEqual(noisy.width / noisy.height, detection.width / detection.height)
        self.assertAlmostEqual(noisy.area / detection.area, info["area_scale"])

    def test_bbox_noise_clamps_to_image_bounds(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        detection = FrameDetection(1, 1.0, (0.5, 0.5, 20.0, 30.0), 1)
        args = SimpleNamespace(bbox_noise=True, bbox_center_noise_px=20.0, bbox_area_noise_ratio=1.0)

        noisy, _ = _apply_bbox_noise(detection, intrinsics, np.random.default_rng(1), args)
        x1, y1, x2, y2 = noisy.bbox_xyxy

        self.assertGreaterEqual(x1, 0.0)
        self.assertGreaterEqual(y1, 0.0)
        self.assertLessEqual(x2, intrinsics.width)
        self.assertLessEqual(y2, intrinsics.height)
        self.assertGreater(x2, x1)
        self.assertGreater(y2, y1)

    def test_fixed_camera_pose_uses_positive_airsim_pitch_for_upward_mount(self):
        class FakeAirSim:
            @staticmethod
            def Vector3r(x, y, z):
                return (x, y, z)

            @staticmethod
            def to_quaternion(pitch, roll, yaw):
                return (pitch, roll, yaw)

            @staticmethod
            def Pose(position, orientation):
                return position, orientation

        args = SimpleNamespace(
            camera_x=0.0,
            camera_y=0.0,
            camera_z=-0.5,
            camera_pitch_deg=-15.0,
            camera_roll_deg=0.0,
            camera_yaw_deg=0.0,
        )

        _, orientation = _fixed_camera_pose(FakeAirSim, args)

        self.assertAlmostEqual(orientation[0], np.deg2rad(15.0))

    def test_default_output_prefix_is_strapdown(self):
        args = SimpleNamespace(trajectory_prefix="", trajectory_dir="/tmp")

        csv_path, plot_path = _output_paths_strapdown(args)

        self.assertTrue(csv_path.name.startswith("strapdown_vision_png_"))
        self.assertEqual(csv_path.suffix, ".csv")
        self.assertEqual(plot_path.suffix, ".png")

    def test_plot_strapdown_does_not_require_gimbal_fields(self):
        rows = [
            {
                "t": 0.0,
                "range": 10.0,
                "pixel_error_x": 0.0,
                "pixel_error_y": 0.0,
                "yaw_rate_cmd_deg_s": 0.0,
                "body_yaw_deg": 0.0,
                "cmd_yaw_deg": 0.0,
                "target_body_bearing_deg": 0.0,
                "interceptor_x": 0.0,
                "interceptor_y": 0.0,
                "interceptor_z": -50.0,
                "intruder_x": 10.0,
                "intruder_y": 0.0,
                "intruder_z": -80.0,
            },
            {
                "t": 1.0,
                "range": 8.0,
                "pixel_error_x": 5.0,
                "pixel_error_y": -2.0,
                "yaw_rate_cmd_deg_s": 4.0,
                "body_yaw_deg": 2.0,
                "cmd_yaw_deg": 5.0,
                "target_body_bearing_deg": 3.0,
                "interceptor_x": 1.0,
                "interceptor_y": 0.0,
                "interceptor_z": -51.0,
                "intruder_x": 10.0,
                "intruder_y": 1.0,
                "intruder_z": -80.0,
            },
        ]
        with tempfile.TemporaryDirectory() as tmp:
            plot_path = Path(tmp) / "strapdown.png"

            ok = _plot_strapdown(rows, plot_path)

            self.assertTrue(ok)
            self.assertTrue(plot_path.exists())

    def test_plot_rows_stop_at_first_hit(self):
        rows = [{"t": 0.0, "hit": 0}, {"t": 1.0, "hit": 1}, {"t": 2.0, "hit": 0}]

        clipped = _rows_until_first_hit(rows)

        self.assertEqual([row["t"] for row in clipped], [0.0, 1.0])

    def test_start_geometry_offsets_from_horizontal_range(self):
        args = SimpleNamespace(
            start_horizontal_range_m=100.0,
            start_forward_offset_m=None,
            start_lateral_offset_m=-20.0,
        )

        forward, lateral, horizontal = _start_geometry_offsets(args)

        self.assertAlmostEqual(horizontal, 100.0)
        self.assertAlmostEqual(lateral, -20.0)
        self.assertAlmostEqual(forward, np.sqrt(100.0 * 100.0 - 20.0 * 20.0))

    def test_start_geometry_rejects_impossible_lateral_offset(self):
        args = SimpleNamespace(
            start_horizontal_range_m=10.0,
            start_forward_offset_m=None,
            start_lateral_offset_m=-20.0,
        )

        with self.assertRaises(SystemExit):
            _start_geometry_offsets(args)


if __name__ == "__main__":
    unittest.main()
