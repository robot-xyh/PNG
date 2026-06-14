import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from examples.run_airsim_strapdown_vision_png import (
    _output_paths_strapdown,
    _plot_strapdown,
    _rows_until_first_hit,
    _terminal_trigger_strapdown,
    _yaw_rate_from_pixel_error,
)
from vision_guidance.types import CameraIntrinsics


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

    def test_terminal_trigger_strapdown_conditions(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        args = SimpleNamespace(terminal_bbox_area_ratio=0.25)

        self.assertEqual(_terminal_trigger_strapdown("bbox_clipped", True, 1.0, intrinsics, args), "bbox_clipped")
        self.assertEqual(_terminal_trigger_strapdown("", True, 90000.0, intrinsics, args), "bbox_area_large")
        self.assertEqual(_terminal_trigger_strapdown("", True, 1000.0, intrinsics, args), "")

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


if __name__ == "__main__":
    unittest.main()
