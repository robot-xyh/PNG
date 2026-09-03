import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

from tools.analyze_betaflight_virtual_bbox_load import analyze_log
from tools.generate_betaflight_virtual_bbox_sequence import generate_rows, motion_at


class BetaflightVirtualBboxGeneratorTest(unittest.TestCase):
    def test_sequence_is_continuous_bounded_and_covers_all_motion_phases(self):
        rows = generate_rows(fps=10.0, duration_s=60.0)

        self.assertEqual(len(rows), 600)
        phases = {row["motion_phase"] for row in rows}
        self.assertEqual(
            phases,
            {
                "center_start",
                "horizontal_crossing",
                "vertical_crossing",
                "diagonal_crossing",
                "high_rate_stress",
                "center_end",
            },
        )
        self.assertTrue(all(float(row["x1"]) >= 0.0 for row in rows))
        self.assertTrue(all(float(row["x2"]) <= 640.0 for row in rows))
        self.assertTrue(all(float(row["y1"]) >= 0.0 for row in rows))
        self.assertTrue(all(float(row["y2"]) <= 512.0 for row in rows))
        self.assertEqual({row["track_id"] for row in rows}, {1})

    def test_motion_crosses_both_image_axes(self):
        horizontal_right = motion_at(9.0)
        horizontal_left = motion_at(11.0)
        vertical_down = motion_at(21.0)
        vertical_up = motion_at(23.0)

        self.assertGreater(horizontal_right[0], 320.0)
        self.assertLess(horizontal_left[0], 320.0)
        self.assertGreater(vertical_down[1], 256.0)
        self.assertLess(vertical_up[1], 256.0)


class BetaflightVirtualBboxLoadAuditTest(unittest.TestCase):
    def test_safe_log_reports_staged_theoretical_load(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = self._write_log(Path(directory))

            report = analyze_log(csv_path)

        self.assertTrue(report["passed"])
        self.assertEqual(report["violations"], [])
        self.assertEqual(
            [item["acceleration_limit_m_s2"] for item in report["staged_load_profiles"]],
            [1.0, 3.0, 5.0, 7.0],
        )
        self.assertGreater(report["actual_command"]["load_factor_g"]["max"], 1.6)
        self.assertEqual(report["safety"]["set_raw_rc_count_maxima"]["msp_set_raw_rc_attempt_count"], 0.0)
        self.assertGreater(
            report["actual_command"]["bbox_to_command_correlation"][
                "horizontal_offset_vs_desired_roll"
            ],
            0.99,
        )
        self.assertLess(
            report["actual_command"]["bbox_to_command_correlation"][
                "vertical_offset_vs_desired_pitch"
            ],
            -0.99,
        )

    def test_nonzero_rc_output_fails_audit(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = self._write_log(Path(directory), set_raw_rc_attempt_count=1)

            report = analyze_log(csv_path)

        self.assertFalse(report["passed"])
        self.assertIn("set_raw_rc_nonzero", {item["code"] for item in report["violations"]})

    def test_requested_load_cannot_exceed_bound_config(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = self._write_log(Path(directory))

            with self.assertRaisesRegex(RuntimeError, "exceeds configured total limit"):
                analyze_log(csv_path, load_levels_m_s2=(1.0, 8.0))

    def _write_log(self, directory: Path, *, set_raw_rc_attempt_count: int = 0) -> Path:
        csv_path = directory / "virtual_bbox.csv"
        fields = [
            "elapsed_s",
            "safety_state",
            "msp_publish_mode",
            "control_requested",
            "allow_control",
            "intercept_valid",
            "intercept_total_accel_n",
            "intercept_total_accel_e",
            "intercept_total_accel_d",
            "bbox_x1",
            "bbox_y1",
            "bbox_x2",
            "bbox_y2",
            "command_desired_roll_angle_deg",
            "command_desired_pitch_angle_deg",
            "pre_shape_sp_roll_rate_deg_s",
            "pre_shape_sp_pitch_rate_deg_s",
            "msp_set_raw_rc_attempt_count",
            "msp_set_raw_rc_success_count",
            "msp_set_raw_rc_write_attempt_count",
            "msp_set_raw_rc_write_success_count",
            "msp_set_raw_rc_write_error_count",
            "msp_set_raw_rc_ack_count",
        ]
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index in range(40):
                x_offset = -180.0 + 360.0 * index / 39.0
                y_offset = 140.0 * math.sin(2.0 * math.pi * index / 20.0)
                accel_n = 2.0 * y_offset / 140.0
                accel_e = 2.0 * x_offset / 180.0
                accel_d = -math.sqrt(max(0.0, 49.0 - accel_n * accel_n - accel_e * accel_e))
                writer.writerow(
                    {
                        "elapsed_s": index / 10.0,
                        "safety_state": "LOG_ONLY",
                        "msp_publish_mode": "disabled",
                        "control_requested": 0,
                        "allow_control": 0,
                        "intercept_valid": 1,
                        "intercept_total_accel_n": accel_n,
                        "intercept_total_accel_e": accel_e,
                        "intercept_total_accel_d": accel_d,
                        "bbox_x1": 321.0 + x_offset - 48.0,
                        "bbox_y1": 247.0 + y_offset - 36.0,
                        "bbox_x2": 321.0 + x_offset + 48.0,
                        "bbox_y2": 247.0 + y_offset + 36.0,
                        "command_desired_roll_angle_deg": x_offset / 20.0,
                        "command_desired_pitch_angle_deg": -y_offset / 20.0,
                        "pre_shape_sp_roll_rate_deg_s": 4.0 * x_offset / 20.0,
                        "pre_shape_sp_pitch_rate_deg_s": -4.0 * (-y_offset / 20.0),
                        "msp_set_raw_rc_attempt_count": set_raw_rc_attempt_count,
                        "msp_set_raw_rc_success_count": 0,
                        "msp_set_raw_rc_write_attempt_count": 0,
                        "msp_set_raw_rc_write_success_count": 0,
                        "msp_set_raw_rc_write_error_count": 0,
                        "msp_set_raw_rc_ack_count": 0,
                    }
                )
        meta = {
            "config": {
                "candidate_profile": {"id": "velocity_png_flight_log_only"},
                "runtime_policy": {
                    "allowed_control_modes": ["log_only"],
                    "msp_set_raw_rc_permitted": False,
                },
                "control_authorization": {"enabled": False},
                "camera": {"width": 640, "height": 512, "cx": 321.0, "cy": 247.0},
                "guidance": {
                    "law": "velocity_establishing_png",
                    "velocity_source": "msp_kinematics",
                    "velocity_establishing_png": {
                        "gravity_m_s2": 9.80665,
                        "total_accel_limit_m_s2": 7.0,
                    },
                },
                "guidance_command": {
                    "mapping_type": "accel_tilt_rate",
                    "accel_tilt_rate": {
                        "gravity_mps2": 9.80665,
                        "roll_rate_sign": 1.0,
                        "pitch_rate_sign": -1.0,
                    },
                },
                "rc_mapping": {
                    "roll_command_limit_deg_s": 60.0,
                    "pitch_command_limit_deg_s": 60.0,
                },
            }
        }
        csv_path.with_name("virtual_bbox_meta.json").write_text(
            json.dumps(meta), encoding="utf-8"
        )
        return csv_path


if __name__ == "__main__":
    unittest.main()
