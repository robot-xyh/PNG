import csv
import json
import tempfile
import unittest
from pathlib import Path

from tools.windows_experiments import expand_cases
from tools.windows_report import evaluate_case, wilson_interval


class WindowsExperimentToolsTest(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).resolve().parents[1]
        self.config = json.loads((root / "config" / "windows_scenarios.json").read_text(encoding="utf-8"))

    def test_standard_case_counts(self):
        self.assertEqual(len(expand_cases(self.config, "standard", "fast")), 120)
        self.assertEqual(len(expand_cases(self.config, "standard", "sitl")), 24)
        self.assertEqual(len(expand_cases(self.config, "standard", "all")), 144)

    def test_wilson_interval_contains_observed_rate(self):
        low, high = wilson_interval(6, 10)
        self.assertLess(low, 0.6)
        self.assertGreater(high, 0.6)

    def _write_case(self, directory: Path, *, near_hit: bool, collision: bool, detector_fps: float = 25.0):
        csv_path = directory / "trajectory.csv"
        fields = [
            "t", "range", "detected", "valid", "detector_fps", "wall_fps", "sim_clock_ratio",
            "deadline_miss", "near_hit", "collision_accepted", "n_cmd_g", "load_factor_fd_g",
            "yolo_cuda_available", "yolo_half_effective", "interceptor_x", "interceptor_y", "interceptor_z",
            "intruder_x", "intruder_y", "intruder_z", "lambda_x", "lambda_y", "lambda_z"
        ]
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index in range(24):
                writer.writerow(
                    {
                        "t": index * 0.05,
                        "range": 10.0 - index * 0.2,
                        "detected": 1,
                        "valid": 1,
                        "detector_fps": detector_fps,
                        "wall_fps": min(20.0, detector_fps),
                        "sim_clock_ratio": 1.0,
                        "deadline_miss": 0,
                        "near_hit": int(near_hit and index == 20),
                        "collision_accepted": int(collision and index == 21),
                        "n_cmd_g": 1.0,
                        "load_factor_fd_g": 0.8,
                        "yolo_cuda_available": 1,
                        "yolo_half_effective": 1,
                        "interceptor_x": 0,
                        "interceptor_y": 0,
                        "interceptor_z": 0,
                        "intruder_x": 10,
                        "intruder_y": 0,
                        "intruder_z": 0,
                        "lambda_x": 1,
                        "lambda_y": 0,
                        "lambda_z": 0,
                    }
                )
        log_path = directory / "runner.log"
        log_path.write_text("run complete\n", encoding="utf-8")
        return csv_path, log_path

    def test_near_hit_is_not_collision_success(self):
        with tempfile.TemporaryDirectory() as tempdir:
            root = Path(tempdir)
            csv_path, log_path = self._write_case(root, near_hit=True, collision=False)
            result = evaluate_case(
                csv_path,
                case_info={"case_key": "fast_M01_TTC_r01"},
                thresholds={
                    "min_frames": 20,
                    "min_detector_fps": 18,
                    "min_loop_fps": 15,
                    "min_sim_clock_ratio": 0.75,
                    "max_deadline_miss_ratio": 0.35,
                },
                return_code=0,
                timed_out=False,
                log_path=log_path,
                simulator_alive=True,
            )
            self.assertTrue(result["infrastructure_valid"])
            self.assertTrue(result["near_hit"])
            self.assertFalse(result["collision_hit"])
            self.assertAlmostEqual(result["truth_los_error_p95_deg"], 0.0)

    def test_slow_detector_marks_infrastructure_invalid(self):
        with tempfile.TemporaryDirectory() as tempdir:
            root = Path(tempdir)
            csv_path, log_path = self._write_case(root, near_hit=False, collision=True, detector_fps=8.0)
            result = evaluate_case(
                csv_path,
                case_info={"case_key": "fast_M01_TTC_r01"},
                thresholds={
                    "min_frames": 20,
                    "min_detector_fps": 18,
                    "min_loop_fps": 15,
                    "min_sim_clock_ratio": 0.75,
                    "max_deadline_miss_ratio": 0.35,
                },
                return_code=0,
                timed_out=False,
                log_path=log_path,
                simulator_alive=True,
            )
            self.assertFalse(result["infrastructure_valid"])
            self.assertIn("detector_fps_below_gate", result["infra_invalid_reasons"])
            self.assertIn("loop_fps_below_gate", result["infra_invalid_reasons"])


if __name__ == "__main__":
    unittest.main()
