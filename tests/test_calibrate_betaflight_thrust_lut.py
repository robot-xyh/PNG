import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def _load_tool():
    path = ROOT / "tools" / "calibrate_betaflight_thrust_lut.py"
    spec = importlib.util.spec_from_file_location("calibrate_thrust_lut_for_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tool = _load_tool()


class CalibrateBetaflightThrustLutTest(unittest.TestCase):
    def test_synthetic_full_coverage_can_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            host_path = root / "host.csv"
            blackbox_path = root / "blackbox.csv"
            sample_count = 1500
            time_s = np.arange(sample_count, dtype=float) * 0.01
            phase = np.arange(sample_count) % 300 / 299.0
            throttle_us = 1175.0 + 350.0 * phase
            voltage_v = 25.3 - 5.5 * time_s / time_s[-1]
            force = (
                5.0
                + 0.045 * (throttle_us - 1175.0)
                + 0.35 * (voltage_v - 19.8)
            )
            command = 1000.0 + (throttle_us - 1050.0) * 1000.0 / 950.0
            acc_z = force / 9.80665 * 2048.0

            with host_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=["elapsed_s", "armed", "rc_in_ch4"],
                )
                writer.writeheader()
                for timestamp, throttle in zip(time_s, throttle_us):
                    writer.writerow(
                        {
                            "elapsed_s": timestamp,
                            "armed": 1,
                            "rc_in_ch4": throttle,
                        }
                    )
            with blackbox_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(tool.REQUIRED_BLACKBOX_FIELDS)
                for timestamp, throttle, battery, acceleration in zip(
                    time_s,
                    command,
                    voltage_v,
                    acc_z,
                ):
                    writer.writerow(
                        [timestamp * 1.0e6, throttle, battery, 0.0, 0.0, acceleration]
                    )

            result = tool.calibrate(
                host_csv=host_path,
                blackbox_csv=blackbox_path,
                blackbox_bfl=None,
                calibration_id="synthetic-full",
                host_throttle_field="rc_in_ch4",
                acc_1g_raw=2048.0,
                min_check_us=1050.0,
                max_pwm_us=2000.0,
                idle_command=1000.0,
                alignment_search_s=0.1,
                alignment_step_s=0.01,
                voltage_knot_count=5,
                throttle_knot_count=7,
                minimum_samples=500,
                required_voltage_v=(20.0, 25.2),
                required_throttle_us=(1200.0, 1500.0),
            )

            self.assertTrue(result["validation"]["passed"])
            self.assertLessEqual(
                result["validation"]["p95_relative_error"],
                0.20,
            )

    def test_narrow_voltage_evidence_is_marked_non_passing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            host_path = root / "host.csv"
            blackbox_path = root / "blackbox.csv"
            sample_count = 600
            time_s = np.arange(sample_count, dtype=float) * 0.01
            throttle_us = 1175.0 + 350.0 * (np.arange(sample_count) % 100) / 99.0
            voltage_v = np.linspace(24.1, 25.0, sample_count)
            force = 5.0 + 0.04 * (throttle_us - 1175.0) + voltage_v - 24.1
            command = 1000.0 + (throttle_us - 1050.0) * 1000.0 / 950.0
            acc_z = force / 9.80665 * 2048.0

            with host_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(["elapsed_s", "armed", "rc_in_ch4"])
                writer.writerows(zip(time_s, np.ones(sample_count, dtype=int), throttle_us))
            with blackbox_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(tool.REQUIRED_BLACKBOX_FIELDS)
                writer.writerows(
                    zip(
                        time_s * 1.0e6,
                        command,
                        voltage_v,
                        np.zeros(sample_count),
                        np.zeros(sample_count),
                        acc_z,
                    )
                )

            result = tool.calibrate(
                host_csv=host_path,
                blackbox_csv=blackbox_path,
                blackbox_bfl=None,
                calibration_id="synthetic-narrow",
                host_throttle_field="rc_in_ch4",
                acc_1g_raw=2048.0,
                min_check_us=1050.0,
                max_pwm_us=2000.0,
                idle_command=1000.0,
                alignment_search_s=0.1,
                alignment_step_s=0.01,
                voltage_knot_count=4,
                throttle_knot_count=5,
                minimum_samples=500,
                required_voltage_v=(20.0, 25.2),
                required_throttle_us=(1200.0, 1500.0),
            )

            self.assertFalse(result["validation"]["passed"])
            self.assertIn(
                "voltage_coverage_insufficient",
                result["validation"]["blockers"],
            )


if __name__ == "__main__":
    unittest.main()
