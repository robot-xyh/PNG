import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def _load_tool_module():
    path = ROOT / "tools" / "analyze_betaflight_blackbox_alignment.py"
    spec = importlib.util.spec_from_file_location(
        "analyze_betaflight_blackbox_alignment_for_test", path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tool = _load_tool_module()


class BetaflightBlackboxAlignmentTest(unittest.TestCase):
    def test_motor_fit_recovers_time_and_affine_conversion(self):
        blackbox_time_s = np.linspace(0.0, 8.0, 8001)
        blackbox_motors = np.column_stack(
            [
                200.0 + 80.0 * np.sin(blackbox_time_s * (0.7 + channel * 0.1) + channel)
                for channel in range(4)
            ]
        )
        sample_indexes = np.arange(100, 7900, 37)
        offset_s = 7.12345
        host_samples = np.column_stack(
            [
                blackbox_time_s[sample_indexes] + offset_s,
                blackbox_motors[sample_indexes] * 0.5 + 977.0,
            ]
        )

        result = tool._fit_motor_alignment(
            host_samples,
            blackbox_time_s,
            blackbox_motors,
            initial_offset_s=7.1,
        )

        self.assertAlmostEqual(result["host_minus_blackbox_s"], offset_s, places=4)
        self.assertAlmostEqual(result["motor_scale_us_per_raw"], 0.5, places=4)
        self.assertAlmostEqual(result["motor_offset_us"], 977.0, places=3)
        self.assertLess(result["motor_fit_rmse_us"], 0.01)

    def test_converted_gyro_csv_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "converted.csv"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(["time (us)", "gyroADC[0] (deg/s)"])
                writer.writerow([0, 0])

            with self.assertRaisesRegex(RuntimeError, "--unit-rotation raw"):
                tool._read_blackbox_numeric(path)


if __name__ == "__main__":
    unittest.main()
