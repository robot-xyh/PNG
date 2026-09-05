import importlib.util
from pathlib import Path
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "audit_betaflight_thrust_coverage",
    ROOT / "tools" / "audit_betaflight_thrust_coverage.py",
)
audit_module = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(audit_module)


class BetaflightThrustCoverageAuditTest(unittest.TestCase):
    def test_resample_uses_per_bucket_medians(self):
        samples = np.asarray(
            [
                [0.001, 24.0, 1200.0, 1.0],
                [0.049, 24.2, 1220.0, 1.2],
                [0.101, 23.0, 1300.0, 1.4],
                [0.199, 23.2, 1320.0, 1.6],
            ]
        )

        result = audit_module._resample_medians(samples, 10.0)

        np.testing.assert_allclose(
            result,
            np.asarray(
                [
                    [0.025, 24.1, 1210.0, 1.1],
                    [0.150, 23.1, 1310.0, 1.5],
                ]
            ),
        )

    def test_summary_reports_empty_two_dimensional_cells(self):
        manifest = {
            "voltage_bin_edges_v": [20.0, 22.0, 25.2],
            "throttle_bin_edges_us": [1200.0, 1350.0, 1500.000001],
        }
        samples = np.asarray(
            [
                [0.0, 20.5, 1210.0, 1.0],
                [0.1, 24.5, 1400.0, 1.2],
            ]
        )

        result = audit_module._summarize_coverage(samples, manifest)

        self.assertEqual(result["two_dimensional_sample_counts"], [[1, 0], [0, 1]])
        self.assertEqual(result["two_dimensional_empty_cell_count"], 2)
        self.assertEqual(result["two_dimensional_insufficient_cell_count"], 2)

    def test_coverage_mask_applies_force_voltage_and_throttle_limits(self):
        samples = np.asarray(
            [
                [0.0, 20.0, 1200.0, 0.3],
                [0.1, 25.2, 1500.0, 3.0],
                [0.2, 19.9, 1300.0, 1.0],
                [0.3, 24.0, 1501.0, 1.0],
                [0.4, 24.0, 1300.0, 3.1],
            ]
        )

        result = audit_module._coverage_mask(
            samples,
            voltage_range=(20.0, 25.2),
            throttle_range=(1200.0, 1500.0),
            force_range_g=(0.3, 3.0),
        )

        self.assertEqual(result.tolist(), [True, True, False, False, False])


if __name__ == "__main__":
    unittest.main()
