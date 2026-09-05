import importlib.util
from pathlib import Path
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "audit_betaflight_historical_blackbox",
    ROOT / "tools" / "audit_betaflight_historical_blackbox.py",
)
audit_module = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(audit_module)


class BetaflightHistoricalBlackboxAuditTest(unittest.TestCase):
    def test_classification_requires_duration_and_two_dynamic_signals(self):
        base = {
            "powered_duration_s": 8.0,
            "internal_throttle_max": 1400.0,
            "powered_specific_force_g_p50_p95_max": [1.0, 1.3, 1.5],
            "powered_gyro_norm_raw_p95_max": [25.0, 40.0],
            "powered_baro_altitude_range_m": 0.1,
            "gps_distinct_update_count": 0,
        }

        self.assertEqual(
            audit_module._classify_physics(base),
            "flight_or_unrestrained_dynamic",
        )
        self.assertEqual(
            audit_module._classify_physics(
                {**base, "powered_duration_s": 0.5}
            ),
            "no_material_powered_excitation",
        )
        self.assertEqual(
            audit_module._classify_physics(
                {
                    **base,
                    "powered_specific_force_g_p50_p95_max": [1.0, 1.05, 1.1],
                }
            ),
            "brief_or_fixed_powered_test",
        )

    def test_masked_duration_caps_decode_gaps(self):
        time_s = np.asarray([0.0, 0.01, 0.02, 5.0, 5.01])
        mask = np.asarray([True, True, True, True, False])

        result = audit_module._masked_duration(time_s, mask)

        self.assertAlmostEqual(result, 0.07)

    def test_summary_keeps_internal_throttle_bins_explicit(self):
        samples = np.asarray(
            [
                [0.0, 24.0, 1225.0, 1.0],
                [0.1, 23.0, 1375.0, 1.2],
                [0.2, 22.0, 1499.0, 1.4],
            ]
        )

        result = audit_module._summarize_samples([samples])

        self.assertEqual(result["sample_count_10hz"], 3)
        self.assertEqual(result["throttle_bin_counts"], [1, 0, 0, 1, 0, 1])


if __name__ == "__main__":
    unittest.main()
