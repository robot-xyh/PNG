from __future__ import annotations

import unittest

from tools.validate_win10_gpu import evaluate_gpu_profile, parse_version, version_at_least
from tools.windows_experiments import parse_wsl_distribution_version


class Win10EnvironmentTest(unittest.TestCase):
    def test_driver_version_comparison_is_numeric(self):
        self.assertEqual(parse_version("560.76"), (560, 76))
        self.assertTrue(version_at_least("561.2", (560, 76)))
        self.assertFalse(version_at_least("560.7", (560, 76)))

    def test_rtx3080_profile_passes(self):
        errors, warnings = evaluate_gpu_profile(
            name="NVIDIA GeForce RTX 3080",
            driver="560.76",
            vram_bytes=10 * 1024**3,
            compute_capability=(8, 6),
        )
        self.assertEqual(errors, [])
        self.assertEqual(warnings, [])

    def test_compatible_non_rtx3080_is_warning_only(self):
        errors, warnings = evaluate_gpu_profile(
            name="NVIDIA RTX A5000",
            driver="570.10",
            vram_bytes=24 * 1024**3,
            compute_capability=(8, 6),
        )
        self.assertEqual(errors, [])
        self.assertEqual(len(warnings), 1)

    def test_old_driver_and_small_gpu_fail(self):
        errors, _ = evaluate_gpu_profile(
            name="NVIDIA GeForce RTX 2060",
            driver="552.44",
            vram_bytes=6 * 1024**3,
            compute_capability=(7, 5),
        )
        self.assertEqual(len(errors), 3)

    def test_wsl1_listing_parses_with_windows_nulls(self):
        listing = "  NAME\x00  STATE\x00  VERSION\x00\n* PNG-PX4-Ubuntu20.04\x00  Stopped\x00  1\x00\n"
        self.assertEqual(parse_wsl_distribution_version(listing, "PNG-PX4-Ubuntu20.04"), 1)

    def test_wsl2_and_missing_distribution_are_distinct(self):
        listing = "* PNG-PX4-Ubuntu20.04 Running 2\n  Ubuntu Stopped 1\n"
        self.assertEqual(parse_wsl_distribution_version(listing, "PNG-PX4-Ubuntu20.04"), 2)
        self.assertIsNone(parse_wsl_distribution_version(listing, "Missing"))


if __name__ == "__main__":
    unittest.main()
