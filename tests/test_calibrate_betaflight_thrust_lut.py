import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

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
            sample_count = 7000
            time_s = np.arange(sample_count, dtype=float) * 0.01
            phase = np.arange(sample_count) % 100 / 99.0
            throttle_us = 1175.0 + 350.0 * phase
            voltage_v = 25.3 - 3.4 * time_s / time_s[-1]
            force = (
                8.0
                + 0.035 * (throttle_us - 1175.0)
                + 0.20 * (voltage_v - 21.9)
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
                for index, (timestamp, throttle, battery, acceleration) in enumerate(
                    zip(time_s, command, voltage_v, acc_z)
                ):
                    gyro = 200.0 if index == 3050 else 0.0
                    motor = 1950.0 if index == 3051 else 500.0
                    if index == 3052:
                        acceleration = 3.0 * 2048.0
                    writer.writerow([
                        timestamp * 1.0e6,
                        throttle,
                        battery,
                        0.0,
                        0.0,
                        acceleration,
                        gyro,
                        0.0,
                        0.0,
                        motor,
                        500.0,
                        500.0,
                        500.0,
                    ])

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
                voltage_knot_count=3,
                throttle_knot_count=5,
                minimum_samples=500,
                required_voltage_v=(22.0, 25.2),
                required_throttle_us=(1200.0, 1500.0),
            )

            self.assertTrue(result["validation"]["passed"])
            self.assertLessEqual(
                result["validation"]["p95_relative_error"],
                0.20,
            )
            self.assertEqual(
                np.asarray(result["validation"]["three_by_five_sample_counts"]).shape,
                (3, 5),
            )
            self.assertGreater(
                result["dynamics"]["first_order_time_constant_s"],
                0.0,
            )
            self.assertEqual(
                result["fit"]["method"],
                "voltage_scaled_effective_input_quadratic_v1",
            )
            self.assertEqual(
                np.asarray(
                    result["validation"][
                        "holdout_three_by_five_sample_counts"
                    ]
                ).shape,
                (3, 5),
            )
            filters = result["validation"]["filter_counts"]
            self.assertGreater(filters["collision_or_force_outlier"], 0)
            self.assertGreater(filters["high_angular_rate"], 0)
            self.assertGreater(filters["motor_saturation"], 0)

    def test_narrow_voltage_evidence_is_marked_non_passing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            host_path = root / "host.csv"
            blackbox_path = root / "blackbox.csv"
            sample_count = 2000
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
                    [
                        timestamp * 1.0e6,
                        throttle,
                        battery,
                        0.0,
                        0.0,
                        acceleration,
                        0.0,
                        0.0,
                        0.0,
                        500.0,
                        500.0,
                        500.0,
                        500.0,
                    ]
                    for timestamp, throttle, battery, acceleration in zip(
                        time_s, command, voltage_v, acc_z
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
                required_voltage_v=(22.0, 25.2),
                required_throttle_us=(1200.0, 1500.0),
            )

            self.assertFalse(result["validation"]["passed"])
            self.assertIn(
                "voltage_extrapolation_exceeds_limit",
                result["validation"]["blockers"],
            )

    def test_manifest_combines_sources_and_groups_holdout_by_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            decoder_path = root / "blackbox_decode"
            decoder_path.write_text("test decoder", encoding="utf-8")
            sources = []
            source_arrays = {}
            sample_count = 5000
            time_s = np.arange(sample_count, dtype=float) * 0.01
            bucket = np.floor(time_s * 10.0).astype(int)
            throttle_nodes = np.asarray([1200.0, 1275.0, 1350.0, 1425.0, 1500.0])
            voltage_nodes = np.asarray([22.0, 23.6, 25.2])

            for source_index in range(2):
                source_id = f"SOURCE{source_index}"
                host_path = root / f"host_{source_index}.csv"
                bfl_path = root / f"LOG{source_index:05d}.BFL"
                bfl_path.write_bytes(f"bfl-{source_index}".encode("ascii"))
                throttle_us = throttle_nodes[(bucket + source_index) % len(throttle_nodes)]
                voltage_v = voltage_nodes[
                    ((bucket // len(throttle_nodes)) + source_index) % len(voltage_nodes)
                ]
                force = (
                    6.0
                    + 0.025 * (throttle_us - 1200.0)
                    + 0.25 * (voltage_v - 22.0)
                )
                command = 1000.0 + (throttle_us - 1050.0) * 1000.0 / 950.0
                source_arrays[bfl_path.stem] = (command, voltage_v, force)
                with host_path.open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(["elapsed_s", "armed", "rc_in_ch4"])
                    writer.writerows(
                        zip(time_s, np.ones(sample_count, dtype=int), throttle_us)
                    )
                sources.append(
                    {
                        "id": source_id,
                        "blackbox_bfl": str(bfl_path),
                        "host_csv": str(host_path),
                        "host_arm_interval_index": 0,
                    }
                )

            manifest_path = root / "manifest.json"
            manifest_path.write_text(
                json.dumps({"schema_version": 1, "sources": sources}),
                encoding="utf-8",
            )

            def write_decoded(_decoder, decoded_sources, output_root):
                for source in decoded_sources:
                    stem = Path(source["blackbox_bfl"]).stem
                    command, voltage_v, force = source_arrays[stem]
                    decoded_path = output_root / f"{stem}.01.csv"
                    with decoded_path.open("w", newline="", encoding="utf-8") as stream:
                        writer = csv.writer(stream)
                        writer.writerow(tool.REQUIRED_BLACKBOX_FIELDS)
                        for timestamp, throttle, battery, acceleration in zip(
                            time_s,
                            command,
                            voltage_v,
                            force / 9.80665 * 2048.0,
                        ):
                            writer.writerow(
                                [
                                    timestamp * 1.0e6,
                                    throttle,
                                    battery,
                                    0.0,
                                    0.0,
                                    acceleration,
                                    0.0,
                                    0.0,
                                    0.0,
                                    500.0,
                                    500.0,
                                    500.0,
                                    500.0,
                                ]
                            )

            with mock.patch.object(
                tool,
                "_decode_manifest_sources",
                side_effect=write_decoded,
            ):
                result = tool.calibrate_manifest(
                    manifest_path=manifest_path,
                    decoder_path=decoder_path,
                    calibration_id="synthetic-manifest",
                    host_throttle_field="rc_in_ch4",
                    acc_1g_raw=2048.0,
                    min_check_us=1050.0,
                    max_pwm_us=2000.0,
                    idle_command=1000.0,
                    alignment_search_s=0.1,
                    alignment_step_s=0.01,
                    voltage_knot_count=3,
                    throttle_knot_count=5,
                    minimum_samples=500,
                    required_voltage_v=(22.0, 25.2),
                    required_throttle_us=(1200.0, 1500.0),
                )

            self.assertTrue(result["validation"]["passed"])
            self.assertEqual(result["provenance"]["source_count"], 2)
            self.assertEqual(len(result["provenance"]["sources"]), 2)
            self.assertEqual(
                result["provenance"]["holdout_method"],
                "every_fifth_10hz_sample_within_each_source_and_throttle_band",
            )
            self.assertGreaterEqual(result["validation"]["sample_count"], 100)
            self.assertTrue(
                all(
                    source["filtered_sample_count"] > 0
                    for source in result["provenance"]["sources"]
                )
            )


if __name__ == "__main__":
    unittest.main()
