import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def _load_tool_module():
    path = ROOT / "tools" / "analyze_betaflight_blackbox_flight.py"
    spec = importlib.util.spec_from_file_location(
        "analyze_betaflight_blackbox_flight_for_test", path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tool = _load_tool_module()


class BetaflightBlackboxFlightTest(unittest.TestCase):
    def test_manual_flight_selects_matching_arm_interval_and_aligns_throttle(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            blackbox_path = root / "flight.csv"
            host_path = root / "host.csv"
            blackbox_time_s = np.arange(0.0, 6.0, 0.02)
            throttle = np.full(len(blackbox_time_s), 1000.0)
            ramp = (blackbox_time_s >= 1.0) & (blackbox_time_s < 2.0)
            throttle[ramp] = 1001.0 + (blackbox_time_s[ramp] - 1.0) * 249.0
            throttle[(blackbox_time_s >= 2.0) & (blackbox_time_s <= 5.5)] = 1250.0
            self._write_blackbox(blackbox_path, blackbox_time_s, throttle)
            self._write_host(host_path, blackbox_time_s, throttle, offset_s=10.15)

            result = tool.analyze(
                host_path,
                blackbox_path,
                endpoint_window_s=0.5,
                alignment_step_s=0.005,
                motor_scale_us_per_raw=0.5,
                motor_offset_us=977.0,
            )

            self.assertEqual(result["selection"]["selected_host_armed_interval_index"], 1)
            self.assertAlmostEqual(
                result["selection"]["throttle_alignment"]["host_minus_blackbox_s"],
                10.15,
                places=2,
            )
            self.assertGreater(
                result["selection"]["throttle_alignment"]["correlation"], 0.99
            )
            self.assertIn("steady_hover", result["segments"])
            self.assertAlmostEqual(result["segments"]["steady_hover"]["duration_s"], 3.2, places=1)
            self.assertEqual(result["endpoint_transients"]["maximum_motor_raw"], 2047.0)
            self.assertGreater(
                result["endpoint_transients"]["motor_saturation_duration_s"], 0.0
            )
            self.assertEqual(result["control_evidence"]["set_raw_rc_write_success_max"], 0.0)
            self.assertEqual(result["control_evidence"]["override_active_rows"], 0)
            self.assertEqual(result["motor_conversion"]["steady_motor_p50_us"][0], 1227.0)

    def test_idle_flight_uses_arm_start_fallback(self):
        rows = [
            {"elapsed_s": str(index * 0.1), "armed": "1", "rc_in_ch4": "1000"}
            for index in range(30)
        ]
        blackbox_time_s = np.arange(0.0, 2.9, 0.1)
        result = tool._fit_throttle_alignment(
            rows,
            host_throttle_field="rc_in_ch4",
            blackbox_time_s=blackbox_time_s,
            blackbox_throttle=np.full(len(blackbox_time_s), 1000.0),
            armed_interval=(0.0, 2.9),
            min_check_us=1050.0,
            max_pwm_us=2000.0,
            idle_command=1000.0,
            search_s=1.0,
            step_s=0.01,
        )

        self.assertEqual(result["method"], "arm_start_fallback")
        self.assertEqual(result["host_minus_blackbox_s"], 0.0)

    @staticmethod
    def _write_blackbox(path: Path, time_s: np.ndarray, throttle: np.ndarray) -> None:
        fields = list(tool.BLACKBOX_NUMERIC_FIELDS) + list(tool.BLACKBOX_CATEGORY_FIELDS)
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index, (timestamp, command) in enumerate(zip(time_s, throttle)):
                endpoint = timestamp >= 5.0
                row = {field: "0" for field in fields}
                row.update(
                    {
                        "time (us)": f"{timestamp * 1.0e6:.0f}",
                        "rcCommand[3]": f"{command:.3f}",
                        "vbatLatest (V)": "22.5" if endpoint else "24.5",
                        "amperageLatest (A)": "30.0" if endpoint else "5.0",
                        "rssi": "800",
                        "motor[0]": "2047" if endpoint else "500",
                        "motor[1]": "520",
                        "motor[2]": "510",
                        "motor[3]": "530",
                        "energyCumulative (mAh)": str(index // 20),
                        "rxSignalReceived": "1",
                        "rxFlightChannelsValid": "1",
                        "GPS_numSat": "0",
                        "flightModeFlags (flags)": "ANGLE_MODE",
                        "stateFlags (flags)": "0",
                        "failsafePhase (flags)": "IDLE",
                    }
                )
                writer.writerow(row)

    @staticmethod
    def _write_host(
        path: Path,
        blackbox_time_s: np.ndarray,
        throttle: np.ndarray,
        *,
        offset_s: float,
    ) -> None:
        fields = (
            "elapsed_s",
            "armed",
            "rc_in_ch4",
            "msp_analog_age_s",
            "vbat_v",
            "amperage_a",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "msp_set_raw_rc_write_success_count",
            "msp_override_active",
        )
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for timestamp in np.arange(0.0, 17.0, 0.02):
                first_arm = 1.0 <= timestamp <= 3.0
                second_arm = 10.0 <= timestamp <= 16.2
                blackbox_t = timestamp - offset_s
                command = float(
                    np.interp(blackbox_t, blackbox_time_s, throttle)
                    if 0.0 <= blackbox_t <= blackbox_time_s[-1]
                    else 1000.0
                )
                pwm = (
                    1000.0
                    if command <= 1000.0
                    else 1050.0 + (command - 1000.0) * 950.0 / 1000.0
                )
                writer.writerow(
                    {
                        "elapsed_s": f"{timestamp:.3f}",
                        "armed": int(first_arm or second_arm),
                        "rc_in_ch4": f"{pwm:.3f}",
                        "msp_analog_age_s": "0.0",
                        "vbat_v": "24.5",
                        "amperage_a": "5.0",
                        "roll_deg": "1.0",
                        "pitch_deg": "-1.0",
                        "yaw_deg": "10.0",
                        "msp_set_raw_rc_write_success_count": "0",
                        "msp_override_active": "0",
                    }
                )


if __name__ == "__main__":
    unittest.main()
