import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _load_tool_module():
    path = ROOT / "tools" / "analyze_betaflight_noprop_log.py"
    spec = importlib.util.spec_from_file_location("analyze_betaflight_noprop_log_for_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tool = _load_tool_module()


class BetaflightLogAuditTest(unittest.TestCase):
    def test_safe_noprop_log_passes(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = self._write_log(Path(directory), self._safe_row())

            result = tool.analyze_log(csv_path)

            self.assertTrue(result["passed"])
            self.assertEqual(result["violations"], [])
            self.assertEqual(result["metrics"]["algorithm_rows"], 1)

    def test_unsafe_noprop_log_reports_envelope_and_transport_failures(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                rc_sent_ch1="885",
                rc_sent_ch3="1200",
                map_limited_roll_rate_deg_s="4.0",
                msp_worker_override_active="0",
                msp_last_publish_override_active="0",
                msp_send_success_max_interval_s="0.10",
                msp_worker_send_error_count="1",
                msp_cmd_set_raw_rc_error_count="1",
            )
            csv_path = self._write_log(Path(directory), row)

            result = tool.analyze_log(csv_path)
            codes = {item["code"] for item in result["violations"]}

            self.assertFalse(result["passed"])
            self.assertTrue(
                {
                    "invalid_sent_rc",
                    "sent_885_us",
                    "algorithm_throttle_envelope",
                    "algorithm_without_worker_gates",
                    "roll_rate_limit",
                    "set_raw_rc_gap",
                    "set_raw_rc_errors",
                }.issubset(codes)
            )

    def test_enabled_web_requires_published_error_free_telemetry(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(web_publish_count="0", web_error_count="2", web_last_error="preview failed")
            csv_path = self._write_log(Path(directory), row, web_enabled=True)

            result = tool.analyze_log(csv_path)
            codes = {item["code"] for item in result["violations"]}

            self.assertIn("web_no_telemetry_published", codes)
            self.assertIn("web_runtime_errors", codes)

    def test_schema_v4_rejects_guidance_hold_outside_worker_gap(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                sp_source="guidance_hold",
                detector_reject_reason="area_not_expanding",
                perception_new_result="1",
                watchdog_ok="1",
            )
            csv_path = self._write_log(Path(directory), row, schema_version=4)

            result = tool.analyze_log(csv_path)

            self.assertFalse(result["passed"])
            self.assertIn(
                "guidance_hold_outside_perception_gap",
                {item["code"] for item in result["violations"]},
            )

    def test_schema_v6_allows_hold_while_waiting_for_attitude(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                sp_source="guidance_hold",
                detector_reject_reason="fusion_waiting_for_attitude",
                perception_new_result="0",
                watchdog_ok="1",
            )
            csv_path = self._write_log(Path(directory), row, schema_version=6)

            result = tool.analyze_log(csv_path)

            self.assertTrue(result["passed"])

    def test_schema_v3_cannot_prove_publish_time_gates(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = self._write_log(Path(directory), self._safe_row(), schema_version=3)

            result = tool.analyze_log(csv_path)

            self.assertFalse(result["passed"])
            self.assertIn(
                "publish_gate_timebase_unavailable",
                {item["code"] for item in result["violations"]},
            )

    @staticmethod
    def _safe_row():
        return {
            "elapsed_s": "1.0",
            "msp_publish_mode": "algorithm",
            "rc_sent_ch1": "1500",
            "rc_sent_ch2": "1500",
            "rc_sent_ch3": "1078",
            "rc_sent_ch4": "1500",
            "msp_output_enabled": "1",
            "msp_algorithm_authorized": "1",
            "msp_worker_override_active": "1",
            "msp_prefill_ready": "1",
            "physical_rc_fresh": "1",
            "msp_last_publish_output_enabled": "1",
            "msp_last_publish_algorithm_authorized": "1",
            "msp_last_publish_override_active": "1",
            "msp_last_publish_prefill_ready": "1",
            "msp_last_publish_physical_rc_fresh": "1",
            "msp_last_publish_command_fresh": "1",
            "map_limited_roll_rate_deg_s": "3.0",
            "map_limited_pitch_rate_deg_s": "-3.0",
            "map_limited_yaw_rate_deg_s": "0.0",
            "msp_send_success_max_interval_s": "0.02",
            "msp_worker_send_error_count": "0",
            "msp_cmd_set_raw_rc_error_count": "0",
            "msp_set_raw_rc_success_count": "50",
            "msp_publish_deadline_miss_count": "0",
            "msp_cmd_set_raw_rc_max_rtt_ms": "2.0",
            "msp_cmd_raw_imu_max_rtt_ms": "2.5",
            "rknn_total_ms": "8.0",
            "host_thermal_max_c": "65.0",
            "gyro_roll_deg_s": "1.0",
            "gyro_pitch_deg_s": "-2.0",
            "gyro_yaw_deg_s": "0.5",
        }

    @staticmethod
    def _write_log(directory: Path, row, *, web_enabled=False, schema_version=4):
        csv_path = directory / "bench.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(row))
            writer.writeheader()
            writer.writerow(row)
        meta = {
            "config": {
                "msp_runtime": {
                    "control_publish_hz": 50.0,
                    "prefill_valid_min_us": 900,
                    "prefill_valid_max_us": 2100,
                    "throttle_channel_zero_based": 2,
                },
                "rc_mapping": {
                    "roll_command_limit_deg_s": 3.0,
                    "pitch_command_limit_deg_s": 3.0,
                    "yaw_command_limit_deg_s": 0.0,
                    "throttle_max_us": 1100,
                },
                "telemetry_web": {"enabled": web_enabled},
            },
            "log_schema_version": schema_version,
            "log_events_jsonl": "",
        }
        csv_path.with_name("bench_meta.json").write_text(json.dumps(meta), encoding="utf-8")
        return csv_path


if __name__ == "__main__":
    unittest.main()
