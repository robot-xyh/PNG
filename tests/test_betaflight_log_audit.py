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
            self.assertNotIn("unsupported_log_schema_version:10", result["warnings"])

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
                msp_set_raw_rc_write_max_interval_s="0.10",
                msp_worker_send_error_count="1",
                msp_cmd_set_raw_rc_error_count="1",
                msp_set_raw_rc_write_error_count="1",
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

    def test_armed_motor_output_and_spread_exceed_noprop_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                armed="1",
                motor_output_count="4",
                motor_output_ch1="1363",
                motor_output_ch2="1456",
                motor_output_ch3="1056",
                motor_output_ch4="1431",
            )
            csv_path = self._write_log(Path(directory), row)

            result = tool.analyze_log(csv_path)
            codes = {item["code"] for item in result["violations"]}

            self.assertIn("armed_motor_output_high", codes)
            self.assertIn("armed_motor_spread_high", codes)
            self.assertEqual(result["metrics"]["max_armed_motor_output"], 1456.0)
            self.assertEqual(result["metrics"]["max_armed_motor_spread"], 400.0)
            output_violation = next(
                item for item in result["violations"] if item["code"] == "armed_motor_output_high"
            )
            self.assertEqual(output_violation["first_elapsed_s"], 1.0)
            self.assertEqual(output_violation["limit"], 1200)

    def test_motor_output_limits_ignore_disarmed_telemetry(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                armed="0",
                msp_publish_mode="passthrough",
                motor_output_count="4",
                motor_output_ch1="1363",
                motor_output_ch2="1456",
                motor_output_ch3="1056",
                motor_output_ch4="1431",
            )
            csv_path = self._write_log(Path(directory), row)

            result = tool.analyze_log(csv_path)
            codes = {item["code"] for item in result["violations"]}

            self.assertNotIn("armed_motor_output_high", codes)
            self.assertNotIn("armed_motor_spread_high", codes)

    def test_schema_v13_rejects_algorithm_output_without_motor_interlock(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                motor_interlock_ok="0",
                motor_interlock_reason="motor_output_high",
                motor_interlock_latched="1",
            )
            csv_path = self._write_log(Path(directory), row, schema_version=13)

            result = tool.analyze_log(csv_path)
            violation = next(
                item
                for item in result["violations"]
                if item["code"] == "algorithm_without_motor_interlock"
            )

            self.assertEqual(violation["count"], 1)
            self.assertEqual(violation["first_elapsed_s"], 1.0)

    def test_schema_v14_rejects_active_output_without_takeover_duration_interlock(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                safety_state="ACTIVE",
                takeover_duration_interlock_ok="0",
                takeover_duration_interlock_reason="takeover_duration_exceeded",
                takeover_duration_interlock_latched="1",
                takeover_duration_s="3.01",
            )
            csv_path = self._write_log(Path(directory), row, schema_version=14)

            result = tool.analyze_log(csv_path)
            violation = next(
                item
                for item in result["violations"]
                if item["code"] == "active_without_takeover_duration_interlock"
            )

            self.assertEqual(violation["count"], 1)
            self.assertEqual(violation["first_elapsed_s"], 1.0)

    def test_transport_gap_reports_first_cumulative_crossing_time(self):
        with tempfile.TemporaryDirectory() as directory:
            first = self._safe_row()
            second = dict(first)
            first.update(elapsed_s="1.0", msp_set_raw_rc_write_max_interval_s="0.03")
            second.update(elapsed_s="2.0", msp_set_raw_rc_write_max_interval_s="0.081534")
            csv_path = self._write_log(Path(directory), [first, second])

            result = tool.analyze_log(csv_path)
            violation = next(item for item in result["violations"] if item["code"] == "set_raw_rc_gap")

            self.assertEqual(violation["first_elapsed_s"], 2.0)
            self.assertEqual(violation["observed"], 0.081534)

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

    def test_schema_v7_rejects_ack_stall_low_write_rate_and_parser_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                msp_set_raw_rc_write_rate_hz="45.0",
                msp_set_raw_rc_write_p999_interval_s="0.05",
                msp_set_raw_rc_ack_age_s="0.30",
                msp_last_publish_set_raw_rc_ack_fresh="0",
                msp_rx_checksum_error_count="1",
            )
            csv_path = self._write_log(Path(directory), row, schema_version=7)

            result = tool.analyze_log(csv_path)
            codes = {item["code"] for item in result["violations"]}

            self.assertTrue(
                {
                    "algorithm_without_worker_gates",
                    "set_raw_rc_write_rate_low",
                    "set_raw_rc_write_p999_gap",
                    "algorithm_with_stale_set_ack",
                    "set_raw_rc_ack_stall",
                    "msp_response_parser_errors",
                }.issubset(codes)
            )

    def test_schema_v8_rejects_invalid_shaping_and_nonleveling_hardcap(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                entry_handoff_progress="1.2",
                tilt_roll_attitude_deg="40.0",
                sp_roll_rate_deg_s="1.0",
                tilt_hardcap_active="0",
            )
            csv_path = self._write_log(Path(directory), row, schema_version=8)

            result = tool.analyze_log(csv_path)
            codes = {item["code"] for item in result["violations"]}

            self.assertTrue(
                {
                    "command_shaping_factor_out_of_range",
                    "tilt_hardcap_not_leveling",
                    "tilt_hardcap_flag_missing",
                }.issubset(codes)
            )

    def test_schema_v8_rejects_nonfinite_shaping_and_invalid_algorithm_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(sp_pitch_rate_deg_s="nan", shaping_valid="0")
            csv_path = self._write_log(Path(directory), row, schema_version=8)

            result = tool.analyze_log(csv_path)
            codes = {item["code"] for item in result["violations"]}

            self.assertIn("command_shaping_nonfinite", codes)
            self.assertIn("algorithm_with_invalid_command_shaping", codes)

    def test_schema_v11_requires_explicit_frames_and_body_guidance_vector(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                guidance_eval_frame="inertial_ned",
                rate_gain_input_frame="inertial_ned",
                guidance_valid="1",
                g_eval_body_frd_x="",
            )
            csv_path = self._write_log(Path(directory), row, schema_version=11)

            result = tool.analyze_log(csv_path)
            codes = {item["code"] for item in result["violations"]}

            self.assertNotIn("unsupported_log_schema_version:11", result["warnings"])
            self.assertIn("invalid_guidance_command_frames", codes)
            self.assertIn("guidance_body_vector_missing", codes)

    def test_schema_v12_uses_last_published_command_timebase(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                shaping_valid="0",
                sp_roll_rate_deg_s="",
                sp_pitch_rate_deg_s="",
                msp_last_publish_command_active="1",
                msp_last_publish_command_reason="active",
            )
            csv_path = self._write_log(Path(directory), row, schema_version=12)

            result = tool.analyze_log(csv_path)
            codes = {item["code"] for item in result["violations"]}

            self.assertNotIn("command_shaping_nonfinite", codes)
            self.assertNotIn("algorithm_with_invalid_command_shaping", codes)

            row["msp_last_publish_command_active"] = "0"
            csv_path = self._write_log(Path(directory), row, schema_version=12)
            result = tool.analyze_log(csv_path)
            self.assertIn(
                "algorithm_with_invalid_command_shaping",
                {item["code"] for item in result["violations"]},
            )

    def test_schema_v14_allows_gate_closed_async_publish_transition(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                safety_state="FAILSAFE",
                shaping_reason="gate_closed",
                tilt_roll_attitude_deg="",
                tilt_pitch_attitude_deg="",
                map_limited_roll_rate_deg_s="",
                map_limited_pitch_rate_deg_s="",
                map_limited_yaw_rate_deg_s="",
                takeover_duration_interlock_ok="0",
                takeover_duration_interlock_latched="1",
                takeover_duration_s="3.01",
            )
            csv_path = self._write_log(Path(directory), row, schema_version=14)

            result = tool.analyze_log(csv_path)
            codes = {item["code"] for item in result["violations"]}

            self.assertNotIn("command_shaping_nonfinite", codes)
            self.assertNotIn("algorithm_with_invalid_command_shaping", codes)

    def test_schema_v15_requires_finite_accel_tilt_rate_diagnostics(self):
        with tempfile.TemporaryDirectory() as directory:
            row = self._safe_row()
            row.update(
                command_mapping_type="accel_tilt_rate",
                pre_shape_sp_valid="1",
                command_desired_roll_angle_deg="5.0",
                command_desired_pitch_angle_deg="-3.0",
                command_current_roll_angle_deg="1.0",
                command_current_pitch_angle_deg="-1.0",
                command_roll_attitude_error_deg="4.0",
                command_pitch_attitude_error_deg="",
            )
            csv_path = self._write_log(Path(directory), row, schema_version=15)

            result = tool.analyze_log(csv_path)
            codes = {item["code"] for item in result["violations"]}

            self.assertIn("accel_tilt_rate_diagnostics_nonfinite", codes)
            self.assertNotIn("invalid_guidance_command_mapping", codes)

    def test_schema_v16_is_supported(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = self._write_log(
                Path(directory),
                self._safe_row(),
                schema_version=16,
            )

            result = tool.analyze_log(csv_path)

            self.assertNotIn("unsupported_log_schema_version:16", result["warnings"])

    def test_schema_v17_validates_complete_post_disarm_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            rows = []
            for elapsed_s, armed in ((0.0, "1"), (1.0, "0"), (11.0, "0")):
                row = self._safe_row()
                row["elapsed_s"] = str(elapsed_s)
                row["armed"] = armed
                rows.append(row)
            csv_path = self._write_log(Path(directory), rows, schema_version=17)
            meta_path = csv_path.with_name("bench_meta.json")
            meta = json.loads(meta_path.read_text())
            meta["args"] = {"stop_after_disarm_s": 10.0}
            meta["completion"] = {
                "complete": True,
                "stop_reason": "post_disarm_tail_complete",
                "post_disarm_tail_completed": True,
            }
            meta_path.write_text(json.dumps(meta), encoding="utf-8")

            result = tool.analyze_log(csv_path)

            self.assertNotIn("unsupported_log_schema_version:17", result["warnings"])
            self.assertNotIn(
                "post_disarm_log_tail_incomplete",
                {item["code"] for item in result["violations"]},
            )
            self.assertEqual(result["metrics"]["post_disarm_tail_logged_s"], 10.0)

    def test_schema_v17_rejects_truncated_post_disarm_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            rows = []
            for elapsed_s, armed in ((0.0, "1"), (1.0, "0"), (4.0, "0")):
                row = self._safe_row()
                row["elapsed_s"] = str(elapsed_s)
                row["armed"] = armed
                rows.append(row)
            csv_path = self._write_log(Path(directory), rows, schema_version=17)
            meta_path = csv_path.with_name("bench_meta.json")
            meta = json.loads(meta_path.read_text())
            meta["args"] = {"stop_after_disarm_s": 10.0}
            meta["completion"] = {
                "complete": False,
                "stop_reason": "keyboard_interrupt",
                "post_disarm_tail_completed": False,
            }
            meta_path.write_text(json.dumps(meta), encoding="utf-8")

            result = tool.analyze_log(csv_path)

            self.assertIn(
                "post_disarm_log_tail_incomplete",
                {item["code"] for item in result["violations"]},
            )

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
            "safety_state": "ACTIVE",
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
            "msp_last_publish_command_active": "1",
            "msp_last_publish_command_reason": "active",
            "msp_last_publish_set_raw_rc_ack_fresh": "1",
            "map_limited_roll_rate_deg_s": "3.0",
            "map_limited_pitch_rate_deg_s": "-3.0",
            "map_limited_yaw_rate_deg_s": "0.0",
            "pre_shape_sp_roll_rate_deg_s": "3.0",
            "pre_shape_sp_pitch_rate_deg_s": "-3.0",
            "sp_roll_rate_deg_s": "3.0",
            "sp_pitch_rate_deg_s": "-3.0",
            "shaping_valid": "1",
            "shaping_reason": "",
            "entry_handoff_active": "0",
            "entry_handoff_progress": "1.0",
            "entry_handoff_source": "gyro",
            "tilt_roll_attitude_deg": "30.0",
            "tilt_pitch_attitude_deg": "-30.0",
            "tilt_roll_softcap_factor": "0.5",
            "tilt_pitch_softcap_factor": "0.5",
            "tilt_roll_level_weight": "0.0",
            "tilt_pitch_level_weight": "0.0",
            "tilt_hardcap_active": "0",
            "msp_send_success_max_interval_s": "0.02",
            "msp_worker_send_error_count": "0",
            "msp_cmd_set_raw_rc_error_count": "0",
            "msp_set_raw_rc_success_count": "50",
            "msp_set_raw_rc_write_success_count": "50",
            "msp_set_raw_rc_write_error_count": "0",
            "msp_set_raw_rc_ack_count": "50",
            "msp_set_raw_rc_ack_age_s": "0.02",
            "msp_set_raw_rc_write_rate_hz": "50.0",
            "msp_set_raw_rc_write_max_interval_s": "0.02",
            "msp_set_raw_rc_write_p999_interval_s": "0.02",
            "msp_rx_checksum_error_count": "0",
            "msp_rx_parser_error_count": "0",
            "msp_publish_deadline_miss_count": "0",
            "msp_cmd_set_raw_rc_max_rtt_ms": "2.0",
            "msp_cmd_raw_imu_max_rtt_ms": "2.5",
            "rknn_total_ms": "8.0",
            "host_thermal_max_c": "65.0",
            "gyro_msp_raw_x": "16",
            "gyro_msp_raw_y": "-32",
            "gyro_msp_raw_z": "8",
            "gyro_roll_deg_s": "",
            "gyro_pitch_deg_s": "",
            "gyro_yaw_deg_s": "",
            "guidance_eval_frame": "inertial_ned",
            "rate_gain_input_frame": "body_frd",
            "command_mapping_type": "direct_rate_matrix",
            "guidance_valid": "1",
            "g_eval_body_frd_x": "0.1",
            "g_eval_body_frd_y": "0.2",
            "g_eval_body_frd_z": "0.3",
            "motor_interlock_ok": "1",
            "motor_interlock_latched": "0",
            "takeover_duration_interlock_ok": "1",
            "takeover_duration_interlock_latched": "0",
            "takeover_duration_s": "1.0",
        }

    @staticmethod
    def _write_log(directory: Path, row, *, web_enabled=False, schema_version=10):
        rows = row if isinstance(row, list) else [row]
        csv_path = directory / "bench.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        meta = {
            "config": {
                "msp_runtime": {
                    "control_publish_hz": 50.0,
                    "prefill_valid_min_us": 900,
                    "prefill_valid_max_us": 2100,
                    "throttle_channel_zero_based": 2,
                    "response_stale_s": 0.25,
                },
                "rc_mapping": {
                    "roll_command_limit_deg_s": 3.0,
                    "pitch_command_limit_deg_s": 3.0,
                    "yaw_command_limit_deg_s": 0.0,
                    "throttle_max_us": 1100,
                },
                "guidance_command": {
                    "tilt_envelope": {
                        "enabled": True,
                        "max_roll_angle_deg": 35.0,
                        "max_pitch_angle_deg": 35.0,
                        "hardcap_margin_deg": 5.0,
                    }
                },
                "safety": {
                    "takeover_duration_interlock": {
                        "enabled": True,
                        "max_duration_s": 3.0,
                        "latch_until_disarm": True,
                    }
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
