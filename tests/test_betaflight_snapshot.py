import csv
import json
import tempfile
import unittest
from pathlib import Path

from vision_guidance.betaflight_msp import (
    AnalogTelemetry,
    AltitudeTelemetry,
    ApiVersion,
    AttitudeTelemetry,
    BetaflightTelemetry,
    FcVersion,
    RawGpsTelemetry,
    StatusTelemetry,
)
from vision_guidance.betaflight_snapshot import (
    _clock_dates_match,
    capture_betaflight_snapshot,
    classify_cli_export,
    parse_cli_export,
    review_cli_exports,
)


class _Clock:
    def __init__(self):
        self.value = 0.0

    def monotonic(self):
        return self.value

    def sleep(self, duration):
        self.value += duration


class _Adapter:
    port = "/dev/test"
    baudrate = 115200
    timeout_s = 0.1

    def read_api_version(self):
        return ApiVersion(1, 1, 46)

    def read_fc_variant(self):
        return "BTFL"

    def read_fc_version(self):
        return FcVersion(4, 5, 2)

    def read_box_ids(self):
        return (0, 1, 50)

    def read_box_names(self):
        return ("ARM", "ANGLE", "MSP OVERRIDE")

    def read_telemetry(self):
        return BetaflightTelemetry(
            timestamp=10.0,
            status=StatusTelemetry(125, 0, 3, 1, 0),
            attitude=AttitudeTelemetry(1.0, -2.0, 30.0),
            analog=AnalogTelemetry(0.0, 0, 0, 0.0),
            rc_channels=(1500, 1500, 1000, 1500, 1000),
        )


class _KinematicAdapter(_Adapter):
    def read_telemetry(self, *, include_raw_gps=False, include_altitude=False):
        telemetry = super().read_telemetry()
        return BetaflightTelemetry(
            **{
                **telemetry.__dict__,
                "raw_gps": RawGpsTelemetry(
                    fix=1,
                    satellites=9,
                    latitude_deg=22.799,
                    longitude_deg=113.86,
                    altitude_m=15.2,
                    ground_speed_m_s=0.1,
                    ground_course_deg=90.0,
                    hdop=80,
                )
                if include_raw_gps
                else None,
                "altitude": AltitudeTelemetry(altitude_m=1.2, vertical_speed_m_s=-0.1)
                if include_altitude
                else None,
            }
        )


class BetaflightSnapshotTest(unittest.TestCase):
    def test_clock_date_comparison_requires_matching_system_and_rtc_dates(self):
        self.assertTrue(_clock_dates_match("Sat 2026-07-11 22:00:00 CST", "Sat 2026-07-11 22:00:01 CST"))
        self.assertFalse(_clock_dates_match("Sat 2026-07-11 22:00:00 CST", "Fri 2021-01-01 12:00:00 CST"))
        self.assertFalse(_clock_dates_match("", ""))

    def test_classifies_cli_export_categories(self):
        text = """
serial 0 1 115200 57600 0 115200
map AETR1234
aux 0 0 0 1700 2100 0 0
set failsafe_delay = 4
profile 0
set p_roll = 40
rateprofile 0
set roll_rc_rate = 100
set blackbox_device = SDCARD
set vbat_min_cell_voltage = 330
"""
        self.assertTrue(all(classify_cli_export(text).values()))

    def test_parses_structured_cli_values(self):
        parsed = parse_cli_export(
            "# Betaflight / TEST 4.5.2\n"
            "serial 1 64 115200 57600 0 115200\n"
            "map TAER1234\n"
            "aux 0 0 1 1700 2100 0 0\n"
            "set failsafe_delay = 4\n"
            "profile 2\nset p_roll = 45\n"
            "rateprofile 1\nset roll_rc_rate = 100\n"
        )

        self.assertEqual(parsed["firmware_header"], "Betaflight / TEST 4.5.2")
        self.assertEqual(parsed["serial_ports"][0]["identifier"], 1)
        self.assertEqual(parsed["receiver"]["channel_map"], "TAER1234")
        self.assertEqual(parsed["aux_ranges"][0]["range_start_us"], 1700)
        self.assertEqual(parsed["pid_profiles"]["2"]["p_roll"], "45")
        self.assertEqual(parsed["rate_profiles"]["1"]["roll_rc_rate"], "100")

    def test_parses_symbolic_serial_ports_and_rx_failsafe(self):
        parsed = parse_cli_export(
            "serial UART2 131073 230400 57600 0 115200\n"
            "rxfail 0 a\n"
            "rxfail 4 s 1200\n"
        )

        self.assertEqual(parsed["serial_ports"][0]["identifier"], "UART2")
        self.assertEqual(parsed["serial_ports"][0]["function_mask"], 131073)
        self.assertEqual(parsed["rx_failsafe"][0]["mode"], "a")
        self.assertEqual(parsed["rx_failsafe"][1]["value"], 1200)
        self.assertEqual(parsed["malformed_commands"], [])

    def test_reviews_diff_dump_conflicts(self):
        diff = parse_cli_export("map AETR1234\nset failsafe_delay = 4\n")
        dump = parse_cli_export("map TAER1234\nset failsafe_delay = 4\n")

        review = review_cli_exports({"diff_all": diff, "dump_all": dump})

        self.assertFalse(review["configuration_evidence_complete"])
        self.assertEqual(review["cross_export_conflicts"][0]["key"], "receiver.channel_map")

    def test_review_allows_dump_defaults_outside_diff_subset(self):
        diff = parse_cli_export(
            "serial UART1 1 115200 57600 0 115200\n"
            "aux 0 0 0 900 1300 0 0\n"
        )
        dump = parse_cli_export(
            "serial VCP 1 115200 57600 0 115200\n"
            "serial UART1 1 115200 57600 0 115200\n"
            "aux 0 0 0 900 1300 0 0\n"
            "aux 1 0 0 900 900 0 0\n"
        )

        review = review_cli_exports({"diff_all": diff, "dump_all": dump})

        self.assertEqual(review["cross_export_conflicts"], [])

    def test_reports_malformed_and_duplicate_cli_commands(self):
        parsed = parse_cli_export(
            "serial invalid\n"
            "profile 0\n"
            "set p_roll = 40\n"
            "set p_roll = 45\n"
        )

        self.assertEqual(parsed["malformed_commands"], ["serial invalid"])
        self.assertEqual(parsed["duplicate_assignments"][0]["scope"], "profile:0")
        self.assertEqual(parsed["pid_profiles"]["0"]["p_roll"], "45")

        review = review_cli_exports({"diff_all": parsed, "dump_all": parsed})
        self.assertFalse(review["configuration_evidence_complete"])
        self.assertEqual(review["duplicate_assignment_count"], 2)

    def test_master_command_restores_global_setting_scope(self):
        parsed = parse_cli_export(
            "profile 0\n"
            "set p_roll = 40\n"
            "master\n"
            "set failsafe_delay = 4\n"
        )

        self.assertEqual(parsed["settings"]["failsafe_delay"], "4")
        self.assertNotIn("failsafe_delay", parsed["pid_profiles"]["0"])

    def test_section_comments_scope_concatenated_configurator_exports(self):
        parsed = parse_cli_export(
            "rateprofile 0\n"
            "set roll_rc_rate = 100\n"
            "# dump all\n"
            "batch start\n"
            "# master\n"
            "set failsafe_delay = 15\n"
            "# profile 0\n"
            "set p_roll = 51\n"
            "# rateprofile 0\n"
            "set roll_srate = 70\n"
        )

        self.assertEqual(parsed["settings"]["failsafe_delay"], "15")
        self.assertEqual(parsed["pid_profiles"]["0"]["p_roll"], "51")
        self.assertEqual(parsed["rate_profiles"]["0"]["roll_srate"], "70")

    def test_snapshot_archives_cli_and_remains_control_blocked(self):
        clock = _Clock()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cli = root / "diff.txt"
            cli.write_text("serial 0 1 115200 57600 0 115200\nmap AETR1234\n", encoding="utf-8")

            manifest_path = capture_betaflight_snapshot(
                _Adapter(),
                root / "out",
                duration_s=0.4,
                rate_hz=5.0,
                cli_export=cli,
                sleep_fn=clock.sleep,
                monotonic_fn=clock.monotonic,
            )
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

            self.assertTrue(manifest["msp_override_available"])
            self.assertEqual(manifest["schema_version"], 2)
            self.assertEqual(manifest["msp_override_mode"]["name"], "MSP OVERRIDE")
            self.assertTrue(manifest["readiness"]["log_only_ready"])
            self.assertFalse(manifest["readiness"]["control_ready"])
            self.assertIn("manual_control_approval_required", manifest["readiness"]["control_blockers"])
            self.assertEqual(manifest["capture"]["sample_count"], 2)
            self.assertIn("betaflight_cli.txt", manifest["artifacts"])
            self.assertIn("configuration_review.json", manifest["artifacts"])
            self.assertTrue((manifest_path.parent / "telemetry.csv").is_file())

    def test_snapshot_archives_diff_and_dump_separately(self):
        clock = _Clock()
        complete = """
serial 0 1 115200 57600 0 115200
map AETR1234
aux 0 0 0 1700 2100 0 0
set failsafe_delay = 4
profile 0
set p_roll = 40
rateprofile 0
set roll_rc_rate = 100
set blackbox_device = SDCARD
set vbat_min_cell_voltage = 330
"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            diff = root / "diff.txt"
            dump = root / "dump.txt"
            diff.write_text(complete, encoding="utf-8")
            dump.write_text(complete, encoding="utf-8")

            manifest_path = capture_betaflight_snapshot(
                _Adapter(),
                root / "out",
                duration_s=0.2,
                rate_hz=5.0,
                cli_diff_all=diff,
                cli_dump_all=dump,
                sleep_fn=clock.sleep,
                monotonic_fn=clock.monotonic,
            )
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

            self.assertTrue(manifest["cli_configuration"]["configuration_evidence_complete"])
            self.assertIn("betaflight_diff_all.txt", manifest["artifacts"])
            self.assertIn("betaflight_dump_all.txt", manifest["artifacts"])
            self.assertNotIn("cli_configuration_incomplete", manifest["readiness"]["control_blockers"])
            self.assertFalse(manifest["readiness"]["control_ready"])

    def test_snapshot_option_captures_gps_and_altitude_for_flight_approval(self):
        clock = _Clock()
        with tempfile.TemporaryDirectory() as directory:
            manifest_path = capture_betaflight_snapshot(
                _KinematicAdapter(),
                Path(directory) / "out",
                duration_s=0.2,
                rate_hz=5.0,
                include_kinematics=True,
                sleep_fn=clock.sleep,
                monotonic_fn=clock.monotonic,
            )
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            with (manifest_path.parent / "telemetry.csv").open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))

            self.assertTrue(manifest["capture"]["include_kinematics"])
            self.assertEqual(rows[0]["gps_fix"], "1")
            self.assertEqual(rows[0]["gps_satellites"], "9")
            self.assertEqual(rows[0]["baro_altitude_m"], "1.2")


if __name__ == "__main__":
    unittest.main()
