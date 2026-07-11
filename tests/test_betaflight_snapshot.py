import json
import tempfile
import unittest
from pathlib import Path

from vision_guidance.betaflight_msp import (
    AnalogTelemetry,
    ApiVersion,
    AttitudeTelemetry,
    BetaflightTelemetry,
    FcVersion,
    StatusTelemetry,
)
from vision_guidance.betaflight_snapshot import capture_betaflight_snapshot, classify_cli_export


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

    def read_telemetry(self):
        return BetaflightTelemetry(
            timestamp=10.0,
            status=StatusTelemetry(125, 0, 3, 1, 0),
            attitude=AttitudeTelemetry(1.0, -2.0, 30.0),
            analog=AnalogTelemetry(0.0, 0, 0, 0.0),
            rc_channels=(1500, 1500, 1000, 1500, 1000),
        )


class BetaflightSnapshotTest(unittest.TestCase):
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
            self.assertTrue(manifest["readiness"]["log_only_ready"])
            self.assertFalse(manifest["readiness"]["control_ready"])
            self.assertIn("manual_control_approval_required", manifest["readiness"]["control_blockers"])
            self.assertEqual(manifest["capture"]["sample_count"], 2)
            self.assertIn("betaflight_cli.txt", manifest["artifacts"])
            self.assertTrue((manifest_path.parent / "telemetry.csv").is_file())


if __name__ == "__main__":
    unittest.main()
