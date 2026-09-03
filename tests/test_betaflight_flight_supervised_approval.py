import copy
import csv
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from create_betaflight_flight_supervised_approval import (  # noqa: E402
    validate_flight_supervised_config,
    validate_snapshot_flight_state,
)


class BetaflightFlightSupervisedApprovalTest(unittest.TestCase):
    def setUp(self):
        path = ROOT / "config" / "betaflight.rk3588.velocity_png.flight_supervised.json"
        self.config = json.loads(path.read_text(encoding="utf-8"))
        self.output = ROOT / "logs" / "betaflight_velocity_png_flight_supervised_approval.json"
        self.parsed_cli = {
            "rate_profiles": {
                "0": {
                    "roll_rc_rate": 100,
                    "pitch_rc_rate": 100,
                    "yaw_rc_rate": 100,
                    "roll_srate": 70,
                    "pitch_srate": 70,
                    "yaw_srate": 70,
                    "roll_expo": 0,
                    "pitch_expo": 0,
                    "yaw_expo": 0,
                }
            }
        }
        self.fc_identity = {
            "fc_variant": "BTFL",
            "fc_version_major": 25,
            "fc_version_minor": 12,
            "fc_version_patch": 2,
            "api_major": 1,
            "api_minor": 47,
        }

    def validate(self, config):
        return validate_flight_supervised_config(
            config,
            output_path=self.output,
            parsed_cli=self.parsed_cli,
            fc_identity=self.fc_identity,
        )

    def test_accepts_exact_supervised_profile(self):
        evidence = self.validate(self.config)

        self.assertEqual(evidence["guidance"]["velocity_source"], "msp_kinematics")
        self.assertEqual(evidence["msp_runtime"]["poll_total_hz"], 46.0)
        self.assertEqual(
            evidence["guidance_command"]["accel_tilt_rate"]["thrust_feedforward"][
                "calibration_id"
            ],
            "LOG00062_1275_1500",
        )

    def test_rejects_rate_acceleration_and_throttle_expansion(self):
        config = copy.deepcopy(self.config)
        config["rc_mapping"]["roll_command_limit_deg_s"] = 61
        with self.assertRaisesRegex(RuntimeError, "60/60/0"):
            self.validate(config)

        config = copy.deepcopy(self.config)
        config["guidance"]["velocity_establishing_png"]["total_accel_limit_m_s2"] = 7.1
        with self.assertRaisesRegex(RuntimeError, "exceeds 7"):
            self.validate(config)

        config = copy.deepcopy(self.config)
        config["msp_runtime"]["throttle_command_max_us"] = 1501
        with self.assertRaisesRegex(RuntimeError, "throttle runtime envelope"):
            self.validate(config)

    def test_rejects_old_relative_limit_and_wrong_thrust_binding(self):
        config = copy.deepcopy(self.config)
        config["msp_runtime"]["throttle_relative_limit_us"] = 40
        with self.assertRaisesRegex(RuntimeError, "relative throttle"):
            self.validate(config)

        config = copy.deepcopy(self.config)
        config["guidance_command"]["accel_tilt_rate"]["thrust_feedforward"][
            "calibration_id"
        ] = "unverified"
        with self.assertRaisesRegex(RuntimeError, "LOG00062"):
            self.validate(config)

    def test_rejects_enabled_timer_or_wrong_poll_schedule(self):
        config = copy.deepcopy(self.config)
        config["safety"]["takeover_duration_interlock"].update(
            enabled=True,
            max_duration_s=10,
        )
        with self.assertRaisesRegex(RuntimeError, "explicitly disabled"):
            self.validate(config)

        config = copy.deepcopy(self.config)
        config["msp_runtime"]["attitude_poll_hz"] = 10
        with self.assertRaisesRegex(RuntimeError, "attitude_poll_hz"):
            self.validate(config)

    def test_snapshot_flight_state_requires_hashed_gps_and_voltage_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            telemetry = root / "telemetry.csv"
            with telemetry.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=("gps_fix", "gps_satellites", "vbat_v"),
                )
                writer.writeheader()
                for satellites in (8, 9, 10):
                    writer.writerow(
                        {"gps_fix": 1, "gps_satellites": satellites, "vbat_v": 24.1}
                    )
            digest = hashlib.sha256(telemetry.read_bytes()).hexdigest()
            manifest_path = root / "manifest.json"
            snapshot = {
                "capture": {"include_kinematics": True},
                "artifacts": {"telemetry.csv": digest},
            }

            evidence = validate_snapshot_flight_state(snapshot, manifest_path)
            self.assertEqual(evidence["valid_sample_count"], 3)
            self.assertEqual(evidence["minimum_satellites"], 8)

            snapshot["capture"]["include_kinematics"] = False
            with self.assertRaisesRegex(RuntimeError, "include-kinematics"):
                validate_snapshot_flight_state(snapshot, manifest_path)


if __name__ == "__main__":
    unittest.main()
