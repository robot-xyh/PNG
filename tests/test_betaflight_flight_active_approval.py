import copy
import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from create_betaflight_flight_active_approval import validate_flight_active_config


class BetaflightFlightActiveApprovalTest(unittest.TestCase):
    def setUp(self):
        path = ROOT / "config" / "betaflight.rk3588.velocity_png.flight_active_1s.json"
        self.config = json.loads(path.read_text(encoding="utf-8"))
        self.output = ROOT / "logs" / "betaflight_velocity_png_flight_active_1s_approval.json"
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
        return validate_flight_active_config(
            config,
            output_path=self.output,
            parsed_cli=self.parsed_cli,
            fc_identity=self.fc_identity,
        )

    def test_accepts_bounded_one_second_profile(self):
        evidence = self.validate(self.config)

        self.assertEqual(evidence["guidance"]["velocity_source"], "msp_kinematics")
        self.assertEqual(evidence["msp_raw_imu_gyro"]["reason"], "firmware_binding_match")

    def test_rejects_wider_relative_throttle_limit(self):
        config = copy.deepcopy(self.config)
        config["msp_runtime"]["throttle_relative_limit_us"] = 41

        with self.assertRaisesRegex(RuntimeError, "relative throttle limit"):
            self.validate(config)

    def test_rejects_longer_or_rearmable_takeover(self):
        config = copy.deepcopy(self.config)
        config["safety"]["takeover_duration_interlock"]["max_duration_s"] = 1.01

        with self.assertRaisesRegex(RuntimeError, "takeover duration"):
            self.validate(config)

        config = copy.deepcopy(self.config)
        config["safety"]["takeover_duration_interlock"]["latch_until_disarm"] = False
        with self.assertRaisesRegex(RuntimeError, "latch until DISARM"):
            self.validate(config)

    def test_rejects_non_four_axis_override_mask(self):
        config = copy.deepcopy(self.config)
        config["msp_runtime"]["override_channels_mask"] = 7

        with self.assertRaisesRegex(RuntimeError, "mask 15"):
            self.validate(config)


if __name__ == "__main__":
    unittest.main()
