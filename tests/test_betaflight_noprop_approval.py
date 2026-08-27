import copy
import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _load_tool_module():
    path = ROOT / "tools" / "create_betaflight_noprop_approval.py"
    spec = importlib.util.spec_from_file_location("create_betaflight_noprop_approval_for_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tool = _load_tool_module()


class BetaflightNoPropApprovalTest(unittest.TestCase):
    def test_example_config_passes_noprop_limits(self):
        config = json.loads((ROOT / "config/betaflight.rk3588.noprop.example.json").read_text())
        output = (ROOT / config["control_authorization"]["approval_manifest"]).resolve()

        tool._validate_noprop_config(config, output)

        unsafe = copy.deepcopy(config)
        unsafe["rc_mapping"]["roll_command_limit_deg_s"] = 3.1
        with self.assertRaisesRegex(RuntimeError, "roll_command_limit"):
            tool._validate_noprop_config(unsafe, output)

    def test_snapshot_requires_verified_override_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            review_path = root / "configuration_review.json"
            parsed = {
                "settings": {
                    "msp_override_channels_mask": "15",
                    "msp_override_failsafe": "OFF",
                },
                "receiver": {"channel_map": "AETR1234"},
                "aux_ranges": [
                    {
                        "mode_id": 50,
                        "aux_channel_index": 2,
                        "range_start_us": 1700,
                        "range_end_us": 2100,
                    }
                ],
                "rate_profiles": {
                    "0": {
                        "roll_rc_rate": "100",
                        "pitch_rc_rate": "100",
                        "yaw_rc_rate": "100",
                        "roll_srate": "70",
                        "pitch_srate": "70",
                        "yaw_srate": "70",
                        "roll_expo": "0",
                        "pitch_expo": "0",
                        "yaw_expo": "0",
                    }
                },
            }
            review = {"exports": {"dump_all": parsed}}
            review_path.write_text(json.dumps(review) + "\n", encoding="utf-8")
            review_sha = hashlib.sha256(review_path.read_bytes()).hexdigest()
            snapshot_path = root / "manifest.json"
            snapshot = {
                "readiness": {"log_only_ready": True},
                "capture": {"error_count": 0},
                "fc_identity": {"fc_variant": "BTFL"},
                "box_ids": [0, 50],
                "msp_override_available": True,
                "msp_override_mode": {"permanent_id": 50, "name": "MSP OVERRIDE"},
                "cli_configuration": {
                    "configuration_evidence_complete": True,
                    "review_artifact": review_path.name,
                },
                "artifacts": {review_path.name: review_sha},
            }
            snapshot_path.write_text(json.dumps(snapshot) + "\n", encoding="utf-8")

            config = json.loads((ROOT / "config/betaflight.rk3588.noprop.example.json").read_text())
            parsed_cli = tool._validate_snapshot(
                snapshot,
                snapshot_path,
                expected_override_mode_cli_id=config["msp_runtime"]["override_mode_cli_id"],
            )
            output = (ROOT / config["control_authorization"]["approval_manifest"]).resolve()
            tool._validate_noprop_config(config, output, parsed_cli=parsed_cli)

            with self.assertRaisesRegex(RuntimeError, "MSP OVERRIDE must be assigned"):
                tool._validate_snapshot(
                    snapshot,
                    snapshot_path,
                    expected_override_mode_cli_id=42,
                )

            mismatched_rate = copy.deepcopy(parsed_cli)
            mismatched_rate["rate_profiles"]["0"]["roll_srate"] = "65"
            with self.assertRaisesRegex(RuntimeError, "betaflight_super_rate"):
                tool._validate_noprop_config(config, output, parsed_cli=mismatched_rate)

            snapshot["capture"]["error_count"] = 1
            with self.assertRaisesRegex(RuntimeError, "capture errors"):
                tool._validate_snapshot(
                    snapshot,
                    snapshot_path,
                    expected_override_mode_cli_id=50,
                )

    def test_override_cli_mode_id_must_be_explicit(self):
        config = json.loads((ROOT / "config/betaflight.rk3588.noprop.example.json").read_text())
        del config["msp_runtime"]["override_mode_cli_id"]
        output = (ROOT / config["control_authorization"]["approval_manifest"]).resolve()

        with self.assertRaisesRegex(RuntimeError, "override_mode_cli_id must be explicitly configured"):
            tool._validate_noprop_config(config, output)


if __name__ == "__main__":
    unittest.main()
