import copy
import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np


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


def _with_verified_upward_camera(config):
    result = copy.deepcopy(config)
    result["camera"]["R_BC"] = [
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    result["camera"]["extrinsic_validation"]["verified"] = True
    return result


class BetaflightNoPropApprovalTest(unittest.TestCase):
    def test_verified_example_config_passes_noprop_limits(self):
        example = json.loads((ROOT / "config/betaflight.rk3588.noprop.example.json").read_text())
        config = _with_verified_upward_camera(example)
        output = (ROOT / config["control_authorization"]["approval_manifest"]).resolve()

        tool._validate_noprop_config(config, output)

        missing_eval_frame = copy.deepcopy(config)
        missing_eval_frame["guidance_command"].pop("guidance_eval_frame")
        with self.assertRaisesRegex(RuntimeError, "guidance_eval_frame"):
            tool._validate_noprop_config(missing_eval_frame, output)

        wrong_rate_frame = copy.deepcopy(config)
        wrong_rate_frame["guidance_command"]["rate_gain_input_frame"] = "inertial_ned"
        with self.assertRaisesRegex(RuntimeError, "rate_gain_input_frame"):
            tool._validate_noprop_config(wrong_rate_frame, output)

        legacy = copy.deepcopy(example)
        legacy["camera"].pop("R_BC")
        legacy["camera"]["pitch_up_deg"] = 90.0
        legacy["camera"]["extrinsic_validation"]["verified"] = False
        with self.assertRaisesRegex(RuntimeError, "R_BC must be explicit"):
            tool._validate_noprop_config(legacy, output)

        unverified = copy.deepcopy(config)
        unverified["camera"]["extrinsic_validation"]["verified"] = False
        with self.assertRaisesRegex(RuntimeError, "verified=true"):
            tool._validate_noprop_config(unverified, output)

        forward = copy.deepcopy(config)
        forward["camera"]["R_BC"] = np.eye(3).tolist()
        with self.assertRaisesRegex(RuntimeError, "not aligned with body-up"):
            tool._validate_noprop_config(forward, output)

        unsafe = copy.deepcopy(config)
        unsafe["rc_mapping"]["roll_command_limit_deg_s"] = 3.1
        with self.assertRaisesRegex(RuntimeError, "roll_command_limit"):
            tool._validate_noprop_config(unsafe, output)

        no_entry = copy.deepcopy(config)
        no_entry["guidance_command"]["entry_handoff"]["enabled"] = False
        with self.assertRaisesRegex(RuntimeError, "entry_handoff must be enabled"):
            tool._validate_noprop_config(no_entry, output)

        zero_entry = copy.deepcopy(config)
        zero_entry["guidance_command"]["entry_handoff"]["rate_source"] = "zero"
        tool._validate_noprop_config(zero_entry, output)

        invalid_gyro_entry = copy.deepcopy(config)
        invalid_gyro_entry["msp_runtime"]["raw_imu_gyro"]["scale_deg_s_per_lsb"] = 0.1
        with self.assertRaisesRegex(RuntimeError, "scale must be 0.0625"):
            tool._validate_noprop_config(invalid_gyro_entry, output)

        wrong_gyro_sign = copy.deepcopy(config)
        wrong_gyro_sign["msp_runtime"]["raw_imu_gyro"]["axis_sign"] = [1.0, 1.0, 1.0]
        with self.assertRaisesRegex(RuntimeError, "measured x,y,z to FRD sign mapping"):
            tool._validate_noprop_config(wrong_gyro_sign, output)

        missing_rate_source = copy.deepcopy(config)
        missing_rate_source["guidance_command"]["entry_handoff"].pop("rate_source")
        with self.assertRaisesRegex(RuntimeError, "rate_source must be zero or gyro"):
            tool._validate_noprop_config(missing_rate_source, output)

        short_entry = copy.deepcopy(config)
        short_entry["guidance_command"]["entry_handoff"]["duration_s"] = 0.5
        with self.assertRaisesRegex(RuntimeError, "duration_s must be at least"):
            tool._validate_noprop_config(short_entry, output)

        stale_entry_gyro = copy.deepcopy(config)
        stale_entry_gyro["guidance_command"]["entry_handoff"]["gyro_max_age_s"] = 0.3
        with self.assertRaisesRegex(RuntimeError, "gyro_max_age_s"):
            tool._validate_noprop_config(stale_entry_gyro, output)

        no_tilt = copy.deepcopy(config)
        no_tilt["guidance_command"]["tilt_envelope"]["enabled"] = False
        with self.assertRaisesRegex(RuntimeError, "tilt_envelope must be enabled"):
            tool._validate_noprop_config(no_tilt, output)

        wide_tilt = copy.deepcopy(config)
        wide_tilt["guidance_command"]["tilt_envelope"]["max_roll_angle_deg"] = 36.0
        with self.assertRaisesRegex(RuntimeError, "max_roll_angle_deg"):
            tool._validate_noprop_config(wide_tilt, output)

        fast_leveling = copy.deepcopy(config)
        fast_leveling["guidance_command"]["tilt_envelope"][
            "hardcap_max_level_rate_deg_s"
        ] = 3.1
        with self.assertRaisesRegex(RuntimeError, "hardcap_max_level_rate_deg_s"):
            tool._validate_noprop_config(fast_leveling, output)

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

            config = _with_verified_upward_camera(
                json.loads((ROOT / "config/betaflight.rk3588.noprop.example.json").read_text())
            )
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

    def test_fixed_vm_guidance_is_explicit_and_bounded_for_approval(self):
        config = _with_verified_upward_camera(
            json.loads((ROOT / "config/betaflight.rk3588.noprop.example.json").read_text())
        )
        output = (ROOT / config["control_authorization"]["approval_manifest"]).resolve()
        config["guidance"] = {
            "law": "fixed_vm_png",
            "navigation_constant": 3.0,
            "fixed_vm_m_s": 1.0,
            "max_guidance_accel_mps2": 1.0,
        }

        tool._validate_noprop_config(config, output)
        metadata = tool._validate_guidance_config(config)
        self.assertEqual(metadata["fixed_gain"], 3.0)
        self.assertFalse(metadata["ttc_required"])

        for key in ("navigation_constant", "fixed_vm_m_s"):
            invalid = copy.deepcopy(config)
            invalid["guidance"].pop(key)
            with self.subTest(missing=key), self.assertRaisesRegex(RuntimeError, f"guidance.{key}"):
                tool._validate_noprop_config(invalid, output)

        excessive = copy.deepcopy(config)
        excessive["guidance"]["max_guidance_accel_mps2"] = 1.01
        with self.assertRaisesRegex(RuntimeError, "exceeds no-prop limit"):
            tool._validate_noprop_config(excessive, output)

        unknown = copy.deepcopy(config)
        unknown["guidance"]["law"] = "vm"
        with self.assertRaisesRegex(RuntimeError, "unsupported guidance.law"):
            tool._validate_noprop_config(unknown, output)

        implicit = copy.deepcopy(config)
        implicit.pop("guidance")
        with self.assertRaisesRegex(RuntimeError, "guidance.law must be explicitly configured"):
            tool._validate_noprop_config(implicit, output)

    def test_velocity_establishing_guidance_requires_bench_velocity_and_accel_mapping(self):
        config = _with_verified_upward_camera(
            json.loads(
                (
                    ROOT
                    / "config/betaflight.rk3588.noprop.velocity_establishing.example.json"
                ).read_text()
            )
        )
        output = (ROOT / config["control_authorization"]["approval_manifest"]).resolve()

        tool._validate_noprop_config(config, output)
        metadata = tool._validate_guidance_config(config)
        self.assertEqual(metadata["law"], "velocity_establishing_png")
        self.assertEqual(metadata["velocity_source"], "bench_zero_velocity")
        self.assertEqual(metadata["fixed_gain"], 30.0)
        self.assertEqual(
            config["guidance_command"]["accel_tilt_rate"]["pitch_rate_sign"],
            -1.0,
        )

        flight_velocity = copy.deepcopy(config)
        flight_velocity["guidance"]["velocity_source"] = "msp_kinematics"
        with self.assertRaisesRegex(RuntimeError, "bench_zero_velocity"):
            tool._validate_noprop_config(flight_velocity, output)

        direct_mapping = copy.deepcopy(config)
        direct_mapping["guidance_command"]["mapping_type"] = "direct_rate_matrix"
        with self.assertRaisesRegex(RuntimeError, "accel_tilt_rate"):
            tool._validate_noprop_config(direct_mapping, output)

        excessive = copy.deepcopy(config)
        excessive["guidance"]["velocity_establishing_png"][
            "total_accel_limit_m_s2"
        ] = 1.01
        with self.assertRaisesRegex(RuntimeError, "exceeds no-prop guidance limit"):
            tool._validate_noprop_config(excessive, output)

    def test_noprop_approval_requires_latched_motor_output_interlock(self):
        config = _with_verified_upward_camera(
            json.loads((ROOT / "config/betaflight.rk3588.noprop.example.json").read_text())
        )
        output = (ROOT / config["control_authorization"]["approval_manifest"]).resolve()

        tool._validate_noprop_config(config, output)

        disabled = copy.deepcopy(config)
        disabled["safety"]["motor_output_interlock"]["enabled"] = False
        with self.assertRaisesRegex(RuntimeError, "must be enabled"):
            tool._validate_noprop_config(disabled, output)

        excessive = copy.deepcopy(config)
        excessive["safety"]["motor_output_interlock"]["max_output_us"] = 1201
        with self.assertRaisesRegex(RuntimeError, "between throttle max and 1200"):
            tool._validate_noprop_config(excessive, output)

        not_latched = copy.deepcopy(config)
        not_latched["safety"]["motor_output_interlock"]["latch_until_disarm"] = False
        with self.assertRaisesRegex(RuntimeError, "latch until DISARM"):
            tool._validate_noprop_config(not_latched, output)

    def test_noprop_approval_requires_bounded_takeover_duration(self):
        config = _with_verified_upward_camera(
            json.loads((ROOT / "config/betaflight.rk3588.noprop.example.json").read_text())
        )
        output = (ROOT / config["control_authorization"]["approval_manifest"]).resolve()

        tool._validate_noprop_config(config, output)

        disabled = copy.deepcopy(config)
        disabled["safety"]["takeover_duration_interlock"]["enabled"] = False
        with self.assertRaisesRegex(RuntimeError, "takeover_duration_interlock must be enabled"):
            tool._validate_noprop_config(disabled, output)

        excessive = copy.deepcopy(config)
        excessive["safety"]["takeover_duration_interlock"]["max_duration_s"] = 3.01
        with self.assertRaisesRegex(RuntimeError, "max_duration_s"):
            tool._validate_noprop_config(excessive, output)

        not_latched = copy.deepcopy(config)
        not_latched["safety"]["takeover_duration_interlock"]["latch_until_disarm"] = False
        with self.assertRaisesRegex(RuntimeError, "latch until DISARM"):
            tool._validate_noprop_config(not_latched, output)

    def test_override_cli_mode_id_must_be_explicit(self):
        config = _with_verified_upward_camera(
            json.loads((ROOT / "config/betaflight.rk3588.noprop.example.json").read_text())
        )
        del config["msp_runtime"]["override_mode_cli_id"]
        output = (ROOT / config["control_authorization"]["approval_manifest"]).resolve()

        with self.assertRaisesRegex(RuntimeError, "override_mode_cli_id must be explicitly configured"):
            tool._validate_noprop_config(config, output)


if __name__ == "__main__":
    unittest.main()
