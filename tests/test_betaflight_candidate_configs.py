import argparse
import importlib.util
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "config"


def _load_runner_module():
    path = ROOT / "examples" / "run_betaflight_log_only.py"
    spec = importlib.util.spec_from_file_location("candidate_config_runner_for_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


runner = _load_runner_module()


def _read(name):
    return json.loads((CONFIG_DIR / name).read_text(encoding="utf-8"))


class BetaflightCandidateConfigTest(unittest.TestCase):
    def setUp(self):
        self.log_only = _read("betaflight.rk3588.velocity_png.flight_log_only.json")
        self.noprop = _read("betaflight.rk3588.velocity_png.noprop_fault.json")
        self.virtual_noprop = _read(
            "betaflight.rk3588.velocity_png.virtual_bbox_noprop.json"
        )
        self.prop_rig = _read("betaflight.rk3588.velocity_png.prop_rig_log_only.json")
        self.prop_rig_active = _read(
            "betaflight.rk3588.velocity_png.prop_rig_active.json"
        )
        self.limited = _read("betaflight.rk3588.velocity_png.flight_limited.json")
        self.flight_active = _read(
            "betaflight.rk3588.velocity_png.flight_active_1s.json"
        )
        self.flight_supervised = _read(
            "betaflight.rk3588.velocity_png.flight_supervised.json"
        )

    def test_final_vm_log_only_contract(self):
        config = self.log_only
        self.assertEqual(config["guidance"]["law"], "velocity_establishing_png")
        self.assertEqual(config["guidance"]["velocity_source"], "msp_kinematics")
        self.assertEqual(config["guidance_command"]["mapping_type"], "accel_tilt_rate")
        mapping = config["guidance_command"]["accel_tilt_rate"]
        self.assertEqual(mapping["roll_rate_sign"], 1.0)
        self.assertEqual(mapping["pitch_rate_sign"], -1.0)
        self.assertEqual(config["rc_mapping"]["throttle_hover_us"], 1275)
        self.assertFalse(config["control_authorization"]["enabled"])
        self.assertFalse(config["runtime_policy"]["msp_set_raw_rc_permitted"])
        self.assertEqual(
            config["runtime_policy"]["allowed_control_modes"],
            ["log_only"],
        )

    def test_noprop_fault_contract(self):
        config = self.noprop
        self.assertEqual(config["bench_profile"]["scope"], "noprop_bench")
        self.assertTrue(config["bench_profile"]["all_propellers_removed_required"])
        self.assertTrue(config["candidate_profile"]["historical_fault_validation_reused"])
        self.assertFalse(config["candidate_profile"]["repeat_full_fault_matrix_required"])
        self.assertEqual(config["msp_runtime"]["override_channels_mask"], 3)
        self.assertEqual(config["guidance"]["velocity_source"], "msp_kinematics")
        self.assertEqual(
            config["guidance"]["velocity_establishing_png"]["total_accel_limit_m_s2"],
            1.0,
        )
        self.assertEqual(config["rc_mapping"]["roll_command_limit_deg_s"], 3.0)
        self.assertEqual(config["rc_mapping"]["pitch_command_limit_deg_s"], 3.0)
        motor = config["safety"]["motor_output_interlock"]
        self.assertEqual(motor["max_output_us"], 1200)
        self.assertEqual(motor["max_spread_us"], 150)
        takeover = config["safety"]["takeover_duration_interlock"]
        self.assertEqual(takeover["max_duration_s"], 3.0)
        self.assertTrue(takeover["latch_until_disarm"])

    def test_noprop_and_limited_share_low_authority_controller(self):
        noprop = self.noprop
        limited = self.limited
        self.assertEqual(noprop["guidance"], limited["guidance"])
        self.assertEqual(
            noprop["guidance_command"]["accel_tilt_rate"],
            limited["guidance_command"]["accel_tilt_rate"],
        )
        self.assertEqual(noprop["msp_runtime"]["override_channels_mask"], 3)
        self.assertEqual(limited["msp_runtime"]["override_channels_mask"], 3)

    def test_virtual_bbox_noprop_is_explicitly_bench_scoped(self):
        config = self.virtual_noprop
        self.assertEqual(config["candidate_profile"]["scope"], "noprop_bench")
        self.assertTrue(config["candidate_profile"]["all_propellers_removed_required"])
        self.assertEqual(config["msp_runtime"]["override_channels_mask"], 3)
        self.assertEqual(config["guidance"]["velocity_source"], "bench_zero_velocity")
        self.assertEqual(
            config["guidance"]["velocity_establishing_png"]["total_accel_limit_m_s2"],
            1.0,
        )
        self.assertEqual(config["rc_mapping"]["roll_command_limit_deg_s"], 3.0)
        self.assertEqual(config["rc_mapping"]["pitch_command_limit_deg_s"], 3.0)
        self.assertEqual(config["rc_mapping"]["yaw_command_limit_deg_s"], 0.0)
        self.assertEqual(
            config["control_authorization"]["approval_manifest"],
            "logs/betaflight_velocity_png_virtual_bbox_noprop_approval.json",
        )

    def test_prop_rig_profile_is_manual_pulse_log_only(self):
        config = self.prop_rig
        profile = config["prop_rig_profile"]
        self.assertEqual(config["candidate_profile"]["scope"], "prop_rig_log_only")
        self.assertTrue(profile["propellers_installed"])
        self.assertFalse(profile["active_control_permitted"])
        self.assertEqual(profile["required_rc7_state"], "manual")
        self.assertEqual(
            profile["allowed_test"],
            "armed_idle_and_manual_roll_pitch_pulses",
        )
        self.assertEqual(profile["manual_pulse_limits"]["permitted_axes"], ["roll", "pitch"])
        self.assertEqual(profile["manual_pulse_limits"]["max_input_delta_us"], 75)
        self.assertEqual(config["msp_runtime"]["override_channels_mask"], 3)
        self.assertEqual(config["msp_runtime"]["motor_poll_hz"], 10.0)
        self.assertEqual(config["msp_runtime"]["rc_poll_hz"], 10.0)
        self.assertEqual(config["msp_runtime"]["raw_imu_poll_hz"], 10.0)
        self.assertFalse(config["control_authorization"]["enabled"])
        self.assertFalse(config["runtime_policy"]["allow_control_flag_permitted"])
        self.assertFalse(config["runtime_policy"]["msp_set_raw_rc_permitted"])
        self.assertEqual(config["runtime_policy"]["allowed_control_modes"], ["log_only"])

    def test_active_prop_rig_profile_is_bounded(self):
        config = self.prop_rig_active
        profile = config["bench_profile"]
        self.assertEqual(profile["scope"], "prop_rig_active")
        self.assertTrue(profile["propellers_installed"])
        self.assertTrue(profile["acro_rate_mode_required"])
        self.assertEqual(config["msp_runtime"]["override_channels_mask"], 15)
        self.assertEqual(config["guidance"]["velocity_source"], "bench_zero_velocity")
        self.assertEqual(config["rc_mapping"]["roll_command_limit_deg_s"], 3.0)
        self.assertEqual(config["rc_mapping"]["pitch_command_limit_deg_s"], 3.0)
        self.assertEqual(config["rc_mapping"]["yaw_command_limit_deg_s"], 0.0)
        self.assertEqual(
            config["safety"]["takeover_duration_interlock"]["max_duration_s"],
            100.0,
        )
        self.assertFalse(
            config["safety"]["takeover_duration_interlock"]["latch_until_disarm"]
        )
        self.assertEqual(
            config["safety"]["takeover_duration_interlock"]["rearm_release_s"],
            0.5,
        )
        self.assertEqual(config["safety"]["motor_output_interlock"]["max_output_us"], 1500)
        self.assertEqual(config["safety"]["motor_output_interlock"]["max_spread_us"], 250)
        self.assertEqual(
            config["safety"]["motor_output_interlock"]["violation_grace_s"],
            1.0,
        )
        self.assertEqual(config["rc_mapping"]["throttle_hover_us"], 1275)
        self.assertEqual(config["rc_mapping"]["throttle_max_us"], 1500)
        self.assertTrue(config["safety"]["require_acro_rate_mode"])
        self.assertEqual(config["control_authorization"]["required_scope"], "prop_rig_active")

    def test_all_configs_load_through_runner_contracts(self):
        for config in (
            self.log_only,
            self.noprop,
            self.virtual_noprop,
            self.prop_rig,
            self.prop_rig_active,
            self.limited,
            self.flight_active,
            self.flight_supervised,
        ):
            with self.subTest(profile=config["candidate_profile"]["id"]):
                _, metadata = runner._guidance_evaluator(config)
                self.assertEqual(metadata["law"], "velocity_establishing_png")
                self.assertEqual(
                    metadata["velocity_source"],
                    (
                        "bench_zero_velocity"
                        if config in (self.virtual_noprop, self.prop_rig_active)
                        else "msp_kinematics"
                    ),
                )
                runner._rc_mapping_config(config)
                runner.GuidanceCommandShaperConfig.from_mapping(
                    config["guidance_command"]
                )

    def test_limited_flight_is_not_authorized_for_control(self):
        config = self.limited
        self.assertFalse(config["candidate_profile"]["active_control_runnable"])
        self.assertFalse(config["candidate_profile"]["propeller_control_authorized"])
        self.assertFalse(config["control_authorization"]["enabled"])
        self.assertEqual(config["control_transport_release"]["status"], "blocked")
        self.assertEqual(config["rc_mapping"]["throttle_hover_us"], 1275)
        self.assertEqual(config["rc_mapping"]["yaw_command_limit_deg_s"], 0.0)
        self.assertEqual(config["runtime_policy"]["allowed_control_modes"], ["log_only"])

    def test_one_second_flight_profile_uses_relative_throttle_envelope(self):
        config = self.flight_active
        self.assertEqual(config["candidate_profile"]["scope"], "flight_active_1s")
        self.assertEqual(config["guidance"]["velocity_source"], "msp_kinematics")
        self.assertEqual(config["msp_runtime"]["override_channels_mask"], 15)
        self.assertEqual(
            config["flight_profile"]["controlled_axes"],
            ["roll", "pitch", "throttle", "yaw"],
        )
        self.assertEqual(config["flight_profile"]["yaw_policy"], "neutral_1500_us")
        self.assertEqual(config["msp_runtime"]["throttle_relative_limit_us"], 40)
        self.assertEqual(config["msp_runtime"]["throttle_reference_min_us"], 1200)
        self.assertEqual(config["msp_runtime"]["throttle_reference_max_us"], 1400)
        self.assertEqual(config["msp_runtime"]["throttle_command_max_us"], 1500)
        self.assertEqual(config["rc_mapping"]["throttle_max_us"], 1500)
        takeover = config["safety"]["takeover_duration_interlock"]
        self.assertEqual(takeover["max_duration_s"], 0.9)
        self.assertTrue(takeover["latch_until_disarm"])
        self.assertTrue(config["control_authorization"]["enabled"])
        self.assertEqual(
            config["control_authorization"]["required_scope"],
            "flight_active_1s",
        )

    def test_supervised_flight_profile_uses_measured_variable_thrust(self):
        config = self.flight_supervised
        self.assertEqual(
            config["candidate_profile"]["scope"],
            "flight_active_supervised",
        )
        self.assertEqual(config["msp_runtime"]["override_channels_mask"], 15)
        self.assertEqual(config["msp_runtime"]["throttle_relative_limit_us"], 0)
        self.assertEqual(config["msp_runtime"]["throttle_slew_limit_us_per_s"], 600)
        self.assertEqual(config["msp_runtime"]["throttle_command_min_us"], 1200)
        self.assertEqual(config["msp_runtime"]["throttle_command_max_us"], 1500)
        self.assertEqual(
            sum(
                config["msp_runtime"][key]
                for key in (
                    "status_poll_hz",
                    "attitude_poll_hz",
                    "raw_imu_poll_hz",
                    "raw_gps_poll_hz",
                    "altitude_poll_hz",
                    "motor_poll_hz",
                    "rc_poll_hz",
                    "analog_poll_hz",
                )
            ),
            46,
        )
        self.assertEqual(config["rc_mapping"]["roll_command_limit_deg_s"], 60)
        self.assertEqual(config["rc_mapping"]["pitch_command_limit_deg_s"], 60)
        self.assertEqual(config["rc_mapping"]["throttle_min_us"], 1200)
        self.assertEqual(config["rc_mapping"]["throttle_hover_us"], 1275)
        self.assertEqual(config["rc_mapping"]["throttle_max_us"], 1500)
        self.assertEqual(
            config["guidance"]["velocity_establishing_png"]["total_accel_limit_m_s2"],
            7,
        )
        thrust = config["guidance_command"]["accel_tilt_rate"]["thrust_feedforward"]
        self.assertTrue(thrust["enabled"])
        self.assertEqual(thrust["calibration_id"], "LOG00062_1275_1500")
        takeover = config["safety"]["takeover_duration_interlock"]
        self.assertFalse(takeover["enabled"])
        self.assertIsNone(takeover["max_duration_s"])
        self.assertFalse(takeover["latch_until_disarm"])
        self.assertEqual(takeover["rearm_release_s"], 0)

    def test_runtime_policy_rejects_output_for_log_only_profiles(self):
        log_args = argparse.Namespace(control_mode="log_only", allow_control=False)
        output_args = argparse.Namespace(control_mode="msp_raw_rc", allow_control=True)
        runner._validate_runtime_policy(self.log_only, log_args)
        runner._validate_runtime_policy(self.limited, log_args)
        with self.assertRaisesRegex(RuntimeError, "forbids control mode"):
            runner._validate_runtime_policy(self.log_only, output_args)
        with self.assertRaisesRegex(RuntimeError, "forbids control mode"):
            runner._validate_runtime_policy(self.limited, output_args)

    def test_runtime_policy_allows_only_explicit_noprop_output(self):
        output_args = argparse.Namespace(control_mode="msp_raw_rc", allow_control=True)
        runner._validate_runtime_policy(self.noprop, output_args)

    def test_prop_rig_active_requires_isolated_real_detector(self):
        base = {
            "control_mode": "msp_raw_rc",
            "allow_control": True,
            "detector_source": "rknn_bytetrack",
            "isolate_rknn_process": True,
            "main_cpu_affinity": "6,7",
            "rknn_cpu_affinity": "4,5",
        }
        runner._validate_runtime_policy(
            self.prop_rig_active,
            argparse.Namespace(**base),
        )

        without_isolation = dict(base, isolate_rknn_process=False)
        with self.assertRaisesRegex(RuntimeError, "requires --isolate-rknn-process"):
            runner._validate_runtime_policy(
                self.prop_rig_active,
                argparse.Namespace(**without_isolation),
            )

        without_affinity = dict(base, main_cpu_affinity="")
        with self.assertRaisesRegex(RuntimeError, "requires explicit"):
            runner._validate_runtime_policy(
                self.prop_rig_active,
                argparse.Namespace(**without_affinity),
            )

    def test_flight_active_requires_isolated_real_detector(self):
        runner._validate_runtime_policy(
            self.flight_active,
            argparse.Namespace(
                control_mode="msp_raw_rc",
                allow_control=True,
                detector_source="rknn_bytetrack",
                isolate_rknn_process=True,
                main_cpu_affinity="6,7",
                rknn_cpu_affinity="4,5",
            ),
        )
        runner._validate_runtime_policy(
            self.flight_supervised,
            argparse.Namespace(
                control_mode="msp_raw_rc",
                allow_control=True,
                detector_source="rknn_bytetrack",
                isolate_rknn_process=True,
                main_cpu_affinity="6,7",
                rknn_cpu_affinity="4,5",
            ),
        )


if __name__ == "__main__":
    unittest.main()
