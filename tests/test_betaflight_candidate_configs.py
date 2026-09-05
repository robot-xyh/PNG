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
        self.flight_supervised = _read(
            "betaflight.rk3588.velocity_png.flight_supervised.json"
        )
        self.contact_candidate = _read(
            "betaflight.rk3588.velocity_png.flight_contact_candidate.json"
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

    def test_supervised_thrust_feedforward_config_is_runtime_loadable(self):
        mapping = runner._acceleration_tilt_rate_config(self.flight_supervised)

        self.assertTrue(mapping.thrust_feedforward.enabled)
        self.assertEqual(mapping.thrust_feedforward.model, "voltage_throttle_lut")
        self.assertEqual(
            mapping.thrust_feedforward.calibration_id,
            "PENDING_FULL_6S_THRUST_LUT",
        )
        self.assertEqual(mapping.thrust_feedforward.model_sha256, "0" * 64)

        model, status = runner._load_voltage_thrust_model(
            self.flight_supervised,
            config_path=CONFIG_DIR / "betaflight.rk3588.velocity_png.flight_supervised.json",
            required=False,
        )
        self.assertIsNone(model)
        self.assertFalse(status["ready"])
        with self.assertRaisesRegex(RuntimeError, "before hardware initialization"):
            runner._load_voltage_thrust_model(
                self.flight_supervised,
                config_path=CONFIG_DIR
                / "betaflight.rk3588.velocity_png.flight_supervised.json",
                required=True,
            )

    def test_noprop_fault_contract(self):
        config = self.noprop
        self.assertEqual(config["bench_profile"]["scope"], "noprop_bench")
        self.assertTrue(config["bench_profile"]["all_propellers_removed_required"])
        self.assertTrue(config["candidate_profile"]["historical_fault_validation_reused"])
        self.assertFalse(config["candidate_profile"]["repeat_full_fault_matrix_required"])
        self.assertEqual(config["msp_runtime"]["override_channels_mask"], 3)
        self.assertEqual(
            {
                key: config["msp_runtime"][key]
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
            },
            {
                "status_poll_hz": 5,
                "attitude_poll_hz": 20,
                "raw_imu_poll_hz": 5,
                "raw_gps_poll_hz": 5,
                "altitude_poll_hz": 5,
                "motor_poll_hz": 2,
                "rc_poll_hz": 5,
                "analog_poll_hz": 1,
            },
        )
        self.assertEqual(
            config["logging"]["evidence_frames"],
            {"enabled": True, "max_fps": 5, "jpeg_quality": 80},
        )
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
            self.flight_supervised,
            self.contact_candidate,
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

    def test_contact_candidate_is_log_only_and_requires_explicit_waiver(self):
        config = self.contact_candidate
        self.assertEqual(
            config["candidate_profile"]["scope"],
            "flight_contact_candidate_v1",
        )
        self.assertEqual(
            config["guidance"]["velocity_establishing_png"]["engagement_policy"],
            "contact",
        )
        self.assertFalse(config["control_authorization"]["enabled"])
        self.assertFalse(config["runtime_policy"]["msp_set_raw_rc_permitted"])
        self.assertTrue(config["contact_risk_policy"]["explicit_risk_waiver_required"])

    def test_final_candidates_use_reduced_detection_and_fusion_waits(self):
        for config in (self.log_only, self.flight_supervised, self.contact_candidate):
            with self.subTest(profile=config["candidate_profile"]["id"]):
                controller = config["guidance"]["velocity_establishing_png"]
                self.assertEqual(controller["detection_timeout_s"], 0.15)
                self.assertEqual(controller["velocity_reference_slew_m_s2"], 3)
                self.assertEqual(config["attitude_fusion"]["max_wait_s"], 0.06)

    def test_limited_flight_is_not_authorized_for_control(self):
        config = self.limited
        self.assertFalse(config["candidate_profile"]["active_control_runnable"])
        self.assertFalse(config["candidate_profile"]["propeller_control_authorized"])
        self.assertFalse(config["control_authorization"]["enabled"])
        self.assertEqual(config["control_transport_release"]["status"], "blocked")
        self.assertEqual(config["rc_mapping"]["throttle_hover_us"], 1275)
        self.assertEqual(config["rc_mapping"]["yaw_command_limit_deg_s"], 0.0)
        self.assertEqual(config["runtime_policy"]["allowed_control_modes"], ["log_only"])

    def test_supervised_flight_profile_uses_measured_variable_thrust(self):
        config = self.flight_supervised
        self.assertEqual(
            config["candidate_profile"]["scope"],
            "flight_noncollision_supervised_v2",
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
        self.assertEqual(config["safety"]["min_vbat_v"], 22)
        self.assertEqual(config["safety"]["max_vbat_v"], 25.2)
        self.assertEqual(
            config["guidance"]["velocity_establishing_png"]["total_accel_limit_m_s2"],
            7,
        )
        fov_priority = config["guidance"]["velocity_establishing_png"][
            "fov_priority"
        ]
        self.assertTrue(fov_priority["enabled"])
        self.assertEqual(fov_priority["start_ratio"], 0.75)
        self.assertEqual(fov_priority["full_ratio"], 0.95)
        thrust = config["guidance_command"]["accel_tilt_rate"]["thrust_feedforward"]
        self.assertTrue(thrust["enabled"])
        self.assertEqual(thrust["model"], "voltage_throttle_lut")
        self.assertEqual(thrust["calibration_id"], "PENDING_FULL_6S_THRUST_LUT")
        self.assertEqual(config["control_authorization"]["minimum_approval_schema_version"], 4)
        self.assertTrue(
            config["control_authorization"]["finalized_run_evidence_required"]
        )
        self.assertEqual(
            config["logging"]["evidence_frames"],
            {"enabled": True, "max_fps": 5, "jpeg_quality": 80},
        )
        self.assertTrue(
            config["control_authorization"]["thrust_model_evidence_required"]
        )
        takeover = config["safety"]["takeover_duration_interlock"]
        self.assertTrue(takeover["enabled"])
        self.assertEqual(takeover["max_duration_s"], 2.0)
        self.assertTrue(takeover["latch_until_disarm"])
        self.assertEqual(takeover["max_takeovers_per_arm"], 1)
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
        base = {
            "control_mode": "msp_raw_rc",
            "allow_control": True,
            "detector_source": "rknn_bytetrack",
            "isolate_rknn_process": True,
            "main_cpu_affinity": "6,7",
            "rknn_cpu_affinity": "4,5",
        }
        runner._validate_runtime_policy(
            self.flight_supervised,
            argparse.Namespace(**base),
        )

        with self.assertRaisesRegex(RuntimeError, "requires --isolate-rknn-process"):
            runner._validate_runtime_policy(
                self.flight_supervised,
                argparse.Namespace(**dict(base, isolate_rknn_process=False)),
            )


if __name__ == "__main__":
    unittest.main()
