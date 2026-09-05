import unittest
from collections import deque
import hashlib
import json
import tempfile
from pathlib import Path

import numpy as np

from vision_guidance.betaflight_png_sim import (
    MATRIX15_CASES,
    ClosedLoopSimulationConfig,
    MatrixCase,
    _delayed_body_rate_command,
    _rotation_matrix_frd,
    simulate_case,
    simulate_matrix15,
)


class BetaflightPngClosedLoopSimulationTest(unittest.TestCase):
    def test_body_rate_delay_interpolates_between_simulation_steps(self):
        history = deque([(0.0, 0.0, 0.0), (0.01, 10.0, -20.0)])

        delayed = _delayed_body_rate_command(history, 0.005)

        self.assertEqual(delayed, (5.0, -10.0))

    def test_body_rate_delay_must_be_non_negative(self):
        with self.assertRaisesRegex(ValueError, "body_rate_command_delay_s"):
            ClosedLoopSimulationConfig(body_rate_command_delay_s=-0.001)

    def test_runtime_fidelity_models_held_control_entry_and_throttle(self):
        with tempfile.TemporaryDirectory() as directory:
            model_path, model_sha256 = self._write_thrust_model(Path(directory))
            config = ClosedLoopSimulationConfig(
                duration_s=1.0,
                dt_s=0.01,
                control_rate_hz=50.0,
                entry_handoff_enabled=True,
                entry_handoff_duration_s=0.8,
                throttle_dynamics_enabled=True,
                thrust_response_tau_s=0.08,
                throttle_handover_duration_s=0.8,
                throttle_slew_limit_us_per_s=600.0,
                battery_voltage_v=22.6,
                thrust_model_path=str(model_path),
                thrust_model_sha256=model_sha256,
                thrust_model_calibration_id="simulation-unit-test",
                perception_rate_hz=30.0,
                kinematic_rate_hz=5.0,
                kinematic_latency_s=0.0,
                kinematic_dropout_probability=0.0,
                kinematic_velocity_noise_std_m_s=0.0,
                candidate_acquire_consecutive_frames=1,
            )

            result = simulate_case(
                MATRIX15_CASES[0],
                controller_mode="candidate_velocity_hold_variable_thrust",
                start_profile="hover",
                config=config,
            )

        self.assertEqual(result.control_update_count, 50)
        self.assertGreater(result.entry_handoff_active_fraction, 0.7)
        self.assertGreater(result.throttle_handover_active_fraction, 0.7)
        self.assertEqual(result.throttle_slew_saturation_fraction, 0.0)
        self.assertGreater(result.maximum_throttle_us, config.throttle_hover_us)
        self.assertLessEqual(result.maximum_throttle_us, config.throttle_max_us)
        self.assertLessEqual(result.maximum_load_factor_g, config.max_load_factor_g)

    def test_legacy_simulation_leaves_throttle_diagnostics_unset(self):
        result = simulate_case(
            MATRIX15_CASES[0],
            controller_mode="fixed_thrust",
            start_profile="hover",
            config=ClosedLoopSimulationConfig(duration_s=0.05),
        )

        self.assertIsNone(result.minimum_throttle_us)
        self.assertIsNone(result.maximum_throttle_us)
        self.assertIsNone(result.maximum_load_factor_g)

    def test_enabled_runtime_dynamics_require_explicit_positive_parameters(self):
        with self.assertRaisesRegex(ValueError, "entry handoff"):
            ClosedLoopSimulationConfig(entry_handoff_enabled=True)
        with self.assertRaisesRegex(ValueError, "throttle slew"):
            ClosedLoopSimulationConfig(throttle_dynamics_enabled=True)
        with self.assertRaisesRegex(ValueError, "battery_voltage_v"):
            ClosedLoopSimulationConfig(
                throttle_dynamics_enabled=True,
                thrust_response_tau_s=0.08,
                throttle_slew_limit_us_per_s=600.0,
            )

    def test_runtime_dynamics_reject_missing_or_uncovered_lut(self):
        with tempfile.TemporaryDirectory() as directory:
            model_path, model_sha256 = self._write_thrust_model(Path(directory))
            common = {
                "duration_s": 0.05,
                "throttle_dynamics_enabled": True,
                "thrust_response_tau_s": 0.08,
                "throttle_slew_limit_us_per_s": 600.0,
                "battery_voltage_v": 22.6,
                "thrust_model_path": str(model_path),
                "thrust_model_sha256": model_sha256,
                "thrust_model_calibration_id": "simulation-unit-test",
            }
            with self.assertRaisesRegex(ValueError, "SHA256"):
                simulate_case(
                    MATRIX15_CASES[0],
                    controller_mode="candidate_velocity_hold_variable_thrust",
                    start_profile="hover",
                    config=ClosedLoopSimulationConfig(
                        **{**common, "thrust_model_sha256": "0" * 64}
                    ),
                )
            with self.assertRaisesRegex(ValueError, "outside thrust LUT coverage"):
                simulate_case(
                    MATRIX15_CASES[0],
                    controller_mode="candidate_velocity_hold_variable_thrust",
                    start_profile="hover",
                    config=ClosedLoopSimulationConfig(
                        **{**common, "battery_voltage_v": 19.9}
                    ),
                )

    def test_matrix_contains_reported_fifteen_cases(self):
        self.assertEqual(len(MATRIX15_CASES), 15)
        self.assertEqual(MATRIX15_CASES[0].case_id, "M01")
        self.assertEqual(MATRIX15_CASES[-1].case_id, "M15")
        self.assertEqual(MATRIX15_CASES[-1].lateral_offset_m, 20.0)

    def test_rotation_matrix_is_orthonormal(self):
        rotation = _rotation_matrix_frd(0.2, -0.3, 0.5)
        self.assertTrue(np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-12))
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0)

    def test_fixed_thrust_changes_altitude_while_ideal_altitude_hold_does_not(self):
        case = MatrixCase("test", 25.0, -10.0, 30.0, 5.0)
        config = ClosedLoopSimulationConfig(duration_s=3.0, dt_s=0.01)
        fixed = simulate_case(
            case,
            controller_mode="fixed_thrust",
            start_profile="hover",
            config=config,
        )
        ideal = simulate_case(
            case,
            controller_mode="ideal_altitude_hold",
            start_profile="hover",
            config=config,
        )

        self.assertGreater(fixed.maximum_vertical_displacement_m, 0.01)
        self.assertLess(ideal.maximum_vertical_displacement_m, 1.0e-9)

    def test_speed_hold_path_closes_from_hover_better_than_current_fixed_thrust(self):
        case = MatrixCase("test", 25.0, -10.0, 30.0, 5.0)
        config = ClosedLoopSimulationConfig(duration_s=12.0, dt_s=0.01)
        fixed = simulate_case(
            case,
            controller_mode="fixed_thrust",
            start_profile="hover",
            config=config,
        )
        complete = simulate_case(
            case,
            controller_mode="speed_hold_variable_thrust",
            start_profile="hover",
            config=config,
        )

        self.assertLess(complete.minimum_range_m, fixed.minimum_range_m)
        self.assertGreater(complete.maximum_speed_m_s, fixed.maximum_speed_m_s)

    def test_matrix_report_is_deterministic_and_has_expected_result_count(self):
        config = ClosedLoopSimulationConfig(duration_s=0.05, dt_s=0.01)
        first = simulate_matrix15(config=config)
        second = simulate_matrix15(config=config)

        self.assertEqual(first["schema_version"], 2)
        self.assertEqual(len(first["results"]), 120)
        self.assertEqual(len(first["summaries"]), 8)
        self.assertEqual(first, second)

    def test_candidate_uses_sampled_los_and_delayed_noisy_own_velocity(self):
        config = ClosedLoopSimulationConfig(
            duration_s=8.0,
            dt_s=0.01,
            perception_rate_hz=30.0,
            perception_latency_s=0.1,
            perception_fov_gate_enabled=True,
            kinematic_rate_hz=5.0,
            kinematic_latency_s=0.15,
            kinematic_velocity_noise_std_m_s=0.25,
            kinematic_dropout_probability=0.0,
        )

        result = simulate_case(
            MATRIX15_CASES[0],
            controller_mode="candidate_velocity_hold_variable_thrust",
            start_profile="hover",
            config=config,
        )

        self.assertGreater(result.measurement_delivered_count, 5)
        self.assertGreater(result.kinematic_valid_fraction, 0.8)
        self.assertGreater(result.maximum_kinematic_velocity_error_m_s, 0.0)
        self.assertEqual(result.controller_final_phase, "COMPLETE")
        self.assertNotEqual(result.outcome_reason, "controller_abort")

    def test_candidate_does_not_consume_relative_velocity_measurement(self):
        base = dict(
            duration_s=2.0,
            dt_s=0.01,
            perception_rate_hz=30.0,
            perception_latency_s=0.1,
            random_seed=42,
        )
        clean = simulate_case(
            MATRIX15_CASES[0],
            controller_mode="candidate_velocity_hold_variable_thrust",
            start_profile="hover",
            config=ClosedLoopSimulationConfig(**base, relative_velocity_noise_std_m_s=0.0),
        )
        noisy = simulate_case(
            MATRIX15_CASES[0],
            controller_mode="candidate_velocity_hold_variable_thrust",
            start_profile="hover",
            config=ClosedLoopSimulationConfig(**base, relative_velocity_noise_std_m_s=100.0),
        )

        self.assertEqual(clean, noisy)

    def test_sampled_measurement_is_unavailable_until_latency_expires(self):
        config = ClosedLoopSimulationConfig(
            duration_s=0.5,
            dt_s=0.01,
            perception_latency_s=0.2,
            perception_rate_hz=30.0,
            perception_stale_timeout_s=0.35,
        )
        result = simulate_case(
            MATRIX15_CASES[0],
            controller_mode="fixed_thrust",
            start_profile="hover",
            config=config,
        )

        self.assertAlmostEqual(result.first_measurement_valid_time_s, 0.2)
        self.assertGreater(result.measurement_capture_count, 10)
        self.assertGreater(result.measurement_delivered_count, 0)
        self.assertLess(result.measurement_valid_fraction, 0.7)
        self.assertGreaterEqual(result.maximum_measurement_age_s, 0.2)
        self.assertLess(result.maximum_measurement_age_s, 0.24)

    def test_empirical_burst_dropout_uses_configured_run_lengths(self):
        config = ClosedLoopSimulationConfig(
            duration_s=1.0,
            dt_s=0.01,
            perception_rate_hz=30.0,
            measurement_dropout_burst_start_probability=1.0,
            measurement_dropout_burst_lengths=(2,),
        )
        result = simulate_case(
            MATRIX15_CASES[0],
            controller_mode="fixed_thrust",
            start_profile="hover",
            config=config,
        )

        self.assertEqual(result.measurement_dropout_count, result.measurement_capture_count)
        self.assertEqual(result.measurement_delivered_count, 0)
        self.assertEqual(result.measurement_dropout_fraction, 1.0)

    def test_burst_dropout_configuration_is_explicit_and_validated(self):
        with self.assertRaisesRegex(ValueError, "burst_lengths are required"):
            ClosedLoopSimulationConfig(
                measurement_dropout_burst_start_probability=0.1,
            )
        with self.assertRaisesRegex(ValueError, "cannot both be enabled"):
            ClosedLoopSimulationConfig(
                measurement_dropout_probability=0.1,
                measurement_dropout_burst_start_probability=0.1,
                measurement_dropout_burst_lengths=(1,),
            )
        with self.assertRaisesRegex(ValueError, "positive integers"):
            ClosedLoopSimulationConfig(
                measurement_dropout_burst_lengths=(0,),
            )

    def test_candidate_acquisition_count_is_configurable(self):
        common = dict(
            duration_s=0.25,
            dt_s=0.01,
            perception_rate_hz=30.0,
            kinematic_rate_hz=30.0,
            kinematic_latency_s=0.0,
            kinematic_dropout_probability=0.0,
            kinematic_velocity_noise_std_m_s=0.0,
        )
        fast = simulate_case(
            MATRIX15_CASES[0],
            controller_mode="candidate_velocity_hold_variable_thrust",
            start_profile="hover",
            config=ClosedLoopSimulationConfig(
                **common,
                candidate_acquire_consecutive_frames=1,
            ),
        )
        slow = simulate_case(
            MATRIX15_CASES[0],
            controller_mode="candidate_velocity_hold_variable_thrust",
            start_profile="hover",
            config=ClosedLoopSimulationConfig(
                **common,
                candidate_acquire_consecutive_frames=10,
            ),
        )

        self.assertGreater(fast.maximum_control_accel_m_s2, 0.0)
        self.assertLess(slow.maximum_control_accel_m_s2, fast.maximum_control_accel_m_s2)

    def test_fov_gate_does_not_supply_truth_los_for_unseen_target(self):
        case = MatrixCase("out_of_fov", 40.0, 0.0, 1.0, 0.1)
        config = ClosedLoopSimulationConfig(
            duration_s=0.2,
            dt_s=0.01,
            camera_half_fov_deg=10.0,
            perception_rate_hz=30.0,
            perception_fov_gate_enabled=True,
        )
        result = simulate_case(
            case,
            controller_mode="speed_hold_variable_thrust",
            start_profile="hover",
            config=config,
        )

        self.assertFalse(result.initial_target_in_fov)
        self.assertEqual(result.measurement_delivered_count, 0)
        self.assertEqual(result.measurement_valid_fraction, 0.0)
        self.assertEqual(
            result.measurement_fov_reject_count, result.measurement_capture_count
        )
        self.assertEqual(result.outcome_reason, "initial_target_out_of_fov")

    def test_rectangular_fov_uses_hardware_vertical_limit(self):
        case = MatrixCase("hardware_fov", 40.0, 0.0, 30.0, 1.0)
        legacy = simulate_case(
            case,
            controller_mode="fixed_thrust",
            start_profile="hover",
            config=ClosedLoopSimulationConfig(duration_s=0.05),
        )
        hardware = simulate_case(
            case,
            controller_mode="fixed_thrust",
            start_profile="hover",
            config=ClosedLoopSimulationConfig(
                duration_s=0.05,
                camera_horizontal_half_fov_deg=30.9,
                camera_vertical_half_fov_deg=24.9,
            ),
        )

        self.assertTrue(legacy.initial_target_in_fov)
        self.assertFalse(hardware.initial_target_in_fov)

    def test_candidate_fixed_vm_does_not_depend_on_target_truth_speed(self):
        config = ClosedLoopSimulationConfig(
            duration_s=0.05,
            candidate_fixed_vm_m_s=10.0,
        )
        slow = simulate_case(
            MatrixCase("slow", 10.0, 0.0, 30.0, 3.0),
            controller_mode="candidate_velocity_hold_variable_thrust",
            start_profile="hover",
            config=config,
        )
        fast = simulate_case(
            MatrixCase("fast", 10.0, 0.0, 30.0, 7.0),
            controller_mode="candidate_velocity_hold_variable_thrust",
            start_profile="hover",
            config=config,
        )

        self.assertEqual(slow.fixed_vm_m_s, 10.0)
        self.assertEqual(fast.fixed_vm_m_s, 10.0)

    def test_rejects_latency_above_stale_timeout(self):
        with self.assertRaisesRegex(ValueError, "latency cannot exceed"):
            ClosedLoopSimulationConfig(
                perception_latency_s=0.4,
                perception_stale_timeout_s=0.35,
            )

    def test_out_of_fov_geometry_is_not_counted_as_visual_feasible_hit(self):
        result = simulate_case(
            MATRIX15_CASES[13],
            controller_mode="speed_hold_variable_thrust",
            start_profile="hover",
        )

        self.assertTrue(result.hit)
        self.assertFalse(result.target_continuously_in_fov)
        self.assertFalse(result.fov_feasible_hit)
        self.assertGreater(result.maximum_target_off_up_axis_deg, 60.0)

    @staticmethod
    def _write_thrust_model(directory: Path) -> tuple[Path, str]:
        path = directory / "thrust_lut.json"
        values = {
            "schema_version": 1,
            "model_type": "voltage_throttle_specific_force_lut",
            "calibration_id": "simulation-unit-test",
            "voltage_v": [20.0, 25.2],
            "throttle_us": [1200.0, 1275.0, 1500.0],
            "specific_force_m_s2": [
                [4.0, 9.80665, 20.0],
                [4.5, 10.2, 22.0],
            ],
            "validation": {
                "passed": True,
                "median_relative_error": 0.05,
                "p95_relative_error": 0.15,
            },
        }
        path.write_text(json.dumps(values), encoding="utf-8")
        return path, hashlib.sha256(path.read_bytes()).hexdigest()

    def test_rejects_unknown_mode(self):
        with self.assertRaisesRegex(ValueError, "unsupported controller mode"):
            simulate_case(
                MATRIX15_CASES[0],
                controller_mode="unknown",
                start_profile="hover",
            )


if __name__ == "__main__":
    unittest.main()
