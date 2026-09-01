import unittest

import numpy as np

from vision_guidance.betaflight_png_sim import (
    MATRIX15_CASES,
    ClosedLoopSimulationConfig,
    MatrixCase,
    _rotation_matrix_frd,
    simulate_case,
    simulate_matrix15,
)


class BetaflightPngClosedLoopSimulationTest(unittest.TestCase):
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
        self.assertIn(result.controller_final_phase, {"ACCELERATE", "PNG_TRACK"})
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

    def test_rejects_unknown_mode(self):
        with self.assertRaisesRegex(ValueError, "unsupported controller mode"):
            simulate_case(
                MATRIX15_CASES[0],
                controller_mode="unknown",
                start_profile="hover",
            )


if __name__ == "__main__":
    unittest.main()
