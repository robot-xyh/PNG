import unittest
import hashlib
import json
from pathlib import Path
import tempfile

from tools.run_betaflight_intercept_monte_carlo import (
    _bind_runtime_config,
    _build_tasks,
    _cases,
    _initial_performance_verdict,
    _release_policy_verdict,
    _paired_screening_verdict,
)
from vision_guidance.betaflight_intercept_eval import (
    InterceptionAcceptanceCriteria,
    evaluate_interception_results,
)
from vision_guidance.betaflight_png_sim import (
    MATRIX15_CASES,
    ClosedLoopSimulationConfig,
    simulate_case,
)


class BetaflightInterceptionEvaluationTest(unittest.TestCase):
    def test_initial_performance_verdict_is_separate_from_release(self):
        summaries = [
            {
                "initially_visible_hit_rate": 0.9692,
                "initially_visible_fov_hit_rate": 0.8654,
            },
            {
                "initially_visible_hit_rate": 0.9731,
                "initially_visible_fov_hit_rate": 0.8938,
            },
        ]
        verdict = _initial_performance_verdict(
            {
                "initially_visible_hit_rate_min": 0.80,
                "initially_visible_fov_hit_rate_min": 0.80,
                "description": "initial model-performance target",
            },
            summaries,
        )

        self.assertIsNotNone(verdict)
        self.assertTrue(verdict["passed"])
        self.assertAlmostEqual(
            verdict["checks"]["initially_visible_hit_rate"]["observed_minimum"],
            0.9692,
        )
        self.assertAlmostEqual(
            verdict["checks"]["initially_visible_fov_hit_rate"][
                "observed_minimum"
            ],
            0.8654,
        )
        self.assertTrue(verdict["does_not_imply_release"])

        failing = _initial_performance_verdict(
            {"initially_visible_hit_rate_min": 0.98}, summaries
        )
        self.assertFalse(failing["passed"])
        with self.assertRaisesRegex(ValueError, "must be in"):
            _initial_performance_verdict(
                {"initially_visible_hit_rate_min": 1.1}, summaries
            )
        with self.assertRaisesRegex(ValueError, "must be numeric"):
            _initial_performance_verdict(
                {
                    "initially_visible_hit_rate_min": 0.8,
                    "initially_visible_fov_hit_rate_min": "invalid",
                },
                summaries,
            )

    def test_explicit_case_matrix_is_validated(self):
        cases = _cases(
            [
                {
                    "case_id": "U01",
                    "horizontal_range_m": 10.0,
                    "lateral_offset_m": 0.0,
                    "altitude_offset_m": 30.0,
                    "target_speed_m_s": 5.0,
                }
            ]
        )

        self.assertEqual(len(cases), 1)
        self.assertEqual(cases[0].case_id, "U01")
        with self.assertRaisesRegex(ValueError, "duplicate case_id"):
            _cases([dict(vars(cases[0])), dict(vars(cases[0]))])

    def test_case_matrix_can_be_mirrored_with_course_and_offset(self):
        cases = _cases(
            [
                {
                    "case_id": "U01",
                    "horizontal_range_m": 10.0,
                    "lateral_offset_m": 3.0,
                    "altitude_offset_m": 30.0,
                    "target_speed_m_s": 5.0,
                    "target_course_deg": 75.0,
                }
            ],
            mirror=True,
        )

        self.assertEqual(len(cases), 2)
        self.assertEqual(cases[1].case_id, "U01M")
        self.assertEqual(cases[1].lateral_offset_m, -3.0)
        self.assertEqual(cases[1].target_course_deg, 285.0)

    def test_runtime_binding_derives_fidelity_parameters_and_rejects_drift(self):
        source_path = Path(
            "config/betaflight.rk3588.velocity_png.flight_supervised.json"
        ).resolve()
        runtime = json.loads(source_path.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model_path = root / "thrust_lut.json"
            model = self._thrust_model_values()
            model_path.write_text(json.dumps(model), encoding="utf-8")
            thrust = runtime["guidance_command"]["accel_tilt_rate"]["thrust_feedforward"]
            thrust["model_path"] = str(model_path)
            thrust["model_sha256"] = hashlib.sha256(model_path.read_bytes()).hexdigest()
            thrust["calibration_id"] = model["calibration_id"]
            runtime_path = root / "runtime.json"
            runtime_path.write_text(json.dumps(runtime), encoding="utf-8")
            digest = hashlib.sha256(runtime_path.read_bytes()).hexdigest()
            bound, metadata = _bind_runtime_config(
                {"battery_voltage_v": 22.6},
                {"config": str(runtime_path), "sha256": digest},
            )

            self.assertEqual(bound["control_rate_hz"], 50.0)
            self.assertEqual(bound["perception_stale_timeout_s"], 0.15)
            self.assertEqual(bound["perception_result_age_limit_s"], 0.20)
            self.assertEqual(bound["entry_handoff_duration_s"], 0.8)
            self.assertEqual(bound["throttle_handover_duration_s"], 0.8)
            self.assertEqual(bound["throttle_slew_limit_us_per_s"], 600.0)
            self.assertTrue(bound["throttle_dynamics_enabled"])
            self.assertEqual(bound["thrust_model_sha256"], thrust["model_sha256"])
            self.assertAlmostEqual(bound["max_load_factor_g"], 2.37)
            self.assertGreater(bound["camera_horizontal_half_fov_deg"], 30.0)
            self.assertEqual(metadata["sha256"], digest)
            self.assertEqual(
                metadata["thrust_model"]["calibration_id"],
                model["calibration_id"],
            )

            with self.assertRaisesRegex(ValueError, "disagree"):
                _bind_runtime_config(
                    {"battery_voltage_v": 22.6, "control_rate_hz": 100.0},
                    {"config": str(runtime_path), "sha256": digest},
                )
            with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
                _bind_runtime_config(
                    {"battery_voltage_v": 22.6},
                    {"config": str(runtime_path), "sha256": "0" * 64},
                )

            narrow = self._thrust_model_values()
            narrow["voltage_v"] = [22.5, 25.2]
            narrow_path = root / "narrow_lut.json"
            narrow_path.write_text(json.dumps(narrow), encoding="utf-8")
            thrust["model_path"] = str(narrow_path)
            thrust["model_sha256"] = hashlib.sha256(narrow_path.read_bytes()).hexdigest()
            runtime_path.write_text(json.dumps(runtime), encoding="utf-8")
            narrow_digest = hashlib.sha256(runtime_path.read_bytes()).hexdigest()
            with self.assertRaisesRegex(ValueError, "22.0-25.2"):
                _bind_runtime_config(
                    {"battery_voltage_v": 22.6},
                    {"config": str(runtime_path), "sha256": narrow_digest},
                )

    def test_scenario_seed_is_stable_between_full_and_subset_runs(self):
        target = {"name": "target", "perception_latency_s": 0.1}
        evaluations = [
            {
                "name": "current",
                "controller_mode": "fixed_thrust",
                "start_profile": "hover",
            }
        ]
        common = {
            "base_simulation": {},
            "evaluations": evaluations,
            "trials_per_case": 2,
            "base_seed": 1234,
        }

        full = _build_tasks(
            scenarios=[{"name": "other"}, target],
            **common,
        )
        subset = _build_tasks(scenarios=[target], **common)
        selected = [task for task in full if task["scenario_name"] == "target"]

        self.assertEqual(
            [
                (task["trial_index"], task["random_seed"], task["case"].case_id)
                for task in selected
            ],
            [
                (task["trial_index"], task["random_seed"], task["case"].case_id)
                for task in subset
            ],
        )

    def test_evaluations_use_paired_random_seeds(self):
        tasks = _build_tasks(
            base_simulation={
                "camera_horizontal_half_fov_deg": 30.0,
                "camera_vertical_half_fov_deg": 25.0,
            },
            scenarios=[{"name": "paired"}],
            evaluations=[
                {
                    "name": "baseline",
                    "controller_mode": "candidate_velocity_hold_variable_thrust",
                    "start_profile": "hover",
                },
                {
                    "name": "candidate",
                    "controller_mode": "candidate_velocity_hold_variable_thrust",
                    "start_profile": "hover",
                    "simulation_overrides": {
                        "candidate_fov_priority_enabled": True,
                    },
                },
            ],
            trials_per_case=2,
            base_seed=42,
            cases=(MATRIX15_CASES[0],),
        )

        self.assertEqual(
            [task["random_seed"] for task in tasks[:2]],
            [task["random_seed"] for task in tasks[2:]],
        )

    def test_paired_screening_selects_only_non_regressing_fov_candidate(self):
        cases = (
            MATRIX15_CASES[5],
            MATRIX15_CASES[4],
        )
        scenarios = [{"name": "latency"}]
        evaluations = [
            {
                "name": "baseline",
                "controller_mode": "candidate_velocity_hold_variable_thrust",
                "start_profile": "hover",
            },
            {
                "name": "candidate",
                "controller_mode": "candidate_velocity_hold_variable_thrust",
                "start_profile": "hover",
                "simulation_overrides": {
                    "candidate_fov_priority_enabled": True,
                },
            },
        ]
        rows = []
        for evaluation in ("baseline", "candidate"):
            for case in cases:
                for trial in range(10):
                    outward = case.case_id == MATRIX15_CASES[5].case_id
                    baseline_fov_hit = not outward or trial < 5
                    candidate_fov_hit = not outward or trial < 7
                    rows.append(
                        {
                            "scenario_name": "latency",
                            "evaluation_name": evaluation,
                            "case_id": case.case_id,
                            "trial_index": trial,
                            "random_seed": trial,
                            "initial_target_in_fov": True,
                            "hit": True,
                            "fov_feasible_hit": (
                                baseline_fov_hit
                                if evaluation == "baseline"
                                else candidate_fov_hit
                            ),
                        }
                    )

        result = _paired_screening_verdict(
            {
                "baseline_evaluation": "baseline",
                "outward_fov_improvement_min": 0.1,
            },
            rows=rows,
            cases=cases,
            scenarios=scenarios,
            evaluations=evaluations,
        )

        self.assertTrue(result["passed"])
        self.assertEqual(result["selected_evaluation"], "candidate")

    def test_seeded_disturbance_simulation_is_repeatable(self):
        config = ClosedLoopSimulationConfig(
            duration_s=0.5,
            perception_rate_hz=30.0,
            perception_fov_gate_enabled=True,
            measurement_dropout_probability=0.2,
            los_angle_noise_std_deg=0.5,
            relative_velocity_noise_std_m_s=0.5,
            wind_accel_std_m_s2=0.5,
            random_seed=1234,
        )

        first = simulate_case(
            MATRIX15_CASES[0],
            controller_mode="speed_hold_variable_thrust",
            start_profile="hover",
            config=config,
        )
        second = simulate_case(
            MATRIX15_CASES[0],
            controller_mode="speed_hold_variable_thrust",
            start_profile="hover",
            config=config,
        )

        self.assertEqual(first, second)
        self.assertGreater(first.maximum_measurement_angle_error_deg, 0.0)
        self.assertGreater(first.maximum_relative_velocity_error_m_s, 0.0)
        self.assertGreater(first.maximum_wind_accel_m_s2, 0.0)

    def test_release_policy_separates_contact_performance_from_noncollision(self):
        scenarios = [{"name": "latency"}]
        evaluations = [
            {
                "name": "contact",
                "evidence_role": "contact_performance",
                "required_for_release": True,
            },
            {
                "name": "safety",
                "evidence_role": "noncollision_safety",
                "required_for_release": True,
            },
        ]
        summaries = [
            {
                "scenario_name": "latency",
                "evaluation_name": "contact",
                "evidence_role": "contact_performance",
                "engagement_policy": "contact",
                "passed": True,
            },
            {
                "scenario_name": "latency",
                "evaluation_name": "safety",
                "evidence_role": "noncollision_safety",
                "engagement_policy": "noncollision",
                "passed": False,
            },
        ]
        rows = [
            {
                "scenario_name": "latency",
                "evaluation_name": "safety",
                "initial_target_in_fov": True,
                "controller_abort_time_s": 0.2,
                "hit": True,
                "elapsed_s": 1.1,
            },
            {
                "scenario_name": "latency",
                "evaluation_name": "safety",
                "initial_target_in_fov": True,
                "controller_abort_time_s": 0.3,
                "hit": False,
                "elapsed_s": 2.0,
            },
        ]

        result = _release_policy_verdict(
            config_schema_version=2,
            rows=rows,
            summaries=summaries,
            scenarios=scenarios,
            evaluations=evaluations,
            paired_screening={"passed": True, "selected_evaluation": "contact"},
            noncollision_acceptance={
                "timely_abort_rate_min": 0.99,
                "unsafe_contact_rate_max": 0.01,
                "minimum_abort_lead_time_s": 0.75,
            },
            runtime_binding={
                "derived_simulation": {
                    "candidate_engagement_policy": "noncollision"
                }
            },
        )

        self.assertTrue(result["passed"])
        self.assertTrue(result["contact_performance"]["passed"])
        self.assertTrue(result["noncollision_safety"]["passed"])
        self.assertFalse(
            result["contact_performance"]["authorizes_contact_flight"]
        )

    def test_contact_release_policy_reports_performance_without_authorizing_flight(self):
        scenarios = [{"name": "high_voltage"}, {"name": "low_voltage"}]
        evaluations = [
            {
                "name": "contact",
                "evidence_role": "contact_performance",
                "required_for_release": True,
            }
        ]
        summaries = [
            {
                "scenario_name": scenario["name"],
                "evaluation_name": "contact",
                "evidence_role": "contact_performance",
                "engagement_policy": "contact",
                "passed": True,
            }
            for scenario in scenarios
        ]

        result = _release_policy_verdict(
            config_schema_version=2,
            rows=[],
            summaries=summaries,
            scenarios=scenarios,
            evaluations=evaluations,
            paired_screening=None,
            noncollision_acceptance=None,
            runtime_binding={
                "derived_simulation": {
                    "candidate_engagement_policy": "contact"
                }
            },
        )

        self.assertTrue(result["passed"])
        self.assertEqual(result["runtime_engagement_policy"], "contact")
        self.assertIsNone(result["noncollision_safety"])
        self.assertTrue(result["contact_mc_is_performance_evidence_only"])
        self.assertFalse(result["contact_performance"]["authorizes_contact_flight"])

    def test_full_dropout_never_delivers_visible_measurement(self):
        config = ClosedLoopSimulationConfig(
            duration_s=0.2,
            perception_rate_hz=30.0,
            perception_fov_gate_enabled=True,
            measurement_dropout_probability=1.0,
        )
        result = simulate_case(
            MATRIX15_CASES[0],
            controller_mode="fixed_thrust",
            start_profile="hover",
            config=config,
        )

        self.assertTrue(result.initial_target_in_fov)
        self.assertEqual(result.measurement_fov_reject_count, 0)
        self.assertEqual(result.measurement_dropout_count, result.measurement_capture_count)
        self.assertEqual(result.measurement_dropout_fraction, 1.0)
        self.assertEqual(result.measurement_delivered_count, 0)
        self.assertEqual(result.measurement_valid_fraction, 0.0)

    def test_acceptance_passes_only_when_every_threshold_passes(self):
        criteria = InterceptionAcceptanceCriteria()
        passing_rows = [self._row(case_id=f"M{index:02d}") for index in range(1, 13)]

        passing = evaluate_interception_results(passing_rows, criteria)
        self.assertTrue(passing["passed"])
        self.assertEqual(passing["initially_visible_hit_rate"], 1.0)

        failing_rows = [dict(row) for row in passing_rows]
        failing_rows[0]["hit"] = False
        failing_rows[0]["fov_feasible_hit"] = False
        failing_rows[0]["minimum_range_m"] = 2.0
        failing_rows[0]["outcome_reason"] = "target_stale"
        failing = evaluate_interception_results(failing_rows, criteria)

        self.assertFalse(failing["passed"])
        self.assertFalse(failing["checks"]["initially_visible_hit_rate"]["passed"])
        self.assertFalse(failing["checks"]["target_stale_failure_rate"]["passed"])
        self.assertFalse(failing["checks"]["worst_minimum_range_m"]["passed"])

        invalid_state_rows = [dict(row) for row in passing_rows]
        invalid_state_rows[0]["kinematic_valid_fraction"] = 0.0
        invalid_state_rows[1]["kinematic_valid_fraction"] = 0.0
        invalid_state = evaluate_interception_results(invalid_state_rows, criteria)
        self.assertFalse(invalid_state["passed"])
        self.assertFalse(
            invalid_state["checks"]["mean_kinematic_valid_fraction"]["passed"]
        )

        candidate_stale_rows = [dict(row) for row in passing_rows]
        candidate_stale_rows[0]["hit"] = False
        candidate_stale_rows[0]["fov_feasible_hit"] = False
        candidate_stale_rows[0]["outcome_reason"] = "controller_abort"
        candidate_stale_rows[0]["controller_final_reason"] = "detection_stale"
        candidate_stale = evaluate_interception_results(candidate_stale_rows, criteria)
        self.assertEqual(candidate_stale["target_stale_failure_count"], 1)
        self.assertFalse(
            candidate_stale["checks"]["target_stale_failure_rate"]["passed"]
        )

    def test_probabilistic_release_can_report_worst_range_without_gating(self):
        criteria = InterceptionAcceptanceCriteria(
            initially_visible_hit_rate_min=0.8,
            initially_visible_fov_hit_rate_min=0.8,
            worst_minimum_range_m_max=None,
        )
        rows = [self._row(case_id=f"M{index:02d}") for index in range(1, 6)]
        rows[-1]["hit"] = False
        rows[-1]["fov_feasible_hit"] = False
        rows[-1]["minimum_range_m"] = 1.4
        rows[-1]["outcome_reason"] = "miss"

        result = evaluate_interception_results(rows, criteria)

        self.assertTrue(result["passed"])
        self.assertEqual(result["initially_visible_hit_rate"], 0.8)
        self.assertEqual(result["initially_visible_fov_hit_rate"], 0.8)
        self.assertEqual(
            result["checks"]["worst_minimum_range_m"]["operator"],
            "report_only",
        )
        self.assertFalse(result["checks"]["worst_minimum_range_m"]["required"])

    @staticmethod
    def _row(*, case_id):
        return {
            "case_id": case_id,
            "trial_index": 0,
            "initial_target_in_fov": True,
            "hit": True,
            "fov_feasible_hit": True,
            "outcome_reason": "hit",
            "controller_final_reason": "active",
            "measurement_valid_fraction": 1.0,
            "kinematic_valid_fraction": 1.0,
            "minimum_range_m": 0.95,
            "tilt_saturation_fraction": 0.01,
            "rate_saturation_fraction": 0.01,
            "speed_hold_accel_saturation_fraction": 0.01,
            "total_accel_saturation_fraction": 0.01,
        }

    @staticmethod
    def _thrust_model_values():
        return {
            "schema_version": 1,
            "model_type": "voltage_throttle_specific_force_lut",
            "calibration_id": "mc-unit-test-lut",
            "voltage_v": [22.0, 25.2],
            "throttle_us": [1200.0, 1275.0, 1500.0],
            "specific_force_m_s2": [
                [4.0, 9.5, 20.0],
                [4.5, 10.2, 22.0],
            ],
            "validation": {
                "passed": True,
                "sample_count": 200,
                "median_relative_error": 0.05,
                "p95_relative_error": 0.15,
            },
            "dynamics": {
                "model": "first_order_specific_force",
                "first_order_time_constant_s": 0.08,
                "fit_sample_count": 600,
            },
        }


if __name__ == "__main__":
    unittest.main()
