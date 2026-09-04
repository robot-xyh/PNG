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
    RELEASE_SOURCE_PATHS,
    validate_rc_interlock_evidence,
    validate_finalized_run_evidence,
    validate_release_evidence,
    validate_flight_supervised_config,
    validate_snapshot_flight_state,
)


class BetaflightFlightSupervisedApprovalTest(unittest.TestCase):
    def setUp(self):
        path = ROOT / "config" / "betaflight.rk3588.velocity_png.flight_supervised.json"
        self.config = json.loads(path.read_text(encoding="utf-8"))
        self.output = ROOT / "logs" / "betaflight_velocity_png_flight_noncollision_v2_approval.json"
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
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.thrust_model_path = Path(self.temporary_directory.name) / "thrust_lut.json"
        model = {
            "schema_version": 1,
            "model_type": "voltage_throttle_specific_force_lut",
            "calibration_id": "test-full-6s-lut",
            "voltage_v": [20.0, 25.2],
            "throttle_us": [1200.0, 1300.0, 1500.0],
            "specific_force_m_s2": [
                [6.0, 10.0, 20.0],
                [7.0, 11.0, 22.0],
            ],
            "validation": {
                "passed": True,
                "sample_count": 200,
                "median_relative_error": 0.05,
                "p95_relative_error": 0.15,
            },
        }
        self.thrust_model_path.write_text(json.dumps(model), encoding="utf-8")
        thrust = self.config["guidance_command"]["accel_tilt_rate"]["thrust_feedforward"]
        thrust["calibration_id"] = model["calibration_id"]
        thrust["model_path"] = str(self.thrust_model_path)
        thrust["model_sha256"] = hashlib.sha256(
            self.thrust_model_path.read_bytes()
        ).hexdigest()

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
            "test-full-6s-lut",
        )
        self.assertEqual(
            evidence["guidance_command"]["thrust_model"]["sha256"],
            self.config["guidance_command"]["accel_tilt_rate"]["thrust_feedforward"][
                "model_sha256"
            ],
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
        with self.assertRaisesRegex(RuntimeError, "calibration_id"):
            self.validate(config)

    def test_rejects_tampered_or_incomplete_thrust_lut(self):
        config = copy.deepcopy(self.config)
        config["guidance_command"]["accel_tilt_rate"]["thrust_feedforward"][
            "model_sha256"
        ] = "0" * 64
        with self.assertRaisesRegex(RuntimeError, "SHA256"):
            self.validate(config)

        config = copy.deepcopy(self.config)
        model = json.loads(self.thrust_model_path.read_text(encoding="utf-8"))
        model["voltage_v"] = [22.0, 25.2]
        narrow_path = Path(self.temporary_directory.name) / "narrow.json"
        narrow_path.write_text(json.dumps(model), encoding="utf-8")
        thrust = config["guidance_command"]["accel_tilt_rate"]["thrust_feedforward"]
        thrust["model_path"] = str(narrow_path)
        thrust["model_sha256"] = hashlib.sha256(narrow_path.read_bytes()).hexdigest()
        with self.assertRaisesRegex(RuntimeError, "20.0-25.2"):
            self.validate(config)

    def test_rejects_wrong_timer_or_poll_schedule(self):
        config = copy.deepcopy(self.config)
        config["safety"]["takeover_duration_interlock"].update(
            enabled=True,
            max_duration_s=10,
        )
        with self.assertRaisesRegex(RuntimeError, "one 2 s pulse"):
            self.validate(config)

        config = copy.deepcopy(self.config)
        config["msp_runtime"]["attitude_poll_hz"] = 10
        with self.assertRaisesRegex(RuntimeError, "attitude_poll_hz"):
            self.validate(config)

    def test_release_evidence_requires_bound_passing_mc100(self):
        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / "mc100.json"
            config_sha256 = "a" * 64
            scenario_names = [
                "final_chain_software_p95",
                "observed_active_flight_p95",
                "conservative_physical_p95_budget",
            ]
            thrust = self.config["guidance_command"]["accel_tilt_rate"][
                "thrust_feedforward"
            ]
            thrust_evidence = {
                "path": str(self.thrust_model_path.resolve()),
                "sha256": thrust["model_sha256"],
                "calibration_id": thrust["calibration_id"],
                "voltage_coverage_v": [20.0, 25.2],
                "throttle_coverage_us": [1200.0, 1500.0],
            }

            def summary(scenario_name, *, role, policy, passed):
                return {
                    "scenario_name": scenario_name,
                    "evaluation_name": (
                        "candidate" if role == "contact_performance" else "safety"
                    ),
                    "required_for_release": True,
                    "evidence_role": role,
                    "engagement_policy": policy,
                    "passed": passed,
                    "initially_visible_hit_rate": 0.8,
                    "initially_visible_fov_hit_rate": 0.8,
                    "checks": {
                        "worst_minimum_range_m": {
                            "operator": "report_only",
                            "threshold": None,
                            "required": False,
                        }
                    },
                }

            safety_summaries = [
                {
                    "scenario_name": name,
                    "evaluation_name": "safety",
                    "engagement_policy": "noncollision",
                    "initially_visible_count": 3000,
                    "timely_abort_rate": 1.0,
                    "unsafe_contact_rate": 0.0,
                    "passed": True,
                }
                for name in scenario_names
            ]
            report = {
                "schema_version": 3,
                "purpose": "stochastic interception release evaluation",
                "release_passed": True,
                "runtime_binding": {
                    "sha256": config_sha256,
                    "thrust_model": dict(thrust_evidence),
                },
                "thrust_model_binding": dict(thrust_evidence),
                "source_bindings": {
                    name: {
                        "path": str(path.resolve()),
                        "repository_path": str(path.resolve().relative_to(ROOT)),
                        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    }
                    for name, path in RELEASE_SOURCE_PATHS.items()
                },
                "simulation": {
                    "battery_voltage_v": 22.6,
                    "thrust_model_sha256": thrust["model_sha256"],
                    "thrust_model_calibration_id": thrust["calibration_id"],
                },
                "scenarios": [
                    {"name": scenario_names[0], "battery_voltage_v": 25.2},
                    {"name": scenario_names[1], "battery_voltage_v": 22.6},
                    {"name": scenario_names[2], "battery_voltage_v": 20.0},
                ],
                "acceptance": {
                    "initially_visible_hit_rate_min": 0.8,
                    "initially_visible_fov_hit_rate_min": 0.8,
                    "worst_minimum_range_m_max": None,
                },
                "paired_screening": {
                    "passed": True,
                    "selected_evaluation": "candidate",
                },
                "trials_per_case": 100,
                "case_count": 30,
                "row_count": 27000,
                "required_summary_count": 6,
                "summaries": [
                    *[
                        summary(
                            name,
                            role="contact_performance",
                            policy="contact",
                            passed=True,
                        )
                        for name in scenario_names
                    ],
                    *[
                        summary(
                            name,
                            role="noncollision_safety",
                            policy="noncollision",
                            passed=False,
                        )
                        for name in scenario_names
                    ],
                ],
                "policy_results": {
                    "passed": True,
                    "runtime_engagement_policy": "noncollision",
                    "contact_evidence_is_not_noncollision_flight_authority": True,
                    "contact_performance": {
                        "passed": True,
                        "engagement_policy": "contact",
                        "evaluation_name": "candidate",
                        "authorizes_contact_flight": False,
                        "scenario_names": scenario_names,
                    },
                    "noncollision_safety": {
                        "passed": True,
                        "engagement_policy": "noncollision",
                        "evaluation_name": "safety",
                        "requires_pilot_action_after_abort": True,
                        "acceptance": {
                            "timely_abort_rate_min": 0.99,
                            "unsafe_contact_rate_max": 0.01,
                            "minimum_abort_lead_time_s": 0.75,
                        },
                        "summaries": safety_summaries,
                    },
                },
            }
            report_path.write_text(json.dumps(report), encoding="utf-8")

            evidence = validate_release_evidence(
                report,
                report_path,
                runtime_config_sha256=config_sha256,
                runtime_thrust_model=thrust_evidence,
            )
            self.assertEqual(evidence["required_scenario_count"], 3)
            self.assertEqual(evidence["required_noncollision_scenario_count"], 3)

            report["release_passed"] = False
            with self.assertRaisesRegex(RuntimeError, "did not pass"):
                validate_release_evidence(
                    report,
                    report_path,
                    runtime_config_sha256=config_sha256,
                    runtime_thrust_model=thrust_evidence,
                )

            report["release_passed"] = True
            report["summaries"] = [report["summaries"][0]] * 3
            with self.assertRaisesRegex(RuntimeError, "coverage is incomplete"):
                validate_release_evidence(
                    report,
                    report_path,
                    runtime_config_sha256=config_sha256,
                    runtime_thrust_model=thrust_evidence,
                )

            report["summaries"] = ["not-an-object"]
            with self.assertRaisesRegex(RuntimeError, "list of objects"):
                validate_release_evidence(
                    report,
                    report_path,
                    runtime_config_sha256=config_sha256,
                    runtime_thrust_model=thrust_evidence,
                )

    def test_rc_interlock_evidence_is_hash_bound_and_under_200_ms(self):
        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / "rc_interlock.json"
            config_sha256 = "b" * 64
            report = {
                "schema_version": 1,
                "passed": True,
                "runtime_binding": {"sha256": config_sha256},
                "max_release_latency_ms": 50.0,
                "checks": {
                    "override_seen": True,
                    "release_mode_seen": True,
                    "rc7_low_seen": True,
                    "override_cleared": True,
                },
            }
            report_path.write_text(json.dumps(report), encoding="utf-8")
            evidence = validate_rc_interlock_evidence(
                report,
                report_path,
                runtime_config_sha256=config_sha256,
            )
            self.assertEqual(evidence["max_release_latency_ms"], 50.0)

            report["max_release_latency_ms"] = 201.0
            with self.assertRaisesRegex(RuntimeError, "exceeds 200"):
                validate_rc_interlock_evidence(
                    report,
                    report_path,
                    runtime_config_sha256=config_sha256,
                )

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

    def test_finalized_log_only_run_is_required_and_config_bound(self):
        root = Path(self.temporary_directory.name)
        csv_path = root / "field.csv"
        fields = (
            "msp_set_raw_rc_attempt_count",
            "evidence_frame_write_count",
            "evidence_frame_error_count",
            "kinematics_valid",
            "gps_fix",
            "gps_satellites",
            "vbat_v",
        )
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index in range(100):
                writer.writerow(
                    {
                        "msp_set_raw_rc_attempt_count": 0,
                        "evidence_frame_write_count": min(25, index + 1),
                        "evidence_frame_error_count": 0,
                        "kinematics_valid": 1,
                        "gps_fix": 1,
                        "gps_satellites": 8,
                        "vbat_v": 24.0,
                    }
                )
        config_sha256 = "c" * 64
        meta_path = root / "field_meta.json"
        meta_path.write_text(
            json.dumps(
                {
                    "config_sha256": config_sha256,
                    "allow_control": False,
                    "control_mode": "log_only",
                    "repository_commit": "d" * 40,
                    "repository_dirty": False,
                    "source_files": [
                        {"path": "/runtime.py", "sha256": "e" * 64}
                    ],
                }
            ),
            encoding="utf-8",
        )
        artifact = lambda path: {
            "path": str(path),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
        manifest_path = root / "field_manifest.json"
        manifest = {
            "schema_version": 2,
            "finalized": True,
            "completion": {"complete": True},
            "missing_runtime_artifacts": [],
            "pairing": {"confidence": "unique"},
            "visual_evidence": {"enabled": True, "frame_count": 25},
            "external_artifacts": {"blackbox": {"sha256": "f" * 64}},
            "blackbox_interpretation": {
                "authoritative_mode_source": "host_msp_status_box_ids",
                "decoder_labels_used_for_mode_decisions": False,
            },
            "artifacts": [artifact(csv_path), artifact(meta_path)],
        }
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

        evidence = validate_finalized_run_evidence(
            manifest,
            manifest_path,
            runtime_config_sha256=config_sha256,
        )
        self.assertEqual(evidence["row_count"], 100)
        self.assertEqual(evidence["set_raw_rc_attempt_count"], 0)

        manifest["pairing"]["confidence"] = "time_only"
        with self.assertRaisesRegex(RuntimeError, "unique pairing"):
            validate_finalized_run_evidence(
                manifest,
                manifest_path,
                runtime_config_sha256=config_sha256,
            )


if __name__ == "__main__":
    unittest.main()
