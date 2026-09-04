import csv
from datetime import datetime, timezone
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _load_tool():
    path = ROOT / "tools" / "replay_betaflight_outdoor_pulses.py"
    spec = importlib.util.spec_from_file_location("replay_betaflight_pulses_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tool = _load_tool()


class ReplayBetaflightOutdoorPulsesTest(unittest.TestCase):
    def test_replays_segments_and_measures_abort_lead(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "config.json"
            config_path.write_text(
                json.dumps(
                    {
                        "guidance": {
                            "law": "velocity_establishing_png",
                            "velocity_establishing_png": {
                                "fixed_vm_m_s": 2.0,
                                "speed_accel_limit_m_s2": 20.0,
                                "png_accel_limit_m_s2": 20.0,
                                "fov_centering_accel_limit_m_s2": 20.0,
                                "total_accel_limit_m_s2": 30.0,
                                "velocity_reference_slew_m_s2": 3.0,
                                "acquire_consecutive_frames": 1,
                                "engagement_policy": "noncollision",
                                "noncollision_bbox_abort_ratio": 0.1,
                                "noncollision_ttc_abort_s": 0.1,
                            },
                        }
                    }
                ),
                encoding="utf-8",
            )
            log_path = root / "FLIGHT_ACTIVE_TEST_IMPACT.csv"
            rows = [
                self._row(0.0, "algorithm", 0.01),
                self._row(0.1, "algorithm", 0.01),
                self._row(0.2, "passthrough", 0.01),
                self._row(2.0, "algorithm", 0.01),
                self._row(2.2, "algorithm", 0.2),
            ]
            with log_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            meta_path = root / "FLIGHT_ACTIVE_TEST_IMPACT_meta.json"
            meta_path.write_text(
                json.dumps({"created_unix_s": 1000.0}), encoding="utf-8"
            )
            impact_path = root / "impact.json"
            impact_path.write_text(
                json.dumps(
                    {
                        "events": {
                            "target_main_impact": {
                                "utc": datetime.fromtimestamp(
                                    1003.2, tz=timezone.utc
                                ).isoformat()
                            }
                        }
                    }
                ),
                encoding="utf-8",
            )

            result = tool.replay_logs(
                [log_path],
                config_path=config_path,
                impact_metrics_path=impact_path,
                impact_run_token="TEST_IMPACT",
                expected_pulse_count=2,
                max_saturation_fraction=0.4,
                minimum_abort_lead_s=0.75,
            )

            self.assertEqual(result["aggregate"]["pulse_count"], 2)
            self.assertEqual(result["pulses"][1]["first_abort_reason"], "noncollision_bbox_abort")
            self.assertAlmostEqual(result["log00106_noncollision_abort"]["lead_s"], 1.0)
            self.assertTrue(result["release_passed"])

    def test_rejects_unexpected_pulse_count(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "config.json"
            config_path.write_text(
                json.dumps(
                    {
                        "guidance": {
                            "law": "velocity_establishing_png",
                            "velocity_establishing_png": {"fixed_vm_m_s": 2.0},
                        }
                    }
                ),
                encoding="utf-8",
            )
            log_path = root / "run.csv"
            row = self._row(0.0, "algorithm", 0.01)
            with log_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(row))
                writer.writeheader()
                writer.writerow(row)

            with self.assertRaisesRegex(RuntimeError, "expected 2.*found 1"):
                tool.replay_logs(
                    [log_path],
                    config_path=config_path,
                    impact_metrics_path=root / "unused.json",
                    impact_run_token="unused",
                    expected_pulse_count=2,
                    max_saturation_fraction=0.4,
                    minimum_abort_lead_s=0.75,
                )

    @staticmethod
    def _row(elapsed_s, publish_mode, bbox_area_ratio):
        return {
            "elapsed_s": elapsed_s,
            "msp_publish_mode": publish_mode,
            "intercept_speed_saturated": 0,
            "intercept_total_saturated": 0,
            "intercept_detection_age_s": 0.02,
            "intercept_velocity_age_s": 0.02,
            "lambda_I_x": 0.0,
            "lambda_I_y": 0.0,
            "lambda_I_z": -1.0,
            "lambda_dot_I_x": 0.0,
            "lambda_dot_I_y": 0.0,
            "lambda_dot_I_z": 0.0,
            "los_valid": 1,
            "los_reject_reason": "",
            "bbox_area_ratio": bbox_area_ratio,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": 0.0,
            "attitude_synced": 1,
            "kinematics_velocity_filtered_n_m_s": 0.0,
            "kinematics_velocity_filtered_e_m_s": 0.0,
            "kinematics_velocity_filtered_d_m_s": 0.0,
            "kinematics_valid": 1,
            "ttc_valid": 0,
            "ttc_s": "",
            "track_id": 7,
        }


if __name__ == "__main__":
    unittest.main()
