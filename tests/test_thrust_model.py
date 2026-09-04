import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from vision_guidance.flight_control import (
    RcCommandMapper,
    RcMappingConfig,
    guidance_eval_to_setpoint,
)
from vision_guidance.thrust_model import VoltageThrottleThrustModel
from vision_guidance.types import GuidanceEval


def _model_values():
    return {
        "schema_version": 1,
        "model_type": "voltage_throttle_specific_force_lut",
        "calibration_id": "unit-test-lut",
        "voltage_v": [20.0, 25.2],
        "throttle_us": [1200.0, 1300.0, 1500.0],
        "specific_force_m_s2": [
            [6.0, 10.0, 20.0],
            [7.0, 11.0, 22.0],
        ],
        "validation": {
            "passed": True,
            "sample_count": 200,
            "median_relative_error": 0.04,
            "p95_relative_error": 0.12,
        },
    }


class VoltageThrottleThrustModelTest(unittest.TestCase):
    def test_forward_and_inverse_interpolation_are_voltage_aware(self):
        model = VoltageThrottleThrustModel.from_mapping(_model_values())

        force = model.specific_force(22.6, 1300.0)
        lookup = model.throttle_for_specific_force(22.6, force)

        self.assertAlmostEqual(force, 10.5)
        self.assertAlmostEqual(lookup.throttle_us, 1300.0)
        self.assertFalse(lookup.limited)
        self.assertTrue(model.covers_voltage(20.0))
        self.assertFalse(model.covers_voltage(19.9))

    def test_file_load_requires_exact_hash_and_calibration_id(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "lut.json"
            payload = json.dumps(_model_values(), sort_keys=True).encode("utf-8")
            path.write_bytes(payload)
            sha256 = hashlib.sha256(payload).hexdigest()

            model = VoltageThrottleThrustModel.from_file(
                path,
                expected_sha256=sha256,
                expected_calibration_id="unit-test-lut",
            )
            self.assertEqual(model.source_sha256, sha256)
            with self.assertRaisesRegex(ValueError, "SHA256"):
                VoltageThrottleThrustModel.from_file(
                    path,
                    expected_sha256="0" * 64,
                )

    def test_model_rejects_non_monotonic_or_failed_evidence(self):
        values = _model_values()
        values["specific_force_m_s2"][0] = [6.0, 5.0, 20.0]
        with self.assertRaisesRegex(ValueError, "increase with throttle"):
            VoltageThrottleThrustModel.from_mapping(values)

        values = _model_values()
        values["validation"]["passed"] = False
        with self.assertRaisesRegex(ValueError, "not passed"):
            VoltageThrottleThrustModel.from_mapping(values)

    def test_guidance_uses_direct_lut_throttle_target(self):
        model = VoltageThrottleThrustModel.from_mapping(_model_values())
        setpoint = guidance_eval_to_setpoint(
            GuidanceEval(
                timestamp=1.0,
                g_eval=np.zeros(3),
                valid=True,
                quality=1.0,
            ),
            R_IB=np.eye(3),
            rate_gain_matrix=np.zeros((3, 3)),
            hover_thrust=0.5,
            mapping_type="accel_tilt_rate",
            accel_tilt_rate={
                "thrust_feedforward": {
                    "enabled": True,
                    "model": "voltage_throttle_lut",
                    "calibration_id": "unit-test-lut",
                    "model_path": "unused.json",
                    "model_sha256": "0" * 64,
                }
            },
            thrust_model=model,
            battery_voltage_v=22.6,
        )

        self.assertTrue(setpoint.valid)
        self.assertEqual(setpoint.thrust_model, "voltage_throttle_lut")
        self.assertAlmostEqual(setpoint.thrust_model_voltage_v, 22.6)
        self.assertAlmostEqual(setpoint.throttle_target_us, 1282.66625, places=3)
        command = RcCommandMapper(
            RcMappingConfig(
                throttle_min_us=1200,
                throttle_hover_us=1275,
                throttle_max_us=1500,
                neutral_throttle_us=1200,
            )
        ).map_setpoint(setpoint)
        self.assertEqual(command.channels[2], 1283)
        self.assertAlmostEqual(command.requested_throttle_us, 1282.66625, places=3)

    def test_guidance_rejects_missing_model_or_uncovered_voltage(self):
        kwargs = {
            "guidance": GuidanceEval(
                timestamp=1.0,
                g_eval=np.zeros(3),
                valid=True,
                quality=1.0,
            ),
            "R_IB": np.eye(3),
            "rate_gain_matrix": np.zeros((3, 3)),
            "hover_thrust": 0.5,
            "mapping_type": "accel_tilt_rate",
            "accel_tilt_rate": {
                "thrust_feedforward": {
                    "enabled": True,
                    "model": "voltage_throttle_lut",
                    "calibration_id": "unit-test-lut",
                    "model_path": "unused.json",
                    "model_sha256": "0" * 64,
                }
            },
        }
        missing = guidance_eval_to_setpoint(**kwargs, battery_voltage_v=22.0)
        uncovered = guidance_eval_to_setpoint(
            **kwargs,
            thrust_model=VoltageThrottleThrustModel.from_mapping(_model_values()),
            battery_voltage_v=19.0,
        )

        self.assertFalse(missing.valid)
        self.assertEqual(missing.reject_reason, "thrust_lut_unavailable")
        self.assertFalse(uncovered.valid)
        self.assertEqual(
            uncovered.reject_reason,
            "thrust_lut_voltage_outside_coverage",
        )


if __name__ == "__main__":
    unittest.main()
