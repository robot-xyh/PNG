import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _load_tool_module():
    path = ROOT / "tools" / "evaluate_betaflight_png_authority.py"
    spec = importlib.util.spec_from_file_location("evaluate_betaflight_png_authority_for_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tool = _load_tool_module()


class BetaflightPngAuthorityTest(unittest.TestCase):
    def test_recomputes_fixed_vm_acceleration_and_accel_tilt_rate(self):
        rows = [
            {
                "elapsed_s": "1.0",
                "guidance_valid": "1",
                "lambda_I_x": "0",
                "lambda_I_y": "0",
                "lambda_I_z": "-1",
                "omega_los_x": "0.1",
                "omega_los_y": "0",
                "omega_los_z": "0",
                "roll_deg": "0",
                "pitch_deg": "0",
                "yaw_deg": "0",
            }
        ]

        report = tool.analyze_rows(
            rows,
            navigation_constant=3.0,
            vm_values=(2.0,),
            accel_limits_mps2=(1.0,),
            attitude_kp_s_inv=(2.0,),
            tilt_limits_deg=(20.0,),
            rate_limits_deg_s=(30.0,),
        )

        candidate = report["candidates"][0]
        self.assertEqual(report["sample_selection"]["eligible_rows"], 1)
        self.assertAlmostEqual(candidate["acceleration"]["raw_norm_mps2"]["max"], 0.6)
        self.assertAlmostEqual(candidate["target_attitude"]["roll_abs_deg"]["max"], 3.501162, places=5)
        self.assertAlmostEqual(
            candidate["guidance_only_rate"]["roll_abs_deg_s"]["max"],
            7.002324,
            places=5,
        )
        self.assertEqual(candidate["guidance_only_rate"]["saturation_fraction"], 0.0)

    def test_requires_valid_guidance_and_finite_attitude(self):
        with self.assertRaisesRegex(RuntimeError, "no finite guidance-valid rows"):
            tool.analyze_rows(
                [{"elapsed_s": "1", "guidance_valid": "0"}],
                vm_values=(1.0,),
                accel_limits_mps2=(1.0,),
                attitude_kp_s_inv=(1.0,),
                tilt_limits_deg=(10.0,),
                rate_limits_deg_s=(30.0,),
            )


if __name__ == "__main__":
    unittest.main()
