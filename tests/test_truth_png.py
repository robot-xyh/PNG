import unittest
from types import SimpleNamespace

import numpy as np

from examples.run_airsim_truth_png import _apply_altitude_correction, _world_position
from vision_guidance.truth_png import compute_truth_png, integrate_velocity_command


class TruthPNGTest(unittest.TestCase):
    def test_collision_course_has_near_zero_acceleration(self):
        result = compute_truth_png(
            relative_position=np.array([100.0, 0.0, 0.0]),
            relative_velocity=np.array([-20.0, 0.0, 0.0]),
            navigation_constant=3.0,
        )

        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.closing_speed, 20.0)
        self.assertLess(np.linalg.norm(result.acceleration), 1.0e-9)

    def test_crossing_target_generates_normal_acceleration(self):
        result = compute_truth_png(
            relative_position=np.array([100.0, 0.0, 0.0]),
            relative_velocity=np.array([-10.0, 10.0, 0.0]),
            navigation_constant=3.0,
        )

        self.assertTrue(result.valid)
        self.assertGreater(np.linalg.norm(result.acceleration), 0.0)
        self.assertAlmostEqual(float(np.dot(result.acceleration, result.los)), 0.0, places=9)

    def test_receding_target_is_rejected(self):
        result = compute_truth_png(
            relative_position=np.array([100.0, 0.0, 0.0]),
            relative_velocity=np.array([5.0, 0.0, 0.0]),
        )

        self.assertFalse(result.valid)
        self.assertEqual(result.reject_reason, "not_closing")
        self.assertTrue(np.allclose(result.acceleration, np.zeros(3)))

    def test_acceleration_is_clipped(self):
        result = compute_truth_png(
            relative_position=np.array([10.0, 0.0, 0.0]),
            relative_velocity=np.array([-30.0, 30.0, 0.0]),
            navigation_constant=5.0,
            max_accel=4.0,
        )

        self.assertTrue(result.valid)
        self.assertAlmostEqual(np.linalg.norm(result.acceleration), 4.0)

    def test_velocity_command_uses_fallback_speed_and_caps_output(self):
        command = integrate_velocity_command(
            current_velocity=np.array([0.0, 0.0, -1.0]),
            acceleration=np.array([0.0, 10.0, 0.0]),
            dt=0.1,
            speed_cap=3.0,
            min_speed=2.0,
            fallback_direction=np.array([1.0, 0.0, 0.0]),
        )

        self.assertLessEqual(np.linalg.norm(command), 3.0)
        self.assertGreater(command[0], 0.0)
        self.assertGreater(command[1], 0.0)
        self.assertGreater(command[0], abs(command[2]))

    def test_world_position_adds_vehicle_start_origin(self):
        kinematics = SimpleNamespace(
            position=SimpleNamespace(x_val=1.0, y_val=2.0, z_val=-3.0)
        )
        origins = {"Intruder": np.array([300.0, -20.0, -2.0])}

        position = _world_position(kinematics, "Intruder", origins)

        self.assertTrue(np.allclose(position, np.array([301.0, -18.0, -5.0])))

    def test_altitude_correction_respects_speed_cap(self):
        args = SimpleNamespace(
            altitude_correction=True,
            vertical_kp=1.5,
            vertical_speed_limit=3.0,
        )

        command = _apply_altitude_correction(
            v_cmd=np.array([8.0, 0.0, 0.0]),
            relative_position=np.array([0.0, 0.0, 2.0]),
            intruder_velocity=np.zeros(3),
            speed_cap=8.0,
            args=args,
        )

        self.assertAlmostEqual(command[2], 3.0)
        self.assertLessEqual(np.linalg.norm(command), 8.0 + 1.0e-9)


if __name__ == "__main__":
    unittest.main()
