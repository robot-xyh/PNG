import unittest

import numpy as np

from vision_guidance.betaflight_intercept_controller import (
    InterceptPhase,
    VelocityEstablishingPngConfig,
    VelocityEstablishingPngController,
    VelocityEstablishingPngInput,
)


def _input(timestamp: float, *, los_timestamp=None, speed=0.0, valid=True):
    return VelocityEstablishingPngInput(
        timestamp_s=timestamp,
        los_timestamp_s=timestamp if los_timestamp is None else los_timestamp,
        lambda_ned=np.array([1.0, 0.0, 0.0]),
        lambda_dot_ned_s=np.array([0.0, 0.1, 0.0]),
        tracking_valid=valid,
        bbox_area_ratio=0.01,
        attitude_R_IB=np.eye(3),
        attitude_valid=True,
        velocity_timestamp_s=timestamp,
        velocity_ned_m_s=np.array([speed, 0.0, 0.0]),
        velocity_valid=True,
    )


class VelocityEstablishingPngControllerTest(unittest.TestCase):
    def test_bounded_los_prediction_advances_velocity_reference(self):
        controller = VelocityEstablishingPngController(
            VelocityEstablishingPngConfig(
                fixed_vm_m_s=10.0,
                acquire_consecutive_frames=1,
                los_prediction_max_s=0.1,
            )
        )
        value = _input(1.2, los_timestamp=1.0)
        value = VelocityEstablishingPngInput(
            **{
                **value.__dict__,
                "lambda_dot_ned_s": np.array([0.0, 1.0, 0.0]),
            }
        )

        output = controller.update(value)

        self.assertTrue(output.valid)
        self.assertAlmostEqual(output.los_prediction_horizon_s, 0.1)
        self.assertGreater(output.velocity_reference_ned_m_s[1], 0.0)

    def test_fov_constraint_bounds_commanded_body_up_from_los(self):
        controller = VelocityEstablishingPngController(
            VelocityEstablishingPngConfig(
                fixed_vm_m_s=10.0,
                acquire_consecutive_frames=1,
                total_accel_limit_m_s2=7.0,
                fov_constraint_half_angle_deg=15.0,
            )
        )
        value = _input(0.0)
        value = VelocityEstablishingPngInput(
            **{
                **value.__dict__,
                "lambda_ned": np.array([0.3, 0.0, -0.953939]),
                "lambda_dot_ned_s": np.array([0.0, 1.0, 0.0]),
            }
        )

        output = controller.update(value)

        acceleration = np.array(output.acceleration_ned_m_s2)
        body_up = acceleration - np.array([0.0, 0.0, 9.80665])
        body_up /= np.linalg.norm(body_up)
        los = value.lambda_ned / np.linalg.norm(value.lambda_ned)
        error_deg = np.degrees(np.arccos(np.clip(np.dot(body_up, los), -1.0, 1.0)))
        self.assertTrue(output.fov_constraint_active)
        self.assertLessEqual(error_deg, 15.0 + 1.0e-9)
        self.assertLessEqual(np.linalg.norm(acceleration), 7.0 + 1.0e-9)

    def test_requires_consecutive_distinct_frames_then_accelerates(self):
        controller = VelocityEstablishingPngController(
            VelocityEstablishingPngConfig(fixed_vm_m_s=10.0, acquire_consecutive_frames=3)
        )

        self.assertFalse(controller.update(_input(0.0)).valid)
        self.assertFalse(controller.update(_input(0.01, los_timestamp=0.0)).valid)
        self.assertFalse(controller.update(_input(0.1)).valid)
        output = controller.update(_input(0.2))

        self.assertTrue(output.valid)
        self.assertEqual(output.phase, InterceptPhase.ACCELERATE)
        self.assertGreater(output.acceleration_ned_m_s2[0], 0.0)

    def test_transitions_to_png_track_at_speed_ratio(self):
        controller = VelocityEstablishingPngController(
            VelocityEstablishingPngConfig(fixed_vm_m_s=10.0, acquire_consecutive_frames=1)
        )

        output = controller.update(_input(0.0, speed=8.0))

        self.assertEqual(output.phase, InterceptPhase.PNG_TRACK)
        self.assertAlmostEqual(output.png_acceleration_ned_m_s2[1], 3.0)

    def test_limits_each_term_and_total(self):
        controller = VelocityEstablishingPngController(
            VelocityEstablishingPngConfig(
                fixed_vm_m_s=20.0,
                acquire_consecutive_frames=1,
                speed_accel_limit_m_s2=2.0,
                png_accel_limit_m_s2=1.0,
                fov_centering_accel_limit_m_s2=0.5,
                total_accel_limit_m_s2=2.5,
            )
        )
        value = _input(0.0)
        value = VelocityEstablishingPngInput(
            **{**value.__dict__, "lambda_ned": np.array([1.0, 1.0, 0.0]), "lambda_dot_ned_s": np.array([0.0, 10.0, 0.0])}
        )

        output = controller.update(value)

        self.assertTrue(output.speed_saturated)
        self.assertTrue(output.png_saturated)
        self.assertTrue(output.fov_saturated)
        self.assertLessEqual(np.linalg.norm(output.acceleration_ned_m_s2), 2.5 + 1e-9)

    def test_stale_velocity_aborts_and_latches(self):
        controller = VelocityEstablishingPngController(
            VelocityEstablishingPngConfig(fixed_vm_m_s=10.0, acquire_consecutive_frames=1)
        )
        self.assertTrue(controller.update(_input(0.0)).valid)
        stale = _input(1.0)
        stale = VelocityEstablishingPngInput(
            **{**stale.__dict__, "velocity_timestamp_s": 0.0}
        )

        aborted = controller.update(stale)
        latched = controller.update(_input(1.1))

        self.assertEqual(aborted.phase, InterceptPhase.ABORT)
        self.assertEqual(aborted.reason, "velocity_stale")
        self.assertEqual(latched.reason, "velocity_stale")
        self.assertFalse(latched.valid)

    def test_nonfinite_input_fails_closed(self):
        controller = VelocityEstablishingPngController(
            VelocityEstablishingPngConfig(fixed_vm_m_s=10.0)
        )
        value = _input(0.0)
        value = VelocityEstablishingPngInput(
            **{**value.__dict__, "lambda_dot_ned_s": np.array([0.0, np.nan, 0.0])}
        )

        output = controller.update(value)

        self.assertFalse(output.valid)
        self.assertEqual(output.reason, "nonfinite_input")

    def test_zero_los_and_nonfinite_timestamp_fail_closed(self):
        controller = VelocityEstablishingPngController(
            VelocityEstablishingPngConfig(fixed_vm_m_s=10.0)
        )
        value = _input(0.0)
        zero_los = VelocityEstablishingPngInput(
            **{**value.__dict__, "lambda_ned": np.zeros(3)}
        )

        self.assertEqual(controller.update(zero_los).reason, "los_zero")
        timestamp_output = controller.update(_input(float("nan")))
        self.assertFalse(timestamp_output.valid)
        self.assertEqual(timestamp_output.reason, "nonfinite_input")
        self.assertEqual(timestamp_output.timestamp_s, 0.0)


if __name__ == "__main__":
    unittest.main()
