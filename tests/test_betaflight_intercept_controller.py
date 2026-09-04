import unittest

import numpy as np

from vision_guidance.betaflight_intercept_controller import (
    EngagementPolicy,
    FovPriorityConfig,
    InterceptPhase,
    VelocityEstablishingPngConfig,
    VelocityEstablishingPngController,
    VelocityEstablishingPngInput,
)


def _input(
    timestamp: float,
    *,
    los_timestamp=None,
    velocity=(0.0, 0.0, 0.0),
    valid=True,
    reason=None,
    bbox_area_ratio=0.001,
    track_id=7,
    los=(1.0, 0.0, 0.0),
    los_dot=(0.0, 0.1, 0.0),
    ttc_valid=False,
    ttc_s=None,
):
    return VelocityEstablishingPngInput(
        timestamp_s=timestamp,
        los_timestamp_s=timestamp if los_timestamp is None else los_timestamp,
        lambda_ned=np.asarray(los, dtype=float),
        lambda_dot_ned_s=np.asarray(los_dot, dtype=float),
        tracking_valid=valid,
        bbox_area_ratio=bbox_area_ratio,
        attitude_R_IB=np.eye(3),
        attitude_valid=True,
        velocity_timestamp_s=timestamp,
        velocity_ned_m_s=np.asarray(velocity, dtype=float),
        velocity_valid=True,
        tracking_reason=reason,
        ttc_valid=ttc_valid,
        ttc_s=ttc_s,
        track_id=track_id,
    )


class VelocityEstablishingPngControllerTest(unittest.TestCase):
    def _controller(self, **overrides):
        values = {
            "fixed_vm_m_s": 10.0,
            "acquire_consecutive_frames": 1,
            "total_accel_limit_m_s2": 7.0,
        }
        values.update(overrides)
        return VelocityEstablishingPngController(
            VelocityEstablishingPngConfig(**values)
        )

    def test_velocity_reference_starts_at_measurement_and_slews_as_vector(self):
        controller = self._controller(velocity_reference_slew_m_s2=3.0)
        first = controller.update(
            _input(0.0, velocity=(2.0, 0.0, 0.0), los=(0.0, 1.0, 0.0))
        )
        second = controller.update(
            _input(0.5, velocity=(2.0, 0.0, 0.0), los=(0.0, 1.0, 0.0))
        )

        self.assertEqual(first.phase, InterceptPhase.TRACKING)
        np.testing.assert_allclose(first.velocity_reference_ned_m_s, (2.0, 0.0, 0.0))
        np.testing.assert_allclose(second.velocity_reference_raw_ned_m_s, (0.0, 10.0, 0.0))
        delta = np.asarray(second.velocity_reference_ned_m_s) - np.array([2.0, 0.0, 0.0])
        self.assertAlmostEqual(float(np.linalg.norm(delta)), 1.5)

    def test_png_and_fov_are_protected_before_exact_speed_budget(self):
        controller = self._controller(
            navigation_constant=1.0,
            speed_gain_s_inv=1.0,
            speed_accel_limit_m_s2=7.0,
            png_accel_limit_m_s2=6.0,
            fov_centering_gain_s2=0.0,
            fov_centering_accel_limit_m_s2=1.0,
            velocity_reference_slew_m_s2=100.0,
        )
        controller.update(_input(0.0, los_dot=(0.0, 0.6, 0.0)))
        output = controller.update(_input(0.1, los_dot=(0.0, 0.6, 0.0)))

        np.testing.assert_allclose(output.protected_acceleration_ned_m_s2, (0.0, 6.0, 0.0))
        np.testing.assert_allclose(output.speed_acceleration_ned_m_s2, (7.0, 0.0, 0.0))
        self.assertAlmostEqual(output.speed_budget_scale, np.sqrt(13.0) / 7.0)
        self.assertAlmostEqual(np.linalg.norm(output.acceleration_ned_m_s2), 7.0)
        self.assertTrue(output.total_saturated)

    def test_component_raw_and_limited_values_are_exposed(self):
        controller = self._controller(
            navigation_constant=2.0,
            png_accel_limit_m_s2=1.0,
            fov_centering_gain_s2=10.0,
            fov_centering_accel_limit_m_s2=0.5,
        )
        output = controller.update(
            _input(0.0, velocity=(10.0, 0.0, 0.0), los=(1.0, 1.0, 0.0), los_dot=(0.0, 2.0, 0.0))
        )

        self.assertGreater(np.linalg.norm(output.png_acceleration_raw_ned_m_s2), 1.0)
        self.assertAlmostEqual(np.linalg.norm(output.png_acceleration_ned_m_s2), 1.0)
        self.assertGreater(np.linalg.norm(output.fov_acceleration_raw_ned_m_s2), 0.5)
        self.assertAlmostEqual(np.linalg.norm(output.fov_acceleration_ned_m_s2), 0.5)
        self.assertTrue(output.png_saturated)
        self.assertTrue(output.fov_saturated)

    def test_fov_priority_suppresses_only_opposing_speed_component(self):
        common = dict(
            navigation_constant=1.0,
            speed_gain_s_inv=1.0,
            speed_accel_limit_m_s2=20.0,
            png_accel_limit_m_s2=20.0,
            fov_centering_gain_s2=1.0,
            fov_centering_accel_limit_m_s2=20.0,
            total_accel_limit_m_s2=20.0,
            velocity_reference_slew_m_s2=100.0,
        )
        priority = self._controller(
            **common,
            fov_priority=FovPriorityConfig(
                enabled=True,
                start_ratio=0.5,
                full_ratio=0.8,
                horizontal_half_fov_deg=30.0,
                vertical_half_fov_deg=25.0,
            ),
        )
        value = dict(
            velocity=(0.0, 10.0, 0.0),
            los=(0.0, 0.5, -0.8660254),
            los_dot=(0.0, 0.0, 0.0),
        )
        priority.update(_input(0.0, **value))
        output = priority.update(_input(0.1, **value))

        self.assertTrue(output.fov_priority_active)
        self.assertAlmostEqual(output.fov_priority_weight, 1.0)
        self.assertAlmostEqual(output.speed_acceleration_ned_m_s2[1], 0.0)
        self.assertGreater(output.acceleration_ned_m_s2[1], 0.0)

    def test_enabled_fov_priority_requires_rectangular_fov(self):
        with self.assertRaisesRegex(ValueError, "requires positive"):
            FovPriorityConfig(enabled=True)

    def test_bounded_los_prediction_changes_raw_reference(self):
        controller = self._controller(los_prediction_max_s=0.1)
        output = controller.update(
            _input(1.2, los_timestamp=1.1, los_dot=(0.0, 1.0, 0.0))
        )

        self.assertTrue(output.valid)
        self.assertAlmostEqual(output.los_prediction_horizon_s, 0.1)
        self.assertGreater(output.velocity_reference_raw_ned_m_s[1], 0.0)

    def test_requires_consecutive_distinct_frames(self):
        controller = self._controller(acquire_consecutive_frames=3)

        self.assertFalse(controller.update(_input(0.0)).valid)
        self.assertFalse(controller.update(_input(0.01, los_timestamp=0.0)).valid)
        self.assertFalse(controller.update(_input(0.1)).valid)
        output = controller.update(_input(0.2))

        self.assertTrue(output.valid)
        self.assertEqual(output.phase, InterceptPhase.TRACKING)

    def test_noncollision_bbox_and_ttc_abort_immediately_and_latch(self):
        bbox_controller = self._controller()
        bbox = bbox_controller.update(_input(0.0, bbox_area_ratio=0.012))
        self.assertEqual(bbox.phase, InterceptPhase.ABORT)
        self.assertEqual(bbox.terminal_trigger, "noncollision_bbox_abort")

        ttc_controller = self._controller()
        ttc = ttc_controller.update(_input(0.0, ttc_valid=True, ttc_s=2.0))
        latched = ttc_controller.update(_input(0.1))
        self.assertEqual(ttc.terminal_trigger, "noncollision_ttc_abort")
        self.assertEqual(latched.phase, InterceptPhase.ABORT)
        self.assertFalse(latched.valid)

    def test_robust_area_growth_ttc_triggers_after_five_samples(self):
        controller = self._controller()
        output = None
        for index in range(5):
            timestamp = 0.03 * index
            area = 0.002 + 0.004 * timestamp
            output = controller.update(
                _input(timestamp, bbox_area_ratio=area, los_timestamp=timestamp)
            )

        assert output is not None
        self.assertEqual(output.phase, InterceptPhase.ABORT)
        self.assertEqual(output.terminal_trigger, "noncollision_area_ttc_abort")
        self.assertAlmostEqual(output.area_ttc_s, 1.24, places=6)

    def test_contact_terminal_disables_speed_and_completes_on_area(self):
        controller = self._controller(engagement_policy=EngagementPolicy.CONTACT.value)
        terminal = controller.update(_input(0.0, bbox_area_ratio=0.05))

        self.assertEqual(terminal.phase, InterceptPhase.TERMINAL_VISUAL)
        np.testing.assert_allclose(terminal.speed_acceleration_ned_m_s2, np.zeros(3))
        self.assertEqual(terminal.terminal_track_id, 7)

        complete_controller = self._controller(
            engagement_policy=EngagementPolicy.CONTACT.value
        )
        complete = complete_controller.update(_input(0.0, bbox_area_ratio=0.25))
        self.assertEqual(complete.phase, InterceptPhase.COMPLETE)
        self.assertFalse(complete.valid)
        self.assertEqual(complete.terminal_trigger, "contact_bbox_complete")

    def test_contact_blind_hold_decays_linearly_and_expires_at_point_two_seconds(self):
        controller = self._controller(engagement_policy=EngagementPolicy.CONTACT.value)
        terminal = controller.update(
            _input(0.0, bbox_area_ratio=0.05, velocity=(3.0, 0.0, 0.0))
        )
        missing = controller.update(
            _input(0.01, valid=False, reason="no_detection", bbox_area_ratio=None)
        )
        halfway = controller.update(
            _input(0.11, valid=False, reason="no_detection", bbox_area_ratio=None)
        )
        expired = controller.update(
            _input(0.21, valid=False, reason="no_detection", bbox_area_ratio=None)
        )

        self.assertEqual(missing.phase, InterceptPhase.BLIND_HOLD)
        self.assertTrue(missing.valid)
        self.assertAlmostEqual(missing.blind_scale, 1.0)
        self.assertAlmostEqual(halfway.blind_scale, 0.5)
        np.testing.assert_allclose(
            np.asarray(halfway.acceleration_ned_m_s2),
            0.5 * np.asarray(terminal.acceleration_ned_m_s2),
        )
        self.assertEqual(expired.phase, InterceptPhase.ABORT)
        self.assertEqual(expired.reason, "blind_hold_expired")

    def test_contact_reacquires_only_same_track_after_two_distinct_frames(self):
        controller = self._controller(engagement_policy=EngagementPolicy.CONTACT.value)
        controller.update(_input(0.0, bbox_area_ratio=0.05))
        controller.update(
            _input(0.01, valid=False, reason="no_detection", bbox_area_ratio=None)
        )
        wrong = controller.update(_input(0.04, track_id=8, bbox_area_ratio=0.05))
        first = controller.update(_input(0.06, track_id=7, bbox_area_ratio=0.05))
        repeated = controller.update(
            _input(0.07, los_timestamp=0.06, track_id=7, bbox_area_ratio=0.05)
        )
        second = controller.update(_input(0.09, track_id=7, bbox_area_ratio=0.05))

        self.assertEqual(wrong.phase, InterceptPhase.BLIND_HOLD)
        self.assertEqual(first.phase, InterceptPhase.BLIND_HOLD)
        self.assertEqual(first.terminal_reacquire_count, 1)
        self.assertEqual(repeated.terminal_reacquire_count, 1)
        self.assertEqual(second.phase, InterceptPhase.TERMINAL_VISUAL)

    def test_stale_velocity_aborts_and_latches(self):
        controller = self._controller()
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

    def test_nonfinite_input_fails_closed_without_nonfinite_output(self):
        controller = self._controller()
        value = _input(0.0)
        value = VelocityEstablishingPngInput(
            **{**value.__dict__, "lambda_dot_ned_s": np.array([0.0, np.nan, 0.0])}
        )

        output = controller.update(value)

        self.assertFalse(output.valid)
        self.assertEqual(output.reason, "nonfinite_input")
        for item in output.to_dict().values():
            if isinstance(item, (float, int)):
                self.assertTrue(np.isfinite(item))


if __name__ == "__main__":
    unittest.main()
