import unittest

import numpy as np

from vision_guidance.terminal_extrapolation import (
    ABORT_HOLD,
    BLIND_PUSH,
    DISABLED,
    TERMINAL_VISUAL,
    TRACKING,
    TerminalConfig,
    TerminalExtrapolator,
)


class TerminalExtrapolationTest(unittest.TestCase):
    def test_disabled_passthrough(self):
        extrapolator = TerminalExtrapolator(TerminalConfig(enable=False))
        command = np.array([1.0, 2.0, -0.5])

        result = extrapolator.update(
            timestamp=0.0,
            detected=True,
            measurement_valid=True,
            measurement_score=1.0,
            bbox_area=100.0,
            image_width=100,
            image_height=100,
            reject_reason="",
            v_cmd=command,
            lambda_I=np.array([1.0, 0.0, 0.0]),
            omega_los=np.zeros(3),
            speed_cap=10.0,
            max_vertical_speed=3.0,
        )

        self.assertEqual(result.state, DISABLED)
        self.assertFalse(result.using_blind_push)
        self.assertTrue(np.allclose(result.v_cmd, command))

    def test_terminal_visual_then_blind_push_from_clipped_bbox(self):
        extrapolator = TerminalExtrapolator(
            TerminalConfig(
                terminal_enter_area_ratio=0.20,
                cutoff_area_ratio=0.60,
                min_tracking_time_s=0.10,
                max_measurement_age_s=0.20,
                command_average_window_s=0.10,
                pitch_up_bias_mps=0.8,
                trend_bias_gain=0.0,
            )
        )
        for ts, command in [
            (0.00, np.array([5.0, 0.0, 0.0])),
            (0.05, np.array([7.0, 0.0, 0.0])),
            (0.10, np.array([9.0, 0.0, 0.0])),
        ]:
            result = extrapolator.update(
                timestamp=ts,
                detected=True,
                measurement_valid=True,
                measurement_score=1.0,
                bbox_area=2500.0,
                image_width=100,
                image_height=100,
                reject_reason="",
                v_cmd=command,
                lambda_I=np.array([1.0, 0.0, 0.0]),
                omega_los=np.zeros(3),
                speed_cap=20.0,
                max_vertical_speed=3.0,
            )
        self.assertEqual(result.state, TERMINAL_VISUAL)

        result = extrapolator.update(
            timestamp=0.12,
            detected=True,
            measurement_valid=False,
            measurement_score=1.0,
            bbox_area=6500.0,
            image_width=100,
            image_height=100,
            reject_reason="bbox_clipped",
            v_cmd=np.array([100.0, 0.0, 0.0]),
            lambda_I=np.array([1.0, 0.0, 0.0]),
            omega_los=np.zeros(3),
            speed_cap=20.0,
            max_vertical_speed=3.0,
        )

        self.assertEqual(result.state, BLIND_PUSH)
        self.assertTrue(result.using_blind_push)
        self.assertEqual(result.terminal_arm_source, "valid_guidance")
        self.assertEqual(result.terminal_cutoff_source, "bbox_clipped")
        self.assertAlmostEqual(result.v_cmd_base[0], 8.0)
        self.assertLess(result.v_cmd[2], 0.0)
        self.assertLess(result.v_cmd[0], 100.0)

    def test_directional_clipped_bbox_enters_blind_push(self):
        extrapolator = TerminalExtrapolator(
            TerminalConfig(
                terminal_enter_area_ratio=0.10,
                min_tracking_time_s=0.0,
                max_measurement_age_s=0.20,
                command_average_window_s=0.10,
                pitch_up_bias_mps=0.0,
            )
        )
        extrapolator.update(
            timestamp=0.0,
            detected=True,
            measurement_valid=True,
            measurement_score=1.0,
            bbox_area=2000.0,
            image_width=100,
            image_height=100,
            reject_reason="",
            v_cmd=np.array([5.0, 0.0, 0.0]),
            lambda_I=np.array([1.0, 0.0, 0.0]),
            omega_los=np.zeros(3),
            speed_cap=10.0,
            max_vertical_speed=3.0,
        )

        result = extrapolator.update(
            timestamp=0.05,
            detected=True,
            measurement_valid=False,
            measurement_score=1.0,
            bbox_area=2000.0,
            image_width=100,
            image_height=100,
            reject_reason="bbox_top_clipped",
            v_cmd=np.array([5.0, 0.0, 0.0]),
            lambda_I=np.array([1.0, 0.0, 0.0]),
            omega_los=np.zeros(3),
            speed_cap=10.0,
            max_vertical_speed=3.0,
        )

        self.assertEqual(result.state, BLIND_PUSH)
        self.assertTrue(result.using_blind_push)
        self.assertEqual(result.terminal_cutoff_source, "bbox_top_clipped")

    def test_soft_image_kf_measurement_arms_terminal_visual(self):
        extrapolator = TerminalExtrapolator(
            TerminalConfig(
                terminal_enter_area_ratio=0.20,
                soft_enter_area_ratio=0.05,
            )
        )

        result = extrapolator.update(
            timestamp=0.0,
            detected=True,
            measurement_valid=False,
            measurement_score=1.0,
            bbox_area=600.0,
            image_width=100,
            image_height=100,
            reject_reason="area_not_expanding",
            v_cmd=np.array([5.0, 0.0, 0.0]),
            lambda_I=np.array([1.0, 0.0, 0.0]),
            omega_los=np.zeros(3),
            speed_cap=10.0,
            max_vertical_speed=3.0,
            soft_measurement_valid=True,
        )

        self.assertEqual(result.state, TERMINAL_VISUAL)
        self.assertFalse(result.using_blind_push)
        self.assertEqual(result.terminal_arm_source, "image_kf_soft")

    def test_gimbal_limit_cutoff_uses_lower_terminal_gimbal_area(self):
        extrapolator = TerminalExtrapolator(
            TerminalConfig(
                terminal_enter_area_ratio=0.20,
                soft_enter_area_ratio=0.05,
                terminal_gimbal_limit_area_ratio=0.05,
                min_tracking_time_s=0.0,
                max_measurement_age_s=0.50,
                command_average_window_s=0.20,
            )
        )
        extrapolator.update(
            timestamp=0.0,
            detected=True,
            measurement_valid=True,
            measurement_score=1.0,
            bbox_area=400.0,
            image_width=100,
            image_height=100,
            reject_reason="",
            v_cmd=np.array([5.0, 0.0, 0.0]),
            lambda_I=np.array([1.0, 0.0, 0.0]),
            omega_los=np.zeros(3),
            speed_cap=10.0,
            max_vertical_speed=3.0,
        )

        result = extrapolator.update(
            timestamp=0.05,
            detected=True,
            measurement_valid=False,
            measurement_score=1.0,
            bbox_area=600.0,
            image_width=100,
            image_height=100,
            reject_reason="",
            v_cmd=np.array([5.0, 0.0, 0.0]),
            lambda_I=np.array([1.0, 0.0, 0.0]),
            omega_los=np.zeros(3),
            speed_cap=10.0,
            max_vertical_speed=3.0,
            gimbal_at_limit=True,
            soft_measurement_valid=True,
        )

        self.assertEqual(result.state, BLIND_PUSH)
        self.assertEqual(result.reason, "gimbal_limit")
        self.assertEqual(result.terminal_cutoff_source, "gimbal_limit")

    def test_terminal_lost_enters_blind_push_after_miss_count(self):
        extrapolator = TerminalExtrapolator(
            TerminalConfig(
                cutoff_miss_count=2,
                min_tracking_time_s=0.05,
                max_measurement_age_s=0.20,
                command_average_window_s=0.20,
            )
        )
        for ts in (0.00, 0.05, 0.10):
            extrapolator.update(
                timestamp=ts,
                detected=True,
                measurement_valid=True,
                measurement_score=1.0,
                bbox_area=2500.0,
                image_width=100,
                image_height=100,
                reject_reason="",
                v_cmd=np.array([5.0, 0.0, 0.0]),
                lambda_I=np.array([1.0, 0.0, 0.0]),
                omega_los=np.zeros(3),
                speed_cap=10.0,
                max_vertical_speed=3.0,
            )

        extrapolator.update(
            timestamp=0.15,
            detected=False,
            measurement_valid=False,
            measurement_score=0.0,
            bbox_area=0.0,
            image_width=100,
            image_height=100,
            reject_reason="no_detection",
            v_cmd=np.array([5.0, 0.0, 0.0]),
            lambda_I=None,
            omega_los=None,
            speed_cap=10.0,
            max_vertical_speed=3.0,
        )
        result = extrapolator.update(
            timestamp=0.20,
            detected=False,
            measurement_valid=False,
            measurement_score=0.0,
            bbox_area=0.0,
            image_width=100,
            image_height=100,
            reject_reason="no_detection",
            v_cmd=np.array([5.0, 0.0, 0.0]),
            lambda_I=None,
            omega_los=None,
            speed_cap=10.0,
            max_vertical_speed=3.0,
        )

        self.assertEqual(result.state, BLIND_PUSH)
        self.assertEqual(result.reason, "terminal_lost")

    def test_tracking_without_terminal_area_stays_tracking(self):
        extrapolator = TerminalExtrapolator(TerminalConfig(terminal_enter_area_ratio=0.20))

        result = extrapolator.update(
            timestamp=0.0,
            detected=True,
            measurement_valid=True,
            measurement_score=1.0,
            bbox_area=100.0,
            image_width=100,
            image_height=100,
            reject_reason="",
            v_cmd=np.array([5.0, 0.0, 0.0]),
            lambda_I=np.array([1.0, 0.0, 0.0]),
            omega_los=np.zeros(3),
            speed_cap=10.0,
            max_vertical_speed=3.0,
        )

        self.assertEqual(result.state, TRACKING)
        self.assertFalse(result.using_blind_push)

    def test_safety_gate_enters_abort_hold(self):
        extrapolator = TerminalExtrapolator()

        result = extrapolator.update(
            timestamp=0.0,
            detected=True,
            measurement_valid=True,
            measurement_score=1.0,
            bbox_area=9000.0,
            image_width=100,
            image_height=100,
            reject_reason="bbox_clipped",
            v_cmd=np.array([5.0, 0.0, 0.0]),
            lambda_I=np.array([1.0, 0.0, 0.0]),
            omega_los=np.zeros(3),
            speed_cap=10.0,
            max_vertical_speed=3.0,
            safety_ok=False,
        )

        self.assertEqual(result.state, ABORT_HOLD)
        self.assertFalse(result.using_blind_push)
        self.assertEqual(result.reason, "safety_abort")


if __name__ == "__main__":
    unittest.main()
