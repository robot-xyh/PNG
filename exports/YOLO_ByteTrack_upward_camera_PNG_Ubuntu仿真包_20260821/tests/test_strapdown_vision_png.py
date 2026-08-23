import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from examples.run_airsim_strapdown_vision_png import (
    _accel_integrated_velocity,
    _append_yaw_rate_sample,
    _apply_bbox_noise,
    _body_rate_control_acceleration,
    _body_rate_command_from_accel,
    _body_rate_neutral_thrust,
    _candidate_guidance_velocity,
    _command_vehicle_velocity,
    _clipped_los_uses_image_kf_predict,
    _fixed_camera_R_BC,
    _fixed_camera_pose,
    _frame_centering_yaw_rate,
    _image_kf_takeover_allowed,
    _attitude_command_from_accel,
    _attitude_control_acceleration,
    _attitude_neutral_thrust,
    _normalized_thrust_from_accel,
    _predict_los_delay,
    _kf_yaw_rate_command,
    _output_paths_strapdown,
    _png_acceleration_command,
    _plot_strapdown,
    _rows_until_first_hit,
    _append_terminal_accel_sample,
    _should_update_last_velocity,
    _terminal_accel_hold_command,
    _start_geometry_offsets,
    _terminal_trigger_strapdown,
    _terminal_yaw_rate_command,
    _upward_centering_acceleration,
    _validate_runtime_args,
    _yaw_rate_from_angle_error,
    _yaw_rate_from_pixel_error,
)
from vision_guidance.terminal_image_kf import TerminalImageEstimate
from vision_guidance.terminal_extrapolation import BLIND_PUSH, TERMINAL_VISUAL
from vision_guidance.types import CameraIntrinsics, FrameDetection


AIRSIM_HOVER_THRUST = 9.80665 / 16.717785072


def _runtime_args(**overrides):
    args = {
        "rate_hz": 8.0,
        "speed_ratio": 2.0,
        "navigation_constant": 3.0,
        "guidance_output_mode": "accel_body_rate",
        "px4_interceptor": True,
        "px4_command_mode": "mavlink_body_rate",
        "thrust_model": "airsim_generic_quad",
        "vehicle_mass_kg": 1.0,
        "vehicle_max_total_thrust_n": 16.717785072,
        "body_rate_min_thrust": 0.25,
        "body_rate_max_thrust": 0.75,
        "body_rate_v2_kp_roll": 5.0,
        "body_rate_v2_kp_pitch": 5.0,
        "body_rate_v2_kp_yaw": 3.0,
        "body_rate_v2_max_pq_rate_deg_s": 120.0,
        "body_rate_v2_slew_pq_deg_s2": 720.0,
        "body_rate_v2_slew_r_deg_s2": 540.0,
        "body_rate_v2_thrust_reserve": 0.15,
        "body_rate_v2_guard_png_scale": 0.60,
        "body_rate_v2_guard_speed_hold_scale": 0.45,
        "attitude_min_thrust": 0.25,
        "attitude_max_thrust": 0.80,
        "los_filter_process_lambda": 1e-4,
        "los_filter_process_lambda_dot": 5e-3,
        "los_filter_measurement_noise": 5e-3,
        "los_filter_innovation_reject": 0.25,
        "los_filter_terminal_innovation_reject": 1.20,
        "los_filter_terminal_area_ratio": 0.01,
        "los_filter_terminal_error_ratio": 0.55,
        "los_delay_compensation_s": 0.18,
        "terminal_accel_hold_window_s": 0.35,
        "terminal_accel_decay_tau_s": 0.60,
        "terminal_accel_hold_max_mps2": 0.0,
        "terminal_velocity_blind_push": True,
        "terminal_accel_hold": False,
        "terminal_clipped_los_kf_predict": None,
        "terminal_blind_duration_s": 1.0,
        "near_hit_radius_m": 1.5,
        "upward_centering": False,
        "upward_centering_gain": 8.0,
        "upward_centering_max_accel_mps2": 4.0,
        "camera_pitch_deg": 0.0,
    }
    args.update(overrides)
    return SimpleNamespace(**args)


class StrapdownVisionPNGTest(unittest.TestCase):
    def test_airsim_generic_quad_thrust_model_uses_mass_and_max_thrust(self):
        args = SimpleNamespace(
            thrust_model="airsim_generic_quad",
            vehicle_mass_kg=1.0,
            vehicle_max_total_thrust_n=16.717785072,
        )

        hover, raw_hover, cos_tilt = _normalized_thrust_from_accel(
            np.zeros(3),
            roll_rad=0.0,
            pitch_rad=0.0,
            min_thrust=0.0,
            max_thrust=1.0,
            hover_thrust=0.5,
            thrust_gain=0.5,
            args=args,
        )
        climb, raw_climb, _ = _normalized_thrust_from_accel(
            np.array([0.0, 0.0, -4.903325]),
            roll_rad=0.0,
            pitch_rad=0.0,
            min_thrust=0.0,
            max_thrust=1.0,
            hover_thrust=0.5,
            thrust_gain=0.5,
            args=args,
        )
        tilted, raw_tilted, tilted_cos = _normalized_thrust_from_accel(
            np.zeros(3),
            roll_rad=np.deg2rad(20.0),
            pitch_rad=0.0,
            min_thrust=0.0,
            max_thrust=1.0,
            hover_thrust=0.5,
            thrust_gain=0.5,
            args=args,
        )

        self.assertAlmostEqual(hover, AIRSIM_HOVER_THRUST, places=6)
        self.assertAlmostEqual(raw_hover, AIRSIM_HOVER_THRUST, places=6)
        self.assertAlmostEqual(cos_tilt, 1.0)
        self.assertAlmostEqual(climb, 1.5 * AIRSIM_HOVER_THRUST, places=6)
        self.assertAlmostEqual(raw_climb, climb, places=6)
        self.assertGreater(tilted, hover)
        self.assertLess(tilted_cos, 1.0)

    def test_empirical_thrust_model_remains_available(self):
        args = SimpleNamespace(thrust_model="empirical")

        thrust, raw, _ = _normalized_thrust_from_accel(
            np.array([0.0, 0.0, -4.903325]),
            roll_rad=np.deg2rad(20.0),
            pitch_rad=0.0,
            min_thrust=0.0,
            max_thrust=1.0,
            hover_thrust=0.5,
            thrust_gain=0.5,
            args=args,
        )

        self.assertAlmostEqual(raw, 0.75)
        self.assertAlmostEqual(thrust, 0.75)

    def test_neutral_thrust_uses_airsim_model_defaults(self):
        args = _runtime_args(
            body_rate_hover_thrust=0.5,
            body_rate_thrust_gain=0.5,
            body_rate_min_thrust=0.0,
            body_rate_max_thrust=1.0,
            attitude_hover_thrust=0.5,
            attitude_thrust_gain=0.5,
            attitude_min_thrust=0.0,
            attitude_max_thrust=1.0,
        )

        self.assertAlmostEqual(_body_rate_neutral_thrust(args), AIRSIM_HOVER_THRUST, places=6)
        self.assertAlmostEqual(_attitude_neutral_thrust(args), AIRSIM_HOVER_THRUST, places=6)

    def test_png_acceleration_command_has_acceleration_units_and_limit(self):
        args = SimpleNamespace(max_guidance_accel_mps2=2.0)
        lambda_i = np.array([1.0, 0.0, 0.0])
        omega_los = np.array([0.0, 0.0, 1.0])

        accel = _png_acceleration_command(lambda_i, omega_los, 5.0, args)

        self.assertAlmostEqual(np.linalg.norm(accel), 2.0)
        self.assertAlmostEqual(accel[1], 2.0)

    def test_upward_centering_acceleration_uses_body_xy_los_error(self):
        args = SimpleNamespace(
            upward_centering=True,
            camera_pitch_deg=-90.0,
            upward_centering_gain=8.0,
            upward_centering_max_accel_mps2=10.0,
        )
        lambda_i = np.array([0.5, -0.25, -1.0])

        accel, err, active = _upward_centering_acceleration(lambda_i, np.eye(3), args)

        self.assertEqual(active, 1)
        expected_err = lambda_i[:2] / np.linalg.norm(lambda_i)
        np.testing.assert_allclose(err, expected_err)
        np.testing.assert_allclose(accel, np.array([8.0 * expected_err[0], 8.0 * expected_err[1], 0.0]))

    def test_upward_centering_acceleration_requires_upward_camera_and_clips(self):
        args = SimpleNamespace(
            upward_centering=True,
            camera_pitch_deg=-90.0,
            upward_centering_gain=10.0,
            upward_centering_max_accel_mps2=2.0,
        )

        accel, _, active = _upward_centering_acceleration(np.array([1.0, 0.0, -1.0]), np.eye(3), args)

        self.assertEqual(active, 1)
        self.assertAlmostEqual(np.linalg.norm(accel), 2.0)
        self.assertGreater(accel[0], 0.0)

        args.camera_pitch_deg = -60.0
        accel, err, active = _upward_centering_acceleration(np.array([1.0, 0.0, -1.0]), np.eye(3), args)

        self.assertEqual(active, 0)
        np.testing.assert_allclose(accel, np.zeros(3))
        np.testing.assert_allclose(err, np.zeros(2))

    def test_terminal_accel_hold_uses_recent_decaying_average(self):
        args = SimpleNamespace(
            terminal_accel_hold=True,
            terminal_accel_hold_window_s=0.30,
            terminal_accel_decay_tau_s=0.60,
            terminal_accel_hold_max_mps2=10.0,
            max_guidance_accel_mps2=20.0,
            terminal_blind_duration_s=1.0,
        )
        samples: list[tuple[float, np.ndarray]] = []
        _append_terminal_accel_sample(samples, 0.00, np.array([4.0, 0.0, 0.0]), True, args)
        _append_terminal_accel_sample(samples, 0.20, np.array([2.0, 2.0, 0.0]), True, args)

        command, base, count, decay, active = _terminal_accel_hold_command(
            current_accel_I=np.zeros(3),
            samples=samples,
            timestamp=0.31,
            using_blind_push=True,
            blind_elapsed_s=0.30,
            args=args,
        )

        self.assertEqual(active, 1)
        self.assertEqual(count, 1)
        np.testing.assert_allclose(base, np.array([2.0, 2.0, 0.0]))
        self.assertAlmostEqual(decay, np.exp(-0.5))
        np.testing.assert_allclose(command, decay * base)

    def test_terminal_accel_hold_respects_disable_and_limit(self):
        args = SimpleNamespace(
            terminal_accel_hold=True,
            terminal_accel_hold_window_s=1.0,
            terminal_accel_decay_tau_s=1.0,
            terminal_accel_hold_max_mps2=3.0,
            max_guidance_accel_mps2=20.0,
            terminal_blind_duration_s=1.0,
        )
        samples = [(0.0, np.array([10.0, 0.0, 0.0]))]

        command, _, _, _, active = _terminal_accel_hold_command(
            current_accel_I=np.array([1.0, 2.0, 3.0]),
            samples=samples,
            timestamp=0.0,
            using_blind_push=True,
            blind_elapsed_s=0.0,
            args=args,
        )

        self.assertEqual(active, 1)
        self.assertAlmostEqual(np.linalg.norm(command), 3.0)

        args.terminal_accel_hold = False
        command, _, _, _, active = _terminal_accel_hold_command(
            current_accel_I=np.array([1.0, 2.0, 3.0]),
            samples=samples,
            timestamp=0.0,
            using_blind_push=True,
            blind_elapsed_s=0.0,
            args=args,
        )

        self.assertEqual(active, 0)
        np.testing.assert_allclose(command, np.array([1.0, 2.0, 3.0]))

    def test_accel_integrated_velocity_uses_dt_and_speed_cap(self):
        args = SimpleNamespace(min_speed_ratio=0.0)
        current = np.array([5.0, 0.0, 0.0])
        accel = np.array([0.0, 4.0, 0.0])

        command = _accel_integrated_velocity(current, accel, 0.25, 10.0, np.array([1.0, 0.0, 0.0]), args)

        self.assertAlmostEqual(command[0], 5.0)
        self.assertAlmostEqual(command[1], 1.0)
        self.assertLessEqual(np.linalg.norm(command), 10.0)

    def test_candidate_guidance_velocity_accel_integral_and_legacy_modes(self):
        current = np.array([4.0, 0.0, 0.0])
        lambda_i = np.array([1.0, 0.0, 0.0])
        omega_los = np.array([0.0, 0.0, 1.0])
        args = SimpleNamespace(
            guidance_output_mode="accel_integral",
            max_guidance_accel_mps2=100.0,
            min_speed_ratio=0.0,
            accel_integral_reset_on_invalid=False,
            max_vision_lateral_speed=100.0,
            max_vision_vertical_speed=100.0,
        )

        command, accel, dt = _candidate_guidance_velocity(current, lambda_i, omega_los, 3.0, 10.0, 0.2, True, args)

        self.assertAlmostEqual(accel[1], 3.0)
        self.assertAlmostEqual(command[1], 0.6)
        self.assertAlmostEqual(dt, 0.2)

        args.guidance_output_mode = "velocity_bias"
        command, accel, dt = _candidate_guidance_velocity(current, lambda_i, omega_los, 3.0, 10.0, 0.2, True, args)

        self.assertGreater(command[1], 0.0)
        self.assertLessEqual(np.linalg.norm(command), 10.0)
        self.assertAlmostEqual(accel[1], 3.0)
        self.assertAlmostEqual(dt, 0.0)

    def test_candidate_guidance_velocity_accel_body_rate_computes_accel_without_integrating_velocity(self):
        current = np.array([4.0, 0.0, 0.0])
        lambda_i = np.array([1.0, 0.0, 0.0])
        omega_los = np.array([0.0, 0.0, 1.0])
        args = SimpleNamespace(
            guidance_output_mode="accel_body_rate",
            max_guidance_accel_mps2=100.0,
            min_speed_ratio=0.0,
            accel_integral_reset_on_invalid=False,
            max_vision_lateral_speed=100.0,
            max_vision_vertical_speed=100.0,
        )

        command, accel, dt = _candidate_guidance_velocity(current, lambda_i, omega_los, 3.0, 10.0, 0.2, True, args)

        self.assertAlmostEqual(accel[1], 3.0)
        self.assertAlmostEqual(command[0], 10.0)
        self.assertAlmostEqual(command[1], 0.0)
        self.assertAlmostEqual(dt, 0.0)

        args.guidance_output_mode = "accel_attitude"
        command, accel, dt = _candidate_guidance_velocity(current, lambda_i, omega_los, 3.0, 10.0, 0.2, True, args)

        self.assertAlmostEqual(accel[1], 3.0)
        self.assertAlmostEqual(command[0], 10.0)
        self.assertAlmostEqual(command[1], 0.0)
        self.assertAlmostEqual(dt, 0.0)

    def test_body_rate_command_from_accel_maps_body_lateral_and_vertical(self):
        args = SimpleNamespace(
            thrust_model="airsim_generic_quad",
            vehicle_mass_kg=1.0,
            vehicle_max_total_thrust_n=16.717785072,
            body_rate_max_tilt_deg=20.0,
            body_rate_roll_gain=1.0,
            body_rate_pitch_gain=1.0,
            body_rate_attitude_p=4.0,
            body_rate_max_roll_rate_deg=60.0,
            body_rate_max_pitch_rate_deg=60.0,
            max_yaw_rate_deg=90.0,
            body_rate_hover_thrust=AIRSIM_HOVER_THRUST,
            body_rate_thrust_gain=AIRSIM_HOVER_THRUST,
            body_rate_min_thrust=0.25,
            body_rate_max_thrust=0.95,
        )

        result = _body_rate_command_from_accel(
            np.array([0.0, 4.903325, -4.903325]),
            np.eye(3),
            0.0,
            0.0,
            0.0,
            30.0,
            0.1,
            args,
        )

        self.assertGreater(result["roll_sp_rad"], 0.0)
        self.assertAlmostEqual(result["pitch_sp_rad"], 0.0)
        self.assertGreater(result["body_rates_rad_s"][0], 0.0)
        self.assertAlmostEqual(result["body_rates_rad_s"][2], np.deg2rad(30.0))
        self.assertGreater(result["thrust"], AIRSIM_HOVER_THRUST)
        self.assertAlmostEqual(result["thrust"], result["thrust_raw"])

    def test_body_rate_command_from_accel_limits_tilt_rates_and_thrust(self):
        args = SimpleNamespace(
            thrust_model="airsim_generic_quad",
            vehicle_mass_kg=1.0,
            vehicle_max_total_thrust_n=16.717785072,
            body_rate_max_tilt_deg=10.0,
            body_rate_roll_gain=10.0,
            body_rate_pitch_gain=10.0,
            body_rate_attitude_p=20.0,
            body_rate_max_roll_rate_deg=45.0,
            body_rate_max_pitch_rate_deg=40.0,
            max_yaw_rate_deg=30.0,
            body_rate_hover_thrust=AIRSIM_HOVER_THRUST,
            body_rate_thrust_gain=AIRSIM_HOVER_THRUST,
            body_rate_min_thrust=0.2,
            body_rate_max_thrust=0.8,
        )

        result = _body_rate_command_from_accel(
            np.array([100.0, -100.0, -100.0]),
            np.eye(3),
            0.0,
            0.0,
            0.0,
            90.0,
            0.1,
            args,
        )

        self.assertAlmostEqual(result["roll_sp_rad"], -np.deg2rad(10.0))
        self.assertAlmostEqual(result["pitch_sp_rad"], -np.deg2rad(10.0))
        self.assertAlmostEqual(result["body_rates_rad_s"][0], -np.deg2rad(45.0))
        self.assertAlmostEqual(result["body_rates_rad_s"][1], -np.deg2rad(40.0))
        self.assertAlmostEqual(result["body_rates_rad_s"][2], np.deg2rad(30.0))
        self.assertAlmostEqual(result["thrust"], 0.8)

    def test_body_rate_control_acceleration_adds_limited_speed_hold(self):
        args = SimpleNamespace(
            body_rate_speed_hold_gain=2.0,
            body_rate_speed_hold_max_accel_mps2=3.0,
            body_rate_total_accel_limit_mps2=10.0,
        )

        total, speed_hold = _body_rate_control_acceleration(
            png_acceleration_I=np.array([0.0, 1.0, 0.0]),
            current_velocity_I=np.zeros(3),
            velocity_reference_I=np.array([10.0, 0.0, 0.0]),
            args=args,
        )

        self.assertAlmostEqual(np.linalg.norm(speed_hold), 3.0)
        self.assertAlmostEqual(total[0], 3.0)
        self.assertAlmostEqual(total[1], 1.0)

    def test_attitude_command_from_accel_maps_accel_to_quaternion_and_thrust(self):
        args = SimpleNamespace(
            thrust_model="airsim_generic_quad",
            vehicle_mass_kg=1.0,
            vehicle_max_total_thrust_n=16.717785072,
            attitude_max_tilt_deg=25.0,
            attitude_yaw_lookahead_s=0.25,
            max_yaw_rate_deg=60.0,
            attitude_hover_thrust=AIRSIM_HOVER_THRUST,
            attitude_thrust_gain=AIRSIM_HOVER_THRUST,
            attitude_min_thrust=0.25,
            attitude_max_thrust=0.95,
        )

        result = _attitude_command_from_accel(
            np.array([0.0, 4.903325, -4.903325]),
            np.array([1.0, 0.0, 0.0]),
            0.0,
            0.0,
            args,
        )

        self.assertGreater(result["roll_sp_rad"], 0.0)
        self.assertAlmostEqual(result["pitch_sp_rad"], 0.0)
        self.assertAlmostEqual(np.linalg.norm(result["attitude_quat_wxyz"]), 1.0)
        self.assertGreater(result["thrust"], AIRSIM_HOVER_THRUST)

    def test_attitude_command_maps_inertial_accel_through_commanded_yaw(self):
        args = SimpleNamespace(
            thrust_model="airsim_generic_quad",
            vehicle_mass_kg=1.0,
            vehicle_max_total_thrust_n=16.717785072,
            attitude_max_tilt_deg=25.0,
            attitude_yaw_lookahead_s=0.0,
            max_yaw_rate_deg=60.0,
            attitude_hover_thrust=AIRSIM_HOVER_THRUST,
            attitude_thrust_gain=AIRSIM_HOVER_THRUST,
            attitude_min_thrust=0.25,
            attitude_max_thrust=0.8,
        )

        result = _attitude_command_from_accel(
            np.array([0.0, 4.903325, 0.0]),
            None,
            0.0,
            np.deg2rad(90.0),
            args,
        )

        self.assertAlmostEqual(result["roll_sp_rad"], 0.0, places=6)
        self.assertLess(result["pitch_sp_rad"], 0.0)
        np.testing.assert_allclose(result["accel_yaw_body"][:2], np.array([4.903325, 0.0]), atol=1.0e-6)

    def test_attitude_control_acceleration_uses_attitude_limits(self):
        args = SimpleNamespace(
            attitude_speed_hold_gain=2.0,
            attitude_speed_hold_max_accel_mps2=3.0,
            attitude_total_accel_limit_mps2=10.0,
        )

        total, speed_hold = _attitude_control_acceleration(
            png_acceleration_I=np.array([0.0, 1.0, 0.0]),
            current_velocity_I=np.zeros(3),
            velocity_reference_I=np.array([10.0, 0.0, 0.0]),
            args=args,
        )

        self.assertAlmostEqual(np.linalg.norm(speed_hold), 3.0)
        self.assertAlmostEqual(total[0], 3.0)
        self.assertAlmostEqual(total[1], 1.0)

    def test_predict_los_delay_advances_los_with_los_rate(self):
        lam, omega = _predict_los_delay(
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
            0.1,
        )

        self.assertGreater(lam[1], 0.0)
        self.assertAlmostEqual(np.linalg.norm(lam), 1.0)
        self.assertLess(abs(float(np.dot(lam, omega))), 1.0e-9)

    def test_accel_body_rate_runtime_validation_requires_px4_body_rate_mode(self):
        args = _runtime_args(
            guidance_output_mode="accel_body_rate",
            px4_interceptor=False,
            px4_command_mode="velocity_yaw_rate",
        )

        with self.assertRaisesRegex(SystemExit, "requires --px4-interceptor"):
            _validate_runtime_args(args)

        args.px4_interceptor = True
        with self.assertRaisesRegex(SystemExit, "requires --px4-command-mode mavlink_body_rate"):
            _validate_runtime_args(args)

        args.px4_command_mode = "mavlink_body_rate"
        _validate_runtime_args(args)

    def test_terminal_strategy_defaults_split_upward_accel_from_forward_camera(self):
        args = _runtime_args(
            camera_pitch_deg=-90.0,
            terminal_velocity_blind_push=None,
            terminal_accel_hold=None,
            terminal_blind_requires_visual_loss=None,
            terminal_clipped_los_kf_predict=None,
        )

        _validate_runtime_args(args)

        self.assertFalse(args.terminal_velocity_blind_push)
        self.assertTrue(args.terminal_accel_hold)
        self.assertTrue(args.terminal_blind_requires_visual_loss)
        self.assertTrue(args.terminal_clipped_los_kf_predict)

        args = _runtime_args(
            camera_pitch_deg=0.0,
            terminal_velocity_blind_push=None,
            terminal_accel_hold=None,
            terminal_blind_requires_visual_loss=None,
            terminal_clipped_los_kf_predict=None,
        )

        _validate_runtime_args(args)

        self.assertTrue(args.terminal_velocity_blind_push)
        self.assertFalse(args.terminal_accel_hold)
        self.assertFalse(args.terminal_blind_requires_visual_loss)
        self.assertFalse(args.terminal_clipped_los_kf_predict)

    def test_terminal_strategy_explicit_overrides_are_preserved(self):
        args = _runtime_args(
            camera_pitch_deg=-90.0,
            terminal_velocity_blind_push=True,
            terminal_accel_hold=False,
            terminal_blind_requires_visual_loss=False,
            terminal_clipped_los_kf_predict=False,
        )

        _validate_runtime_args(args)

        self.assertTrue(args.terminal_velocity_blind_push)
        self.assertFalse(args.terminal_accel_hold)
        self.assertFalse(args.terminal_blind_requires_visual_loss)
        self.assertFalse(args.terminal_clipped_los_kf_predict)

    def test_clipped_los_kf_predict_requires_clipped_bbox_and_image_kf_guidance(self):
        args = SimpleNamespace(
            terminal_clipped_los_kf_predict=True,
            terminal_image_kf=True,
            terminal_image_kf_guidance=True,
        )

        self.assertTrue(_clipped_los_uses_image_kf_predict(True, args))
        self.assertFalse(_clipped_los_uses_image_kf_predict(False, args))

        args.terminal_image_kf_guidance = False
        self.assertFalse(_clipped_los_uses_image_kf_predict(True, args))

        args.terminal_image_kf_guidance = True
        args.terminal_image_kf = False
        self.assertFalse(_clipped_los_uses_image_kf_predict(True, args))

    def test_upward_loss_hold_does_not_refresh_last_velocity_from_kf_prediction(self):
        args = _runtime_args(camera_pitch_deg=-90.0, guidance_output_mode="accel_body_rate")

        self.assertFalse(_should_update_last_velocity(True, False, True, args))
        self.assertTrue(_should_update_last_velocity(True, True, True, args))
        self.assertTrue(_should_update_last_velocity(True, False, False, args))
        self.assertFalse(_should_update_last_velocity(False, False, True, args))

        args.camera_pitch_deg = 0.0
        self.assertTrue(_should_update_last_velocity(True, False, True, args))

    def test_mavlink_body_rate_runtime_validation_rejects_other_output_modes(self):
        args = _runtime_args(
            guidance_output_mode="accel_integral",
            px4_command_mode="mavlink_body_rate",
        )

        with self.assertRaisesRegex(SystemExit, "requires --guidance-output-mode accel_body_rate"):
            _validate_runtime_args(args)

    def test_accel_attitude_runtime_validation_requires_px4_attitude_mode(self):
        args = _runtime_args(
            guidance_output_mode="accel_attitude",
            px4_interceptor=False,
            px4_command_mode="velocity_yaw_rate",
        )

        with self.assertRaisesRegex(SystemExit, "requires --px4-interceptor"):
            _validate_runtime_args(args)

        args.px4_interceptor = True
        with self.assertRaisesRegex(SystemExit, "requires --px4-command-mode mavlink_attitude"):
            _validate_runtime_args(args)

        args.px4_command_mode = "mavlink_attitude"
        _validate_runtime_args(args)

    def test_mavlink_attitude_runtime_validation_rejects_other_output_modes(self):
        args = _runtime_args(
            guidance_output_mode="accel_integral",
            px4_command_mode="mavlink_attitude",
        )

        with self.assertRaisesRegex(SystemExit, "requires --guidance-output-mode accel_attitude"):
            _validate_runtime_args(args)

    def test_body_rate_runtime_validation_rejects_invalid_thrust_range(self):
        args = _runtime_args(
            body_rate_min_thrust=0.8,
            body_rate_max_thrust=0.2,
        )

        with self.assertRaisesRegex(SystemExit, "min-thrust cannot exceed"):
            _validate_runtime_args(args)

    def test_attitude_runtime_validation_rejects_invalid_thrust_range(self):
        args = _runtime_args(
            guidance_output_mode="accel_attitude",
            px4_command_mode="mavlink_attitude",
            attitude_min_thrust=0.9,
            attitude_max_thrust=0.2,
        )

        with self.assertRaisesRegex(SystemExit, "attitude-min-thrust cannot exceed"):
            _validate_runtime_args(args)

    def test_runtime_validation_rejects_invalid_los_filter_parameters(self):
        for key, value, message in (
            ("los_filter_process_lambda", -1e-4, "process-lambda must be non-negative"),
            ("los_filter_process_lambda_dot", -1e-3, "process-lambda-dot must be non-negative"),
            ("los_filter_measurement_noise", 0.0, "measurement-noise must be positive"),
            ("los_filter_innovation_reject", 0.0, "innovation-reject must be positive"),
            ("los_filter_terminal_innovation_reject", 0.0, "terminal-innovation-reject must be positive"),
            ("los_filter_terminal_area_ratio", -0.1, "terminal-area-ratio must be non-negative"),
            ("los_filter_terminal_error_ratio", -0.1, "terminal-error-ratio must be non-negative"),
            ("los_delay_compensation_s", -0.1, "delay-compensation-s must be non-negative"),
            ("terminal_accel_hold_window_s", -0.1, "terminal-accel-hold-window-s must be non-negative"),
            ("terminal_accel_decay_tau_s", -0.1, "terminal-accel-decay-tau-s must be non-negative"),
            ("terminal_accel_hold_max_mps2", -0.1, "terminal-accel-hold-max-mps2 must be non-negative"),
            ("near_hit_radius_m", -0.1, "near-hit-radius-m must be non-negative"),
            ("upward_centering_gain", -0.1, "upward-centering-gain must be non-negative"),
            ("upward_centering_max_accel_mps2", -0.1, "upward-centering-max-accel-mps2 must be non-negative"),
        ):
            args = _runtime_args(**{key: value})
            with self.subTest(key=key), self.assertRaisesRegex(SystemExit, message):
                _validate_runtime_args(args)

    def test_mavlink_attitude_velocity_command_still_streams_velocity_for_preparation(self):
        class FakeOffboard:
            def __init__(self):
                self.velocity_calls = []
                self.armed = 0
                self.offboard = 0

            def send_velocity(self, command, yaw_rate_deg_s):
                self.velocity_calls.append((np.asarray(command, dtype=float), float(yaw_rate_deg_s)))

            def arm(self):
                self.armed += 1

            def request_offboard(self):
                self.offboard += 1

        args = SimpleNamespace(
            px4_interceptor=True,
            px4_intruder=False,
            interceptor="Interceptor",
            intruder="Intruder",
            px4_command_mode="mavlink_attitude",
            px4_max_vertical_speed=2.0,
            _px4_mavlink_offboard=FakeOffboard(),
        )

        future = _command_vehicle_velocity(
            client=None,
            airsim_module=None,
            vehicle_name="Interceptor",
            velocity=np.array([1.0, 2.0, 5.0]),
            yaw_rate_deg_s=12.0,
            command_duration=0.2,
            args=args,
        )

        self.assertIsNone(future)
        self.assertEqual(len(args._px4_mavlink_offboard.velocity_calls), 1)
        command, yaw_rate = args._px4_mavlink_offboard.velocity_calls[0]
        np.testing.assert_allclose(command, np.array([1.0, 2.0, 2.0]))
        self.assertAlmostEqual(yaw_rate, 12.0)
        self.assertEqual(args._px4_mavlink_offboard.armed, 1)
        self.assertEqual(args._px4_mavlink_offboard.offboard, 1)

    def test_yaw_rate_from_pixel_error_turns_toward_right_side_target(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        args = SimpleNamespace(yaw_control=True, yaw_error_gain=2.0, max_yaw_rate_deg=90.0)

        self.assertGreater(_yaw_rate_from_pixel_error(80.0, intrinsics, args), 0.0)
        self.assertLess(_yaw_rate_from_pixel_error(-80.0, intrinsics, args), 0.0)

    def test_yaw_rate_respects_limit_and_disable_switch(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        args = SimpleNamespace(yaw_control=True, yaw_error_gain=20.0, max_yaw_rate_deg=30.0)

        self.assertAlmostEqual(_yaw_rate_from_pixel_error(640.0, intrinsics, args), 30.0)

        args.yaw_control = False
        self.assertAlmostEqual(_yaw_rate_from_pixel_error(640.0, intrinsics, args), 0.0)

    def test_yaw_rate_from_angle_error_and_kf_feedforward(self):
        args = SimpleNamespace(yaw_control=True, yaw_error_gain=2.0, max_yaw_rate_deg=30.0)

        self.assertAlmostEqual(_yaw_rate_from_angle_error(0.10, args), np.rad2deg(0.20))

        image_kf = TerminalImageEstimate(
            timestamp=0.0,
            theta_x=0.10,
            theta_y=0.0,
            theta_dot_x=0.20,
            theta_dot_y=0.0,
            valid=True,
            mode="predict",
            age_s=0.05,
            quality=0.8,
        )

        command = _kf_yaw_rate_command(image_kf, args)

        self.assertAlmostEqual(command, np.rad2deg(0.40))

        args.max_yaw_rate_deg = 10.0
        self.assertAlmostEqual(_kf_yaw_rate_command(image_kf, args), 10.0)

    def test_strapdown_image_kf_can_take_over_on_terminal_los_reject(self):
        image_kf = TerminalImageEstimate(
            timestamp=1.0,
            theta_x=0.05,
            theta_y=0.02,
            theta_dot_x=0.10,
            theta_dot_y=0.0,
            valid=True,
            mode="predict",
            age_s=0.06,
            quality=0.8,
        )
        terminal_result = SimpleNamespace(
            state=TERMINAL_VISUAL,
            reason="",
            using_blind_push=False,
        )
        args = SimpleNamespace(terminal_soft_enter_area_ratio=0.05)

        allowed, reason = _image_kf_takeover_allowed(
            image_kf,
            terminal_result,
            "los_innovation_reject",
            True,
            0.08,
            args,
            profile="strapdown",
        )

        self.assertTrue(allowed)
        self.assertEqual(reason, "terminal_state")

    def test_terminal_yaw_rate_scales_terminal_visual_and_extrapolates_blind_push(self):
        args = SimpleNamespace(
            terminal_yaw_rate_extrapolation=True,
            terminal_yaw_rate_average_window_s=0.10,
            terminal_yaw_rate_decay_tau_s=0.20,
            terminal_yaw_rate_scale=0.50,
            max_yaw_rate_deg=90.0,
        )
        samples: list[tuple[float, float]] = []
        for ts, value in [(0.00, 20.0), (0.04, 30.0), (0.08, 40.0)]:
            _append_yaw_rate_sample(samples, ts, value, True, args)

        command, base, count, decay = _terminal_yaw_rate_command(
            current_yaw_rate_deg_s=30.0,
            samples=samples,
            timestamp=0.10,
            terminal_state=TERMINAL_VISUAL,
            using_blind_push=False,
            blind_elapsed_s=0.0,
            args=args,
        )
        self.assertAlmostEqual(command, 15.0)
        self.assertAlmostEqual(base, 0.0)
        self.assertEqual(count, 0)
        self.assertAlmostEqual(decay, 1.0)

        command, base, count, decay = _terminal_yaw_rate_command(
            current_yaw_rate_deg_s=0.0,
            samples=samples,
            timestamp=0.10,
            terminal_state=BLIND_PUSH,
            using_blind_push=True,
            blind_elapsed_s=0.10,
            args=args,
        )
        self.assertAlmostEqual(base, 30.0)
        self.assertEqual(count, 3)
        self.assertGreater(decay, 0.0)
        self.assertLess(command, base)
        self.assertGreater(command, 0.0)

    def test_terminal_yaw_rate_extrapolation_can_be_disabled(self):
        args = SimpleNamespace(
            terminal_yaw_rate_extrapolation=False,
            terminal_yaw_rate_average_window_s=0.10,
            terminal_yaw_rate_decay_tau_s=0.20,
            terminal_yaw_rate_scale=0.50,
            max_yaw_rate_deg=90.0,
        )

        command, base, count, decay = _terminal_yaw_rate_command(
            current_yaw_rate_deg_s=0.0,
            samples=[(0.0, 40.0)],
            timestamp=0.10,
            terminal_state=BLIND_PUSH,
            using_blind_push=True,
            blind_elapsed_s=0.10,
            args=args,
        )

        self.assertAlmostEqual(command, 0.0)
        self.assertAlmostEqual(base, 0.0)
        self.assertEqual(count, 0)
        self.assertAlmostEqual(decay, 0.0)

    def test_terminal_yaw_hold_survives_frame_centering_loss_hold(self):
        args = SimpleNamespace(frame_centering=True)
        image_kf = TerminalImageEstimate(
            timestamp=1.0,
            theta_x=0.0,
            theta_y=0.0,
            theta_dot_x=0.0,
            theta_dot_y=0.0,
            valid=False,
            mode="invalid",
            age_s=0.0,
            quality=0.0,
        )
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)

        command, source, decay, sample_count = _frame_centering_yaw_rate(
            yaw_rate_deg_s=-20.0,
            px_err_x=0.0,
            intrinsics=intrinsics,
            state="loss_hold",
            timestamp=1.0,
            yaw_rate_samples=[],
            image_kf=image_kf,
            yaw_rate_source="terminal_yaw_hold",
            args=args,
        )

        self.assertAlmostEqual(command, -20.0)
        self.assertEqual(source, "terminal_yaw_hold")
        self.assertAlmostEqual(decay, 1.0)
        self.assertEqual(sample_count, 0)

    def test_terminal_trigger_strapdown_conditions(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        args = SimpleNamespace(terminal_bbox_area_ratio=0.25)

        self.assertEqual(_terminal_trigger_strapdown("bbox_clipped", True, 1.0, intrinsics, args), "bbox_clipped")
        self.assertEqual(_terminal_trigger_strapdown("bbox_top_clipped", True, 1.0, intrinsics, args), "bbox_top_clipped")
        self.assertEqual(_terminal_trigger_strapdown("", True, 90000.0, intrinsics, args), "bbox_area_large")
        self.assertEqual(_terminal_trigger_strapdown("", True, 1000.0, intrinsics, args), "")

    def test_bbox_noise_disabled_preserves_detection(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        detection = FrameDetection(1, 1.0, (100.0, 120.0, 180.0, 220.0), 1)
        args = SimpleNamespace(bbox_noise=False, bbox_center_noise_px=3.0, bbox_area_noise_ratio=0.08)

        noisy, info = _apply_bbox_noise(detection, intrinsics, np.random.default_rng(7), args)

        self.assertEqual(noisy.bbox_xyxy, detection.bbox_xyxy)
        self.assertEqual(info["raw_bbox"], detection.bbox_xyxy)
        self.assertEqual(info["noisy_bbox"], detection.bbox_xyxy)
        self.assertAlmostEqual(info["area_scale"], 1.0)

    def test_bbox_noise_is_reproducible_with_seed(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        detection = FrameDetection(1, 1.0, (100.0, 120.0, 180.0, 220.0), 1)
        args = SimpleNamespace(bbox_noise=True, bbox_center_noise_px=3.0, bbox_area_noise_ratio=0.08)

        noisy_a, info_a = _apply_bbox_noise(detection, intrinsics, np.random.default_rng(123), args)
        noisy_b, info_b = _apply_bbox_noise(detection, intrinsics, np.random.default_rng(123), args)

        self.assertEqual(noisy_a.bbox_xyxy, noisy_b.bbox_xyxy)
        self.assertAlmostEqual(info_a["dx"], info_b["dx"])
        self.assertAlmostEqual(info_a["dy"], info_b["dy"])
        self.assertAlmostEqual(info_a["area_scale"], info_b["area_scale"])

    def test_bbox_noise_preserves_aspect_ratio_when_unclipped(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        detection = FrameDetection(1, 1.0, (200.0, 180.0, 300.0, 230.0), 1)
        args = SimpleNamespace(bbox_noise=True, bbox_center_noise_px=0.0, bbox_area_noise_ratio=0.30)

        noisy, info = _apply_bbox_noise(detection, intrinsics, np.random.default_rng(42), args)

        self.assertAlmostEqual(noisy.width / noisy.height, detection.width / detection.height)
        self.assertAlmostEqual(noisy.area / detection.area, info["area_scale"])

    def test_bbox_noise_clamps_to_image_bounds(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        detection = FrameDetection(1, 1.0, (0.5, 0.5, 20.0, 30.0), 1)
        args = SimpleNamespace(bbox_noise=True, bbox_center_noise_px=20.0, bbox_area_noise_ratio=1.0)

        noisy, _ = _apply_bbox_noise(detection, intrinsics, np.random.default_rng(1), args)
        x1, y1, x2, y2 = noisy.bbox_xyxy

        self.assertGreaterEqual(x1, 0.0)
        self.assertGreaterEqual(y1, 0.0)
        self.assertLessEqual(x2, intrinsics.width)
        self.assertLessEqual(y2, intrinsics.height)
        self.assertGreater(x2, x1)
        self.assertGreater(y2, y1)

    def test_fixed_camera_pose_uses_positive_airsim_pitch_for_upward_mount(self):
        class FakeAirSim:
            @staticmethod
            def Vector3r(x, y, z):
                return (x, y, z)

            @staticmethod
            def to_quaternion(pitch, roll, yaw):
                return (pitch, roll, yaw)

            @staticmethod
            def Pose(position, orientation):
                return position, orientation

        args = SimpleNamespace(
            camera_x=0.0,
            camera_y=0.0,
            camera_z=-0.5,
            camera_pitch_deg=-15.0,
            camera_roll_deg=0.0,
            camera_yaw_deg=0.0,
        )

        _, orientation = _fixed_camera_pose(FakeAirSim, args)

        self.assertAlmostEqual(orientation[0], np.deg2rad(15.0))

    def test_fixed_camera_rotation_points_center_ray_upward_for_vertical_mount(self):
        args = SimpleNamespace(
            camera_pitch_deg=-90.0,
            camera_roll_deg=0.0,
            camera_yaw_deg=0.0,
        )

        forward_body = _fixed_camera_R_BC(args) @ np.array([0.0, 0.0, 1.0])

        np.testing.assert_allclose(forward_body, np.array([0.0, 0.0, -1.0]), atol=1.0e-9)

    def test_default_output_prefix_is_strapdown(self):
        args = SimpleNamespace(trajectory_prefix="", trajectory_dir="/tmp")

        csv_path, plot_path = _output_paths_strapdown(args)

        self.assertTrue(csv_path.name.startswith("strapdown_vision_png_"))
        self.assertEqual(csv_path.suffix, ".csv")
        self.assertEqual(plot_path.suffix, ".png")

    def test_plot_strapdown_does_not_require_gimbal_fields(self):
        rows = [
            {
                "t": 0.0,
                "range": 10.0,
                "pixel_error_x": 0.0,
                "pixel_error_y": 0.0,
                "yaw_rate_cmd_deg_s": 0.0,
                "body_yaw_deg": 0.0,
                "cmd_yaw_deg": 0.0,
                "target_body_bearing_deg": 0.0,
                "interceptor_x": 0.0,
                "interceptor_y": 0.0,
                "interceptor_z": -50.0,
                "intruder_x": 10.0,
                "intruder_y": 0.0,
                "intruder_z": -80.0,
            },
            {
                "t": 1.0,
                "range": 8.0,
                "pixel_error_x": 5.0,
                "pixel_error_y": -2.0,
                "yaw_rate_cmd_deg_s": 4.0,
                "body_yaw_deg": 2.0,
                "cmd_yaw_deg": 5.0,
                "target_body_bearing_deg": 3.0,
                "interceptor_x": 1.0,
                "interceptor_y": 0.0,
                "interceptor_z": -51.0,
                "intruder_x": 10.0,
                "intruder_y": 1.0,
                "intruder_z": -80.0,
            },
        ]
        with tempfile.TemporaryDirectory() as tmp:
            plot_path = Path(tmp) / "strapdown.png"

            ok = _plot_strapdown(rows, plot_path)

            self.assertTrue(ok)
            self.assertTrue(plot_path.exists())

    def test_plot_rows_stop_at_first_hit(self):
        rows = [{"t": 0.0, "hit": 0}, {"t": 1.0, "hit": 1}, {"t": 2.0, "hit": 0}]

        clipped = _rows_until_first_hit(rows)

        self.assertEqual([row["t"] for row in clipped], [0.0, 1.0])

    def test_start_geometry_offsets_from_horizontal_range(self):
        args = SimpleNamespace(
            start_horizontal_range_m=100.0,
            start_forward_offset_m=None,
            start_lateral_offset_m=-20.0,
        )

        forward, lateral, horizontal = _start_geometry_offsets(args)

        self.assertAlmostEqual(horizontal, 100.0)
        self.assertAlmostEqual(lateral, -20.0)
        self.assertAlmostEqual(forward, np.sqrt(100.0 * 100.0 - 20.0 * 20.0))

    def test_start_geometry_rejects_impossible_lateral_offset(self):
        args = SimpleNamespace(
            start_horizontal_range_m=10.0,
            start_forward_offset_m=None,
            start_lateral_offset_m=-20.0,
        )

        with self.assertRaises(SystemExit):
            _start_geometry_offsets(args)


if __name__ == "__main__":
    unittest.main()
