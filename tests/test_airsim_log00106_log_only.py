from __future__ import annotations

import math
import unittest
from pathlib import Path

import numpy as np

from vision_guidance.airsim_log00106_log_only import (
    AlgorithmExitStateMachine,
    CSV_FIELDS,
    DelayedVectorQueue,
    LowRateVelocityObserver,
    R_BC_UPWARD_FRD,
    REAL_INTRINSICS,
    ReplayTimingSample,
    ReplayTimingSchedule,
    ThrottleCalibrationTable,
    ThrottleHandover,
    airsim_flu_rates_to_frd,
    closest_point_confirmed,
    bbox_center_and_area,
    frd_rates_to_airsim_flu,
    load_log00106_replay_timing,
    measured_los_ned_from_bbox,
    project_los_to_real_pixel,
    remap_render_bbox_to_real_intrinsics,
    render_intrinsics,
    validate_csv_row,
)
from vision_guidance.flight_control import guidance_eval_to_setpoint
from vision_guidance.types import GuidanceEval


ROOT = Path(__file__).resolve().parents[1]
MAIN_CSV = ROOT / "logs/flight_active_supervised/FLIGHT_ACTIVE_05S_VIDEO_20260904_183721_20260904_183722.csv"


class AirSimLog00106LogOnlyTests(unittest.TestCase):
    def test_closest_point_requires_prior_closing(self) -> None:
        self.assertFalse(closest_point_confirmed([-0.2] * 8))
        self.assertFalse(closest_point_confirmed([0.4, -0.1, -0.1, -0.1, -0.1]))
        self.assertTrue(closest_point_confirmed([0.4, -0.1, -0.1, -0.1, -0.1, -0.1]))

    def test_upward_camera_axes(self) -> None:
        np.testing.assert_allclose(R_BC_UPWARD_FRD.T @ R_BC_UPWARD_FRD, np.eye(3))
        self.assertAlmostEqual(float(np.linalg.det(R_BC_UPWARD_FRD)), 1.0)
        np.testing.assert_allclose(R_BC_UPWARD_FRD @ np.array([0.0, 0.0, 1.0]), [0.0, 0.0, -1.0])

    def test_initial_los_projects_to_recorded_bbox_center(self) -> None:
        roll, pitch, yaw = np.deg2rad([0.2, -3.0, 333.0])
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        R_IB = np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ]
        )
        los = np.array([-0.030416, 0.066726, -0.997308])
        u, v = project_los_to_real_pixel(los, R_IB)
        self.assertLess(abs(u - 342.746), 1.0)
        self.assertLess(abs(v - 188.480), 1.0)
        box = (u - 24.0, v - 22.0, u + 24.0, v + 22.0)
        measured = measured_los_ned_from_bbox(box, R_IB)
        np.testing.assert_allclose(measured, los / np.linalg.norm(los), atol=1.0e-6)

    def test_render_bbox_remap_preserves_normalized_ray(self) -> None:
        source = render_intrinsics(640, 512, 62.16426133)
        source_box = (300.0, 190.0, 350.0, 240.0)
        target_box = remap_render_bbox_to_real_intrinsics(source_box, source)
        su, sv, _ = bbox_center_and_area(source_box, source)
        tu, tv, _ = bbox_center_and_area(target_box, REAL_INTRINSICS)
        self.assertAlmostEqual((su - source.cx) / source.fx, (tu - REAL_INTRINSICS.cx) / REAL_INTRINSICS.fx)
        self.assertAlmostEqual((sv - source.cy) / source.fy, (tv - REAL_INTRINSICS.cy) / REAL_INTRINSICS.fy)

    def test_frd_flu_rates_round_trip(self) -> None:
        rates = np.array([0.1, -0.2, 0.3])
        np.testing.assert_allclose(frd_rates_to_airsim_flu(rates), [0.1, 0.2, -0.3])
        np.testing.assert_allclose(airsim_flu_rates_to_frd(frd_rates_to_airsim_flu(rates)), rates)

    def test_physical_frd_pitch_rate_moves_toward_desired_pitch(self) -> None:
        pitch = math.radians(5.0)
        R_IB = np.array(
            [[math.cos(pitch), 0.0, math.sin(pitch)], [0.0, 1.0, 0.0], [-math.sin(pitch), 0.0, math.cos(pitch)]]
        )
        setpoint = guidance_eval_to_setpoint(
            GuidanceEval(0.0, np.array([1.0, 0.0, 0.0]), True, 1.0),
            R_IB=R_IB,
            rate_gain_matrix=np.zeros((3, 3)),
            hover_thrust=0.5,
            mapping_type="accel_tilt_rate",
            accel_tilt_rate={"pitch_rate_sign": 1.0},
        )
        self.assertLess(setpoint.desired_pitch_angle_deg, 0.0)
        self.assertLess(setpoint.pitch_rate_deg_s, 0.0)

    def test_replay_timing_is_paired_and_cycles(self) -> None:
        samples = (
            ReplayTimingSample(0, 0.0, 0.0, 0.08, 0.05),
            ReplayTimingSample(1, 0.03, 0.02, 0.04, 0.01),
        )
        schedule = ReplayTimingSchedule(samples)
        first = schedule.pop_due(0.031)
        self.assertEqual(len(first), 2)
        self.assertAlmostEqual(first[0].available_time_s - first[0].sample_time_s, 0.08)
        second_cycle = schedule.pop_due(schedule.period_s + 0.001)
        self.assertEqual(len(second_cycle), 1)
        self.assertTrue(second_cycle[0].extrapolated)

    def test_real_replay_has_40_samples(self) -> None:
        samples = load_log00106_replay_timing(MAIN_CSV)
        self.assertEqual(len(samples), 40)
        self.assertAlmostEqual(samples[0].measurement_age_s, 0.084583, places=6)
        self.assertAlmostEqual(samples[0].fusion_wait_s, 0.052851, places=6)

    def test_velocity_observer_updates_only_at_5_hz(self) -> None:
        observer = LowRateVelocityObserver([0.0, 0.0, 0.0], update_rate_hz=5.0, time_constant_s=0.25)
        before, updated = observer.update(0.19, [1.0, 0.0, 0.0])
        self.assertFalse(updated)
        np.testing.assert_allclose(before, [0.0, 0.0, 0.0])
        after, updated = observer.update(0.20, [1.0, 0.0, 0.0])
        self.assertTrue(updated)
        self.assertAlmostEqual(after[0], 1.0 - math.exp(-0.2 / 0.25))

    def test_delayed_queue_and_clear(self) -> None:
        queue = DelayedVectorQueue(0.015)
        queue.push(1.0, [1.0, 2.0, 3.0])
        np.testing.assert_allclose(queue.output(1.014), [0.0, 0.0, 0.0])
        np.testing.assert_allclose(queue.output(1.015), [1.0, 2.0, 3.0])
        queue.push(1.02, [4.0, 5.0, 6.0])
        queue.clear()
        np.testing.assert_allclose(queue.output(2.0), [0.0, 0.0, 0.0])

    def test_early_exit_requires_post_exit_observation(self) -> None:
        state = AlgorithmExitStateMachine(early_exit=True, stop_time_s=1.670804, post_exit_min_s=1.2)
        self.assertTrue(state.update(1.0).algorithm_active)
        event = state.update(1.7)
        self.assertTrue(event.exit_event)
        self.assertFalse(event.may_stop_run)
        self.assertTrue(state.update(2.9).may_stop_run)

    def test_throttle_handover_is_bounded_and_monotonic(self) -> None:
        handover = ThrottleHandover()
        outputs = [handover.update(t, 1400.0).output_us for t in np.linspace(0.0, 1.0, 51)]
        self.assertEqual(outputs[0], 1303.0)
        self.assertTrue(all(b >= a for a, b in zip(outputs, outputs[1:])))
        self.assertLessEqual(max(outputs), 1500.0)

    def test_throttle_calibration_interpolation(self) -> None:
        table = ThrottleCalibrationTable((0.4, 0.6, 0.8), (0.7, 1.0, 1.4), "test")
        command, limited = table.command_for_load(1.2)
        self.assertAlmostEqual(command, 0.7)
        self.assertFalse(limited)
        _, limited = table.command_for_load(2.0)
        self.assertTrue(limited)

    def test_csv_contract_and_static_log_only_safety(self) -> None:
        validate_csv_row({field: "" for field in CSV_FIELDS})
        runner = (ROOT / "examples/run_airsim_log00106_log_only.py").read_text(encoding="utf-8")
        forbidden = ("MSP_SET_RAW_RC", "--allow-control", "serial.Serial", "pyserial")
        for token in forbidden:
            self.assertNotIn(token, runner)


if __name__ == "__main__":
    unittest.main()
