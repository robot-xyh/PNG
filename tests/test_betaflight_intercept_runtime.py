import unittest

import numpy as np

from vision_guidance.betaflight_intercept_controller import (
    InterceptPhase,
    VelocityEstablishingPngConfig,
    VelocityEstablishingPngController,
)
from vision_guidance.betaflight_intercept_runtime import VelocityEstablishingPngRuntime
from vision_guidance.betaflight_kinematics import VehicleKinematicState
from vision_guidance.fusion import VisionGuidanceResult
from vision_guidance.types import FrameDetection, GuidanceEval, LOSEstimate


def _kinematics(*, valid=True, velocity=(0.0, 0.0, 0.0), age=0.01):
    return VehicleKinematicState(
        timestamp_s=1.0,
        valid=valid,
        reason="valid" if valid else "gps_missing",
        source="msp_raw_gps+msp_altitude",
        horizontal_valid=valid,
        vertical_valid=valid,
        position_ned_m=(0.0, 0.0, 0.0),
        velocity_ned_raw_m_s=velocity,
        velocity_ned_filtered_m_s=velocity,
        latitude_deg=0.0,
        longitude_deg=0.0,
        gps_altitude_m=0.0,
        baro_altitude_m=0.0,
        ground_speed_m_s=0.0,
        ground_course_deg=0.0,
        vertical_speed_up_m_s=0.0,
        fix=2,
        satellites=8,
        hdop=100,
        gps_age_s=age,
        altitude_age_s=age,
        origin_locked=valid,
        origin_latitude_deg=0.0,
        origin_longitude_deg=0.0,
        origin_baro_altitude_m=0.0,
    )


def _vision(timestamp=1.0, *, track_id=7, valid=True, reason=None):
    detection = FrameDetection(int(timestamp * 1000), timestamp, (300, 220, 340, 260), track_id, 0.9)
    los = (
        LOSEstimate(
            timestamp=timestamp,
            lambda_I=np.array([0.0, 0.0, -1.0]),
            lambda_dot_I=np.array([0.0, 0.1, 0.0]),
            omega_los=np.array([0.1, 0.0, 0.0]),
            innovation_norm=0.0,
            quality=0.9,
            valid=True,
        )
        if valid
        else None
    )
    return VisionGuidanceResult(
        detection=detection,
        los=los,
        ttc=None,
        guidance=GuidanceEval(
            timestamp,
            np.zeros(3),
            valid,
            0.9 if valid else 0.0,
            reason,
        ),
        R_IB=np.eye(3) if valid else None,
    )


class VelocityEstablishingPngRuntimeTest(unittest.TestCase):
    def _runtime(self, source="bench_zero_velocity"):
        return VelocityEstablishingPngRuntime(
            VelocityEstablishingPngController(
                VelocityEstablishingPngConfig(
                    fixed_vm_m_s=10.0,
                    acquire_consecutive_frames=1,
                    detection_timeout_s=0.35,
                    velocity_timeout_s=0.5,
                )
            ),
            velocity_source=source,
            image_width=640,
            image_height=512,
        )

    def test_holds_last_los_between_perception_results_then_aborts_stale(self):
        runtime = self._runtime()
        first = runtime.update(
            timestamp_s=1.0,
            vision_result=_vision(1.0),
            attitude_R_IB=np.eye(3),
            attitude_valid=True,
            kinematics=_kinematics(valid=False),
        )
        held = runtime.update(
            timestamp_s=1.2,
            vision_result=None,
            attitude_R_IB=np.eye(3),
            attitude_valid=True,
            kinematics=_kinematics(valid=False),
        )
        stale = runtime.update(
            timestamp_s=1.36,
            vision_result=None,
            attitude_R_IB=np.eye(3),
            attitude_valid=True,
            kinematics=_kinematics(valid=False),
        )
        recovered = runtime.update(
            timestamp_s=1.4,
            vision_result=_vision(1.4),
            attitude_R_IB=np.eye(3),
            attitude_valid=True,
            kinematics=_kinematics(valid=False),
        )

        self.assertTrue(first.result.guidance.valid)
        self.assertTrue(held.result.guidance.valid)
        self.assertAlmostEqual(held.controller.detection_age_s, 0.2)
        self.assertEqual(stale.controller.phase, InterceptPhase.ABORT)
        self.assertEqual(stale.controller.reason, "detection_stale")
        self.assertFalse(recovered.result.guidance.valid)
        self.assertEqual(recovered.controller.reason, "detection_stale")

    def test_track_change_reason_aborts_immediately(self):
        runtime = self._runtime()
        runtime.update(
            timestamp_s=1.0,
            vision_result=_vision(1.0),
            attitude_R_IB=np.eye(3),
            attitude_valid=True,
            kinematics=_kinematics(valid=False),
        )
        changed = runtime.update(
            timestamp_s=1.1,
            vision_result=_vision(1.1, track_id=8, valid=False, reason="track_id_changed"),
            attitude_R_IB=np.eye(3),
            attitude_valid=True,
            kinematics=_kinematics(valid=False),
        )

        self.assertEqual(changed.controller.phase, InterceptPhase.ABORT)
        self.assertEqual(changed.controller.reason, "track_id_changed")

    def test_msp_kinematics_uses_filtered_velocity_and_oldest_sample_age(self):
        runtime = self._runtime("msp_kinematics")
        output = runtime.update(
            timestamp_s=2.0,
            vision_result=_vision(2.0),
            attitude_R_IB=np.eye(3),
            attitude_valid=True,
            kinematics=_kinematics(velocity=(3.0, 0.0, 0.0), age=0.1),
        )

        self.assertTrue(output.result.guidance.valid)
        self.assertAlmostEqual(output.controller.velocity_age_s, 0.1)
        self.assertEqual(output.velocity_reason, "msp_kinematics")

    def test_invalid_msp_kinematics_fails_closed(self):
        runtime = self._runtime("msp_kinematics")
        output = runtime.update(
            timestamp_s=1.0,
            vision_result=_vision(1.0),
            attitude_R_IB=np.eye(3),
            attitude_valid=True,
            kinematics=_kinematics(valid=False),
        )

        self.assertFalse(output.result.guidance.valid)
        self.assertEqual(output.controller.reason, "velocity_invalid")
        self.assertEqual(output.velocity_reason, "gps_missing")


if __name__ == "__main__":
    unittest.main()
