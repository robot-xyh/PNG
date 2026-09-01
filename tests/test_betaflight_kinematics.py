import math
import unittest

from vision_guidance.betaflight_kinematics import (
    BetaflightKinematicEstimator,
    KinematicEstimatorConfig,
)
from vision_guidance.betaflight_msp import (
    AltitudeTelemetry,
    BetaflightTelemetry,
    RawGpsTelemetry,
)


def _telemetry(
    timestamp: float,
    *,
    latitude: float = 37.0,
    longitude: float = -122.0,
    speed: float = 10.0,
    course: float = 90.0,
    altitude: float = 100.0,
    vario: float = 2.0,
    fix: int = 1,
    satellites: int = 10,
) -> BetaflightTelemetry:
    return BetaflightTelemetry(
        timestamp=timestamp,
        raw_gps=RawGpsTelemetry(
            fix,
            satellites,
            latitude,
            longitude,
            50.0,
            speed,
            course,
            80,
        ),
        altitude=AltitudeTelemetry(altitude, vario),
        raw_gps_timestamp_s=timestamp,
        altitude_timestamp_s=timestamp,
    )


class BetaflightKinematicsTest(unittest.TestCase):
    def test_locks_stable_origin_and_converts_position_velocity_to_ned(self):
        estimator = BetaflightKinematicEstimator(
            KinematicEstimatorConfig(origin_lock_samples=2, velocity_filter_tau_s=0.0)
        )
        self.assertEqual(estimator.update(_telemetry(1.0), 1.0).reason, "origin_pending")
        state = estimator.update(
            _telemetry(1.2, latitude=37.00001, longitude=-121.99999, altitude=101.0),
            1.2,
        )

        self.assertTrue(state.valid)
        self.assertTrue(state.origin_locked)
        self.assertGreater(state.position_ned_m[0], 0.0)
        self.assertGreater(state.position_ned_m[1], 0.0)
        self.assertAlmostEqual(state.position_ned_m[2], -1.0)
        self.assertAlmostEqual(state.velocity_ned_raw_m_s[0], 0.0, places=6)
        self.assertAlmostEqual(state.velocity_ned_raw_m_s[1], 10.0, places=6)
        self.assertAlmostEqual(state.velocity_ned_raw_m_s[2], -2.0)

    def test_velocity_filter_is_first_order(self):
        estimator = BetaflightKinematicEstimator(
            KinematicEstimatorConfig(origin_lock_samples=1, velocity_filter_tau_s=0.25)
        )
        first = estimator.update(_telemetry(1.0, speed=0.0), 1.0)
        second = estimator.update(_telemetry(1.25, speed=10.0), 1.25)

        self.assertTrue(first.valid)
        self.assertAlmostEqual(
            second.velocity_ned_filtered_m_s[1],
            10.0 * (1.0 - math.exp(-1.0)),
            places=6,
        )

    def test_fails_closed_for_fix_loss_and_stale_axes(self):
        estimator = BetaflightKinematicEstimator(
            KinematicEstimatorConfig(origin_lock_samples=1, gps_timeout_s=0.5, altitude_timeout_s=0.3)
        )
        self.assertTrue(estimator.update(_telemetry(1.0), 1.0).valid)

        altitude_stale = estimator.update(_telemetry(1.0), 1.4)
        self.assertFalse(altitude_stale.valid)
        self.assertEqual(altitude_stale.reason, "altitude_stale")

        no_fix = estimator.update(_telemetry(2.0, fix=0), 2.0)
        self.assertFalse(no_fix.valid)
        self.assertEqual(no_fix.reason, "gps_fix_invalid")

    def test_rejects_nonfinite_gps_state(self):
        estimator = BetaflightKinematicEstimator(
            KinematicEstimatorConfig(origin_lock_samples=1)
        )
        state = estimator.update(_telemetry(1.0, latitude=float("nan")), 1.0)

        self.assertFalse(state.valid)
        self.assertEqual(state.reason, "gps_fix_invalid")


if __name__ == "__main__":
    unittest.main()
