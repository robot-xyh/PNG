import unittest

import numpy as np

from vision_guidance.terminal_image_kf import (
    IMAGE_KF_INVALID,
    IMAGE_KF_PREDICT,
    IMAGE_KF_UPDATE,
    TerminalImageKF,
    TerminalImageKFConfig,
    angle_error_from_center,
    center_from_angle_error,
)
from vision_guidance.types import CameraIntrinsics


class TerminalImageKFTest(unittest.TestCase):
    def test_angle_center_roundtrip(self):
        intrinsics = CameraIntrinsics(320.0, 300.0, 320.0, 240.0, 640, 480)
        center = (430.0, 180.0)

        theta = angle_error_from_center(center, intrinsics)
        reconstructed = center_from_angle_error(theta, intrinsics)

        self.assertAlmostEqual(reconstructed[0], center[0])
        self.assertAlmostEqual(reconstructed[1], center[1])

    def test_constant_velocity_prediction_after_measurement_loss(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        kf = TerminalImageKF(
            TerminalImageKFConfig(
                measurement_noise_rad=0.002,
                accel_noise_rad_s2=4.0,
                max_predict_s=0.30,
                innovation_reject_rad=0.5,
            )
        )
        estimate = None
        for i in range(12):
            ts = i * 0.05
            theta = np.array([0.02 + 0.30 * ts, -0.01 + 0.10 * ts])
            estimate = kf.update(
                timestamp=ts,
                center=center_from_angle_error(theta, intrinsics),
                intrinsics=intrinsics,
                detected=True,
                measurement_valid=True,
                clipped=False,
                track_id=1,
            )

        self.assertIsNotNone(estimate)
        self.assertEqual(estimate.mode, IMAGE_KF_UPDATE)
        self.assertGreater(estimate.theta_dot_x, 0.10)
        self.assertGreater(estimate.theta_dot_y, 0.03)

        predicted = kf.update(
            timestamp=0.65,
            center=None,
            intrinsics=intrinsics,
            detected=False,
            measurement_valid=False,
            clipped=False,
            track_id=1,
        )

        self.assertTrue(predicted.valid)
        self.assertEqual(predicted.mode, IMAGE_KF_PREDICT)
        self.assertGreater(predicted.theta_x, estimate.theta_x)
        self.assertGreater(predicted.theta_y, estimate.theta_y)

    def test_clipped_bbox_predicts_without_update(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        kf = TerminalImageKF(TerminalImageKFConfig(max_predict_s=0.30))
        kf.update(
            timestamp=0.0,
            center=(360.0, 250.0),
            intrinsics=intrinsics,
            detected=True,
            measurement_valid=True,
            clipped=False,
            track_id=1,
        )

        predicted = kf.update(
            timestamp=0.05,
            center=(640.0, 250.0),
            intrinsics=intrinsics,
            detected=True,
            measurement_valid=True,
            clipped=True,
            track_id=1,
        )

        self.assertTrue(predicted.valid)
        self.assertEqual(predicted.mode, IMAGE_KF_PREDICT)
        self.assertEqual(predicted.reject_reason, "bbox_clipped")

    def test_prediction_times_out(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        kf = TerminalImageKF(TerminalImageKFConfig(max_predict_s=0.10))
        kf.update(
            timestamp=0.0,
            center=(360.0, 250.0),
            intrinsics=intrinsics,
            detected=True,
            measurement_valid=True,
            clipped=False,
            track_id=1,
        )

        estimate = kf.update(
            timestamp=0.20,
            center=None,
            intrinsics=intrinsics,
            detected=False,
            measurement_valid=False,
            clipped=False,
            track_id=1,
        )

        self.assertFalse(estimate.valid)
        self.assertEqual(estimate.mode, IMAGE_KF_INVALID)
        self.assertEqual(estimate.reject_reason, "image_kf_predict_timeout")

    def test_large_innovation_rejects_and_resets(self):
        intrinsics = CameraIntrinsics(320.0, 320.0, 320.0, 240.0, 640, 480)
        kf = TerminalImageKF(TerminalImageKFConfig(innovation_reject_rad=0.05))
        kf.update(
            timestamp=0.0,
            center=(320.0, 240.0),
            intrinsics=intrinsics,
            detected=True,
            measurement_valid=True,
            clipped=False,
            track_id=1,
        )

        estimate = kf.update(
            timestamp=0.05,
            center=center_from_angle_error(np.array([0.4, 0.0]), intrinsics),
            intrinsics=intrinsics,
            detected=True,
            measurement_valid=True,
            clipped=False,
            track_id=1,
        )

        self.assertFalse(estimate.valid)
        self.assertEqual(estimate.reject_reason, "image_kf_innovation_reject")


if __name__ == "__main__":
    unittest.main()
