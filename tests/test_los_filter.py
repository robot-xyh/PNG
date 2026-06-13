import numpy as np
import unittest

from vision_guidance.los_filter import LOSKalmanFilter6D


class LOSFilterTest(unittest.TestCase):
    def test_static_los_keeps_low_rate(self):
        kf = LOSKalmanFilter6D()
        for i in range(20):
            est = kf.update(i * 0.02, np.array([0.0, 0.0, 1.0]))
        self.assertTrue(est.valid)
        self.assertLess(np.linalg.norm(est.lambda_dot_I), 1e-3)
        self.assertLess(abs(np.linalg.norm(est.lambda_I) - 1.0), 1e-9)

    def test_rejects_large_innovation(self):
        kf = LOSKalmanFilter6D()
        kf.update(0.0, np.array([0.0, 0.0, 1.0]))
        est = kf.update(0.02, np.array([1.0, 0.0, 0.0]))
        self.assertFalse(est.valid)
        self.assertEqual(est.reject_reason, "los_innovation_reject")


if __name__ == "__main__":
    unittest.main()
