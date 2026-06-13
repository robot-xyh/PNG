import numpy as np
import unittest

from vision_guidance.attitude_buffer import AttitudeHistoryBuffer
from vision_guidance.types import AttitudeSample


class AttitudeBufferTest(unittest.TestCase):
    def test_lookup_inside_buffer(self):
        buf = AttitudeHistoryBuffer(duration_s=1.0)
        buf.push(AttitudeSample(0.0, np.eye(3)))
        buf.push(AttitudeSample(0.1, np.eye(3)))
        got = buf.lookup(0.05)
        self.assertTrue(got.valid)
        self.assertIsNotNone(got.sample)
        self.assertTrue(np.allclose(got.sample.R_IB.T @ got.sample.R_IB, np.eye(3), atol=1e-8))

    def test_lookup_rejects_future(self):
        buf = AttitudeHistoryBuffer(duration_s=1.0)
        buf.push(AttitudeSample(0.0, np.eye(3)))
        got = buf.lookup(0.2)
        self.assertFalse(got.valid)
        self.assertEqual(got.reason, "timestamp_after_buffer")


if __name__ == "__main__":
    unittest.main()
