import unittest

from vision_guidance.ttc import ScaleExpansionTTC
from vision_guidance.types import FrameDetection


class TTCTest(unittest.TestCase):
    def test_ttc_becomes_valid_for_expanding_area(self):
        ttc = ScaleExpansionTTC()
        state = None
        for i in range(1, 8):
            size = 10 + i
            det = FrameDetection(i, i * 0.1, (100, 100, 100 + size, 100 + size), 1)
            state = ttc.update(det, 640, 480)
        self.assertIsNotNone(state)
        self.assertTrue(state.valid)
        self.assertIsNotNone(state.ttc)

    def test_ttc_rejects_clipped_bbox(self):
        ttc = ScaleExpansionTTC()
        det = FrameDetection(1, 0.1, (0, 100, 20, 120), 1)
        state = ttc.update(det, 640, 480)
        self.assertFalse(state.valid)
        self.assertEqual(state.reject_reason, "bbox_left_clipped")

    def test_ttc_reports_top_clipped_bbox(self):
        ttc = ScaleExpansionTTC()
        det = FrameDetection(1, 0.1, (100, 0, 140, 40), 1)
        state = ttc.update(det, 640, 480)
        self.assertFalse(state.valid)
        self.assertEqual(state.reject_reason, "bbox_top_clipped")


if __name__ == "__main__":
    unittest.main()
