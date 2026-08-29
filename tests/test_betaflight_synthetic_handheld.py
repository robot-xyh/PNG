import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from tools.generate_betaflight_handheld_sequence import _compose, _motion
from tools.run_rknn_bytetrack_video_eval import _summary


class SyntheticHandheldGeneratorTests(unittest.TestCase):
    def setUp(self):
        self.args = SimpleNamespace(width=640, height=512, horizontal_amplitude_px=190.0)

    def test_motion_has_static_ends_and_bidirectional_horizontal_leg(self):
        start = _motion(0.0, self.args)
        right = _motion(5.5, self.args)
        left = _motion(10.5, self.args)
        end = _motion(29.0, self.args)

        self.assertEqual(start[3], "static_start")
        self.assertEqual(end[3], "static_end")
        self.assertAlmostEqual(start[0], 320.0)
        self.assertAlmostEqual(end[0], 320.0)
        self.assertGreater(right[0], 320.0)
        self.assertLess(left[0], 320.0)

    def test_compose_alpha_blends_inside_reported_bbox(self):
        background = np.full((40, 60, 3), 100, dtype=np.uint8)
        foreground = np.zeros((10, 20, 4), dtype=np.uint8)
        foreground[:, :, :3] = (200, 0, 0)
        foreground[:, :, 3] = 255

        frame, bbox = _compose(background, foreground, center_x=30.0, center_y=20.0, width_px=20.0)

        self.assertEqual(bbox, (20, 15, 40, 25))
        self.assertTrue(np.array_equal(frame[15:25, 20:40, 0], np.full((10, 20), 200)))
        self.assertTrue(np.array_equal(frame[0, 0], np.array([100, 100, 100])))

    def test_compose_scales_foreground_opacity(self):
        background = np.full((40, 60, 3), 100, dtype=np.uint8)
        foreground = np.zeros((10, 20, 4), dtype=np.uint8)
        foreground[:, :, :3] = (200, 0, 0)
        foreground[:, :, 3] = 255

        frame, _ = _compose(
            background,
            foreground,
            center_x=30.0,
            center_y=20.0,
            width_px=20.0,
            foreground_opacity=0.25,
        )

        self.assertTrue(np.array_equal(frame[15:25, 20:40, 0], np.full((10, 20), 125)))
        self.assertTrue(np.array_equal(frame[15:25, 20:40, 1], np.full((10, 20), 75)))


class RknnVideoSummaryTests(unittest.TestCase):
    def test_summary_reports_continuous_track_and_truth_correlation(self):
        rows = []
        for frame_id, center_x in enumerate((100.0, 120.0, 140.0), start=1):
            rows.append(
                {
                    "detection_valid": 1,
                    "detector_class_filtered_count": 1,
                    "track_id": 7,
                    "score": 0.8,
                    "rknn_inference_ms": 5.0,
                    "truth_center_x": center_x,
                    "x1": center_x - 10.0,
                    "x2": center_x + 10.0,
                    "tracker_switch_count": 0,
                    "tracker_fragment_count": 0,
                }
            )

        result = _summary(rows, fps=20.0, metadata={"backend": "test"}, video=Path(__file__))

        self.assertEqual(result["frames"], 3)
        self.assertEqual(result["track_ids"], [7])
        self.assertEqual(result["selected_rate"], 1.0)
        self.assertAlmostEqual(result["truth_center_x_correlation"], 1.0)


if __name__ == "__main__":
    unittest.main()
