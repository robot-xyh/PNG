import unittest

from vision_guidance.bytetrack_adapter import ByteTrackAdapter, ByteTrackConfig
from vision_guidance.rknn_native_detector import RknnDetection


def _detection(x1=10.0, score=0.8):
    return RknnDetection(
        bbox_xyxy=(x1, 10.0, x1 + 20.0, 30.0),
        score=score,
        class_id=0,
        candidate_index=0,
    )


class ByteTrackAdapterTest(unittest.TestCase):
    def make_tracker(self, **overrides):
        values = {
            "minimum_confirmed_frames": 3,
            "frame_rate": 5.0,
            "track_buffer_s": 0.5,
            "final_min_score": 0.25,
        }
        values.update(overrides)
        return ByteTrackAdapter(ByteTrackConfig(**values))

    def test_locks_after_three_measurements(self):
        tracker = self.make_tracker()

        first = tracker.update([_detection(10.0)], timestamp=0.0)
        second = tracker.update([_detection(11.0)], timestamp=0.2)
        third = tracker.update([_detection(12.0)], timestamp=0.4)

        self.assertIsNone(first.selected)
        self.assertIsNone(second.selected)
        self.assertIsNotNone(third.selected)
        self.assertEqual(third.selected.track_id, 1)
        self.assertEqual(third.stats["target_selector_reason"], "new_track_locked")
        self.assertEqual(third.stats["tracker_hits"], 3)

    def test_lost_track_emits_no_measurement_and_recovers_same_id(self):
        tracker = self.make_tracker()
        for frame in range(3):
            locked = tracker.update([_detection(10.0 + frame)], timestamp=frame * 0.2)
        track_id = locked.selected.track_id

        lost = tracker.update([], timestamp=0.6)
        recovered = tracker.update([_detection(14.0)], timestamp=0.8)

        self.assertIsNone(lost.selected)
        self.assertEqual(lost.stats["tracker_state"], "lost")
        self.assertEqual(lost.stats["bbox_measurement_source"], "none")
        self.assertEqual(recovered.selected.track_id, track_id)
        self.assertEqual(recovered.stats["tracker_fragment_count"], 1)

    def test_low_score_detection_preserves_track_but_does_not_update_guidance(self):
        tracker = self.make_tracker(fuse_score=False)
        for frame in range(3):
            locked = tracker.update([_detection(10.0 + frame)], timestamp=frame * 0.2)
        track_id = locked.selected.track_id

        low = tracker.update([_detection(13.0, score=0.15)], timestamp=0.6)
        high = tracker.update([_detection(14.0, score=0.8)], timestamp=0.8)

        self.assertIsNone(low.selected)
        self.assertEqual(low.stats["target_selector_reason"], "track_score_below_final")
        self.assertEqual(low.stats["tracker_low_count"], 1)
        self.assertEqual(low.stats["tracker_association_stage"], "low")
        self.assertGreater(low.stats["tracker_match_iou"], 0.0)
        self.assertEqual(low.stats["tracker_confirmed"], 1)
        self.assertEqual(low.stats["bbox_measurement_source"], "none")
        self.assertEqual(high.selected.track_id, track_id)

    def test_low_match_threshold_can_retain_fast_low_score_detection(self):
        baseline = self.make_tracker(
            fuse_score=False,
            track_low_thresh=0.05,
            final_min_score=0.05,
            low_match_thresh=0.5,
        )
        permissive = self.make_tracker(
            fuse_score=False,
            track_low_thresh=0.05,
            final_min_score=0.05,
            low_match_thresh=0.8,
        )
        for frame in range(3):
            baseline.update([_detection(10.0 + frame)], timestamp=frame * 0.2)
            permissive_locked = permissive.update([_detection(10.0 + frame)], timestamp=frame * 0.2)

        baseline_low = baseline.update([_detection(24.0, score=0.08)], timestamp=0.6)
        permissive_low = permissive.update([_detection(24.0, score=0.08)], timestamp=0.6)

        self.assertIsNone(baseline_low.selected)
        self.assertEqual(baseline_low.stats["tracker_state"], "lost")
        self.assertIsNotNone(permissive_low.selected)
        self.assertEqual(permissive_low.selected.track_id, permissive_locked.selected.track_id)
        self.assertEqual(permissive_low.stats["tracker_association_stage"], "low")

    def test_does_not_switch_to_other_track_while_active_track_is_lost(self):
        tracker = self.make_tracker(track_buffer_s=1.0)
        for frame in range(3):
            locked = tracker.update([_detection(10.0 + frame)], timestamp=frame * 0.2)
        active_id = locked.selected.track_id

        for frame in range(3, 6):
            update = tracker.update([_detection(300.0)], timestamp=frame * 0.2)

        self.assertIsNone(update.selected)
        self.assertEqual(tracker.active_track_id, active_id)
        self.assertEqual(update.stats["tracker_switch_count"], 0)

    def test_rejects_invalid_threshold_order(self):
        with self.assertRaisesRegex(ValueError, "low < high"):
            ByteTrackAdapter(ByteTrackConfig(track_low_thresh=0.3, track_high_thresh=0.25))

        with self.assertRaisesRegex(ValueError, "low_match_thresh"):
            ByteTrackAdapter(ByteTrackConfig(low_match_thresh=0.0))


if __name__ == "__main__":
    unittest.main()
