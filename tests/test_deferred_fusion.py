import unittest

import numpy as np

from vision_guidance.attitude_buffer import AttitudeHistoryBuffer
from vision_guidance.fusion import DeferredAttitudeFusion
from vision_guidance.types import AttitudeSample, FrameDetection


class _Pipeline:
    def __init__(self):
        self.attitude_buffer = AttitudeHistoryBuffer(duration_s=2.0)
        self.calls = []

    def process(self, detection):
        self.calls.append(detection)
        return detection


def _detection(frame_id, timestamp):
    return FrameDetection(
        frame_id=frame_id,
        exposure_ts=timestamp,
        bbox_xyxy=(10.0, 10.0, 20.0, 20.0),
        track_id=3,
        score=0.9,
    )


class DeferredAttitudeFusionTests(unittest.TestCase):
    def test_waits_until_attitude_brackets_detection(self):
        pipeline = _Pipeline()
        pipeline.attitude_buffer.push(AttitudeSample(1.0, np.eye(3)))
        fusion = DeferredAttitudeFusion(pipeline, max_wait_s=0.2)
        detection = _detection(1, 1.05)

        waiting = fusion.update(detection, timestamp=1.06, context={"seq": 1})
        self.assertEqual(waiting.status, "waiting_for_attitude")
        self.assertEqual(waiting.pending_count, 1)
        self.assertIsNone(waiting.result)

        pipeline.attitude_buffer.push(AttitudeSample(1.10, np.eye(3)))
        processed = fusion.update(None, timestamp=1.11, perception_new_result=False)
        self.assertEqual(processed.status, "processed")
        self.assertEqual(processed.pending_count, 0)
        self.assertIs(processed.detection, detection)
        self.assertEqual(processed.context, {"seq": 1})
        self.assertEqual(pipeline.calls, [detection])

    def test_real_no_detection_clears_pending_frame(self):
        pipeline = _Pipeline()
        pipeline.attitude_buffer.push(AttitudeSample(2.0, np.eye(3)))
        fusion = DeferredAttitudeFusion(pipeline)
        fusion.update(_detection(1, 2.05), timestamp=2.06)

        cleared = fusion.update(None, timestamp=2.07, perception_new_result=True)
        self.assertEqual(cleared.status, "no_detection")
        self.assertEqual(cleared.pending_count, 0)
        self.assertEqual(cleared.dropped_count, 1)
        self.assertEqual(pipeline.calls, [])

    def test_timeout_processes_frame_through_fail_closed_pipeline(self):
        pipeline = _Pipeline()
        pipeline.attitude_buffer.push(AttitudeSample(3.0, np.eye(3)))
        fusion = DeferredAttitudeFusion(pipeline, max_wait_s=0.1)
        detection = _detection(1, 3.05)
        fusion.update(detection, timestamp=3.06)

        timed_out = fusion.update(None, timestamp=3.17, perception_new_result=False)
        self.assertEqual(timed_out.status, "attitude_wait_timeout")
        self.assertEqual(timed_out.pending_count, 0)
        self.assertEqual(pipeline.calls, [detection])

    def test_buffer_exposes_timestamp_bounds(self):
        buffer = AttitudeHistoryBuffer(duration_s=2.0)
        self.assertIsNone(buffer.oldest_timestamp)
        self.assertIsNone(buffer.latest_timestamp)
        buffer.push(AttitudeSample(4.0, np.eye(3)))
        buffer.push(AttitudeSample(4.1, np.eye(3)))
        self.assertEqual(buffer.oldest_timestamp, 4.0)
        self.assertEqual(buffer.latest_timestamp, 4.1)


if __name__ == "__main__":
    unittest.main()
