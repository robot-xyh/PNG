from __future__ import annotations

import unittest

import numpy as np

from vision_guidance.camera_latency_probe import (
    BrightnessSample,
    DisplayTransition,
    LatencyMatch,
    central_roi,
    detect_brightness_edges,
    match_display_to_camera_edges,
    ntp_clock_offset_and_delay_s,
    roi_mean_luma,
    summarize_latency,
    validate_roi,
)


class CameraLatencyProbeTests(unittest.TestCase):
    def test_ntp_exchange_recovers_server_minus_client_offset(self) -> None:
        offset_s, delay_s = ntp_clock_offset_and_delay_s(
            100.000,
            100.035,
            100.036,
            100.011,
        )
        self.assertAlmostEqual(offset_s, 0.030)
        self.assertAlmostEqual(delay_s, 0.010)

        with self.assertRaisesRegex(ValueError, "receive time precedes"):
            ntp_clock_offset_and_delay_s(2.0, 2.0, 2.1, 1.0)

    def test_roi_helpers_validate_bounds_and_compute_luma(self) -> None:
        self.assertEqual(central_roi(100, 80, 0.5), (25, 20, 50, 40))
        self.assertEqual(validate_roi((1, 2, 3, 4), width=10, height=10), (1, 2, 3, 4))
        with self.assertRaises(ValueError):
            validate_roi((8, 2, 3, 4), width=10, height=10)

        image = np.zeros((4, 4, 3), dtype=np.uint8)
        image[1:3, 1:3] = (10, 20, 30)
        expected = 0.114 * 10 + 0.587 * 20 + 0.299 * 30
        self.assertAlmostEqual(roi_mean_luma(image, (1, 1, 2, 2)), expected)

    def test_detects_debounced_noisy_edges_at_first_new_frame(self) -> None:
        samples = self._samples((0.20, 0.60, 1.00))
        edges, levels = detect_brightness_edges(samples, debounce_frames=2)

        self.assertEqual([edge.state for edge in edges], [1, 0, 1])
        self.assertEqual([edge.frame_id for edge in edges], [21, 61, 101])
        self.assertGreater(levels["contrast"], 190.0)

    def test_rejects_low_contrast_signal(self) -> None:
        samples = [
            BrightnessSample(index, index * 0.01, 1000.0 + index * 0.01, 100.0 + index % 2)
            for index in range(1, 21)
        ]
        with self.assertRaisesRegex(ValueError, "contrast is too low"):
            detect_brightness_edges(samples, minimum_contrast=10.0)

    def test_clock_offset_is_removed_from_latency(self) -> None:
        samples = self._samples((0.20, 0.60, 1.00), camera_clock_offset_s=0.012)
        edges, _ = detect_brightness_edges(samples, debounce_frames=2)
        transitions = self._transitions((0.20, 0.60, 1.00), latency_s=0.04)

        matches, unmatched = match_display_to_camera_edges(
            transitions,
            edges,
            camera_minus_display_clock_s=0.012,
            minimum_latency_s=0.0,
            maximum_latency_s=0.20,
        )

        self.assertEqual(unmatched, [])
        self.assertEqual(len(matches), 3)
        for match in matches:
            self.assertAlmostEqual(match.latency_ms, 40.0, places=5)

    def test_missing_camera_edge_remains_unmatched(self) -> None:
        samples = self._samples((0.20, 0.60, 1.00))
        edges, _ = detect_brightness_edges(samples, debounce_frames=2)
        edges = [edge for edge in edges if edge.frame_id != 61]
        transitions = self._transitions((0.20, 0.60, 1.00), latency_s=0.04)

        matches, unmatched = match_display_to_camera_edges(
            transitions,
            edges,
            minimum_latency_s=0.0,
            maximum_latency_s=0.20,
        )

        self.assertEqual([match.display_sequence for match in matches], [1, 3])
        self.assertEqual(unmatched, [2])

    def test_summary_reports_percentiles_and_equivalent_frames(self) -> None:
        samples = [
            BrightnessSample(index, index * 0.01, 1000.0 + index * 0.01, 20.0)
            for index in range(1, 101)
        ]
        latencies = (10.0, 20.0, 30.0, 40.0)
        matches = [
            LatencyMatch(index, index % 2, 1000.0 + index, index, 1000.0 + index, latency)
            for index, latency in enumerate(latencies, start=1)
        ]

        summary = summarize_latency(matches, camera_samples=samples, transition_count=4)

        self.assertEqual(summary["matched_ratio"], 1.0)
        self.assertAlmostEqual(summary["camera_frame_period_ms"]["p50"], 10.0)
        self.assertAlmostEqual(summary["latency_ms"]["p50"], 25.0)
        self.assertAlmostEqual(summary["equivalent_buffer_frames"]["p50"], 2.5)

    @staticmethod
    def _samples(
        edge_times: tuple[float, ...],
        *,
        camera_clock_offset_s: float = 0.0,
    ) -> list[BrightnessSample]:
        samples: list[BrightnessSample] = []
        state = 0
        edge_index = 0
        for frame_id in range(1, 141):
            timestamp_s = (frame_id - 1) * 0.01
            while edge_index < len(edge_times) and timestamp_s >= edge_times[edge_index] - 1e-9:
                state = 1 - state
                edge_index += 1
            noise = 1.5 if frame_id % 2 else -1.5
            brightness = (220.0 if state else 20.0) + noise
            samples.append(
                BrightnessSample(
                    frame_id=frame_id,
                    capture_monotonic_s=timestamp_s,
                    capture_unix_s=1000.0 + timestamp_s + camera_clock_offset_s,
                    roi_mean=brightness,
                )
            )
        return samples

    @staticmethod
    def _transitions(
        edge_times: tuple[float, ...],
        *,
        latency_s: float,
    ) -> list[DisplayTransition]:
        return [
            DisplayTransition(
                sequence=index,
                state=index % 2,
                display_monotonic_s=edge_time - latency_s,
                display_unix_s=1000.0 + edge_time - latency_s,
            )
            for index, edge_time in enumerate(edge_times, start=1)
        ]


if __name__ == "__main__":
    unittest.main()
