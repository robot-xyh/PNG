import tempfile
import time
import unittest
from pathlib import Path

from vision_guidance.platform_health import PlatformHealthSampler


class PlatformHealthTest(unittest.TestCase):
    def test_sampler_collects_cached_process_and_disk_health(self):
        with tempfile.TemporaryDirectory() as directory:
            sampler = PlatformHealthSampler(sample_hz=20.0, log_directory=Path(directory))

            sampler.start()
            time.sleep(0.08)
            snapshot = sampler.snapshot()
            sampler.close()

            self.assertIsNotNone(snapshot.timestamp_s)
            self.assertIsNotNone(snapshot.process_rss_mb)
            self.assertGreater(snapshot.process_rss_mb, 0.0)
            self.assertIsNotNone(snapshot.mem_available_mb)
            self.assertGreater(snapshot.disk_free_gb, 0.0)
            self.assertEqual(sampler.metadata()["sample_hz"], 20.0)

    def test_sampler_rejects_nonpositive_rate(self):
        with self.assertRaisesRegex(ValueError, "positive"):
            PlatformHealthSampler(sample_hz=0.0, log_directory=".")


if __name__ == "__main__":
    unittest.main()
