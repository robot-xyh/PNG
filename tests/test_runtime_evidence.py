import json
import tempfile
import time
import unittest
from pathlib import Path

from vision_guidance.runtime_evidence import (
    AsyncJpegEvidenceRecorder,
    EvidenceFrameConfig,
    ExclusiveResourceLock,
    OperatorMarkerInbox,
    append_operator_marker,
    verify_evidence_frame_index,
    write_run_manifest,
)


class RuntimeEvidenceTest(unittest.TestCase):
    def test_exclusive_lock_reports_existing_owner(self):
        with tempfile.TemporaryDirectory() as directory:
            first = ExclusiveResourceLock("/dev/test-camera", lock_directory=directory)
            second = ExclusiveResourceLock("/dev/test-camera", lock_directory=directory)
            first.acquire()
            self.addCleanup(first.close)

            with self.assertRaisesRegex(RuntimeError, "pid"):
                second.acquire()
            first.close()
            second.acquire()
            second.close()

    def test_operator_marker_is_fsynced_and_consumed_once(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "markers.jsonl"
            inbox = OperatorMarkerInbox(path)
            record = append_operator_marker(
                path,
                event="target_crossing",
                note="left_to_right",
                tags=("F04",),
            )

            self.assertEqual(inbox.poll(), [record])
            self.assertEqual(inbox.poll(), [])

    def test_encoded_evidence_is_written_with_hash_index(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorder = AsyncJpegEvidenceRecorder(
                root / "frames",
                root / "index.jsonl",
                EvidenceFrameConfig(enabled=True, max_fps=10.0, jpeg_quality=80),
            )
            recorder.start()
            recorder.offer_encoded_preview(b"jpeg-evidence")
            deadline = time.monotonic() + 2.0
            while recorder.stats()["evidence_frame_write_count"] < 1:
                if time.monotonic() >= deadline:
                    self.fail("evidence recorder did not write queued JPEG")
                time.sleep(0.01)
            recorder.close()

            record = json.loads((root / "index.jsonl").read_text(encoding="utf-8"))
            self.assertEqual(record["bytes"], len(b"jpeg-evidence"))
            self.assertTrue(Path(record["path"]).is_file())
            self.assertEqual(recorder.stats()["evidence_frame_error_count"], 0)
            verified = verify_evidence_frame_index(root / "index.jsonl")
            self.assertEqual(verified["frame_count"], 1)

            Path(record["path"]).write_bytes(b"tampered")
            with self.assertRaisesRegex(RuntimeError, "changed"):
                verify_evidence_frame_index(root / "index.jsonl")

    def test_run_manifest_hashes_available_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = root / "run.csv"
            log.write_text("x\n1\n", encoding="utf-8")
            manifest_path = root / "manifest.json"
            manifest = write_run_manifest(
                manifest_path,
                artifacts=(log, root / "missing.json"),
                completion={"complete": True, "stop_reason": "duration_complete"},
            )

            self.assertFalse(manifest["finalized"])
            self.assertEqual(manifest["artifacts"][0]["path"], str(log.resolve()))
            self.assertEqual(manifest["missing_runtime_artifacts"], [str(root / "missing.json")])


if __name__ == "__main__":
    unittest.main()
