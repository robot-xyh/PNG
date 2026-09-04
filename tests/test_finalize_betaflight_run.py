import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

from vision_guidance.runtime_evidence import write_run_manifest


ROOT = Path(__file__).resolve().parents[1]
MODE_BINDING = (
    ROOT
    / "config"
    / "betaflight.blackbox_mode_binding.btfl-25.12.2-micoair743v2.json"
)


def _load_tool():
    path = ROOT / "tools" / "finalize_betaflight_run.py"
    spec = importlib.util.spec_from_file_location("finalize_betaflight_run_for_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tool = _load_tool()


class FinalizeBetaflightRunTest(unittest.TestCase):
    def test_finalization_binds_external_evidence_and_clock_uncertainty(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            csv_path = root / "run.csv"
            csv_path.write_text("x\n1\n", encoding="utf-8")
            meta_path = root / "run_meta.json"
            meta_path.write_text(
                json.dumps(
                    {
                        "fc_identity": {
                            "fc_variant": "BTFL",
                            "fc_version_major": 25,
                            "fc_version_minor": 12,
                            "fc_version_patch": 2,
                            "api_major": 1,
                            "api_minor": 47,
                        }
                    }
                ),
                encoding="utf-8",
            )
            runtime_path = root / "runtime_manifest.json"
            write_run_manifest(
                runtime_path,
                artifacts=(csv_path, meta_path),
                completion={"complete": True},
            )
            blackbox = root / "LOG.BFL"
            blackbox.write_bytes(b"blackbox")

            result = tool.finalize(
                runtime_path,
                artifacts={"blackbox": blackbox},
                pairing_confidence="unique",
                clock_uncertainty_ms=4.0,
                miss_distance_method="contact_anchor",
                acknowledge_incomplete_run=False,
                blackbox_mode_binding_path=MODE_BINDING,
            )

            self.assertTrue(result["finalized"])
            self.assertTrue(result["pairing"]["hardware_latency_claim_allowed"])
            self.assertIn("blackbox", result["external_artifacts"])
            self.assertFalse(
                result["blackbox_interpretation"][
                    "decoder_labels_used_for_mode_decisions"
                ]
            )
            self.assertFalse(
                result["miss_distance"]["independent_absolute_gnss_allowed"]
            )

    def test_blackbox_requires_firmware_mode_binding(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "run.csv"
            artifact.write_text("x\n", encoding="utf-8")
            runtime_path = root / "runtime_manifest.json"
            write_run_manifest(
                runtime_path,
                artifacts=(artifact,),
                completion={"complete": True},
            )
            blackbox = root / "LOG.BFL"
            blackbox.write_bytes(b"blackbox")

            with self.assertRaisesRegex(RuntimeError, "mode-binding"):
                tool.finalize(
                    runtime_path,
                    artifacts={"blackbox": blackbox},
                    pairing_confidence="unique",
                    clock_uncertainty_ms=10.0,
                    miss_distance_method="not_evaluated",
                    acknowledge_incomplete_run=False,
                )

    def test_independent_absolute_gnss_miss_distance_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "run.csv"
            artifact.write_text("x\n", encoding="utf-8")
            runtime_path = root / "runtime_manifest.json"
            write_run_manifest(
                runtime_path,
                artifacts=(artifact,),
                completion={"complete": True},
            )

            with self.assertRaisesRegex(RuntimeError, "absolute GNSS"):
                tool.finalize(
                    runtime_path,
                    artifacts={},
                    pairing_confidence="unique",
                    clock_uncertainty_ms=10.0,
                    miss_distance_method="independent_absolute_gnss",
                    acknowledge_incomplete_run=False,
                )

    def test_runtime_artifact_tamper_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "run.csv"
            artifact.write_text("x\n", encoding="utf-8")
            runtime_path = root / "runtime_manifest.json"
            write_run_manifest(
                runtime_path,
                artifacts=(artifact,),
                completion={"complete": True},
            )
            artifact.write_text("changed\n", encoding="utf-8")

            with self.assertRaisesRegex(RuntimeError, "changed"):
                tool.finalize(
                    runtime_path,
                    artifacts={},
                    pairing_confidence="unique",
                    clock_uncertainty_ms=10.0,
                    miss_distance_method="not_evaluated",
                    acknowledge_incomplete_run=False,
                )


if __name__ == "__main__":
    unittest.main()
