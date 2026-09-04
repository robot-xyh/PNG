import importlib.util
import json
import queue
import tempfile
import threading
import time
import unittest
from unittest import mock
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from vision_guidance.betaflight_msp import (
    AnalogTelemetry,
    AttitudeTelemetry,
    BetaflightTelemetry,
    RawImuTelemetry,
    StatusTelemetry,
)
from vision_guidance.flight_control import (
    GuidanceCommandShapingDiagnostics,
    GuidanceSetpoint,
    RcCommand,
)
from vision_guidance.fusion import VisionGuidanceResult
from vision_guidance.runtime_evidence import (
    AsyncJpegEvidenceRecorder,
    EvidenceFrameConfig,
    PreviewEvidenceMux,
)
from vision_guidance.types import CameraIntrinsics, FrameDetection, GuidanceEval, LOSEstimate, TTCState


def _load_runner_module():
    path = Path(__file__).resolve().parents[1] / "examples" / "run_betaflight_log_only.py"
    spec = importlib.util.spec_from_file_location("run_betaflight_log_only_for_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


runner = _load_runner_module()


class PostDisarmTailTest(unittest.TestCase):
    def test_starts_only_after_valid_arm_to_disarm_edge(self):
        tail = runner.PostDisarmTail(10.0)

        self.assertFalse(tail.update(1.0, False))
        self.assertFalse(tail.update(2.0, True))
        self.assertFalse(tail.update(3.0, None))
        self.assertTrue(tail.update(4.0, False))
        self.assertAlmostEqual(tail.remaining_s(9.0), 5.0)
        self.assertFalse(tail.complete(13.999))
        self.assertTrue(tail.complete(14.0))

    def test_rearm_cancels_active_tail(self):
        tail = runner.PostDisarmTail(10.0)
        tail.update(1.0, True)
        tail.update(2.0, False)

        self.assertFalse(tail.update(3.0, True))
        self.assertIsNone(tail.deadline_s)
        self.assertFalse(tail.complete(20.0))


class _FakeCapture:
    def __init__(self, image, *, accept_settings=True):
        self.image = image
        self.accept_settings = accept_settings
        self.opened = True
        self.released = False
        self.values = {
            1: float(image.shape[1]),
            2: float(image.shape[0]),
            3: 30.0,
            4: 0.0,
            5: 1.0,
        }

    def isOpened(self):
        return self.opened

    def set(self, prop, value):
        if self.accept_settings:
            self.values[prop] = float(value)
        return self.accept_settings

    def get(self, prop):
        return self.values.get(prop, 0.0)

    def read(self):
        return True, self.image.copy()

    def release(self):
        self.released = True


class _FakeCv2:
    CAP_PROP_FRAME_WIDTH = 1
    CAP_PROP_FRAME_HEIGHT = 2
    CAP_PROP_FPS = 3
    CAP_PROP_FOURCC = 4
    CAP_PROP_BUFFERSIZE = 5
    INTER_LINEAR = 6
    IMWRITE_JPEG_QUALITY = 7
    FONT_HERSHEY_SIMPLEX = 8
    LINE_AA = 9

    @staticmethod
    def VideoWriter_fourcc(*characters):
        return sum(ord(character) << (8 * index) for index, character in enumerate(characters))

    @staticmethod
    def resize(image, dimensions, interpolation):
        del interpolation
        width, height = dimensions
        return np.zeros((height, width, image.shape[2]), dtype=image.dtype)

    @staticmethod
    def undistort(image, matrix, distortion, _unused, new_matrix):
        del matrix, distortion, new_matrix
        return image

    @staticmethod
    def rectangle(*_args, **_kwargs):
        return None

    @staticmethod
    def putText(*_args, **_kwargs):
        return None

    @staticmethod
    def imencode(_extension, _image, _params):
        return True, np.asarray([0xFF, 0xD8, 0xFF, 0xD9], dtype=np.uint8)


class BetaflightLoggingTest(unittest.TestCase):
    def test_csv_source_wait_does_not_report_a_new_empty_detection(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "detections.csv"
            path.write_text(
                "frame_id,exposure_ts,x1,y1,x2,y2,track_id,score\n"
                "1,1.0,10,20,30,40,7,0.9\n",
                encoding="utf-8",
            )
            source = runner.DetectionCsvSource(str(path))

            waiting, waiting_stats = source.detect(
                elapsed_s=0.5,
                timestamp=10.5,
                frame_id=1,
            )
            detection, detection_stats = source.detect(
                elapsed_s=1.0,
                timestamp=11.0,
                frame_id=2,
            )

        self.assertIsNone(waiting)
        self.assertEqual(waiting_stats["detector_reject_reason"], "csv_waiting")
        self.assertEqual(waiting_stats["perception_new_result"], 0)
        self.assertIsNotNone(detection)
        self.assertEqual(detection.track_id, 7)
        self.assertEqual(detection_stats["perception_new_result"], 1)

    def test_cpu_affinity_parser_accepts_ranges_and_rejects_overlap(self):
        self.assertEqual(runner._parse_cpu_affinity("4-5,7"), (4, 5, 7))
        self.assertEqual(runner._parse_cpu_affinity(""), ())
        with self.assertRaisesRegex(ValueError, "invalid CPU affinity range"):
            runner._parse_cpu_affinity("5-4")
        with self.assertRaisesRegex(ValueError, "overlap"):
            runner._validate_cpu_affinity_plan((6, 7), (4, 7), isolate_rknn_process=True)
        with self.assertRaisesRegex(ValueError, "requires --isolate-rknn-process"):
            runner._validate_cpu_affinity_plan((6, 7), (4, 5), isolate_rknn_process=False)

    def test_latest_queue_replaces_stale_perception_result(self):
        channel = queue.Queue(maxsize=1)
        self.assertFalse(runner._queue_replace(channel, "first"))
        self.assertTrue(runner._queue_replace(channel, "second"))
        self.assertEqual(runner._queue_latest(channel), "second")
        self.assertIsNone(runner._queue_latest(channel))

    def test_isolated_preview_records_evidence_when_web_preview_is_disabled(self):
        class DisabledWebSink:
            config = SimpleNamespace(
                preview=SimpleNamespace(enabled=False, max_fps=1.0, jpeg_quality=70)
            )

            def wants_preview(self):
                return False

            def offer_encoded_preview(self, _jpeg):
                raise AssertionError("disabled web preview must not receive JPEG data")

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            recorder = AsyncJpegEvidenceRecorder(
                root / "frames",
                root / "frames.jsonl",
                EvidenceFrameConfig(enabled=True, max_fps=5.0, jpeg_quality=80),
            )
            recorder.start()
            self.addCleanup(recorder.close)
            mux = PreviewEvidenceMux(DisabledWebSink(), recorder)
            source = runner.IsolatedRknnByteTrackSource.__new__(
                runner.IsolatedRknnByteTrackSource
            )
            source._preview_sink = mux
            source._preview_request_event = threading.Event()
            source._preview_queue = queue.Queue(maxsize=1)
            source._preview_queue.put(
                (
                    b"isolated-jpeg",
                    {
                        "camera_capture_monotonic_s": 12.5,
                        "preview_encoded_monotonic_s": 12.55,
                    },
                )
            )

            source._relay_preview()

            deadline = time.monotonic() + 2.0
            while recorder.stats()["evidence_frame_write_count"] < 1:
                if time.monotonic() >= deadline:
                    self.fail("isolated preview evidence was not written")
                time.sleep(0.01)
            recorder.close()
            record = json.loads((root / "frames.jsonl").read_text(encoding="utf-8"))
            self.assertTrue(source._preview_request_event.is_set())
            self.assertEqual(record["metadata"]["camera_capture_monotonic_s"], 12.5)
            self.assertEqual(record["metadata"]["preview_encoded_monotonic_s"], 12.55)

    def test_isolated_preview_encoder_is_demand_driven_and_latest_only(self):
        channel = queue.Queue(maxsize=1)
        requested = threading.Event()
        encoder = runner._IsolatedPreviewEncoder(
            channel,
            requested,
            max_fps=10.0,
            jpeg_quality=70,
            cv2_module=_FakeCv2,
        )
        image = np.zeros((20, 20, 3), dtype=np.uint8)

        encoder.offer_preview(image, {"bbox_xyxy": [1, 2, 10, 12]})
        self.assertIsNone(runner._queue_latest(channel))

        requested.set()
        encoder.offer_preview(
            image,
            {"bbox_xyxy": [1, 2, 10, 12], "track_id": 4, "score": 0.75},
        )
        jpeg, metadata = runner._queue_latest(channel)
        self.assertEqual(jpeg, b"\xff\xd8\xff\xd9")
        self.assertEqual(metadata["track_id"], 4)
        self.assertIn("preview_encoded_monotonic_s", metadata)
        encoder.offer_preview(image)
        self.assertIsNone(runner._queue_latest(channel))
        self.assertEqual(encoder.stats()["perception_preview_encode_count"], 1)
        self.assertEqual(encoder.stats()["perception_preview_error_count"], 0)

    def test_camera_only_source_configures_resizes_and_logs_frame(self):
        image = np.zeros((1024, 1280, 3), dtype=np.uint8)
        capture = _FakeCapture(image)
        args = SimpleNamespace(camera_device="/dev/video-test")
        config = {
            "camera": {
                "capture_width": 1280,
                "capture_height": 1024,
                "width": 640,
                "height": 512,
                "fps": 180.0,
                "fourcc": "MJPG",
                "buffer_size": 1,
            }
        }
        source = runner.OpenCvCameraSource(
            args,
            config,
            cv2_module=_FakeCv2,
            capture_factory=lambda _device: capture,
        )

        resized = source.read_image()
        detection, stats = source.detect(timestamp=1.0, frame_id=1, active_track_id=None)

        self.assertEqual(resized.shape, (512, 640, 3))
        self.assertIsNone(detection)
        self.assertEqual(stats["detector_source"], "camera_only")
        self.assertEqual(stats["camera_device"], "/dev/video-test")
        self.assertEqual(stats["camera_frame_ok"], 1)
        self.assertEqual(stats["camera_input_width"], 1280)
        self.assertEqual(stats["camera_input_height"], 1024)
        self.assertEqual(stats["camera_output_width"], 640)
        self.assertEqual(stats["camera_output_height"], 512)
        self.assertEqual(stats["camera_reported_fourcc"], "MJPG")
        source.close()
        self.assertTrue(capture.released)

    def test_camera_source_rejects_capture_dimension_mismatch(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        capture = _FakeCapture(image, accept_settings=False)
        args = SimpleNamespace(camera_device="0")
        config = {"camera": {"capture_width": 1280, "capture_height": 1024, "width": 640, "height": 512}}

        with self.assertRaisesRegex(RuntimeError, "camera width mismatch"):
            runner.OpenCvCameraSource(
                args,
                config,
                cv2_module=_FakeCv2,
                capture_factory=lambda _device: capture,
            )

    def test_camera_device_accepts_index_or_path(self):
        self.assertEqual(runner._camera_device_value("1"), 1)
        self.assertEqual(runner._camera_device_value("/dev/v4l/by-id/camera"), "/dev/v4l/by-id/camera")

    def test_rknn_source_converts_bgr_to_rgb_and_uses_capture_timestamp(self):
        class FakeCamera:
            def __init__(self):
                self.last_stats = {"camera_capture_ts": 4.25, "camera_frame_ok": 1}
                self.closed = False

            def read_image(self):
                return np.array([[[1, 2, 3]]], dtype=np.uint8)

            def close(self):
                self.closed = True

        class FakeDetector:
            def __init__(self):
                self.closed = False

            def detect(self, image, *, frame_id, exposure_ts):
                self.call = (image.copy(), frame_id, exposure_ts)
                detection = FrameDetection(frame_id, exposure_ts, (0.0, 0.0, 1.0, 1.0), 1, 0.9)
                return detection, {"detector_source": "rknn_native", "rknn_inference_ms": 3.0}

            def metadata(self):
                return {"backend": "rknn_native"}

            def close(self):
                self.closed = True

        camera = FakeCamera()
        detector = FakeDetector()
        args = SimpleNamespace(rknn_library="bridge.so", rknn_model="model.rknn")
        source = runner.OpenCvRknnSource(args, {"rknn_detector": {}}, camera_source=camera, detector=detector)

        detection, stats = source.detect(timestamp=4.0, frame_id=9, active_track_id=None)

        np.testing.assert_array_equal(detector.call[0], np.array([[[3, 2, 1]]], dtype=np.uint8))
        self.assertEqual(detector.call[1:], (9, 4.25))
        self.assertEqual(detection.exposure_ts, 4.25)
        self.assertEqual(stats["camera_frame_ok"], 1)
        self.assertEqual(source.metadata()["backend"], "rknn_native")
        source.close()
        self.assertTrue(camera.closed)
        self.assertTrue(detector.closed)

    def test_rknn_bytetrack_worker_publishes_latest_result_without_backlog(self):
        class FakeCamera:
            def __init__(self):
                self.last_stats = {}

            def read_image(self):
                self.last_stats = {"camera_capture_ts": time.monotonic(), "camera_frame_ok": 1}
                return np.zeros((2, 2, 3), dtype=np.uint8)

            def close(self):
                self.closed = True

        class FakeDetector:
            def detect(self, image, *, frame_id, exposure_ts):
                del image
                detection = FrameDetection(frame_id, exposure_ts, (0.0, 0.0, 1.0, 1.0), 1, 0.9)
                return detection, {
                    "detector_source": "rknn_bytetrack",
                    "detector_reject_reason": "",
                    "bbox_measurement_source": "detector_update",
                }

            def metadata(self):
                return {"backend": "rknn_bytetrack"}

            def close(self):
                self.closed = True

        camera = FakeCamera()
        detector = FakeDetector()
        args = SimpleNamespace(rknn_library="bridge.so", rknn_model="model.rknn")
        config = {"rknn_bytetrack": {"perception_rate_hz": 100.0}}
        source = runner.OpenCvRknnByteTrackSource(args, config, camera_source=camera, detector=detector)
        time.sleep(0.04)

        detection, stats = source.detect(timestamp=time.monotonic(), frame_id=1, active_track_id=None)

        self.assertIsNotNone(detection)
        self.assertGreaterEqual(stats["perception_seq"], 2)
        self.assertGreaterEqual(stats["perception_queue_dropped"], 1)
        self.assertGreaterEqual(stats["perception_result_age_ms"], 0.0)
        source.close()
        self.assertTrue(camera.closed)
        self.assertTrue(detector.closed)

    def test_rknn_bytetrack_runtime_rate_override_is_explicit_and_recorded(self):
        class FakeCamera:
            last_stats = {"camera_frame_ok": 0}

            def read_image(self):
                return None

            def close(self):
                self.closed = True

        class FakeDetector:
            def metadata(self):
                return {"backend": "rknn_bytetrack"}

            def close(self):
                self.closed = True

        args = SimpleNamespace(
            rknn_library="bridge.so",
            rknn_model="model.rknn",
            rknn_perception_rate_hz=15.0,
        )
        source = runner.OpenCvRknnByteTrackSource(
            args,
            {"rknn_bytetrack": {"perception_rate_hz": 30.0}},
            camera_source=FakeCamera(),
            detector=FakeDetector(),
        )
        try:
            self.assertEqual(source.perception_rate_hz, 15.0)
            self.assertEqual(source.metadata()["perception_rate_hz"], 15.0)
        finally:
            source.close()

    def test_direct_rc_path_never_sends_without_msp_worker(self):
        class RejectingAdapter:
            def send_raw_rc(self, _command):
                raise AssertionError("direct RC send must not be called")

        args = SimpleNamespace(control_mode="msp_raw_rc", allow_control=True)
        command = RcCommand(timestamp=1.0, channels=(1500,) * 8, active=True)

        error = runner._maybe_send_rc(RejectingAdapter(), args, command, True, {})

        self.assertEqual(error, "msp_io_worker_required")

    def test_override_mode_can_explicitly_satisfy_shared_rc7_aux_gate(self):
        telemetry = BetaflightTelemetry(
            timestamp=1.0,
            rc_channels=(885, 885, 885, 885, 1000, 1000, 1000, 1000),
        )
        safety = {
            "require_aux_enable": True,
            "aux_enable": {
                "channel_index": 7,
                "min_us": 1700,
                "max_us": 2100,
                "satisfied_by_override_mode": True,
            },
        }

        self.assertFalse(runner._aux_enabled(telemetry, safety, override_active=False))
        self.assertTrue(runner._aux_enabled(telemetry, safety, override_active=True))

    def test_rk3588_torch_runtime_disables_mkldnn_and_limits_threads(self):
        fake_torch = SimpleNamespace(
            backends=SimpleNamespace(mkldnn=SimpleNamespace(enabled=True)),
            set_num_threads=lambda value: setattr(fake_torch, "num_threads", value),
        )

        runner._configure_torch_runtime(
            {"torch_runtime": {"num_threads": 1, "disable_mkldnn": True}},
            torch_module=fake_torch,
        )

        self.assertEqual(fake_torch.num_threads, 1)
        self.assertFalse(fake_torch.backends.mkldnn.enabled)

    def test_rk3588_config_blocks_cpu_yolo_after_bench_reboot(self):
        config = {"torch_runtime": {"allow_cpu_inference": False}}

        with self.assertRaisesRegex(RuntimeError, "CPU YOLO inference is disabled"):
            runner._validate_yolo_runtime(config, "cpu")

        runner._validate_yolo_runtime(config, "rknn")

    def test_guidance_evaluator_defaults_to_ttc_and_builds_fixed_vm(self):
        evaluator, metadata = runner._guidance_evaluator({})
        self.assertEqual(type(evaluator).__name__, "GuidanceEvaluator")
        self.assertEqual(metadata["law"], "ttc_png")
        self.assertTrue(metadata["ttc_required"])

        evaluator, metadata = runner._guidance_evaluator(
            {
                "guidance": {
                    "law": "fixed_vm_png",
                    "navigation_constant": 3.0,
                    "fixed_vm_m_s": 1.5,
                    "max_guidance_accel_mps2": 1.0,
                }
            }
        )
        self.assertEqual(type(evaluator).__name__, "FixedVmGuidanceEvaluator")
        self.assertEqual(metadata["fixed_gain"], 4.5)
        self.assertFalse(metadata["ttc_required"])

    def test_guidance_evaluator_rejects_unknown_or_incomplete_vm_config(self):
        with self.assertRaisesRegex(RuntimeError, "unsupported guidance.law"):
            runner._guidance_evaluator({"guidance": {"law": "vm"}})
        with self.assertRaisesRegex(RuntimeError, "guidance.fixed_vm_m_s is required"):
            runner._guidance_evaluator(
                {"guidance": {"law": "fixed_vm_png", "navigation_constant": 3.0}}
            )
        with self.assertRaisesRegex(RuntimeError, "finite and positive"):
            runner._guidance_evaluator(
                {
                    "guidance": {
                        "law": "fixed_vm_png",
                        "navigation_constant": 3.0,
                        "fixed_vm_m_s": 1.0,
                        "max_guidance_accel_mps2": 0.0,
                    }
                }
            )

    def test_velocity_establishing_guidance_is_explicit_and_bench_scoped(self):
        config = json.loads(
            (
                Path(__file__).resolve().parents[1]
                / "config/betaflight.rk3588.noprop.velocity_establishing.example.json"
            ).read_text()
        )
        evaluator, metadata = runner._guidance_evaluator(config)
        runtime = runner._velocity_establishing_runtime(
            config,
            intrinsics=runner._camera_intrinsics(config),
            bench_scope="noprop_bench",
        )

        self.assertEqual(type(evaluator).__name__, "FixedVmGuidanceEvaluator")
        self.assertEqual(metadata["law"], "velocity_establishing_png")
        self.assertEqual(metadata["velocity_source"], "bench_zero_velocity")
        self.assertEqual(metadata["fixed_gain"], 30.0)
        self.assertIsNotNone(runtime)
        command_metadata = runner._guidance_command_frame_metadata(config)
        self.assertEqual(
            command_metadata["accel_tilt_rate"]["pitch_rate_sign"],
            -1.0,
        )

        prop_rig_config = json.loads(
            (
                Path(__file__).resolve().parents[1]
                / "config/betaflight.rk3588.velocity_png.prop_rig_active.json"
            ).read_text()
        )
        prop_rig_runtime = runner._velocity_establishing_runtime(
            prop_rig_config,
            intrinsics=runner._camera_intrinsics(prop_rig_config),
            bench_scope="prop_rig_active",
        )
        self.assertIsNotNone(prop_rig_runtime)

        with self.assertRaisesRegex(RuntimeError, "restricted"):
            runner._velocity_establishing_runtime(
                config,
                intrinsics=runner._camera_intrinsics(config),
                bench_scope="flight_candidate",
            )

        excessive = json.loads(json.dumps(config))
        excessive["guidance"]["velocity_establishing_png"][
            "total_accel_limit_m_s2"
        ] = 1.1
        with self.assertRaisesRegex(RuntimeError, "must not exceed"):
            runner._guidance_evaluator(excessive)

    def test_guidance_command_frames_are_explicit_and_body_frd(self):
        metadata = runner._guidance_command_frame_metadata(
            {
                "guidance_command": {
                    "guidance_eval_frame": "inertial_ned",
                    "rate_gain_input_frame": "body_frd",
                }
            }
        )
        self.assertEqual(metadata["guidance_eval_frame"], "inertial_ned")
        self.assertEqual(metadata["rate_gain_input_frame"], "body_frd")
        self.assertEqual(metadata["command_mapping_type"], "direct_rate_matrix")

        accel_metadata = runner._guidance_command_frame_metadata(
            {
                "guidance_command": {
                    "guidance_eval_frame": "inertial_ned",
                    "rate_gain_input_frame": "body_frd",
                    "mapping_type": "accel_tilt_rate",
                    "accel_tilt_rate": {
                        "gravity_mps2": 9.80665,
                        "roll_attitude_kp_s_inv": 4.0,
                        "pitch_attitude_kp_s_inv": 4.0,
                        "max_roll_tilt_deg": 15.0,
                        "max_pitch_tilt_deg": 15.0,
                        "max_roll_rate_deg_s": 60.0,
                        "max_pitch_rate_deg_s": 60.0,
                        "roll_rate_sign": 1.0,
                        "pitch_rate_sign": 1.0,
                        "min_vertical_specific_force_mps2": 0.5,
                    },
                }
            }
        )
        self.assertEqual(accel_metadata["command_mapping_type"], "accel_tilt_rate")
        self.assertEqual(accel_metadata["accel_tilt_rate"]["max_roll_tilt_deg"], 15.0)

        with self.assertRaisesRegex(RuntimeError, "requires explicit fields"):
            runner._guidance_command_frame_metadata(
                {
                    "guidance_command": {
                        "guidance_eval_frame": "inertial_ned",
                        "rate_gain_input_frame": "body_frd",
                        "mapping_type": "accel_tilt_rate",
                        "accel_tilt_rate": {"max_roll_tilt_deg": 15.0},
                    }
                }
            )

        with self.assertRaisesRegex(RuntimeError, "guidance_eval_frame"):
            runner._guidance_command_frame_metadata({"guidance_command": {}})
        with self.assertRaisesRegex(RuntimeError, "rate_gain_input_frame"):
            runner._guidance_command_frame_metadata(
                {
                    "guidance_command": {
                        "guidance_eval_frame": "inertial_ned",
                        "rate_gain_input_frame": "inertial_ned",
                    }
                }
            )
        with self.assertRaisesRegex(RuntimeError, "mapping_type"):
            runner._guidance_command_frame_metadata(
                {
                    "guidance_command": {
                        "guidance_eval_frame": "inertial_ned",
                        "rate_gain_input_frame": "body_frd",
                        "mapping_type": "acceleration",
                    }
                }
            )

    def test_log_row_includes_expanded_telemetry_guidance_and_rc_fields(self):
        intrinsics = CameraIntrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        detection = FrameDetection(
            frame_id=7,
            exposure_ts=10.1,
            bbox_xyxy=(0.0, 5.0, 20.0, 35.0),
            track_id=42,
            score=0.8,
        )
        los = LOSEstimate(
            timestamp=10.1,
            lambda_I=np.array([1.0, 0.0, 0.0]),
            lambda_dot_I=np.array([0.0, 0.1, 0.0]),
            omega_los=np.array([0.0, 0.0, 0.1]),
            innovation_norm=0.25,
            quality=0.9,
            valid=True,
        )
        ttc = TTCState(
            timestamp=10.1,
            ttc=2.5,
            quality=0.8,
            area_filtered=100.0,
            area_dot_filtered=20.0,
            valid=True,
        )
        guidance = GuidanceEval(
            timestamp=10.1,
            g_eval=np.array([0.1, 0.2, 0.3]),
            valid=True,
            quality=0.7,
        )
        result = VisionGuidanceResult(
            detection=detection,
            los=los,
            ttc=ttc,
            guidance=guidance,
            R_IB=np.eye(3),
        )
        telemetry = BetaflightTelemetry(
            timestamp=10.0,
            status=StatusTelemetry(cycle_time_us=1000, i2c_error_count=1, sensor_flags=3, mode_flags=5, profile=2),
            attitude=AttitudeTelemetry(roll_deg=1.0, pitch_deg=-2.0, yaw_deg=30.0),
            analog=AnalogTelemetry(vbat_v=12.3, mah_drawn=100, rssi=900, amperage_a=3.21),
            rc_channels=(1000, 1100, 1200, 1300, 1800, 1500, 1500, 1500),
            motor_outputs=(1000, 1010, 1020, 1030, 0, 0, 0, 0),
            raw_imu=RawImuTelemetry((1, 2, 3), (4.0, 5.0, 6.0), (7, 8, 9)),
        )
        setpoint = GuidanceSetpoint(
            timestamp=10.1,
            roll_rate_deg_s=1.0,
            pitch_rate_deg_s=2.0,
            yaw_rate_deg_s=3.0,
            thrust=0.5,
            source="guidance_eval",
            thrust_model="measured_load_factor",
            thrust_required_specific_force_mps2=12.0,
            thrust_load_factor_raw_g=1.223,
            thrust_command_raw=0.581,
            thrust_command_limited=True,
        )
        pre_shape_setpoint = GuidanceSetpoint(
            timestamp=10.1,
            roll_rate_deg_s=4.0,
            pitch_rate_deg_s=6.0,
            yaw_rate_deg_s=3.0,
            thrust=0.5,
            source="guidance_eval",
            mapping_type="accel_tilt_rate",
            desired_roll_angle_deg=5.0,
            desired_pitch_angle_deg=-4.0,
            current_roll_angle_deg=1.0,
            current_pitch_angle_deg=-2.0,
            roll_attitude_error_deg=4.0,
            pitch_attitude_error_deg=-2.0,
            thrust_model="measured_load_factor",
            thrust_required_specific_force_mps2=12.0,
            thrust_load_factor_raw_g=1.223,
            thrust_command_raw=0.581,
            thrust_command_limited=True,
        )
        shaping = GuidanceCommandShapingDiagnostics(
            input_roll_rate_deg_s=4.0,
            input_pitch_rate_deg_s=6.0,
            output_roll_rate_deg_s=1.0,
            output_pitch_rate_deg_s=2.0,
            entry_active=True,
            entry_progress=0.25,
            entry_source="gyro",
            entry_start_roll_rate_deg_s=0.5,
            entry_start_pitch_rate_deg_s=-0.5,
            roll_attitude_deg=30.0,
            pitch_attitude_deg=-20.0,
            roll_softcap_factor=0.5,
            pitch_softcap_factor=1.0,
            roll_level_weight=0.0,
            pitch_level_weight=0.0,
        )
        rc_command = RcCommand(
            timestamp=10.1,
            channels=(1500, 1500, 1000, 1500, 1800, 1500, 1500, 1500),
            active=True,
            reason="active",
            raw_channels=(1500, 1500, 900, 1500, 1800, 1500, 1500, 1500),
            target_channels=(1500, 1500, 1000, 1500, 1800, 1500, 1500, 1500),
            clipped_flags=(0, 0, 1, 0, 0, 0, 0, 0),
            slew_limited_flags=(0, 0, 1, 0, 0, 0, 0, 0),
            requested_rates_deg_s=(10.0, 20.0, 30.0),
            limited_rates_deg_s=(3.0, 3.0, 0.0),
            stick_deflections=(0.01, 0.01, 0.0),
            requested_thrust=0.5,
            limited_thrust=0.1,
        )

        row = runner._log_row(
            timestamp=10.2,
            elapsed_s=0.2,
            telemetry=telemetry,
            telemetry_error="",
            detector_stats={
                "detector_source": "csv",
                "detector_reject_reason": "",
                "loop_period_s": 0.05,
                "camera_device": "/dev/video1",
                "camera_frame_ok": 1,
                "camera_capture_ts": 10.15,
                "camera_read_ms": 2.5,
                "camera_input_width": 1280,
                "camera_input_height": 1024,
                "camera_output_width": 640,
                "camera_output_height": 512,
                "camera_requested_fps": 180.0,
                "camera_reported_fps": 180.0,
                "camera_reported_fourcc": "MJPG",
                "camera_failed_frames": 0,
                "detector_raw_count": 2,
                "detector_class_filtered_count": 1,
                "detector_track_filtered_count": 1,
                "detector_best_score": 0.875,
                "rknn_selected_index": 0,
                "rknn_preprocess_ms": 1.0,
                "rknn_inference_ms": 4.0,
                "rknn_postprocess_ms": 0.5,
                "rknn_total_ms": 5.5,
                "guidance_law": "fixed_vm_png",
                "guidance_navigation_constant": 3.0,
                "guidance_fixed_vm_m_s": 1.5,
                "guidance_fixed_gain": 4.5,
                "guidance_max_accel_mps2": 1.0,
                "guidance_ttc_required": 0,
                "guidance_eval_frame": "inertial_ned",
                "rate_gain_input_frame": "body_frd",
                "command_mapping_type": "accel_tilt_rate",
                "msp_last_sent_channels": tuple(range(1000, 1016)),
                "python_gc_collection_count": 4,
                "python_gc_last_generation": 2,
                "python_gc_last_pause_ms": 1.25,
                "python_gc_max_pause_ms": 3.5,
                "python_gc_total_pause_ms": 7.75,
            },
            detection=detection,
            result=result,
            pre_shape_setpoint=pre_shape_setpoint,
            setpoint=setpoint,
            shaping=shaping,
            rc_command=rc_command,
            safety_state="ACTIVE",
            safety_reason="active",
            send_error="",
            telemetry_age_s=0.2,
            attitude_age_s=0.1,
            watchdog_age_s=0.05,
            telemetry_fresh=True,
            attitude_synced=True,
            watchdog_ok=True,
            voltage_ok=True,
            aux_enabled=True,
            control_requested=True,
            allow_control=True,
            intrinsics=intrinsics,
            channel_count=8,
            guidance_body_frd=np.array([0.1, 0.2, 0.3]),
        )

        fields = runner._log_fields(8)
        self.assertFalse(set(row) - set(fields))
        self.assertEqual(row["cycle_time_us"], 1000)
        self.assertEqual(row["mode_flags"], 5)
        self.assertEqual(row["mah_drawn"], 100)
        self.assertEqual(row["camera_device"], "/dev/video1")
        self.assertEqual(row["camera_read_ms"], "2.500")
        self.assertEqual(row["loop_period_s"], "0.050000")
        self.assertEqual(row["rknn_inference_ms"], "4.000")
        self.assertEqual(row["detector_best_score"], "0.875000")
        self.assertEqual(row["rc_in_ch5"], 1800)
        self.assertEqual(row["rc_in_all"], "1000,1100,1200,1300,1800,1500,1500,1500")
        self.assertEqual(row["rc_sent_all"], ",".join(str(value) for value in range(1000, 1016)))
        self.assertEqual(row["rc_sent_ch8"], 1007)
        self.assertEqual(row["gyro_msp_raw_x"], 4.0)
        self.assertEqual(row["motor_output_all"], "1000,1010,1020,1030,0,0,0,0")
        self.assertEqual(row["motor_output_ch4"], 1030)
        self.assertEqual(row["gyro_roll_deg_s"], "")
        self.assertEqual(row["map_limited_roll_rate_deg_s"], 3.0)
        self.assertEqual(row["rc_target_ch3"], 1000)
        self.assertEqual(row["bbox_clip_left"], 1)
        self.assertEqual(row["bbox_area"], "600.000")
        self.assertEqual(row["los_valid"], 1)
        self.assertEqual(row["lambda_dot_I_y"], "0.100000000")
        self.assertEqual(row["ttc_s"], "2.500000000")
        self.assertEqual(row["guidance_law"], "fixed_vm_png")
        self.assertEqual(row["guidance_fixed_gain"], "4.500000")
        self.assertEqual(row["guidance_ttc_required"], 0)
        self.assertEqual(row["guidance_eval_frame"], "inertial_ned")
        self.assertEqual(row["rate_gain_input_frame"], "body_frd")
        self.assertEqual(row["command_mapping_type"], "accel_tilt_rate")
        self.assertEqual(row["g_eval_body_frd_y"], "0.200000000")
        self.assertEqual(row["command_desired_roll_angle_deg"], "5.000000")
        self.assertEqual(row["command_current_pitch_angle_deg"], "-2.000000")
        self.assertEqual(row["command_roll_attitude_error_deg"], "4.000000")
        self.assertEqual(row["sp_source"], "guidance_eval")
        self.assertEqual(row["pre_shape_sp_roll_rate_deg_s"], "4.000000")
        self.assertEqual(row["sp_roll_rate_deg_s"], "1.000000")
        self.assertEqual(row["command_thrust_model"], "measured_load_factor")
        self.assertEqual(row["command_thrust_load_factor_raw_g"], "1.223000")
        self.assertEqual(row["command_thrust_raw"], "0.581000")
        self.assertEqual(row["command_thrust_limited"], 1)
        self.assertEqual(row["entry_handoff_progress"], "0.250000")
        self.assertEqual(row["entry_handoff_source"], "gyro")
        self.assertEqual(row["tilt_roll_softcap_factor"], "0.500000")
        self.assertEqual(row["tilt_roll_attitude_deg"], "30.000000")
        self.assertEqual(row["rc_raw_ch3"], 900)
        self.assertEqual(row["rc_clipped_ch3"], 1)
        self.assertEqual(row["rc_slew_limited_ch3"], 1)
        self.assertEqual(row["python_gc_collection_count"], 4)
        self.assertEqual(row["python_gc_max_pause_ms"], "3.500000")

    def test_log_row_uses_empty_strings_for_missing_optional_data(self):
        intrinsics = CameraIntrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
        rc_command = RcCommand(timestamp=1.0, channels=(1500,) * 8, active=False)

        row = runner._log_row(
            timestamp=1.0,
            elapsed_s=0.0,
            telemetry=None,
            telemetry_error="timeout",
            detector_stats={"detector_source": "none", "detector_reject_reason": "detector_disabled"},
            detection=None,
            result=None,
            pre_shape_setpoint=GuidanceSetpoint(
                timestamp=1.0, valid=False, reject_reason="guidance_missing"
            ),
            setpoint=GuidanceSetpoint(timestamp=1.0, valid=False, reject_reason="guidance_missing"),
            shaping=GuidanceCommandShapingDiagnostics(
                valid=False, reason="guidance_missing"
            ),
            rc_command=rc_command,
            safety_state="LOG_ONLY",
            safety_reason="log_only",
            send_error="",
            telemetry_age_s=None,
            attitude_age_s=None,
            watchdog_age_s=None,
            telemetry_fresh=False,
            attitude_synced=False,
            watchdog_ok=False,
            voltage_ok=True,
            aux_enabled=False,
            control_requested=False,
            allow_control=False,
            intrinsics=intrinsics,
            channel_count=8,
        )

        self.assertEqual(row["telemetry_age_s"], "")
        self.assertEqual(row["vbat_v"], "")
        self.assertEqual(row["bbox_area"], "")
        self.assertEqual(row["los_valid"], "")
        self.assertEqual(row["ttc_s"], "")
        self.assertEqual(row["rc_raw_ch1"], "")
        self.assertEqual(row["command_desired_roll_angle_deg"], "")

    def test_write_run_meta_records_config_args_fields_and_identity(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "betaflight_log_meta.json"
            log_path = Path(tmpdir) / "betaflight_log.csv"
            args = SimpleNamespace(control_mode="log_only", allow_control=False, duration_s=1.0)

            runner._write_run_meta(
                path,
                args=args,
                config={"serial": {"port": "/dev/null"}},
                log_path=log_path,
                fields=["timestamp", "mode_flags"],
                fc_identity={"fc_variant": "BTFL"},
                kinematics={"control_connected": False},
                runtime_diagnostics={"python_gc_pause_monitor": True},
            )

            data = json.loads(path.read_text())
            self.assertEqual(data["log_csv"], str(log_path))
            self.assertEqual(data["args"]["control_mode"], "log_only")
            self.assertEqual(data["config"]["serial"]["port"], "/dev/null")
            self.assertEqual(data["fields"], ["timestamp", "mode_flags"])
            self.assertEqual(data["fc_identity"]["fc_variant"], "BTFL")
            self.assertEqual(data["log_schema_version"], 21)
            self.assertFalse(data["kinematics"]["control_connected"])
            self.assertTrue(data["runtime_diagnostics"]["python_gc_pause_monitor"])
            self.assertIn("repository_dirty", data)
            self.assertTrue(data["source_files"])

            completion = {
                "complete": True,
                "stop_reason": "post_disarm_tail_complete",
                "rows_written": 123,
            }
            runner._update_run_completion(path, completion)
            updated = json.loads(path.read_text())
            self.assertEqual(updated["completion"], completion)

    def test_camera_mount_requires_explicit_verified_upward_extrinsic_for_control(self):
        legacy = {"camera": {"pitch_up_deg": 90.0}}
        metadata = runner._camera_calibration_metadata(legacy, runner._camera_mount(legacy))
        self.assertEqual(metadata["extrinsic"]["source"], "legacy_pitch_up_deg")
        self.assertAlmostEqual(metadata["extrinsic"]["optical_axis_error_deg"], 90.0)
        self.assertFalse(metadata["extrinsic"]["control_ready"])
        with self.assertRaisesRegex(RuntimeError, "R_BC must be explicit"):
            runner._camera_mount(legacy, require_control_ready=True)

        explicit = {
            "camera": {
                "R_BC": [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
                "extrinsic_validation": {
                    "verified": True,
                    "body_frame": "FRD",
                    "camera_frame": "opencv_x_right_y_down_z_forward",
                    "expected_optical_axis_body": [0.0, 0.0, -1.0],
                    "max_optical_axis_error_deg": 5.0,
                },
            }
        }
        rotation = runner._camera_mount(explicit, require_control_ready=True)
        metadata = runner._camera_calibration_metadata(explicit, rotation)
        self.assertTrue(metadata["extrinsic"]["control_ready"])
        self.assertFalse(metadata["timestamp"]["hardware_exposure"])

    def test_edge_event_logger_writes_only_state_changes(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "events.jsonl"
            logger = runner.EdgeEventLogger(path, start_s=10.0)
            logger.update({"armed": 0}, timestamp_s=10.1)
            logger.update({"armed": 0}, timestamp_s=10.2)
            logger.update(
                {"armed": 1},
                timestamp_s=10.3,
                context={"rc_sent": [1500, 1500, 1000, 1500]},
            )
            logger.close()

            events = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len(events), 2)
            self.assertIsNone(events[0]["old"])
            self.assertEqual(events[1]["old"], 0)
            self.assertEqual(events[1]["new"], 1)
            self.assertEqual(events[1]["context"]["rc_sent"][2], 1000)

    def test_edge_event_logger_fsyncs_each_event(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "events.jsonl"
            with mock.patch.object(runner.os, "fsync") as fsync:
                logger = runner.EdgeEventLogger(path, start_s=10.0)
                logger.write("run_start", timestamp_s=10.0, new="running")
                logger.close()

            fsync.assert_called_once()

    def test_disabled_evidence_index_is_not_a_required_runtime_artifact(self):
        root = Path("/tmp/runtime-artifacts")
        artifacts = runner._runtime_artifacts(
            meta_path=root / "run_meta.json",
            log_path=root / "run.csv",
            events_path=root / "run_events.jsonl",
            marker_path=root / "run_markers.jsonl",
            evidence_index_path=root / "run_evidence_frames.jsonl",
            evidence_enabled=False,
        )

        self.assertNotIn(root / "run_evidence_frames.jsonl", artifacts)

    def test_track_id_event_update_ignores_perception_wait_cycles(self):
        detection = FrameDetection(1, 10.0, (1.0, 2.0, 10.0, 12.0), 7, 0.8)

        self.assertEqual(
            runner._track_id_event_update(detection, perception_new_result=0),
            {},
        )
        self.assertEqual(
            runner._track_id_event_update(None, perception_new_result="0"),
            {},
        )

    def test_track_id_event_update_records_new_detection_and_real_loss(self):
        detection = FrameDetection(1, 10.0, (1.0, 2.0, 10.0, 12.0), 7, 0.8)

        self.assertEqual(
            runner._track_id_event_update(detection, perception_new_result=1),
            {"track_id": 7},
        )
        self.assertEqual(
            runner._track_id_event_update(None, perception_new_result=1),
            {"track_id": None},
        )

    def test_durable_transition_state_ignores_non_algorithm_publish_substates(self):
        common = {"armed": False, "override_active": False, "safety_state": "READY"}

        states = {
            runner._durable_transition_state(**common, publish_mode=mode)
            for mode in ("disabled", "prefill", "passthrough", "physical_rc_stale")
        }
        self.assertEqual(len(states), 1)
        self.assertNotEqual(
            states.pop(),
            runner._durable_transition_state(**common, publish_mode="algorithm"),
        )

    def test_durable_csv_flushes_periodically_and_syncs_transitions_and_close(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "durable.csv"
            with mock.patch.object(runner.os, "fsync") as fsync:
                logger = runner.DurableCsvLogger(
                    path,
                    ["timestamp", "state"],
                    flush_interval_s=1.0,
                )
                self.assertEqual(fsync.call_count, 1)
                with mock.patch.object(logger, "flush", wraps=logger.flush) as flush:
                    logger.write_row(
                        {"timestamp": "1.0", "state": "idle"},
                        timestamp_s=1.0,
                        transition_state=(0, 0, "LOG_ONLY", "disabled"),
                    )
                    logger.write_row(
                        {"timestamp": "1.5", "state": "idle"},
                        timestamp_s=1.5,
                        transition_state=(0, 0, "LOG_ONLY", "disabled"),
                    )
                    logger.write_row(
                        {"timestamp": "2.1", "state": "idle"},
                        timestamp_s=2.1,
                        transition_state=(0, 0, "LOG_ONLY", "disabled"),
                    )
                    self.assertEqual(flush.call_count, 2)
                logger.write_row(
                    {"timestamp": "2.2", "state": "active"},
                    timestamp_s=2.2,
                    transition_state=(1, 1, "ACTIVE", "algorithm"),
                )
                self.assertEqual(fsync.call_count, 2)
                logger.close()
                self.assertEqual(fsync.call_count, 3)

            self.assertEqual(len(path.read_text(encoding="utf-8").splitlines()), 5)

    def test_python_gc_pause_monitor_records_incremental_pauses(self):
        times = iter((1.0, 1.0125, 2.0, 2.003))
        monitor = runner.PythonGcPauseMonitor(clock=lambda: next(times))

        monitor._callback("start", {"generation": 2})
        monitor._callback("stop", {"generation": 2})
        monitor._callback("start", {"generation": 0})
        monitor._callback("stop", {"generation": 0})
        snapshot = monitor.snapshot()

        self.assertEqual(snapshot["python_gc_collection_count"], 2)
        self.assertEqual(snapshot["python_gc_last_generation"], 0)
        self.assertAlmostEqual(snapshot["python_gc_last_pause_ms"], 3.0)
        self.assertAlmostEqual(snapshot["python_gc_max_pause_ms"], 12.5)
        self.assertAlmostEqual(snapshot["python_gc_total_pause_ms"], 15.5)

    def test_realtime_gc_guard_collects_disables_and_restores_gc(self):
        with mock.patch.object(runner.gc, "isenabled", return_value=True), mock.patch.object(
            runner.gc, "collect"
        ) as collect, mock.patch.object(runner.gc, "disable") as disable, mock.patch.object(
            runner.gc, "enable"
        ) as enable:
            guard = runner.RealtimeGcGuard()
            guard.start()
            guard.start()
            guard.close()
            guard.close()

        collect.assert_called_once_with()
        disable.assert_called_once_with()
        enable.assert_called_once_with()
        self.assertTrue(guard.metadata()["automatic_gc_disabled_during_run"])

    def test_realtime_gc_guard_preserves_preexisting_disabled_state(self):
        with mock.patch.object(runner.gc, "isenabled", return_value=False), mock.patch.object(
            runner.gc, "collect"
        ) as collect, mock.patch.object(runner.gc, "disable") as disable, mock.patch.object(
            runner.gc, "enable"
        ) as enable:
            guard = runner.RealtimeGcGuard()
            guard.start()
            guard.close()

        collect.assert_not_called()
        disable.assert_not_called()
        enable.assert_not_called()


if __name__ == "__main__":
    unittest.main()
