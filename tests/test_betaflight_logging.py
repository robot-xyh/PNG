import importlib.util
import json
import tempfile
import time
import unittest
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
from vision_guidance.flight_control import GuidanceSetpoint, RcCommand
from vision_guidance.fusion import VisionGuidanceResult
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


class BetaflightLoggingTest(unittest.TestCase):
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
        result = VisionGuidanceResult(detection=detection, los=los, ttc=ttc, guidance=guidance)
        telemetry = BetaflightTelemetry(
            timestamp=10.0,
            status=StatusTelemetry(cycle_time_us=1000, i2c_error_count=1, sensor_flags=3, mode_flags=5, profile=2),
            attitude=AttitudeTelemetry(roll_deg=1.0, pitch_deg=-2.0, yaw_deg=30.0),
            analog=AnalogTelemetry(vbat_v=12.3, mah_drawn=100, rssi=900, amperage_a=3.21),
            rc_channels=(1000, 1100, 1200, 1300, 1800, 1500, 1500, 1500),
            raw_imu=RawImuTelemetry((1, 2, 3), (4.0, 5.0, 6.0), (7, 8, 9)),
        )
        setpoint = GuidanceSetpoint(
            timestamp=10.1,
            roll_rate_deg_s=1.0,
            pitch_rate_deg_s=2.0,
            yaw_rate_deg_s=3.0,
            thrust=0.5,
            source="guidance_eval",
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
                "msp_last_sent_channels": tuple(range(1000, 1016)),
            },
            detection=detection,
            result=result,
            setpoint=setpoint,
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
        self.assertEqual(row["gyro_roll_deg_s"], 4.0)
        self.assertEqual(row["map_limited_roll_rate_deg_s"], 3.0)
        self.assertEqual(row["rc_target_ch3"], 1000)
        self.assertEqual(row["bbox_clip_left"], 1)
        self.assertEqual(row["bbox_area"], "600.000")
        self.assertEqual(row["los_valid"], 1)
        self.assertEqual(row["lambda_dot_I_y"], "0.100000000")
        self.assertEqual(row["ttc_s"], "2.500000000")
        self.assertEqual(row["sp_source"], "guidance_eval")
        self.assertEqual(row["rc_raw_ch3"], 900)
        self.assertEqual(row["rc_clipped_ch3"], 1)
        self.assertEqual(row["rc_slew_limited_ch3"], 1)

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
            setpoint=GuidanceSetpoint(timestamp=1.0, valid=False, reject_reason="guidance_missing"),
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
            )

            data = json.loads(path.read_text())
            self.assertEqual(data["log_csv"], str(log_path))
            self.assertEqual(data["args"]["control_mode"], "log_only")
            self.assertEqual(data["config"]["serial"]["port"], "/dev/null")
            self.assertEqual(data["fields"], ["timestamp", "mode_flags"])
            self.assertEqual(data["fc_identity"]["fc_variant"], "BTFL")
            self.assertEqual(data["log_schema_version"], 5)

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


if __name__ == "__main__":
    unittest.main()
