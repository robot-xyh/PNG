import http.client
import json
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np

from vision_guidance.betaflight_web import (
    DEFAULT_DASHBOARD_PATH,
    PreviewWebConfig,
    TelemetryHub,
    TelemetryWebConfig,
    TelemetryWebService,
    telemetry_payload_from_log_row,
)


class _FakeCv2:
    IMWRITE_JPEG_QUALITY = 1
    MARKER_CROSS = 2
    LINE_AA = 3
    FONT_HERSHEY_SIMPLEX = 4

    @staticmethod
    def drawMarker(*_args, **_kwargs):
        return None

    @staticmethod
    def rectangle(*_args, **_kwargs):
        return None

    @staticmethod
    def putText(*_args, **_kwargs):
        return None

    @staticmethod
    def imencode(_extension, _image, _params):
        return True, np.asarray([0xFF, 0xD8, 0xFF, 0xD9], dtype=np.uint8)


class BetaflightWebTest(unittest.TestCase):
    def test_dashboard_exposes_control_readiness_without_raw_json(self):
        dashboard = DEFAULT_DASHBOARD_PATH.read_text(encoding="utf-8")

        for element_id in (
            "controlSummary",
            "gateApproval",
            "gateArm",
            "gateOverride",
            "gatePrefill",
            "gatePhysicalRc",
            "gateTelemetry",
            "gateAttitude",
            "gateTracker",
            "gateLos",
            "gateTtc",
            "gateWatchdog",
            "gateGuidance",
            "gateCommand",
            "gatePublish",
            "sentFrame",
            "setRawRcCount",
            "setRawRcErrors",
            "deadlineMisses",
        ):
            self.assertIn(f'id="{element_id}"', dashboard)
        self.assertIn('["rc", "physical_us", 0]', dashboard)
        self.assertNotIn('id="targetPill"', dashboard)

    def test_config_validates_network_and_preview_bounds(self):
        config = TelemetryWebConfig.from_mapping(
            {
                "enabled": True,
                "bind": "0.0.0.0",
                "port": 8080,
                "allowed_subnets": ["127.0.0.0/8", "192.168.124.0/24"],
                "preview": {"max_fps": 10, "jpeg_quality": 70, "max_clients": 2},
            }
        )

        self.assertTrue(config.enabled)
        self.assertEqual(config.history_capacity, 301)
        self.assertEqual(config.preview.max_clients, 2)
        with self.assertRaises(ValueError):
            TelemetryWebConfig.from_mapping({"allowed_subnets": ["not-a-network"]})
        with self.assertRaises(ValueError):
            TelemetryWebConfig.from_mapping({"preview": {"jpeg_quality": 10}})

    def test_payload_from_log_row_is_typed_and_grouped(self):
        row = {
            "elapsed_s": "1.25",
            "safety_state": "LOG_ONLY",
            "armed": "0",
            "msp_override_available": "1",
            "msp_override_active": "0",
            "guidance_valid": "1",
            "roll_deg": "2.5",
            "pitch_deg": "-1.0",
            "yaw_deg": "90.0",
            "vbat_v": "4.20",
            "track_id": "7",
            "detection_score": "0.75",
            "bbox_x1": "1",
            "bbox_y1": "2",
            "bbox_x2": "3",
            "bbox_y2": "4",
            "ttc_s": "0.5",
            "rknn_total_ms": "6.2",
            "rc_in_ch1": "1500",
            "rc_in_ch2": "1490",
            "rc_in_ch3": "1510",
            "rc_in_ch4": "885",
            "rc_sent_ch1": "",
            "web_running": "1",
            "web_error_count": "0",
        }

        payload = telemetry_payload_from_log_row(row, channel_count=8)

        self.assertEqual(payload["safety"]["state"], "LOG_ONLY")
        self.assertFalse(payload["safety"]["armed"])
        self.assertTrue(payload["safety"]["target_valid"])
        self.assertEqual(payload["flight_controller"]["attitude_deg"], [2.5, -1.0, 90.0])
        self.assertEqual(payload["vision"]["bbox_xyxy"], [1.0, 2.0, 3.0, 4.0])
        self.assertEqual(payload["rc"]["input_us"][0], 1500)
        self.assertEqual(payload["rc"]["input_order"], "AERT1234")
        self.assertEqual(payload["rc"]["wire_order"], "AETR1234")
        self.assertEqual(payload["rc"]["physical_us"][:4], [1500, 1490, 885, 1510])
        self.assertIsNone(payload["rc"]["sent_us"][0])
        self.assertTrue(payload["web"]["running"])

    def test_hub_decimates_history_and_marks_stale(self):
        config = TelemetryWebConfig(sample_hz=2.0, history_s=2.0, stale_after_s=0.5)
        hub = TelemetryHub(config)
        hub.start()
        try:
            hub.publish({"value": 1}, timestamp_s=10.0)
            hub.publish({"value": 2}, timestamp_s=10.1)
            hub.publish({"value": 3}, timestamp_s=10.6)

            self.assertEqual(hub.latest(timestamp_s=10.7)["value"], 3)
            self.assertFalse(hub.latest(timestamp_s=10.7)["stale"])
            self.assertTrue(hub.latest(timestamp_s=11.2)["stale"])
            self.assertEqual([item["value"] for item in hub.history(timestamp_s=10.7)["samples"]], [1, 3])
        finally:
            hub.close()

    def test_preview_encodes_only_with_client_and_uses_latest_slot(self):
        config = TelemetryWebConfig(
            preview=PreviewWebConfig(enabled=True, max_fps=30.0, jpeg_quality=70, max_clients=1)
        )
        hub = TelemetryHub(config, cv2_module=_FakeCv2)
        hub.start()
        frame = np.zeros((20, 20, 3), dtype=np.uint8)
        try:
            hub.offer_preview(frame)
            time.sleep(0.05)
            self.assertEqual(hub.stats()["web_preview_encode_count"], 0)
            self.assertTrue(hub.try_add_client("mjpeg"))
            self.assertFalse(hub.try_add_client("mjpeg"))
            hub.offer_preview(frame, {"bbox_xyxy": [1, 1, 10, 10], "track_id": 2, "score": 0.8})
            sequence, jpeg = hub.wait_for_jpeg(0, timeout_s=1.0)

            self.assertEqual(sequence, 1)
            self.assertEqual(jpeg, b"\xff\xd8\xff\xd9")
            self.assertEqual(hub.stats()["web_preview_encode_count"], 1)
            hub.remove_client("mjpeg")
        finally:
            hub.close()

    def test_http_json_sse_mjpeg_and_read_only_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            dashboard = Path(directory) / "dashboard.html"
            dashboard.write_text("<!doctype html><title>test</title>", encoding="utf-8")
            config = TelemetryWebConfig(
                enabled=True,
                bind="127.0.0.1",
                port=0,
                allowed_subnets=("127.0.0.0/8",),
                sample_hz=20.0,
                preview=PreviewWebConfig(enabled=True, max_fps=30.0, jpeg_quality=70, max_clients=1),
            )
            service = TelemetryWebService(config, dashboard_path=dashboard, cv2_module=_FakeCv2)
            service.start()
            port = service._server.server_address[1]
            service.publish({"safety": {"state": "LOG_ONLY"}}, timestamp_s=time.monotonic())
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=2) as response:
                    self.assertEqual(response.status, 200)
                    self.assertIn(b"<title>test</title>", response.read())
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/v1/telemetry", timeout=2) as response:
                    data = json.load(response)
                    self.assertEqual(data["safety"]["state"], "LOG_ONLY")
                    self.assertFalse(data["stale"])
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/healthz", timeout=2) as response:
                    self.assertTrue(json.load(response)["ok"])

                request = urllib.request.Request(f"http://127.0.0.1:{port}/api/v1/telemetry", method="POST")
                with self.assertRaises(urllib.error.HTTPError) as raised:
                    urllib.request.urlopen(request, timeout=2)
                self.assertEqual(raised.exception.code, 405)

                sse = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
                sse.request("GET", "/api/v1/stream")
                sse_response = sse.getresponse()
                self.assertEqual(sse_response.status, 200)
                self.assertEqual(sse_response.readline().strip(), b"event: telemetry")
                self.assertTrue(sse_response.readline().startswith(b"data: {") )
                sse.close()

                mjpeg = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
                mjpeg.request("GET", "/api/v1/video/mjpeg")
                mjpeg_response = mjpeg.getresponse()
                self.assertEqual(mjpeg_response.status, 200)
                service.offer_preview(np.zeros((20, 20, 3), dtype=np.uint8))
                self.assertEqual(mjpeg_response.readline().strip(), b"--frame")
                self.assertEqual(mjpeg_response.readline().strip(), b"Content-Type: image/jpeg")
                length_line = mjpeg_response.readline().decode("ascii").strip()
                self.assertEqual(length_line, "Content-Length: 4")
                self.assertEqual(mjpeg_response.readline(), b"\r\n")
                self.assertEqual(mjpeg_response.read(4), b"\xff\xd8\xff\xd9")
                mjpeg.close()
            finally:
                service.close()

    def test_http_rejects_client_outside_allowlist(self):
        with tempfile.TemporaryDirectory() as directory:
            dashboard = Path(directory) / "dashboard.html"
            dashboard.write_text("test", encoding="utf-8")
            config = TelemetryWebConfig(
                enabled=True,
                bind="127.0.0.1",
                port=0,
                allowed_subnets=("192.0.2.0/24",),
                preview=PreviewWebConfig(enabled=False),
            )
            service = TelemetryWebService(config, dashboard_path=dashboard)
            service.start()
            port = service._server.server_address[1]
            try:
                with self.assertRaises(urllib.error.HTTPError) as raised:
                    urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=2)
                self.assertEqual(raised.exception.code, 403)
                self.assertEqual(service.log_stats()["web_denied_count"], 1)
            finally:
                service.close()


if __name__ == "__main__":
    unittest.main()
