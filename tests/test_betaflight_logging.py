import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from vision_guidance.betaflight_msp import (
    AnalogTelemetry,
    AttitudeTelemetry,
    BetaflightTelemetry,
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


class BetaflightLoggingTest(unittest.TestCase):
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
            clipped_flags=(0, 0, 1, 0, 0, 0, 0, 0),
            slew_limited_flags=(0, 0, 1, 0, 0, 0, 0, 0),
        )

        row = runner._log_row(
            timestamp=10.2,
            elapsed_s=0.2,
            telemetry=telemetry,
            telemetry_error="",
            detector_stats={"detector_source": "csv", "detector_reject_reason": ""},
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
        self.assertEqual(row["rc_in_ch5"], 1800)
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


if __name__ == "__main__":
    unittest.main()
