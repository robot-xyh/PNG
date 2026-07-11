from __future__ import annotations

import argparse
import csv
import importlib
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision_guidance.airsim_adapter import AirSimDetectionConfig  # noqa: E402
from vision_guidance.attitude_buffer import AttitudeHistoryBuffer  # noqa: E402
from vision_guidance.betaflight_msp import BetaflightMSPAdapter, BetaflightTelemetry  # noqa: E402
from vision_guidance.flight_control import (  # noqa: E402
    BetaflightSafetyStateMachine,
    CommandWatchdog,
    GuidanceSetpoint,
    RcCommand,
    RcCommandMapper,
    RcMappingConfig,
    SafetyInputs,
    aux_range_enabled,
    guidance_eval_to_setpoint,
)
from vision_guidance.fusion import PureVisionGuidancePipeline, VisionGuidanceResult  # noqa: E402
from vision_guidance.geometry import camera_to_body_mount  # noqa: E402
from vision_guidance.types import AttitudeSample, CameraIntrinsics, FrameDetection  # noqa: E402
from vision_guidance.yolo_bytetrack_detector import YoloByteTrackDetector  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Betaflight MSP telemetry logging with supervised PNG RC candidates.")
    parser.add_argument("--config", default="config/betaflight.example.json")
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--log-dir", default="logs")
    parser.add_argument("--log-prefix", default="betaflight_log")
    parser.add_argument("--serial-port", default="", help="Override config serial.port.")
    parser.add_argument("--msp-baud", type=int, default=0, help="Override config serial.baud.")
    parser.add_argument("--control-mode", choices=("log_only", "msp_raw_rc"), default="log_only")
    parser.add_argument("--allow-control", action="store_true", help="Required before MSP_SET_RAW_RC is sent.")
    parser.add_argument("--detector-source", choices=("none", "csv", "yolo_bytetrack"), default="none")
    parser.add_argument("--detections-csv", default="", help="CSV with x1,y1,x2,y2 and optional exposure_ts,track_id,score.")
    parser.add_argument("--camera-device", type=int, default=0)
    parser.add_argument("--yolo-model", default="")
    parser.add_argument("--yolo-class-id", type=int, default=None)
    parser.add_argument("--yolo-conf", type=float, default=0.25)
    parser.add_argument("--yolo-iou", type=float, default=0.70)
    parser.add_argument("--yolo-imgsz", type=int, default=640)
    parser.add_argument("--yolo-device", default="")
    parser.add_argument("--yolo-tracker", default="bytetrack.yaml")
    parser.add_argument("--yolo-allow-untracked-fallback", action="store_true")
    return parser.parse_args()


class DetectionCsvSource:
    def __init__(self, path: str):
        csv_path = Path(path).expanduser()
        if not csv_path.exists():
            raise RuntimeError(f"detections CSV not found: {csv_path}")
        with csv_path.open("r", newline="") as stream:
            self.rows = list(csv.DictReader(stream))
        self.index = 0

    def detect(self, *, elapsed_s: float, timestamp: float, frame_id: int) -> tuple[FrameDetection | None, dict[str, Any]]:
        if self.index >= len(self.rows):
            return None, {"detector_source": "csv", "detector_reject_reason": "csv_exhausted"}
        row = self.rows[self.index]
        row_ts = _float_or_none(row.get("exposure_ts"))
        if row_ts is not None and row_ts > elapsed_s:
            return None, {"detector_source": "csv", "detector_reject_reason": "csv_waiting"}
        self.index += 1
        detection = FrameDetection(
            frame_id=int(float(row.get("frame_id") or frame_id)),
            exposure_ts=float(timestamp),
            bbox_xyxy=(
                float(row["x1"]),
                float(row["y1"]),
                float(row["x2"]),
                float(row["y2"]),
            ),
            track_id=int(float(row.get("track_id") or 1)),
            score=float(row.get("score") or 1.0),
        )
        return detection, {"detector_source": "csv", "detector_reject_reason": ""}


class OpenCvYoloSource:
    def __init__(self, args: argparse.Namespace):
        self.cv2 = importlib.import_module("cv2")
        self.capture = self.cv2.VideoCapture(int(args.camera_device))
        if not self.capture.isOpened():
            raise RuntimeError(f"failed to open camera device {args.camera_device}")
        self.detector = YoloByteTrackDetector(
            model_path=str(args.yolo_model),
            class_id=args.yolo_class_id,
            conf=float(args.yolo_conf),
            iou=float(args.yolo_iou),
            imgsz=int(args.yolo_imgsz),
            device=str(args.yolo_device or ""),
            tracker=str(args.yolo_tracker or "bytetrack.yaml"),
            allow_untracked_fallback=bool(args.yolo_allow_untracked_fallback),
            image_reader=self._read_image,
            cv2_module=self.cv2,
        )
        self.config = AirSimDetectionConfig(camera_name="opencv", vehicle_name="Betaflight")

    def close(self) -> None:
        self.capture.release()

    def detect(self, *, timestamp: float, frame_id: int, active_track_id: int | None) -> tuple[FrameDetection | None, dict[str, Any]]:
        frame = self.detector.detect(
            client=None,
            config=self.config,
            frame_id=frame_id,
            exposure_ts=timestamp,
            active_track_id=active_track_id,
        )
        return frame.frame_detection, frame.stats

    def _read_image(self, _client: Any, _config: AirSimDetectionConfig):
        ok, image = self.capture.read()
        if not ok:
            return None
        return image


def main() -> None:
    args = parse_args()
    config = _load_config(args.config)
    serial_cfg = dict(config.get("serial", {}))
    port = args.serial_port or str(serial_cfg.get("port", ""))
    if not port:
        raise RuntimeError("serial.port is required in config or via --serial-port")
    baudrate = int(args.msp_baud or serial_cfg.get("baud", 115200))
    timeout_s = float(serial_cfg.get("timeout_s", 0.2))

    rc_mapper = RcCommandMapper(_rc_mapping_config(config))
    safety_cfg = dict(config.get("safety", {}))
    watchdog = CommandWatchdog(float(safety_cfg.get("watchdog_timeout_s", 0.25)))
    safety = BetaflightSafetyStateMachine()
    adapter = BetaflightMSPAdapter(port, baudrate, timeout_s=timeout_s)
    adapter.open()
    fc_identity = _read_fc_identity(adapter)

    detection_source = _create_detection_source(args)
    intrinsics = _camera_intrinsics(config)
    attitude_buffer = AttitudeHistoryBuffer(duration_s=float(config.get("attitude_buffer_s", 2.0)))
    pipeline = PureVisionGuidancePipeline(
        intrinsics=intrinsics,
        R_BC=_camera_mount(config),
        attitude_buffer=attitude_buffer,
    )

    log_path = _log_path(args.log_dir, args.log_prefix)
    fields = _log_fields(rc_mapper.config.channel_count)
    meta_path = _meta_path(log_path)
    _write_run_meta(
        meta_path,
        args=args,
        config=config,
        log_path=log_path,
        fields=fields,
        fc_identity=fc_identity,
    )
    start = time.monotonic()
    frame_id = 0
    last_telemetry_s: float | None = None
    last_attitude_s: float | None = None

    print(f"Logging Betaflight MSP telemetry to: {log_path}")
    print(f"Control mode: {args.control_mode}; allow_control={int(args.allow_control)}")
    try:
        with log_path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            while time.monotonic() - start < float(args.duration_s):
                loop_start = time.monotonic()
                elapsed = loop_start - start
                frame_id += 1
                telemetry, telemetry_error = _read_telemetry(adapter)
                if telemetry is not None:
                    last_telemetry_s = telemetry.timestamp
                    if telemetry.attitude is not None:
                        attitude_buffer.push(AttitudeSample(timestamp=loop_start, R_IB=telemetry.attitude.R_IB))
                        last_attitude_s = loop_start

                detection, detector_stats = _read_detection(
                    detection_source,
                    args,
                    elapsed_s=elapsed,
                    timestamp=loop_start,
                    frame_id=frame_id,
                    active_track_id=pipeline.active_track_id,
                )
                result = _process_detection(pipeline, detection)
                guidance = None if result is None else result.guidance
                if guidance is not None and guidance.valid:
                    watchdog.kick(loop_start)
                setpoint = _guidance_setpoint(config, guidance, loop_start)

                telemetry_age_s = None if last_telemetry_s is None else max(0.0, loop_start - last_telemetry_s)
                attitude_age_s = None if last_attitude_s is None else max(0.0, loop_start - last_attitude_s)
                watchdog_age_s = watchdog.age_s(loop_start)
                telemetry_fresh = telemetry_age_s is not None and telemetry_age_s <= float(safety_cfg.get("telemetry_timeout_s", 0.5))
                attitude_synced = attitude_age_s is not None and attitude_age_s <= float(safety_cfg.get("attitude_timeout_s", 0.5))
                voltage_ok = _voltage_ok(telemetry, safety_cfg)
                aux_enabled = _aux_enabled(telemetry, safety_cfg)
                watchdog_ok = watchdog.fresh(loop_start)
                control_requested = args.control_mode == "msp_raw_rc"
                allow_control = bool(args.allow_control)
                decision = safety.update(
                    SafetyInputs(
                        control_requested=control_requested,
                        allow_control=allow_control,
                        target_valid=bool(guidance is not None and guidance.valid),
                        aux_enabled=aux_enabled,
                        telemetry_fresh=telemetry_fresh,
                        attitude_synced=attitude_synced,
                        voltage_ok=voltage_ok,
                        watchdog_ok=watchdog_ok,
                    )
                )
                rc_command = rc_mapper.map_setpoint(setpoint, active=decision.command_active)
                send_error = _maybe_send_rc(adapter, args, rc_command, decision.command_active, config)
                writer.writerow(
                    _log_row(
                        timestamp=loop_start,
                        elapsed_s=elapsed,
                        telemetry=telemetry,
                        telemetry_error=telemetry_error,
                        detector_stats=detector_stats,
                        detection=detection,
                        result=result,
                        setpoint=setpoint,
                        rc_command=rc_command,
                        safety_state=str(decision.state.value),
                        safety_reason=decision.reason,
                        send_error=send_error,
                        telemetry_age_s=telemetry_age_s,
                        attitude_age_s=attitude_age_s,
                        watchdog_age_s=watchdog_age_s,
                        telemetry_fresh=telemetry_fresh,
                        attitude_synced=attitude_synced,
                        watchdog_ok=watchdog_ok,
                        voltage_ok=voltage_ok,
                        aux_enabled=aux_enabled,
                        control_requested=control_requested,
                        allow_control=allow_control,
                        intrinsics=intrinsics,
                        channel_count=rc_mapper.config.channel_count,
                    )
                )

                sleep_s = max(0.0, (1.0 / max(1.0, float(args.rate_hz))) - (time.monotonic() - loop_start))
                time.sleep(sleep_s)
    finally:
        if args.control_mode == "msp_raw_rc" and args.allow_control:
            _send_neutral_stop(adapter, rc_mapper)
        close = getattr(detection_source, "close", None)
        if callable(close):
            close()
        adapter.close()


def _load_config(path: str) -> dict[str, Any]:
    config_path = Path(path).expanduser()
    with config_path.open("r") as stream:
        return json.load(stream)


def _rc_mapping_config(config: dict[str, Any]) -> RcMappingConfig:
    rc = dict(config.get("rc_mapping", {}))
    aux_values = {int(key): int(value) for key, value in dict(rc.get("aux_values_us", {})).items()}
    return RcMappingConfig(
        channel_map=str(rc.get("channel_map", "AETR1234")),
        channel_count=int(rc.get("channel_count", 8)),
        roll_rate_limit_deg_s=float(rc.get("roll_rate_limit_deg_s", 120.0)),
        pitch_rate_limit_deg_s=float(rc.get("pitch_rate_limit_deg_s", 120.0)),
        yaw_rate_limit_deg_s=float(rc.get("yaw_rate_limit_deg_s", 90.0)),
        thrust_min=float(rc.get("thrust_min", 0.0)),
        thrust_hover=float(rc.get("thrust_hover", 0.5)),
        thrust_max=float(rc.get("thrust_max", 1.0)),
        throttle_min_us=int(rc.get("throttle_min_us", 1000)),
        throttle_hover_us=int(rc.get("throttle_hover_us", 1500)),
        throttle_max_us=int(rc.get("throttle_max_us", 2000)),
        neutral_throttle_us=int(rc.get("neutral_throttle_us", 1000)),
        max_delta_us_per_s=float(rc.get("max_delta_us_per_s", 0.0)),
        aux_values_us=aux_values,
    )


def _camera_intrinsics(config: dict[str, Any]) -> CameraIntrinsics:
    camera = dict(config.get("camera", {}))
    width = int(camera.get("width", 640))
    height = int(camera.get("height", 480))
    return CameraIntrinsics(
        fx=float(camera.get("fx", 500.0)),
        fy=float(camera.get("fy", 500.0)),
        cx=float(camera.get("cx", width / 2.0)),
        cy=float(camera.get("cy", height / 2.0)),
        width=width,
        height=height,
    )


def _camera_mount(config: dict[str, Any]) -> np.ndarray:
    camera = dict(config.get("camera", {}))
    if "R_BC" in camera:
        matrix = np.asarray(camera["R_BC"], dtype=float)
        if matrix.shape != (3, 3):
            raise ValueError("camera.R_BC must be 3x3")
        return matrix
    return camera_to_body_mount(float(camera.get("pitch_up_deg", 90.0)))


def _create_detection_source(args: argparse.Namespace):
    if args.detector_source == "none":
        return None
    if args.detector_source == "csv":
        if not args.detections_csv:
            raise RuntimeError("--detections-csv is required for --detector-source csv")
        return DetectionCsvSource(args.detections_csv)
    if args.detector_source == "yolo_bytetrack":
        return OpenCvYoloSource(args)
    raise ValueError(f"unsupported detector source: {args.detector_source}")


def _read_fc_identity(adapter: BetaflightMSPAdapter) -> dict[str, Any]:
    identity: dict[str, Any] = {}
    try:
        api = adapter.read_api_version()
        identity["api_protocol_version"] = api.protocol_version
        identity["api_major"] = api.api_major
        identity["api_minor"] = api.api_minor
        identity["fc_variant"] = adapter.read_fc_variant()
        version = adapter.read_fc_version()
        identity["fc_version_major"] = version.major
        identity["fc_version_minor"] = version.minor
        identity["fc_version_patch"] = version.patch
    except Exception as exc:
        identity["fc_identity_error"] = str(exc)
    return identity


def _read_telemetry(adapter: BetaflightMSPAdapter) -> tuple[BetaflightTelemetry | None, str]:
    try:
        return adapter.read_telemetry(), ""
    except Exception as exc:
        return None, str(exc)


def _read_detection(
    detection_source,
    args: argparse.Namespace,
    *,
    elapsed_s: float,
    timestamp: float,
    frame_id: int,
    active_track_id: int | None,
) -> tuple[FrameDetection | None, dict[str, Any]]:
    if detection_source is None:
        return None, {"detector_source": "none", "detector_reject_reason": "detector_disabled"}
    if args.detector_source == "csv":
        return detection_source.detect(elapsed_s=elapsed_s, timestamp=timestamp, frame_id=frame_id)
    return detection_source.detect(timestamp=timestamp, frame_id=frame_id, active_track_id=active_track_id)


def _process_detection(pipeline: PureVisionGuidancePipeline, detection: FrameDetection | None) -> VisionGuidanceResult | None:
    if detection is None:
        return None
    return pipeline.process(detection)


def _guidance_setpoint(config: dict[str, Any], guidance, timestamp: float) -> GuidanceSetpoint:
    if guidance is None:
        return GuidanceSetpoint(timestamp=timestamp, valid=False, source="guidance_eval", reject_reason="guidance_missing")
    command_cfg = dict(config.get("guidance_command", {}))
    setpoint = guidance_eval_to_setpoint(
        guidance,
        rate_gain_matrix=command_cfg.get("rate_gain_matrix", [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        hover_thrust=float(command_cfg.get("hover_thrust", config.get("rc_mapping", {}).get("thrust_hover", 0.5))),
        yaw_rate_deg_s=float(command_cfg.get("yaw_rate_bias_deg_s", 0.0)),
    )
    if setpoint.timestamp == 0.0:
        return GuidanceSetpoint(timestamp=timestamp, valid=setpoint.valid, source=setpoint.source, reject_reason=setpoint.reject_reason)
    return setpoint


def _voltage_ok(telemetry: BetaflightTelemetry | None, safety_cfg: dict[str, Any]) -> bool:
    threshold = float(safety_cfg.get("min_vbat_v", 0.0))
    if threshold <= 0.0:
        return True
    return bool(telemetry is not None and telemetry.analog is not None and telemetry.analog.vbat_v >= threshold)


def _aux_enabled(telemetry: BetaflightTelemetry | None, safety_cfg: dict[str, Any]) -> bool:
    if not bool(safety_cfg.get("require_aux_enable", True)):
        return True
    if telemetry is None or not telemetry.rc_channels:
        return False
    aux = dict(safety_cfg.get("aux_enable", {}))
    return aux_range_enabled(
        telemetry.rc_channels,
        channel_index=int(aux.get("channel_index", 5)),
        min_us=int(aux.get("min_us", 1700)),
        max_us=int(aux.get("max_us", 2100)),
    )


def _maybe_send_rc(
    adapter: BetaflightMSPAdapter,
    args: argparse.Namespace,
    command: RcCommand,
    command_active: bool,
    config: dict[str, Any],
) -> str:
    if args.control_mode != "msp_raw_rc" or not args.allow_control:
        return ""
    if not command_active and not bool(config.get("safety", {}).get("send_neutral_when_inactive", True)):
        return ""
    try:
        adapter.send_raw_rc(command)
        return ""
    except Exception as exc:
        return str(exc)


def _send_neutral_stop(adapter: BetaflightMSPAdapter, mapper: RcCommandMapper) -> None:
    for _ in range(5):
        try:
            adapter.send_raw_rc(mapper.neutral(time.monotonic(), "stop"))
        except Exception:
            break
        time.sleep(0.03)


def _log_path(log_dir: str, prefix: str) -> Path:
    directory = Path(log_dir)
    directory.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return directory / f"{prefix}_{stamp}.csv"


def _meta_path(log_path: Path) -> Path:
    return log_path.with_name(f"{log_path.stem}_meta.json")


def _write_run_meta(
    path: Path,
    *,
    args: argparse.Namespace,
    config: dict[str, Any],
    log_path: Path,
    fields: list[str],
    fc_identity: dict[str, Any],
) -> None:
    meta = {
        "created_unix_s": time.time(),
        "created_local": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "log_csv": str(log_path),
        "args": vars(args),
        "config": config,
        "fields": fields,
        "control_mode": args.control_mode,
        "allow_control": bool(args.allow_control),
        "fc_identity": fc_identity,
    }
    with path.open("w") as stream:
        json.dump(meta, stream, indent=2, sort_keys=True)
        stream.write("\n")


def _log_fields(channel_count: int) -> list[str]:
    fields = [
        "timestamp",
        "elapsed_s",
        "safety_state",
        "safety_reason",
        "control_requested",
        "allow_control",
        "telemetry_fresh",
        "attitude_synced",
        "watchdog_ok",
        "voltage_ok",
        "aux_enabled",
        "telemetry_age_s",
        "attitude_age_s",
        "watchdog_age_s",
        "telemetry_error",
        "send_error",
        "cycle_time_us",
        "i2c_error_count",
        "sensor_flags",
        "mode_flags",
        "profile",
        "vbat_v",
        "mah_drawn",
        "rssi",
        "amperage_a",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "rc_in_count",
        "detector_source",
        "detector_reject_reason",
        "frame_id",
        "detection_exposure_ts",
        "detection_score",
        "bbox_x1",
        "bbox_y1",
        "bbox_x2",
        "bbox_y2",
        "bbox_width",
        "bbox_height",
        "bbox_area",
        "bbox_area_ratio",
        "bbox_clip_left",
        "bbox_clip_top",
        "bbox_clip_right",
        "bbox_clip_bottom",
        "bbox_clipped",
        "track_id",
        "los_valid",
        "los_reject_reason",
        "los_quality",
        "los_innovation_norm",
        "lambda_I_x",
        "lambda_I_y",
        "lambda_I_z",
        "lambda_dot_I_x",
        "lambda_dot_I_y",
        "lambda_dot_I_z",
        "omega_los_x",
        "omega_los_y",
        "omega_los_z",
        "ttc_valid",
        "ttc_reject_reason",
        "ttc_quality",
        "ttc_s",
        "ttc_area_filtered",
        "ttc_area_dot_filtered",
        "guidance_valid",
        "guidance_reject_reason",
        "guidance_quality",
        "g_eval_x",
        "g_eval_y",
        "g_eval_z",
        "sp_valid",
        "sp_source",
        "sp_reject_reason",
        "sp_roll_rate_deg_s",
        "sp_pitch_rate_deg_s",
        "sp_yaw_rate_deg_s",
        "sp_thrust",
        "rc_active",
        "rc_reason",
    ]
    fields.extend(f"rc_in_ch{i}" for i in range(1, channel_count + 1))
    fields.extend(f"rc_raw_ch{i}" for i in range(1, channel_count + 1))
    fields.extend(f"rc_ch{i}" for i in range(1, channel_count + 1))
    fields.extend(f"rc_clipped_ch{i}" for i in range(1, channel_count + 1))
    fields.extend(f"rc_slew_limited_ch{i}" for i in range(1, channel_count + 1))
    return fields


def _log_row(
    *,
    timestamp: float,
    elapsed_s: float,
    telemetry: BetaflightTelemetry | None,
    telemetry_error: str,
    detector_stats: dict[str, Any],
    detection: FrameDetection | None,
    result: VisionGuidanceResult | None,
    setpoint: GuidanceSetpoint,
    rc_command: RcCommand,
    safety_state: str,
    safety_reason: str,
    send_error: str,
    telemetry_age_s: float | None,
    attitude_age_s: float | None,
    watchdog_age_s: float | None,
    telemetry_fresh: bool,
    attitude_synced: bool,
    watchdog_ok: bool,
    voltage_ok: bool,
    aux_enabled: bool,
    control_requested: bool,
    allow_control: bool,
    intrinsics: CameraIntrinsics,
    channel_count: int,
) -> dict[str, Any]:
    guidance = None if result is None else result.guidance
    los = None if result is None else result.los
    ttc = None if result is None else result.ttc
    status = None if telemetry is None else telemetry.status
    analog = None if telemetry is None else telemetry.analog
    attitude = None if telemetry is None else telemetry.attitude
    clip_flags = (
        {"left": "", "top": "", "right": "", "bottom": ""}
        if detection is None
        else detection.clip_flags(intrinsics.width, intrinsics.height)
    )
    bbox_area_ratio = "" if detection is None else detection.area / max(1.0, float(intrinsics.width * intrinsics.height))
    row: dict[str, Any] = {
        "timestamp": f"{timestamp:.6f}",
        "elapsed_s": f"{elapsed_s:.6f}",
        "safety_state": safety_state,
        "safety_reason": safety_reason,
        "control_requested": int(control_requested),
        "allow_control": int(allow_control),
        "telemetry_fresh": int(telemetry_fresh),
        "attitude_synced": int(attitude_synced),
        "watchdog_ok": int(watchdog_ok),
        "voltage_ok": int(voltage_ok),
        "aux_enabled": int(aux_enabled),
        "telemetry_age_s": _format_optional_float(telemetry_age_s, precision=6),
        "attitude_age_s": _format_optional_float(attitude_age_s, precision=6),
        "watchdog_age_s": _format_optional_float(watchdog_age_s, precision=6),
        "telemetry_error": telemetry_error,
        "send_error": send_error,
        "cycle_time_us": "" if status is None else status.cycle_time_us,
        "i2c_error_count": "" if status is None else status.i2c_error_count,
        "sensor_flags": "" if status is None else status.sensor_flags,
        "mode_flags": "" if status is None else status.mode_flags,
        "profile": "" if status is None or status.profile is None else status.profile,
        "vbat_v": "" if analog is None else f"{analog.vbat_v:.2f}",
        "mah_drawn": "" if analog is None or analog.mah_drawn is None else analog.mah_drawn,
        "rssi": "" if analog is None or analog.rssi is None else analog.rssi,
        "amperage_a": "" if analog is None or analog.amperage_a is None else f"{analog.amperage_a:.3f}",
        "roll_deg": "" if attitude is None else f"{attitude.roll_deg:.3f}",
        "pitch_deg": "" if attitude is None else f"{attitude.pitch_deg:.3f}",
        "yaw_deg": "" if attitude is None else f"{attitude.yaw_deg:.3f}",
        "rc_in_count": "" if telemetry is None else len(telemetry.rc_channels),
        "detector_source": detector_stats.get("detector_source", ""),
        "detector_reject_reason": detector_stats.get("detector_reject_reason", ""),
        "frame_id": "" if detection is None else detection.frame_id,
        "detection_exposure_ts": "" if detection is None else f"{detection.exposure_ts:.6f}",
        "detection_score": "" if detection is None else f"{detection.score:.6f}",
        "bbox_x1": "" if detection is None else f"{detection.bbox_xyxy[0]:.3f}",
        "bbox_y1": "" if detection is None else f"{detection.bbox_xyxy[1]:.3f}",
        "bbox_x2": "" if detection is None else f"{detection.bbox_xyxy[2]:.3f}",
        "bbox_y2": "" if detection is None else f"{detection.bbox_xyxy[3]:.3f}",
        "bbox_width": "" if detection is None else f"{detection.width:.3f}",
        "bbox_height": "" if detection is None else f"{detection.height:.3f}",
        "bbox_area": "" if detection is None else f"{detection.area:.3f}",
        "bbox_area_ratio": "" if detection is None else f"{bbox_area_ratio:.9f}",
        "bbox_clip_left": "" if detection is None else int(bool(clip_flags["left"])),
        "bbox_clip_top": "" if detection is None else int(bool(clip_flags["top"])),
        "bbox_clip_right": "" if detection is None else int(bool(clip_flags["right"])),
        "bbox_clip_bottom": "" if detection is None else int(bool(clip_flags["bottom"])),
        "bbox_clipped": "" if detection is None else int(detection.is_clipped(intrinsics.width, intrinsics.height)),
        "track_id": "" if detection is None else detection.track_id,
        "los_valid": "" if los is None else int(los.valid),
        "los_reject_reason": "" if los is None else los.reject_reason or "",
        "los_quality": "" if los is None else f"{los.quality:.6f}",
        "los_innovation_norm": "" if los is None else f"{los.innovation_norm:.9f}",
        "lambda_I_x": _vector_field(los.lambda_I if los is not None else None, 0),
        "lambda_I_y": _vector_field(los.lambda_I if los is not None else None, 1),
        "lambda_I_z": _vector_field(los.lambda_I if los is not None else None, 2),
        "lambda_dot_I_x": _vector_field(los.lambda_dot_I if los is not None else None, 0),
        "lambda_dot_I_y": _vector_field(los.lambda_dot_I if los is not None else None, 1),
        "lambda_dot_I_z": _vector_field(los.lambda_dot_I if los is not None else None, 2),
        "omega_los_x": _vector_field(los.omega_los if los is not None else None, 0),
        "omega_los_y": _vector_field(los.omega_los if los is not None else None, 1),
        "omega_los_z": _vector_field(los.omega_los if los is not None else None, 2),
        "ttc_valid": "" if ttc is None else int(ttc.valid),
        "ttc_reject_reason": "" if ttc is None else ttc.reject_reason or "",
        "ttc_quality": "" if ttc is None else f"{ttc.quality:.6f}",
        "ttc_s": "" if ttc is None or ttc.ttc is None else f"{ttc.ttc:.9f}",
        "ttc_area_filtered": "" if ttc is None else f"{ttc.area_filtered:.9f}",
        "ttc_area_dot_filtered": "" if ttc is None else f"{ttc.area_dot_filtered:.9f}",
        "guidance_valid": "" if guidance is None else int(guidance.valid),
        "guidance_reject_reason": "" if guidance is None else guidance.reject_reason or "",
        "guidance_quality": "" if guidance is None else f"{guidance.quality:.6f}",
        "g_eval_x": "" if guidance is None else f"{guidance.g_eval[0]:.9f}",
        "g_eval_y": "" if guidance is None else f"{guidance.g_eval[1]:.9f}",
        "g_eval_z": "" if guidance is None else f"{guidance.g_eval[2]:.9f}",
        "sp_valid": int(setpoint.valid),
        "sp_source": setpoint.source,
        "sp_reject_reason": setpoint.reject_reason,
        "sp_roll_rate_deg_s": f"{setpoint.roll_rate_deg_s:.6f}",
        "sp_pitch_rate_deg_s": f"{setpoint.pitch_rate_deg_s:.6f}",
        "sp_yaw_rate_deg_s": f"{setpoint.yaw_rate_deg_s:.6f}",
        "sp_thrust": f"{setpoint.thrust:.6f}",
        "rc_active": int(rc_command.active),
        "rc_reason": rc_command.reason,
    }
    rc_input = () if telemetry is None else telemetry.rc_channels
    row.update({f"rc_in_ch{i}": _sequence_field(rc_input, i - 1) for i in range(1, channel_count + 1)})
    row.update({f"rc_raw_ch{i}": _sequence_field(rc_command.raw_channels, i - 1) for i in range(1, channel_count + 1)})
    row.update({f"rc_ch{i}": value for i, value in enumerate(rc_command.channels, start=1)})
    row.update({f"rc_clipped_ch{i}": _sequence_field(rc_command.clipped_flags, i - 1) for i in range(1, channel_count + 1)})
    row.update({f"rc_slew_limited_ch{i}": _sequence_field(rc_command.slew_limited_flags, i - 1) for i in range(1, channel_count + 1)})
    return row


def _format_optional_float(value: float | None, *, precision: int) -> str:
    if value is None:
        return ""
    return f"{float(value):.{precision}f}"


def _vector_field(vector: Any, index: int) -> str:
    if vector is None:
        return ""
    value = np.asarray(vector, dtype=float)
    if value.shape != (3,) or not np.all(np.isfinite(value)):
        return ""
    return f"{float(value[index]):.9f}"


def _sequence_field(values: Any, index: int) -> Any:
    if values is None or index >= len(values):
        return ""
    return values[index]


def _float_or_none(value: Any) -> float | None:
    if value in (None, ""):
        return None
    return float(value)


if __name__ == "__main__":
    main()
