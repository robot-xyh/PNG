from __future__ import annotations

import copy
import importlib
import ipaddress
import json
import math
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Mapping
from urllib.parse import parse_qs, urlsplit


WEB_SCHEMA_VERSION = 11
DEFAULT_DASHBOARD_PATH = Path(__file__).with_name("web") / "betaflight_telemetry.html"
VISION_MEASUREMENT_KEYS = (
    "camera_ok",
    "camera_read_ms",
    "camera_size",
    "camera_failed_frames",
    "detector_source",
    "detector_best_score",
    "detector_counts",
    "track_id",
    "tracker_state",
    "tracker_confirmed",
    "tracker_hits",
    "tracker_association_stage",
    "tracker_match_iou",
    "target_selector_reason",
    "score",
    "bbox_xyxy",
    "bbox_area_ratio",
    "rknn_ms",
    "tracker_update_ms",
    "tracker_fps",
    "result_age_ms",
    "attitude_offset_ms",
)


@dataclass(frozen=True)
class PreviewWebConfig:
    enabled: bool = True
    max_fps: float = 10.0
    jpeg_quality: int = 70
    max_clients: int = 2

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "PreviewWebConfig":
        config = cls(
            enabled=bool(values.get("enabled", True)),
            max_fps=float(values.get("max_fps", 10.0)),
            jpeg_quality=int(values.get("jpeg_quality", 70)),
            max_clients=int(values.get("max_clients", 2)),
        )
        if not 1.0 <= config.max_fps <= 30.0:
            raise ValueError("telemetry_web.preview.max_fps must be within 1-30")
        if not 20 <= config.jpeg_quality <= 95:
            raise ValueError("telemetry_web.preview.jpeg_quality must be within 20-95")
        if not 1 <= config.max_clients <= 8:
            raise ValueError("telemetry_web.preview.max_clients must be within 1-8")
        return config


@dataclass(frozen=True)
class TelemetryWebConfig:
    enabled: bool = False
    bind: str = "127.0.0.1"
    port: int = 8080
    allowed_subnets: tuple[str, ...] = ("127.0.0.0/8",)
    sample_hz: float = 5.0
    history_s: float = 60.0
    stale_after_s: float = 1.0
    max_sse_clients: int = 4
    dashboard_path: str = ""
    preview: PreviewWebConfig = PreviewWebConfig()

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "TelemetryWebConfig":
        raw_networks = values.get("allowed_subnets", ["127.0.0.0/8"])
        if isinstance(raw_networks, str):
            raw_networks = [raw_networks]
        config = cls(
            enabled=bool(values.get("enabled", False)),
            bind=str(values.get("bind", "127.0.0.1")).strip(),
            port=int(values.get("port", 8080)),
            allowed_subnets=tuple(str(value).strip() for value in raw_networks),
            sample_hz=float(values.get("sample_hz", 5.0)),
            history_s=float(values.get("history_s", 60.0)),
            stale_after_s=float(values.get("stale_after_s", 1.0)),
            max_sse_clients=int(values.get("max_sse_clients", 4)),
            dashboard_path=str(values.get("dashboard_path", "") or ""),
            preview=PreviewWebConfig.from_mapping(dict(values.get("preview", {}))),
        )
        if not config.bind:
            raise ValueError("telemetry_web.bind must not be empty")
        if not 1 <= config.port <= 65535:
            raise ValueError("telemetry_web.port must be within 1-65535")
        if not 0.2 <= config.sample_hz <= 20.0:
            raise ValueError("telemetry_web.sample_hz must be within 0.2-20")
        if not 1.0 <= config.history_s <= 3600.0:
            raise ValueError("telemetry_web.history_s must be within 1-3600")
        if config.stale_after_s <= 0.0:
            raise ValueError("telemetry_web.stale_after_s must be positive")
        if not 1 <= config.max_sse_clients <= 32:
            raise ValueError("telemetry_web.max_sse_clients must be within 1-32")
        if not config.allowed_subnets:
            raise ValueError("telemetry_web.allowed_subnets must not be empty")
        for network in config.allowed_subnets:
            ipaddress.ip_network(network, strict=False)
        return config

    @property
    def history_capacity(self) -> int:
        return max(2, int(math.ceil(self.history_s * self.sample_hz)) + 1)


def telemetry_payload_from_log_row(
    row: Mapping[str, Any],
    *,
    channel_count: int,
    channel_map: str = "AETR1234",
) -> dict[str, Any]:
    """Build the typed, stable browser API payload from one CSV-compatible row."""

    input_channels = _channels(row, "rc_in_ch", channel_count)
    physical_channels = _reorder_rc_input(input_channels, channel_map=channel_map)

    commands: dict[str, Any] = {}
    for name in ("status", "raw_imu", "motor", "rc", "attitude", "analog", "set_raw_rc"):
        prefix = f"msp_cmd_{name}"
        commands[name] = {
            "attempt_count": _integer(row.get(f"{prefix}_attempt_count")),
            "success_count": _integer(row.get(f"{prefix}_success_count")),
            "error_count": _integer(row.get(f"{prefix}_error_count")),
            "last_rtt_ms": _number(row.get(f"{prefix}_last_rtt_ms")),
            "max_rtt_ms": _number(row.get(f"{prefix}_max_rtt_ms")),
            "last_success_age_s": _number(row.get(f"{prefix}_last_success_age_s")),
            "last_error": _text(row.get(f"{prefix}_last_error")),
        }

    payload = {
        "run": {
            "elapsed_s": _number(row.get("elapsed_s")),
            "loop_period_s": _number(row.get("loop_period_s")),
        },
        "safety": {
            "state": _text(row.get("safety_state")),
            "reason": _text(row.get("safety_reason")),
            "control_requested": _boolean(row.get("control_requested")),
            "allow_control": _boolean(row.get("allow_control")),
            "armed": _boolean(row.get("armed")),
            "override_available": _boolean(row.get("msp_override_available")),
            "override_active": _boolean(row.get("msp_override_active")),
            "prefill_ready": _boolean(row.get("msp_prefill_ready")),
            "msp_response_fresh": _boolean(row.get("msp_set_raw_rc_ack_fresh")),
            "target_valid": _boolean(row.get("sp_valid")),
            "publish_mode": _text(row.get("msp_publish_mode")),
            "telemetry_fresh": _boolean(row.get("telemetry_fresh")),
            "attitude_synced": _boolean(row.get("attitude_synced")),
            "motor_interlock": {
                "ok": _boolean(row.get("motor_interlock_ok")),
                "reason": _text(row.get("motor_interlock_reason")),
                "latched": _boolean(row.get("motor_interlock_latched")),
                "output_max_us": _number(row.get("motor_interlock_output_max_us")),
                "output_spread_us": _number(row.get("motor_interlock_output_spread_us")),
            },
            "takeover_duration_interlock": {
                "ok": _boolean(row.get("takeover_duration_interlock_ok")),
                "reason": _text(row.get("takeover_duration_interlock_reason")),
                "latched": _boolean(row.get("takeover_duration_interlock_latched")),
                "active_duration_s": _number(row.get("takeover_duration_s")),
                "max_duration_s": _number(row.get("takeover_duration_limit_s")),
            },
            "physical_rc_fresh": _boolean(row.get("physical_rc_fresh")),
            "watchdog_ok": _boolean(row.get("watchdog_ok")),
            "voltage_ok": _boolean(row.get("voltage_ok")),
            "aux_enabled": _boolean(row.get("aux_enabled")),
            "snapshot_approved": _boolean(row.get("control_snapshot_approved")),
            "authorization_reason": _text(row.get("control_authorization_reason")),
        },
        "flight_controller": {
            "attitude_deg": _vector(row, ("roll_deg", "pitch_deg", "yaw_deg")),
            "gyro_deg_s": _vector(
                row,
                ("gyro_roll_deg_s", "gyro_pitch_deg_s", "gyro_yaw_deg_s"),
            ),
            "gyro_msp_raw": _vector(
                row,
                ("gyro_msp_raw_x", "gyro_msp_raw_y", "gyro_msp_raw_z"),
            ),
            "motor_outputs": _channels(row, "motor_output_ch", 8),
            "motor_output_count": _integer(row.get("motor_output_count")),
            "vbat_v": _number(row.get("vbat_v")),
            "amperage_a": _number(row.get("amperage_a")),
            "mah_drawn": _integer(row.get("mah_drawn")),
            "rssi": _integer(row.get("rssi")),
            "cycle_time_us": _integer(row.get("cycle_time_us")),
            "i2c_error_count": _integer(row.get("i2c_error_count")),
            "sensor_flags": _integer(row.get("sensor_flags")),
            "mode_flags": _integer(row.get("mode_flags")),
            "profile": _integer(row.get("profile")),
        },
        "msp": {
            "telemetry_age_s": _number(row.get("telemetry_age_s")),
            "attitude_age_s": _number(row.get("attitude_age_s")),
            "motor_age_s": _number(row.get("msp_motor_age_s")),
            "physical_rc_age_s": _number(row.get("physical_rc_age_s")),
            "request_count": _integer(row.get("msp_request_count")),
            "request_error_count": _integer(row.get("msp_request_error_count")),
            "tx_bytes": _integer(row.get("msp_tx_bytes")),
            "rx_bytes": _integer(row.get("msp_rx_bytes")),
            "worker_poll_error_count": _integer(row.get("msp_worker_poll_error_count")),
            "worker_send_error_count": _integer(row.get("msp_worker_send_error_count")),
            "set_raw_rc_attempt_count": _integer(row.get("msp_set_raw_rc_attempt_count")),
            "set_raw_rc_success_count": _integer(row.get("msp_set_raw_rc_success_count")),
            "transport_mode": _text(row.get("msp_transport_mode")),
            "set_raw_rc": {
                "write_attempt_count": _integer(row.get("msp_set_raw_rc_write_attempt_count")),
                "write_success_count": _integer(row.get("msp_set_raw_rc_write_success_count")),
                "write_error_count": _integer(row.get("msp_set_raw_rc_write_error_count")),
                "ack_count": _integer(row.get("msp_set_raw_rc_ack_count")),
                "ack_age_s": _number(row.get("msp_set_raw_rc_ack_age_s")),
                "ack_fresh": _boolean(row.get("msp_set_raw_rc_ack_fresh")),
                "pending_depth": _integer(row.get("msp_set_raw_rc_pending_depth")),
                "write_rate_hz": _number(row.get("msp_set_raw_rc_write_rate_hz")),
                "write_interval_s": _number(row.get("msp_set_raw_rc_write_interval_s")),
                "write_max_interval_s": _number(row.get("msp_set_raw_rc_write_max_interval_s")),
                "write_p50_interval_s": _number(row.get("msp_set_raw_rc_write_p50_interval_s")),
                "write_p95_interval_s": _number(row.get("msp_set_raw_rc_write_p95_interval_s")),
                "write_p99_interval_s": _number(row.get("msp_set_raw_rc_write_p99_interval_s")),
                "write_p999_interval_s": _number(row.get("msp_set_raw_rc_write_p999_interval_s")),
            },
            "parser": {
                "pending_telemetry_count": _integer(row.get("msp_async_pending_telemetry_count")),
                "discarded_bytes": _integer(row.get("msp_rx_discarded_bytes")),
                "checksum_error_count": _integer(row.get("msp_rx_checksum_error_count")),
                "parser_error_count": _integer(row.get("msp_rx_parser_error_count")),
                "rc_poll_suspended": _boolean(row.get("msp_rc_poll_suspended")),
                "override_release_hold_active": _boolean(
                    row.get("msp_override_release_hold_active")
                ),
            },
            "publish_deadline_miss_count": _integer(row.get("msp_publish_deadline_miss_count")),
            "last_publish_gates": {
                "output_enabled": _boolean(row.get("msp_last_publish_output_enabled")),
                "algorithm_authorized": _boolean(
                    row.get("msp_last_publish_algorithm_authorized")
                ),
                "override_active": _boolean(row.get("msp_last_publish_override_active")),
                "override_release_hold_active": _boolean(
                    row.get("msp_last_publish_override_release_hold_active")
                ),
                "prefill_ready": _boolean(row.get("msp_last_publish_prefill_ready")),
                "physical_rc_fresh": _boolean(
                    row.get("msp_last_publish_physical_rc_fresh")
                ),
                "command_fresh": _boolean(row.get("msp_last_publish_command_fresh")),
                "command_active": _boolean(row.get("msp_last_publish_command_active")),
                "command_reason": _text(row.get("msp_last_publish_command_reason")),
                "set_raw_rc_ack_fresh": _boolean(
                    row.get("msp_last_publish_set_raw_rc_ack_fresh")
                ),
            },
            "worker_error": _text(row.get("msp_worker_error")),
            "telemetry_error": _text(row.get("telemetry_error")),
            "send_error": _text(row.get("send_error")),
            "commands": commands,
        },
        "rc": {
            "input_us": input_channels,
            "input_order": _msp_rc_order(channel_count),
            "physical_us": physical_channels,
            "wire_order": channel_map,
            "raw_us": _channels(row, "rc_raw_ch", channel_count),
            "target_us": _channels(row, "rc_target_ch", channel_count),
            "mapped_us": _channels(row, "rc_ch", channel_count),
            "sent_us": _channels(row, "rc_sent_ch", channel_count),
            "active": _boolean(row.get("rc_active")),
            "reason": _text(row.get("rc_reason")),
            "requested_rate_deg_s": _vector(
                row,
                (
                    "map_requested_roll_rate_deg_s",
                    "map_requested_pitch_rate_deg_s",
                    "map_requested_yaw_rate_deg_s",
                ),
            ),
            "limited_rate_deg_s": _vector(
                row,
                (
                    "map_limited_roll_rate_deg_s",
                    "map_limited_pitch_rate_deg_s",
                    "map_limited_yaw_rate_deg_s",
                ),
            ),
            "throttle_handover": {
                "source_us": _integer(row.get("throttle_handover_source_us")),
                "target_us": _integer(row.get("throttle_handover_target_us")),
                "output_us": _integer(row.get("throttle_handover_output_us")),
                "alpha": _number(row.get("throttle_handover_alpha")),
                "active": _boolean(row.get("throttle_handover_active")),
            },
        },
        "vision": {
            "new_result": _boolean(row.get("perception_new_result")),
            "display_held": False,
            "camera_ok": _boolean(row.get("camera_frame_ok")),
            "camera_read_ms": _number(row.get("camera_read_ms")),
            "camera_size": [
                _integer(row.get("camera_output_width")),
                _integer(row.get("camera_output_height")),
            ],
            "camera_failed_frames": _integer(row.get("camera_failed_frames")),
            "detector_source": _text(row.get("detector_source")),
            "detector_reason": _text(row.get("detector_reject_reason")),
            "detector_best_score": _number(row.get("detector_best_score")),
            "detector_counts": {
                "raw": _integer(row.get("detector_raw_count")),
                "class_filtered": _integer(row.get("detector_class_filtered_count")),
                "high": _integer(row.get("tracker_high_count")),
                "low": _integer(row.get("tracker_low_count")),
                "tracker_output": _integer(row.get("tracker_output_count")),
            },
            "track_id": _integer(row.get("track_id")),
            "tracker_state": _text(row.get("tracker_state")),
            "tracker_confirmed": _boolean(row.get("tracker_confirmed")),
            "tracker_hits": _integer(row.get("tracker_hits")),
            "tracker_association_stage": _text(row.get("tracker_association_stage")),
            "tracker_match_iou": _number(row.get("tracker_match_iou")),
            "target_selector_reason": _text(row.get("target_selector_reason")),
            "score": _number(row.get("detection_score")),
            "bbox_xyxy": _vector(row, ("bbox_x1", "bbox_y1", "bbox_x2", "bbox_y2")),
            "bbox_area_ratio": _number(row.get("bbox_area_ratio")),
            "rknn_ms": {
                "preprocess": _number(row.get("rknn_preprocess_ms")),
                "inference": _number(row.get("rknn_inference_ms")),
                "postprocess": _number(row.get("rknn_postprocess_ms")),
                "total": _number(row.get("rknn_total_ms")),
            },
            "tracker_update_ms": _number(row.get("tracker_update_ms")),
            "tracker_fps": _number(row.get("tracker_actual_fps")),
            "result_age_ms": _number(row.get("perception_result_age_ms")),
            "attitude_offset_ms": _number(row.get("detection_attitude_offset_ms")),
            "fusion": {
                "status": _text(row.get("fusion_status")),
                "pending_count": _integer(row.get("fusion_pending_count")),
                "dropped_count": _integer(row.get("fusion_dropped_count")),
                "wait_ms": _number(row.get("fusion_wait_ms")),
            },
            "queue_dropped": _integer(row.get("perception_queue_dropped")),
            "worker_error": _text(row.get("perception_worker_error")),
        },
        "guidance": {
            "law": _text(row.get("guidance_law")),
            "navigation_constant": _number(row.get("guidance_navigation_constant")),
            "fixed_vm_m_s": _number(row.get("guidance_fixed_vm_m_s")),
            "fixed_gain": _number(row.get("guidance_fixed_gain")),
            "max_accel_mps2": _number(row.get("guidance_max_accel_mps2")),
            "ttc_required": _boolean(row.get("guidance_ttc_required")),
            "los_valid": _boolean(row.get("los_valid")),
            "los_reason": _text(row.get("los_reject_reason")),
            "los_quality": _number(row.get("los_quality")),
            "lambda": _vector(row, ("lambda_I_x", "lambda_I_y", "lambda_I_z")),
            "lambda_dot": _vector(
                row,
                ("lambda_dot_I_x", "lambda_dot_I_y", "lambda_dot_I_z"),
            ),
            "omega_los": _vector(row, ("omega_los_x", "omega_los_y", "omega_los_z")),
            "ttc_valid": _boolean(row.get("ttc_valid")),
            "ttc_s": _number(row.get("ttc_s")),
            "ttc_reason": _text(row.get("ttc_reject_reason")),
            "valid": _boolean(row.get("guidance_valid")),
            "reason": _text(row.get("guidance_reject_reason")),
            "quality": _number(row.get("guidance_quality")),
            "eval_frame": _text(row.get("guidance_eval_frame")),
            "rate_gain_input_frame": _text(row.get("rate_gain_input_frame")),
            "g_eval": _vector(row, ("g_eval_x", "g_eval_y", "g_eval_z")),
            "g_eval_body_frd": _vector(
                row,
                ("g_eval_body_frd_x", "g_eval_body_frd_y", "g_eval_body_frd_z"),
            ),
        },
        "command": {
            "valid": _boolean(row.get("sp_valid")),
            "source": _text(row.get("sp_source")),
            "reason": _text(row.get("sp_reject_reason")),
            "rate_deg_s": _vector(
                row,
                ("sp_roll_rate_deg_s", "sp_pitch_rate_deg_s", "sp_yaw_rate_deg_s"),
            ),
            "thrust": _number(row.get("sp_thrust")),
            "shaping": {
                "valid": _boolean(row.get("shaping_valid")),
                "reason": _text(row.get("shaping_reason")),
                "input_rate_deg_s": _vector(
                    row,
                    (
                        "pre_shape_sp_roll_rate_deg_s",
                        "pre_shape_sp_pitch_rate_deg_s",
                    ),
                ),
                "output_rate_deg_s": _vector(
                    row,
                    ("sp_roll_rate_deg_s", "sp_pitch_rate_deg_s"),
                ),
                "entry_handoff": {
                    "active": _boolean(row.get("entry_handoff_active")),
                    "progress": _number(row.get("entry_handoff_progress")),
                    "source": _text(row.get("entry_handoff_source")),
                    "start_rate_deg_s": _vector(
                        row,
                        (
                            "entry_handoff_start_roll_rate_deg_s",
                            "entry_handoff_start_pitch_rate_deg_s",
                        ),
                    ),
                },
                "tilt_envelope": {
                    "attitude_deg": _vector(
                        row,
                        ("tilt_roll_attitude_deg", "tilt_pitch_attitude_deg"),
                    ),
                    "softcap_factor": _vector(
                        row,
                        ("tilt_roll_softcap_factor", "tilt_pitch_softcap_factor"),
                    ),
                    "level_weight": _vector(
                        row,
                        ("tilt_roll_level_weight", "tilt_pitch_level_weight"),
                    ),
                    "hardcap_active": _boolean(row.get("tilt_hardcap_active")),
                },
            },
        },
        "host": {
            "sample_age_s": _number(row.get("host_sample_age_s")),
            "load_1m": _number(row.get("host_load_1m")),
            "rss_mb": _number(row.get("host_process_rss_mb")),
            "memory_available_mb": _number(row.get("host_mem_available_mb")),
            "disk_free_gb": _number(row.get("host_disk_free_gb")),
            "thermal_max_c": _number(row.get("host_thermal_max_c")),
            "soc_temp_c": _number(row.get("host_soc_temp_c")),
            "npu_temp_c": _number(row.get("host_npu_temp_c")),
            "cpu_freq_mhz": [
                _number(row.get("host_cpu_freq_min_mhz")),
                _number(row.get("host_cpu_freq_max_mhz")),
            ],
            "npu_freq_mhz": _number(row.get("host_npu_freq_mhz")),
            "python_gc": {
                "collection_count": _integer(row.get("python_gc_collection_count")),
                "last_generation": _integer(row.get("python_gc_last_generation")),
                "last_pause_ms": _number(row.get("python_gc_last_pause_ms")),
                "max_pause_ms": _number(row.get("python_gc_max_pause_ms")),
                "total_pause_ms": _number(row.get("python_gc_total_pause_ms")),
            },
            "error": _text(row.get("host_health_error")),
        },
        "web": {
            "running": _boolean(row.get("web_running")),
            "sse_clients": _integer(row.get("web_sse_clients")),
            "mjpeg_clients": _integer(row.get("web_mjpeg_clients")),
            "publish_count": _integer(row.get("web_publish_count")),
            "preview_offer_count": _integer(row.get("web_preview_offer_count")),
            "preview_encode_count": _integer(row.get("web_preview_encode_count")),
            "preview_drop_count": _integer(row.get("web_preview_drop_count")),
            "error_count": _integer(row.get("web_error_count")),
            "last_error": _text(row.get("web_last_error")),
        },
    }
    return payload


class TelemetryHub:
    def __init__(self, config: TelemetryWebConfig, *, cv2_module: Any = None) -> None:
        self.config = config
        self._cv2_module = cv2_module
        self._lock = threading.RLock()
        self._preview_condition = threading.Condition(self._lock)
        self._jpeg_condition = threading.Condition(self._lock)
        self._history: deque[dict[str, Any]] = deque(maxlen=config.history_capacity)
        self._latest: dict[str, Any] | None = None
        self._last_history_s: float | None = None
        self._last_vision_measurement: dict[str, Any] | None = None
        self._last_vision_measurement_s: float | None = None
        self._sequence = 0
        self._running = False
        self._preview_pending: tuple[Any, dict[str, Any]] | None = None
        self._jpeg: bytes | None = None
        self._jpeg_sequence = 0
        self._encoder_thread: threading.Thread | None = None
        self._sse_clients = 0
        self._mjpeg_clients = 0
        self._publish_count = 0
        self._preview_offer_count = 0
        self._preview_encode_count = 0
        self._preview_drop_count = 0
        self._request_count = 0
        self._denied_count = 0
        self._error_count = 0
        self._last_error = ""

    def start(self) -> None:
        with self._lock:
            if self._running:
                return
            self._running = True
        if self.config.preview.enabled:
            self._encoder_thread = threading.Thread(
                target=self._encoder_loop,
                name="betaflight-web-preview",
                daemon=True,
            )
            self._encoder_thread.start()

    def close(self) -> None:
        with self._lock:
            self._running = False
            self._preview_condition.notify_all()
            self._jpeg_condition.notify_all()
        if self._encoder_thread is not None:
            self._encoder_thread.join(timeout=2.0)

    @property
    def running(self) -> bool:
        with self._lock:
            return self._running

    def publish(self, payload: Mapping[str, Any], *, timestamp_s: float | None = None) -> dict[str, Any]:
        now = time.monotonic() if timestamp_s is None else float(timestamp_s)
        snapshot = copy.deepcopy(dict(payload))
        with self._lock:
            self._stabilize_vision(snapshot, now)
            self._sequence += 1
            snapshot["schema_version"] = WEB_SCHEMA_VERSION
            snapshot["sequence"] = self._sequence
            snapshot["sample_monotonic_s"] = now
            snapshot["sample_unix_s"] = time.time()
            self._latest = snapshot
            self._publish_count += 1
            period_s = 1.0 / self.config.sample_hz
            if self._last_history_s is None or now - self._last_history_s >= period_s:
                self._history.append(copy.deepcopy(snapshot))
                self._last_history_s = now
            return copy.deepcopy(snapshot)

    def _stabilize_vision(self, snapshot: dict[str, Any], now: float) -> None:
        vision = snapshot.get("vision")
        if not isinstance(vision, dict):
            return
        gap = vision.get("detector_reason") in {
            "perception_no_new_result",
            "fusion_waiting_for_attitude",
        }
        vision["new_result"] = not gap
        vision["display_held"] = False
        if not gap:
            self._last_vision_measurement = {
                key: copy.deepcopy(vision.get(key)) for key in VISION_MEASUREMENT_KEYS
            }
            self._last_vision_measurement_s = now
            return
        if self._last_vision_measurement is None or self._last_vision_measurement_s is None:
            return
        for key, value in self._last_vision_measurement.items():
            vision[key] = copy.deepcopy(value)
        base_age_ms = vision.get("result_age_ms")
        if isinstance(base_age_ms, (int, float)) and math.isfinite(float(base_age_ms)):
            vision["result_age_ms"] = float(base_age_ms) + 1000.0 * max(
                0.0, now - self._last_vision_measurement_s
            )
        vision["display_held"] = True

    def latest(self, *, timestamp_s: float | None = None) -> dict[str, Any]:
        now = time.monotonic() if timestamp_s is None else float(timestamp_s)
        with self._lock:
            if self._latest is None:
                return {
                    "schema_version": WEB_SCHEMA_VERSION,
                    "sequence": 0,
                    "sample_age_ms": None,
                    "stale": True,
                    "status": "waiting_for_first_sample",
                }
            value = copy.deepcopy(self._latest)
        age_s = max(0.0, now - float(value["sample_monotonic_s"]))
        value["sample_age_ms"] = 1000.0 * age_s
        value["stale"] = age_s > self.config.stale_after_s
        return value

    def history(self, seconds: float | None = None, *, timestamp_s: float | None = None) -> dict[str, Any]:
        now = time.monotonic() if timestamp_s is None else float(timestamp_s)
        requested = self.config.history_s if seconds is None else float(seconds)
        window_s = min(self.config.history_s, max(1.0, requested))
        cutoff = now - window_s
        with self._lock:
            samples = [copy.deepcopy(item) for item in self._history if item["sample_monotonic_s"] >= cutoff]
        return {
            "schema_version": WEB_SCHEMA_VERSION,
            "window_s": window_s,
            "samples": samples,
        }

    def offer_preview(self, frame_bgr: Any, overlay: Mapping[str, Any] | None = None) -> None:
        if not self.config.preview.enabled or frame_bgr is None:
            return
        with self._lock:
            self._preview_offer_count += 1
            if not self._running or self._mjpeg_clients <= 0:
                return
            if self._preview_pending is not None:
                self._preview_drop_count += 1
            self._preview_pending = (frame_bgr, dict(overlay or {}))
            self._preview_condition.notify()

    def wants_preview(self) -> bool:
        """Return whether a connected client currently needs preview frames."""
        with self._lock:
            return bool(
                self.config.preview.enabled
                and self._running
                and self._mjpeg_clients > 0
            )

    def offer_encoded_preview(self, jpeg: bytes | bytearray | memoryview) -> None:
        """Publish JPEG bytes produced outside the control process."""
        if not self.config.preview.enabled or not jpeg:
            return
        payload = bytes(jpeg)
        with self._jpeg_condition:
            self._preview_offer_count += 1
            if not self._running or self._mjpeg_clients <= 0:
                return
            self._jpeg = payload
            self._jpeg_sequence += 1
            self._preview_encode_count += 1
            self._jpeg_condition.notify_all()

    def try_add_client(self, kind: str) -> bool:
        with self._lock:
            if kind == "sse":
                if self._sse_clients >= self.config.max_sse_clients:
                    return False
                self._sse_clients += 1
                return True
            if kind == "mjpeg":
                if self._mjpeg_clients >= self.config.preview.max_clients:
                    return False
                self._mjpeg_clients += 1
                return True
        raise ValueError(f"unknown web client kind: {kind}")

    def remove_client(self, kind: str) -> None:
        with self._lock:
            if kind == "sse":
                self._sse_clients = max(0, self._sse_clients - 1)
            elif kind == "mjpeg":
                self._mjpeg_clients = max(0, self._mjpeg_clients - 1)
                if self._mjpeg_clients == 0 and self._preview_pending is not None:
                    self._preview_pending = None
                    self._preview_drop_count += 1
            else:
                raise ValueError(f"unknown web client kind: {kind}")

    def wait_for_jpeg(self, after_sequence: int, *, timeout_s: float) -> tuple[int, bytes | None]:
        with self._jpeg_condition:
            self._jpeg_condition.wait_for(
                lambda: not self._running or self._jpeg_sequence > after_sequence,
                timeout=max(0.0, float(timeout_s)),
            )
            return self._jpeg_sequence, self._jpeg

    def note_request(self, *, denied: bool = False) -> None:
        with self._lock:
            self._request_count += 1
            if denied:
                self._denied_count += 1

    def record_error(self, message: str) -> None:
        with self._lock:
            self._error_count += 1
            self._last_error = str(message)[:500]

    def stats(self) -> dict[str, Any]:
        with self._lock:
            return {
                "web_running": int(self._running),
                "web_sse_clients": self._sse_clients,
                "web_mjpeg_clients": self._mjpeg_clients,
                "web_publish_count": self._publish_count,
                "web_preview_offer_count": self._preview_offer_count,
                "web_preview_encode_count": self._preview_encode_count,
                "web_preview_drop_count": self._preview_drop_count,
                "web_request_count": self._request_count,
                "web_denied_count": self._denied_count,
                "web_error_count": self._error_count,
                "web_last_error": self._last_error,
            }

    def _encoder_loop(self) -> None:
        try:
            cv2 = self._cv2_module if self._cv2_module is not None else importlib.import_module("cv2")
        except Exception as exc:
            self.record_error(f"preview_cv2_import_failed:{exc}")
            return
        period_s = 1.0 / self.config.preview.max_fps
        next_encode_s = 0.0
        while True:
            with self._preview_condition:
                self._preview_condition.wait_for(
                    lambda: not self._running or self._preview_pending is not None,
                    timeout=0.5,
                )
                if not self._running:
                    return
                pending = self._preview_pending
                self._preview_pending = None
            if pending is None:
                continue
            delay_s = next_encode_s - time.monotonic()
            if delay_s > 0.0:
                time.sleep(delay_s)
            frame_bgr, overlay = pending
            try:
                canvas = _draw_preview_overlay(cv2, frame_bgr, overlay, self.latest())
                ok, encoded = cv2.imencode(
                    ".jpg",
                    canvas,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.config.preview.jpeg_quality],
                )
                if not ok:
                    raise RuntimeError("cv2.imencode returned false")
                jpeg = encoded.tobytes()
            except Exception as exc:
                self.record_error(f"preview_encode_failed:{exc}")
                continue
            next_encode_s = time.monotonic() + period_s
            with self._jpeg_condition:
                self._jpeg = jpeg
                self._jpeg_sequence += 1
                self._preview_encode_count += 1
                self._jpeg_condition.notify_all()


class _TelemetryHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, server_address: tuple[str, int], service: "TelemetryWebService") -> None:
        self.telemetry_service = service
        super().__init__(server_address, _TelemetryRequestHandler)


class TelemetryWebService:
    def __init__(
        self,
        config: TelemetryWebConfig,
        *,
        dashboard_path: str | Path | None = None,
        cv2_module: Any = None,
    ) -> None:
        self.config = config
        self.hub = TelemetryHub(config, cv2_module=cv2_module)
        selected_path = Path(dashboard_path) if dashboard_path is not None else (
            Path(config.dashboard_path).expanduser() if config.dashboard_path else DEFAULT_DASHBOARD_PATH
        )
        self.dashboard_path = selected_path
        self._dashboard = b""
        self._allowed_networks = tuple(
            ipaddress.ip_network(value, strict=False) for value in config.allowed_subnets
        )
        self._server: _TelemetryHttpServer | None = None
        self._thread: threading.Thread | None = None

    @property
    def running(self) -> bool:
        return self._server is not None and self.hub.running

    @property
    def url(self) -> str:
        host = self.config.bind if self.config.bind not in ("0.0.0.0", "::") else "<orangepi-ip>"
        return f"http://{host}:{self.config.port}/"

    def start(self) -> None:
        if not self.config.enabled or self._server is not None:
            return
        try:
            self._dashboard = self.dashboard_path.read_bytes()
        except OSError as exc:
            raise RuntimeError(f"failed to read telemetry dashboard {self.dashboard_path}: {exc}") from exc
        try:
            server = _TelemetryHttpServer((self.config.bind, self.config.port), self)
        except OSError as exc:
            raise RuntimeError(
                f"failed to bind telemetry web server {self.config.bind}:{self.config.port}: {exc}"
            ) from exc
        self._server = server
        self.hub.start()
        self._thread = threading.Thread(
            target=server.serve_forever,
            kwargs={"poll_interval": 0.2},
            name="betaflight-web-http",
            daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        server = self._server
        self._server = None
        if server is not None:
            server.shutdown()
            server.server_close()
        self.hub.close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def publish(self, payload: Mapping[str, Any], *, timestamp_s: float | None = None) -> None:
        if self.config.enabled:
            self.hub.publish(payload, timestamp_s=timestamp_s)

    def offer_preview(self, frame_bgr: Any, overlay: Mapping[str, Any] | None = None) -> None:
        if self.config.enabled:
            self.hub.offer_preview(frame_bgr, overlay)

    def wants_preview(self) -> bool:
        return bool(self.config.enabled and self.hub.wants_preview())

    def offer_encoded_preview(self, jpeg: bytes | bytearray | memoryview) -> None:
        if self.config.enabled:
            self.hub.offer_encoded_preview(jpeg)

    def log_stats(self) -> dict[str, Any]:
        if not self.config.enabled:
            return {
                "web_running": 0,
                "web_sse_clients": 0,
                "web_mjpeg_clients": 0,
                "web_publish_count": 0,
                "web_preview_offer_count": 0,
                "web_preview_encode_count": 0,
                "web_preview_drop_count": 0,
                "web_request_count": 0,
                "web_denied_count": 0,
                "web_error_count": 0,
                "web_last_error": "",
            }
        return self.hub.stats()

    def metadata(self) -> dict[str, Any]:
        return {
            "schema_version": WEB_SCHEMA_VERSION,
            "config": asdict(self.config),
            "dashboard_path": str(self.dashboard_path),
            "url": self.url,
            "read_only": True,
        }

    def client_allowed(self, address: str) -> bool:
        try:
            client = ipaddress.ip_address(address)
        except ValueError:
            return False
        return any(client in network for network in self._allowed_networks if client.version == network.version)


class _TelemetryRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "BetaflightTelemetry/1"
    sys_version = ""

    @property
    def service(self) -> TelemetryWebService:
        return self.server.telemetry_service  # type: ignore[attr-defined]

    def do_HEAD(self) -> None:  # noqa: N802
        self._handle(head_only=True)

    def do_GET(self) -> None:  # noqa: N802
        self._handle(head_only=False)

    def do_POST(self) -> None:  # noqa: N802
        self._method_not_allowed()

    def do_PUT(self) -> None:  # noqa: N802
        self._method_not_allowed()

    def do_DELETE(self) -> None:  # noqa: N802
        self._method_not_allowed()

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def _handle(self, *, head_only: bool) -> None:
        if not self.service.client_allowed(self.client_address[0]):
            self.service.hub.note_request(denied=True)
            self._send_json(HTTPStatus.FORBIDDEN, {"error": "client_not_allowed"}, head_only=head_only)
            return
        self.service.hub.note_request()
        target = urlsplit(self.path)
        try:
            if target.path == "/":
                self._send_bytes(
                    HTTPStatus.OK,
                    "text/html; charset=utf-8",
                    self.service._dashboard,
                    head_only=head_only,
                )
            elif target.path == "/api/v1/telemetry":
                self._send_json(HTTPStatus.OK, self.service.hub.latest(), head_only=head_only)
            elif target.path == "/api/v1/history":
                query = parse_qs(target.query)
                seconds = _query_float(query, "seconds")
                self._send_json(
                    HTTPStatus.OK,
                    self.service.hub.history(seconds),
                    head_only=head_only,
                )
            elif target.path == "/healthz":
                latest = self.service.hub.latest()
                healthy = self.service.running and not bool(latest.get("stale", True))
                self._send_json(
                    HTTPStatus.OK if healthy else HTTPStatus.SERVICE_UNAVAILABLE,
                    {
                        "ok": healthy,
                        "running": self.service.running,
                        "sample_age_ms": latest.get("sample_age_ms"),
                        "stale": latest.get("stale", True),
                        "web": self.service.hub.stats(),
                    },
                    head_only=head_only,
                )
            elif target.path == "/api/v1/stream" and not head_only:
                self._serve_sse()
            elif target.path == "/api/v1/video/mjpeg" and not head_only:
                self._serve_mjpeg()
            elif target.path in ("/api/v1/stream", "/api/v1/video/mjpeg"):
                self._method_not_allowed()
            else:
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "not_found"}, head_only=head_only)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return
        except Exception as exc:
            self.service.hub.record_error(f"http_handler_failed:{type(exc).__name__}:{exc}")
            try:
                self._send_json(
                    HTTPStatus.INTERNAL_SERVER_ERROR,
                    {"error": "internal_server_error"},
                    head_only=head_only,
                )
            except (BrokenPipeError, ConnectionResetError):
                return

    def _serve_sse(self) -> None:
        if not self.service.hub.try_add_client("sse"):
            self._send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "sse_client_limit"})
            return
        try:
            self.send_response(HTTPStatus.OK)
            self._common_headers("text/event-stream; charset=utf-8", content_length=None)
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            period_s = 1.0 / self.service.config.sample_hz
            while self.service.running:
                payload = json.dumps(
                    self.service.hub.latest(),
                    separators=(",", ":"),
                    allow_nan=False,
                )
                self.wfile.write(f"event: telemetry\ndata: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
                time.sleep(period_s)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return
        finally:
            self.service.hub.remove_client("sse")

    def _serve_mjpeg(self) -> None:
        if not self.service.config.preview.enabled:
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "preview_disabled"})
            return
        if not self.service.hub.try_add_client("mjpeg"):
            self._send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "mjpeg_client_limit"})
            return
        try:
            self.send_response(HTTPStatus.OK)
            self._common_headers("multipart/x-mixed-replace; boundary=frame", content_length=None)
            self.send_header("Connection", "close")
            self.end_headers()
            sequence = 0
            while self.service.running:
                new_sequence, jpeg = self.service.hub.wait_for_jpeg(sequence, timeout_s=1.0)
                if jpeg is None or new_sequence <= sequence:
                    continue
                sequence = new_sequence
                header = (
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode("ascii")
                    + b"\r\n\r\n"
                )
                self.wfile.write(header)
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return
        finally:
            self.service.hub.remove_client("mjpeg")

    def _method_not_allowed(self) -> None:
        self._send_json(HTTPStatus.METHOD_NOT_ALLOWED, {"error": "read_only_api"})

    def _send_json(
        self,
        status: HTTPStatus,
        payload: Mapping[str, Any],
        *,
        head_only: bool = False,
    ) -> None:
        body = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self._send_bytes(status, "application/json; charset=utf-8", body, head_only=head_only)

    def _send_bytes(
        self,
        status: HTTPStatus,
        content_type: str,
        body: bytes,
        *,
        head_only: bool,
    ) -> None:
        self.send_response(status)
        self._common_headers(content_type, content_length=len(body))
        self.send_header("Connection", "close")
        self.end_headers()
        if not head_only:
            self.wfile.write(body)

    def _common_headers(self, content_type: str, *, content_length: int | None) -> None:
        self.send_header("Content-Type", content_type)
        if content_length is not None:
            self.send_header("Content-Length", str(content_length))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; img-src 'self' data:; connect-src 'self'; "
            "style-src 'self' 'unsafe-inline'; script-src 'self' 'unsafe-inline'; frame-ancestors 'none'",
        )


def _draw_preview_overlay(cv2: Any, frame_bgr: Any, overlay: Mapping[str, Any], latest: Mapping[str, Any]) -> Any:
    canvas = frame_bgr.copy()
    height, width = canvas.shape[:2]
    center = (width // 2, height // 2)
    cv2.drawMarker(canvas, center, (0, 220, 255), cv2.MARKER_CROSS, 18, 1, cv2.LINE_AA)
    bbox = overlay.get("bbox_xyxy")
    if isinstance(bbox, (list, tuple)) and len(bbox) == 4:
        x1, y1, x2, y2 = [int(round(float(value))) for value in bbox]
        x1, x2 = sorted((max(0, min(width - 1, x1)), max(0, min(width - 1, x2))))
        y1, y2 = sorted((max(0, min(height - 1, y1)), max(0, min(height - 1, y2))))
        color = (80, 220, 90)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        track_id = overlay.get("track_id")
        score = overlay.get("score")
        label = f"UAV #{track_id if track_id is not None else '-'}"
        if score is not None:
            label += f"  {float(score):.2f}"
        cv2.putText(
            canvas,
            label,
            (x1, max(18, y1 - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )
    safety = dict(latest.get("safety", {}))
    state = str(safety.get("state") or "WAIT")
    armed = "ARMED" if safety.get("armed") else "DISARMED"
    override = "OVR ON" if safety.get("override_active") else "OVR OFF"
    publish = str(safety.get("publish_mode") or "-").upper()
    vision = dict(latest.get("vision", {}))
    tracked = "TRACKED" if vision.get("tracker_confirmed") else "NO TRACK"
    reason = str(safety.get("reason") or "-")
    status_text = f"{state}  {armed}  {override}  {publish}"
    detail_text = f"{tracked}  {reason}"
    cv2.rectangle(canvas, (0, 0), (min(width, 500), 45), (20, 24, 30), -1)
    cv2.putText(
        canvas,
        status_text,
        (8, 19),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (235, 238, 242),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        detail_text,
        (8, 38),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43,
        (190, 198, 205),
        1,
        cv2.LINE_AA,
    )
    return canvas


def _query_float(query: Mapping[str, list[str]], key: str) -> float | None:
    values = query.get(key)
    if not values:
        return None
    try:
        return float(values[0])
    except ValueError:
        return None


def _text(value: Any) -> str | None:
    if value in (None, ""):
        return None
    return str(value)


def _number(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _integer(value: Any) -> int | None:
    number = _number(value)
    return None if number is None else int(number)


def _boolean(value: Any) -> bool | None:
    integer = _integer(value)
    return None if integer is None else bool(integer)


def _vector(row: Mapping[str, Any], fields: tuple[str, ...]) -> list[float | None]:
    return [_number(row.get(field)) for field in fields]


def _channels(row: Mapping[str, Any], prefix: str, count: int) -> list[int | None]:
    return [_integer(row.get(f"{prefix}{index}")) for index in range(1, count + 1)]


def _msp_rc_order(channel_count: int) -> str:
    if channel_count < 4:
        raise ValueError("MSP RC telemetry requires at least four channels")
    return "AERT" + "".join(str(index) for index in range(1, channel_count - 3))


def _reorder_rc_input(channels: list[int | None], *, channel_map: str) -> list[int | None]:
    input_order = _msp_rc_order(len(channels))
    normalized_map = str(channel_map).upper()
    if len(normalized_map) != len(input_order) or sorted(normalized_map) != sorted(input_order):
        raise ValueError("telemetry channel_map must match the MSP RC channel roles")
    by_role = dict(zip(input_order, channels))
    return [by_role[role] for role in normalized_map]
