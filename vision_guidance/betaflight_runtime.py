from __future__ import annotations

import hashlib
import json
import math
import threading
import time
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any, Sequence

from .betaflight_msp import (
    MSP_ANALOG,
    MSP_ALTITUDE,
    MSP_ATTITUDE,
    MSP_MOTOR,
    MSP_RAW_IMU,
    MSP_RAW_GPS,
    MSP_RC,
    MSP_SET_RAW_RC,
    MSP_STATUS,
    AsyncMspResponse,
    BetaflightMSPAdapter,
    BetaflightTelemetry,
    MspAdapterStats,
    parse_analog,
    parse_altitude,
    parse_attitude,
    parse_motor_outputs,
    parse_raw_imu,
    parse_raw_gps,
    parse_rc_channels,
    parse_status,
)
from .flight_control import RcCommand


MSP_OVERRIDE_PERMANENT_ID = 50
MSP_GYRO_AXIS_NAMES = ("x", "y", "z")


@dataclass(frozen=True)
class MspRawImuGyroConfig:
    """Convert MSP_RAW_IMU gyro values only for an explicitly bound FC build."""

    enabled: bool = False
    scale_deg_s_per_lsb: float = 0.0625
    axis_order: tuple[str, str, str] = MSP_GYRO_AXIS_NAMES
    axis_sign: tuple[float, float, float] = (1.0, 1.0, 1.0)
    output_frame: str = "body_frd"
    expected_fc_variant: str = ""
    expected_fc_version: tuple[int, int, int] = (0, 0, 0)
    expected_api_version: tuple[int, int] = (0, 0)

    @classmethod
    def from_mapping(cls, values: dict[str, Any]) -> "MspRawImuGyroConfig":
        axis_order = tuple(str(value).strip().lower() for value in values.get("axis_order", MSP_GYRO_AXIS_NAMES))
        axis_sign = tuple(float(value) for value in values.get("axis_sign", (1.0, 1.0, 1.0)))
        expected_fc_version = tuple(int(value) for value in values.get("expected_fc_version", (0, 0, 0)))
        expected_api_version = tuple(int(value) for value in values.get("expected_api_version", (0, 0)))
        config = cls(
            enabled=bool(values.get("enabled", False)),
            scale_deg_s_per_lsb=float(values.get("scale_deg_s_per_lsb", 0.0625)),
            axis_order=axis_order,
            axis_sign=axis_sign,
            output_frame=str(values.get("output_frame", "body_frd")).strip().lower(),
            expected_fc_variant=str(values.get("expected_fc_variant", "")).strip(),
            expected_fc_version=expected_fc_version,
            expected_api_version=expected_api_version,
        )
        if not math.isfinite(config.scale_deg_s_per_lsb) or config.scale_deg_s_per_lsb <= 0.0:
            raise ValueError("raw_imu_gyro.scale_deg_s_per_lsb must be finite and positive")
        if len(config.axis_order) != 3 or set(config.axis_order) != set(MSP_GYRO_AXIS_NAMES):
            raise ValueError("raw_imu_gyro.axis_order must be a permutation of x,y,z")
        if len(config.axis_sign) != 3 or any(value not in (-1.0, 1.0) for value in config.axis_sign):
            raise ValueError("raw_imu_gyro.axis_sign must contain three values in {-1,1}")
        if config.output_frame != "body_frd":
            raise ValueError("raw_imu_gyro.output_frame must be body_frd")
        if len(config.expected_fc_version) != 3 or len(config.expected_api_version) != 2:
            raise ValueError("raw_imu_gyro firmware/API versions have invalid lengths")
        if config.enabled and (
            not config.expected_fc_variant
            or any(value < 0 for value in config.expected_fc_version)
            or config.expected_fc_version == (0, 0, 0)
            or any(value < 0 for value in config.expected_api_version)
            or config.expected_api_version == (0, 0)
        ):
            raise ValueError("enabled raw_imu_gyro requires explicit FC variant, FC version, and API version")
        return config


@dataclass(frozen=True)
class BoundMspRawImuGyroConverter:
    config: MspRawImuGyroConfig
    available: bool
    reason: str

    def convert(self, gyro_msp_raw: Sequence[float] | None) -> tuple[float, float, float] | None:
        if not self.available or gyro_msp_raw is None or len(gyro_msp_raw) < 3:
            return None
        source = {name: float(gyro_msp_raw[index]) for index, name in enumerate(MSP_GYRO_AXIS_NAMES)}
        converted = tuple(
            source[axis] * sign * self.config.scale_deg_s_per_lsb
            for axis, sign in zip(self.config.axis_order, self.config.axis_sign)
        )
        return converted if all(math.isfinite(value) for value in converted) else None

    def metadata(self) -> dict[str, Any]:
        return {
            "available": self.available,
            "reason": self.reason,
            "source": "MSP_RAW_IMU",
            "scale_deg_s_per_lsb": self.config.scale_deg_s_per_lsb,
            "axis_order": list(self.config.axis_order),
            "axis_sign": list(self.config.axis_sign),
            "output_frame": self.config.output_frame,
            "firmware_binding": {
                "fc_variant": self.config.expected_fc_variant,
                "fc_version": list(self.config.expected_fc_version),
                "api_version": list(self.config.expected_api_version),
            },
        }


def bind_msp_raw_imu_gyro(
    config: MspRawImuGyroConfig,
    fc_identity: dict[str, Any],
) -> BoundMspRawImuGyroConverter:
    if not config.enabled:
        return BoundMspRawImuGyroConverter(config, False, "conversion_disabled")
    if "fc_identity_error" in fc_identity:
        return BoundMspRawImuGyroConverter(config, False, "fc_identity_unavailable")
    actual = (
        str(fc_identity.get("fc_variant", "")),
        (
            int(fc_identity.get("fc_version_major", -1)),
            int(fc_identity.get("fc_version_minor", -1)),
            int(fc_identity.get("fc_version_patch", -1)),
        ),
        (int(fc_identity.get("api_major", -1)), int(fc_identity.get("api_minor", -1))),
    )
    expected = (
        config.expected_fc_variant,
        config.expected_fc_version,
        config.expected_api_version,
    )
    if actual != expected:
        return BoundMspRawImuGyroConverter(config, False, "firmware_binding_mismatch")
    return BoundMspRawImuGyroConverter(config, True, "firmware_binding_match")


@dataclass(frozen=True)
class ControlAuthorizationStatus:
    approved: bool
    reason: str
    approval_path: str = ""
    snapshot_path: str = ""
    snapshot_sha256: str = ""
    config_conflict_free: bool = False
    scope: str = ""
    parameters_path: str = ""
    parameters_sha256: str = ""


@dataclass(frozen=True)
class MspRuntimeConfig:
    io_worker_enabled: bool = False
    transport_mode: str = "synchronous"
    telemetry_poll_hz: float = 5.0
    status_poll_hz: float = 5.0
    attitude_poll_hz: float = 5.0
    raw_imu_poll_hz: float = 0.0
    raw_gps_poll_hz: float = 0.0
    altitude_poll_hz: float = 0.0
    motor_poll_hz: float = 0.0
    rc_poll_hz: float = 5.0
    analog_poll_hz: float = 5.0
    control_publish_hz: float = 50.0
    physical_rc_timeout_s: float = 0.25
    override_grace_hold_s: float = 0.35
    override_channels_mask: int = 15
    override_mode_cli_id: int = 50
    aux_arm_channel_zero_based: int = 4
    throttle_channel_zero_based: int = 2
    set_raw_rc_channel_map: str = "AETR1234"
    throttle_handover_s: float = 0.4
    throttle_slew_limit_us_per_s: float = 0.0
    throttle_relative_limit_us: int = 0
    throttle_reference_min_us: int = 1000
    throttle_reference_max_us: int = 2000
    throttle_command_min_us: int = 1000
    throttle_command_max_us: int = 2000
    prefill_enabled: bool = False
    prefill_min_frames: int = 10
    prefill_valid_min_us: int = 900
    prefill_valid_max_us: int = 2100
    staged_command_timeout_s: float = 0.15
    shutdown_passthrough_frames: int = 3
    response_drain_budget_ms: float = 3.0
    response_stale_s: float = 0.25
    raw_imu_gyro: MspRawImuGyroConfig = field(default_factory=MspRawImuGyroConfig)

    @classmethod
    def from_mapping(cls, values: dict[str, Any]) -> "MspRuntimeConfig":
        legacy_poll_hz = float(values.get("telemetry_poll_hz", 5.0))
        config = cls(
            io_worker_enabled=bool(values.get("io_worker_enabled", False)),
            transport_mode=str(values.get("transport_mode", "synchronous")).strip().lower(),
            telemetry_poll_hz=legacy_poll_hz,
            status_poll_hz=float(values.get("status_poll_hz", legacy_poll_hz)),
            attitude_poll_hz=float(values.get("attitude_poll_hz", legacy_poll_hz)),
            raw_imu_poll_hz=float(values.get("raw_imu_poll_hz", 0.0)),
            raw_gps_poll_hz=float(values.get("raw_gps_poll_hz", 0.0)),
            altitude_poll_hz=float(values.get("altitude_poll_hz", 0.0)),
            motor_poll_hz=float(values.get("motor_poll_hz", 0.0)),
            rc_poll_hz=float(values.get("rc_poll_hz", legacy_poll_hz)),
            analog_poll_hz=float(values.get("analog_poll_hz", legacy_poll_hz)),
            control_publish_hz=float(values.get("control_publish_hz", 50.0)),
            physical_rc_timeout_s=float(values.get("physical_rc_timeout_s", 0.25)),
            override_grace_hold_s=float(values.get("override_grace_hold_s", 0.35)),
            override_channels_mask=int(values.get("override_channels_mask", 15)),
            override_mode_cli_id=int(values.get("override_mode_cli_id", 50)),
            aux_arm_channel_zero_based=int(values.get("aux_arm_channel_zero_based", 4)),
            throttle_channel_zero_based=int(values.get("throttle_channel_zero_based", 2)),
            set_raw_rc_channel_map=str(values.get("set_raw_rc_channel_map", "AETR1234")).upper(),
            throttle_handover_s=float(values.get("throttle_handover_s", 0.4)),
            throttle_slew_limit_us_per_s=float(
                values.get("throttle_slew_limit_us_per_s", 0.0)
            ),
            throttle_relative_limit_us=int(values.get("throttle_relative_limit_us", 0)),
            throttle_reference_min_us=int(values.get("throttle_reference_min_us", 1000)),
            throttle_reference_max_us=int(values.get("throttle_reference_max_us", 2000)),
            throttle_command_min_us=int(values.get("throttle_command_min_us", 1000)),
            throttle_command_max_us=int(values.get("throttle_command_max_us", 2000)),
            prefill_enabled=bool(values.get("prefill_enabled", False)),
            prefill_min_frames=int(values.get("prefill_min_frames", 10)),
            prefill_valid_min_us=int(values.get("prefill_valid_min_us", 900)),
            prefill_valid_max_us=int(values.get("prefill_valid_max_us", 2100)),
            staged_command_timeout_s=float(values.get("staged_command_timeout_s", 0.15)),
            shutdown_passthrough_frames=int(values.get("shutdown_passthrough_frames", 3)),
            response_drain_budget_ms=float(values.get("response_drain_budget_ms", 3.0)),
            response_stale_s=float(values.get("response_stale_s", 0.25)),
            raw_imu_gyro=MspRawImuGyroConfig.from_mapping(dict(values.get("raw_imu_gyro", {}))),
        )
        if config.transport_mode not in {"synchronous", "async_pipeline"}:
            raise ValueError("msp_runtime.transport_mode must be synchronous or async_pipeline")
        if config.telemetry_poll_hz <= 0.0 or config.control_publish_hz <= 0.0:
            raise ValueError("MSP worker rates must be positive")
        if any(
            rate < 0.0
            for rate in (
                config.status_poll_hz,
                config.attitude_poll_hz,
                config.raw_imu_poll_hz,
                config.raw_gps_poll_hz,
                config.altitude_poll_hz,
                config.motor_poll_hz,
                config.rc_poll_hz,
                config.analog_poll_hz,
            )
        ):
            raise ValueError("MSP per-command poll rates must be non-negative")
        if config.status_poll_hz <= 0.0 or config.rc_poll_hz <= 0.0:
            raise ValueError("MSP STATUS and RC polling must remain enabled")
        if (
            config.physical_rc_timeout_s <= 0.0
            or not 0.0 <= config.override_grace_hold_s <= 2.0
            or config.throttle_handover_s < 0.0
            or not 0.0 <= config.throttle_slew_limit_us_per_s <= 5000.0
        ):
            raise ValueError("MSP worker timeout/handover values are invalid")
        if not 0 <= config.throttle_relative_limit_us <= 500:
            raise ValueError("throttle_relative_limit_us must be in [0, 500]")
        if not (
            750
            <= config.throttle_command_min_us
            <= config.throttle_reference_min_us
            <= config.throttle_reference_max_us
            <= config.throttle_command_max_us
            <= 2250
        ):
            raise ValueError(
                "throttle command/reference limits must be ordered within [750, 2250]"
            )
        if config.prefill_min_frames <= 0 or config.staged_command_timeout_s <= 0.0:
            raise ValueError("MSP worker prefill/command timeout values are invalid")
        if not 0 <= config.override_mode_cli_id <= 255:
            raise ValueError("override_mode_cli_id must be in range 0-255")
        _validate_set_raw_rc_channel_map(config.set_raw_rc_channel_map)
        if not 750 <= config.prefill_valid_min_us < config.prefill_valid_max_us <= 2250:
            raise ValueError("MSP worker prefill RC validity range is invalid")
        throttle_index = config.set_raw_rc_channel_map.index("T")
        if config.throttle_channel_zero_based != throttle_index:
            raise ValueError("throttle_channel_zero_based must match set_raw_rc_channel_map")
        if config.shutdown_passthrough_frames < 0:
            raise ValueError("shutdown_passthrough_frames must be non-negative")
        if config.response_drain_budget_ms < 0.0 or config.response_stale_s <= 0.0:
            raise ValueError("MSP async response budget/stale timeout values are invalid")
        if config.raw_imu_gyro.enabled and config.raw_imu_poll_hz <= 0.0:
            raise ValueError("enabled raw_imu_gyro requires raw_imu_poll_hz > 0")
        return config


@dataclass(frozen=True)
class MspWorkerSnapshot:
    telemetry: BetaflightTelemetry | None
    telemetry_error: str
    telemetry_age_s: float | None
    status_age_s: float | None
    attitude_age_s: float | None
    analog_age_s: float | None
    raw_imu_age_s: float | None
    raw_gps_age_s: float | None
    altitude_age_s: float | None
    motor_age_s: float | None
    physical_rc_age_s: float | None
    physical_rc_fresh: bool
    poll_count: int
    poll_error_count: int
    staged_count: int
    send_skip_count: int
    send_error_count: int
    worker_error: str
    output_enabled: bool
    algorithm_authorized: bool
    override_active: bool
    override_release_hold_active: bool
    prefill_ready: bool
    prefill_success_count: int
    passthrough_send_count: int
    algorithm_send_count: int
    stale_command_count: int
    staged_command_age_s: float | None
    publish_mode: str
    publish_reason: str
    rc_source: str
    pilot_control_available: bool
    last_sent_channels: tuple[int, ...]
    last_publish_output_enabled: bool
    last_publish_algorithm_authorized: bool
    last_publish_override_active: bool
    last_publish_override_release_hold_active: bool
    last_publish_prefill_ready: bool
    last_publish_physical_rc_fresh: bool
    last_publish_command_fresh: bool
    last_publish_command_active: bool
    last_publish_command_reason: str
    last_publish_set_raw_rc_ack_fresh: bool
    publish_tick_interval_s: float | None
    publish_tick_max_interval_s: float | None
    publish_deadline_miss_count: int
    send_success_interval_s: float | None
    send_success_max_interval_s: float | None
    last_send_success_age_s: float | None
    consecutive_send_error_count: int
    set_raw_rc_ack_age_s: float | None
    set_raw_rc_ack_fresh: bool
    rc_poll_suspended: bool
    throttle_handover: "ThrottleHandoverSnapshot"
    throttle_slew_limited: bool
    throttle_slew_output_us: int | None
    adapter_stats: MspAdapterStats


def box_mode_index(box_ids: Sequence[int], permanent_id: int) -> int | None:
    try:
        return tuple(int(value) for value in box_ids).index(int(permanent_id))
    except ValueError:
        return None


def box_mode_active(mode_flags: int, box_ids: Sequence[int], permanent_id: int) -> bool:
    index = box_mode_index(box_ids, permanent_id)
    return bool(index is not None and int(mode_flags) & (1 << index))


def armed_from_telemetry(telemetry: BetaflightTelemetry | None, box_ids: Sequence[int]) -> bool:
    if telemetry is None or telemetry.status is None:
        return False
    return box_mode_active(telemetry.status.mode_flags, box_ids, 0)


def resolve_control_authorization(
    values: dict[str, Any],
    *,
    fc_identity: dict[str, Any],
    box_ids: Sequence[int],
    parameters_path: str | Path | None = None,
) -> ControlAuthorizationStatus:
    if not bool(values.get("enabled", False)):
        return ControlAuthorizationStatus(False, "authorization_disabled")
    approval_path = Path(str(values.get("approval_manifest", ""))).expanduser()
    if not approval_path.is_file():
        return ControlAuthorizationStatus(False, "approval_manifest_missing", str(approval_path))
    try:
        approval = json.loads(approval_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return ControlAuthorizationStatus(False, f"approval_manifest_invalid:{exc}", str(approval_path))
    if approval.get("approved") is not True:
        return ControlAuthorizationStatus(False, "approval_not_granted", str(approval_path))
    minimum_schema_version = int(values.get("minimum_approval_schema_version", 1))
    try:
        approval_schema_version = int(approval.get("schema_version", 1))
    except (TypeError, ValueError):
        approval_schema_version = 0
    if approval_schema_version < minimum_schema_version:
        return ControlAuthorizationStatus(
            False,
            "approval_schema_version_too_old",
            str(approval_path),
        )
    if approval.get("source_conflicts_resolved") is not True:
        return ControlAuthorizationStatus(False, "source_conflicts_unresolved", str(approval_path))
    scope = str(approval.get("scope", "")).strip()
    required_scope = str(values.get("required_scope", "")).strip()
    if required_scope and scope != required_scope:
        return ControlAuthorizationStatus(False, "authorization_scope_mismatch", str(approval_path), scope=scope)
    snapshot_path = Path(str(approval.get("snapshot_manifest", ""))).expanduser()
    if not snapshot_path.is_file():
        return ControlAuthorizationStatus(False, "snapshot_manifest_missing", str(approval_path), str(snapshot_path))
    actual_sha = _sha256(snapshot_path)
    expected_sha = str(approval.get("snapshot_sha256", ""))
    if not expected_sha or actual_sha != expected_sha:
        return ControlAuthorizationStatus(
            False, "snapshot_sha256_mismatch", str(approval_path), str(snapshot_path), actual_sha
        )
    try:
        snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return ControlAuthorizationStatus(
            False,
            f"snapshot_manifest_invalid:{exc}",
            str(approval_path),
            str(snapshot_path),
            actual_sha,
            scope=scope,
        )
    if snapshot.get("readiness", {}).get("log_only_ready") is not True:
        return ControlAuthorizationStatus(
            False, "snapshot_not_log_only_ready", str(approval_path), str(snapshot_path), actual_sha
        )
    expected_identity = dict(approval.get("expected_fc_identity", {}))
    if not expected_identity or any(fc_identity.get(key) != value for key, value in expected_identity.items()):
        return ControlAuthorizationStatus(
            False, "fc_identity_mismatch", str(approval_path), str(snapshot_path), actual_sha
        )
    if MSP_OVERRIDE_PERMANENT_ID not in tuple(int(value) for value in box_ids):
        return ControlAuthorizationStatus(
            False, "msp_override_box_missing", str(approval_path), str(snapshot_path), actual_sha
        )
    expected_parameters_sha = str(approval.get("parameters_sha256", ""))
    if not expected_parameters_sha:
        return ControlAuthorizationStatus(
            False, "parameters_sha256_missing", str(approval_path), str(snapshot_path), actual_sha
        )
    if parameters_path is None:
        return ControlAuthorizationStatus(
            False,
            "parameters_path_required",
            str(approval_path),
            str(snapshot_path),
            actual_sha,
            scope=scope,
        )
    resolved_parameters_path = Path(parameters_path).expanduser()
    actual_parameters_sha = ""
    if not resolved_parameters_path.is_file():
        return ControlAuthorizationStatus(
            False,
            "parameters_file_missing",
            str(approval_path),
            str(snapshot_path),
            actual_sha,
            scope=scope,
            parameters_path=str(resolved_parameters_path),
        )
    actual_parameters_sha = _sha256(resolved_parameters_path)
    if actual_parameters_sha != expected_parameters_sha:
        return ControlAuthorizationStatus(
            False,
            "parameters_sha256_mismatch",
            str(approval_path),
            str(snapshot_path),
            actual_sha,
            scope=scope,
            parameters_path=str(resolved_parameters_path),
            parameters_sha256=actual_parameters_sha,
        )
    if bool(values.get("release_evidence_required", False)):
        release_evidence = approval.get("release_evidence")
        if not isinstance(release_evidence, dict):
            return ControlAuthorizationStatus(
                False,
                "release_evidence_missing",
                str(approval_path),
                str(snapshot_path),
                actual_sha,
                scope=scope,
                parameters_path=str(resolved_parameters_path),
                parameters_sha256=actual_parameters_sha,
            )
        evidence_path = Path(str(release_evidence.get("path", ""))).expanduser()
        if not evidence_path.is_file():
            reason = "release_evidence_file_missing"
        elif _sha256(evidence_path) != str(release_evidence.get("sha256", "")):
            reason = "release_evidence_sha256_mismatch"
        else:
            try:
                evidence_report = json.loads(evidence_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                reason = "release_evidence_invalid"
            else:
                if not isinstance(evidence_report, dict):
                    reason = "release_evidence_invalid"
                elif evidence_report.get("release_passed") is not True:
                    reason = "release_evidence_not_passed"
                else:
                    evidence_binding = evidence_report.get("runtime_binding", {})
                    if not isinstance(evidence_binding, dict) or (
                        evidence_binding.get("sha256") != actual_parameters_sha
                    ):
                        reason = "release_evidence_parameters_mismatch"
                    else:
                        reason = ""
        if reason:
            return ControlAuthorizationStatus(
                False,
                reason,
                str(approval_path),
                str(snapshot_path),
                actual_sha,
                scope=scope,
                parameters_path=str(resolved_parameters_path),
                parameters_sha256=actual_parameters_sha,
            )
    if bool(values.get("rc_interlock_evidence_required", False)):
        interlock_evidence = approval.get("rc_interlock_evidence")
        reason = ""
        if not isinstance(interlock_evidence, dict):
            reason = "rc_interlock_evidence_missing"
        else:
            evidence_path = Path(str(interlock_evidence.get("path", ""))).expanduser()
            if not evidence_path.is_file():
                reason = "rc_interlock_evidence_file_missing"
            elif _sha256(evidence_path) != str(interlock_evidence.get("sha256", "")):
                reason = "rc_interlock_evidence_sha256_mismatch"
            else:
                try:
                    evidence_report = json.loads(evidence_path.read_text(encoding="utf-8"))
                except (OSError, json.JSONDecodeError):
                    reason = "rc_interlock_evidence_invalid"
                else:
                    binding = (
                        evidence_report.get("runtime_binding", {})
                        if isinstance(evidence_report, dict)
                        else {}
                    )
                    if evidence_report.get("passed") is not True:
                        reason = "rc_interlock_evidence_not_passed"
                    elif binding.get("sha256") != actual_parameters_sha:
                        reason = "rc_interlock_evidence_parameters_mismatch"
        if reason:
            return ControlAuthorizationStatus(
                False,
                reason,
                str(approval_path),
                str(snapshot_path),
                actual_sha,
                scope=scope,
                parameters_path=str(resolved_parameters_path),
                parameters_sha256=actual_parameters_sha,
            )
    return ControlAuthorizationStatus(
        True,
        "approved",
        str(approval_path.resolve()),
        str(snapshot_path.resolve()),
        actual_sha,
        True,
        scope,
        str(resolved_parameters_path.resolve()),
        actual_parameters_sha,
    )


def merge_physical_rc(
    physical_channels: Sequence[int],
    algorithm_channels: Sequence[int],
    *,
    override_channels_mask: int,
    aux_arm_channel_zero_based: int,
) -> tuple[int, ...]:
    if len(physical_channels) < len(algorithm_channels):
        raise ValueError("physical RC must contain every algorithm channel")
    result = [int(value) for value in physical_channels]
    for index, value in enumerate(algorithm_channels):
        if index == int(aux_arm_channel_zero_based):
            continue
        if int(override_channels_mask) & (1 << index):
            result[index] = int(value)
    return tuple(result)


def reorder_msp_rc_to_set_raw_rc(
    logical_channels: Sequence[int],
    set_raw_rc_channel_map: str,
) -> tuple[int, ...]:
    """Convert MSP_RC logical R/P/Y/T order to the receiver-mapped wire order."""
    channel_map = str(set_raw_rc_channel_map).upper()
    _validate_set_raw_rc_channel_map(channel_map)
    if len(logical_channels) < len(channel_map):
        raise ValueError("MSP_RC channel count is shorter than set_raw_rc_channel_map")
    logical_by_role = {
        "A": int(logical_channels[0]),
        "E": int(logical_channels[1]),
        "R": int(logical_channels[2]),
        "T": int(logical_channels[3]),
    }
    for aux_number in range(1, len(logical_channels) - 3):
        logical_by_role[str(aux_number)] = int(logical_channels[aux_number + 3])
    result = [int(value) for value in logical_channels]
    for index, role in enumerate(channel_map):
        if role not in logical_by_role:
            raise ValueError(f"MSP_RC does not contain channel role {role}")
        result[index] = logical_by_role[role]
    return tuple(result)


def _validate_set_raw_rc_channel_map(channel_map: str) -> None:
    if len(channel_map) < 4:
        raise ValueError("set_raw_rc_channel_map must contain at least four channels")
    for role in ("A", "E", "T", "R"):
        if channel_map.count(role) != 1:
            raise ValueError(f"set_raw_rc_channel_map must contain exactly one {role}")
    aux_roles = channel_map[4:]
    if aux_roles != "".join(str(index) for index in range(1, len(aux_roles) + 1)):
        raise ValueError("set_raw_rc_channel_map AUX roles must be sequential from 1")


@dataclass(frozen=True)
class ThrottleHandoverSnapshot:
    source_us: int | None = None
    target_us: int | None = None
    requested_target_us: int | None = None
    lower_limit_us: int | None = None
    upper_limit_us: int | None = None
    target_limited: bool = False
    alpha: float | None = None
    output_us: int | None = None
    active: bool = False


class ThrottleHandover:
    def __init__(self, duration_s: float):
        if duration_s < 0.0:
            raise ValueError("duration_s must be non-negative")
        self.duration_s = float(duration_s)
        self._start_s: float | None = None
        self._from_us = 1000
        self._snapshot = ThrottleHandoverSnapshot()

    def reset(self, timestamp: float, physical_throttle_us: int) -> None:
        self._start_s = float(timestamp)
        self._from_us = int(physical_throttle_us)
        self._snapshot = ThrottleHandoverSnapshot(source_us=self._from_us, alpha=0.0, active=True)

    def apply(
        self,
        timestamp: float,
        target_us: int,
        *,
        requested_target_us: int | None = None,
        lower_limit_us: int | None = None,
        upper_limit_us: int | None = None,
    ) -> int:
        if self._start_s is None or self.duration_s <= 0.0:
            alpha = 1.0
        else:
            alpha = min(1.0, max(0.0, (float(timestamp) - self._start_s) / self.duration_s))
        output_us = int(round((1.0 - alpha) * self._from_us + alpha * int(target_us)))
        self._snapshot = ThrottleHandoverSnapshot(
            source_us=self._from_us,
            target_us=int(target_us),
            requested_target_us=(
                int(target_us) if requested_target_us is None else int(requested_target_us)
            ),
            lower_limit_us=lower_limit_us,
            upper_limit_us=upper_limit_us,
            target_limited=(
                requested_target_us is not None and int(requested_target_us) != int(target_us)
            ),
            alpha=alpha,
            output_us=output_us,
            active=alpha < 1.0,
        )
        return output_us

    def clear(self) -> None:
        self._start_s = None
        self._snapshot = ThrottleHandoverSnapshot()

    def snapshot(self) -> ThrottleHandoverSnapshot:
        return self._snapshot


class BetaflightMspIoWorker:
    def __init__(
        self,
        adapter: BetaflightMSPAdapter,
        config: MspRuntimeConfig,
        *,
        box_ids: Sequence[int] = (),
    ):
        self.adapter = adapter
        self.config = config
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._telemetry: BetaflightTelemetry | None = None
        self._telemetry_error = ""
        self._poll_errors: dict[str, str] = {}
        self._telemetry_received_s: float | None = None
        self._physical_rc_received_s: float | None = None
        self._staged: RcCommand | None = None
        self._staged_received_s: float | None = None
        self._output_enabled = False
        self._algorithm_authorized = False
        self._override_active = False
        self._override_released_s: float | None = None
        self._poll_count = 0
        self._poll_error_count = 0
        self._staged_count = 0
        self._send_skip_count = 0
        self._send_error_count = 0
        self._worker_error = ""
        self._handover = ThrottleHandover(config.throttle_handover_s)
        self._throttle_reference_us: int | None = None
        self._throttle_slew_output_us: int | None = None
        self._throttle_slew_timestamp_s: float | None = None
        self._throttle_slew_limited = False
        self._was_algorithm_authorized = False
        self._manual_rc: tuple[int, ...] = ()
        self._prefill_success_count = 0
        self._passthrough_send_count = 0
        self._algorithm_send_count = 0
        self._stale_command_count = 0
        self._publish_mode = "disabled"
        self._publish_reason = "not_started"
        self._rc_source = "none"
        self._pilot_control_available = False
        self._algorithm_release_active = False
        self._release_throttle_reference_us: int | None = None
        self._last_sent_channels: tuple[int, ...] = ()
        self._last_publish_output_enabled = False
        self._last_publish_algorithm_authorized = False
        self._last_publish_override_active = False
        self._last_publish_override_release_hold_active = False
        self._last_publish_prefill_ready = False
        self._last_publish_physical_rc_fresh = False
        self._last_publish_command_fresh = False
        self._last_publish_command_active = False
        self._last_publish_command_reason = ""
        self._last_publish_set_raw_rc_ack_fresh = False
        self._last_publish_tick_s: float | None = None
        self._publish_tick_interval_s: float | None = None
        self._publish_tick_max_interval_s: float | None = None
        self._publish_deadline_miss_count = 0
        self._last_send_success_s: float | None = None
        self._send_success_interval_s: float | None = None
        self._send_success_max_interval_s: float | None = None
        self._consecutive_send_error_count = 0
        self._async_set_writes: dict[int, tuple[str, bool]] = {}
        self._override_mode_index = box_mode_index(box_ids, MSP_OVERRIDE_PERMANENT_ID)
        self._poll_rates_hz = {
            "status": config.status_poll_hz,
            "attitude": config.attitude_poll_hz,
            "raw_imu": config.raw_imu_poll_hz,
            "raw_gps": config.raw_gps_poll_hz,
            "altitude": config.altitude_poll_hz,
            "motor": config.motor_poll_hz,
            "rc": config.rc_poll_hz,
            "analog": config.analog_poll_hz,
        }
        self._next_poll_s = {name: 0.0 for name, rate in self._poll_rates_hz.items() if rate > 0.0}

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="betaflight-msp-io", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(2.0, 4.0 * self.adapter.timeout_s))
            if self._thread.is_alive():
                raise RuntimeError("Betaflight MSP worker did not stop")
        with self._lock:
            output_enabled = self._output_enabled
            self._algorithm_authorized = False
        if output_enabled and self.config.prefill_enabled:
            for _ in range(self.config.shutdown_passthrough_frames):
                self._publish(time.monotonic())
                if self.config.transport_mode == "async_pipeline":
                    self._handle_async_responses(
                        self.adapter.drain_async_responses(self.config.response_drain_budget_ms)
                    )
                time.sleep(min(0.02, 1.0 / self.config.control_publish_hz))
        if self.config.transport_mode == "async_pipeline":
            self.adapter.end_async_pipeline()
        self._thread = None

    def stage(
        self,
        command: RcCommand | None,
        *,
        output_enabled: bool | None = None,
        algorithm_authorized: bool | None = None,
        override_active: bool = False,
        authorized: bool | None = None,
    ) -> None:
        if authorized is not None:
            if output_enabled is None:
                output_enabled = bool(authorized)
            if algorithm_authorized is None:
                algorithm_authorized = bool(authorized)
        with self._lock:
            self._staged = command
            now = time.monotonic()
            self._staged_received_s = now
            self._output_enabled = bool(output_enabled)
            self._algorithm_authorized = bool(algorithm_authorized)
            self._set_override_active_locked(bool(override_active), now)
            self._staged_count += 1

    def snapshot(self, timestamp: float | None = None) -> MspWorkerSnapshot:
        now = time.monotonic() if timestamp is None else float(timestamp)
        with self._lock:
            telemetry = self._telemetry
            status_timestamp = None if telemetry is None else telemetry.status_timestamp_s
            attitude_timestamp = None if telemetry is None else telemetry.attitude_timestamp_s
            analog_timestamp = None if telemetry is None else telemetry.analog_timestamp_s
            raw_imu_timestamp = None if telemetry is None else telemetry.raw_imu_timestamp_s
            raw_gps_timestamp = None if telemetry is None else telemetry.raw_gps_timestamp_s
            altitude_timestamp = None if telemetry is None else telemetry.altitude_timestamp_s
            motor_timestamp = None if telemetry is None else telemetry.motor_timestamp_s
            if telemetry is not None:
                if status_timestamp is None and telemetry.status is not None:
                    status_timestamp = telemetry.timestamp
                if attitude_timestamp is None and telemetry.attitude is not None:
                    attitude_timestamp = telemetry.timestamp
                if analog_timestamp is None and telemetry.analog is not None:
                    analog_timestamp = telemetry.timestamp
                if raw_imu_timestamp is None and telemetry.raw_imu is not None:
                    raw_imu_timestamp = telemetry.timestamp
                if raw_gps_timestamp is None and telemetry.raw_gps is not None:
                    raw_gps_timestamp = telemetry.timestamp
                if altitude_timestamp is None and telemetry.altitude is not None:
                    altitude_timestamp = telemetry.timestamp
                if motor_timestamp is None and telemetry.motor_outputs:
                    motor_timestamp = telemetry.timestamp
            status_age = None if status_timestamp is None else max(0.0, now - status_timestamp)
            attitude_age = None if attitude_timestamp is None else max(0.0, now - attitude_timestamp)
            analog_age = None if analog_timestamp is None else max(0.0, now - analog_timestamp)
            raw_imu_age = None if raw_imu_timestamp is None else max(0.0, now - raw_imu_timestamp)
            raw_gps_age = None if raw_gps_timestamp is None else max(0.0, now - raw_gps_timestamp)
            altitude_age = None if altitude_timestamp is None else max(0.0, now - altitude_timestamp)
            motor_age = None if motor_timestamp is None else max(0.0, now - motor_timestamp)
            telemetry_age = status_age
            rc_age = None if self._physical_rc_received_s is None else max(0.0, now - self._physical_rc_received_s)
            command_age = None if self._staged_received_s is None else max(0.0, now - self._staged_received_s)
            send_success_age = (
                None if self._last_send_success_s is None else max(0.0, now - self._last_send_success_s)
            )
            adapter_stats = self.adapter.snapshot_stats()
            ack_age = (
                None
                if adapter_stats.set_raw_rc_last_ack_monotonic_s is None
                else max(0.0, now - adapter_stats.set_raw_rc_last_ack_monotonic_s)
            )
            ack_fresh = self.config.transport_mode != "async_pipeline" or (
                ack_age is not None and ack_age <= self.config.response_stale_s
            )
            release_hold_active = self._override_release_hold_active_locked(now)
            rc_poll_suspended = bool(self._override_active and self._manual_rc)
            return MspWorkerSnapshot(
                telemetry=telemetry,
                telemetry_error=self._telemetry_error,
                telemetry_age_s=telemetry_age,
                status_age_s=status_age,
                attitude_age_s=attitude_age,
                analog_age_s=analog_age,
                raw_imu_age_s=raw_imu_age,
                raw_gps_age_s=raw_gps_age,
                altitude_age_s=altitude_age,
                motor_age_s=motor_age,
                physical_rc_age_s=rc_age,
                physical_rc_fresh=bool(
                    (self._override_active or release_hold_active) and self._manual_rc
                    or rc_age is not None and rc_age <= self.config.physical_rc_timeout_s
                ),
                poll_count=self._poll_count,
                poll_error_count=self._poll_error_count,
                staged_count=self._staged_count,
                send_skip_count=self._send_skip_count,
                send_error_count=self._send_error_count,
                worker_error=self._worker_error,
                output_enabled=self._output_enabled,
                algorithm_authorized=self._algorithm_authorized,
                override_active=self._override_active,
                override_release_hold_active=release_hold_active,
                prefill_ready=self._prefill_ready(),
                prefill_success_count=self._prefill_success_count,
                passthrough_send_count=self._passthrough_send_count,
                algorithm_send_count=self._algorithm_send_count,
                stale_command_count=self._stale_command_count,
                staged_command_age_s=command_age,
                publish_mode=self._publish_mode,
                publish_reason=self._publish_reason,
                rc_source=self._rc_source,
                pilot_control_available=self._pilot_control_available,
                last_sent_channels=self._last_sent_channels,
                last_publish_output_enabled=self._last_publish_output_enabled,
                last_publish_algorithm_authorized=self._last_publish_algorithm_authorized,
                last_publish_override_active=self._last_publish_override_active,
                last_publish_override_release_hold_active=(
                    self._last_publish_override_release_hold_active
                ),
                last_publish_prefill_ready=self._last_publish_prefill_ready,
                last_publish_physical_rc_fresh=self._last_publish_physical_rc_fresh,
                last_publish_command_fresh=self._last_publish_command_fresh,
                last_publish_command_active=self._last_publish_command_active,
                last_publish_command_reason=self._last_publish_command_reason,
                last_publish_set_raw_rc_ack_fresh=self._last_publish_set_raw_rc_ack_fresh,
                publish_tick_interval_s=self._publish_tick_interval_s,
                publish_tick_max_interval_s=self._publish_tick_max_interval_s,
                publish_deadline_miss_count=self._publish_deadline_miss_count,
                send_success_interval_s=self._send_success_interval_s,
                send_success_max_interval_s=self._send_success_max_interval_s,
                last_send_success_age_s=send_success_age,
                consecutive_send_error_count=self._consecutive_send_error_count,
                set_raw_rc_ack_age_s=ack_age,
                set_raw_rc_ack_fresh=ack_fresh,
                rc_poll_suspended=rc_poll_suspended,
                throttle_handover=self._handover.snapshot(),
                throttle_slew_limited=self._throttle_slew_limited,
                throttle_slew_output_us=self._throttle_slew_output_us,
                adapter_stats=adapter_stats,
            )

    def _run(self) -> None:
        if self.config.transport_mode == "async_pipeline":
            self._run_async()
            return
        self._run_synchronous()

    def _run_synchronous(self) -> None:
        publish_period = 1.0 / self.config.control_publish_hz
        next_publish = time.monotonic()
        poll_budget = False
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_publish:
                self._publish(now)
                periods = max(1, int((now - next_publish) / publish_period) + 1)
                next_publish += periods * publish_period
                poll_budget = True
            if poll_budget:
                poll_name = self._next_due_poll_name(time.monotonic())
                if poll_name is not None:
                    self._poll_one(poll_name)
                    poll_budget = False
                    continue
            deadlines = [next_publish]
            if poll_budget and self._next_poll_s:
                deadlines.append(min(self._next_poll_s.values()))
            wait_s = max(0.0, min(deadlines) - time.monotonic())
            self._stop.wait(min(0.005, wait_s))

    def _run_async(self) -> None:
        try:
            self.adapter.begin_async_pipeline()
            publish_period = 1.0 / self.config.control_publish_hz
            next_publish = time.monotonic()
            while not self._stop.is_set():
                now = time.monotonic()
                if now < next_publish:
                    self._stop.wait(min(0.001, next_publish - now))
                    continue
                self._publish(now)
                self.adapter.expire_async_requests(self.config.response_stale_s, now)
                self._queue_one_async_poll(time.monotonic())
                self._handle_async_responses(
                    self.adapter.drain_async_responses(self.config.response_drain_budget_ms)
                )
                completed = time.monotonic()
                periods = max(1, int((completed - next_publish) / publish_period) + 1)
                next_publish += periods * publish_period
                with self._lock:
                    last_send_success_s = self._last_send_success_s
                if last_send_success_s is not None:
                    minimum_spacing_s = 0.8 * publish_period
                    next_publish = max(next_publish, last_send_success_s + minimum_spacing_s)
        except Exception as exc:
            with self._lock:
                self._worker_error = str(exc)
                self._poll_error_count += 1

    def _queue_one_async_poll(self, now: float) -> None:
        due = sorted(
            (name for name, timestamp in self._next_poll_s.items() if now >= timestamp),
            key=lambda name: (name != "rc", self._next_poll_s[name]),
        )
        command_by_name = {
            "status": MSP_STATUS,
            "attitude": MSP_ATTITUDE,
            "raw_imu": MSP_RAW_IMU,
            "raw_gps": MSP_RAW_GPS,
            "altitude": MSP_ALTITUDE,
            "motor": MSP_MOTOR,
            "rc": MSP_RC,
            "analog": MSP_ANALOG,
        }
        with self._lock:
            suspend_rc = bool(self._override_active and self._manual_rc)
        for name in due:
            if name == "rc" and suspend_rc:
                self._next_poll_s[name] = now + 1.0 / self._poll_rates_hz[name]
                continue
            command = command_by_name[name]
            if self.adapter.async_request_pending(command):
                continue
            try:
                queued = self.adapter.queue_async_request(command)
            except Exception as exc:
                self._record_poll_error(name, exc)
                queued = None
            if queued is not None:
                self._next_poll_s[name] = now + 1.0 / self._poll_rates_hz[name]
                return

    def _handle_async_responses(self, responses: Sequence[AsyncMspResponse]) -> None:
        name_by_command = {
            MSP_STATUS: "status",
            MSP_ATTITUDE: "attitude",
            MSP_RAW_IMU: "raw_imu",
            MSP_RAW_GPS: "raw_gps",
            MSP_ALTITUDE: "altitude",
            MSP_MOTOR: "motor",
            MSP_RC: "rc",
            MSP_ANALOG: "analog",
        }
        parser_by_command = {
            MSP_STATUS: parse_status,
            MSP_ATTITUDE: parse_attitude,
            MSP_RAW_IMU: parse_raw_imu,
            MSP_RAW_GPS: parse_raw_gps,
            MSP_ALTITUDE: parse_altitude,
            MSP_MOTOR: parse_motor_outputs,
            MSP_RC: parse_rc_channels,
            MSP_ANALOG: parse_analog,
        }
        for response in responses:
            frame = response.frame
            if frame.command == MSP_SET_RAW_RC:
                self._handle_async_set_response(response)
                continue
            name = name_by_command.get(frame.command)
            if name is None or response.request_id is None:
                continue
            if frame.direction == "!":
                self._record_poll_error(name, RuntimeError(f"MSP error response for {frame.command}"))
                continue
            try:
                value = parser_by_command[frame.command](frame.payload)
                self._merge_poll_value(name, value, response.response_monotonic_s)
            except Exception as exc:
                self._record_poll_error(name, exc)

    def _handle_async_set_response(self, response: AsyncMspResponse) -> None:
        if response.request_id is None:
            return
        with self._lock:
            context = self._async_set_writes.pop(response.request_id, None)
            if response.frame.direction == "!":
                self._send_error_count += 1
                self._consecutive_send_error_count += 1
                self._worker_error = "MSP error response for SET_RAW_RC"
                self._prefill_success_count = 0
                return
            if context is None:
                return
            publish_mode, use_algorithm = context
            if use_algorithm:
                return
            if publish_mode in {
                "live_passthrough",
                "override_frozen_hold",
                "release_hold",
            }:
                self._prefill_success_count += 1

    def _next_due_poll_name(self, now: float) -> str | None:
        due = [name for name, timestamp in self._next_poll_s.items() if now >= timestamp]
        if not due:
            return None
        return min(due, key=lambda name: self._next_poll_s[name])

    def _poll_one(self, name: str) -> None:
        try:
            if name == "status":
                value = self.adapter.read_status()
            elif name == "attitude":
                value = self.adapter.read_attitude()
            elif name == "raw_imu":
                value = self.adapter.read_raw_imu()
            elif name == "raw_gps":
                value = self.adapter.read_raw_gps()
            elif name == "altitude":
                value = self.adapter.read_altitude()
            elif name == "motor":
                value = self.adapter.read_motor_outputs()
            elif name == "rc":
                value = tuple(self.adapter.read_rc())
            elif name == "analog":
                value = self.adapter.read_analog()
            else:
                raise ValueError(f"unsupported MSP poll source: {name}")
            self._merge_poll_value(name, value, time.monotonic())
        except Exception as exc:
            self._record_poll_error(name, exc)
        finally:
            rate = self._poll_rates_hz.get(name, 0.0)
            if rate > 0.0:
                self._next_poll_s[name] = time.monotonic() + 1.0 / rate

    def _merge_poll_value(self, name: str, value: Any, received_s: float) -> None:
        with self._lock:
            telemetry = self._telemetry or BetaflightTelemetry(timestamp=received_s)
            if name == "status":
                telemetry = replace(
                    telemetry,
                    timestamp=received_s,
                    status=value,
                    status_timestamp_s=received_s,
                )
                if self._override_mode_index is not None:
                    self._set_override_active_locked(
                        bool(value.mode_flags & (1 << self._override_mode_index)),
                        received_s,
                    )
            elif name == "attitude":
                telemetry = replace(
                    telemetry,
                    timestamp=received_s,
                    attitude=value,
                    attitude_timestamp_s=received_s,
                )
            elif name == "raw_imu":
                telemetry = replace(
                    telemetry,
                    timestamp=received_s,
                    raw_imu=value,
                    raw_imu_timestamp_s=received_s,
                )
            elif name == "raw_gps":
                telemetry = replace(
                    telemetry,
                    timestamp=received_s,
                    raw_gps=value,
                    raw_gps_timestamp_s=received_s,
                )
            elif name == "altitude":
                telemetry = replace(
                    telemetry,
                    timestamp=received_s,
                    altitude=value,
                    altitude_timestamp_s=received_s,
                )
            elif name == "motor":
                telemetry = replace(
                    telemetry,
                    timestamp=received_s,
                    motor_outputs=tuple(value),
                    motor_timestamp_s=received_s,
                )
            elif name == "rc":
                telemetry = replace(
                    telemetry,
                    timestamp=received_s,
                    rc_channels=tuple(value),
                    rc_timestamp_s=received_s,
                )
                physical = reorder_msp_rc_to_set_raw_rc(value, self.config.set_raw_rc_channel_map)
                if not self._override_active and self._physical_rc_valid(physical):
                    self._manual_rc = physical
                    self._physical_rc_received_s = received_s
                    self._override_released_s = None
                elif not self._manual_rc:
                    self._physical_rc_received_s = received_s
            elif name == "analog":
                telemetry = replace(
                    telemetry,
                    timestamp=received_s,
                    analog=value,
                    analog_timestamp_s=received_s,
                )
            else:
                raise ValueError(f"unsupported MSP poll source: {name}")
            self._telemetry = telemetry
            self._telemetry_received_s = received_s
            self._poll_count += 1
            self._poll_errors.pop(name, None)
            self._telemetry_error = "; ".join(
                f"{source}:{message}" for source, message in sorted(self._poll_errors.items())
            )

    def _record_poll_error(self, name: str, exc: Exception) -> None:
        with self._lock:
            self._poll_errors[name] = str(exc)
            self._telemetry_error = "; ".join(
                f"{source}:{message}" for source, message in sorted(self._poll_errors.items())
            )
            self._poll_error_count += 1

    def _set_override_active_locked(self, active: bool, timestamp_s: float) -> None:
        active = bool(active)
        if self._override_active and not active:
            self._override_released_s = float(timestamp_s)
        elif active:
            self._override_released_s = None
        self._override_active = active

    def _override_release_hold_active_locked(self, timestamp_s: float) -> bool:
        return bool(
            not self._override_active
            and self._manual_rc
            and self._override_released_s is not None
            and float(timestamp_s) - self._override_released_s <= self.config.override_grace_hold_s
        )

    def _poll(self, now: float) -> None:
        del now
        try:
            telemetry = self.adapter.read_telemetry()
            received_s = telemetry.timestamp
            telemetry = replace(
                telemetry,
                status_timestamp_s=(
                    telemetry.status_timestamp_s
                    if telemetry.status_timestamp_s is not None
                    else received_s if telemetry.status is not None else None
                ),
                attitude_timestamp_s=(
                    telemetry.attitude_timestamp_s
                    if telemetry.attitude_timestamp_s is not None
                    else received_s if telemetry.attitude is not None else None
                ),
                analog_timestamp_s=(
                    telemetry.analog_timestamp_s
                    if telemetry.analog_timestamp_s is not None
                    else received_s if telemetry.analog is not None else None
                ),
                rc_timestamp_s=(
                    telemetry.rc_timestamp_s
                    if telemetry.rc_timestamp_s is not None
                    else received_s if telemetry.rc_channels else None
                ),
                raw_imu_timestamp_s=(
                    telemetry.raw_imu_timestamp_s
                    if telemetry.raw_imu_timestamp_s is not None
                    else received_s if telemetry.raw_imu is not None else None
                ),
                raw_gps_timestamp_s=(
                    telemetry.raw_gps_timestamp_s
                    if telemetry.raw_gps_timestamp_s is not None
                    else received_s if telemetry.raw_gps is not None else None
                ),
                altitude_timestamp_s=(
                    telemetry.altitude_timestamp_s
                    if telemetry.altitude_timestamp_s is not None
                    else received_s if telemetry.altitude is not None else None
                ),
            )
            with self._lock:
                observed_override = self._override_active
                if self._override_mode_index is not None and telemetry.status is not None:
                    observed_override = bool(telemetry.status.mode_flags & (1 << self._override_mode_index))
                    self._set_override_active_locked(observed_override, received_s)
                self._telemetry = telemetry
                self._telemetry_error = ""
                self._poll_errors.clear()
                self._telemetry_received_s = telemetry.timestamp
                if telemetry.rc_channels:
                    self._physical_rc_received_s = telemetry.timestamp
                    physical = reorder_msp_rc_to_set_raw_rc(
                        telemetry.rc_channels,
                        self.config.set_raw_rc_channel_map,
                    )
                    if not observed_override and self._physical_rc_valid(physical):
                        self._manual_rc = physical
                        self._override_released_s = None
                self._poll_count += 1
        except Exception as exc:
            with self._lock:
                self._telemetry_error = str(exc)
                self._poll_error_count += 1

    def _publish(self, now: float) -> None:
        with self._lock:
            if self._last_publish_tick_s is not None:
                interval_s = max(0.0, now - self._last_publish_tick_s)
                self._publish_tick_interval_s = interval_s
                self._publish_tick_max_interval_s = max(self._publish_tick_max_interval_s or 0.0, interval_s)
                if interval_s > 1.5 / self.config.control_publish_hz:
                    self._publish_deadline_miss_count += 1
            self._last_publish_tick_s = now
            command = self._staged
            command_received_s = self._staged_received_s
            output_enabled = self._output_enabled
            algorithm_authorized = self._algorithm_authorized
            override_active = self._override_active
            telemetry = self._telemetry
            manual_rc = self._manual_rc
            rc_age = None if self._physical_rc_received_s is None else now - self._physical_rc_received_s
            release_hold_active = self._override_release_hold_active_locked(now)
            fresh = bool(
                (override_active or release_hold_active) and manual_rc
                or rc_age is not None and rc_age <= self.config.physical_rc_timeout_s
            )
            prefill_ready = self._prefill_ready()
        last_ack_s = self.adapter.last_set_raw_rc_ack_monotonic_s()
        ack_age = (
            None
            if last_ack_s is None
            else max(0.0, now - last_ack_s)
        )
        ack_fresh = self.config.transport_mode != "async_pipeline" or (
            ack_age is not None and ack_age <= self.config.response_stale_s
        )
        if not output_enabled:
            with self._lock:
                self._send_skip_count += 1
                self._was_algorithm_authorized = False
                self._prefill_success_count = 0
                self._publish_mode = "disabled"
                self._publish_reason = "output_disabled"
                self._rc_source = "none"
                self._pilot_control_available = not override_active
                self._algorithm_release_active = False
                self._release_throttle_reference_us = None
                self._handover.clear()
                self._throttle_reference_us = None
                self._clear_throttle_slew()
            return
        if telemetry is None or not telemetry.rc_channels or not fresh:
            with self._lock:
                self._send_skip_count += 1
                self._was_algorithm_authorized = False
                self._prefill_success_count = 0
                self._publish_mode = "disabled"
                self._publish_reason = "physical_rc_stale"
                self._rc_source = "none"
                self._pilot_control_available = not override_active
                self._handover.clear()
                self._throttle_reference_us = None
                self._clear_throttle_slew()
            return
        try:
            physical = (
                tuple(manual_rc)
                if (override_active or release_hold_active) and manual_rc
                else reorder_msp_rc_to_set_raw_rc(
                    telemetry.rc_channels,
                    self.config.set_raw_rc_channel_map,
                )
            )
        except ValueError as exc:
            with self._lock:
                self._send_skip_count += 1
                self._prefill_success_count = 0
                self._publish_mode = "disabled"
                self._publish_reason = "channel_map_error"
                self._rc_source = "none"
                self._pilot_control_available = not override_active
                self._worker_error = str(exc)
                self._handover.clear()
                self._throttle_reference_us = None
                self._clear_throttle_slew()
            return
        if not self._physical_rc_valid(physical):
            with self._lock:
                self._send_skip_count += 1
                self._prefill_success_count = 0
                self._publish_mode = "disabled"
                self._publish_reason = "physical_rc_invalid"
                self._rc_source = "none"
                self._pilot_control_available = not override_active
                self._handover.clear()
                self._throttle_reference_us = None
                self._clear_throttle_slew()
            return
        if override_active and not manual_rc:
            with self._lock:
                self._send_skip_count += 1
                self._prefill_success_count = 0
                self._publish_mode = "disabled"
                self._publish_reason = "manual_rc_unavailable"
                self._rc_source = "none"
                self._pilot_control_available = False
                self._handover.clear()
                self._throttle_reference_us = None
                self._clear_throttle_slew()
            return
        command_age = None if command_received_s is None else max(0.0, now - command_received_s)
        command_fresh = command_age is not None and command_age <= self.config.staged_command_timeout_s
        use_algorithm = bool(
            algorithm_authorized
            and override_active
            and command is not None
            and command.active
            and command_fresh
            and (prefill_ready or not self.config.prefill_enabled)
            and ack_fresh
        )
        if algorithm_authorized and not command_fresh:
            with self._lock:
                self._stale_command_count += 1
        if use_algorithm and command is not None and len(physical) < len(command.channels):
            with self._lock:
                self._send_skip_count += 1
                self._publish_mode = "disabled"
                self._publish_reason = "channel_count_mismatch"
                self._rc_source = "none"
                self._pilot_control_available = not override_active
            return
        throttle_reference_invalid = False
        throttle = self.config.throttle_channel_zero_based
        handover_source = manual_rc if len(manual_rc) > throttle else physical
        if use_algorithm and not self._was_algorithm_authorized:
            candidate_reference = int(handover_source[throttle])
            if not self.config.throttle_reference_min_us <= candidate_reference <= self.config.throttle_reference_max_us:
                use_algorithm = False
                throttle_reference_invalid = True
            else:
                self._throttle_reference_us = candidate_reference
                self._release_throttle_reference_us = candidate_reference
                self._handover.reset(now, candidate_reference)
                self._reset_throttle_slew(now, candidate_reference)
        if use_algorithm and command is not None:
            channels = list(
                merge_physical_rc(
                    physical,
                    command.channels,
                    override_channels_mask=self.config.override_channels_mask,
                    aux_arm_channel_zero_based=self.config.aux_arm_channel_zero_based,
                )
            )
            requested_throttle = int(
                command.target_channels[throttle]
                if len(command.target_channels) > throttle
                else channels[throttle]
            )
            channels[throttle] = requested_throttle
            lower_limit_us = None
            upper_limit_us = None
            if self.config.throttle_relative_limit_us > 0:
                if self._throttle_reference_us is None:
                    raise RuntimeError("algorithm throttle reference is unavailable")
                lower_limit_us = max(
                    self.config.throttle_command_min_us,
                    self._throttle_reference_us - self.config.throttle_relative_limit_us,
                )
                upper_limit_us = min(
                    self.config.throttle_command_max_us,
                    self._throttle_reference_us + self.config.throttle_relative_limit_us,
                )
                channels[throttle] = min(
                    upper_limit_us,
                    max(lower_limit_us, requested_throttle),
                )
            handover_output_us = self._handover.apply(
                now,
                channels[throttle],
                requested_target_us=requested_throttle,
                lower_limit_us=lower_limit_us,
                upper_limit_us=upper_limit_us,
            )
            channels[throttle] = self._apply_throttle_slew(now, handover_output_us)
            publish_mode = "algorithm"
            publish_reason = "active"
            rc_source = "algorithm"
            self._algorithm_release_active = False
        elif self.config.prefill_enabled:
            if self._was_algorithm_authorized and override_active:
                self._algorithm_release_active = True
            if self._algorithm_release_active and (override_active or release_hold_active):
                channels = list(self._release_hold_channels(physical, now))
                publish_mode = "release_hold"
                publish_reason = "algorithm_released"
                rc_source = "synthesized_release_hold"
            else:
                channels = list(self._passthrough_channels(physical, manual_rc, override_active))
                if release_hold_active:
                    publish_mode = "release_hold"
                    publish_reason = "override_release_grace"
                    rc_source = "cached_pre_override"
                elif override_active:
                    publish_mode = "override_frozen_hold"
                    publish_reason = "algorithm_unavailable"
                    rc_source = "cached_pre_override"
                else:
                    publish_mode = "live_passthrough"
                    publish_reason = "prefill" if not prefill_ready else "manual"
                    rc_source = "live_msp_rc"
            if throttle_reference_invalid:
                publish_reason = "throttle_reference_out_of_range"
            elif not ack_fresh and prefill_ready:
                publish_reason = "set_ack_stale"
            self._handover.clear()
            if publish_mode != "release_hold":
                self._throttle_reference_us = None
                self._clear_throttle_slew()
            if not override_active and not release_hold_active:
                self._algorithm_release_active = False
                self._release_throttle_reference_us = None
        else:
            with self._lock:
                self._send_skip_count += 1
                self._was_algorithm_authorized = False
                self._publish_mode = "disabled"
                self._publish_reason = "algorithm_not_authorized"
                self._rc_source = "none"
                self._pilot_control_available = not override_active
                self._handover.clear()
                self._throttle_reference_us = None
                self._clear_throttle_slew()
            return
        try:
            request_id = None
            if self.config.transport_mode == "async_pipeline":
                request_id = self.adapter.write_raw_rc_async(tuple(channels))
            else:
                self.adapter.send_raw_rc(tuple(channels))
            sent_s = now
            if self.config.transport_mode == "async_pipeline":
                last_write_s = self.adapter.last_set_raw_rc_write_monotonic_s()
                if last_write_s is not None:
                    sent_s = last_write_s
            with self._lock:
                if self._last_send_success_s is not None:
                    send_interval_s = max(0.0, sent_s - self._last_send_success_s)
                    self._send_success_interval_s = send_interval_s
                    self._send_success_max_interval_s = max(
                        self._send_success_max_interval_s or 0.0,
                        send_interval_s,
                    )
                self._last_send_success_s = sent_s
                self._consecutive_send_error_count = 0
                self._worker_error = ""
                self._last_sent_channels = tuple(channels)
                self._publish_mode = publish_mode
                self._publish_reason = publish_reason
                self._rc_source = rc_source
                self._pilot_control_available = not override_active
                self._last_publish_output_enabled = output_enabled
                self._last_publish_algorithm_authorized = algorithm_authorized
                self._last_publish_override_active = override_active
                self._last_publish_override_release_hold_active = release_hold_active
                self._last_publish_prefill_ready = prefill_ready or not self.config.prefill_enabled
                self._last_publish_physical_rc_fresh = fresh
                self._last_publish_command_fresh = command_fresh
                self._last_publish_command_active = bool(command is not None and command.active)
                self._last_publish_command_reason = "" if command is None else str(command.reason)
                self._last_publish_set_raw_rc_ack_fresh = ack_fresh
                if use_algorithm:
                    self._algorithm_send_count += 1
                else:
                    self._passthrough_send_count += 1
                    if self.config.transport_mode != "async_pipeline":
                        self._prefill_success_count += 1
                if request_id is not None:
                    if len(self._async_set_writes) >= 4096:
                        self._async_set_writes.pop(min(self._async_set_writes), None)
                    self._async_set_writes[request_id] = (publish_mode, use_algorithm)
                self._was_algorithm_authorized = use_algorithm
        except Exception as exc:
            with self._lock:
                self._send_error_count += 1
                self._consecutive_send_error_count += 1
                self._worker_error = str(exc)
                self._was_algorithm_authorized = False
                self._prefill_success_count = 0

    def _reset_throttle_slew(self, timestamp: float, source_us: int) -> None:
        self._throttle_slew_output_us = int(source_us)
        self._throttle_slew_timestamp_s = float(timestamp)
        self._throttle_slew_limited = False

    def _clear_throttle_slew(self) -> None:
        self._throttle_slew_output_us = None
        self._throttle_slew_timestamp_s = None
        self._throttle_slew_limited = False

    def _apply_throttle_slew(self, timestamp: float, target_us: int) -> int:
        target = int(target_us)
        limit = float(self.config.throttle_slew_limit_us_per_s)
        previous = self._throttle_slew_output_us
        previous_s = self._throttle_slew_timestamp_s
        now = float(timestamp)
        if limit <= 0.0 or previous is None or previous_s is None or now < previous_s:
            output = target
        else:
            max_delta = limit * max(0.0, now - previous_s)
            output = int(
                round(min(previous + max_delta, max(previous - max_delta, target)))
            )
        self._throttle_slew_output_us = output
        self._throttle_slew_timestamp_s = now
        self._throttle_slew_limited = output != target
        return output

    def _prefill_ready(self) -> bool:
        return bool(not self.config.prefill_enabled or self._prefill_success_count >= self.config.prefill_min_frames)

    def _physical_rc_valid(self, channels: Sequence[int]) -> bool:
        if len(channels) < 4:
            return False
        return all(
            self.config.prefill_valid_min_us <= int(value) <= self.config.prefill_valid_max_us
            for value in channels[:4]
        )

    def _passthrough_channels(
        self,
        current_channels: Sequence[int],
        manual_channels: Sequence[int],
        override_active: bool,
    ) -> tuple[int, ...]:
        result = [int(value) for value in current_channels]
        if override_active and len(manual_channels) >= len(result):
            for index in range(len(result)):
                if self.config.override_channels_mask & (1 << index):
                    result[index] = int(manual_channels[index])
        return tuple(result)

    def _release_hold_channels(
        self,
        current_channels: Sequence[int],
        timestamp_s: float,
    ) -> tuple[int, ...]:
        """Stop body-rate commands while returning throttle to the entry reference."""

        result = [int(value) for value in current_channels]
        for role in ("A", "E", "R"):
            index = self.config.set_raw_rc_channel_map.index(role)
            if self.config.override_channels_mask & (1 << index):
                result[index] = 1500
        throttle = self.config.throttle_channel_zero_based
        if self.config.override_channels_mask & (1 << throttle):
            target = self._release_throttle_reference_us
            if target is None:
                target = int(result[throttle])
            result[throttle] = self._apply_throttle_slew(timestamp_s, int(target))
        return tuple(result)

    def metadata(self) -> dict[str, Any]:
        return {"config": asdict(self.config)}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
