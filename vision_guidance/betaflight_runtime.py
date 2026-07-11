from __future__ import annotations

import hashlib
import json
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence

from .betaflight_msp import BetaflightMSPAdapter, BetaflightTelemetry, MspAdapterStats
from .flight_control import RcCommand


MSP_OVERRIDE_PERMANENT_ID = 50


@dataclass(frozen=True)
class ControlAuthorizationStatus:
    approved: bool
    reason: str
    approval_path: str = ""
    snapshot_path: str = ""
    snapshot_sha256: str = ""
    config_conflict_free: bool = False


@dataclass(frozen=True)
class MspRuntimeConfig:
    io_worker_enabled: bool = False
    telemetry_poll_hz: float = 5.0
    control_publish_hz: float = 50.0
    physical_rc_timeout_s: float = 0.25
    override_channels_mask: int = 15
    aux_arm_channel_zero_based: int = 4
    throttle_channel_zero_based: int = 2
    throttle_handover_s: float = 0.4

    @classmethod
    def from_mapping(cls, values: dict[str, Any]) -> "MspRuntimeConfig":
        config = cls(
            io_worker_enabled=bool(values.get("io_worker_enabled", False)),
            telemetry_poll_hz=float(values.get("telemetry_poll_hz", 5.0)),
            control_publish_hz=float(values.get("control_publish_hz", 50.0)),
            physical_rc_timeout_s=float(values.get("physical_rc_timeout_s", 0.25)),
            override_channels_mask=int(values.get("override_channels_mask", 15)),
            aux_arm_channel_zero_based=int(values.get("aux_arm_channel_zero_based", 4)),
            throttle_channel_zero_based=int(values.get("throttle_channel_zero_based", 2)),
            throttle_handover_s=float(values.get("throttle_handover_s", 0.4)),
        )
        if config.telemetry_poll_hz <= 0.0 or config.control_publish_hz <= 0.0:
            raise ValueError("MSP worker rates must be positive")
        if config.physical_rc_timeout_s <= 0.0 or config.throttle_handover_s < 0.0:
            raise ValueError("MSP worker timeout/handover values are invalid")
        return config


@dataclass(frozen=True)
class MspWorkerSnapshot:
    telemetry: BetaflightTelemetry | None
    telemetry_error: str
    telemetry_age_s: float | None
    physical_rc_age_s: float | None
    physical_rc_fresh: bool
    poll_count: int
    poll_error_count: int
    staged_count: int
    send_skip_count: int
    send_error_count: int
    worker_error: str
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
    if approval.get("source_conflicts_resolved") is not True:
        return ControlAuthorizationStatus(False, "source_conflicts_unresolved", str(approval_path))
    snapshot_path = Path(str(approval.get("snapshot_manifest", ""))).expanduser()
    if not snapshot_path.is_file():
        return ControlAuthorizationStatus(False, "snapshot_manifest_missing", str(approval_path), str(snapshot_path))
    actual_sha = _sha256(snapshot_path)
    expected_sha = str(approval.get("snapshot_sha256", ""))
    if not expected_sha or actual_sha != expected_sha:
        return ControlAuthorizationStatus(
            False, "snapshot_sha256_mismatch", str(approval_path), str(snapshot_path), actual_sha
        )
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
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
    if not str(approval.get("parameters_sha256", "")):
        return ControlAuthorizationStatus(
            False, "parameters_sha256_missing", str(approval_path), str(snapshot_path), actual_sha
        )
    return ControlAuthorizationStatus(
        True,
        "approved",
        str(approval_path.resolve()),
        str(snapshot_path.resolve()),
        actual_sha,
        True,
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


class ThrottleHandover:
    def __init__(self, duration_s: float):
        if duration_s < 0.0:
            raise ValueError("duration_s must be non-negative")
        self.duration_s = float(duration_s)
        self._start_s: float | None = None
        self._from_us = 1000

    def reset(self, timestamp: float, physical_throttle_us: int) -> None:
        self._start_s = float(timestamp)
        self._from_us = int(physical_throttle_us)

    def apply(self, timestamp: float, target_us: int) -> int:
        if self._start_s is None or self.duration_s <= 0.0:
            return int(target_us)
        alpha = min(1.0, max(0.0, (float(timestamp) - self._start_s) / self.duration_s))
        return int(round((1.0 - alpha) * self._from_us + alpha * int(target_us)))


class BetaflightMspIoWorker:
    def __init__(self, adapter: BetaflightMSPAdapter, config: MspRuntimeConfig):
        self.adapter = adapter
        self.config = config
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._telemetry: BetaflightTelemetry | None = None
        self._telemetry_error = ""
        self._telemetry_received_s: float | None = None
        self._physical_rc_received_s: float | None = None
        self._staged: RcCommand | None = None
        self._authorized = False
        self._poll_count = 0
        self._poll_error_count = 0
        self._staged_count = 0
        self._send_skip_count = 0
        self._send_error_count = 0
        self._worker_error = ""
        self._handover = ThrottleHandover(config.throttle_handover_s)
        self._was_authorized = False

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
        self._thread = None

    def stage(self, command: RcCommand | None, *, authorized: bool) -> None:
        with self._lock:
            self._staged = command
            self._authorized = bool(authorized)
            self._staged_count += 1

    def snapshot(self, timestamp: float | None = None) -> MspWorkerSnapshot:
        now = time.monotonic() if timestamp is None else float(timestamp)
        with self._lock:
            telemetry_age = None if self._telemetry_received_s is None else max(0.0, now - self._telemetry_received_s)
            rc_age = None if self._physical_rc_received_s is None else max(0.0, now - self._physical_rc_received_s)
            return MspWorkerSnapshot(
                telemetry=self._telemetry,
                telemetry_error=self._telemetry_error,
                telemetry_age_s=telemetry_age,
                physical_rc_age_s=rc_age,
                physical_rc_fresh=rc_age is not None and rc_age <= self.config.physical_rc_timeout_s,
                poll_count=self._poll_count,
                poll_error_count=self._poll_error_count,
                staged_count=self._staged_count,
                send_skip_count=self._send_skip_count,
                send_error_count=self._send_error_count,
                worker_error=self._worker_error,
                adapter_stats=self.adapter.snapshot_stats(),
            )

    def _run(self) -> None:
        poll_period = 1.0 / self.config.telemetry_poll_hz
        publish_period = 1.0 / self.config.control_publish_hz
        next_poll = 0.0
        next_publish = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_poll:
                self._poll(now)
                next_poll = now + poll_period
            if now >= next_publish:
                self._publish(now)
                next_publish = now + publish_period
            self._stop.wait(min(0.005, max(0.0, min(next_poll, next_publish) - time.monotonic())))

    def _poll(self, now: float) -> None:
        try:
            telemetry = self.adapter.read_telemetry()
            with self._lock:
                self._telemetry = telemetry
                self._telemetry_error = ""
                self._telemetry_received_s = telemetry.timestamp
                if telemetry.rc_channels:
                    self._physical_rc_received_s = telemetry.timestamp
                self._poll_count += 1
        except Exception as exc:
            with self._lock:
                self._telemetry_error = str(exc)
                self._poll_error_count += 1

    def _publish(self, now: float) -> None:
        with self._lock:
            command = self._staged
            authorized = self._authorized
            telemetry = self._telemetry
            rc_age = None if self._physical_rc_received_s is None else now - self._physical_rc_received_s
            fresh = rc_age is not None and rc_age <= self.config.physical_rc_timeout_s
        if not authorized or command is None or telemetry is None or not telemetry.rc_channels or not fresh:
            with self._lock:
                self._send_skip_count += 1
                self._was_authorized = False
            return
        physical = telemetry.rc_channels
        if len(physical) < len(command.channels):
            with self._lock:
                self._send_skip_count += 1
            return
        channels = list(
            merge_physical_rc(
                physical,
                command.channels,
                override_channels_mask=self.config.override_channels_mask,
                aux_arm_channel_zero_based=self.config.aux_arm_channel_zero_based,
            )
        )
        throttle = self.config.throttle_channel_zero_based
        if not self._was_authorized:
            self._handover.reset(now, physical[throttle])
        channels[throttle] = self._handover.apply(now, channels[throttle])
        try:
            self.adapter.send_raw_rc(tuple(channels))
            self._was_authorized = True
        except Exception as exc:
            with self._lock:
                self._send_error_count += 1
                self._worker_error = str(exc)
                self._was_authorized = False

    def metadata(self) -> dict[str, Any]:
        return {"config": asdict(self.config)}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
