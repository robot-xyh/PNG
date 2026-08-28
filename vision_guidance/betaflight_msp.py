from __future__ import annotations

import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from .flight_control import RcCommand
from .geometry import rotation_x, rotation_y, rotation_z


MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ANALOG = 110
MSP_BOXNAMES = 116
MSP_BOXIDS = 119
MSP_SET_RAW_RC = 200


class MSPError(RuntimeError):
    pass


@dataclass(frozen=True)
class MSPFrame:
    direction: str
    command: int
    payload: bytes


@dataclass(frozen=True)
class ApiVersion:
    protocol_version: int
    api_major: int
    api_minor: int


@dataclass(frozen=True)
class FcVersion:
    major: int
    minor: int
    patch: int


@dataclass(frozen=True)
class StatusTelemetry:
    cycle_time_us: int
    i2c_error_count: int
    sensor_flags: int
    mode_flags: int
    profile: int | None = None


@dataclass(frozen=True)
class AttitudeTelemetry:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float

    @property
    def pitch_nose_up_deg(self) -> float:
        """Return FRD/NED pitch, where positive pitch raises the nose."""

        return -self.pitch_deg

    @property
    def euler_frd_deg(self) -> tuple[float, float, float]:
        """Return roll/pitch/yaw using the FRD body-to-NED convention."""

        return self.roll_deg, self.pitch_nose_up_deg, self.yaw_deg

    @property
    def R_IB(self) -> np.ndarray:
        return attitude_degrees_to_R_IB(self.roll_deg, self.pitch_deg, self.yaw_deg)


@dataclass(frozen=True)
class AnalogTelemetry:
    vbat_v: float
    mah_drawn: int | None = None
    rssi: int | None = None
    amperage_a: float | None = None


@dataclass(frozen=True)
class RawImuTelemetry:
    acc_raw: tuple[int, int, int]
    gyro_msp_raw: tuple[int, int, int]
    mag_raw: tuple[int, int, int]


@dataclass(frozen=True)
class BetaflightTelemetry:
    timestamp: float
    status: StatusTelemetry | None = None
    attitude: AttitudeTelemetry | None = None
    analog: AnalogTelemetry | None = None
    rc_channels: tuple[int, ...] = ()
    raw_imu: RawImuTelemetry | None = None
    status_timestamp_s: float | None = None
    attitude_timestamp_s: float | None = None
    analog_timestamp_s: float | None = None
    rc_timestamp_s: float | None = None
    raw_imu_timestamp_s: float | None = None


@dataclass(frozen=True)
class MspCommandStats:
    command: int
    attempt_count: int = 0
    success_count: int = 0
    error_count: int = 0
    last_rtt_ms: float | None = None
    max_rtt_ms: float | None = None
    last_success_monotonic_s: float | None = None
    last_error: str = ""


@dataclass(frozen=True)
class AsyncMspResponse:
    frame: MSPFrame
    request_id: int | None
    request_monotonic_s: float | None
    response_monotonic_s: float


@dataclass(frozen=True)
class _PendingAsyncRequest:
    request_id: int
    command: int
    sent_monotonic_s: float


@dataclass
class _MutableMspCommandStats:
    attempt_count: int = 0
    success_count: int = 0
    error_count: int = 0
    last_rtt_ms: float | None = None
    max_rtt_ms: float | None = None
    last_success_monotonic_s: float | None = None
    last_error: str = ""


@dataclass(frozen=True)
class MspAdapterStats:
    request_count: int = 0
    request_error_count: int = 0
    tx_bytes: int = 0
    rx_bytes: int = 0
    set_raw_rc_attempt_count: int = 0
    set_raw_rc_success_count: int = 0
    set_raw_rc_write_attempt_count: int = 0
    set_raw_rc_write_success_count: int = 0
    set_raw_rc_write_error_count: int = 0
    set_raw_rc_ack_count: int = 0
    set_raw_rc_pending_depth: int = 0
    set_raw_rc_last_write_monotonic_s: float | None = None
    set_raw_rc_last_ack_monotonic_s: float | None = None
    set_raw_rc_write_interval_s: float | None = None
    set_raw_rc_write_max_interval_s: float | None = None
    set_raw_rc_write_rate_hz: float | None = None
    set_raw_rc_write_p50_interval_s: float | None = None
    set_raw_rc_write_p95_interval_s: float | None = None
    set_raw_rc_write_p99_interval_s: float | None = None
    set_raw_rc_write_p999_interval_s: float | None = None
    async_pending_telemetry_count: int = 0
    rx_discarded_bytes: int = 0
    rx_checksum_error_count: int = 0
    rx_parser_error_count: int = 0
    command_stats: tuple[MspCommandStats, ...] = ()

    def for_command(self, command: int) -> MspCommandStats | None:
        return next((item for item in self.command_stats if item.command == int(command)), None)


def encode_msp_frame(command: int, payload: bytes | bytearray = b"", direction: str = "<") -> bytes:
    payload_bytes = bytes(payload)
    if len(payload_bytes) > 255:
        raise ValueError("MSP v1 payload cannot exceed 255 bytes")
    if direction not in {"<", ">", "!"}:
        raise ValueError("invalid MSP direction")
    command = int(command)
    if command < 0 or command > 255:
        raise ValueError("MSP v1 command must fit in one byte")
    size = len(payload_bytes)
    checksum = _msp_checksum(size, command, payload_bytes)
    return b"$M" + direction.encode("ascii") + bytes([size, command]) + payload_bytes + bytes([checksum])


def decode_msp_frame(data: bytes | bytearray) -> MSPFrame:
    raw = bytes(data)
    if len(raw) < 6:
        raise MSPError("MSP frame too short")
    if raw[:2] != b"$M":
        raise MSPError("invalid MSP header")
    direction = chr(raw[2])
    if direction not in {"<", ">", "!"}:
        raise MSPError("invalid MSP direction")
    size_marker = int(raw[3])
    payload_offset = 5
    checksum_payload = raw[5:-1]
    if size_marker == 255:
        if len(raw) < 8:
            raise MSPError("MSP jumbo frame too short")
        size = struct.unpack_from("<H", raw, 5)[0]
        payload_offset = 7
    else:
        size = size_marker
    expected_len = payload_offset + size + 1
    if len(raw) != expected_len:
        raise MSPError("MSP frame length mismatch")
    command = int(raw[4])
    payload = raw[payload_offset : payload_offset + size]
    checksum = int(raw[-1])
    expected = _msp_checksum(size_marker, command, checksum_payload)
    if checksum != expected:
        raise MSPError("MSP checksum mismatch")
    return MSPFrame(direction=direction, command=command, payload=payload)


def parse_api_version(payload: bytes | bytearray) -> ApiVersion:
    data = bytes(payload)
    if len(data) < 3:
        raise MSPError("MSP_API_VERSION payload too short")
    return ApiVersion(data[0], data[1], data[2])


def parse_fc_variant(payload: bytes | bytearray) -> str:
    data = bytes(payload)
    if len(data) < 4:
        raise MSPError("MSP_FC_VARIANT payload too short")
    return data[:4].decode("ascii", errors="replace")


def parse_fc_version(payload: bytes | bytearray) -> FcVersion:
    data = bytes(payload)
    if len(data) < 3:
        raise MSPError("MSP_FC_VERSION payload too short")
    return FcVersion(data[0], data[1], data[2])


def parse_status(payload: bytes | bytearray) -> StatusTelemetry:
    data = bytes(payload)
    if len(data) < 10:
        raise MSPError("MSP_STATUS payload too short")
    cycle_time_us, i2c_error_count, sensor_flags, mode_flags = struct.unpack_from("<HHHI", data, 0)
    profile = data[10] if len(data) > 10 else None
    return StatusTelemetry(cycle_time_us, i2c_error_count, sensor_flags, mode_flags, profile)


def parse_attitude(payload: bytes | bytearray) -> AttitudeTelemetry:
    data = bytes(payload)
    if len(data) < 6:
        raise MSPError("MSP_ATTITUDE payload too short")
    roll_decideg, pitch_decideg, heading_deg = struct.unpack_from("<hhh", data, 0)
    return AttitudeTelemetry(
        roll_deg=float(roll_decideg) / 10.0,
        pitch_deg=float(pitch_decideg) / 10.0,
        yaw_deg=float(heading_deg),
    )


def parse_analog(payload: bytes | bytearray) -> AnalogTelemetry:
    data = bytes(payload)
    if len(data) < 1:
        raise MSPError("MSP_ANALOG payload too short")
    vbat_v = float(data[0]) / 10.0
    mah_drawn = struct.unpack_from("<H", data, 1)[0] if len(data) >= 3 else None
    rssi = struct.unpack_from("<H", data, 3)[0] if len(data) >= 5 else None
    amperage_a = float(struct.unpack_from("<h", data, 5)[0]) / 100.0 if len(data) >= 7 else None
    return AnalogTelemetry(vbat_v=vbat_v, mah_drawn=mah_drawn, rssi=rssi, amperage_a=amperage_a)


def parse_raw_imu(payload: bytes | bytearray) -> RawImuTelemetry:
    data = bytes(payload)
    if len(data) < 18:
        raise MSPError("MSP_RAW_IMU payload too short")
    values = struct.unpack_from("<9h", data, 0)
    return RawImuTelemetry(
        acc_raw=tuple(int(value) for value in values[0:3]),
        gyro_msp_raw=tuple(int(value) for value in values[3:6]),
        mag_raw=tuple(int(value) for value in values[6:9]),
    )


def parse_rc_channels(payload: bytes | bytearray) -> tuple[int, ...]:
    data = bytes(payload)
    if len(data) < 2 or len(data) % 2 != 0:
        raise MSPError("MSP_RC payload length must be an even number of bytes")
    return tuple(int(value) for value in struct.unpack("<" + "H" * (len(data) // 2), data))


def parse_box_ids(payload: bytes | bytearray) -> tuple[int, ...]:
    data = bytes(payload)
    if not data:
        raise MSPError("MSP_BOXIDS payload must not be empty")
    return tuple(int(value) for value in data)


def parse_box_names(payload: bytes | bytearray) -> tuple[str, ...]:
    data = bytes(payload)
    if not data:
        raise MSPError("MSP_BOXNAMES payload must not be empty")
    names = tuple(name for name in data.decode("ascii", errors="replace").split(";") if name)
    if not names:
        raise MSPError("MSP_BOXNAMES payload contains no names")
    return names


def pack_rc_channels(channels: Sequence[int]) -> bytes:
    if not channels:
        raise ValueError("channels must not be empty")
    values = []
    for channel in channels:
        value = int(channel)
        if value < 750 or value > 2250:
            raise ValueError(f"RC channel out of sane range: {value}")
        values.append(value)
    return struct.pack("<" + "H" * len(values), *values)


def attitude_degrees_to_R_IB(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Convert raw MSP_ATTITUDE Euler angles to an FRD body-to-NED rotation.

    MSP_ATTITUDE reports pitch with the Betaflight display convention: raising
    the nose makes pitch negative. FRD/NED right-handed pitch is positive for
    the same motion, so only pitch is negated here. Roll and heading already
    match the FRD/NED convention.
    """

    roll = np.deg2rad(float(roll_deg))
    pitch = -np.deg2rad(float(pitch_deg))
    yaw = np.deg2rad(float(yaw_deg))
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)


class BetaflightMSPAdapter:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        *,
        timeout_s: float = 0.2,
        transport: Any = None,
    ) -> None:
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.transport = transport
        self._owns_transport = False
        self._io_lock = threading.Lock()
        self._request_count = 0
        self._request_error_count = 0
        self._tx_bytes = 0
        self._rx_bytes = 0
        self._set_raw_rc_attempt_count = 0
        self._set_raw_rc_success_count = 0
        self._set_raw_rc_write_attempt_count = 0
        self._set_raw_rc_write_success_count = 0
        self._set_raw_rc_write_error_count = 0
        self._set_raw_rc_ack_count = 0
        self._set_raw_rc_last_write_s: float | None = None
        self._set_raw_rc_last_ack_s: float | None = None
        self._set_raw_rc_write_interval_s: float | None = None
        self._set_raw_rc_write_max_interval_s: float | None = None
        self._set_raw_rc_write_times: deque[float] = deque(maxlen=4096)
        self._set_raw_rc_write_intervals: deque[float] = deque(maxlen=4096)
        self._async_active = False
        self._async_original_timeout: float | None = None
        self._async_rx_buffer = bytearray()
        self._async_request_sequence = 0
        self._async_pending_telemetry: dict[int, _PendingAsyncRequest] = {}
        self._async_pending_set_raw_rc: deque[_PendingAsyncRequest] = deque(maxlen=4096)
        self._rx_discarded_bytes = 0
        self._rx_checksum_error_count = 0
        self._rx_parser_error_count = 0
        self._command_stats: dict[int, _MutableMspCommandStats] = {}

    def open(self) -> None:
        if self.transport is not None:
            return
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("pyserial is required for Betaflight MSP. Install with: python3 -m pip install pyserial") from exc
        self.transport = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout_s,
            write_timeout=self.timeout_s,
        )
        self._owns_transport = True

    def close(self) -> None:
        self.end_async_pipeline()
        if self.transport is not None and self._owns_transport:
            close = getattr(self.transport, "close", None)
            if callable(close):
                close()
        self.transport = None
        self._owns_transport = False

    def request(self, command: int, payload: bytes | bytearray = b"") -> MSPFrame:
        with self._io_lock:
            if self._async_active:
                raise RuntimeError("synchronous MSP request is unavailable while async pipeline is active")
            command = int(command)
            started_s = time.monotonic()
            command_stats = self._command_stats.setdefault(command, _MutableMspCommandStats())
            command_stats.attempt_count += 1
            self._request_count += 1
            if command == MSP_SET_RAW_RC:
                self._set_raw_rc_attempt_count += 1
                self._set_raw_rc_write_attempt_count += 1
            request_frame = encode_msp_frame(command, payload, direction="<")
            self._tx_bytes += len(request_frame)
            try:
                response = self._request_locked(command, payload, request_frame)
            except Exception as exc:
                completed_s = time.monotonic()
                rtt_ms = 1000.0 * max(0.0, completed_s - started_s)
                command_stats.error_count += 1
                command_stats.last_rtt_ms = rtt_ms
                command_stats.max_rtt_ms = max(command_stats.max_rtt_ms or 0.0, rtt_ms)
                command_stats.last_error = f"{type(exc).__name__}: {exc}"
                self._request_error_count += 1
                if command == MSP_SET_RAW_RC:
                    self._set_raw_rc_write_error_count += 1
                raise
            completed_s = time.monotonic()
            rtt_ms = 1000.0 * max(0.0, completed_s - started_s)
            command_stats.success_count += 1
            command_stats.last_rtt_ms = rtt_ms
            command_stats.max_rtt_ms = max(command_stats.max_rtt_ms or 0.0, rtt_ms)
            command_stats.last_success_monotonic_s = completed_s
            command_stats.last_error = ""
            self._rx_bytes += 6 + len(response.payload)
            if command == MSP_SET_RAW_RC:
                self._set_raw_rc_success_count += 1
                self._set_raw_rc_write_success_count += 1
                self._set_raw_rc_ack_count += 1
                self._record_set_raw_rc_write(completed_s)
                self._set_raw_rc_last_ack_s = completed_s
            return response

    def _request_locked(
        self,
        command: int,
        payload: bytes | bytearray,
        request_frame: bytes,
    ) -> MSPFrame:
        self.open()
        assert self.transport is not None
        self.transport.write(request_frame)
        flush = getattr(self.transport, "flush", None)
        if callable(flush):
            flush()
        deadline = time.monotonic() + max(0.01, self.timeout_s)
        while time.monotonic() <= deadline:
            frame = self._read_frame(deadline)
            if frame is None:
                continue
            if frame.direction == "!":
                raise MSPError(f"Betaflight returned MSP error for command {frame.command}")
            if frame.direction == ">" and frame.command == int(command):
                return frame
        raise TimeoutError(f"timed out waiting for MSP command {command}")

    def snapshot_stats(self) -> MspAdapterStats:
        with self._io_lock:
            intervals = tuple(self._set_raw_rc_write_intervals)
            write_times = tuple(self._set_raw_rc_write_times)
            return MspAdapterStats(
                request_count=self._request_count,
                request_error_count=self._request_error_count,
                tx_bytes=self._tx_bytes,
                rx_bytes=self._rx_bytes,
                set_raw_rc_attempt_count=self._set_raw_rc_attempt_count,
                set_raw_rc_success_count=self._set_raw_rc_success_count,
                set_raw_rc_write_attempt_count=self._set_raw_rc_write_attempt_count,
                set_raw_rc_write_success_count=self._set_raw_rc_write_success_count,
                set_raw_rc_write_error_count=self._set_raw_rc_write_error_count,
                set_raw_rc_ack_count=self._set_raw_rc_ack_count,
                set_raw_rc_pending_depth=len(self._async_pending_set_raw_rc),
                set_raw_rc_last_write_monotonic_s=self._set_raw_rc_last_write_s,
                set_raw_rc_last_ack_monotonic_s=self._set_raw_rc_last_ack_s,
                set_raw_rc_write_interval_s=self._set_raw_rc_write_interval_s,
                set_raw_rc_write_max_interval_s=self._set_raw_rc_write_max_interval_s,
                set_raw_rc_write_rate_hz=_sample_rate_hz(write_times),
                set_raw_rc_write_p50_interval_s=_percentile(intervals, 50.0),
                set_raw_rc_write_p95_interval_s=_percentile(intervals, 95.0),
                set_raw_rc_write_p99_interval_s=_percentile(intervals, 99.0),
                set_raw_rc_write_p999_interval_s=_percentile(intervals, 99.9),
                async_pending_telemetry_count=len(self._async_pending_telemetry),
                rx_discarded_bytes=self._rx_discarded_bytes,
                rx_checksum_error_count=self._rx_checksum_error_count,
                rx_parser_error_count=self._rx_parser_error_count,
                command_stats=tuple(
                    MspCommandStats(
                        command=command,
                        attempt_count=stats.attempt_count,
                        success_count=stats.success_count,
                        error_count=stats.error_count,
                        last_rtt_ms=stats.last_rtt_ms,
                        max_rtt_ms=stats.max_rtt_ms,
                        last_success_monotonic_s=stats.last_success_monotonic_s,
                        last_error=stats.last_error,
                    )
                    for command, stats in sorted(self._command_stats.items())
                ),
            )

    def read_api_version(self) -> ApiVersion:
        return parse_api_version(self.request(MSP_API_VERSION).payload)

    def read_fc_variant(self) -> str:
        return parse_fc_variant(self.request(MSP_FC_VARIANT).payload)

    def read_fc_version(self) -> FcVersion:
        return parse_fc_version(self.request(MSP_FC_VERSION).payload)

    def read_status(self) -> StatusTelemetry:
        return parse_status(self.request(MSP_STATUS).payload)

    def read_raw_imu(self) -> RawImuTelemetry:
        return parse_raw_imu(self.request(MSP_RAW_IMU).payload)

    def read_attitude(self) -> AttitudeTelemetry:
        return parse_attitude(self.request(MSP_ATTITUDE).payload)

    def read_analog(self) -> AnalogTelemetry:
        return parse_analog(self.request(MSP_ANALOG).payload)

    def read_rc(self) -> tuple[int, ...]:
        return parse_rc_channels(self.request(MSP_RC).payload)

    def read_box_ids(self) -> tuple[int, ...]:
        return parse_box_ids(self.request(MSP_BOXIDS).payload)

    def read_box_names(self) -> tuple[str, ...]:
        return parse_box_names(self.request(MSP_BOXNAMES).payload)

    def read_telemetry(
        self,
        *,
        include_status: bool = True,
        include_attitude: bool = True,
        include_analog: bool = True,
        include_rc: bool = True,
        include_raw_imu: bool = False,
    ) -> BetaflightTelemetry:
        status = self.read_status() if include_status else None
        attitude = self.read_attitude() if include_attitude else None
        analog = self.read_analog() if include_analog else None
        rc = self.read_rc() if include_rc else ()
        raw_imu = self.read_raw_imu() if include_raw_imu else None
        timestamp = time.monotonic()
        return BetaflightTelemetry(
            timestamp=timestamp,
            status=status,
            attitude=attitude,
            analog=analog,
            rc_channels=tuple(rc),
            raw_imu=raw_imu,
            status_timestamp_s=timestamp if status is not None else None,
            attitude_timestamp_s=timestamp if attitude is not None else None,
            analog_timestamp_s=timestamp if analog is not None else None,
            rc_timestamp_s=timestamp if rc else None,
            raw_imu_timestamp_s=timestamp if raw_imu is not None else None,
        )

    def send_raw_rc(self, command: RcCommand | Sequence[int]) -> None:
        channels = command.channels if isinstance(command, RcCommand) else tuple(int(v) for v in command)
        self.request(MSP_SET_RAW_RC, pack_rc_channels(channels))

    def send_rc(self, command: RcCommand) -> None:
        self.send_raw_rc(command)

    def begin_async_pipeline(self) -> None:
        with self._io_lock:
            self.open()
            if self._async_active:
                return
            assert self.transport is not None
            if hasattr(self.transport, "timeout"):
                self._async_original_timeout = getattr(self.transport, "timeout")
                setattr(self.transport, "timeout", 0)
            self._async_active = True

    def end_async_pipeline(self) -> None:
        with self._io_lock:
            if not self._async_active:
                return
            if self.transport is not None and hasattr(self.transport, "timeout"):
                setattr(self.transport, "timeout", self._async_original_timeout)
            self._async_active = False
            self._async_original_timeout = None

    def write_raw_rc_async(self, command: RcCommand | Sequence[int]) -> int:
        channels = command.channels if isinstance(command, RcCommand) else tuple(int(v) for v in command)
        with self._io_lock:
            self._require_async_pipeline()
            request = self._write_async_request_locked(MSP_SET_RAW_RC, pack_rc_channels(channels))
            if len(self._async_pending_set_raw_rc) == self._async_pending_set_raw_rc.maxlen:
                self._async_pending_set_raw_rc.popleft()
                self._record_command_error_locked(MSP_SET_RAW_RC, "async SET_RAW_RC pending FIFO overflow")
            self._async_pending_set_raw_rc.append(request)
            return request.request_id

    def queue_async_request(self, command: int) -> int | None:
        command = int(command)
        if command == MSP_SET_RAW_RC:
            raise ValueError("use write_raw_rc_async for MSP_SET_RAW_RC")
        with self._io_lock:
            self._require_async_pipeline()
            if command in self._async_pending_telemetry:
                return None
            request = self._write_async_request_locked(command, b"")
            self._async_pending_telemetry[command] = request
            return request.request_id

    def async_request_pending(self, command: int) -> bool:
        with self._io_lock:
            return int(command) in self._async_pending_telemetry

    def expire_async_requests(self, timeout_s: float, timestamp: float | None = None) -> tuple[int, ...]:
        now = time.monotonic() if timestamp is None else float(timestamp)
        timeout = max(0.001, float(timeout_s))
        with self._io_lock:
            expired = tuple(
                command
                for command, request in self._async_pending_telemetry.items()
                if now - request.sent_monotonic_s > timeout
            )
            for command in expired:
                self._async_pending_telemetry.pop(command, None)
                self._record_command_error_locked(command, "async response timeout")
            return expired

    def drain_async_responses(self, budget_ms: float = 3.0) -> tuple[AsyncMspResponse, ...]:
        deadline = time.monotonic() + max(0.0, float(budget_ms)) / 1000.0
        responses: list[AsyncMspResponse] = []
        while True:
            with self._io_lock:
                self._require_async_pipeline()
                responses.extend(self._extract_async_responses_locked())
                chunk = self._read_async_available_locked()
                if chunk:
                    self._rx_bytes += len(chunk)
                    self._async_rx_buffer.extend(chunk)
                    responses.extend(self._extract_async_responses_locked())
            if time.monotonic() >= deadline:
                break
            if not chunk:
                time.sleep(min(0.0002, max(0.0, deadline - time.monotonic())))
        return tuple(responses)

    def _write_async_request_locked(self, command: int, payload: bytes) -> _PendingAsyncRequest:
        assert self.transport is not None
        command = int(command)
        frame = encode_msp_frame(command, payload, direction="<")
        stats = self._command_stats.setdefault(command, _MutableMspCommandStats())
        stats.attempt_count += 1
        self._request_count += 1
        if command == MSP_SET_RAW_RC:
            self._set_raw_rc_attempt_count += 1
            self._set_raw_rc_write_attempt_count += 1
        self._tx_bytes += len(frame)
        try:
            written = self.transport.write(frame)
            if written is not None and int(written) != len(frame):
                raise OSError(f"partial MSP write: {written}/{len(frame)} bytes")
        except Exception as exc:
            stats.error_count += 1
            stats.last_error = f"{type(exc).__name__}: {exc}"
            self._request_error_count += 1
            if command == MSP_SET_RAW_RC:
                self._set_raw_rc_write_error_count += 1
            raise
        sent_s = time.monotonic()
        self._async_request_sequence += 1
        if command == MSP_SET_RAW_RC:
            self._set_raw_rc_write_success_count += 1
            self._record_set_raw_rc_write(sent_s)
        return _PendingAsyncRequest(self._async_request_sequence, command, sent_s)

    def _read_async_available_locked(self) -> bytes:
        assert self.transport is not None
        waiting_value = getattr(self.transport, "in_waiting", None)
        if waiting_value is not None:
            waiting = int(waiting_value)
            return b"" if waiting <= 0 else bytes(self.transport.read(min(waiting, 4096)))
        return bytes(self.transport.read(4096))

    def _extract_async_responses_locked(self) -> list[AsyncMspResponse]:
        responses: list[AsyncMspResponse] = []
        while True:
            header = self._async_rx_buffer.find(b"$M")
            if header < 0:
                keep = 1 if self._async_rx_buffer.endswith(b"$") else 0
                discarded = len(self._async_rx_buffer) - keep
                if discarded > 0:
                    del self._async_rx_buffer[:discarded]
                    self._rx_discarded_bytes += discarded
                break
            if header > 0:
                del self._async_rx_buffer[:header]
                self._rx_discarded_bytes += header
            if len(self._async_rx_buffer) < 6:
                break
            if self._async_rx_buffer[2] not in (ord(">"), ord("!")):
                del self._async_rx_buffer[0]
                self._rx_discarded_bytes += 1
                self._rx_parser_error_count += 1
                continue
            size_marker = int(self._async_rx_buffer[3])
            payload_offset = 5
            if size_marker == 255:
                if len(self._async_rx_buffer) < 8:
                    break
                size = struct.unpack_from("<H", self._async_rx_buffer, 5)[0]
                payload_offset = 7
            else:
                size = size_marker
            frame_size = payload_offset + size + 1
            if len(self._async_rx_buffer) < frame_size:
                break
            raw = bytes(self._async_rx_buffer[:frame_size])
            try:
                frame = decode_msp_frame(raw)
            except MSPError as exc:
                del self._async_rx_buffer[0]
                self._rx_discarded_bytes += 1
                if "checksum" in str(exc):
                    self._rx_checksum_error_count += 1
                else:
                    self._rx_parser_error_count += 1
                continue
            del self._async_rx_buffer[:frame_size]
            now = time.monotonic()
            pending: _PendingAsyncRequest | None
            if frame.command == MSP_SET_RAW_RC:
                pending = self._async_pending_set_raw_rc.popleft() if self._async_pending_set_raw_rc else None
            else:
                pending = self._async_pending_telemetry.pop(frame.command, None)
            if pending is not None:
                self._record_async_response_locked(frame, pending, now)
            responses.append(
                AsyncMspResponse(
                    frame=frame,
                    request_id=None if pending is None else pending.request_id,
                    request_monotonic_s=None if pending is None else pending.sent_monotonic_s,
                    response_monotonic_s=now,
                )
            )
        return responses

    def _record_async_response_locked(
        self,
        frame: MSPFrame,
        pending: _PendingAsyncRequest,
        timestamp: float,
    ) -> None:
        stats = self._command_stats.setdefault(frame.command, _MutableMspCommandStats())
        rtt_ms = 1000.0 * max(0.0, timestamp - pending.sent_monotonic_s)
        stats.last_rtt_ms = rtt_ms
        stats.max_rtt_ms = max(stats.max_rtt_ms or 0.0, rtt_ms)
        if frame.direction == "!":
            stats.error_count += 1
            stats.last_error = f"Betaflight returned MSP error for command {frame.command}"
            self._request_error_count += 1
            return
        stats.success_count += 1
        stats.last_success_monotonic_s = timestamp
        stats.last_error = ""
        if frame.command == MSP_SET_RAW_RC:
            self._set_raw_rc_success_count += 1
            self._set_raw_rc_ack_count += 1
            self._set_raw_rc_last_ack_s = timestamp

    def _record_command_error_locked(self, command: int, message: str) -> None:
        stats = self._command_stats.setdefault(int(command), _MutableMspCommandStats())
        stats.error_count += 1
        stats.last_error = str(message)
        self._request_error_count += 1

    def _record_set_raw_rc_write(self, timestamp: float) -> None:
        if self._set_raw_rc_last_write_s is not None:
            interval = max(0.0, timestamp - self._set_raw_rc_last_write_s)
            self._set_raw_rc_write_interval_s = interval
            self._set_raw_rc_write_max_interval_s = max(
                self._set_raw_rc_write_max_interval_s or 0.0,
                interval,
            )
            self._set_raw_rc_write_intervals.append(interval)
        self._set_raw_rc_last_write_s = timestamp
        self._set_raw_rc_write_times.append(timestamp)

    def _require_async_pipeline(self) -> None:
        if not self._async_active or self.transport is None:
            raise RuntimeError("MSP async pipeline is not active")

    def _read_frame(self, deadline: float) -> MSPFrame | None:
        assert self.transport is not None
        while time.monotonic() <= deadline:
            byte = self.transport.read(1)
            if not byte:
                time.sleep(0.001)
                continue
            if byte != b"$":
                continue
            marker = self.transport.read(1)
            if marker != b"M":
                continue
            direction = self.transport.read(1)
            size_raw = self.transport.read(1)
            command_raw = self.transport.read(1)
            if len(direction) != 1 or len(size_raw) != 1 or len(command_raw) != 1:
                return None
            size = int(size_raw[0])
            jumbo_size_raw = b""
            if size == 255:
                jumbo_size_raw = self.transport.read(2)
                if len(jumbo_size_raw) != 2:
                    return None
                size = struct.unpack("<H", jumbo_size_raw)[0]
            payload = self.transport.read(size)
            checksum = self.transport.read(1)
            if len(payload) != size or len(checksum) != 1:
                return None
            return decode_msp_frame(
                b"$M" + direction + size_raw + command_raw + jumbo_size_raw + payload + checksum
            )
        return None


def _msp_checksum(size: int, command: int, payload: bytes) -> int:
    checksum = int(size) ^ int(command)
    for value in payload:
        checksum ^= int(value)
    return checksum & 0xFF


def _sample_rate_hz(timestamps: Sequence[float]) -> float | None:
    if len(timestamps) < 2:
        return None
    duration = float(timestamps[-1]) - float(timestamps[0])
    return None if duration <= 0.0 else float(len(timestamps) - 1) / duration


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    rank = max(0, min(len(ordered) - 1, int(np.ceil(float(percentile) * len(ordered) / 100.0)) - 1))
    return ordered[rank]
