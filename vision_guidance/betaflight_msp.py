from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from .flight_control import RcCommand
from .geometry import rotation_x, rotation_y, rotation_z


MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_STATUS = 101
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ANALOG = 110
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
    def R_IB(self) -> np.ndarray:
        return attitude_degrees_to_R_IB(self.roll_deg, self.pitch_deg, self.yaw_deg)


@dataclass(frozen=True)
class AnalogTelemetry:
    vbat_v: float
    mah_drawn: int | None = None
    rssi: int | None = None
    amperage_a: float | None = None


@dataclass(frozen=True)
class BetaflightTelemetry:
    timestamp: float
    status: StatusTelemetry | None = None
    attitude: AttitudeTelemetry | None = None
    analog: AnalogTelemetry | None = None
    rc_channels: tuple[int, ...] = ()


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
    size = int(raw[3])
    expected_len = 6 + size
    if len(raw) != expected_len:
        raise MSPError("MSP frame length mismatch")
    command = int(raw[4])
    payload = raw[5 : 5 + size]
    checksum = int(raw[-1])
    expected = _msp_checksum(size, command, payload)
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


def parse_rc_channels(payload: bytes | bytearray) -> tuple[int, ...]:
    data = bytes(payload)
    if len(data) < 2 or len(data) % 2 != 0:
        raise MSPError("MSP_RC payload length must be an even number of bytes")
    return tuple(int(value) for value in struct.unpack("<" + "H" * (len(data) // 2), data))


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
    roll = np.deg2rad(float(roll_deg))
    pitch = np.deg2rad(float(pitch_deg))
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
        if self.transport is not None and self._owns_transport:
            close = getattr(self.transport, "close", None)
            if callable(close):
                close()
        self.transport = None
        self._owns_transport = False

    def request(self, command: int, payload: bytes | bytearray = b"") -> MSPFrame:
        self.open()
        assert self.transport is not None
        self.transport.write(encode_msp_frame(command, payload, direction="<"))
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

    def read_api_version(self) -> ApiVersion:
        return parse_api_version(self.request(MSP_API_VERSION).payload)

    def read_fc_variant(self) -> str:
        return parse_fc_variant(self.request(MSP_FC_VARIANT).payload)

    def read_fc_version(self) -> FcVersion:
        return parse_fc_version(self.request(MSP_FC_VERSION).payload)

    def read_status(self) -> StatusTelemetry:
        return parse_status(self.request(MSP_STATUS).payload)

    def read_attitude(self) -> AttitudeTelemetry:
        return parse_attitude(self.request(MSP_ATTITUDE).payload)

    def read_analog(self) -> AnalogTelemetry:
        return parse_analog(self.request(MSP_ANALOG).payload)

    def read_rc(self) -> tuple[int, ...]:
        return parse_rc_channels(self.request(MSP_RC).payload)

    def read_telemetry(
        self,
        *,
        include_status: bool = True,
        include_attitude: bool = True,
        include_analog: bool = True,
        include_rc: bool = True,
    ) -> BetaflightTelemetry:
        status = self.read_status() if include_status else None
        attitude = self.read_attitude() if include_attitude else None
        analog = self.read_analog() if include_analog else None
        rc = self.read_rc() if include_rc else ()
        return BetaflightTelemetry(
            timestamp=time.monotonic(),
            status=status,
            attitude=attitude,
            analog=analog,
            rc_channels=tuple(rc),
        )

    def send_raw_rc(self, command: RcCommand | Sequence[int]) -> None:
        channels = command.channels if isinstance(command, RcCommand) else tuple(int(v) for v in command)
        self.request(MSP_SET_RAW_RC, pack_rc_channels(channels))

    def send_rc(self, command: RcCommand) -> None:
        self.send_raw_rc(command)

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
            payload = self.transport.read(size)
            checksum = self.transport.read(1)
            if len(payload) != size or len(checksum) != 1:
                return None
            return decode_msp_frame(b"$M" + direction + size_raw + command_raw + payload + checksum)
        return None


def _msp_checksum(size: int, command: int, payload: bytes) -> int:
    checksum = int(size) ^ int(command)
    for value in payload:
        checksum ^= int(value)
    return checksum & 0xFF
