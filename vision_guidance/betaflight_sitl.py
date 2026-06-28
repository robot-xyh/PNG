from __future__ import annotations

from dataclasses import dataclass, field
import math
import select
import socket
import struct
import time
from typing import Any, Iterable, Sequence

import numpy as np


FDM_PACKET_STRUCT = struct.Struct("<18d")
LEGACY_FDM_PACKET_STRUCT = struct.Struct("<17d")
RC_PACKET_STRUCT = struct.Struct("<d16H")
SERVO_PACKET_STRUCT = struct.Struct("<4f")
SERVO_RAW_PACKET_STRUCT = struct.Struct("<H2x16f")

DEFAULT_BETAFLIGHT_HOST = "127.0.0.1"
DEFAULT_PWM_RAW_PORT = 9001
DEFAULT_PWM_PORT = 9002
DEFAULT_STATE_PORT = 9003
DEFAULT_RC_PORT = 9004
DEFAULT_MSP_PORT = 5761

MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_MOTOR = 104
MSP_ATTITUDE = 108
MSP_SET_RAW_RC = 200
GRAVITY_MPS2 = 9.80665


@dataclass(frozen=True)
class BetaflightFdmPacket:
    timestamp_s: float
    imu_angular_velocity_rpy: np.ndarray
    imu_linear_acceleration_xyz: np.ndarray
    imu_orientation_quat_wxyz: np.ndarray
    velocity_xyz: np.ndarray
    position_xyz: np.ndarray
    pressure_pa: float


@dataclass(frozen=True)
class BetaflightServoPacket:
    motor_speed: tuple[float, float, float, float]


@dataclass(frozen=True)
class BetaflightServoRawPacket:
    motor_count: int
    pwm_output_raw: tuple[float, ...]


@dataclass(frozen=True)
class BetaflightMSPFrame:
    command: int
    payload: bytes
    direction: bytes = b">"


@dataclass(frozen=True)
class BetaflightMSPStatus:
    cycle_time_us: int
    i2c_error_count: int
    sensor_flags: int
    flight_mode_flags: int
    profile: int
    average_system_load_percent: float
    arming_disable_flags: int | None = None


@dataclass(frozen=True)
class BetaflightMSPAttitude:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class BetaflightMSPMotor:
    outputs_us: tuple[int, ...]


@dataclass(frozen=True)
class BetaflightRCCommand:
    roll: int = 1500
    pitch: int = 1500
    throttle: int = 1000
    yaw: int = 1500
    aux: tuple[int, ...] = (2000, 2000, 1000, 1000)

    def channels(self, total_channels: int = 16) -> tuple[int, ...]:
        if total_channels < 4:
            raise ValueError("total_channels must be at least 4")
        values = [self.roll, self.pitch, self.throttle, self.yaw, *self.aux]
        if len(values) < total_channels:
            values.extend([1000] * (total_channels - len(values)))
        return tuple(clamp_rc(value) for value in values[:total_channels])


@dataclass(frozen=True)
class RateCommand:
    roll_rate_rad_s: float
    pitch_rate_rad_s: float
    yaw_rate_rad_s: float = 0.0
    thrust_z: float = 0.0


@dataclass(frozen=True)
class BodyRateRCConfig:
    rc_min: int = 1000
    rc_mid: int = 1500
    rc_max: int = 2000
    max_roll_rate_rad_s: float = math.radians(200.0)
    max_pitch_rate_rad_s: float = math.radians(200.0)
    max_yaw_rate_rad_s: float = math.radians(200.0)
    roll_sign: float = 1.0
    pitch_sign: float = 1.0
    yaw_sign: float = 1.0
    min_throttle: int = 1000
    max_throttle: int = 2000
    arm_aux: int = 2000
    angle_aux: int = 1000
    extra_aux: tuple[int, ...] = field(default_factory=lambda: (1000, 1000))
    total_channels: int = 16


@dataclass(frozen=True)
class BodyRateRCResult:
    command: BetaflightRCCommand
    roll_norm: float
    pitch_norm: float
    yaw_norm: float
    throttle_norm: float


@dataclass(frozen=True)
class AngleRCConfig:
    max_tilt_deg: float = 25.0
    rc_full_scale_tilt_deg: float = 35.0
    hover_throttle: int = 1500
    min_throttle: int = 1100
    max_throttle: int = 1900
    roll_sign: float = 1.0
    pitch_sign: float = 1.0
    yaw_sign: float = 1.0
    yaw_full_scale_deg: float = 45.0
    max_yaw_rc_delta: float = 150.0
    yaw_deadband_deg: float = 2.0
    vertical_position_gain_rc_per_m: float = 8.0
    vertical_velocity_gain_rc_per_mps: float = 25.0
    vertical_accel_gain_rc_per_mps2: float = 25.0
    tilt_throttle_compensation: bool = True
    min_tilt_comp_cos: float = 0.65
    arm_aux: int = 2000
    angle_aux: int = 2000
    extra_aux: tuple[int, ...] = field(default_factory=lambda: (1000, 1000))


@dataclass(frozen=True)
class AngleRCResult:
    command: BetaflightRCCommand
    roll_target_deg: float
    pitch_target_deg: float
    yaw_error_deg: float
    throttle_delta: float
    forward_accel_mps2: float
    right_accel_mps2: float
    desired_tilt_deg: float
    tilt_scale: float
    vertical_accel_throttle_delta: float
    tilt_throttle_delta: float
    yaw_rc_delta: float


def _as_vector(value: Any, *, length: int, name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (length,):
        raise ValueError(f"{name} must be a {length}-vector, got shape {vector.shape}")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} contains non-finite values")
    return vector


def _vector_xyz(value: Any) -> np.ndarray:
    if value is None:
        return np.zeros(3, dtype=float)
    return np.array([float(value.x_val), float(value.y_val), float(value.z_val)], dtype=float)


def clamp_rc(value: float | int, minimum: int = 1000, maximum: int = 2000) -> int:
    return int(round(min(maximum, max(minimum, float(value)))))


def _rate_to_rc(rate_rad_s: float, max_rate_rad_s: float, sign: float, cfg: BodyRateRCConfig) -> tuple[int, float]:
    max_rate = max(1.0e-9, abs(float(max_rate_rad_s)))
    norm = float(np.clip(float(sign) * float(rate_rad_s) / max_rate, -1.0, 1.0))
    half_span = 0.5 * float(int(cfg.rc_max) - int(cfg.rc_min))
    rc = float(cfg.rc_mid) + norm * half_span
    return clamp_rc(rc, int(cfg.rc_min), int(cfg.rc_max)), norm


def body_rate_rc_from_rate_command(command: RateCommand, config: BodyRateRCConfig | None = None) -> BodyRateRCResult:
    cfg = config or BodyRateRCConfig()
    roll, roll_norm = _rate_to_rc(command.roll_rate_rad_s, cfg.max_roll_rate_rad_s, cfg.roll_sign, cfg)
    pitch, pitch_norm = _rate_to_rc(command.pitch_rate_rad_s, cfg.max_pitch_rate_rad_s, cfg.pitch_sign, cfg)
    yaw, yaw_norm = _rate_to_rc(command.yaw_rate_rad_s, cfg.max_yaw_rate_rad_s, cfg.yaw_sign, cfg)
    throttle_norm = float(np.clip(float(command.thrust_z), 0.0, 1.0))
    throttle = clamp_rc(
        float(cfg.min_throttle) + throttle_norm * float(int(cfg.max_throttle) - int(cfg.min_throttle)),
        int(cfg.min_throttle),
        int(cfg.max_throttle),
    )
    aux = (cfg.arm_aux, cfg.angle_aux, *cfg.extra_aux)
    rc_command = BetaflightRCCommand(
        roll=roll,
        pitch=pitch,
        throttle=throttle,
        yaw=yaw,
        aux=tuple(clamp_rc(value) for value in aux),
    )
    return BodyRateRCResult(
        command=rc_command,
        roll_norm=roll_norm,
        pitch_norm=pitch_norm,
        yaw_norm=yaw_norm,
        throttle_norm=throttle_norm,
    )


def rate_command_from_body_rate_output(body_rate_output: dict[str, Any]) -> RateCommand:
    rates = _as_vector(body_rate_output.get("body_rates_rad_s"), length=3, name="body_rates_rad_s")
    thrust = float(body_rate_output.get("thrust", 0.0))
    if not math.isfinite(thrust):
        thrust = 0.0
    return RateCommand(
        roll_rate_rad_s=float(rates[0]),
        pitch_rate_rad_s=float(rates[1]),
        yaw_rate_rad_s=float(rates[2]),
        thrust_z=float(np.clip(thrust, 0.0, 1.0)),
    )


def wrap_pi(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def pressure_from_ned_z(z_m: float) -> float:
    altitude_m = -float(z_m)
    base = max(0.01, 1.0 - 2.25577e-5 * altitude_m)
    return float(101325.0 * base**5.25588)


def yaw_from_quat_wxyz(quat_wxyz: Sequence[float]) -> float:
    quat = _as_vector(quat_wxyz, length=4, name="quat_wxyz")
    norm = float(np.linalg.norm(quat))
    if norm <= 1.0e-12:
        raise ValueError("quat_wxyz is near zero")
    w, x, y, z = quat / norm
    return float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def quat_multiply_wxyz(lhs: Sequence[float], rhs: Sequence[float]) -> np.ndarray:
    a = _as_vector(lhs, length=4, name="lhs")
    b = _as_vector(rhs, length=4, name="rhs")
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def gazebo_bridge_fdm_quat_from_airsim(quat_wxyz: Sequence[float]) -> np.ndarray:
    """Precondition AirSim NED quaternion for Betaflight SITL_GAZEBO input.

    SITL_GAZEBO pre-multiplies received orientation by Rz(+90 deg). Passing
    Rz(-90 deg) * q_airsim keeps Betaflight's internal attitude aligned with
    AirSim's NED attitude after that correction.
    """

    quat = _as_vector(quat_wxyz, length=4, name="quat_wxyz")
    norm = float(np.linalg.norm(quat))
    if norm <= 1.0e-12:
        raise ValueError("quat_wxyz is near zero")
    rz_minus_90 = np.array([math.sqrt(0.5), 0.0, 0.0, -math.sqrt(0.5)], dtype=float)
    out = quat_multiply_wxyz(rz_minus_90, quat / norm)
    return out / float(np.linalg.norm(out))


def pack_fdm_packet(packet: BetaflightFdmPacket, *, legacy_without_pressure: bool = False) -> bytes:
    angular = _as_vector(packet.imu_angular_velocity_rpy, length=3, name="imu_angular_velocity_rpy")
    accel = _as_vector(packet.imu_linear_acceleration_xyz, length=3, name="imu_linear_acceleration_xyz")
    quat = _as_vector(packet.imu_orientation_quat_wxyz, length=4, name="imu_orientation_quat_wxyz")
    velocity = _as_vector(packet.velocity_xyz, length=3, name="velocity_xyz")
    position = _as_vector(packet.position_xyz, length=3, name="position_xyz")
    fields = [
        float(packet.timestamp_s),
        *angular,
        *accel,
        *quat,
        *velocity,
        *position,
    ]
    if legacy_without_pressure:
        return LEGACY_FDM_PACKET_STRUCT.pack(*fields)
    return FDM_PACKET_STRUCT.pack(*fields, float(packet.pressure_pa))


def pack_rc_packet(timestamp_s: float, channels: Sequence[int]) -> bytes:
    values = list(channels)
    if len(values) > 16:
        raise ValueError("Betaflight SITL rc_packet accepts at most 16 channels")
    values.extend([1000] * (16 - len(values)))
    return RC_PACKET_STRUCT.pack(float(timestamp_s), *(clamp_rc(value) for value in values))


def unpack_servo_packet(data: bytes) -> BetaflightServoPacket:
    if len(data) != SERVO_PACKET_STRUCT.size:
        raise ValueError(f"servo_packet must be {SERVO_PACKET_STRUCT.size} bytes, got {len(data)}")
    return BetaflightServoPacket(tuple(float(value) for value in SERVO_PACKET_STRUCT.unpack(data)))


def unpack_servo_raw_packet(data: bytes) -> BetaflightServoRawPacket:
    if len(data) != SERVO_RAW_PACKET_STRUCT.size:
        raise ValueError(f"servo_packet_raw must be {SERVO_RAW_PACKET_STRUCT.size} bytes, got {len(data)}")
    motor_count, *outputs = SERVO_RAW_PACKET_STRUCT.unpack(data)
    return BetaflightServoRawPacket(int(motor_count), tuple(float(value) for value in outputs))


def betaflight_to_airsim_motor_order(motor_speed: Sequence[float]) -> tuple[float, float, float, float]:
    """Map Betaflight QUADX order to AirSim moveByMotorPWMsAsync order.

    Betaflight QUADX mixer order is rear-right, front-right, rear-left,
    front-left. AirSim expects front-right, rear-left, front-left, rear-right.
    """

    if len(motor_speed) != 4:
        raise ValueError("motor_speed must contain exactly four Betaflight motors")
    values = [float(value) for value in motor_speed]
    return tuple(float(np.clip(value, 0.0, 1.0)) for value in (values[1], values[2], values[3], values[0]))


def transform_betaflight_motor_output(
    motor_speed: Sequence[float],
    *,
    mode: str = "identity",
    gamma: float = 1.0,
    scale: float = 1.0,
    bias: float = 0.0,
) -> tuple[float, float, float, float]:
    """Transform Betaflight normalized motor output before applying it in AirSim.

    Betaflight's Gazebo bridge treats the normalized output as a rotor speed
    target. AirSim's motor PWM API treats the same 0..1 value as a direct
    thrust fraction, so this hook allows empirical equivalence calibration.
    """

    if len(motor_speed) != 4:
        raise ValueError("motor_speed must contain exactly four Betaflight motors")
    values = np.clip(np.asarray([float(value) for value in motor_speed], dtype=float), 0.0, 1.0)

    if mode == "identity":
        transformed = values
    elif mode == "sqrt":
        transformed = np.sqrt(values)
    elif mode == "gamma":
        if gamma <= 0.0:
            raise ValueError("gamma must be positive")
        transformed = np.power(values, float(gamma))
    elif mode == "scale_bias":
        transformed = values * float(scale) + float(bias)
    else:
        raise ValueError("mode must be one of 'identity', 'sqrt', 'gamma', or 'scale_bias'")

    return tuple(float(value) for value in np.clip(transformed, 0.0, 1.0))


def betaflight_to_airsim_motor_pwms(
    motor_speed: Sequence[float],
    *,
    transform: str = "identity",
    gamma: float = 1.0,
    scale: float = 1.0,
    bias: float = 0.0,
) -> tuple[float, float, float, float]:
    """Map Betaflight QUADX motor output to AirSim API order with calibration."""

    transformed = transform_betaflight_motor_output(
        motor_speed,
        mode=transform,
        gamma=gamma,
        scale=scale,
        bias=bias,
    )
    return betaflight_to_airsim_motor_order(transformed)


def encode_msp_v1(command: int, payload: bytes = b"", direction: bytes = b"<") -> bytes:
    if direction not in {b"<", b">", b"!"}:
        raise ValueError("MSP direction must be one of b'<', b'>', b'!'")
    if not 0 <= int(command) <= 255:
        raise ValueError("MSP v1 command must fit in one byte")
    if len(payload) > 255:
        raise ValueError("MSP v1 payload is limited to 255 bytes")
    checksum = len(payload) ^ int(command)
    for byte in payload:
        checksum ^= byte
    return b"$M" + direction + bytes([len(payload), int(command)]) + payload + bytes([checksum])


def decode_msp_v1_frame(data: bytes) -> BetaflightMSPFrame:
    if len(data) < 6:
        raise ValueError("MSP v1 frame is too short")
    if data[:2] != b"$M":
        raise ValueError("MSP v1 frame must start with b'$M'")
    direction = data[2:3]
    if direction not in {b"<", b">", b"!"}:
        raise ValueError("MSP v1 frame has invalid direction")
    size = int(data[3])
    expected_len = 6 + size
    if len(data) != expected_len:
        raise ValueError(f"MSP v1 frame length mismatch: expected {expected_len}, got {len(data)}")
    command = int(data[4])
    payload = data[5 : 5 + size]
    checksum = size ^ command
    for byte in payload:
        checksum ^= byte
    if checksum != data[-1]:
        raise ValueError("MSP v1 checksum mismatch")
    return BetaflightMSPFrame(command=command, payload=payload, direction=direction)


def encode_msp_set_raw_rc(channels: Sequence[int]) -> bytes:
    values = list(channels)
    if len(values) < 8:
        values.extend([1000] * (8 - len(values)))
    payload = struct.pack("<8H", *(clamp_rc(value) for value in values[:8]))
    return encode_msp_v1(MSP_SET_RAW_RC, payload)


def parse_msp_status(payload: bytes) -> BetaflightMSPStatus:
    if len(payload) < 11:
        raise ValueError(f"MSP_STATUS payload too short: {len(payload)}")
    cycle_time_us, i2c_error_count, sensor_flags, flight_mode_flags, profile, load_raw = struct.unpack_from("<HHHIBH", payload, 0)
    arming_disable_flags = struct.unpack_from("<I", payload, 16)[0] if len(payload) >= 20 else None
    return BetaflightMSPStatus(
        cycle_time_us=int(cycle_time_us),
        i2c_error_count=int(i2c_error_count),
        sensor_flags=int(sensor_flags),
        flight_mode_flags=int(flight_mode_flags),
        profile=int(profile),
        average_system_load_percent=float(load_raw) / 10.0,
        arming_disable_flags=None if arming_disable_flags is None else int(arming_disable_flags),
    )


def parse_msp_attitude(payload: bytes) -> BetaflightMSPAttitude:
    if len(payload) < 6:
        raise ValueError(f"MSP_ATTITUDE payload too short: {len(payload)}")
    roll_decideg, pitch_decideg, yaw_deg = struct.unpack_from("<hhh", payload, 0)
    return BetaflightMSPAttitude(
        roll_deg=float(roll_decideg) / 10.0,
        pitch_deg=float(pitch_decideg) / 10.0,
        yaw_deg=float(yaw_deg),
    )


def parse_msp_motor(payload: bytes) -> BetaflightMSPMotor:
    if len(payload) < 16:
        raise ValueError(f"MSP_MOTOR payload too short: {len(payload)}")
    return BetaflightMSPMotor(outputs_us=tuple(int(value) for value in struct.unpack_from("<8H", payload, 0)))


def fdm_packet_from_airsim(
    kinematics: Any,
    *,
    timestamp_s: float,
    position_xyz: np.ndarray | None = None,
    pressure_pa: float | None = None,
    frame_mode: str = "airsim_direct",
) -> BetaflightFdmPacket:
    position = _as_vector(position_xyz, length=3, name="position_xyz") if position_xyz is not None else _vector_xyz(kinematics.position)
    pressure = pressure_from_ned_z(float(position[2])) if pressure_pa is None else float(pressure_pa)
    orientation = getattr(kinematics, "orientation")
    angular = _vector_xyz(getattr(kinematics, "angular_velocity", None))
    quat = np.array(
        [
            float(orientation.w_val),
            float(orientation.x_val),
            float(orientation.y_val),
            float(orientation.z_val),
        ],
        dtype=float,
    )
    if frame_mode == "gazebo_bridge":
        angular = np.array([float(angular[0]), float(angular[1]), float(angular[2])], dtype=float)
        quat = gazebo_bridge_fdm_quat_from_airsim(quat)
    elif frame_mode != "airsim_direct":
        raise ValueError("frame_mode must be 'airsim_direct' or 'gazebo_bridge'")
    return BetaflightFdmPacket(
        timestamp_s=float(timestamp_s),
        imu_angular_velocity_rpy=angular,
        imu_linear_acceleration_xyz=_vector_xyz(getattr(kinematics, "linear_acceleration", None)),
        imu_orientation_quat_wxyz=quat,
        velocity_xyz=_vector_xyz(kinematics.linear_velocity),
        position_xyz=position,
        pressure_pa=pressure,
    )


class BetaflightMSPClient:
    def __init__(self, *, host: str = DEFAULT_BETAFLIGHT_HOST, port: int = DEFAULT_MSP_PORT, timeout_s: float = 0.05) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self.sock: socket.socket | None = None
        self._rx = bytearray()

    def connect(self) -> None:
        if self.sock is not None:
            return
        self.sock = socket.create_connection((self.host, self.port), timeout=max(0.001, self.timeout_s))
        self.sock.settimeout(max(0.001, self.timeout_s))

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def __enter__(self) -> "BetaflightMSPClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def request(self, command: int, *, timeout_s: float | None = None) -> BetaflightMSPFrame:
        self.connect()
        if self.sock is None:
            raise OSError("MSP socket is not connected")
        deadline = time.monotonic() + max(0.001, self.timeout_s if timeout_s is None else float(timeout_s))
        self.sock.sendall(encode_msp_v1(command))
        while True:
            frame = self._pop_frame()
            if frame is not None:
                if frame.command == int(command):
                    if frame.direction == b"!":
                        raise ValueError(f"MSP command {command} returned an error frame")
                    return frame
                continue

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f"timed out waiting for MSP command {command}")
            self.sock.settimeout(remaining)
            chunk = self.sock.recv(256)
            if not chunk:
                raise ConnectionError("MSP socket closed")
            self._rx.extend(chunk)

    def send_raw_rc(self, channels: Sequence[int]) -> None:
        self.connect()
        if self.sock is None:
            raise OSError("MSP socket is not connected")
        self.sock.sendall(encode_msp_set_raw_rc(channels))

    def send_rc_command(self, command: BetaflightRCCommand, *, total_channels: int = 16) -> None:
        self.send_raw_rc(command.channels(total_channels))

    def send_rate_command(self, command: RateCommand, config: BodyRateRCConfig | None = None) -> BodyRateRCResult:
        result = body_rate_rc_from_rate_command(command, config)
        total_channels = (config or BodyRateRCConfig()).total_channels
        self.send_rc_command(result.command, total_channels=total_channels)
        return result

    def read_status(self, *, timeout_s: float | None = None) -> BetaflightMSPStatus:
        return parse_msp_status(self.request(MSP_STATUS, timeout_s=timeout_s).payload)

    def read_attitude(self, *, timeout_s: float | None = None) -> BetaflightMSPAttitude:
        return parse_msp_attitude(self.request(MSP_ATTITUDE, timeout_s=timeout_s).payload)

    def read_motor(self, *, timeout_s: float | None = None) -> BetaflightMSPMotor:
        return parse_msp_motor(self.request(MSP_MOTOR, timeout_s=timeout_s).payload)

    def _pop_frame(self) -> BetaflightMSPFrame | None:
        while True:
            start = self._rx.find(b"$M")
            if start < 0:
                self._rx.clear()
                return None
            if start > 0:
                del self._rx[:start]
            if len(self._rx) < 6:
                return None

            size = int(self._rx[3])
            frame_len = 6 + size
            if len(self._rx) < frame_len:
                return None

            raw = bytes(self._rx[:frame_len])
            del self._rx[:frame_len]
            try:
                return decode_msp_v1_frame(raw)
            except ValueError:
                continue


def angle_rc_from_png_accel(
    acceleration_ned: Sequence[float],
    relative_position_ned: Sequence[float],
    relative_velocity_ned: Sequence[float],
    current_yaw_rad: float,
    config: AngleRCConfig | None = None,
) -> AngleRCResult:
    cfg = config or AngleRCConfig()
    accel = _as_vector(acceleration_ned, length=3, name="acceleration_ned")
    rel_pos = _as_vector(relative_position_ned, length=3, name="relative_position_ned")
    rel_vel = _as_vector(relative_velocity_ned, length=3, name="relative_velocity_ned")

    yaw = float(current_yaw_rad)
    forward = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=float)
    right = np.array([-math.sin(yaw), math.cos(yaw), 0.0], dtype=float)
    forward_accel = float(np.dot(accel, forward))
    right_accel = float(np.dot(accel, right))

    max_tilt = max(1.0e-6, abs(float(cfg.max_tilt_deg)))
    raw_pitch_target = math.degrees(math.atan2(forward_accel, GRAVITY_MPS2))
    raw_roll_target = math.degrees(math.atan2(right_accel, GRAVITY_MPS2))
    raw_tilt = math.hypot(raw_roll_target, raw_pitch_target)
    tilt_scale = 1.0 if raw_tilt <= max_tilt or raw_tilt <= 1.0e-9 else max_tilt / raw_tilt
    pitch_target = float(raw_pitch_target * tilt_scale)
    roll_target = float(raw_roll_target * tilt_scale)
    desired_tilt = float(math.hypot(roll_target, pitch_target))

    rc_full_scale_tilt = max(max_tilt, abs(float(cfg.rc_full_scale_tilt_deg)), 1.0e-6)
    roll = 1500 + float(cfg.roll_sign) * roll_target / rc_full_scale_tilt * 500.0
    pitch = 1500 + float(cfg.pitch_sign) * pitch_target / rc_full_scale_tilt * 500.0

    target_heading = math.atan2(float(rel_pos[1]), float(rel_pos[0]))
    yaw_error = wrap_pi(target_heading - yaw)
    yaw_full_scale = max(1.0e-6, math.radians(abs(float(cfg.yaw_full_scale_deg))))
    yaw_deadband = math.radians(max(0.0, abs(float(cfg.yaw_deadband_deg))))
    yaw_scale = 0.0 if abs(yaw_error) <= yaw_deadband else float(np.clip(yaw_error / yaw_full_scale, -1.0, 1.0))
    max_yaw_delta = max(0.0, abs(float(cfg.max_yaw_rc_delta)))
    yaw_rc_delta = float(cfg.yaw_sign) * yaw_scale * max_yaw_delta
    yaw_rc = 1500 + yaw_rc_delta

    vertical_pd_delta = (
        -float(cfg.vertical_position_gain_rc_per_m) * float(rel_pos[2])
        - float(cfg.vertical_velocity_gain_rc_per_mps) * float(rel_vel[2])
    )
    vertical_accel_delta = -float(cfg.vertical_accel_gain_rc_per_mps2) * float(accel[2])
    tilt_throttle_delta = 0.0
    if bool(cfg.tilt_throttle_compensation):
        min_cos = float(np.clip(abs(float(cfg.min_tilt_comp_cos)), 1.0e-6, 1.0))
        tilt_cos = max(min_cos, math.cos(math.radians(desired_tilt)))
        tilt_throttle_delta = float(cfg.hover_throttle) * (1.0 / tilt_cos - 1.0)
    throttle_delta = vertical_pd_delta + vertical_accel_delta + tilt_throttle_delta
    throttle = float(cfg.hover_throttle) + throttle_delta
    aux = (cfg.arm_aux, cfg.angle_aux, *cfg.extra_aux)
    command = BetaflightRCCommand(
        roll=clamp_rc(roll),
        pitch=clamp_rc(pitch),
        throttle=clamp_rc(throttle, cfg.min_throttle, cfg.max_throttle),
        yaw=clamp_rc(yaw_rc),
        aux=tuple(clamp_rc(value) for value in aux),
    )
    return AngleRCResult(
        command=command,
        roll_target_deg=roll_target,
        pitch_target_deg=pitch_target,
        yaw_error_deg=math.degrees(yaw_error),
        throttle_delta=float(throttle_delta),
        forward_accel_mps2=forward_accel,
        right_accel_mps2=right_accel,
        desired_tilt_deg=desired_tilt,
        tilt_scale=float(tilt_scale),
        vertical_accel_throttle_delta=float(vertical_accel_delta),
        tilt_throttle_delta=float(tilt_throttle_delta),
        yaw_rc_delta=float(yaw_rc_delta),
    )


class BetaflightSITLBridge:
    def __init__(
        self,
        *,
        host: str = DEFAULT_BETAFLIGHT_HOST,
        bind_host: str = "0.0.0.0",
        state_port: int = DEFAULT_STATE_PORT,
        rc_port: int = DEFAULT_RC_PORT,
        pwm_port: int = DEFAULT_PWM_PORT,
        pwm_raw_port: int | None = None,
        legacy_fdm_without_pressure: bool = False,
    ) -> None:
        self.host = host
        self.pwm_port = int(pwm_port)
        self.state_endpoint = (host, int(state_port))
        self.rc_endpoint = (host, int(rc_port))
        self.legacy_fdm_without_pressure = bool(legacy_fdm_without_pressure)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pwm_rx = self._bind_udp(bind_host, self.pwm_port)
        self.pwm_raw_rx = self._bind_udp(bind_host, int(pwm_raw_port)) if pwm_raw_port is not None else None
        self.latest_motor_speed: tuple[float, float, float, float] | None = None
        self.latest_raw_pwm: BetaflightServoRawPacket | None = None
        self.latest_motor_time_s: float | None = None

    @staticmethod
    def _bind_udp(host: str, port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
        sock.setblocking(False)
        return sock

    def close(self) -> None:
        for sock in (self.tx, self.pwm_rx, self.pwm_raw_rx):
            if sock is not None:
                sock.close()

    def __enter__(self) -> "BetaflightSITLBridge":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def send_fdm(self, packet: BetaflightFdmPacket) -> None:
        data = pack_fdm_packet(packet, legacy_without_pressure=self.legacy_fdm_without_pressure)
        self.tx.sendto(data, self.state_endpoint)

    def send_rc(self, timestamp_s: float, channels: Sequence[int]) -> None:
        self.tx.sendto(pack_rc_packet(timestamp_s, channels), self.rc_endpoint)

    def send_rc_command(self, timestamp_s: float, command: BetaflightRCCommand) -> None:
        self.send_rc(timestamp_s, command.channels())

    def poll_motor_output(self, timeout_s: float = 0.0) -> tuple[float, float, float, float] | None:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        sockets = [self.pwm_rx]
        if self.pwm_raw_rx is not None:
            sockets.append(self.pwm_raw_rx)

        while True:
            wait = max(0.0, deadline - time.monotonic()) if timeout_s > 0.0 else 0.0
            readable, _, _ = select.select(sockets, [], [], wait)
            for sock in readable:
                while True:
                    try:
                        data, _addr = sock.recvfrom(4096)
                    except BlockingIOError:
                        break
                    if sock is self.pwm_rx:
                        try:
                            packet = unpack_servo_packet(data)
                        except ValueError:
                            continue
                        self.latest_motor_speed = packet.motor_speed
                        self.latest_motor_time_s = time.monotonic()
                    elif self.pwm_raw_rx is not None and sock is self.pwm_raw_rx:
                        try:
                            self.latest_raw_pwm = unpack_servo_raw_packet(data)
                        except ValueError:
                            continue
            if self.latest_motor_speed is not None or timeout_s <= 0.0 or time.monotonic() >= deadline:
                return self.latest_motor_speed

    def await_motor_output(self, timeout_s: float) -> tuple[float, float, float, float]:
        motor_speed = self.poll_motor_output(timeout_s)
        if motor_speed is None:
            raise TimeoutError(f"no Betaflight motor output received on UDP {self.pwm_port} within {timeout_s:.1f}s")
        return motor_speed


__all__ = [
    "AngleRCConfig",
    "AngleRCResult",
    "BodyRateRCConfig",
    "BodyRateRCResult",
    "BetaflightFdmPacket",
    "BetaflightMSPAttitude",
    "BetaflightMSPClient",
    "BetaflightMSPFrame",
    "BetaflightMSPMotor",
    "BetaflightMSPStatus",
    "BetaflightRCCommand",
    "BetaflightSITLBridge",
    "BetaflightServoPacket",
    "BetaflightServoRawPacket",
    "FDM_PACKET_STRUCT",
    "LEGACY_FDM_PACKET_STRUCT",
    "RC_PACKET_STRUCT",
    "SERVO_PACKET_STRUCT",
    "SERVO_RAW_PACKET_STRUCT",
    "angle_rc_from_png_accel",
    "betaflight_to_airsim_motor_pwms",
    "betaflight_to_airsim_motor_order",
    "body_rate_rc_from_rate_command",
    "decode_msp_v1_frame",
    "encode_msp_set_raw_rc",
    "encode_msp_v1",
    "fdm_packet_from_airsim",
    "gazebo_bridge_fdm_quat_from_airsim",
    "pack_fdm_packet",
    "pack_rc_packet",
    "parse_msp_attitude",
    "parse_msp_motor",
    "parse_msp_status",
    "pressure_from_ned_z",
    "quat_multiply_wxyz",
    "rate_command_from_body_rate_output",
    "RateCommand",
    "transform_betaflight_motor_output",
    "unpack_servo_packet",
    "unpack_servo_raw_packet",
    "wrap_pi",
    "yaw_from_quat_wxyz",
]
