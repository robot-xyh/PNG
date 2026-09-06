from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import select
import socket
import struct
import threading
import time
from typing import Any, Sequence

import numpy as np

from .types import CameraIntrinsics, FrameDetection


FDM_PACKET_STRUCT = struct.Struct("<18d")
RC_PACKET_STRUCT = struct.Struct("<d16H")
SERVO_PACKET_STRUCT = struct.Struct("<4f")
SERVO_RAW_PACKET_STRUCT = struct.Struct("<H2x16f")

DEFAULT_STATE_PORT = 9003
DEFAULT_RC_PORT = 9004
DEFAULT_PWM_PORT = 9002
DEFAULT_PWM_RAW_PORT = 9001

SITL_SCOPE = "betaflight_sitl_loopback_v1"
SITL_MSP_URL = "socket://127.0.0.1:5761"
SITL_AUDIT_EVIDENCE_TYPE = "betaflight_gazebo_sil_audit"
SITL_RUN_EVIDENCE_TYPE = "betaflight_gazebo_sil_run"
SITL_REQUIRED_DETECTOR_MODES = frozenset({"projected", "rendered"})
SITL_OFFICIAL_BETAFLIGHT_COMMIT = "79065c96ba0bb5cdc675e67d7093e05dab8b330e"
SITL_OFFICIAL_BETAFLIGHT_ELF_SHA256 = (
    "f4e4456aae4f079d1349dc7bc4037211897260eeeb8cc9c4e5691949996212be"
)
SITL_REQUIRED_ARTIFACTS = frozenset(
    {
        "flight_config",
        "sitl_config",
        "configuration_manifest",
        "betaflight_binary",
        "betaflight_cli",
        "eeprom",
        "gazebo_world",
        "gazebo_bridge_source",
        "gazebo_bridge_library",
        "interceptor_model",
        "target_model",
        "runner_csv",
        "runner_meta",
        "runner_manifest",
        "betaflight_console",
        "gazebo_console",
        "runner_console",
    }
)


def _evidence_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _evidence_mapping(value: object, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise RuntimeError(f"{label} must be an object")
    return value


def _read_evidence_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"invalid JSON evidence file: {path}") from exc
    if not isinstance(value, dict):
        raise RuntimeError(f"JSON evidence file must contain an object: {path}")
    return value


def validate_sitl_audit_evidence(
    reports: list[tuple[dict[str, Any], Path]],
    *,
    runtime_config_sha256: str,
    expected_engagement_policy: str,
    repository_commit: str,
    official_betaflight_commit: str | None = None,
    official_betaflight_elf_sha256: str | None = None,
) -> list[dict[str, Any]]:
    """Validate and summarize the two policy-bound Gazebo SIL audits."""

    official_betaflight_commit = (
        SITL_OFFICIAL_BETAFLIGHT_COMMIT
        if official_betaflight_commit is None
        else official_betaflight_commit
    )
    official_betaflight_elf_sha256 = (
        SITL_OFFICIAL_BETAFLIGHT_ELF_SHA256
        if official_betaflight_elf_sha256 is None
        else official_betaflight_elf_sha256
    )
    if expected_engagement_policy not in {"noncollision", "contact"}:
        raise RuntimeError("unsupported SIL engagement policy")
    if len(reports) != len(SITL_REQUIRED_DETECTOR_MODES):
        raise RuntimeError("SIL evidence requires exactly projected and rendered audits")

    validated: dict[str, dict[str, Any]] = {}
    for report, raw_report_path in reports:
        report_path = raw_report_path.expanduser().resolve()
        if not report_path.is_file():
            raise RuntimeError(f"SIL audit is missing: {report_path}")
        if _read_evidence_json(report_path) != report:
            raise RuntimeError("SIL audit object does not match its bound file")
        detector_mode = str(report.get("detector_mode", ""))
        if detector_mode not in SITL_REQUIRED_DETECTOR_MODES:
            raise RuntimeError("SIL detector mode must be projected or rendered")
        if detector_mode in validated:
            raise RuntimeError(f"duplicate SIL detector mode: {detector_mode}")
        if (
            report.get("schema_version") != 1
            or report.get("evidence_type") != SITL_AUDIT_EVIDENCE_TYPE
            or report.get("scope") != SITL_SCOPE
            or report.get("passed") is not True
            or report.get("violations") != []
        ):
            raise RuntimeError("SIL evidence must be a passing schema v1 audit")
        if report.get("hardware_authorization") is not False:
            raise RuntimeError("SIL evidence must declare hardware_authorization=false")
        if report.get("detector_representative") is not False:
            raise RuntimeError("synthetic SIL must not claim representative detector evidence")
        if report.get("policy") != expected_engagement_policy:
            raise RuntimeError("SIL evidence engagement policy mismatch")

        betaflight = _evidence_mapping(
            report.get("betaflight_binding"), "SIL Betaflight binding"
        )
        if (
            betaflight.get("source_commit") != official_betaflight_commit
            or betaflight.get("elf_sha256") != official_betaflight_elf_sha256
        ):
            raise RuntimeError(
                "SIL evidence does not use the approved official Betaflight build"
            )

        software = _evidence_mapping(
            report.get("software_binding"), "SIL software binding"
        )
        if (
            software.get("repository_commit") != repository_commit
            or software.get("repository_dirty") is not False
        ):
            raise RuntimeError("SIL evidence must bind the exact clean approval commit")
        candidate = _evidence_mapping(
            report.get("flight_candidate_binding"), "SIL flight candidate binding"
        )
        if (
            candidate.get("sha256") != runtime_config_sha256
            or candidate.get("engagement_policy") != expected_engagement_policy
        ):
            raise RuntimeError("SIL evidence runtime config SHA256 or policy mismatch")
        metrics = _evidence_mapping(report.get("metrics"), "SIL metrics")
        terminal = _evidence_mapping(metrics.get("terminal"), "SIL terminal metrics")
        if terminal.get("passed") is not True:
            raise RuntimeError("SIL evidence did not exercise the terminal policy")
        if detector_mode == "rendered" and int(
            metrics.get("rendered_detection_count", 0)
        ) <= 0:
            raise RuntimeError("rendered SIL evidence contains no detections")

        orchestration = _evidence_mapping(
            report.get("orchestration_manifest"), "SIL orchestration binding"
        )
        orchestration_path = Path(str(orchestration.get("path", ""))).expanduser().resolve()
        if (
            not orchestration_path.is_file()
            or _evidence_sha256(orchestration_path) != orchestration.get("sha256")
        ):
            raise RuntimeError("SIL orchestration manifest changed or is missing")
        manifest = _read_evidence_json(orchestration_path)
        if (
            manifest.get("schema_version") != 1
            or manifest.get("evidence_type") != SITL_RUN_EVIDENCE_TYPE
            or manifest.get("scope") != SITL_SCOPE
            or manifest.get("completed") is not True
            or manifest.get("failure") not in {"", None}
            or manifest.get("policy") != expected_engagement_policy
            or manifest.get("detector_mode") != detector_mode
        ):
            raise RuntimeError("SIL orchestration manifest is invalid")
        manifest_software = _evidence_mapping(
            manifest.get("software_binding"), "SIL orchestration software binding"
        )
        if manifest_software != software:
            raise RuntimeError("SIL audit and orchestration software bindings differ")
        artifacts = _evidence_mapping(manifest.get("artifacts"), "SIL artifacts")
        required_artifacts = set(SITL_REQUIRED_ARTIFACTS)
        if detector_mode == "rendered":
            required_artifacts.add("yolo_model")
        missing_artifacts = sorted(required_artifacts - set(artifacts))
        if missing_artifacts:
            raise RuntimeError(
                "SIL orchestration artifacts are incomplete: "
                + ", ".join(missing_artifacts)
            )
        flight_config = _evidence_mapping(
            artifacts.get("flight_config"), "SIL flight config artifact"
        )
        if flight_config.get("sha256") != runtime_config_sha256:
            raise RuntimeError("SIL orchestration flight config SHA256 mismatch")
        binary = _evidence_mapping(
            artifacts.get("betaflight_binary"), "SIL Betaflight binary artifact"
        )
        if binary.get("sha256") != official_betaflight_elf_sha256:
            raise RuntimeError("SIL orchestration Betaflight binary SHA256 mismatch")
        for name, raw_binding in artifacts.items():
            binding = _evidence_mapping(raw_binding, f"SIL artifact {name}")
            artifact_path = Path(str(binding.get("path", ""))).expanduser().resolve()
            if (
                not artifact_path.is_file()
                or _evidence_sha256(artifact_path) != binding.get("sha256")
            ):
                raise RuntimeError(f"SIL artifact changed or is missing: {name}")

        validated[detector_mode] = {
            "path": str(report_path),
            "sha256": _evidence_sha256(report_path),
            "schema_version": 1,
            "scope": SITL_SCOPE,
            "policy": expected_engagement_policy,
            "detector_mode": detector_mode,
            "detector_representative": False,
            "hardware_authorization": False,
            "runtime_config_sha256": runtime_config_sha256,
            "repository_commit": repository_commit,
            "orchestration_manifest": {
                "path": str(orchestration_path),
                "sha256": str(orchestration["sha256"]),
            },
        }

    if set(validated) != set(SITL_REQUIRED_DETECTOR_MODES):
        raise RuntimeError("SIL evidence requires both projected and rendered detector modes")
    return [validated[mode] for mode in sorted(validated)]


def revalidate_bound_sitl_evidence(
    bindings: object,
    *,
    runtime_config_sha256: str,
    expected_engagement_policy: str,
    repository_commit: str,
) -> list[dict[str, Any]]:
    """Reopen approval-bound SIL files and reject any post-approval change."""

    if not isinstance(bindings, list):
        raise RuntimeError("SIL evidence bindings must be a list")
    reports: list[tuple[dict[str, Any], Path]] = []
    for raw_binding in bindings:
        binding = _evidence_mapping(raw_binding, "SIL approval binding")
        report_path = Path(str(binding.get("path", ""))).expanduser().resolve()
        if not report_path.is_file():
            raise RuntimeError(f"SIL audit is missing: {report_path}")
        if _evidence_sha256(report_path) != binding.get("sha256"):
            raise RuntimeError("SIL approval audit SHA256 mismatch")
        reports.append((_read_evidence_json(report_path), report_path))
    validated = validate_sitl_audit_evidence(
        reports,
        runtime_config_sha256=runtime_config_sha256,
        expected_engagement_policy=expected_engagement_policy,
        repository_commit=repository_commit,
    )
    if validated != bindings:
        raise RuntimeError("SIL approval bindings do not match the validated audits")
    return validated


@dataclass(frozen=True)
class BetaflightFdmPacket:
    timestamp_s: float
    angular_velocity_body_rad_s: tuple[float, float, float]
    linear_acceleration_body_m_s2: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]
    velocity_enu_m_s: tuple[float, float, float]
    longitude_latitude_altitude: tuple[float, float, float]
    pressure_pa: float


@dataclass(frozen=True)
class SitlMotorOutput:
    normalized: tuple[float, float, float, float]
    raw_us: tuple[float, ...] | None
    received_monotonic_s: float


@dataclass(frozen=True)
class GazeboPoseSample:
    received_monotonic_s: float
    simulation_time_s: float | None
    position_enu_m: np.ndarray
    velocity_enu_m_s: np.ndarray
    orientation_wxyz: np.ndarray


@dataclass(frozen=True)
class SitlPilotRcConfig:
    """Deterministic physical-pilot input for isolated SITL runs."""

    rate_hz: float = 100.0
    arm_after_s: float = 3.0
    throttle_after_arm_s: float = 4.0
    takeover_after_s: float = 8.0
    takeover_duration_s: float = 0.7
    disarm_after_s: float = 11.0
    throttle_us: int = 1275
    motion_test_after_s: float | None = None
    motion_test_half_duration_s: float = 0.4
    motion_test_delta_us: int = 100
    motion_test_axis: str = "both"

    @classmethod
    def from_mapping(cls, values: dict[str, Any]) -> "SitlPilotRcConfig":
        config = cls(
            rate_hz=float(values.get("rate_hz", 100.0)),
            arm_after_s=float(values.get("arm_after_s", 3.0)),
            throttle_after_arm_s=float(values.get("throttle_after_arm_s", 4.0)),
            takeover_after_s=float(values.get("takeover_after_s", 8.0)),
            takeover_duration_s=float(values.get("takeover_duration_s", 0.7)),
            disarm_after_s=float(values.get("disarm_after_s", 11.0)),
            throttle_us=int(values.get("throttle_us", 1275)),
            motion_test_after_s=(
                None
                if values.get("motion_test_after_s") is None
                else float(values["motion_test_after_s"])
            ),
            motion_test_half_duration_s=float(
                values.get("motion_test_half_duration_s", 0.4)
            ),
            motion_test_delta_us=int(values.get("motion_test_delta_us", 100)),
            motion_test_axis=str(values.get("motion_test_axis", "both")),
        )
        if not all(
            math.isfinite(value)
            for value in (
                config.rate_hz,
                config.arm_after_s,
                config.throttle_after_arm_s,
                config.takeover_after_s,
                config.takeover_duration_s,
                config.disarm_after_s,
                config.motion_test_half_duration_s,
                *(
                    ()
                    if config.motion_test_after_s is None
                    else (config.motion_test_after_s,)
                ),
            )
        ):
            raise ValueError("SITL pilot RC timing must be finite")
        if config.rate_hz < 50.0:
            raise ValueError("SITL pilot RC rate must be at least 50 Hz")
        if not 1.0 <= config.arm_after_s < config.throttle_after_arm_s:
            raise ValueError("SITL pilot must hold low throttle after requesting ARM")
        if config.throttle_after_arm_s >= config.takeover_after_s:
            raise ValueError("SITL pilot must prefill before arming and takeover")
        if not 0.5 <= config.takeover_duration_s <= 0.9:
            raise ValueError("SITL takeover duration must be within 0.5-0.9 s")
        if config.disarm_after_s <= config.takeover_after_s + config.takeover_duration_s:
            raise ValueError("SITL pilot must release RC7 before DISARM")
        if not 1200 <= config.throttle_us <= 1400:
            raise ValueError("SITL pilot throttle must remain within 1200-1400 us")
        if config.motion_test_after_s is not None:
            if not (
                config.throttle_after_arm_s
                <= config.motion_test_after_s
                < config.takeover_after_s - 2.0 * config.motion_test_half_duration_s
            ):
                raise ValueError("SITL motion test must finish before takeover")
            if not 0.2 <= config.motion_test_half_duration_s <= 0.6:
                raise ValueError("SITL motion-test half duration must be within 0.2-0.6 s")
            if not 50 <= config.motion_test_delta_us <= 150:
                raise ValueError("SITL motion-test stick delta must be within 50-150 us")
            if config.motion_test_axis not in {"roll", "pitch", "both"}:
                raise ValueError("SITL motion-test axis must be roll, pitch, or both")
        return config


class SitlPilotRcScheduler:
    """Continuously publish AETR1234 physical RC without sharing flight endpoints."""

    def __init__(
        self,
        config: SitlPilotRcConfig,
        *,
        host: str = "127.0.0.1",
        rc_port: int = DEFAULT_RC_PORT,
    ) -> None:
        if host not in {"127.0.0.1", "localhost"}:
            raise ValueError("SITL pilot RC endpoint must be IPv4 loopback")
        self.config = config
        self.endpoint = (host, int(rc_port))
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._started_monotonic_s: float | None = None
        self._send_count = 0
        self._last_channels: tuple[int, ...] | None = None

    def channels_at(self, elapsed_s: float) -> tuple[int, ...]:
        elapsed = max(0.0, float(elapsed_s))
        armed = self.config.arm_after_s <= elapsed < self.config.disarm_after_s
        takeover = (
            armed
            and self.config.takeover_after_s
            <= elapsed
            < self.config.takeover_after_s + self.config.takeover_duration_s
        )
        throttle = (
            self.config.throttle_us
            if armed and elapsed >= self.config.throttle_after_arm_s
            else 1000
        )
        roll = 1500
        pitch = 1500
        motion_start = self.config.motion_test_after_s
        if motion_start is not None:
            motion_elapsed = elapsed - motion_start
            half = self.config.motion_test_half_duration_s
            if 0.0 <= motion_elapsed < half:
                if self.config.motion_test_axis in {"roll", "both"}:
                    roll += self.config.motion_test_delta_us
                if self.config.motion_test_axis in {"pitch", "both"}:
                    pitch += self.config.motion_test_delta_us
            elif half <= motion_elapsed < 2.0 * half:
                if self.config.motion_test_axis in {"roll", "both"}:
                    roll -= self.config.motion_test_delta_us
                if self.config.motion_test_axis in {"pitch", "both"}:
                    pitch -= self.config.motion_test_delta_us
        # AETR1234: AUX1 low arms, AUX3 high activates MSP OVERRIDE, AUX4 low is Acro.
        return (
            roll,
            pitch,
            throttle,
            1500,
            1000 if armed else 2000,
            1000,
            2000 if takeover else 1000,
            1000,
        )

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("SITL pilot RC scheduler is already started")
        self._started_monotonic_s = time.monotonic()
        self._thread = threading.Thread(
            target=self._run,
            name="betaflight-sitl-pilot-rc",
            daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._socket.close()

    def __enter__(self) -> "SitlPilotRcScheduler":
        self.start()
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()

    def stats(self) -> dict[str, Any]:
        return {
            "send_count": self._send_count,
            "last_channels": self._last_channels,
            "started_monotonic_s": self._started_monotonic_s,
        }

    def _run(self) -> None:
        assert self._started_monotonic_s is not None
        period_s = 1.0 / self.config.rate_hz
        deadline = self._started_monotonic_s
        while not self._stop.is_set():
            now = time.monotonic()
            channels = self.channels_at(now - self._started_monotonic_s)
            self._socket.sendto(pack_rc_packet(now, channels), self.endpoint)
            self._send_count += 1
            self._last_channels = channels
            deadline += period_s
            wait_s = deadline - time.monotonic()
            if wait_s > 0.0:
                self._stop.wait(wait_s)
            else:
                deadline = time.monotonic()


def pack_fdm_packet(packet: BetaflightFdmPacket) -> bytes:
    values = (
        packet.timestamp_s,
        *packet.angular_velocity_body_rad_s,
        *packet.linear_acceleration_body_m_s2,
        *packet.orientation_wxyz,
        *packet.velocity_enu_m_s,
        *packet.longitude_latitude_altitude,
        packet.pressure_pa,
    )
    if len(values) != 18 or not np.all(np.isfinite(values)):
        raise ValueError("Betaflight FDM packet requires 18 finite values")
    return FDM_PACKET_STRUCT.pack(*(float(value) for value in values))


def pack_rc_packet(timestamp_s: float, channels: Sequence[int]) -> bytes:
    values = [int(value) for value in channels]
    if len(values) > 16:
        raise ValueError("Betaflight SITL accepts at most 16 RC channels")
    if any(value < 750 or value > 2250 for value in values):
        raise ValueError("SITL RC channel is outside the sane 750-2250 us range")
    values.extend([1000] * (16 - len(values)))
    return RC_PACKET_STRUCT.pack(float(timestamp_s), *values)


def unpack_servo_packet(data: bytes) -> tuple[float, float, float, float]:
    if len(data) != SERVO_PACKET_STRUCT.size:
        raise ValueError(
            f"normalized motor packet must be {SERVO_PACKET_STRUCT.size} bytes"
        )
    return tuple(float(value) for value in SERVO_PACKET_STRUCT.unpack(data))


def unpack_servo_raw_packet(data: bytes) -> tuple[float, ...]:
    if len(data) != SERVO_RAW_PACKET_STRUCT.size:
        raise ValueError(
            f"raw motor packet must be {SERVO_RAW_PACKET_STRUCT.size} bytes"
        )
    count, *values = SERVO_RAW_PACKET_STRUCT.unpack(data)
    if count > len(values):
        raise ValueError("raw motor packet declares too many outputs")
    return tuple(float(value) for value in values[:count])


class BetaflightSitlUdp:
    """Physical-pilot RC input and motor observation for Betaflight SITL."""

    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        bind_host: str = "127.0.0.1",
        rc_port: int = DEFAULT_RC_PORT,
        pwm_port: int = DEFAULT_PWM_PORT,
        pwm_raw_port: int = DEFAULT_PWM_RAW_PORT,
    ) -> None:
        if host not in {"127.0.0.1", "localhost", "::1"}:
            raise ValueError("SITL UDP endpoints must be loopback-only")
        if bind_host not in {"127.0.0.1", "localhost", "::1"}:
            raise ValueError("SITL UDP listeners must be loopback-only")
        self.rc_endpoint = (host, int(rc_port))
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._pwm = self._bind(bind_host, int(pwm_port))
        self._pwm_raw = self._bind(bind_host, int(pwm_raw_port))
        self._normalized: tuple[float, float, float, float] | None = None
        self._raw: tuple[float, ...] | None = None
        self._received_s: float | None = None

    @staticmethod
    def _bind(host: str, port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
        sock.setblocking(False)
        return sock

    def close(self) -> None:
        self._tx.close()
        self._pwm.close()
        self._pwm_raw.close()

    def __enter__(self) -> "BetaflightSitlUdp":
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()

    def send_physical_rc(self, channels: Sequence[int], *, timestamp_s: float | None = None) -> None:
        stamp = time.monotonic() if timestamp_s is None else float(timestamp_s)
        self._tx.sendto(pack_rc_packet(stamp, channels), self.rc_endpoint)

    def poll_motor_output(self, timeout_s: float = 0.0) -> SitlMotorOutput | None:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while True:
            wait = max(0.0, deadline - time.monotonic()) if timeout_s > 0.0 else 0.0
            readable, _, _ = select.select((self._pwm, self._pwm_raw), (), (), wait)
            for sock in readable:
                while True:
                    try:
                        data, _ = sock.recvfrom(4096)
                    except BlockingIOError:
                        break
                    try:
                        if sock is self._pwm:
                            self._normalized = unpack_servo_packet(data)
                        else:
                            self._raw = unpack_servo_raw_packet(data)
                    except ValueError:
                        continue
                    self._received_s = time.monotonic()
            if self._normalized is not None and self._received_s is not None:
                return SitlMotorOutput(self._normalized, self._raw, self._received_s)
            if timeout_s <= 0.0 or time.monotonic() >= deadline:
                return None


class GazeboPoseStream:
    def __init__(self, *, world: str, model_names: Sequence[str] | None = None) -> None:
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node

        self.topics = (
            (f"/world/{world}/pose/info",)
            if not model_names
            else tuple(f"/model/{name}/pose" for name in model_names)
        )
        self.topic = ",".join(self.topics)
        self._lock = threading.Lock()
        self._samples: dict[str, GazeboPoseSample] = {}
        self._previous: dict[str, tuple[np.ndarray, float | None, float]] = {}
        self._node = Node()
        subscribed = []
        for topic in self.topics:
            if not self._node.subscribe(Pose_V, topic, self._callback):
                for active_topic in subscribed:
                    self._node.unsubscribe(active_topic)
                raise RuntimeError(f"failed to subscribe to {topic}")
            subscribed.append(topic)

    def close(self) -> None:
        for topic in self.topics:
            self._node.unsubscribe(topic)

    def _callback(self, message: Any) -> None:
        sim_time_s = None
        if message.header.HasField("stamp"):
            sim_time_s = float(message.header.stamp.sec) + 1.0e-9 * float(
                message.header.stamp.nsec
            )
        received_s = time.monotonic()
        updates: dict[str, GazeboPoseSample] = {}
        for pose in message.pose:
            if not pose.name:
                continue
            position = np.array(
                [pose.position.x, pose.position.y, pose.position.z], dtype=float
            )
            velocity = np.zeros(3, dtype=float)
            previous = self._previous.get(pose.name)
            if previous is not None:
                dt = (
                    sim_time_s - float(previous[1])
                    if sim_time_s is not None and previous[1] is not None
                    else received_s - previous[2]
                )
                if dt > 1.0e-6:
                    velocity = (position - previous[0]) / dt
            self._previous[pose.name] = (position, sim_time_s, received_s)
            updates[pose.name] = GazeboPoseSample(
                received_monotonic_s=received_s,
                simulation_time_s=sim_time_s,
                position_enu_m=position,
                velocity_enu_m_s=velocity,
                orientation_wxyz=np.array(
                    [
                        pose.orientation.w,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                    ],
                    dtype=float,
                ),
            )
        with self._lock:
            self._samples.update(updates)

    def latest(self, model: str) -> GazeboPoseSample | None:
        with self._lock:
            return self._samples.get(model)

    def wait_latest(self, model: str, timeout_s: float = 5.0) -> GazeboPoseSample:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            sample = self.latest(model)
            if sample is not None:
                return sample
            time.sleep(0.01)
        raise TimeoutError(f"no Gazebo pose for {model!r} on {self.topic}")


def quaternion_rotation_matrix_wxyz(quaternion: Sequence[float]) -> np.ndarray:
    values = np.asarray(quaternion, dtype=float)
    if values.shape != (4,) or not np.all(np.isfinite(values)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(values))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is zero")
    w, x, y, z = values / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def gazebo_pose_to_body_frd_euler_deg(
    orientation_wxyz: Sequence[float],
) -> tuple[float, float, float]:
    """Convert a Gazebo FLU-to-ENU pose into body-FRD-to-NED Euler angles."""

    ned_from_enu = np.array(
        [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
        dtype=float,
    )
    flu_from_frd = np.diag([1.0, -1.0, -1.0])
    body_frd_to_ned = (
        ned_from_enu
        @ quaternion_rotation_matrix_wxyz(orientation_wxyz)
        @ flu_from_frd
    )
    pitch = math.asin(np.clip(-float(body_frd_to_ned[2, 0]), -1.0, 1.0))
    roll = math.atan2(
        float(body_frd_to_ned[2, 1]), float(body_frd_to_ned[2, 2])
    )
    yaw = math.atan2(
        float(body_frd_to_ned[1, 0]), float(body_frd_to_ned[0, 0])
    )
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def project_target_box(
    interceptor: GazeboPoseSample,
    target: GazeboPoseSample,
    *,
    intrinsics: CameraIntrinsics,
    R_BC: np.ndarray,
    target_size_m: Sequence[float],
) -> tuple[tuple[float, float, float, float] | None, float]:
    size = np.asarray(target_size_m, dtype=float)
    if size.shape != (3,) or np.any(size <= 0.0):
        raise ValueError("target_size_m must contain three positive values")
    R_WL = quaternion_rotation_matrix_wxyz(interceptor.orientation_wxyz)
    R_WT = quaternion_rotation_matrix_wxyz(target.orientation_wxyz)
    R_LB = np.diag([1.0, -1.0, -1.0])
    R_CB = np.asarray(R_BC, dtype=float).T
    pixels: list[tuple[float, float]] = []
    depths: list[float] = []
    for sx in (-0.5, 0.5):
        for sy in (-0.5, 0.5):
            for sz in (-0.5, 0.5):
                corner_world = target.position_enu_m + R_WT @ (size * [sx, sy, sz])
                relative_flu = R_WL.T @ (corner_world - interceptor.position_enu_m)
                relative_camera = R_CB @ (R_LB @ relative_flu)
                depth = float(relative_camera[2])
                if depth <= 1.0e-4:
                    continue
                pixels.append(
                    (
                        intrinsics.fx * float(relative_camera[0]) / depth + intrinsics.cx,
                        intrinsics.fy * float(relative_camera[1]) / depth + intrinsics.cy,
                    )
                )
                depths.append(depth)
    if not pixels:
        return None, float("nan")
    xs = [value[0] for value in pixels]
    ys = [value[1] for value in pixels]
    raw = (min(xs), min(ys), max(xs), max(ys))
    clipped = (
        max(0.0, raw[0]),
        max(0.0, raw[1]),
        min(float(intrinsics.width), raw[2]),
        min(float(intrinsics.height), raw[3]),
    )
    if clipped[2] <= clipped[0] or clipped[3] <= clipped[1]:
        return None, float(np.mean(depths))
    return clipped, float(np.mean(depths))


class GazeboProjectedDetectionSource:
    def __init__(
        self,
        *,
        world: str,
        interceptor_model: str,
        target_model: str,
        intrinsics: CameraIntrinsics,
        R_BC: np.ndarray,
        target_size_m: Sequence[float] = (0.55, 0.55, 0.20),
        detection_latency_s: float = 0.04,
    ) -> None:
        self.stream = GazeboPoseStream(
            world=world,
            model_names=(interceptor_model, target_model),
        )
        self.interceptor_model = str(interceptor_model)
        self.target_model = str(target_model)
        self.intrinsics = intrinsics
        self.R_BC = np.asarray(R_BC, dtype=float)
        self.target_size_m = tuple(float(value) for value in target_size_m)
        self.detection_latency_s = float(detection_latency_s)
        if not 0.0 <= self.detection_latency_s <= 0.1:
            raise ValueError("projected SITL detection latency must be within 0-0.1 s")
        self._last_pose_stamp: float | None = None

    def close(self) -> None:
        self.stream.close()

    def detect(
        self,
        *,
        timestamp: float,
        frame_id: int,
        active_track_id: int | None,
    ) -> tuple[FrameDetection | None, dict[str, Any]]:
        del active_track_id
        interceptor = self.stream.latest(self.interceptor_model)
        target = self.stream.latest(self.target_model)
        base = {
            "detector_source": "sitl_projected",
            "camera_device": self.stream.topic,
            "camera_output_width": self.intrinsics.width,
            "camera_output_height": self.intrinsics.height,
        }
        if interceptor is None or target is None:
            return None, {
                **base,
                "detector_reject_reason": "gazebo_pose_missing",
                "perception_new_result": 0,
            }
        base.update(sitl_truth_stats(interceptor, target, timestamp=float(timestamp)))
        pose_stamp = target.simulation_time_s
        if pose_stamp is not None and pose_stamp == self._last_pose_stamp:
            return None, {
                **base,
                "detector_reject_reason": "perception_no_new_result",
                "perception_new_result": 0,
            }
        self._last_pose_stamp = pose_stamp
        bbox, depth = project_target_box(
            interceptor,
            target,
            intrinsics=self.intrinsics,
            R_BC=self.R_BC,
            target_size_m=self.target_size_m,
        )
        if bbox is None:
            return None, {
                **base,
                "detector_reject_reason": "target_out_of_view",
                "perception_new_result": 1,
                "sitl_target_depth_m": depth,
            }
        exposure_s = min(
            float(timestamp),
            target.received_monotonic_s,
        ) - self.detection_latency_s
        return FrameDetection(frame_id, exposure_s, bbox, 1, 1.0), {
            **base,
            "detector_reject_reason": "",
            "perception_new_result": 1,
            "camera_capture_ts": exposure_s,
            "sitl_target_depth_m": depth,
            "sitl_truth_bbox_xyxy": list(bbox),
            "sitl_projected_bbox_center_x": 0.5 * (bbox[0] + bbox[2]),
            "sitl_projected_bbox_center_y": 0.5 * (bbox[1] + bbox[3]),
        }


def sitl_truth_stats(
    interceptor: GazeboPoseSample,
    target: GazeboPoseSample,
    *,
    timestamp: float,
) -> dict[str, float]:
    values: dict[str, float] = {}
    for prefix, sample in (("interceptor", interceptor), ("target", target)):
        position = sample.position_enu_m
        velocity = sample.velocity_enu_m_s
        values.update(
            {
                f"sitl_{prefix}_simulation_time_s": (
                    float("nan")
                    if sample.simulation_time_s is None
                    else float(sample.simulation_time_s)
                ),
                f"sitl_{prefix}_truth_age_s": max(
                    0.0, float(timestamp) - sample.received_monotonic_s
                ),
                f"sitl_{prefix}_position_enu_x_m": float(position[0]),
                f"sitl_{prefix}_position_enu_y_m": float(position[1]),
                f"sitl_{prefix}_position_enu_z_m": float(position[2]),
                f"sitl_{prefix}_velocity_enu_x_m_s": float(velocity[0]),
                f"sitl_{prefix}_velocity_enu_y_m_s": float(velocity[1]),
                f"sitl_{prefix}_velocity_enu_z_m_s": float(velocity[2]),
            }
        )
    velocity = interceptor.velocity_enu_m_s
    roll_deg, pitch_deg, yaw_deg = gazebo_pose_to_body_frd_euler_deg(
        interceptor.orientation_wxyz
    )
    values.update(
        {
            "sitl_expected_velocity_ned_n_m_s": float(velocity[1]),
            "sitl_expected_velocity_ned_e_m_s": float(velocity[0]),
            "sitl_expected_velocity_ned_d_m_s": -float(velocity[2]),
            "sitl_interceptor_roll_frd_deg": roll_deg,
            "sitl_interceptor_pitch_frd_deg": pitch_deg,
            "sitl_interceptor_yaw_ned_deg": yaw_deg % 360.0,
            "sitl_expected_msp_roll_deg": roll_deg,
            "sitl_expected_msp_pitch_deg": -pitch_deg,
            "sitl_expected_msp_yaw_deg": yaw_deg % 360.0,
        }
    )
    return values


class GazeboCameraSource:
    """OpenCV-compatible image source backed by a Gazebo image topic."""

    def __init__(self, *, topic: str, cv2_module: Any = None) -> None:
        from gz.msgs10.image_pb2 import Image
        from gz.transport13 import Node

        if not str(topic).startswith("/world/"):
            raise ValueError("Gazebo camera topic must be an absolute world topic")
        if cv2_module is None:
            import cv2 as cv2_module
        self.cv2 = cv2_module
        self.topic = str(topic)
        self._lock = threading.Lock()
        self._latest: tuple[int, float, np.ndarray] | None = None
        self._sequence = 0
        self._consumed = 0
        self.last_image: np.ndarray | None = None
        self.last_stats: dict[str, Any] = {}
        self._node = Node()
        if not self._node.subscribe(Image, self.topic, self._callback):
            raise RuntimeError(f"failed to subscribe to {self.topic}")

    def _callback(self, message: Any) -> None:
        channels_by_format = {1: 1, 3: 3, 4: 4, 5: 4, 8: 3}
        channels = channels_by_format.get(int(message.pixel_format_type))
        if channels is None or int(message.width) <= 0 or int(message.height) <= 0:
            return
        raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
        expected = int(message.width) * int(message.height) * channels
        if raw.size < expected:
            return
        image = raw[:expected].reshape(int(message.height), int(message.width), channels)
        if int(message.pixel_format_type) == 3:
            image = image[:, :, ::-1]
        elif int(message.pixel_format_type) == 4:
            image = self.cv2.cvtColor(image, self.cv2.COLOR_RGBA2BGR)
        elif int(message.pixel_format_type) == 5:
            image = self.cv2.cvtColor(image, self.cv2.COLOR_BGRA2BGR)
        elif channels == 1:
            image = self.cv2.cvtColor(image, self.cv2.COLOR_GRAY2BGR)
        capture_s = time.monotonic()
        with self._lock:
            self._sequence += 1
            self._latest = (self._sequence, capture_s, np.ascontiguousarray(image))

    def read_image(self) -> np.ndarray | None:
        with self._lock:
            latest = self._latest
        if latest is None or latest[0] == self._consumed:
            self.last_stats = {
                "camera_device": self.topic,
                "camera_frame_ok": 0,
                "perception_new_result": 0,
            }
            return None
        self._consumed = latest[0]
        self.last_image = latest[2].copy()
        height, width = self.last_image.shape[:2]
        self.last_stats = {
            "camera_device": self.topic,
            "camera_frame_ok": 1,
            "camera_capture_ts": latest[1],
            "camera_input_width": width,
            "camera_input_height": height,
            "camera_output_width": width,
            "camera_output_height": height,
            "perception_new_result": 1,
        }
        return self.last_image

    def close(self) -> None:
        self._node.unsubscribe(self.topic)


def validate_loopback_sitl_config(config: dict[str, Any]) -> dict[str, Any]:
    profile = dict(config.get("sitl_profile", {}))
    serial = dict(config.get("serial", {}))
    if profile.get("scope") != SITL_SCOPE:
        raise RuntimeError(f"SITL profile scope must be {SITL_SCOPE}")
    if profile.get("loopback_only") is not True:
        raise RuntimeError("SITL profile must declare loopback_only=true")
    port = str(serial.get("port", ""))
    if port != SITL_MSP_URL:
        raise RuntimeError(f"SITL MSP transport must be {SITL_MSP_URL}")
    if dict(config.get("flight_profile", {})).get("scope"):
        raise RuntimeError("SITL configuration cannot declare a real-flight scope")
    if profile.get("simulated_telemetry_provenance") != "gazebo_truth":
        raise RuntimeError("SITL telemetry must be explicitly marked gazebo_truth")
    try:
        simulated_vbat_v = float(profile["simulated_vbat_v"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("SITL profile requires a finite simulated_vbat_v") from exc
    if not math.isfinite(simulated_vbat_v) or not 22.0 <= simulated_vbat_v <= 25.2:
        raise RuntimeError("SITL simulated voltage must remain within 22.0-25.2 V")
    if profile.get("simulated_voltage_provenance") != "sitl_config_only":
        raise RuntimeError("SITL voltage must declare sitl_config_only provenance")
    try:
        detection_latency_s = float(profile["projected_detection_latency_s"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("SITL profile requires projected_detection_latency_s") from exc
    if not math.isfinite(detection_latency_s) or not 0.0 <= detection_latency_s <= 0.1:
        raise RuntimeError("SITL projected detection latency must be within 0-0.1 s")
    SitlPilotRcConfig.from_mapping(dict(profile.get("pilot_rc", {})))
    return profile


__all__ = [
    "BetaflightFdmPacket",
    "BetaflightSitlUdp",
    "FDM_PACKET_STRUCT",
    "GazeboCameraSource",
    "GazeboPoseSample",
    "GazeboPoseStream",
    "GazeboProjectedDetectionSource",
    "RC_PACKET_STRUCT",
    "SERVO_PACKET_STRUCT",
    "SERVO_RAW_PACKET_STRUCT",
    "SITL_AUDIT_EVIDENCE_TYPE",
    "SitlMotorOutput",
    "SITL_OFFICIAL_BETAFLIGHT_COMMIT",
    "SITL_OFFICIAL_BETAFLIGHT_ELF_SHA256",
    "SitlPilotRcConfig",
    "SitlPilotRcScheduler",
    "SITL_MSP_URL",
    "SITL_REQUIRED_ARTIFACTS",
    "SITL_REQUIRED_DETECTOR_MODES",
    "SITL_RUN_EVIDENCE_TYPE",
    "SITL_SCOPE",
    "pack_fdm_packet",
    "pack_rc_packet",
    "project_target_box",
    "gazebo_pose_to_body_frd_euler_deg",
    "sitl_truth_stats",
    "quaternion_rotation_matrix_wxyz",
    "revalidate_bound_sitl_evidence",
    "unpack_servo_packet",
    "unpack_servo_raw_packet",
    "validate_sitl_audit_evidence",
    "validate_loopback_sitl_config",
]
