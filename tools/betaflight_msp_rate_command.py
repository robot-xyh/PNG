#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import sys
import time
from typing import Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from vision_guidance.betaflight_sitl import (  # noqa: E402
    BodyRateRCConfig,
    DEFAULT_BETAFLIGHT_HOST,
    DEFAULT_MSP_PORT,
    RateCommand,
    body_rate_rc_from_rate_command,
    encode_msp_set_raw_rc,
    rate_command_from_body_rate_output,
)


class RawRCWriter:
    def write(self, frame: bytes) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


class TcpRawRCWriter(RawRCWriter):
    def __init__(self, host: str, port: int, timeout_s: float) -> None:
        self.sock = socket.create_connection((host, int(port)), timeout=max(0.001, float(timeout_s)))
        self.sock.settimeout(max(0.001, float(timeout_s)))

    def write(self, frame: bytes) -> None:
        self.sock.sendall(frame)

    def close(self) -> None:
        self.sock.close()


class SerialRawRCWriter(RawRCWriter):
    def __init__(self, device: str, baud: int, timeout_s: float) -> None:
        try:
            import serial
        except ImportError as exc:
            raise SystemExit("Serial transport requires pyserial: python3 -m pip install pyserial") from exc
        self.serial = serial.Serial(device, int(baud), timeout=max(0.001, float(timeout_s)), write_timeout=max(0.001, float(timeout_s)))

    def write(self, frame: bytes) -> None:
        written = self.serial.write(frame)
        if written != len(frame):
            raise OSError(f"short serial write: {written}/{len(frame)} bytes")
        self.serial.flush()

    def close(self) -> None:
        self.serial.close()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Map a PNG RateCommand to Betaflight RC and inject it with MSP_SET_RAW_RC.",
    )
    parser.add_argument("--transport", choices=("tcp", "serial"), default="tcp")
    parser.add_argument("--host", default=DEFAULT_BETAFLIGHT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_MSP_PORT)
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=1.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--duration-s", type=float, default=1.0, help="Use 0 to send one frame.")
    parser.add_argument(
        "--stdin-jsonl",
        action="store_true",
        help="Read RateCommand JSON lines from stdin instead of using constant CLI rates.",
    )
    parser.add_argument("--roll-rate-rad-s", type=float, default=0.0)
    parser.add_argument("--pitch-rate-rad-s", type=float, default=0.0)
    parser.add_argument("--yaw-rate-rad-s", type=float, default=0.0)
    parser.add_argument("--thrust-z", type=float, default=0.0, help="Normalized throttle command in [0, 1].")
    parser.add_argument("--max-roll-rate-rad-s", type=float, default=None)
    parser.add_argument("--max-pitch-rate-rad-s", type=float, default=None)
    parser.add_argument("--max-yaw-rate-rad-s", type=float, default=None)
    parser.add_argument("--roll-sign", type=float, default=1.0)
    parser.add_argument("--pitch-sign", type=float, default=1.0)
    parser.add_argument("--yaw-sign", type=float, default=1.0)
    parser.add_argument("--min-throttle", type=int, default=1000)
    parser.add_argument("--max-throttle", type=int, default=2000)
    parser.add_argument("--arm", action="store_true", help="Set AUX1 high. Default keeps AUX1 low.")
    parser.add_argument("--angle-aux", type=int, default=1000, help="AUX2 value. Keep low for Betaflight Acro/Rate mode.")
    parser.add_argument("--dry-run", action="store_true", help="Print RC channels and MSP frame without sending.")
    parser.add_argument("--print-every-s", type=float, default=0.5)
    parser.add_argument("--no-stop-neutral", action="store_true", help="Do not send neutral disarmed RC before exiting.")
    return parser


def _config_from_args(args: argparse.Namespace) -> BodyRateRCConfig:
    defaults = BodyRateRCConfig()
    return BodyRateRCConfig(
        max_roll_rate_rad_s=float(args.max_roll_rate_rad_s or defaults.max_roll_rate_rad_s),
        max_pitch_rate_rad_s=float(args.max_pitch_rate_rad_s or defaults.max_pitch_rate_rad_s),
        max_yaw_rate_rad_s=float(args.max_yaw_rate_rad_s or defaults.max_yaw_rate_rad_s),
        roll_sign=float(args.roll_sign),
        pitch_sign=float(args.pitch_sign),
        yaw_sign=float(args.yaw_sign),
        min_throttle=int(args.min_throttle),
        max_throttle=int(args.max_throttle),
        arm_aux=2000 if bool(args.arm) else 1000,
        angle_aux=int(args.angle_aux),
    )


def _writer_from_args(args: argparse.Namespace) -> RawRCWriter:
    if args.transport == "serial":
        return SerialRawRCWriter(args.device, args.baud, args.timeout_s)
    return TcpRawRCWriter(args.host, args.port, args.timeout_s)


def _frame_for(command: RateCommand, config: BodyRateRCConfig) -> tuple[tuple[int, ...], bytes]:
    result = body_rate_rc_from_rate_command(command, config)
    channels = result.command.channels(config.total_channels)
    return channels, encode_msp_set_raw_rc(channels)


def _command_from_record(record: object) -> RateCommand:
    if not isinstance(record, dict):
        raise ValueError("JSON line must be an object")
    if "body_rates_rad_s" in record:
        return rate_command_from_body_rate_output(record)
    return RateCommand(
        roll_rate_rad_s=float(record.get("roll_rate_rad_s", record.get("roll", 0.0))),
        pitch_rate_rad_s=float(record.get("pitch_rate_rad_s", record.get("pitch", 0.0))),
        yaw_rate_rad_s=float(record.get("yaw_rate_rad_s", record.get("yaw", 0.0))),
        thrust_z=float(record.get("thrust_z", record.get("thrust", 0.0))),
    )


def _print_frame(prefix: str, command: RateCommand, channels: Sequence[int], frame: bytes) -> None:
    print(
        f"{prefix} rate=({command.roll_rate_rad_s:.4f}, {command.pitch_rate_rad_s:.4f}, "
        f"{command.yaw_rate_rad_s:.4f}) thrust_z={command.thrust_z:.3f} "
        f"rc8={tuple(channels[:8])} msp_hex={frame.hex()}",
        flush=True,
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    config = _config_from_args(args)
    command = RateCommand(
        roll_rate_rad_s=float(args.roll_rate_rad_s),
        pitch_rate_rad_s=float(args.pitch_rate_rad_s),
        yaw_rate_rad_s=float(args.yaw_rate_rad_s),
        thrust_z=float(args.thrust_z),
    )
    channels, frame = _frame_for(command, config)
    if not args.stdin_jsonl:
        _print_frame("prepared", command, channels, frame)
        if args.dry_run:
            return 0

    writer = None if args.dry_run else _writer_from_args(args)
    stop_command = RateCommand(0.0, 0.0, 0.0, 0.0)
    stop_config = BodyRateRCConfig(
        max_roll_rate_rad_s=config.max_roll_rate_rad_s,
        max_pitch_rate_rad_s=config.max_pitch_rate_rad_s,
        max_yaw_rate_rad_s=config.max_yaw_rate_rad_s,
        roll_sign=config.roll_sign,
        pitch_sign=config.pitch_sign,
        yaw_sign=config.yaw_sign,
        min_throttle=config.min_throttle,
        max_throttle=config.max_throttle,
        arm_aux=1000,
        angle_aux=config.angle_aux,
    )
    try:
        if args.stdin_jsonl:
            sent = 0
            for lineno, line in enumerate(sys.stdin, start=1):
                line = line.strip()
                if not line:
                    continue
                try:
                    stdin_command = _command_from_record(json.loads(line))
                    stdin_channels, stdin_frame = _frame_for(stdin_command, config)
                except (TypeError, ValueError, json.JSONDecodeError) as exc:
                    print(f"warning: skipped stdin line {lineno}: {exc}", file=sys.stderr, flush=True)
                    continue
                if writer is not None:
                    writer.write(stdin_frame)
                    sent += 1
                _print_frame(f"stdin[{lineno}]", stdin_command, stdin_channels, stdin_frame)
            print(f"done sent={sent}", flush=True)
            return 0

        if float(args.duration_s) <= 0.0:
            if writer is not None:
                writer.write(frame)
            _print_frame("sent", command, channels, frame)
            return 0

        period_s = 1.0 / max(1.0e-6, float(args.rate_hz))
        end = time.monotonic() + max(0.0, float(args.duration_s))
        next_print = 0.0
        sent = 0
        while time.monotonic() < end:
            if writer is not None:
                writer.write(frame)
            sent += 1
            now = time.monotonic()
            if now >= next_print:
                _print_frame(f"sent[{sent}]", command, channels, frame)
                next_print = now + max(0.0, float(args.print_every_s))
            sleep_s = min(period_s, max(0.0, end - time.monotonic()))
            if sleep_s > 0.0:
                time.sleep(sleep_s)
        print(f"done sent={sent}", flush=True)
        return 0
    finally:
        if writer is not None and not bool(args.no_stop_neutral):
            stop_channels, stop_frame = _frame_for(stop_command, stop_config)
            try:
                writer.write(stop_frame)
                _print_frame("stop", stop_command, stop_channels, stop_frame)
            except OSError as exc:
                print(f"warning: failed to send neutral stop frame: {exc}", file=sys.stderr, flush=True)
        if writer is not None:
            writer.close()


if __name__ == "__main__":
    raise SystemExit(main())
