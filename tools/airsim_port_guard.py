#!/usr/bin/env python3
"""Prepare AirSim settings with conflict-free local ports.

The script is intentionally small and dependency-free because it runs before
Blocks starts.  It reads an AirSim settings JSON, checks the RPC and PX4/MAVLink
ports, and writes a temporary settings file only when a rewrite is needed.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import socket
import sys
import time
from pathlib import Path
from typing import Any


TCP_FIELDS = {"ApiServerPort", "TcpPort"}
UDP_FIELDS = {
    "ControlPortLocal",
    "ControlPortRemote",
    "QgcPort",
    "LogViewerPort",
    "UdpPort",
}
DEFAULT_API_PORT = 41451
DEFAULT_HOST = "127.0.0.1"


def _port_free(port: int, kind: str, host: str = DEFAULT_HOST) -> bool:
    if port <= 0:
        return True
    sock_type = socket.SOCK_DGRAM if kind == "udp" else socket.SOCK_STREAM
    with socket.socket(socket.AF_INET, sock_type) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((host, int(port)))
        except OSError:
            return False
    return True


def _port_usable(port: int, kind: str, reserved: set[int]) -> bool:
    if port in reserved:
        return False
    if kind == "any":
        return _port_free(port, "tcp") and _port_free(port, "udp")
    return _port_free(port, kind)


def _next_free(start: int, kind: str, reserved: set[int]) -> int:
    port = max(1024, int(start))
    for candidate in range(port, 65535):
        if _port_usable(candidate, kind, reserved):
            reserved.add(candidate)
            return candidate
    raise RuntimeError(f"no free {kind} port found from {start}")


def _iter_vehicle_port_fields(settings: dict[str, Any]):
    vehicles = settings.get("Vehicles", {})
    if not isinstance(vehicles, dict):
        return
    for vehicle_name, vehicle in vehicles.items():
        if not isinstance(vehicle, dict):
            continue
        for field in sorted(TCP_FIELDS | UDP_FIELDS):
            value = vehicle.get(field)
            if isinstance(value, int) and value > 0:
                kind = "tcp" if field in TCP_FIELDS else "udp"
                yield vehicle_name, vehicle, field, value, kind


def _json_dump(path: Path, settings: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(settings, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _write_env(path: Path, values: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"{key}={shlex.quote(str(value))}" for key, value in values.items()]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check and allocate AirSim Blocks ports.")
    parser.add_argument("--settings", required=True, help="Input AirSim settings JSON.")
    parser.add_argument("--output-dir", default=".airsim_runtime/settings", help="Directory for rewritten settings.")
    parser.add_argument("--env-path", default="", help="Optional env file to write.")
    parser.add_argument("--policy", default=os.environ.get("AIRSIM_PORT_POLICY", "auto"), choices=("auto", "strict", "off"))
    parser.add_argument("--label", default=os.environ.get("AIRSIM_INSTANCE_LABEL", "blocks"))
    parser.add_argument("--rpc-base", type=int, default=int(os.environ.get("AIRSIM_RPC_PORT_BASE", DEFAULT_API_PORT)))
    parser.add_argument("--px4-base", type=int, default=int(os.environ.get("AIRSIM_PX4_PORT_BASE", 4560)))
    args = parser.parse_args()

    settings_path = Path(args.settings).expanduser().resolve()
    if not settings_path.exists():
        print(f"settings not found: {settings_path}", file=sys.stderr)
        return 2

    try:
        settings = json.loads(settings_path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"failed to read settings JSON {settings_path}: {exc}", file=sys.stderr)
        return 2
    if not isinstance(settings, dict):
        print(f"settings root must be a JSON object: {settings_path}", file=sys.stderr)
        return 2

    if args.policy == "off":
        rpc_port = int(settings.get("ApiServerPort", DEFAULT_API_PORT) or DEFAULT_API_PORT)
        values = {
            "AIRSIM_SETTINGS_PATH_RESOLVED": str(settings_path),
            "AIRSIM_RPC_HOST": DEFAULT_HOST,
            "AIRSIM_RPC_PORT": str(rpc_port),
            "AIRSIM_PORT_POLICY": "off",
            "AIRSIM_PORT_REWRITTEN": "0",
        }
        if args.env_path:
            _write_env(Path(args.env_path), values)
        print("\n".join(f"{key}={shlex.quote(value)}" for key, value in values.items()))
        return 0

    reserved: set[int] = set()
    changed = False
    notes: list[str] = []

    rpc_port = int(settings.get("ApiServerPort", DEFAULT_API_PORT) or DEFAULT_API_PORT)
    rpc_free = _port_usable(rpc_port, "tcp", reserved)
    if rpc_free:
        reserved.add(rpc_port)
    elif args.policy == "strict":
        print(f"AirSim RPC port {rpc_port} is already in use; refusing to start Blocks.", file=sys.stderr)
        return 3
    else:
        new_port = _next_free(max(args.rpc_base, rpc_port + 1), "tcp", reserved)
        settings["ApiServerPort"] = new_port
        notes.append(f"ApiServerPort {rpc_port} -> {new_port}")
        rpc_port = new_port
        changed = True

    px4_tcp_ports: list[int] = []
    for vehicle_name, vehicle, field, old_port, kind in _iter_vehicle_port_fields(settings):
        free = _port_usable(old_port, kind, reserved)
        if free:
            reserved.add(old_port)
            if field == "TcpPort":
                px4_tcp_ports.append(old_port)
            continue
        if args.policy == "strict":
            print(
                f"{vehicle_name}.{field} {old_port}/{kind} is already in use; refusing to start Blocks.",
                file=sys.stderr,
            )
            return 3
        base = args.px4_base if field == "TcpPort" else old_port + 1
        new_port = _next_free(max(base, old_port + 1), kind, reserved)
        vehicle[field] = new_port
        notes.append(f"{vehicle_name}.{field} {old_port} -> {new_port}")
        if field == "TcpPort":
            px4_tcp_ports.append(new_port)
        changed = True

    output_settings = settings_path
    if changed:
        stem = settings_path.stem
        safe_label = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in args.label)
        output_settings = Path(args.output_dir).expanduser().resolve() / f"{stem}_{safe_label}_{os.getpid()}_{int(time.time())}.json"
        _json_dump(output_settings, settings)
        print("AirSim port guard rewrote settings:", file=sys.stderr)
        for note in notes:
            print(f"  {note}", file=sys.stderr)
        print(f"  resolved_settings={output_settings}", file=sys.stderr)
    else:
        print("AirSim port guard: configured ports are free; using original settings.", file=sys.stderr)

    values = {
        "AIRSIM_SETTINGS_PATH_RESOLVED": str(output_settings),
        "AIRSIM_RPC_HOST": DEFAULT_HOST,
        "AIRSIM_RPC_PORT": str(rpc_port),
        "AIRSIM_PX4_TCP_PORTS": ",".join(str(port) for port in px4_tcp_ports),
        "AIRSIM_PORT_POLICY": args.policy,
        "AIRSIM_PORT_REWRITTEN": "1" if changed else "0",
    }
    if px4_tcp_ports:
        values["AIRSIM_PX4_TCP_PORT"] = str(px4_tcp_ports[0])
    if args.env_path:
        _write_env(Path(args.env_path), values)

    print("\n".join(f"{key}={shlex.quote(value)}" for key, value in values.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
