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
DEFAULT_HOST = "127.0.0.2"
HOST_IP_FIELDS = {"LocalHostIp", "ControlIp", "UdpIp"}
DEFAULT_LOOPBACK_HOSTS = {"127.0.0.1", "localhost"}


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


def _port_usable(port: int, kind: str, reserved: set[int], host: str) -> bool:
    if port in reserved:
        return False
    if kind == "any":
        return _port_free(port, "tcp", host) and _port_free(port, "udp", host)
    return _port_free(port, kind, host)


def _next_free(start: int, kind: str, reserved: set[int], host: str) -> int:
    port = max(1024, int(start))
    for candidate in range(port, 65535):
        if _port_usable(candidate, kind, reserved, host):
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


def _normalize_loopback_host_ips(value: Any, host: str, notes: list[str], path: str = "") -> bool:
    if not host:
        return False
    changed = False
    if isinstance(value, dict):
        for key, child in value.items():
            child_path = f"{path}.{key}" if path else str(key)
            if key in HOST_IP_FIELDS and isinstance(child, str) and child in DEFAULT_LOOPBACK_HOSTS and child != host:
                value[key] = host
                notes.append(f"{child_path} {child} -> {host}")
                changed = True
                continue
            if isinstance(child, (dict, list)):
                changed = _normalize_loopback_host_ips(child, host, notes, child_path) or changed
    elif isinstance(value, list):
        for index, child in enumerate(value):
            if isinstance(child, (dict, list)):
                changed = _normalize_loopback_host_ips(child, host, notes, f"{path}[{index}]") or changed
    return changed


def _write_env(path: Path, values: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"{key}={shlex.quote(str(value))}" for key, value in values.items()]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _runtime_settings_path(settings_path: Path, output_dir: str, label: str) -> Path:
    stem = settings_path.stem
    safe_label = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in label)
    return Path(output_dir).expanduser().resolve() / f"{stem}_{safe_label}_{os.getpid()}_{int(time.time())}.json"


def _write_rewritten_settings(
    *,
    settings_path: Path,
    settings: dict[str, Any],
    output_dir: str,
    label: str,
    notes: list[str],
    reason: str,
) -> Path:
    output_settings = _runtime_settings_path(settings_path, output_dir, label)
    _json_dump(output_settings, settings)
    print(f"AirSim port guard rewrote settings ({reason}):", file=sys.stderr)
    for note in notes:
        print(f"  {note}", file=sys.stderr)
    print(f"  resolved_settings={output_settings}", file=sys.stderr)
    return output_settings


def main() -> int:
    parser = argparse.ArgumentParser(description="Check and allocate AirSim Blocks ports.")
    parser.add_argument("--settings", required=True, help="Input AirSim settings JSON.")
    parser.add_argument("--output-dir", default=".airsim_runtime/settings", help="Directory for rewritten settings.")
    parser.add_argument("--env-path", default="", help="Optional env file to write.")
    parser.add_argument("--policy", default=os.environ.get("AIRSIM_PORT_POLICY", "auto"), choices=("auto", "strict", "off"))
    parser.add_argument("--label", default=os.environ.get("AIRSIM_INSTANCE_LABEL", "blocks"))
    parser.add_argument("--host", default=os.environ.get("AIRSIM_RPC_HOST", DEFAULT_HOST))
    parser.add_argument(
        "--rewrite-host-ips",
        action="store_true",
        default=str(os.environ.get("AIRSIM_REWRITE_HOST_IPS", "0")).strip().lower() in {"1", "true", "yes", "on"},
        help=(
            "Rewrite PX4 vehicle loopback host fields to --host. Disabled by default because "
            "PX4 SITL normally connects to AirSim on 127.0.0.1 even when RPC clients use 127.0.0.2."
        ),
    )
    parser.add_argument("--rpc-base", type=int, default=int(os.environ.get("AIRSIM_RPC_PORT_BASE", DEFAULT_API_PORT)))
    parser.add_argument("--px4-base", type=int, default=int(os.environ.get("AIRSIM_PX4_PORT_BASE", 4560)))
    args = parser.parse_args()
    host = str(args.host or DEFAULT_HOST)

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
        notes: list[str] = []
        changed = _normalize_loopback_host_ips(settings, host, notes) if args.rewrite_host_ips else False
        output_settings = settings_path
        if changed:
            output_settings = _write_rewritten_settings(
                settings_path=settings_path,
                settings=settings,
                output_dir=args.output_dir,
                label=args.label,
                notes=notes,
                reason="host",
            )
        else:
            print("AirSim port guard: port policy off; using original settings.", file=sys.stderr)
        rpc_port = int(settings.get("ApiServerPort", DEFAULT_API_PORT) or DEFAULT_API_PORT)
        values = {
            "AIRSIM_SETTINGS_PATH_RESOLVED": str(output_settings),
            "AIRSIM_RPC_HOST": host,
            "AIRSIM_RPC_PORT": str(rpc_port),
            "AIRSIM_PORT_POLICY": "off",
            "AIRSIM_PORT_REWRITTEN": "1" if changed else "0",
        }
        if args.env_path:
            _write_env(Path(args.env_path), values)
        print("\n".join(f"{key}={shlex.quote(value)}" for key, value in values.items()))
        return 0

    reserved: set[int] = set()
    changed = False
    notes: list[str] = []
    if args.rewrite_host_ips:
        changed = _normalize_loopback_host_ips(settings, host, notes) or changed

    rpc_port = int(settings.get("ApiServerPort", DEFAULT_API_PORT) or DEFAULT_API_PORT)
    rpc_free = _port_usable(rpc_port, "tcp", reserved, host)
    if rpc_free:
        reserved.add(rpc_port)
    elif args.policy == "strict":
        print(f"AirSim RPC port {rpc_port} is already in use; refusing to start Blocks.", file=sys.stderr)
        return 3
    else:
        new_port = _next_free(max(args.rpc_base, rpc_port + 1), "tcp", reserved, host)
        settings["ApiServerPort"] = new_port
        notes.append(f"ApiServerPort {rpc_port} -> {new_port}")
        rpc_port = new_port
        changed = True

    px4_tcp_ports: list[int] = []
    for vehicle_name, vehicle, field, old_port, kind in _iter_vehicle_port_fields(settings):
        free = _port_usable(old_port, kind, reserved, host)
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
        new_port = _next_free(max(base, old_port + 1), kind, reserved, host)
        vehicle[field] = new_port
        notes.append(f"{vehicle_name}.{field} {old_port} -> {new_port}")
        if field == "TcpPort":
            px4_tcp_ports.append(new_port)
        changed = True

    output_settings = settings_path
    if changed:
        output_settings = _write_rewritten_settings(
            settings_path=settings_path,
            settings=settings,
            output_dir=args.output_dir,
            label=args.label,
            notes=notes,
            reason="host/port",
        )
    else:
        print("AirSim port guard: configured ports are free; using original settings.", file=sys.stderr)

    values = {
        "AIRSIM_SETTINGS_PATH_RESOLVED": str(output_settings),
        "AIRSIM_RPC_HOST": host,
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
