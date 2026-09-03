#!/usr/bin/env python3
"""Estimate a remote host clock offset with an NTP-style TCP exchange."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import socket
import socketserver
import sys
import time
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision_guidance.camera_latency_probe import ntp_clock_offset_and_delay_s  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode", required=True)

    serve = subparsers.add_parser("serve", help="Serve timestamps on the camera host.")
    serve.add_argument("--bind", default="0.0.0.0")
    serve.add_argument("--port", type=int, default=8099)
    serve.add_argument("--duration-s", type=float, default=90.0)

    probe = subparsers.add_parser("probe", help="Probe the camera host from the display PC.")
    probe.add_argument("--host", required=True)
    probe.add_argument("--port", type=int, default=8099)
    probe.add_argument("--samples", type=int, default=50)
    probe.add_argument("--interval-ms", type=float, default=20.0)
    probe.add_argument("--timeout-s", type=float, default=2.0)
    probe.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


class _ClockRequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        self.connection.settimeout(2.0)
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        while True:
            try:
                request = self.rfile.readline(4096)
            except (OSError, TimeoutError):
                return
            if not request:
                return
            server_receive_unix_ns = time.time_ns()
            try:
                request_value = json.loads(request)
                sequence = int(request_value["sequence"])
            except (json.JSONDecodeError, KeyError, TypeError, ValueError):
                return
            response: dict[str, Any] = {
                "sequence": sequence,
                "server_receive_unix_ns": server_receive_unix_ns,
                "server_send_unix_ns": time.time_ns(),
            }
            self.wfile.write((json.dumps(response, separators=(",", ":")) + "\n").encode("ascii"))
            self.wfile.flush()


class _ClockServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def run_server(args: argparse.Namespace) -> None:
    _validate_port(args.port)
    if not math.isfinite(args.duration_s) or args.duration_s <= 0.0:
        raise ValueError("server duration must be positive and finite")
    with _ClockServer((args.bind, args.port), _ClockRequestHandler) as server:
        server.timeout = 0.5
        deadline_s = time.monotonic() + args.duration_s
        print(
            f"clock_server_listening={args.bind}:{args.port} duration_s={args.duration_s:.1f}",
            flush=True,
        )
        while time.monotonic() < deadline_s:
            server.handle_request()
    print("clock_server_stopped=1")


def run_probe(args: argparse.Namespace) -> None:
    _validate_port(args.port)
    if args.samples < 5:
        raise ValueError("at least five clock samples are required")
    if not math.isfinite(args.interval_ms) or args.interval_ms < 0.0:
        raise ValueError("probe interval must be finite and non-negative")
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0.0:
        raise ValueError("probe timeout must be positive and finite")

    samples: list[dict[str, Any]] = []
    with socket.create_connection((args.host, args.port), timeout=args.timeout_s) as connection:
        connection.settimeout(args.timeout_s)
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        response_stream = connection.makefile("rb")
        for sequence in range(1, args.samples + 1):
            request = json.dumps({"sequence": sequence}, separators=(",", ":")) + "\n"
            client_send_unix_ns = time.time_ns()
            connection.sendall(request.encode("ascii"))
            response_data = response_stream.readline(4096)
            client_receive_unix_ns = time.time_ns()
            if not response_data:
                raise RuntimeError("clock server closed the connection")
            response = json.loads(response_data)
            if int(response["sequence"]) != sequence:
                raise RuntimeError("clock probe response sequence mismatch")
            values_s = tuple(
                value / 1e9
                for value in (
                    client_send_unix_ns,
                    int(response["server_receive_unix_ns"]),
                    int(response["server_send_unix_ns"]),
                    client_receive_unix_ns,
                )
            )
            offset_s, delay_s = ntp_clock_offset_and_delay_s(*values_s)
            samples.append(
                {
                    "sequence": sequence,
                    "camera_minus_display_clock_ms": 1000.0 * offset_s,
                    "network_delay_ms": 1000.0 * delay_s,
                }
            )
            if sequence < args.samples and args.interval_ms > 0.0:
                time.sleep(args.interval_ms / 1000.0)

    selected = min(samples, key=lambda sample: sample["network_delay_ms"])
    offsets = np.asarray(
        [sample["camera_minus_display_clock_ms"] for sample in samples],
        dtype=float,
    )
    delays = np.asarray([sample["network_delay_ms"] for sample in samples], dtype=float)
    best_count = max(3, int(math.ceil(0.2 * len(samples))))
    best_offsets = offsets[np.argsort(delays)[:best_count]]
    output = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "host": args.host,
        "port": args.port,
        "sample_count": len(samples),
        "selected_sequence": selected["sequence"],
        "camera_minus_display_clock_ms": selected["camera_minus_display_clock_ms"],
        "clock_uncertainty_ms": 0.5 * selected["network_delay_ms"],
        "minimum_network_delay_ms": selected["network_delay_ms"],
        "best_20_percent_offset_span_ms": float(np.ptp(best_offsets)),
        "offset_ms": _distribution(offsets),
        "network_delay_ms": _distribution(delays),
        "samples": samples,
        "semantics": (
            "NTP four-timestamp estimate; offset is camera/server clock minus display/client clock, "
            "and half minimum network delay is the optimistic symmetry uncertainty bound"
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(
        f"camera_minus_display_clock_ms={output['camera_minus_display_clock_ms']:.3f} "
        f"clock_uncertainty_ms={output['clock_uncertainty_ms']:.3f} "
        f"best_offset_span_ms={output['best_20_percent_offset_span_ms']:.3f}"
    )
    print(f"clock_probe_json={args.output}")


def _distribution(values: np.ndarray) -> dict[str, float]:
    return {
        "p50": float(np.percentile(values, 50.0)),
        "p95": float(np.percentile(values, 95.0)),
        "p99": float(np.percentile(values, 99.0)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
    }


def _validate_port(port: int) -> None:
    if not 1 <= port <= 65535:
        raise ValueError("port must be within [1, 65535]")


def main() -> None:
    args = parse_args()
    if args.mode == "serve":
        run_server(args)
    else:
        run_probe(args)


if __name__ == "__main__":
    main()
