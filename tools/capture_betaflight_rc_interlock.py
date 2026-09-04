#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision_guidance.betaflight_msp import BetaflightMSPAdapter  # noqa: E402
from vision_guidance.betaflight_runtime import box_mode_active  # noqa: E402


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Capture evidence that a RadioMaster flight-mode switch releases MSP OVERRIDE."
    )
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--release-mode-id", type=int, action="append", default=[])
    parser.add_argument("--edgetx-model", default="")
    args = parser.parse_args()
    config_path = Path(args.config).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    serial = dict(config.get("serial", {}))
    runtime = dict(config.get("msp_runtime", {}))
    override_id = int(runtime.get("override_mode_cli_id", 50))
    release_mode_ids = tuple(args.release_mode_id or [1])
    rc7_index = int(dict(config.get("safety", {})).get("aux_enable", {}).get("channel_index", 7)) - 1
    adapter = BetaflightMSPAdapter(
        str(serial["port"]),
        int(serial.get("baud", 115200)),
        timeout_s=float(serial.get("timeout_s", 0.1)),
    )
    print("During capture: select Acro, raise RC7, then select ANGLE/rescue without lowering RC7 manually.")
    samples: list[dict[str, object]] = []
    override_seen = False
    release_mode_seen = False
    rc7_low_seen = False
    override_cleared = False
    release_latencies_ms: list[float] = []
    previous_override = False
    previous_sample_s: float | None = None
    adapter.open()
    try:
        box_ids = adapter.read_box_ids()
        deadline = time.monotonic() + max(0.1, float(args.duration_s))
        period = 1.0 / max(1.0, float(args.rate_hz))
        while time.monotonic() < deadline:
            started = time.monotonic()
            status = adapter.read_status()
            override = box_mode_active(status.mode_flags, box_ids, override_id)
            active_release_modes = [
                mode_id
                for mode_id in release_mode_ids
                if box_mode_active(status.mode_flags, box_ids, mode_id)
            ]
            channels: tuple[int, ...] = ()
            if not override:
                channels = adapter.read_rc()
            rc7_us = int(channels[rc7_index]) if len(channels) > rc7_index else None
            samples.append(
                {
                    "monotonic_s": started,
                    "override_active": override,
                    "release_mode_ids": active_release_modes,
                    "rc7_us": rc7_us,
                }
            )
            override_seen = override_seen or override
            if previous_override and not override:
                override_cleared = True
                release_mode_seen = release_mode_seen or bool(active_release_modes)
                rc7_low_seen = rc7_low_seen or bool(rc7_us is not None and rc7_us <= 1300)
                if previous_sample_s is not None:
                    release_latencies_ms.append(1000.0 * max(0.0, started - previous_sample_s))
            previous_override = override
            previous_sample_s = started
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    finally:
        adapter.close()
    max_latency_ms = max(release_latencies_ms, default=float("inf"))
    checks = {
        "override_seen": override_seen,
        "release_mode_seen": release_mode_seen,
        "rc7_low_seen": rc7_low_seen,
        "override_cleared": override_cleared,
    }
    edgetx_path = Path(args.edgetx_model).expanduser().resolve() if args.edgetx_model else None
    report = {
        "schema_version": 1,
        "purpose": "RadioMaster MSP OVERRIDE release interlock evidence",
        "created_unix_s": time.time(),
        "runtime_binding": {"path": str(config_path), "sha256": _sha256(config_path)},
        "override_mode_id": override_id,
        "required_release_mode_ids": list(release_mode_ids),
        "max_release_latency_ms": max_latency_ms,
        "checks": checks,
        "passed": all(checks.values()) and max_latency_ms <= 200.0,
        "edgetx_model": {
            "path": "" if edgetx_path is None else str(edgetx_path),
            "sha256": "" if edgetx_path is None or not edgetx_path.is_file() else _sha256(edgetx_path),
        },
        "samples": samples,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output_path)
    print(f"passed={int(report['passed'])} max_release_latency_ms={max_latency_ms:.3f}")


if __name__ == "__main__":
    main()
