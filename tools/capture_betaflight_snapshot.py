#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision_guidance.betaflight_msp import BetaflightMSPAdapter  # noqa: E402
from vision_guidance.betaflight_snapshot import capture_betaflight_snapshot  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture a read-only Betaflight MSP and CLI configuration snapshot.")
    parser.add_argument("--config", default="config/betaflight.rk3588.example.json")
    parser.add_argument("--serial-port", default="")
    parser.add_argument("--msp-baud", type=int, default=0)
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument(
        "--include-kinematics",
        action="store_true",
        help="Also capture MSP_RAW_GPS and MSP_ALTITUDE for supervised-flight approval.",
    )
    parser.add_argument(
        "--cli-export",
        default="",
        help="Legacy alias for one Configurator-generated CLI export; cannot be combined with the explicit options.",
    )
    parser.add_argument("--cli-diff-all", default="", help="Configurator-generated 'diff all' text file.")
    parser.add_argument("--cli-dump-all", default="", help="Configurator-generated 'dump all' text file.")
    parser.add_argument("--output-root", default="logs/betaflight_snapshots")
    args = parser.parse_args()
    if args.cli_export and (args.cli_diff_all or args.cli_dump_all):
        parser.error("--cli-export cannot be combined with --cli-diff-all or --cli-dump-all")

    config_path = Path(args.config).expanduser()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    serial = dict(config.get("serial", {}))
    port = str(args.serial_port or serial.get("port", ""))
    baud = int(args.msp_baud or serial.get("baud", 115200))
    if not port:
        raise RuntimeError("serial.port or --serial-port is required")
    adapter = BetaflightMSPAdapter(port, baud, timeout_s=float(serial.get("timeout_s", 0.1)))
    adapter.open()
    try:
        manifest = capture_betaflight_snapshot(
            adapter,
            args.output_root,
            duration_s=args.duration_s,
            rate_hz=args.rate_hz,
            cli_export=args.cli_export or None,
            cli_diff_all=args.cli_diff_all or None,
            cli_dump_all=args.cli_dump_all or None,
            source_reference=ROOT / "config/betaflight.src-reference.json",
            include_kinematics=args.include_kinematics,
        )
    finally:
        adapter.close()
    print(manifest)


if __name__ == "__main__":
    main()
