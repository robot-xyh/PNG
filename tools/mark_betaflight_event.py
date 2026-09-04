#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision_guidance.runtime_evidence import append_operator_marker  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Append a durable marker to a running Betaflight experiment.")
    parser.add_argument("--marker-file", required=True)
    parser.add_argument("--event", required=True)
    parser.add_argument("--note", default="")
    parser.add_argument("--tag", action="append", default=[])
    args = parser.parse_args()
    record = append_operator_marker(
        Path(args.marker_file).expanduser().resolve(),
        event=args.event,
        note=args.note,
        tags=args.tag,
    )
    print(f"marker={record['event']} monotonic_s={record['monotonic_s']:.6f}")


if __name__ == "__main__":
    main()
