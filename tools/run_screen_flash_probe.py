#!/usr/bin/env python3
"""Display timed screen transitions for a screen-to-camera latency probe."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import random
from pathlib import Path
import time

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--transitions", type=int, default=40)
    parser.add_argument("--warmup-s", type=float, default=2.0)
    parser.add_argument("--tail-s", type=float, default=2.0)
    parser.add_argument("--interval-min-s", type=float, default=0.4)
    parser.add_argument("--interval-max-s", type=float, default=0.8)
    parser.add_argument("--seed", type=int, default=3588)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--patch-fraction", type=float, default=0.88)
    parser.add_argument("--dark-level", type=int, default=0)
    parser.add_argument("--bright-level", type=int, default=255)
    parser.add_argument("--background-level", type=int, default=96)
    parser.add_argument("--display-refresh-hz", type=float, default=60.0)
    parser.add_argument(
        "--fullscreen",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use an OpenCV fullscreen window; use --no-fullscreen for setup.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _validate_args(args)
    output_dir = args.output_dir or _default_output_dir("display")
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "display_transitions.csv"
    metadata_path = output_dir / "display_metadata.json"

    window_name = "Screen camera latency probe - ESC/Q to stop"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, args.width, args.height)
    if args.fullscreen:
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    frames = {
        state: _make_frame(args, state=state)
        for state in (0, 1)
    }
    rng = random.Random(args.seed)
    transitions: list[dict[str, object]] = []
    aborted = False
    start_unix_s = time.time()
    start_monotonic_s = time.monotonic()

    try:
        cv2.imshow(window_name, frames[0])
        cv2.waitKey(1)
        next_deadline = start_monotonic_s + args.warmup_s
        state = 0
        for sequence in range(1, args.transitions + 1):
            if not _wait_until(next_deadline):
                aborted = True
                break
            state = 1 - state
            request_before_monotonic_s = time.monotonic()
            request_before_unix_s = time.time()
            cv2.imshow(window_name, frames[state])
            key = cv2.waitKey(1) & 0xFF
            request_after_monotonic_s = time.monotonic()
            request_after_unix_s = time.time()
            transitions.append(
                {
                    "sequence": sequence,
                    "state": state,
                    "scheduled_monotonic_s": next_deadline,
                    "scheduled_elapsed_s": next_deadline - start_monotonic_s,
                    "request_before_monotonic_s": request_before_monotonic_s,
                    "request_before_unix_s": request_before_unix_s,
                    "request_after_monotonic_s": request_after_monotonic_s,
                    "request_after_unix_s": request_after_unix_s,
                    "display_monotonic_s": request_after_monotonic_s,
                    "display_unix_s": request_after_unix_s,
                    "event_pump_ms": 1000.0
                    * (request_after_monotonic_s - request_before_monotonic_s),
                }
            )
            print(
                f"transition={sequence:02d}/{args.transitions} state={state} "
                f"elapsed_s={request_after_monotonic_s - start_monotonic_s:.3f}",
                flush=True,
            )
            if key in (27, ord("q"), ord("Q")):
                aborted = True
                break
            next_deadline = request_after_monotonic_s + rng.uniform(
                args.interval_min_s,
                args.interval_max_s,
            )
        if not aborted:
            _wait_until(time.monotonic() + args.tail_s)
    finally:
        cv2.destroyAllWindows()

    _write_csv(csv_path, transitions)
    end_monotonic_s = time.monotonic()
    end_unix_s = time.time()
    metadata = {
        "schema_version": 1,
        "tool": str(Path(__file__).resolve()),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "csv": str(csv_path.resolve()),
        "aborted": aborted,
        "requested_transition_count": args.transitions,
        "recorded_transition_count": len(transitions),
        "seed": args.seed,
        "warmup_s": args.warmup_s,
        "tail_s": args.tail_s,
        "interval_range_s": [args.interval_min_s, args.interval_max_s],
        "window": {
            "fullscreen": args.fullscreen,
            "width": args.width,
            "height": args.height,
            "patch_fraction": args.patch_fraction,
            "dark_level": args.dark_level,
            "bright_level": args.bright_level,
            "background_level": args.background_level,
            "operator_declared_refresh_hz": args.display_refresh_hz,
        },
        "clock": {
            "start_monotonic_s": start_monotonic_s,
            "start_unix_s": start_unix_s,
            "end_monotonic_s": end_monotonic_s,
            "end_unix_s": end_unix_s,
            "unix_minus_monotonic_drift_ms": 1000.0
            * ((end_unix_s - end_monotonic_s) - (start_unix_s - start_monotonic_s)),
        },
        "timestamp_semantics": (
            "display_unix_s is recorded after cv2.waitKey pumps the imshow request; "
            "physical pixel presentation remains quantized by display refresh and compositor timing"
        ),
    }
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"display_csv={csv_path}")
    print(f"display_metadata={metadata_path}")
    print(f"recorded_transitions={len(transitions)} aborted={int(aborted)}")


def _wait_until(deadline_s: float) -> bool:
    while True:
        remaining_s = deadline_s - time.monotonic()
        if remaining_s <= 0.0:
            return True
        delay_ms = max(1, min(10, int(1000.0 * remaining_s)))
        key = cv2.waitKey(delay_ms) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            return False


def _make_frame(args: argparse.Namespace, *, state: int) -> np.ndarray:
    frame = np.full(
        (args.height, args.width, 3),
        args.background_level,
        dtype=np.uint8,
    )
    patch_width = max(1, int(round(args.width * args.patch_fraction)))
    patch_height = max(1, int(round(args.height * args.patch_fraction)))
    x0 = (args.width - patch_width) // 2
    y0 = (args.height - patch_height) // 2
    level = args.bright_level if state else args.dark_level
    frame[y0 : y0 + patch_height, x0 : x0 + patch_width] = level
    return frame


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fields = [
        "sequence",
        "state",
        "scheduled_monotonic_s",
        "scheduled_elapsed_s",
        "request_before_monotonic_s",
        "request_before_unix_s",
        "request_after_monotonic_s",
        "request_after_unix_s",
        "display_monotonic_s",
        "display_unix_s",
        "event_pump_ms",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _default_output_dir(prefix: str) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path("logs") / "camera_latency" / f"{prefix}_{stamp}"


def _validate_args(args: argparse.Namespace) -> None:
    if args.transitions <= 0:
        raise ValueError("transitions must be positive")
    if args.warmup_s < 0.0 or args.tail_s < 0.0:
        raise ValueError("warmup and tail durations must be non-negative")
    if not 0.0 < args.interval_min_s <= args.interval_max_s:
        raise ValueError("transition interval range must be positive and ordered")
    if args.width <= 0 or args.height <= 0:
        raise ValueError("display dimensions must be positive")
    if not 0.2 <= args.patch_fraction <= 1.0:
        raise ValueError("patch fraction must be within [0.2, 1.0]")
    for name in ("dark_level", "bright_level", "background_level"):
        if not 0 <= int(getattr(args, name)) <= 255:
            raise ValueError(f"{name} must be within [0, 255]")
    if args.bright_level - args.dark_level < 40:
        raise ValueError("bright and dark levels must differ by at least 40")
    if args.display_refresh_hz <= 0.0:
        raise ValueError("display refresh rate must be positive")


if __name__ == "__main__":
    main()
