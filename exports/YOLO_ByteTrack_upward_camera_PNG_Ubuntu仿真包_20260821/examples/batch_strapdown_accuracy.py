from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Iterable


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Batch-test strapdown vision PNG accuracy in AirSim headless mode.")
    parser.add_argument("--ranges", nargs="+", type=float, default=[60.0, 80.0, 100.0, 120.0])
    parser.add_argument("--altitude-offsets", nargs="+", type=float, default=[10.0, 20.0, 30.0])
    parser.add_argument("--lateral-offset-m", type=float, default=-20.0)
    parser.add_argument("--duration-s", type=float, default=25.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--intruder-speed", type=float, default=5.0)
    parser.add_argument("--speed-ratio", type=float, default=2.0)
    parser.add_argument("--intercept-altitude-m", type=float, default=50.0)
    parser.add_argument("--trajectory-dir", default=str(PROJECT_ROOT / "logs" / "strapdown_accuracy"))
    parser.add_argument("--prefix", default="")
    parser.add_argument("--settings-path", default=str(PROJECT_ROOT / "config" / "airsim_blocks_settings.json"))
    parser.add_argument("--print-every-n", type=int, default=0)
    parser.add_argument("--continue-on-fail", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--no-reset-between-runs", action="store_true")
    parser.add_argument(
        "--summarize-prefix",
        default="",
        help="Only summarize existing CSV files matching this prefix in --trajectory-dir.",
    )
    parser.add_argument("extra_args", nargs=argparse.REMAINDER, help="Extra args passed after -- to run_airsim_strapdown_vision_png.py")
    return parser.parse_args()


def _read_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _finite_float(value) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if number == number and abs(number) != float("inf") else None


def _csv_summary(csv_path: Path) -> dict:
    rows: list[dict[str, str]] = []
    with csv_path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    ranges = [value for row in rows if (value := _finite_float(row.get("range"))) is not None]
    wall_fps = [value for row in rows if (value := _finite_float(row.get("wall_fps"))) is not None and value > 0.0]
    sim_fps = [value for row in rows if (value := _finite_float(row.get("sim_sample_fps"))) is not None and value > 0.0]
    sim_clock_ratio = [value for row in rows if (value := _finite_float(row.get("sim_clock_ratio"))) is not None and value > 0.0]
    load_g = [value for row in rows if (value := _finite_float(row.get("load_factor_g"))) is not None]
    load_fd_g = [value for row in rows if (value := _finite_float(row.get("load_factor_fd_g"))) is not None]
    detected = sum(row.get("detected") == "1" for row in rows)
    valid = sum(row.get("valid") == "1" for row in rows)
    hit_rows = [row for row in rows if row.get("hit") == "1"]
    near_hit_rows = [row for row in rows if row.get("near_hit") == "1"]
    terminal_states: dict[str, int] = {}
    guidance_modes: dict[str, int] = {}
    for row in rows:
        terminal_states[row.get("terminal_state", "")] = terminal_states.get(row.get("terminal_state", ""), 0) + 1
        guidance_modes[row.get("guidance_mode", "")] = guidance_modes.get(row.get("guidance_mode", ""), 0) + 1
    return {
        "frames": len(rows),
        "detected_frames": detected,
        "valid_frames": valid,
        "hit": bool(hit_rows),
        "hit_t_s": _finite_float(hit_rows[0].get("t")) if hit_rows else None,
        "near_hit": bool(near_hit_rows),
        "near_hit_t_s": _finite_float(near_hit_rows[0].get("t")) if near_hit_rows else None,
        "near_hit_range_m": _finite_float(near_hit_rows[0].get("range")) if near_hit_rows else None,
        "min_range_m": min(ranges) if ranges else None,
        "final_range_m": ranges[-1] if ranges else None,
        "avg_wall_fps": sum(wall_fps) / len(wall_fps) if wall_fps else None,
        "avg_sim_sample_fps": sum(sim_fps) / len(sim_fps) if sim_fps else None,
        "avg_sim_clock_ratio": sum(sim_clock_ratio) / len(sim_clock_ratio) if sim_clock_ratio else None,
        "avg_load_factor_g": sum(load_g) / len(load_g) if load_g else None,
        "max_load_factor_g": max(load_g) if load_g else None,
        "avg_load_factor_fd_g": sum(load_fd_g) / len(load_fd_g) if load_fd_g else None,
        "max_load_factor_fd_g": max(load_fd_g) if load_fd_g else None,
        "terminal_states": terminal_states,
        "guidance_modes": guidance_modes,
    }


def _case_name(prefix: str, range_m: float, alt_m: float) -> str:
    base = prefix or time.strftime("strapdown_accuracy_%Y%m%d-%H%M%S")
    return f"{base}_r{range_m:.0f}_h{alt_m:.0f}".replace("-", "m").replace(".", "p")


def _case_args(args: argparse.Namespace, case_prefix: str, range_m: float, alt_m: float, reset: bool) -> list[str]:
    command = [
        sys.executable,
        str(PROJECT_ROOT / "examples" / "run_airsim_strapdown_vision_png.py"),
        "--enable-motion",
        "--duration-s",
        str(args.duration_s),
        "--rate-hz",
        str(args.rate_hz),
        "--intruder-speed",
        str(args.intruder_speed),
        "--speed-ratio",
        str(args.speed_ratio),
        "--intercept-altitude-m",
        str(args.intercept_altitude_m),
        "--intruder-altitude-offset-m",
        str(alt_m),
        "--start-horizontal-range-m",
        str(range_m),
        "--start-lateral-offset-m",
        str(args.lateral_offset_m),
        "--trajectory-dir",
        str(args.trajectory_dir),
        "--trajectory-prefix",
        case_prefix,
        "--settings-path",
        str(args.settings_path),
        "--no-show-window",
        "--no-record-preview",
        "--no-plot",
        "--print-every-n",
        str(args.print_every_n),
    ]
    command.append("--reset" if reset else "--no-reset")
    extra_args = list(args.extra_args)
    if extra_args and extra_args[0] == "--":
        extra_args = extra_args[1:]
    command.extend(extra_args)
    return command


def _write_summary(rows: Iterable[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    fields = [
        "case",
        "start_horizontal_range_m",
        "intruder_altitude_offset_m",
        "lateral_offset_m",
        "hit",
        "hit_t_s",
        "near_hit",
        "near_hit_t_s",
        "near_hit_range_m",
        "min_range_m",
        "final_range_m",
        "frames",
        "detected_frames",
        "valid_frames",
        "avg_wall_fps",
        "avg_sim_sample_fps",
        "avg_sim_clock_ratio",
        "avg_load_factor_g",
        "max_load_factor_g",
        "avg_load_factor_fd_g",
        "max_load_factor_fd_g",
        "returncode",
        "csv_path",
        "meta_path",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    out_dir = Path(args.trajectory_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    batch_prefix = args.prefix or time.strftime("strapdown_accuracy_%Y%m%d-%H%M%S")
    summary_rows: list[dict] = []

    if args.summarize_prefix:
        for csv_path in sorted(out_dir.glob(f"{args.summarize_prefix}_r*_h*.csv")):
            case = csv_path.stem
            meta_path = csv_path.with_name(f"{case}_meta.json")
            summary = {
                "case": case,
                "start_horizontal_range_m": "",
                "intruder_altitude_offset_m": "",
                "lateral_offset_m": "",
                "returncode": "",
                "csv_path": str(csv_path),
                "meta_path": str(meta_path),
            }
            if meta_path.exists():
                meta = _read_json(meta_path)
                case_args = meta.get("args", {})
                summary.update(
                    {
                        "start_horizontal_range_m": case_args.get("start_horizontal_range_m", ""),
                        "intruder_altitude_offset_m": case_args.get("intruder_altitude_offset_m", ""),
                        "lateral_offset_m": case_args.get("start_lateral_offset_m", ""),
                    }
                )
            summary.update(_csv_summary(csv_path))
            summary_rows.append(summary)
        summary_path = out_dir / f"{args.summarize_prefix}_summary.csv"
        _write_summary(summary_rows, summary_path)
        print(f"strapdown_accuracy_summary={summary_path}")
        return

    first = True
    for range_m in args.ranges:
        for alt_m in args.altitude_offsets:
            case_prefix = _case_name(batch_prefix, range_m, alt_m)
            reset = first or not args.no_reset_between_runs
            first = False
            command = _case_args(args, case_prefix, range_m, alt_m, reset)
            print(f"\n=== case={case_prefix} range={range_m:.1f}m alt_offset={alt_m:.1f}m ===", flush=True)
            print(" ".join(command), flush=True)
            completed = subprocess.run(command, cwd=PROJECT_ROOT, text=True)
            csv_path = out_dir / f"{case_prefix}.csv"
            meta_path = out_dir / f"{case_prefix}_meta.json"
            summary = {
                "case": case_prefix,
                "start_horizontal_range_m": range_m,
                "intruder_altitude_offset_m": alt_m,
                "lateral_offset_m": args.lateral_offset_m,
                "hit": False,
                "hit_t_s": None,
                "near_hit": False,
                "near_hit_t_s": None,
                "near_hit_range_m": None,
                "min_range_m": None,
                "final_range_m": None,
                "frames": 0,
                "detected_frames": 0,
                "valid_frames": 0,
                "returncode": completed.returncode,
                "csv_path": str(csv_path),
                "meta_path": str(meta_path),
                "avg_wall_fps": None,
                "avg_sim_sample_fps": None,
                "avg_sim_clock_ratio": None,
                "avg_load_factor_g": None,
                "max_load_factor_g": None,
                "avg_load_factor_fd_g": None,
                "max_load_factor_fd_g": None,
            }
            if csv_path.exists():
                summary.update(_csv_summary(csv_path))
            elif meta_path.exists():
                meta = _read_json(meta_path)
                derived = meta.get("derived", {})
                summary.update(
                    {
                        "hit": bool(derived.get("hit", False)),
                        "near_hit": bool(derived.get("near_hit", False)),
                        "min_range_m": derived.get("min_range_m"),
                        "final_range_m": derived.get("final_range_m"),
                        "frames": derived.get("frame_count", 0),
                    }
                )
            summary_rows.append(summary)
            print(
                "case_result="
                f"hit={int(bool(summary['hit']))}, near_hit={int(bool(summary.get('near_hit')))}, "
                f"min_range={summary['min_range_m']}, "
                f"final_range={summary['final_range_m']}, frames={summary['frames']}, "
                f"avg_sim_fps={summary.get('avg_sim_sample_fps')}, "
                f"max_load_g={summary.get('max_load_factor_g')}, "
                f"returncode={completed.returncode}",
                flush=True,
            )
            if completed.returncode != 0 and not args.continue_on_fail:
                break

    summary_path = out_dir / f"{batch_prefix}_summary.csv"
    _write_summary(summary_rows, summary_path)
    print(f"\nstrapdown_accuracy_summary={summary_path}")


if __name__ == "__main__":
    main()
