#!/usr/bin/env python3
"""Run deterministic matrix15 closed-loop tests for the Betaflight PNG mapping."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision_guidance.betaflight_png_sim import (  # noqa: E402
    CONTROLLER_MODES,
    START_PROFILES,
    ClosedLoopSimulationConfig,
    simulate_matrix15,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="Output JSON path.")
    parser.add_argument("--csv", default="", help="Optional per-case CSV path.")
    parser.add_argument("--dt-s", type=float, default=0.01)
    parser.add_argument("--duration-s", type=float, default=40.0)
    parser.add_argument("--navigation-constant", type=float, default=3.0)
    parser.add_argument("--guidance-accel-limit-m-s2", type=float, default=20.0)
    parser.add_argument("--max-tilt-deg", type=float, default=35.0)
    parser.add_argument("--max-rate-deg-s", type=float, default=120.0)
    parser.add_argument("--attitude-kp-s-inv", type=float, default=4.0)
    parser.add_argument("--body-rate-response-tau-s", type=float, default=0.04)
    parser.add_argument("--perception-latency-s", type=float, default=0.0)
    parser.add_argument(
        "--perception-rate-hz",
        type=float,
        default=0.0,
        help="Sampled LOS rate; 0 samples every simulation step.",
    )
    parser.add_argument("--perception-stale-timeout-s", type=float, default=0.35)
    parser.add_argument(
        "--perception-fov-gate",
        action="store_true",
        help="Reject LOS samples outside the configured camera FOV.",
    )
    parser.add_argument("--random-seed", type=int, default=0)
    parser.add_argument("--measurement-dropout-probability", type=float, default=0.0)
    parser.add_argument("--los-angle-noise-std-deg", type=float, default=0.0)
    parser.add_argument("--relative-velocity-noise-std-m-s", type=float, default=0.0)
    parser.add_argument("--wind-accel-std-m-s2", type=float, default=0.0)
    parser.add_argument("--wind-time-constant-s", type=float, default=1.0)
    parser.add_argument(
        "--controller-modes",
        default=",".join(CONTROLLER_MODES),
        help="Comma-separated controller modes.",
    )
    parser.add_argument(
        "--start-profiles",
        default=",".join(START_PROFILES),
        help="Comma-separated start profiles.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    modes = _choices(args.controller_modes, CONTROLLER_MODES, "controller mode")
    starts = _choices(args.start_profiles, START_PROFILES, "start profile")
    config = ClosedLoopSimulationConfig(
        dt_s=args.dt_s,
        duration_s=args.duration_s,
        navigation_constant=args.navigation_constant,
        guidance_accel_limit_m_s2=args.guidance_accel_limit_m_s2,
        max_roll_tilt_deg=args.max_tilt_deg,
        max_pitch_tilt_deg=args.max_tilt_deg,
        max_roll_rate_deg_s=args.max_rate_deg_s,
        max_pitch_rate_deg_s=args.max_rate_deg_s,
        attitude_kp_s_inv=args.attitude_kp_s_inv,
        body_rate_response_tau_s=args.body_rate_response_tau_s,
        perception_latency_s=args.perception_latency_s,
        perception_rate_hz=args.perception_rate_hz,
        perception_stale_timeout_s=args.perception_stale_timeout_s,
        perception_fov_gate_enabled=args.perception_fov_gate,
        random_seed=args.random_seed,
        measurement_dropout_probability=args.measurement_dropout_probability,
        los_angle_noise_std_deg=args.los_angle_noise_std_deg,
        relative_velocity_noise_std_m_s=args.relative_velocity_noise_std_m_s,
        wind_accel_std_m_s2=args.wind_accel_std_m_s2,
        wind_time_constant_s=args.wind_time_constant_s,
    )
    report = simulate_matrix15(
        config=config, controller_modes=modes, start_profiles=starts
    )
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if args.csv:
        _write_csv(Path(args.csv).expanduser().resolve(), report["results"])
    for summary in report["summaries"]:
        print(
            f"start={summary['start_profile']} mode={summary['controller_mode']} "
            f"hit={summary['hit_count']}/{summary['case_count']} "
            f"near={summary['near_hit_count']}/{summary['case_count']} "
            f"fov_hit={summary['fov_feasible_hit_count']}/{summary['case_count']} "
            f"visible_hit={summary['initially_visible_hit_count']}/"
            f"{summary['initially_visible_case_count']} "
            f"measurement_valid={summary['mean_measurement_valid_fraction']:.3f} "
            f"mean_min={summary['minimum_range_mean_m']:.3f}m "
            f"worst_min={summary['minimum_range_worst_m']:.3f}m"
        )
    print(f"output={output_path}")


def _choices(raw: str, allowed: tuple[str, ...], label: str) -> tuple[str, ...]:
    values = tuple(value.strip() for value in raw.split(",") if value.strip())
    if not values:
        raise SystemExit(f"at least one {label} is required")
    invalid = [value for value in values if value not in allowed]
    if invalid:
        raise SystemExit(
            f"unsupported {label}: {', '.join(invalid)}; allowed: {', '.join(allowed)}"
        )
    return values


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
