from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path
import sys
from typing import Iterable, Optional

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from vision_guidance.truth_png import compute_truth_png


GRAVITY_MPS2 = 9.80665


def _finite_float(value: object) -> Optional[float]:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        result = float(text)
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def _vector_from_row(row: dict[str, str], keys: tuple[str, str, str]) -> Optional[np.ndarray]:
    values = [_finite_float(row.get(key)) for key in keys]
    if any(value is None for value in values):
        return None
    return np.array(values, dtype=float)


def _percentile(values: list[float], q: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    if len(finite) == 1:
        return finite[0]
    position = (len(finite) - 1) * q
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return finite[lower]
    fraction = position - lower
    return finite[lower] * (1.0 - fraction) + finite[upper] * fraction


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def _resolve(path_text: str) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else PROJECT_ROOT / path


def _paths_from_summary(summary_csv: Path) -> list[tuple[Path, dict[str, str]]]:
    rows = _read_rows(summary_csv)
    result: list[tuple[Path, dict[str, str]]] = []
    for row in rows:
        csv_path = row.get("csv_path")
        if not csv_path:
            continue
        path = _resolve(csv_path)
        if path.exists():
            result.append((path, row))
    return result


def _paths_from_inputs(inputs: Iterable[str]) -> list[tuple[Path, dict[str, str]]]:
    result: list[tuple[Path, dict[str, str]]] = []
    for item in inputs:
        for path in sorted(PROJECT_ROOT.glob(item) if not Path(item).is_absolute() else Path("/").glob(str(Path(item).relative_to("/")))):
            if path.is_file():
                result.append((path, {}))
    return result


def _finite_difference(
    times: list[float],
    positions: list[Optional[np.ndarray]],
    min_dt: float,
) -> tuple[list[Optional[np.ndarray]], list[float]]:
    velocities: list[Optional[np.ndarray]] = [None] * len(times)
    spans: list[float] = [math.nan] * len(times)
    if len(times) < 2:
        return velocities, spans

    for index in range(len(times)):
        if index == 0:
            left, right = 0, 1
        elif index == len(times) - 1:
            left, right = len(times) - 2, len(times) - 1
        else:
            left, right = index - 1, index + 1

        if positions[left] is None or positions[right] is None:
            continue
        dt = times[right] - times[left]
        spans[index] = dt
        if not math.isfinite(dt) or dt <= min_dt:
            continue
        velocities[index] = (positions[right] - positions[left]) / dt

    return velocities, spans


def _velocity_command_required_load(rows: list[dict[str, str]], min_dt: float) -> list[float]:
    result: list[float] = []
    previous_t: Optional[float] = None
    previous_v: Optional[np.ndarray] = None
    for row in rows:
        t = _finite_float(row.get("t"))
        v_cmd = _vector_from_row(row, ("v_cmd_x", "v_cmd_y", "v_cmd_z"))
        if t is None or v_cmd is None or previous_t is None or previous_v is None:
            result.append(0.0 if t is not None and v_cmd is not None else math.nan)
        else:
            dt = t - previous_t
            if math.isfinite(dt) and dt > min_dt:
                result.append(float(np.linalg.norm(v_cmd - previous_v) / dt / GRAVITY_MPS2))
            else:
                result.append(math.nan)
        if t is not None and v_cmd is not None:
            previous_t = t
            previous_v = v_cmd
    return result


def _case_name(path: Path, summary_row: dict[str, str]) -> str:
    return summary_row.get("case") or path.stem


def _case_metadata(first_row: dict[str, str], summary_row: dict[str, str]) -> dict[str, str]:
    metadata: dict[str, str] = {}
    for key in (
        "start_horizontal_range_m",
        "intruder_altitude_offset_m",
        "start_lateral_offset_m",
        "speed_ratio",
        "intruder_speed",
        "rate_hz",
        "duration_s",
    ):
        value = first_row.get(key) or summary_row.get(key) or ""
        metadata[key] = value
    return metadata


def _process_case(
    path: Path,
    summary_row: dict[str, str],
    navigation_constant: float,
    min_dt: float,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    rows = _read_rows(path)
    if not rows:
        raise ValueError(f"empty CSV: {path}")

    times = [_finite_float(row.get("t")) for row in rows]
    if any(value is None for value in times):
        raise ValueError(f"missing per-frame t column in {path}")
    time_values = [float(value) for value in times if value is not None]

    interceptor_pos = [
        _vector_from_row(row, ("interceptor_x", "interceptor_y", "interceptor_z")) for row in rows
    ]
    intruder_pos = [_vector_from_row(row, ("intruder_x", "intruder_y", "intruder_z")) for row in rows]
    if any(pos is None for pos in interceptor_pos) or any(pos is None for pos in intruder_pos):
        raise ValueError(f"missing true position columns in {path}")

    interceptor_vel, velocity_spans = _finite_difference(time_values, interceptor_pos, min_dt)
    intruder_vel_fd, _ = _finite_difference(time_values, intruder_pos, min_dt)
    visual_vcmd_req_g = _velocity_command_required_load(rows, min_dt)

    case = _case_name(path, summary_row)
    metadata = _case_metadata(rows[0], summary_row)
    output_rows: list[dict[str, object]] = []
    n_req_values: list[float] = []
    n_req_valid_values: list[float] = []
    closing_values: list[float] = []

    for index, row in enumerate(rows):
        t = time_values[index]
        interceptor_p = interceptor_pos[index]
        intruder_p = intruder_pos[index]
        interceptor_v = interceptor_vel[index]
        intruder_v = intruder_vel_fd[index]

        out: dict[str, object] = {
            "case": case,
            "source_csv": str(path.relative_to(PROJECT_ROOT) if path.is_relative_to(PROJECT_ROOT) else path),
            "frame": row.get("frame", index),
            "t": t,
            "velocity_diff_span_s": velocity_spans[index],
            **metadata,
        }

        for prefix, vector in (
            ("interceptor_pos", interceptor_p),
            ("intruder_pos", intruder_p),
            ("interceptor_vel_fd", interceptor_v),
            ("intruder_vel_fd", intruder_v),
            ("intruder_vel_logged", _vector_from_row(row, ("intruder_vx", "intruder_vy", "intruder_vz"))),
        ):
            if vector is None:
                out[f"{prefix}_x"] = ""
                out[f"{prefix}_y"] = ""
                out[f"{prefix}_z"] = ""
            else:
                out[f"{prefix}_x"] = float(vector[0])
                out[f"{prefix}_y"] = float(vector[1])
                out[f"{prefix}_z"] = float(vector[2])

        if interceptor_p is None or intruder_p is None or interceptor_v is None or intruder_v is None:
            out.update(
                {
                    "relative_pos_x": "",
                    "relative_pos_y": "",
                    "relative_pos_z": "",
                    "relative_vel_x": "",
                    "relative_vel_y": "",
                    "relative_vel_z": "",
                    "range_m": "",
                    "closing_speed_mps": "",
                    "los_x": "",
                    "los_y": "",
                    "los_z": "",
                    "omega_los_x": "",
                    "omega_los_y": "",
                    "omega_los_z": "",
                    "omega_los_norm_rad_s": "",
                    "lambda_dot_x": "",
                    "lambda_dot_y": "",
                    "lambda_dot_z": "",
                    "lambda_dot_norm_rad_s": "",
                    "a_req_x": "",
                    "a_req_y": "",
                    "a_req_z": "",
                    "a_req_norm_mps2": "",
                    "n_req_g": "",
                    "truth_png_valid": 0,
                    "truth_png_reject_reason": "missing_velocity",
                }
            )
        else:
            relative_position = intruder_p - interceptor_p
            relative_velocity = intruder_v - interceptor_v
            result = compute_truth_png(
                relative_position=relative_position,
                relative_velocity=relative_velocity,
                navigation_constant=navigation_constant,
                max_accel=None,
            )
            accel_norm = float(np.linalg.norm(result.acceleration))
            n_req_g = accel_norm / GRAVITY_MPS2
            omega_norm = float(np.linalg.norm(result.omega_los))
            lambda_dot_norm = float(np.linalg.norm(result.lambda_dot))
            n_req_values.append(n_req_g)
            closing_values.append(float(result.closing_speed))
            if result.valid:
                n_req_valid_values.append(n_req_g)

            out.update(
                {
                    "relative_pos_x": float(relative_position[0]),
                    "relative_pos_y": float(relative_position[1]),
                    "relative_pos_z": float(relative_position[2]),
                    "relative_vel_x": float(relative_velocity[0]),
                    "relative_vel_y": float(relative_velocity[1]),
                    "relative_vel_z": float(relative_velocity[2]),
                    "range_m": float(result.range_m),
                    "closing_speed_mps": float(result.closing_speed),
                    "los_x": float(result.los[0]),
                    "los_y": float(result.los[1]),
                    "los_z": float(result.los[2]),
                    "omega_los_x": float(result.omega_los[0]),
                    "omega_los_y": float(result.omega_los[1]),
                    "omega_los_z": float(result.omega_los[2]),
                    "omega_los_norm_rad_s": omega_norm,
                    "lambda_dot_x": float(result.lambda_dot[0]),
                    "lambda_dot_y": float(result.lambda_dot[1]),
                    "lambda_dot_z": float(result.lambda_dot[2]),
                    "lambda_dot_norm_rad_s": lambda_dot_norm,
                    "a_req_x": float(result.acceleration[0]),
                    "a_req_y": float(result.acceleration[1]),
                    "a_req_z": float(result.acceleration[2]),
                    "a_req_norm_mps2": accel_norm,
                    "n_req_g": n_req_g,
                    "truth_png_valid": int(result.valid),
                    "truth_png_reject_reason": result.reject_reason or "",
                }
            )

        out["visual_vcmd_required_load_g"] = visual_vcmd_req_g[index]
        out["visual_actual_load_fd_g"] = _finite_float(row.get("load_factor_fd_g")) or 0.0
        out["visual_guidance_mode"] = row.get("guidance_mode", "")
        out["visual_terminal_state"] = row.get("terminal_state", "")
        out["visual_detected"] = row.get("detected", "")
        out["visual_hit"] = row.get("hit", "")
        output_rows.append(out)

    summary = {
        "case": case,
        "source_csv": str(path.relative_to(PROJECT_ROOT) if path.is_relative_to(PROJECT_ROOT) else path),
        **metadata,
        "frames": len(output_rows),
        "truth_png_valid_frames": sum(int(row["truth_png_valid"]) for row in output_rows),
        "max_theory_n_req_g": max(n_req_values) if n_req_values else math.nan,
        "p95_theory_n_req_g": _percentile(n_req_values, 0.95),
        "avg_theory_n_req_g": sum(n_req_values) / len(n_req_values) if n_req_values else math.nan,
        "max_valid_theory_n_req_g": max(n_req_valid_values) if n_req_valid_values else math.nan,
        "p95_valid_theory_n_req_g": _percentile(n_req_valid_values, 0.95),
        "avg_valid_theory_n_req_g": sum(n_req_valid_values) / len(n_req_valid_values)
        if n_req_valid_values
        else math.nan,
        "max_closing_speed_mps": max(closing_values) if closing_values else math.nan,
        "min_closing_speed_mps": min(closing_values) if closing_values else math.nan,
        "max_visual_vcmd_required_load_g": max(
            value
            for row in output_rows
            if math.isfinite(value := float(row["visual_vcmd_required_load_g"]))
        )
        if output_rows
        else math.nan,
        "max_visual_actual_load_fd_g": max(
            value for row in output_rows if math.isfinite(value := float(row["visual_actual_load_fd_g"]))
        )
        if output_rows
        else math.nan,
    }
    return output_rows, summary


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export per-frame theoretical truth-PNG required load from strapdown experiment logs."
    )
    parser.add_argument(
        "--summary-csv",
        default="logs/strapdown_accuracy/strapdown_clock0p2_simtime_20260615_061252_summary.csv",
        help="Batch summary containing csv_path entries. Used unless --input is supplied.",
    )
    parser.add_argument("--input", action="append", default=[], help="Input CSV path or glob, repeatable.")
    parser.add_argument("--output-dir", default="logs/strapdown_accuracy/truth_required_load")
    parser.add_argument("--prefix", default="")
    parser.add_argument("--navigation-constant", type=float, default=3.0)
    parser.add_argument("--min-dt-s", type=float, default=1.0e-5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.input:
        cases = _paths_from_inputs(args.input)
    else:
        cases = _paths_from_summary(_resolve(args.summary_csv))
    if not cases:
        raise SystemExit("no input strapdown CSV files found")

    output_dir = _resolve(args.output_dir)
    prefix = args.prefix or f"strapdown_truth_required_load_{time.strftime('%Y%m%d_%H%M%S')}"
    combined_rows: list[dict[str, object]] = []
    summaries: list[dict[str, object]] = []

    for csv_path, summary_row in cases:
        rows, summary = _process_case(
            path=csv_path,
            summary_row=summary_row,
            navigation_constant=float(args.navigation_constant),
            min_dt=float(args.min_dt_s),
        )
        case_output = output_dir / f"{prefix}_{summary['case']}.csv"
        _write_csv(case_output, rows)
        summary["output_csv"] = str(case_output.relative_to(PROJECT_ROOT))
        summaries.append(summary)
        combined_rows.extend(rows)
        print(
            f"{summary['case']}: frames={summary['frames']} "
            f"valid={summary['truth_png_valid_frames']} "
            f"max_n_req={summary['max_theory_n_req_g']:.3f}g "
            f"p95_n_req={summary['p95_theory_n_req_g']:.3f}g"
        )

    combined_path = output_dir / f"{prefix}_combined.csv"
    summary_path = output_dir / f"{prefix}_summary.csv"
    _write_csv(combined_path, combined_rows)
    _write_csv(summary_path, summaries)
    print(f"combined_csv={combined_path}")
    print(f"summary_csv={summary_path}")


if __name__ == "__main__":
    main()
