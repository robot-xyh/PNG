#!/usr/bin/env python3
"""Sweep fixed-VM PNG command authority against recorded Betaflight vision logs.

This tool evaluates command magnitude and saturation only. It does not estimate
interception probability, closed-loop stability, or a safe flight envelope.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import tarfile
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Runner CSV or a .tar.gz archive containing it.")
    parser.add_argument(
        "--archive-member",
        default="",
        help="Exact CSV member path when --input is an archive.",
    )
    parser.add_argument("--output", required=True, help="Output JSON report path.")
    parser.add_argument("--start-s", type=float, default=None)
    parser.add_argument("--end-s", type=float, default=None)
    parser.add_argument("--navigation-constant", type=float, default=3.0)
    parser.add_argument("--gravity-mps2", type=float, default=9.80665)
    parser.add_argument("--min-vertical-specific-force-mps2", type=float, default=0.5)
    parser.add_argument("--vm-values", default="1,3,5,10")
    parser.add_argument("--accel-limits-mps2", default="1,3,5,10")
    parser.add_argument("--attitude-kp-s-inv", default="1,3,5")
    parser.add_argument("--tilt-limits-deg", default="10,20,30")
    parser.add_argument("--rate-limits-deg-s", default="30,60,120")
    parser.add_argument("--roll-rate-sign", type=float, default=1.0)
    parser.add_argument("--pitch-rate-sign", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()
    rows, source = read_input_rows(input_path, archive_member=str(args.archive_member))
    report = analyze_rows(
        rows,
        source=source,
        start_s=args.start_s,
        end_s=args.end_s,
        navigation_constant=float(args.navigation_constant),
        gravity_mps2=float(args.gravity_mps2),
        min_vertical_specific_force_mps2=float(args.min_vertical_specific_force_mps2),
        vm_values=_positive_list(args.vm_values, "vm-values"),
        accel_limits_mps2=_positive_list(args.accel_limits_mps2, "accel-limits-mps2"),
        attitude_kp_s_inv=_positive_list(args.attitude_kp_s_inv, "attitude-kp-s-inv"),
        tilt_limits_deg=_positive_list(args.tilt_limits_deg, "tilt-limits-deg"),
        rate_limits_deg_s=_positive_list(args.rate_limits_deg_s, "rate-limits-deg-s"),
        roll_rate_sign=_axis_sign(args.roll_rate_sign, "roll-rate-sign"),
        pitch_rate_sign=_axis_sign(args.pitch_rate_sign, "pitch-rate-sign"),
    )
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(
        f"samples={report['sample_selection']['eligible_rows']} "
        f"candidates={len(report['candidates'])} output={output}"
    )


def read_input_rows(
    input_path: Path,
    *,
    archive_member: str = "",
) -> tuple[list[dict[str, str]], dict[str, Any]]:
    if not input_path.is_file():
        raise RuntimeError(f"input does not exist: {input_path}")
    source: dict[str, Any] = {
        "path": str(input_path),
        "sha256": _sha256(input_path),
    }
    if tarfile.is_tarfile(input_path):
        with tarfile.open(input_path, "r:*") as archive:
            csv_members = [member.name for member in archive.getmembers() if member.isfile() and member.name.endswith(".csv")]
            member_name = archive_member.strip()
            if not member_name:
                if len(csv_members) != 1:
                    raise RuntimeError(
                        "archive contains multiple CSV files; pass --archive-member exactly: "
                        + ", ".join(csv_members)
                    )
                member_name = csv_members[0]
            if member_name not in csv_members:
                raise RuntimeError(f"CSV archive member not found: {member_name}")
            extracted = archive.extractfile(member_name)
            if extracted is None:
                raise RuntimeError(f"failed to read archive member: {member_name}")
            with io.TextIOWrapper(extracted, encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
        source["archive_member"] = member_name
        return rows, source
    with input_path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream)), source


def analyze_rows(
    rows: Sequence[Mapping[str, Any]],
    *,
    source: Mapping[str, Any] | None = None,
    start_s: float | None = None,
    end_s: float | None = None,
    navigation_constant: float = 3.0,
    gravity_mps2: float = 9.80665,
    min_vertical_specific_force_mps2: float = 0.5,
    vm_values: Sequence[float] = (1.0, 3.0, 5.0, 10.0),
    accel_limits_mps2: Sequence[float] = (1.0, 3.0, 5.0, 10.0),
    attitude_kp_s_inv: Sequence[float] = (1.0, 3.0, 5.0),
    tilt_limits_deg: Sequence[float] = (10.0, 20.0, 30.0),
    rate_limits_deg_s: Sequence[float] = (30.0, 60.0, 120.0),
    roll_rate_sign: float = 1.0,
    pitch_rate_sign: float = 1.0,
) -> dict[str, Any]:
    if not math.isfinite(navigation_constant) or navigation_constant <= 0.0:
        raise ValueError("navigation_constant must be finite and positive")
    if not math.isfinite(gravity_mps2) or gravity_mps2 <= 0.0:
        raise ValueError("gravity_mps2 must be finite and positive")
    if (
        not math.isfinite(min_vertical_specific_force_mps2)
        or min_vertical_specific_force_mps2 <= 0.0
    ):
        raise ValueError("min_vertical_specific_force_mps2 must be finite and positive")
    for name, values in (
        ("vm_values", vm_values),
        ("accel_limits_mps2", accel_limits_mps2),
        ("attitude_kp_s_inv", attitude_kp_s_inv),
        ("tilt_limits_deg", tilt_limits_deg),
        ("rate_limits_deg_s", rate_limits_deg_s),
    ):
        if not values or any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError(f"{name} must contain finite positive values")
    roll_rate_sign = _axis_sign(roll_rate_sign, "roll_rate_sign")
    pitch_rate_sign = _axis_sign(pitch_rate_sign, "pitch_rate_sign")

    samples, rejected = _samples(rows, start_s=start_s, end_s=end_s)
    if not samples:
        raise RuntimeError("no finite guidance-valid rows with synchronized attitude")
    elapsed = np.asarray([sample[0] for sample in samples], dtype=float)
    lambda_i = np.asarray([sample[1] for sample in samples], dtype=float)
    omega_los = np.asarray([sample[2] for sample in samples], dtype=float)
    raw_roll_deg = np.asarray([sample[3] for sample in samples], dtype=float)
    raw_pitch_deg = np.asarray([sample[4] for sample in samples], dtype=float)
    yaw_deg = np.asarray([sample[5] for sample in samples], dtype=float)
    base_accel = np.cross(omega_los, lambda_i)
    current_roll_deg = raw_roll_deg
    current_pitch_deg = -raw_pitch_deg

    candidates: list[dict[str, Any]] = []
    for vm_m_s in vm_values:
        raw_accel = navigation_constant * float(vm_m_s) * base_accel
        raw_accel_norm = np.linalg.norm(raw_accel, axis=1)
        for accel_limit in accel_limits_mps2:
            accel_scale = np.minimum(1.0, float(accel_limit) / np.maximum(raw_accel_norm, 1.0e-12))
            accel = raw_accel * accel_scale[:, None]
            desired_roll_raw, desired_pitch_raw = _desired_tilt_deg(
                accel,
                yaw_deg,
                gravity_mps2=gravity_mps2,
                min_vertical_specific_force_mps2=min_vertical_specific_force_mps2,
            )
            for tilt_limit in tilt_limits_deg:
                desired_roll = np.clip(desired_roll_raw, -tilt_limit, tilt_limit)
                desired_pitch = np.clip(desired_pitch_raw, -tilt_limit, tilt_limit)
                roll_error = desired_roll - current_roll_deg
                pitch_error = desired_pitch - current_pitch_deg
                for kp_s_inv in attitude_kp_s_inv:
                    recorded_roll_unbounded = roll_rate_sign * kp_s_inv * roll_error
                    recorded_pitch_unbounded = pitch_rate_sign * kp_s_inv * pitch_error
                    guidance_roll_unbounded = roll_rate_sign * kp_s_inv * desired_roll
                    guidance_pitch_unbounded = pitch_rate_sign * kp_s_inv * desired_pitch
                    for rate_limit in rate_limits_deg_s:
                        recorded_roll = np.clip(recorded_roll_unbounded, -rate_limit, rate_limit)
                        recorded_pitch = np.clip(recorded_pitch_unbounded, -rate_limit, rate_limit)
                        guidance_roll = np.clip(guidance_roll_unbounded, -rate_limit, rate_limit)
                        guidance_pitch = np.clip(guidance_pitch_unbounded, -rate_limit, rate_limit)
                        candidates.append(
                            {
                                "parameters": {
                                    "navigation_constant": float(navigation_constant),
                                    "fixed_vm_m_s": float(vm_m_s),
                                    "max_guidance_accel_mps2": float(accel_limit),
                                    "attitude_kp_s_inv": float(kp_s_inv),
                                    "max_tilt_deg": float(tilt_limit),
                                    "max_rate_deg_s": float(rate_limit),
                                },
                                "acceleration": {
                                    "raw_norm_mps2": _distribution(raw_accel_norm),
                                    "limited_norm_mps2": _distribution(np.linalg.norm(accel, axis=1)),
                                    "saturation_fraction": _fraction(raw_accel_norm > accel_limit + 1.0e-9),
                                },
                                "target_attitude": {
                                    "roll_abs_deg": _distribution(np.abs(desired_roll)),
                                    "pitch_abs_deg": _distribution(np.abs(desired_pitch)),
                                    "saturation_fraction": _fraction(
                                        (np.abs(desired_roll_raw) > tilt_limit + 1.0e-9)
                                        | (np.abs(desired_pitch_raw) > tilt_limit + 1.0e-9)
                                    ),
                                },
                                "guidance_only_rate": _rate_metrics(
                                    guidance_roll,
                                    guidance_pitch,
                                    guidance_roll_unbounded,
                                    guidance_pitch_unbounded,
                                    float(rate_limit),
                                ),
                                "recorded_attitude_rate": _rate_metrics(
                                    recorded_roll,
                                    recorded_pitch,
                                    recorded_roll_unbounded,
                                    recorded_pitch_unbounded,
                                    float(rate_limit),
                                ),
                            }
                        )

    return {
        "schema_version": 1,
        "analysis_type": "png_command_authority_not_interception_probability",
        "limitations": [
            "Recorded LOS and attitude are replayed open-loop; vehicle translation and target response are not simulated.",
            "Results do not establish closed-loop stability, interception probability, hover throttle, or a safe flight envelope.",
            "guidance_only_rate assumes zero roll/pitch; recorded_attitude_rate uses the logged fixed-airframe attitude.",
            "Axis signs are explicit inputs and still require physical sign calibration before propeller flight.",
        ],
        "source": dict(source or {}),
        "sample_selection": {
            "input_rows": len(rows),
            "eligible_rows": len(samples),
            "rejected_rows": rejected,
            "start_s": None if start_s is None else float(start_s),
            "end_s": None if end_s is None else float(end_s),
            "first_elapsed_s": float(elapsed[0]),
            "last_elapsed_s": float(elapsed[-1]),
        },
        "axis_conventions": {
            "guidance_frame": "inertial_ned",
            "attitude_frame": "body_frd_to_inertial_ned",
            "raw_betaflight_pitch_negated_for_frd": True,
            "roll_rate_sign": roll_rate_sign,
            "pitch_rate_sign": pitch_rate_sign,
            "gravity_mps2": float(gravity_mps2),
            "min_vertical_specific_force_mps2": float(
                min_vertical_specific_force_mps2
            ),
        },
        "sweep_axes": {
            "navigation_constant": float(navigation_constant),
            "fixed_vm_m_s": [float(value) for value in vm_values],
            "max_guidance_accel_mps2": [float(value) for value in accel_limits_mps2],
            "attitude_kp_s_inv": [float(value) for value in attitude_kp_s_inv],
            "max_tilt_deg": [float(value) for value in tilt_limits_deg],
            "max_rate_deg_s": [float(value) for value in rate_limits_deg_s],
        },
        "candidates": candidates,
    }


def _samples(
    rows: Sequence[Mapping[str, Any]],
    *,
    start_s: float | None,
    end_s: float | None,
) -> tuple[list[tuple[float, np.ndarray, np.ndarray, float, float, float]], int]:
    selected: list[tuple[float, np.ndarray, np.ndarray, float, float, float]] = []
    rejected = 0
    for row in rows:
        elapsed = _number(row.get("elapsed_s"))
        if elapsed is None or (start_s is not None and elapsed < start_s) or (end_s is not None and elapsed > end_s):
            continue
        if _integer(row.get("guidance_valid")) != 1:
            rejected += 1
            continue
        values = [
            _number(row.get(name))
            for name in (
                "lambda_I_x",
                "lambda_I_y",
                "lambda_I_z",
                "omega_los_x",
                "omega_los_y",
                "omega_los_z",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
            )
        ]
        if any(value is None for value in values):
            rejected += 1
            continue
        selected.append(
            (
                elapsed,
                np.asarray(values[:3], dtype=float),
                np.asarray(values[3:6], dtype=float),
                float(values[6]),
                float(values[7]),
                float(values[8]),
            )
        )
    return selected, rejected


def _desired_tilt_deg(
    accel_ned: np.ndarray,
    yaw_deg: np.ndarray,
    *,
    gravity_mps2: float,
    min_vertical_specific_force_mps2: float,
) -> tuple[np.ndarray, np.ndarray]:
    yaw = np.deg2rad(yaw_deg)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    accel_forward = cos_yaw * accel_ned[:, 0] + sin_yaw * accel_ned[:, 1]
    accel_right = -sin_yaw * accel_ned[:, 0] + cos_yaw * accel_ned[:, 1]
    vertical_force = np.maximum(
        min_vertical_specific_force_mps2,
        gravity_mps2 - accel_ned[:, 2],
    )
    roll = np.rad2deg(np.arctan2(accel_right, vertical_force))
    pitch = np.rad2deg(np.arctan2(-accel_forward, vertical_force))
    return roll, pitch


def _rate_metrics(
    roll_rate: np.ndarray,
    pitch_rate: np.ndarray,
    unbounded_roll_rate: np.ndarray,
    unbounded_pitch_rate: np.ndarray,
    rate_limit: float,
) -> dict[str, Any]:
    max_axis = np.maximum(np.abs(roll_rate), np.abs(pitch_rate))
    return {
        "roll_abs_deg_s": _distribution(np.abs(roll_rate)),
        "pitch_abs_deg_s": _distribution(np.abs(pitch_rate)),
        "max_axis_abs_deg_s": _distribution(max_axis),
        "saturation_fraction": _fraction(
            (np.abs(unbounded_roll_rate) > rate_limit + 1.0e-9)
            | (np.abs(unbounded_pitch_rate) > rate_limit + 1.0e-9)
        ),
        "over_1_deg_s_fraction": _fraction(max_axis >= 1.0),
        "over_5_deg_s_fraction": _fraction(max_axis >= 5.0),
        "over_10_deg_s_fraction": _fraction(max_axis >= 10.0),
        "over_30_deg_s_fraction": _fraction(max_axis >= 30.0),
    }


def _distribution(values: np.ndarray) -> dict[str, float]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return {"p50": 0.0, "p95": 0.0, "max": 0.0}
    return {
        "p50": _rounded(np.percentile(finite, 50.0)),
        "p95": _rounded(np.percentile(finite, 95.0)),
        "max": _rounded(np.max(finite)),
    }


def _fraction(mask: np.ndarray) -> float:
    return _rounded(float(np.mean(np.asarray(mask, dtype=bool))))


def _positive_list(raw: str, name: str) -> tuple[float, ...]:
    try:
        values = tuple(float(item.strip()) for item in str(raw).split(",") if item.strip())
    except ValueError as exc:
        raise RuntimeError(f"--{name} must be a comma-separated numeric list") from exc
    if not values or any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise RuntimeError(f"--{name} must contain finite positive values")
    return values


def _axis_sign(value: float, name: str) -> float:
    number = float(value)
    if number not in (-1.0, 1.0):
        raise ValueError(f"{name} must be -1 or +1")
    return number


def _number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _integer(value: Any) -> int | None:
    number = _number(value)
    if number is None or not float(number).is_integer():
        return None
    return int(number)


def _rounded(value: float) -> float:
    return round(float(value), 9)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


if __name__ == "__main__":
    main()
