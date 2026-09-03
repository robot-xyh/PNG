#!/usr/bin/env python3
"""Audit virtual-box VM output and compute theoretical load-factor profiles."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


SET_RAW_RC_COUNT_FIELDS = (
    "msp_set_raw_rc_attempt_count",
    "msp_set_raw_rc_success_count",
    "msp_set_raw_rc_write_attempt_count",
    "msp_set_raw_rc_write_success_count",
    "msp_set_raw_rc_write_error_count",
    "msp_set_raw_rc_ack_count",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True, help="Runner CSV from the virtual-box test.")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--load-levels-mps2", default="1,3,5,7")
    parser.add_argument("--mass-kg", type=float, default=2.412)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    csv_path = args.csv.expanduser().resolve()
    report = analyze_log(
        csv_path,
        load_levels_m_s2=_positive_values(args.load_levels_mps2),
        mass_kg=float(args.mass_kg),
    )
    output = (
        args.output.expanduser().resolve()
        if args.output is not None
        else csv_path.with_name(f"{csv_path.stem}_virtual_load.json")
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"passed={int(report['passed'])} output={output}")
    print(
        "valid_rows="
        f"{report['samples']['valid_intercept_rows']} "
        f"max_accel_m_s2={report['actual_command']['acceleration_norm_m_s2']['max']:.3f} "
        f"max_load_g={report['actual_command']['load_factor_g']['max']:.3f}"
    )


def analyze_log(
    csv_path: Path,
    *,
    load_levels_m_s2: Sequence[float] = (1.0, 3.0, 5.0, 7.0),
    mass_kg: float = 2.412,
) -> dict[str, Any]:
    csv_path = csv_path.expanduser().resolve()
    if not csv_path.is_file():
        raise RuntimeError(f"CSV does not exist: {csv_path}")
    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise ValueError("mass_kg must be finite and positive")
    levels = tuple(float(value) for value in load_levels_m_s2)
    if not levels or any(not math.isfinite(value) or value <= 0.0 for value in levels):
        raise ValueError("load_levels_m_s2 must contain finite positive values")
    if tuple(sorted(set(levels))) != levels:
        raise ValueError("load_levels_m_s2 must be strictly increasing")

    with csv_path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    meta_path = csv_path.with_name(f"{csv_path.stem}_meta.json")
    meta = _read_json(meta_path)
    config = dict(meta.get("config", {}))
    guidance = dict(config.get("guidance", {}))
    controller = dict(guidance.get("velocity_establishing_png", {}))
    command = dict(config.get("guidance_command", {}))
    accel_mapping = dict(command.get("accel_tilt_rate", {}))
    rc_mapping = dict(config.get("rc_mapping", {}))
    camera = dict(config.get("camera", {}))
    runtime_policy = dict(config.get("runtime_policy", {}))

    gravity = float(controller.get("gravity_m_s2", accel_mapping.get("gravity_mps2", 9.80665)))
    total_limit = float(controller.get("total_accel_limit_m_s2", float("nan")))
    if not math.isfinite(gravity) or gravity <= 0.0:
        raise RuntimeError("config has no finite positive gravity")
    if not math.isfinite(total_limit) or total_limit <= 0.0:
        raise RuntimeError("config has no finite positive total acceleration limit")
    if levels[-1] > total_limit + 1.0e-9:
        raise RuntimeError(
            f"requested load level {levels[-1]:g} exceeds configured total limit {total_limit:g}"
        )

    violations: list[dict[str, Any]] = []
    if not rows:
        violations.append(_violation("empty_log", 1))
    if not meta:
        violations.append(_violation("meta_missing", 1, detail=str(meta_path)))

    invalid_state = [row for row in rows if row.get("safety_state") not in {"", "LOG_ONLY"}]
    invalid_publish = [row for row in rows if row.get("msp_publish_mode") not in {"", "disabled"}]
    control_requested = [row for row in rows if _integer(row.get("control_requested")) not in {None, 0}]
    allow_control = [row for row in rows if _integer(row.get("allow_control")) not in {None, 0}]
    for code, selected in (
        ("not_log_only", invalid_state),
        ("publish_not_disabled", invalid_publish),
        ("control_requested", control_requested),
        ("allow_control", allow_control),
    ):
        if selected:
            violations.append(_violation(code, len(selected), rows=selected))

    set_raw_rc_maxima = {
        field: max((_number(row.get(field)) or 0.0 for row in rows), default=0.0)
        for field in SET_RAW_RC_COUNT_FIELDS
    }
    nonzero_rc = {key: value for key, value in set_raw_rc_maxima.items() if value > 0.0}
    if nonzero_rc:
        violations.append(_violation("set_raw_rc_nonzero", len(nonzero_rc), detail=nonzero_rc))

    config_contract = {
        "guidance_law": guidance.get("law"),
        "velocity_source": guidance.get("velocity_source"),
        "mapping_type": command.get("mapping_type"),
        "total_accel_limit_m_s2": total_limit,
        "roll_rate_sign": _number(accel_mapping.get("roll_rate_sign")),
        "pitch_rate_sign": _number(accel_mapping.get("pitch_rate_sign")),
        "runtime_allowed_control_modes": runtime_policy.get("allowed_control_modes"),
        "runtime_msp_set_raw_rc_permitted": runtime_policy.get("msp_set_raw_rc_permitted"),
        "control_authorization_enabled": dict(config.get("control_authorization", {})).get("enabled"),
    }
    expected_contract = (
        guidance.get("law") == "velocity_establishing_png"
        and guidance.get("velocity_source") == "msp_kinematics"
        and command.get("mapping_type") == "accel_tilt_rate"
        and _number(accel_mapping.get("roll_rate_sign")) == 1.0
        and _number(accel_mapping.get("pitch_rate_sign")) == -1.0
        and runtime_policy.get("allowed_control_modes") == ["log_only"]
        and runtime_policy.get("msp_set_raw_rc_permitted") is False
        and dict(config.get("control_authorization", {})).get("enabled") is False
    )
    if not expected_contract:
        violations.append(_violation("config_contract_mismatch", 1, detail=config_contract))

    vectors: list[tuple[float, float, float]] = []
    valid_rows: list[dict[str, str]] = []
    for row in rows:
        if _integer(row.get("intercept_valid")) != 1:
            continue
        vector = tuple(
            _number(row.get(field))
            for field in (
                "intercept_total_accel_n",
                "intercept_total_accel_e",
                "intercept_total_accel_d",
            )
        )
        if all(value is not None and math.isfinite(value) for value in vector):
            vectors.append(tuple(float(value) for value in vector if value is not None))
            valid_rows.append(row)
    if len(vectors) < 10:
        violations.append(_violation("insufficient_valid_intercept_rows", len(vectors)))

    vector_array = np.asarray(vectors, dtype=float).reshape((-1, 3))
    accel_norm = np.linalg.norm(vector_array, axis=1) if len(vector_array) else np.asarray([], dtype=float)
    if len(accel_norm) and float(np.max(accel_norm)) > total_limit + 1.0e-5:
        violations.append(
            _violation(
                "total_acceleration_limit_exceeded",
                int(np.sum(accel_norm > total_limit + 1.0e-5)),
                limit=total_limit,
            )
        )
    if len(accel_norm) and float(np.max(accel_norm)) < 0.90 * total_limit:
        violations.append(
            _violation(
                "high_load_level_not_reached",
                1,
                detail={"observed_max": float(np.max(accel_norm)), "required_min": 0.90 * total_limit},
            )
        )

    gravity_ned = np.array([0.0, 0.0, gravity], dtype=float)
    actual_load = (
        np.linalg.norm(vector_array - gravity_ned, axis=1) / gravity
        if len(vector_array)
        else np.asarray([], dtype=float)
    )
    actual_force = actual_load * mass_kg * gravity

    roll_rate_limit = float(rc_mapping.get("roll_command_limit_deg_s", float("nan")))
    pitch_rate_limit = float(rc_mapping.get("pitch_command_limit_deg_s", float("nan")))
    roll_rate = _finite_column(valid_rows, "pre_shape_sp_roll_rate_deg_s")
    pitch_rate = _finite_column(valid_rows, "pre_shape_sp_pitch_rate_deg_s")
    if len(roll_rate) and math.isfinite(roll_rate_limit) and np.max(np.abs(roll_rate)) > roll_rate_limit + 1.0e-5:
        violations.append(_violation("roll_rate_limit_exceeded", int(np.sum(np.abs(roll_rate) > roll_rate_limit + 1.0e-5)), limit=roll_rate_limit))
    if len(pitch_rate) and math.isfinite(pitch_rate_limit) and np.max(np.abs(pitch_rate)) > pitch_rate_limit + 1.0e-5:
        violations.append(_violation("pitch_rate_limit_exceeded", int(np.sum(np.abs(pitch_rate) > pitch_rate_limit + 1.0e-5)), limit=pitch_rate_limit))

    cx = float(camera.get("cx", 0.5 * float(camera.get("width", 640))))
    cy = float(camera.get("cy", 0.5 * float(camera.get("height", 512))))
    x_offsets, desired_roll = _bbox_command_pairs(
        valid_rows,
        center_field="bbox_center_x",
        lower_field="bbox_x1",
        upper_field="bbox_x2",
        command_field="command_desired_roll_angle_deg",
        offset=cx,
    )
    y_offsets, desired_pitch = _bbox_command_pairs(
        valid_rows,
        center_field="bbox_center_y",
        lower_field="bbox_y1",
        upper_field="bbox_y2",
        command_field="command_desired_pitch_angle_deg",
        offset=cy,
    )
    horizontal_correlation = _correlation(x_offsets, desired_roll, minimum_abs_offset=30.0)
    vertical_correlation = _correlation(y_offsets, desired_pitch, minimum_abs_offset=25.0)
    if horizontal_correlation is None or horizontal_correlation < 0.6:
        violations.append(_violation("horizontal_command_sign", 1, detail=horizontal_correlation))
    if vertical_correlation is None or vertical_correlation > -0.6:
        violations.append(_violation("vertical_command_sign", 1, detail=vertical_correlation))

    profiles = []
    for level in levels:
        staged = _clip_vectors(vector_array, level)
        staged_norm = np.linalg.norm(staged, axis=1) if len(staged) else np.asarray([], dtype=float)
        staged_load = (
            np.linalg.norm(staged - gravity_ned, axis=1) / gravity
            if len(staged)
            else np.asarray([], dtype=float)
        )
        profiles.append(
            {
                "acceleration_limit_m_s2": level,
                "acceleration_norm_m_s2": _distribution(staged_norm),
                "load_factor_g": _distribution(staged_load),
                "required_thrust_n": _distribution(staged_load * mass_kg * gravity),
                "direction_independent_load_envelope_g": {
                    "minimum": abs(gravity - level) / gravity,
                    "maximum": (gravity + level) / gravity,
                },
            }
        )

    return {
        "schema_version": 1,
        "passed": not violations,
        "scope": "virtual_bbox_log_only_theoretical_load",
        "limitations": [
            "The report computes candidate specific force from VM acceleration; it does not measure airframe load.",
            "Virtual boxes bypass YOLO and ByteTrack and do not prove detection performance.",
            "Passing does not authorize propeller-on MSP override or interception flight.",
        ],
        "source": {
            "csv": str(csv_path),
            "csv_sha256": _sha256(csv_path),
            "meta": str(meta_path),
            "meta_sha256": _sha256(meta_path) if meta_path.is_file() else None,
        },
        "config_contract": config_contract,
        "calculation": {
            "frame": "inertial_ned",
            "formula": "load_factor = norm(a_cmd_ned - [0,0,g]) / g",
            "gravity_m_s2": gravity,
            "mass_kg": mass_kg,
        },
        "samples": {
            "total_rows": len(rows),
            "valid_intercept_rows": len(vectors),
        },
        "safety": {
            "set_raw_rc_count_maxima": set_raw_rc_maxima,
            "invalid_state_rows": len(invalid_state),
            "invalid_publish_rows": len(invalid_publish),
            "control_requested_rows": len(control_requested),
            "allow_control_rows": len(allow_control),
        },
        "actual_command": {
            "acceleration_norm_m_s2": _distribution(accel_norm),
            "load_factor_g": _distribution(actual_load),
            "required_thrust_n": _distribution(actual_force),
            "roll_rate_abs_deg_s": _distribution(np.abs(roll_rate)),
            "pitch_rate_abs_deg_s": _distribution(np.abs(pitch_rate)),
            "bbox_to_command_correlation": {
                "horizontal_offset_vs_desired_roll": horizontal_correlation,
                "vertical_offset_vs_desired_pitch": vertical_correlation,
            },
        },
        "staged_load_profiles": profiles,
        "violations": violations,
    }


def _clip_vectors(vectors: np.ndarray, limit: float) -> np.ndarray:
    if not len(vectors):
        return np.empty((0, 3), dtype=float)
    norms = np.linalg.norm(vectors, axis=1)
    scale = np.minimum(1.0, float(limit) / np.maximum(norms, 1.0e-12))
    return vectors * scale[:, None]


def _bbox_command_pairs(
    rows: Sequence[Mapping[str, Any]],
    *,
    center_field: str,
    lower_field: str,
    upper_field: str,
    command_field: str,
    offset: float,
) -> tuple[np.ndarray, np.ndarray]:
    pairs = []
    for row in rows:
        center = _number(row.get(center_field))
        if center is None:
            lower = _number(row.get(lower_field))
            upper = _number(row.get(upper_field))
            if lower is not None and upper is not None:
                center = 0.5 * (lower + upper)
        command = _number(row.get(command_field))
        if center is not None and command is not None:
            pairs.append((center - offset, command))
    if not pairs:
        return np.asarray([], dtype=float), np.asarray([], dtype=float)
    values = np.asarray(pairs, dtype=float)
    return values[:, 0], values[:, 1]


def _correlation(first: np.ndarray, second: np.ndarray, *, minimum_abs_offset: float) -> float | None:
    if len(first) != len(second):
        raise ValueError("correlation columns must have equal length")
    selected = np.abs(first) >= minimum_abs_offset
    if int(np.sum(selected)) < 10:
        return None
    x = first[selected]
    y = second[selected]
    if float(np.std(x)) <= 1.0e-12 or float(np.std(y)) <= 1.0e-12:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def _finite_column(rows: Sequence[Mapping[str, Any]], field: str) -> np.ndarray:
    values = [_number(row.get(field)) for row in rows]
    return np.asarray([value for value in values if value is not None], dtype=float)


def _distribution(values: np.ndarray) -> dict[str, float | int | None]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if not len(finite):
        return {"count": 0, "min": None, "mean": None, "p50": None, "p95": None, "p99": None, "max": None}
    return {
        "count": int(len(finite)),
        "min": float(np.min(finite)),
        "mean": float(np.mean(finite)),
        "p50": float(np.percentile(finite, 50)),
        "p95": float(np.percentile(finite, 95)),
        "p99": float(np.percentile(finite, 99)),
        "max": float(np.max(finite)),
    }


def _violation(
    code: str,
    count: int,
    *,
    rows: Sequence[Mapping[str, Any]] | None = None,
    limit: float | None = None,
    detail: Any = None,
) -> dict[str, Any]:
    first_elapsed_s = None
    if rows:
        first_elapsed_s = _number(rows[0].get("elapsed_s"))
    result = {"code": code, "count": int(count), "first_elapsed_s": first_elapsed_s}
    if limit is not None:
        result["limit"] = limit
    if detail is not None:
        result["detail"] = detail
    return result


def _positive_values(text: str) -> tuple[float, ...]:
    try:
        values = tuple(float(item.strip()) for item in text.split(",") if item.strip())
    except ValueError as exc:
        raise ValueError("load levels must be comma-separated numbers") from exc
    if not values or any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise ValueError("load levels must contain finite positive values")
    return values


def _number(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _integer(value: Any) -> int | None:
    number = _number(value)
    return None if number is None else int(number)


def _read_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


if __name__ == "__main__":
    main()
