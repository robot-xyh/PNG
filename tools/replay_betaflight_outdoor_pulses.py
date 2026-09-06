#!/usr/bin/env python3
"""Replay recorded Betaflight algorithm windows through the current controller."""

from __future__ import annotations

import argparse
import csv
from dataclasses import fields
from datetime import datetime
import glob
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Iterable

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision_guidance.betaflight_intercept_controller import (  # noqa: E402
    FovPriorityConfig,
    InterceptPhase,
    VelocityEstablishingPngConfig,
    VelocityEstablishingPngController,
    VelocityEstablishingPngInput,
)
from vision_guidance.betaflight_msp import attitude_degrees_to_R_IB  # noqa: E402


DEFAULT_LOG_GLOB = "logs/flight_active_supervised/FLIGHT_ACTIVE*.csv"
DEFAULT_CONFIG = "config/betaflight.rk3588.velocity_png.flight_supervised.json"
DEFAULT_IMPACT_METRICS = "logs/analysis/LOG00106_target_joint/metrics.json"
DEFAULT_IMPACT_RUN_TOKEN = "20260904_183721_20260904_183722"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--logs-glob", default=DEFAULT_LOG_GLOB)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--impact-metrics", default=DEFAULT_IMPACT_METRICS)
    parser.add_argument("--impact-run-token", default=DEFAULT_IMPACT_RUN_TOKEN)
    parser.add_argument("--expected-pulses", type=int, default=10)
    parser.add_argument("--max-saturation-fraction", type=float, default=0.40)
    parser.add_argument("--minimum-abort-lead-s", type=float, default=0.75)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    paths = [Path(value).resolve() for value in sorted(glob.glob(args.logs_glob))]
    result = replay_logs(
        paths,
        config_path=Path(args.config).expanduser().resolve(),
        impact_metrics_path=Path(args.impact_metrics).expanduser().resolve(),
        impact_run_token=str(args.impact_run_token),
        expected_pulse_count=int(args.expected_pulses),
        max_saturation_fraction=float(args.max_saturation_fraction),
        minimum_abort_lead_s=float(args.minimum_abort_lead_s),
    )
    output_path = Path(args.output).expanduser().resolve()
    _write_json_atomic(output_path, result)
    print(
        f"pulses={result['aggregate']['pulse_count']} "
        f"speed_saturated={result['aggregate']['replay_speed_saturated_fraction']:.6f} "
        f"total_saturated={result['aggregate']['replay_total_saturated_fraction']:.6f} "
        f"abort_lead_s={result['log00106_noncollision_abort']['lead_s']:.6f} "
        f"passed={int(result['release_passed'])}"
    )
    print(output_path)


def replay_logs(
    log_paths: Iterable[Path],
    *,
    config_path: Path,
    impact_metrics_path: Path,
    impact_run_token: str,
    expected_pulse_count: int,
    max_saturation_fraction: float,
    minimum_abort_lead_s: float,
) -> dict[str, Any]:
    if expected_pulse_count <= 0:
        raise ValueError("expected_pulse_count must be positive")
    if not 0.0 <= max_saturation_fraction <= 1.0:
        raise ValueError("max_saturation_fraction must be in [0, 1]")
    if not math.isfinite(minimum_abort_lead_s) or minimum_abort_lead_s < 0.0:
        raise ValueError("minimum_abort_lead_s must be finite and non-negative")

    config = _read_json(config_path)
    controller_config = _controller_config(config)
    pulse_inputs: list[tuple[Path, int, list[dict[str, str]]]] = []
    input_files: dict[str, dict[str, Any]] = {}
    for path in log_paths:
        if path.name.endswith(("_essential_zh.csv", "_png_takeover_zh.csv")):
            continue
        rows = _read_csv(path)
        segments = _algorithm_segments(rows)
        if not segments:
            continue
        input_files[_display_path(path)] = _file_metadata(path)
        pulse_inputs.extend(
            (path, segment_index, segment)
            for segment_index, segment in enumerate(segments, start=1)
        )
    if len(pulse_inputs) != expected_pulse_count:
        raise RuntimeError(
            f"expected {expected_pulse_count} algorithm pulses, found {len(pulse_inputs)}"
        )

    pulses = [
        _replay_pulse(path, segment_index, rows, controller_config)
        for path, segment_index, rows in pulse_inputs
    ]
    replay_valid_rows = sum(int(pulse["replay_valid_rows"]) for pulse in pulses)
    if replay_valid_rows <= 0:
        raise RuntimeError("replay produced no valid controller rows")
    replay_speed_saturated = sum(
        int(pulse["replay_speed_saturated_rows"]) for pulse in pulses
    )
    replay_total_saturated = sum(
        int(pulse["replay_total_saturated_rows"]) for pulse in pulses
    )
    original_rows = sum(int(pulse["recorded_algorithm_rows"]) for pulse in pulses)
    original_speed_saturated = sum(
        int(pulse["recorded_speed_saturated_rows"]) for pulse in pulses
    )
    original_total_saturated = sum(
        int(pulse["recorded_total_saturated_rows"]) for pulse in pulses
    )

    impact_metrics = _read_json(impact_metrics_path)
    input_files[_display_path(impact_metrics_path)] = _file_metadata(impact_metrics_path)
    impact_result = _impact_abort_result(
        pulses,
        impact_run_token=impact_run_token,
        impact_metrics=impact_metrics,
        minimum_abort_lead_s=minimum_abort_lead_s,
        input_files=input_files,
    )
    speed_fraction = replay_speed_saturated / replay_valid_rows
    total_fraction = replay_total_saturated / replay_valid_rows
    pulse_coverage_passed = all(int(pulse["replay_valid_rows"]) > 0 for pulse in pulses)
    acceptance = {
        "expected_pulse_count": expected_pulse_count,
        "maximum_speed_saturation_fraction": max_saturation_fraction,
        "maximum_total_saturation_fraction": max_saturation_fraction,
        "minimum_log00106_abort_lead_s": minimum_abort_lead_s,
        "pulse_count_passed": len(pulses) == expected_pulse_count,
        "pulse_coverage_passed": pulse_coverage_passed,
        "speed_saturation_passed": speed_fraction <= max_saturation_fraction,
        "total_saturation_passed": total_fraction <= max_saturation_fraction,
        "log00106_abort_lead_passed": impact_result["passed"],
    }
    return {
        "schema_version": 1,
        "purpose": "Replay the ten 2026-09-04 active windows through the current non-collision controller",
        "limitations": [
            "This is deterministic log replay, not a closed-loop flight or hit-rate measurement.",
            "Recorded LOS, attitude, and own-velocity inputs are reused; the replay does not regenerate detector outputs.",
            "Rows after a replayed ABORT are retained as invalid latched outputs and are excluded from saturation denominators.",
        ],
        "runtime_binding": _file_metadata(config_path),
        "controller_config": _controller_config_dict(controller_config),
        "inputs": input_files,
        "pulses": pulses,
        "aggregate": {
            "pulse_count": len(pulses),
            "recorded_algorithm_rows": original_rows,
            "recorded_speed_saturated_fraction": (
                original_speed_saturated / original_rows
            ),
            "recorded_total_saturated_fraction": (
                original_total_saturated / original_rows
            ),
            "replay_valid_rows": replay_valid_rows,
            "replay_speed_saturated_fraction": speed_fraction,
            "replay_total_saturated_fraction": total_fraction,
        },
        "log00106_noncollision_abort": impact_result,
        "acceptance": acceptance,
        "release_passed": all(
            bool(value) for key, value in acceptance.items() if key.endswith("_passed")
        ),
    }


def _replay_pulse(
    path: Path,
    segment_index: int,
    rows: list[dict[str, str]],
    config: VelocityEstablishingPngConfig,
) -> dict[str, Any]:
    controller = VelocityEstablishingPngController(config)
    outputs = [controller.update(_controller_input(row)) for row in rows]
    valid_outputs = [output for output in outputs if output.valid]
    abort = next(
        (output for output in outputs if output.phase == InterceptPhase.ABORT),
        None,
    )
    accelerations = [
        float(np.linalg.norm(output.acceleration_ned_m_s2)) for output in valid_outputs
    ]
    return {
        "source_csv": _display_path(path),
        "segment_index": segment_index,
        "start_elapsed_s": _required_float(rows[0], "elapsed_s"),
        "end_elapsed_s": _required_float(rows[-1], "elapsed_s"),
        "recorded_algorithm_rows": len(rows),
        "recorded_speed_saturated_rows": sum(
            _bool(row.get("intercept_speed_saturated")) for row in rows
        ),
        "recorded_total_saturated_rows": sum(
            _bool(row.get("intercept_total_saturated")) for row in rows
        ),
        "replay_valid_rows": len(valid_outputs),
        "replay_speed_saturated_rows": sum(
            output.speed_saturated for output in valid_outputs
        ),
        "replay_total_saturated_rows": sum(
            output.total_saturated for output in valid_outputs
        ),
        "replay_max_acceleration_m_s2": max(accelerations, default=0.0),
        "first_abort_elapsed_s": None if abort is None else abort.timestamp_s,
        "first_abort_reason": None if abort is None else abort.reason,
        "final_phase": outputs[-1].phase.value,
        "final_reason": outputs[-1].reason,
    }


def _controller_input(row: dict[str, str]) -> VelocityEstablishingPngInput:
    timestamp_s = _required_float(row, "elapsed_s")
    detection_age_s = _float(row.get("intercept_detection_age_s"))
    detection_update_age_s = _float(
        row.get("intercept_detection_update_age_s")
    )
    velocity_age_s = _float(row.get("intercept_velocity_age_s"))
    los_values = _vector(row, "lambda_I_x", "lambda_I_y", "lambda_I_z")
    los_dot_values = _vector(
        row, "lambda_dot_I_x", "lambda_dot_I_y", "lambda_dot_I_z"
    )
    velocity = _vector(
        row,
        "kinematics_velocity_filtered_n_m_s",
        "kinematics_velocity_filtered_e_m_s",
        "kinematics_velocity_filtered_d_m_s",
    )
    tracking_valid = bool(
        _bool(row.get("los_valid"))
        and los_values is not None
        and los_dot_values is not None
    )
    velocity_valid = bool(_bool(row.get("kinematics_valid")) and velocity is not None)
    attitude_values = tuple(
        _float(row.get(name)) for name in ("roll_deg", "pitch_deg", "yaw_deg")
    )
    attitude_valid = bool(
        _bool(row.get("attitude_synced"))
        and all(value is not None for value in attitude_values)
    )
    attitude = (
        attitude_degrees_to_R_IB(*attitude_values)
        if attitude_valid
        else None
    )
    return VelocityEstablishingPngInput(
        timestamp_s=timestamp_s,
        los_timestamp_s=(
            None
            if detection_age_s is None
            else timestamp_s - max(0.0, detection_age_s)
        ),
        los_update_timestamp_s=(
            None
            if detection_update_age_s is None
            else timestamp_s - max(0.0, detection_update_age_s)
        ),
        lambda_ned=los_values,
        lambda_dot_ned_s=los_dot_values,
        tracking_valid=tracking_valid,
        bbox_area_ratio=_float(row.get("bbox_area_ratio")),
        attitude_R_IB=attitude,
        attitude_valid=attitude_valid,
        velocity_timestamp_s=(
            None
            if velocity_age_s is None
            else timestamp_s - max(0.0, velocity_age_s)
        ),
        velocity_ned_m_s=velocity,
        velocity_valid=velocity_valid,
        tracking_reason=str(row.get("los_reject_reason", "")).strip() or None,
        ttc_valid=_bool(row.get("ttc_valid")),
        ttc_s=_float(row.get("ttc_s")),
        track_id=_int(row.get("track_id")),
    )


def _impact_abort_result(
    pulses: list[dict[str, Any]],
    *,
    impact_run_token: str,
    impact_metrics: dict[str, Any],
    minimum_abort_lead_s: float,
    input_files: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    matches = [pulse for pulse in pulses if impact_run_token in pulse["source_csv"]]
    if not matches:
        raise RuntimeError(
            "impact run token did not select an algorithm pulse"
        )
    pulse = matches[-1]
    abort_elapsed_s = pulse["first_abort_elapsed_s"]
    if abort_elapsed_s is None:
        return {
            "source_csv": pulse["source_csv"],
            "abort_elapsed_s": None,
            "abort_reason": None,
            "lead_s": -math.inf,
            "minimum_lead_s": minimum_abort_lead_s,
            "passed": False,
        }
    csv_path = Path(pulse["source_csv"])
    if not csv_path.is_absolute():
        csv_path = ROOT / csv_path
    meta_path = csv_path.with_name(f"{csv_path.stem}_meta.json")
    meta = _read_json(meta_path)
    input_files[_display_path(meta_path)] = _file_metadata(meta_path)
    created_unix_s = float(meta["created_unix_s"])
    impact_iso = str(impact_metrics["events"]["target_main_impact"]["utc"])
    impact_unix_s = datetime.fromisoformat(impact_iso.replace("Z", "+00:00")).timestamp()
    abort_unix_s = created_unix_s + float(abort_elapsed_s)
    lead_s = impact_unix_s - abort_unix_s
    return {
        "source_csv": pulse["source_csv"],
        "source_meta": _display_path(meta_path),
        "impact_utc": impact_iso,
        "abort_elapsed_s": abort_elapsed_s,
        "abort_reason": pulse["first_abort_reason"],
        "abort_unix_s": abort_unix_s,
        "lead_s": lead_s,
        "minimum_lead_s": minimum_abort_lead_s,
        "passed": lead_s >= minimum_abort_lead_s,
    }


def _controller_config(config: dict[str, Any]) -> VelocityEstablishingPngConfig:
    guidance = dict(config.get("guidance", {}))
    if guidance.get("law") != "velocity_establishing_png":
        raise RuntimeError("runtime config does not use velocity_establishing_png")
    values = dict(guidance.get("velocity_establishing_png", {}))
    values["fov_priority"] = FovPriorityConfig(**dict(values.get("fov_priority", {})))
    allowed = {field.name for field in fields(VelocityEstablishingPngConfig)}
    unknown = sorted(set(values) - allowed)
    if unknown:
        raise RuntimeError("unknown controller config fields: " + ", ".join(unknown))
    return VelocityEstablishingPngConfig(**values)


def _controller_config_dict(config: VelocityEstablishingPngConfig) -> dict[str, Any]:
    result = {
        field.name: getattr(config, field.name)
        for field in fields(VelocityEstablishingPngConfig)
    }
    result["fov_priority"] = {
        field.name: getattr(config.fov_priority, field.name)
        for field in fields(FovPriorityConfig)
    }
    return result


def _algorithm_segments(rows: list[dict[str, str]]) -> list[list[dict[str, str]]]:
    segments: list[list[dict[str, str]]] = []
    current: list[dict[str, str]] = []
    for row in rows:
        if row.get("msp_publish_mode") == "algorithm":
            current.append(row)
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)
    return segments


def _vector(row: dict[str, str], *names: str) -> np.ndarray | None:
    values = [_float(row.get(name)) for name in names]
    if any(value is None for value in values):
        return None
    return np.asarray(values, dtype=float)


def _float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _required_float(row: dict[str, str], name: str) -> float:
    value = _float(row.get(name))
    if value is None:
        raise RuntimeError(f"CSV field {name} is missing or non-finite")
    return value


def _int(value: Any) -> int | None:
    parsed = _float(value)
    return None if parsed is None else int(parsed)


def _bool(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise RuntimeError(f"JSON root must be an object: {path}")
    return value


def _file_metadata(path: Path) -> dict[str, Any]:
    resolved = path.expanduser().resolve()
    digest = hashlib.sha256()
    with resolved.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {
        "path": _display_path(resolved),
        "sha256": digest.hexdigest(),
        "bytes": resolved.stat().st_size,
    }


def _write_json_atomic(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _display_path(path: Path) -> str:
    resolved = path.expanduser().resolve()
    try:
        return str(resolved.relative_to(ROOT))
    except ValueError:
        return str(resolved)


if __name__ == "__main__":
    main()
