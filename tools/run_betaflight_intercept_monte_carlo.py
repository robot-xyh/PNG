#!/usr/bin/env python3
"""Run parallel matrix15 Monte Carlo interception acceptance tests."""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import csv
from dataclasses import fields
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Iterable, Mapping
import zlib


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision_guidance.betaflight_intercept_eval import (  # noqa: E402
    InterceptionAcceptanceCriteria,
    evaluate_interception_results,
)
from vision_guidance.betaflight_png_sim import (  # noqa: E402
    CONTROLLER_MODES,
    MATRIX15_CASES,
    START_PROFILES,
    ClosedLoopSimulationConfig,
    MatrixCase,
    simulate_case,
)
from vision_guidance.thrust_model import VoltageThrottleThrustModel  # noqa: E402


RELEASE_SOURCE_PATHS = {
    "controller": ROOT / "vision_guidance" / "betaflight_intercept_controller.py",
    "flight_control": ROOT / "vision_guidance" / "flight_control.py",
    "simulation": ROOT / "vision_guidance" / "betaflight_png_sim.py",
    "thrust_model": ROOT / "vision_guidance" / "thrust_model.py",
    "monte_carlo_runner": Path(__file__).resolve(),
}
RELEASE_THRUST_VOLTAGE_MIN_V = 22.0
RELEASE_THRUST_VOLTAGE_MAX_V = 25.2
RELEASE_THRUST_VALIDATION_SAMPLES_MIN = 100
EVIDENCE_ROLES = {
    "diagnostic",
    "screening_baseline",
    "contact_performance",
    "noncollision_safety",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="config/betaflight.intercept_eval.example.json",
        help="Monte Carlo scenario and acceptance configuration.",
    )
    parser.add_argument("--output", required=True, help="Summary JSON path.")
    parser.add_argument("--csv", required=True, help="Per-trial result CSV path.")
    parser.add_argument("--trials-per-case", type=int, default=0)
    parser.add_argument("--workers", type=int, default=min(8, os.cpu_count() or 1))
    parser.add_argument(
        "--scenario-names",
        default="",
        help="Optional comma-separated scenario subset.",
    )
    parser.add_argument(
        "--evaluation-names",
        default="",
        help="Optional comma-separated evaluation subset.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.workers <= 0:
        raise SystemExit("--workers must be positive")
    source_config_path = Path(args.config).expanduser().resolve()
    spec = _read_json(source_config_path)
    scenarios = _select_named(
        _mapping_list(spec, "scenarios"), args.scenario_names, "scenario"
    )
    evaluations = _select_named(
        _mapping_list(spec, "evaluations"), args.evaluation_names, "evaluation"
    )
    trials_per_case = int(args.trials_per_case or spec.get("trials_per_case", 0))
    if trials_per_case <= 0:
        raise SystemExit("trials_per_case must be positive")
    base_seed = int(spec.get("base_seed", 0))
    if base_seed < 0:
        raise SystemExit("base_seed must be non-negative")
    criteria = InterceptionAcceptanceCriteria(**dict(spec.get("acceptance", {})))
    base_simulation = _simulation_values(spec.get("simulation", {}))
    base_simulation, runtime_binding = _bind_runtime_config(
        base_simulation,
        spec.get("runtime_binding"),
    )
    cases = _cases(
        spec.get("cases"),
        mirror=bool(spec.get("mirror_cases", False)),
    )
    tasks = _build_tasks(
        base_simulation=base_simulation,
        scenarios=scenarios,
        evaluations=evaluations,
        trials_per_case=trials_per_case,
        base_seed=base_seed,
        cases=cases,
    )

    if args.workers == 1:
        rows = [_run_task(task) for task in tasks]
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as executor:
            rows = list(executor.map(_run_task, tasks, chunksize=1))

    summaries = _summarize_rows(rows, scenarios, evaluations, criteria)
    contact_summaries = [
        summary
        for summary in summaries
        if summary["evidence_role"] == "contact_performance"
        and summary["required_for_release"]
    ]
    initial_performance_target = _initial_performance_verdict(
        spec.get("initial_performance"), contact_summaries
    )
    paired_screening = _paired_screening_verdict(
        spec.get("paired_screening"),
        rows=rows,
        cases=cases,
        scenarios=scenarios,
        evaluations=evaluations,
    )
    policy_results = _release_policy_verdict(
        config_schema_version=int(spec["schema_version"]),
        rows=rows,
        summaries=summaries,
        scenarios=scenarios,
        evaluations=evaluations,
        paired_screening=paired_screening,
        noncollision_acceptance=spec.get("noncollision_acceptance"),
        runtime_binding=runtime_binding,
    )
    required_summaries = [
        summary for summary in summaries if summary["required_for_release"]
    ]
    report = {
        "schema_version": 3,
        "purpose": "stochastic interception release evaluation",
        "limitations": [
            "Point-mass dynamics and idealized first-order body-rate response are not a flight approval.",
            "Noise models are configured surrogates, not a fitted YOLO/ByteTrack error distribution.",
            "This Monte Carlo runner does not emit Betaflight RC/PWM; active runtime use remains approval-gated.",
            "The candidate uses the production LOS filter and delayed/noisy own velocity, but its noise model is not fitted to flight data.",
            "Contact-policy hit rates are project-performance evidence and are not represented as the non-collision runtime policy.",
            "Non-collision acceptance measures timely transfer to the pilot; it does not claim autonomous collision avoidance after ABORT.",
        ],
        "source_config": str(source_config_path),
        "source_config_binding": _file_binding(source_config_path),
        "source_bindings": _release_source_bindings(),
        "runtime_binding": runtime_binding,
        "thrust_model_binding": (
            None if runtime_binding is None else runtime_binding.get("thrust_model")
        ),
        "evidence": dict(spec.get("evidence", {})),
        "base_seed": base_seed,
        "trials_per_case": trials_per_case,
        "worker_count": args.workers,
        "case_count": len(cases),
        "cases": [dict(vars(case)) for case in cases],
        "row_count": len(rows),
        "simulation": base_simulation,
        "acceptance": criteria.to_dict(),
        "scenarios": scenarios,
        "evaluations": evaluations,
        "summaries": summaries,
        "required_summary_count": len(required_summaries),
        "initial_performance_target": initial_performance_target,
        "initial_performance_target_passed": (
            None
            if initial_performance_target is None
            else bool(initial_performance_target["passed"])
        ),
        "paired_screening": paired_screening,
        "policy_results": policy_results,
        "release_passed": bool(policy_results["passed"]),
    }
    output_path = Path(args.output).expanduser().resolve()
    csv_path = Path(args.csv).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    _write_csv(csv_path, rows)
    for summary in summaries:
        print(
            f"scenario={summary['scenario_name']} "
            f"evaluation={summary['evaluation_name']} "
            f"visible_hit={summary['initially_visible_hit_rate']:.3f} "
            f"fov_hit={summary['initially_visible_fov_hit_rate']:.3f} "
            f"stale={summary['target_stale_failure_rate']:.3f} "
            f"passed={int(bool(summary['passed']))} "
            f"required={int(bool(summary['required_for_release']))}"
        )
    if initial_performance_target is not None:
        print(
            "initial_performance_target_passed="
            f"{int(bool(initial_performance_target['passed']))}"
        )
    if paired_screening is not None:
        print(
            "paired_screening_passed="
            f"{int(bool(paired_screening['passed']))} "
            f"selected={paired_screening['selected_evaluation']}"
        )
    print(f"release_passed={int(report['release_passed'])}")
    print(f"output={output_path}")
    print(f"csv={csv_path}")


def _build_tasks(
    *,
    base_simulation: Mapping[str, object],
    scenarios: list[dict[str, object]],
    evaluations: list[dict[str, object]],
    trials_per_case: int,
    base_seed: int,
    cases: Iterable[MatrixCase] = MATRIX15_CASES,
) -> list[dict[str, object]]:
    tasks = []
    for scenario in scenarios:
        scenario_name = str(scenario["name"])
        scenario_seed = zlib.crc32(scenario_name.encode("utf-8")) & 0xFFFFFFFF
        scenario_values = {
            key: value for key, value in scenario.items() if key != "name"
        }
        for evaluation in evaluations:
            evaluation_name = str(evaluation["name"])
            mode = str(evaluation.get("controller_mode", ""))
            start = str(evaluation.get("start_profile", ""))
            evidence_role = str(
                evaluation.get("evidence_role", "diagnostic")
            ).strip()
            if evidence_role not in EVIDENCE_ROLES:
                raise ValueError(
                    f"unsupported evidence_role for {evaluation_name}: {evidence_role}"
                )
            if mode not in CONTROLLER_MODES:
                raise ValueError(f"unsupported controller_mode for {evaluation_name}: {mode}")
            if start not in START_PROFILES:
                raise ValueError(f"unsupported start_profile for {evaluation_name}: {start}")
            evaluation_overrides = _evaluation_simulation_overrides(evaluation)
            simulation = _simulation_values(
                {
                    **base_simulation,
                    **scenario_values,
                    **evaluation_overrides,
                }
            )
            engagement_policy = (
                str(simulation.get("candidate_engagement_policy", "contact"))
                if mode == "candidate_velocity_hold_variable_thrust"
                else "not_applicable"
            )
            if evidence_role == "contact_performance" and engagement_policy != "contact":
                raise ValueError(
                    f"contact_performance evaluation {evaluation_name} must use contact policy"
                )
            if evidence_role == "noncollision_safety" and engagement_policy != "noncollision":
                raise ValueError(
                    f"noncollision_safety evaluation {evaluation_name} must use noncollision policy"
                )
            for trial_index in range(trials_per_case):
                trial_seed = (base_seed + scenario_seed + trial_index) & 0xFFFFFFFF
                trial_simulation = {**simulation, "random_seed": trial_seed}
                for case in cases:
                    tasks.append(
                        {
                            "scenario_name": scenario_name,
                            "evaluation_name": evaluation_name,
                            "controller_mode": mode,
                            "start_profile": start,
                            "required_for_release": bool(
                                evaluation.get("required_for_release", False)
                            ),
                            "evidence_role": evidence_role,
                            "engagement_policy": engagement_policy,
                            "trial_index": trial_index,
                            "random_seed": trial_seed,
                            "case": case,
                            "simulation": trial_simulation,
                        }
                    )
    return tasks


def _run_task(task: Mapping[str, object]) -> dict[str, object]:
    case = task["case"]
    config = ClosedLoopSimulationConfig(**dict(task["simulation"]))
    result = simulate_case(
        case,
        controller_mode=str(task["controller_mode"]),
        start_profile=str(task["start_profile"]),
        config=config,
    )
    return {
        "scenario_name": str(task["scenario_name"]),
        "evaluation_name": str(task["evaluation_name"]),
        "required_for_release": bool(task["required_for_release"]),
        "evidence_role": str(task["evidence_role"]),
        "engagement_policy": str(task["engagement_policy"]),
        "trial_index": int(task["trial_index"]),
        "random_seed": int(task["random_seed"]),
        **result.to_dict(),
    }


def _summarize_rows(
    rows: list[dict[str, object]],
    scenarios: list[dict[str, object]],
    evaluations: list[dict[str, object]],
    criteria: InterceptionAcceptanceCriteria,
) -> list[dict[str, object]]:
    summaries = []
    for scenario in scenarios:
        for evaluation in evaluations:
            scenario_name = str(scenario["name"])
            evaluation_name = str(evaluation["name"])
            selected = [
                row
                for row in rows
                if row["scenario_name"] == scenario_name
                and row["evaluation_name"] == evaluation_name
            ]
            summary = evaluate_interception_results(selected, criteria)
            summaries.append(
                {
                    "scenario_name": scenario_name,
                    "evaluation_name": evaluation_name,
                    "controller_mode": str(evaluation["controller_mode"]),
                    "start_profile": str(evaluation["start_profile"]),
                    "evidence_role": str(
                        evaluation.get("evidence_role", "diagnostic")
                    ),
                    "engagement_policy": str(
                        selected[0].get("engagement_policy", "not_applicable")
                    ),
                    "required_for_release": bool(
                        evaluation.get("required_for_release", False)
                    ),
                    **summary,
                }
            )
    return summaries


def _simulation_values(raw: object) -> dict[str, object]:
    if not isinstance(raw, Mapping):
        raise ValueError("simulation configuration must be an object")
    allowed = {field.name for field in fields(ClosedLoopSimulationConfig)}
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(f"unknown simulation fields: {', '.join(unknown)}")
    values = dict(raw)
    ClosedLoopSimulationConfig(**values)
    return values


def _evaluation_simulation_overrides(
    evaluation: Mapping[str, object],
) -> dict[str, object]:
    raw = evaluation.get("simulation_overrides", {})
    if not isinstance(raw, Mapping):
        raise ValueError("evaluation simulation_overrides must be an object")
    allowed = {field.name for field in fields(ClosedLoopSimulationConfig)}
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(
            f"unknown evaluation simulation override fields: {', '.join(unknown)}"
        )
    return dict(raw)


def _bind_runtime_config(
    simulation: Mapping[str, object],
    raw: object,
) -> tuple[dict[str, object], dict[str, object] | None]:
    if raw is None:
        return dict(simulation), None
    if not isinstance(raw, Mapping):
        raise ValueError("runtime_binding must be an object")
    allowed = {"config", "sha256"}
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(f"unknown runtime_binding fields: {', '.join(unknown)}")
    config_name = str(raw.get("config", "")).strip()
    expected_sha256 = str(raw.get("sha256", "")).strip().lower()
    if not config_name or len(expected_sha256) != 64:
        raise ValueError("runtime_binding requires config and 64-character sha256")
    config_path = Path(config_name).expanduser()
    if not config_path.is_absolute():
        config_path = ROOT / config_path
    config_path = config_path.resolve()
    payload = config_path.read_bytes()
    actual_sha256 = hashlib.sha256(payload).hexdigest()
    if actual_sha256 != expected_sha256:
        raise ValueError(
            "runtime config SHA256 mismatch: "
            f"expected {expected_sha256}, got {actual_sha256}"
        )
    runtime = json.loads(payload)
    if not isinstance(runtime, Mapping):
        raise ValueError("runtime config root must be an object")
    thrust_model_binding = _runtime_thrust_model_binding(
        runtime,
        config_path=config_path,
    )
    derived = _derive_runtime_simulation(
        runtime,
        thrust_model_binding=thrust_model_binding,
    )
    bound = dict(simulation)
    mismatches = []
    for name, runtime_value in derived.items():
        if name in bound and not _equivalent_config_value(bound[name], runtime_value):
            mismatches.append(
                f"{name}: simulation={bound[name]!r}, runtime={runtime_value!r}"
            )
        else:
            bound[name] = runtime_value
    if mismatches:
        raise ValueError(
            "simulation fields disagree with bound runtime config: "
            + "; ".join(mismatches)
        )
    _simulation_values(bound)
    return bound, {
        "config": str(config_path),
        "sha256": actual_sha256,
        "derived_simulation": derived,
        "thrust_model": thrust_model_binding,
    }


def _derive_runtime_simulation(
    runtime: Mapping[str, object],
    *,
    thrust_model_binding: Mapping[str, object] | None = None,
) -> dict[str, object]:
    guidance = _required_mapping(runtime, "guidance")
    if str(guidance.get("law", "")).strip() != "velocity_establishing_png":
        raise ValueError("runtime binding requires velocity_establishing_png guidance")
    if str(guidance.get("velocity_source", "")).strip() != "msp_kinematics":
        raise ValueError("runtime binding requires msp_kinematics velocity")
    velocity = _required_mapping(guidance, "velocity_establishing_png")
    command = _required_mapping(runtime, "guidance_command")
    accel = _required_mapping(command, "accel_tilt_rate")
    entry = _required_mapping(command, "entry_handoff")
    thrust = _required_mapping(accel, "thrust_feedforward")
    camera = _required_mapping(runtime, "camera")
    tracker = _required_mapping(runtime, "rknn_bytetrack")
    msp = _required_mapping(runtime, "msp_runtime")
    rc = _required_mapping(runtime, "rc_mapping")

    msp_slew = float(msp["throttle_slew_limit_us_per_s"])
    rc_slew = float(rc["max_delta_us_per_s"])
    if not math.isclose(msp_slew, rc_slew, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError("runtime throttle slew values disagree")
    if int(msp["throttle_command_min_us"]) != int(rc["throttle_min_us"]):
        raise ValueError("runtime throttle minimum values disagree")
    if int(msp["throttle_command_max_us"]) != int(rc["throttle_max_us"]):
        raise ValueError("runtime throttle maximum values disagree")
    for axis in ("roll", "pitch"):
        command_limit = float(rc[f"{axis}_command_limit_deg_s"])
        controller_limit = float(accel[f"max_{axis}_rate_deg_s"])
        if not math.isclose(
            command_limit,
            controller_limit,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise ValueError(f"runtime {axis} rate limits disagree")
    roll_kp = float(accel["roll_attitude_kp_s_inv"])
    pitch_kp = float(accel["pitch_attitude_kp_s_inv"])
    if not math.isclose(roll_kp, pitch_kp, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError("runtime attitude gains disagree")
    guidance_limit = float(guidance["max_guidance_accel_mps2"])
    controller_limit = float(velocity["total_accel_limit_m_s2"])
    if not math.isclose(
        guidance_limit,
        controller_limit,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        raise ValueError("runtime guidance acceleration limits disagree")

    width = float(camera["width"])
    height = float(camera["height"])
    fx = float(camera["fx"])
    fy = float(camera["fy"])
    cx = float(camera["cx"])
    cy = float(camera["cy"])
    horizontal_margin_px = min(cx, width - cx)
    vertical_margin_px = min(cy, height - cy)
    if min(horizontal_margin_px, vertical_margin_px, fx, fy) <= 0.0:
        raise ValueError("runtime camera intrinsics do not define a positive FOV")
    horizontal_half_fov_deg = math.degrees(math.atan(horizontal_margin_px / fx))
    vertical_half_fov_deg = math.degrees(math.atan(vertical_margin_px / fy))
    fov_priority = velocity.get("fov_priority", {})
    if not isinstance(fov_priority, Mapping):
        raise ValueError("runtime velocity FOV priority must be an object")
    if bool(fov_priority.get("enabled", False)):
        for field_name, derived_value in (
            ("horizontal_half_fov_deg", horizontal_half_fov_deg),
            ("vertical_half_fov_deg", vertical_half_fov_deg),
        ):
            configured_value = float(fov_priority[field_name])
            if not math.isclose(
                configured_value,
                derived_value,
                rel_tol=1.0e-9,
                abs_tol=1.0e-9,
            ):
                raise ValueError(
                    f"runtime FOV priority {field_name} disagrees with camera intrinsics"
                )
    gravity = float(velocity.get("gravity_m_s2", accel["gravity_mps2"]))
    throttle_dynamics_enabled = bool(thrust.get("enabled", False))
    if throttle_dynamics_enabled:
        if str(thrust.get("model", "")).strip() != "voltage_throttle_lut":
            raise ValueError(
                "runtime throttle dynamics require voltage_throttle_lut"
            )
        if thrust_model_binding is None:
            raise ValueError("runtime thrust LUT binding is missing")
    derived = {
        "navigation_constant": float(velocity["navigation_constant"]),
        "guidance_accel_limit_m_s2": float(velocity["png_accel_limit_m_s2"]),
        "gravity_m_s2": gravity,
        "upward_centering_gain_s2": float(velocity["fov_centering_gain_s2"]),
        "upward_centering_accel_limit_m_s2": float(
            velocity["fov_centering_accel_limit_m_s2"]
        ),
        "speed_hold_gain_s_inv": float(velocity["speed_gain_s_inv"]),
        "speed_hold_accel_limit_m_s2": float(velocity["speed_accel_limit_m_s2"]),
        "total_accel_limit_m_s2": float(velocity["total_accel_limit_m_s2"]),
        "vertical_speed_reference_limit_m_s": float(
            velocity["vertical_speed_reference_limit_m_s"]
        ),
        "max_roll_tilt_deg": float(accel["max_roll_tilt_deg"]),
        "max_pitch_tilt_deg": float(accel["max_pitch_tilt_deg"]),
        "attitude_kp_s_inv": roll_kp,
        "max_roll_rate_deg_s": float(accel["max_roll_rate_deg_s"]),
        "max_pitch_rate_deg_s": float(accel["max_pitch_rate_deg_s"]),
        "control_rate_hz": float(msp["control_publish_hz"]),
        "entry_handoff_enabled": bool(entry["enabled"]),
        "entry_handoff_duration_s": float(entry["duration_s"]),
        "min_thrust_specific_force_m_s2": float(
            accel["min_vertical_specific_force_mps2"]
        ),
        "max_thrust_specific_force_m_s2": (
            float(thrust["max_load_factor_g"]) * gravity
        ),
        "throttle_dynamics_enabled": throttle_dynamics_enabled,
        "thrust_response_tau_s": (
            float(thrust_model_binding["dynamics"]["first_order_time_constant_s"])
            if throttle_dynamics_enabled
            else 0.0
        ),
        "throttle_handover_duration_s": float(msp["throttle_handover_s"]),
        "throttle_slew_limit_us_per_s": msp_slew,
        "throttle_min_us": float(rc["throttle_min_us"]),
        "throttle_hover_us": float(rc["throttle_hover_us"]),
        "throttle_max_us": float(rc["throttle_max_us"]),
        "hover_load_factor_g": float(thrust["hover_load_factor_g"]),
        "max_load_factor_g": float(thrust["max_load_factor_g"]),
        "camera_horizontal_half_fov_deg": horizontal_half_fov_deg,
        "camera_vertical_half_fov_deg": vertical_half_fov_deg,
        "perception_rate_hz": float(tracker["perception_rate_hz"]),
        "perception_stale_timeout_s": float(velocity["detection_timeout_s"]),
        "kinematic_rate_hz": float(msp["raw_gps_poll_hz"]),
        "kinematic_stale_timeout_s": float(velocity["velocity_timeout_s"]),
        "candidate_png_track_speed_ratio": float(
            velocity.get("png_track_speed_ratio", 0.8)
        ),
        "candidate_velocity_reference_slew_m_s2": float(
            velocity.get("velocity_reference_slew_m_s2", 3.0)
        ),
        "candidate_acquire_consecutive_frames": int(
            velocity["acquire_consecutive_frames"]
        ),
        "candidate_los_prediction_max_s": float(velocity["los_prediction_max_s"]),
        "candidate_fixed_vm_m_s": float(velocity["fixed_vm_m_s"]),
        "candidate_fov_constraint_half_angle_deg": float(
            velocity["fov_constraint_half_angle_deg"]
        ),
        "candidate_fov_priority_enabled": bool(
            fov_priority.get("enabled", False)
        ),
        "candidate_fov_priority_start_ratio": float(
            fov_priority.get("start_ratio", 0.70)
        ),
        "candidate_fov_priority_full_ratio": float(
            fov_priority.get("full_ratio", 0.90)
        ),
        "candidate_engagement_policy": str(
            velocity.get("engagement_policy", "noncollision")
        ),
        "candidate_noncollision_bbox_abort_ratio": float(
            velocity.get("noncollision_bbox_abort_ratio", 0.012)
        ),
        "candidate_noncollision_ttc_abort_s": float(
            velocity.get("noncollision_ttc_abort_s", 2.0)
        ),
        "candidate_contact_bbox_terminal_ratio": float(
            velocity.get("contact_bbox_terminal_ratio", 0.05)
        ),
        "candidate_contact_ttc_terminal_s": float(
            velocity.get("contact_ttc_terminal_s", 1.0)
        ),
        "candidate_contact_bbox_complete_ratio": float(
            velocity.get("contact_bbox_complete_ratio", 0.25)
        ),
        "candidate_blind_hold_s": float(velocity.get("blind_hold_s", 0.20)),
    }
    if throttle_dynamics_enabled:
        derived.update(
            thrust_model_path=str(thrust_model_binding["path"]),
            thrust_model_sha256=str(thrust_model_binding["sha256"]),
            thrust_model_calibration_id=str(
                thrust_model_binding["calibration_id"]
            ),
        )
    return derived


def _runtime_thrust_model_binding(
    runtime: Mapping[str, object],
    *,
    config_path: Path,
) -> dict[str, object] | None:
    command = _required_mapping(runtime, "guidance_command")
    accel = _required_mapping(command, "accel_tilt_rate")
    thrust = _required_mapping(accel, "thrust_feedforward")
    if not bool(thrust.get("enabled", False)):
        return None
    if str(thrust.get("model", "")).strip() != "voltage_throttle_lut":
        raise ValueError("release runtime requires a voltage_throttle_lut")
    raw_model_path = str(thrust.get("model_path", "")).strip()
    if not raw_model_path:
        raise ValueError("runtime thrust LUT model_path is missing")
    raw_path = Path(raw_model_path).expanduser()
    model_path = _resolve_config_artifact_path(raw_path, config_path=config_path)
    try:
        model = VoltageThrottleThrustModel.from_file(
            model_path,
            expected_sha256=str(thrust.get("model_sha256", "")),
            expected_calibration_id=str(thrust.get("calibration_id", "")),
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError(f"runtime thrust LUT cannot be loaded: {exc}") from exc
    if (
        model.minimum_voltage_v > RELEASE_THRUST_VOLTAGE_MIN_V
        or model.maximum_voltage_v < RELEASE_THRUST_VOLTAGE_MAX_V
    ):
        raise ValueError("runtime thrust LUT must cover 22.0-25.2 V")
    try:
        validation_samples = int(model.validation["sample_count"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("runtime thrust LUT sample_count is missing") from exc
    if validation_samples < RELEASE_THRUST_VALIDATION_SAMPLES_MIN:
        raise ValueError("runtime thrust LUT validation sample count is insufficient")
    dynamics = model.metadata().get("dynamics")
    if not isinstance(dynamics, dict):
        raise ValueError("runtime thrust LUT dynamics are missing")
    try:
        time_constant_s = float(dynamics["first_order_time_constant_s"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("runtime thrust LUT first-order time constant is missing") from exc
    if not math.isfinite(time_constant_s) or not 0.0 < time_constant_s <= 0.5:
        raise ValueError("runtime thrust LUT first-order time constant is invalid")
    rc = _required_mapping(runtime, "rc_mapping")
    throttle_min = float(rc["throttle_min_us"])
    throttle_max = float(rc["throttle_max_us"])
    if (
        float(model.throttle_us[0]) > throttle_min
        or float(model.throttle_us[-1]) < throttle_max
    ):
        raise ValueError("runtime thrust LUT does not cover the throttle envelope")
    return model.metadata()


def _resolve_config_artifact_path(path: Path, *, config_path: Path) -> Path:
    if path.is_absolute():
        return path.resolve()
    repository_candidate = (ROOT / path).resolve()
    config_candidate = (config_path.resolve().parent / path).resolve()
    if repository_candidate.is_file() or not config_candidate.is_file():
        return repository_candidate
    return config_candidate


def _required_mapping(values: Mapping[str, object], key: str) -> Mapping[str, object]:
    result = values.get(key)
    if not isinstance(result, Mapping):
        raise ValueError(f"runtime config requires object field {key}")
    return result


def _equivalent_config_value(left: object, right: object) -> bool:
    if isinstance(left, bool) or isinstance(right, bool):
        return type(left) is type(right) and left == right
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(float(left), float(right), rel_tol=1.0e-9, abs_tol=1.0e-9)
    return left == right


def _mapping_list(spec: Mapping[str, object], key: str) -> list[dict[str, object]]:
    raw = spec.get(key)
    if not isinstance(raw, list) or not raw:
        raise ValueError(f"{key} must be a non-empty list")
    values = []
    names = set()
    for item in raw:
        if not isinstance(item, Mapping):
            raise ValueError(f"every {key} entry must be an object")
        value = dict(item)
        name = str(value.get("name", "")).strip()
        if not name:
            raise ValueError(f"every {key} entry requires a name")
        if name in names:
            raise ValueError(f"duplicate {key} name: {name}")
        names.add(name)
        value["name"] = name
        values.append(value)
    return values


def _cases(raw: object, *, mirror: bool = False) -> tuple[MatrixCase, ...]:
    if raw is None:
        cases = list(MATRIX15_CASES)
        return _with_mirrored_cases(cases) if mirror else tuple(cases)
    if not isinstance(raw, list) or not raw:
        raise ValueError("cases must be a non-empty list")
    cases = []
    case_ids = set()
    allowed = {field.name for field in fields(MatrixCase)}
    for item in raw:
        if not isinstance(item, Mapping):
            raise ValueError("every cases entry must be an object")
        unknown = sorted(set(item) - allowed)
        if unknown:
            raise ValueError(f"unknown case fields: {', '.join(unknown)}")
        case = MatrixCase(**dict(item))
        if case.case_id in case_ids:
            raise ValueError(f"duplicate case_id: {case.case_id}")
        case_ids.add(case.case_id)
        cases.append(case)
    return _with_mirrored_cases(cases) if mirror else tuple(cases)


def _with_mirrored_cases(cases: Iterable[MatrixCase]) -> tuple[MatrixCase, ...]:
    original = tuple(cases)
    mirrored = tuple(
        MatrixCase(
            case_id=f"{case.case_id}M",
            horizontal_range_m=case.horizontal_range_m,
            lateral_offset_m=-case.lateral_offset_m,
            altitude_offset_m=case.altitude_offset_m,
            target_speed_m_s=case.target_speed_m_s,
            speed_ratio=case.speed_ratio,
            target_course_deg=(-case.target_course_deg) % 360.0,
        )
        for case in original
    )
    return original + mirrored


def _initial_performance_verdict(
    raw: object,
    required_summaries: list[Mapping[str, object]],
) -> dict[str, object] | None:
    if raw is None:
        return None
    if not isinstance(raw, Mapping):
        raise ValueError("initial_performance must be an object")
    allowed = {
        "initially_visible_hit_rate_min",
        "initially_visible_fov_hit_rate_min",
        "description",
    }
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(f"unknown initial_performance fields: {', '.join(unknown)}")
    try:
        threshold = float(raw["initially_visible_hit_rate_min"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            "initial_performance.initially_visible_hit_rate_min must be numeric"
        ) from exc
    if not 0.0 <= threshold <= 1.0:
        raise ValueError(
            "initial_performance.initially_visible_hit_rate_min must be in [0, 1]"
        )
    thresholds = {"initially_visible_hit_rate": threshold}
    if "initially_visible_fov_hit_rate_min" in raw:
        try:
            fov_threshold = float(raw["initially_visible_fov_hit_rate_min"])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "initial_performance.initially_visible_fov_hit_rate_min must be numeric"
            ) from exc
        if not 0.0 <= fov_threshold <= 1.0:
            raise ValueError(
                "initial_performance.initially_visible_fov_hit_rate_min must be in [0, 1]"
            )
        thresholds["initially_visible_fov_hit_rate"] = fov_threshold
    checks = {}
    for metric, metric_threshold in thresholds.items():
        observed = [float(summary[metric]) for summary in required_summaries]
        observed_minimum = min(observed) if observed else None
        checks[metric] = {
            "operator": ">=",
            "threshold": metric_threshold,
            "observed_minimum": observed_minimum,
            "passed": bool(observed)
            and all(value >= metric_threshold for value in observed),
        }
    return {
        "scope": "required_for_release_summaries",
        "summary_count": len(required_summaries),
        "checks": checks,
        "passed": all(bool(check["passed"]) for check in checks.values()),
        "description": str(raw.get("description", "")),
        "does_not_imply_release": True,
    }


def _release_policy_verdict(
    *,
    config_schema_version: int,
    rows: list[dict[str, object]],
    summaries: list[dict[str, object]],
    scenarios: list[dict[str, object]],
    evaluations: list[dict[str, object]],
    paired_screening: Mapping[str, object] | None,
    noncollision_acceptance: object,
    runtime_binding: Mapping[str, object] | None,
) -> dict[str, object]:
    if config_schema_version < 2:
        return {
            "passed": False,
            "reason": "legacy_config_does_not_separate_contact_and_noncollision_evidence",
            "contact_performance": None,
            "noncollision_safety": None,
        }
    if runtime_binding is None:
        raise ValueError("release Monte Carlo requires runtime_binding")
    derived = runtime_binding.get("derived_simulation")
    if not isinstance(derived, Mapping):
        raise ValueError("runtime_binding derived_simulation is missing")
    runtime_policy = str(derived.get("candidate_engagement_policy", ""))
    if runtime_policy != "noncollision":
        raise ValueError("supervised release runtime must use noncollision policy")
    if paired_screening is None or paired_screening.get("passed") is not True:
        paired_selected = None
    else:
        paired_selected = str(paired_screening.get("selected_evaluation", ""))

    scenario_names = {str(scenario["name"]) for scenario in scenarios}
    contact_evaluations = [
        evaluation
        for evaluation in evaluations
        if evaluation.get("evidence_role") == "contact_performance"
        and evaluation.get("required_for_release") is True
    ]
    if len(contact_evaluations) != 1:
        raise ValueError(
            "release Monte Carlo requires exactly one contact_performance evaluation"
        )
    contact_name = str(contact_evaluations[0]["name"])
    contact_summaries = [
        summary
        for summary in summaries
        if summary["evaluation_name"] == contact_name
        and summary["evidence_role"] == "contact_performance"
    ]
    contact_coverage = {
        str(summary["scenario_name"]) for summary in contact_summaries
    } == scenario_names
    contact_passed = bool(
        paired_selected == contact_name
        and contact_coverage
        and contact_summaries
        and all(
            summary.get("engagement_policy") == "contact"
            and summary.get("passed") is True
            for summary in contact_summaries
        )
    )
    contact_result = {
        "engagement_policy": "contact",
        "evaluation_name": contact_name,
        "scenario_count": len(contact_summaries),
        "scenario_names": sorted(
            str(summary["scenario_name"]) for summary in contact_summaries
        ),
        "paired_screening_selected": paired_selected == contact_name,
        "hit_rate_and_fov_hit_rate_required": True,
        "passed": contact_passed,
        "authorizes_contact_flight": False,
    }

    safety_acceptance = _noncollision_acceptance(noncollision_acceptance)
    safety_evaluations = [
        evaluation
        for evaluation in evaluations
        if evaluation.get("evidence_role") == "noncollision_safety"
        and evaluation.get("required_for_release") is True
    ]
    if len(safety_evaluations) != 1:
        raise ValueError(
            "release Monte Carlo requires exactly one noncollision_safety evaluation"
        )
    safety_name = str(safety_evaluations[0]["name"])
    safety_summaries = []
    for scenario_name in sorted(scenario_names):
        selected = [
            row
            for row in rows
            if row["scenario_name"] == scenario_name
            and row["evaluation_name"] == safety_name
        ]
        safety_summaries.append(
            _noncollision_safety_summary(
                selected,
                scenario_name=scenario_name,
                evaluation_name=safety_name,
                acceptance=safety_acceptance,
            )
        )
    noncollision_passed = bool(safety_summaries) and all(
        summary["passed"] for summary in safety_summaries
    )
    noncollision_result = {
        "engagement_policy": "noncollision",
        "evaluation_name": safety_name,
        "acceptance": safety_acceptance,
        "scenario_count": len(safety_summaries),
        "summaries": safety_summaries,
        "passed": noncollision_passed,
        "requires_pilot_action_after_abort": True,
    }
    return {
        "runtime_engagement_policy": runtime_policy,
        "contact_performance": contact_result,
        "noncollision_safety": noncollision_result,
        "contact_evidence_is_not_noncollision_flight_authority": True,
        "passed": bool(contact_passed and noncollision_passed),
        "reason": (
            "contact_and_noncollision_checks_passed"
            if contact_passed and noncollision_passed
            else "contact_or_noncollision_check_failed"
        ),
    }


def _noncollision_acceptance(raw: object) -> dict[str, float]:
    if not isinstance(raw, Mapping):
        raise ValueError("noncollision_acceptance must be an object")
    allowed = {
        "timely_abort_rate_min",
        "unsafe_contact_rate_max",
        "minimum_abort_lead_time_s",
    }
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(
            "unknown noncollision_acceptance fields: " + ", ".join(unknown)
        )
    try:
        result = {
            "timely_abort_rate_min": float(raw["timely_abort_rate_min"]),
            "unsafe_contact_rate_max": float(raw["unsafe_contact_rate_max"]),
            "minimum_abort_lead_time_s": float(raw["minimum_abort_lead_time_s"]),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("noncollision_acceptance fields must be numeric") from exc
    if not 0.0 <= result["timely_abort_rate_min"] <= 1.0:
        raise ValueError("timely_abort_rate_min must be in [0, 1]")
    if not 0.0 <= result["unsafe_contact_rate_max"] <= 1.0:
        raise ValueError("unsafe_contact_rate_max must be in [0, 1]")
    if result["minimum_abort_lead_time_s"] <= 0.0:
        raise ValueError("minimum_abort_lead_time_s must be positive")
    return result


def _noncollision_safety_summary(
    rows: list[Mapping[str, object]],
    *,
    scenario_name: str,
    evaluation_name: str,
    acceptance: Mapping[str, float],
) -> dict[str, object]:
    visible = [row for row in rows if bool(row["initial_target_in_fov"])]
    lead_threshold = float(acceptance["minimum_abort_lead_time_s"])
    timely_count = 0
    unsafe_contact_count = 0
    abort_count = 0
    contact_leads = []
    for row in visible:
        raw_abort = row.get("controller_abort_time_s")
        abort_time = None if raw_abort is None else float(raw_abort)
        hit = bool(row["hit"])
        lead_time = None
        if abort_time is not None:
            abort_count += 1
            if hit:
                lead_time = max(0.0, float(row["elapsed_s"]) - abort_time)
                contact_leads.append(lead_time)
            if not hit or (lead_time is not None and lead_time >= lead_threshold):
                timely_count += 1
        if hit and (abort_time is None or lead_time is None or lead_time < lead_threshold):
            unsafe_contact_count += 1
    denominator = len(visible)
    timely_rate = timely_count / denominator if denominator else 0.0
    unsafe_rate = unsafe_contact_count / denominator if denominator else 1.0
    passed = bool(
        denominator
        and timely_rate >= float(acceptance["timely_abort_rate_min"])
        and unsafe_rate <= float(acceptance["unsafe_contact_rate_max"])
    )
    return {
        "scenario_name": scenario_name,
        "evaluation_name": evaluation_name,
        "engagement_policy": "noncollision",
        "initially_visible_count": denominator,
        "controller_abort_count": abort_count,
        "timely_abort_count": timely_count,
        "timely_abort_rate": timely_rate,
        "unsafe_contact_count": unsafe_contact_count,
        "unsafe_contact_rate": unsafe_rate,
        "minimum_observed_abort_to_contact_s": (
            min(contact_leads) if contact_leads else None
        ),
        "passed": passed,
    }


def _release_source_bindings() -> dict[str, dict[str, str]]:
    return {
        name: _file_binding(path) for name, path in RELEASE_SOURCE_PATHS.items()
    }


def _file_binding(path: Path) -> dict[str, str]:
    resolved = path.expanduser().resolve()
    binding = {
        "path": str(resolved),
        "sha256": hashlib.sha256(resolved.read_bytes()).hexdigest(),
    }
    try:
        binding["repository_path"] = str(resolved.relative_to(ROOT))
    except ValueError:
        pass
    return binding


def _paired_screening_verdict(
    raw: object,
    *,
    rows: list[dict[str, object]],
    cases: Iterable[MatrixCase],
    scenarios: list[dict[str, object]],
    evaluations: list[dict[str, object]],
) -> dict[str, object] | None:
    if raw is None:
        return None
    if not isinstance(raw, Mapping):
        raise ValueError("paired_screening must be an object")
    allowed = {
        "baseline_evaluation",
        "candidate_evaluations",
        "scenario_hit_rate_min",
        "scenario_fov_hit_rate_min",
        "outward_fov_improvement_min",
        "central_inward_hit_drop_max",
    }
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(f"unknown paired_screening fields: {', '.join(unknown)}")
    baseline_name = str(raw.get("baseline_evaluation", "")).strip()
    if not baseline_name:
        raise ValueError("paired_screening requires baseline_evaluation")
    evaluations_by_name = {str(value["name"]): value for value in evaluations}
    if baseline_name not in evaluations_by_name:
        raise ValueError("paired_screening baseline evaluation is unavailable")
    raw_candidates = raw.get("candidate_evaluations")
    if raw_candidates is None:
        candidate_names = [name for name in evaluations_by_name if name != baseline_name]
    elif isinstance(raw_candidates, list):
        candidate_names = [str(name).strip() for name in raw_candidates]
    else:
        raise ValueError("paired_screening candidate_evaluations must be a list")
    if not candidate_names or any(not name for name in candidate_names):
        raise ValueError("paired_screening requires candidate evaluations")
    missing = [name for name in candidate_names if name not in evaluations_by_name]
    if missing:
        raise ValueError(
            "paired_screening candidate evaluations are unavailable: "
            + ", ".join(missing)
        )
    thresholds = {
        "scenario_hit_rate_min": float(raw.get("scenario_hit_rate_min", 0.80)),
        "scenario_fov_hit_rate_min": float(
            raw.get("scenario_fov_hit_rate_min", 0.80)
        ),
        "outward_fov_improvement_min": float(
            raw.get("outward_fov_improvement_min", 0.10)
        ),
        "central_inward_hit_drop_max": float(
            raw.get("central_inward_hit_drop_max", 0.02)
        ),
    }
    if any(not 0.0 <= value <= 1.0 for value in thresholds.values()):
        raise ValueError("paired_screening thresholds must be in [0, 1]")

    case_by_id = {case.case_id: case for case in cases}
    baseline_rows = [
        row for row in rows if row["evaluation_name"] == baseline_name
    ]
    permitted_overrides = {
        "candidate_fov_priority_enabled",
        "candidate_fov_priority_start_ratio",
        "candidate_fov_priority_full_ratio",
        "candidate_engagement_policy",
    }
    candidate_results = []
    for candidate_name in candidate_names:
        candidate_rows = [
            row for row in rows if row["evaluation_name"] == candidate_name
        ]
        baseline_keys = {_paired_row_key(row) for row in baseline_rows}
        candidate_keys = {_paired_row_key(row) for row in candidate_rows}
        pairing_complete = bool(baseline_keys) and baseline_keys == candidate_keys
        scenario_checks = []
        for scenario in scenarios:
            scenario_name = str(scenario["name"])
            selected = [
                row
                for row in candidate_rows
                if row["scenario_name"] == scenario_name
                and bool(row["initial_target_in_fov"])
            ]
            hit_rate = _boolean_rate(selected, "hit")
            fov_rate = _boolean_rate(selected, "fov_feasible_hit")
            scenario_checks.append(
                {
                    "scenario_name": scenario_name,
                    "initially_visible_count": len(selected),
                    "hit_rate": hit_rate,
                    "fov_hit_rate": fov_rate,
                    "passed": bool(selected)
                    and hit_rate >= thresholds["scenario_hit_rate_min"]
                    and fov_rate >= thresholds["scenario_fov_hit_rate_min"],
                }
            )

        outward_case_ids = {
            case.case_id
            for case in case_by_id.values()
            if _case_motion_class(case) == "outward"
        }
        baseline_outward = [
            row
            for row in baseline_rows
            if row["case_id"] in outward_case_ids
            and bool(row["initial_target_in_fov"])
        ]
        candidate_outward = [
            row
            for row in candidate_rows
            if row["case_id"] in outward_case_ids
            and bool(row["initial_target_in_fov"])
        ]
        baseline_outward_fov = _boolean_rate(
            baseline_outward, "fov_feasible_hit"
        )
        candidate_outward_fov = _boolean_rate(
            candidate_outward, "fov_feasible_hit"
        )
        outward_improvement = candidate_outward_fov - baseline_outward_fov

        protected_checks = []
        for scenario in scenarios:
            scenario_name = str(scenario["name"])
            for case in case_by_id.values():
                motion_class = _case_motion_class(case)
                if motion_class == "outward":
                    continue
                baseline_selected = [
                    row
                    for row in baseline_rows
                    if row["scenario_name"] == scenario_name
                    and row["case_id"] == case.case_id
                    and bool(row["initial_target_in_fov"])
                ]
                candidate_selected = [
                    row
                    for row in candidate_rows
                    if row["scenario_name"] == scenario_name
                    and row["case_id"] == case.case_id
                    and bool(row["initial_target_in_fov"])
                ]
                if not baseline_selected or not candidate_selected:
                    continue
                baseline_hit = _boolean_rate(baseline_selected, "hit")
                candidate_hit = _boolean_rate(candidate_selected, "hit")
                hit_drop = baseline_hit - candidate_hit
                protected_checks.append(
                    {
                        "scenario_name": scenario_name,
                        "case_id": case.case_id,
                        "motion_class": motion_class,
                        "baseline_hit_rate": baseline_hit,
                        "candidate_hit_rate": candidate_hit,
                        "hit_rate_drop": hit_drop,
                        "passed": hit_drop
                        <= thresholds["central_inward_hit_drop_max"] + 1.0e-12,
                    }
                )

        evaluation = evaluations_by_name[candidate_name]
        override_fields = set(
            _evaluation_simulation_overrides(evaluation)
        )
        command_limits_unchanged = override_fields <= permitted_overrides
        passed = bool(
            pairing_complete
            and all(check["passed"] for check in scenario_checks)
            and baseline_outward
            and candidate_outward
            and outward_improvement
            >= thresholds["outward_fov_improvement_min"] - 1.0e-12
            and all(check["passed"] for check in protected_checks)
            and command_limits_unchanged
        )
        visible_candidate_rows = [
            row for row in candidate_rows if bool(row["initial_target_in_fov"])
        ]
        candidate_results.append(
            {
                "evaluation_name": candidate_name,
                "simulation_overrides": _evaluation_simulation_overrides(evaluation),
                "pairing_complete": pairing_complete,
                "scenario_checks": scenario_checks,
                "outward_case_ids": sorted(outward_case_ids),
                "outward_baseline_fov_hit_rate": baseline_outward_fov,
                "outward_candidate_fov_hit_rate": candidate_outward_fov,
                "outward_fov_hit_rate_improvement": outward_improvement,
                "protected_case_check_count": len(protected_checks),
                "worst_protected_hit_rate_drop": max(
                    (check["hit_rate_drop"] for check in protected_checks),
                    default=0.0,
                ),
                "protected_case_failures": [
                    check for check in protected_checks if not check["passed"]
                ],
                "command_limits_unchanged": command_limits_unchanged,
                "aggregate_visible_hit_rate": _boolean_rate(
                    visible_candidate_rows, "hit"
                ),
                "aggregate_visible_fov_hit_rate": _boolean_rate(
                    visible_candidate_rows, "fov_feasible_hit"
                ),
                "passed": passed,
            }
        )
    passing = [result for result in candidate_results if result["passed"]]
    selected = max(
        passing,
        key=lambda result: (
            result["outward_fov_hit_rate_improvement"],
            result["aggregate_visible_fov_hit_rate"],
            result["aggregate_visible_hit_rate"],
        ),
        default=None,
    )
    return {
        "baseline_evaluation": baseline_name,
        "thresholds": thresholds,
        "candidate_count": len(candidate_results),
        "candidate_results": candidate_results,
        "selected_evaluation": (
            None if selected is None else selected["evaluation_name"]
        ),
        "passed": selected is not None,
    }


def _paired_row_key(row: Mapping[str, object]) -> tuple[object, ...]:
    return (
        row["scenario_name"],
        row["case_id"],
        row["trial_index"],
        row["random_seed"],
    )


def _boolean_rate(rows: list[Mapping[str, object]], field: str) -> float:
    if not rows:
        return 0.0
    return sum(bool(row[field]) for row in rows) / len(rows)


def _case_motion_class(case: MatrixCase) -> str:
    lateral_velocity = case.target_speed_m_s * math.sin(
        math.radians(case.target_course_deg)
    )
    radial_lateral_motion = case.lateral_offset_m * lateral_velocity
    if radial_lateral_motion > 1.0e-9:
        return "outward"
    if radial_lateral_motion < -1.0e-9:
        return "inward"
    return "central"


def _select_named(
    values: list[dict[str, object]], raw_names: str, label: str
) -> list[dict[str, object]]:
    if not raw_names.strip():
        return values
    selected_names = tuple(
        name.strip() for name in raw_names.split(",") if name.strip()
    )
    by_name = {str(value["name"]): value for value in values}
    missing = [name for name in selected_names if name not in by_name]
    if missing:
        raise ValueError(f"unknown {label} names: {', '.join(missing)}")
    return [by_name[name] for name in selected_names]


def _read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Monte Carlo config root must be an object")
    if int(data.get("schema_version", 0)) not in {1, 2}:
        raise ValueError("unsupported Monte Carlo config schema_version")
    return data


def _write_csv(path: Path, rows: Iterable[Mapping[str, object]]) -> None:
    selected = list(rows)
    if not selected:
        raise ValueError("cannot write empty Monte Carlo CSV")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(selected[0]))
        writer.writeheader()
        writer.writerows(selected)


if __name__ == "__main__":
    main()
