#!/usr/bin/env python3
"""Run parallel matrix15 Monte Carlo interception acceptance tests."""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import csv
from dataclasses import fields
import json
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
    simulate_case,
)


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
    spec = _read_json(Path(args.config).expanduser().resolve())
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
    tasks = _build_tasks(
        base_simulation=base_simulation,
        scenarios=scenarios,
        evaluations=evaluations,
        trials_per_case=trials_per_case,
        base_seed=base_seed,
    )

    if args.workers == 1:
        rows = [_run_task(task) for task in tasks]
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as executor:
            rows = list(executor.map(_run_task, tasks, chunksize=1))

    summaries = _summarize_rows(rows, scenarios, evaluations, criteria)
    required_summaries = [
        summary for summary in summaries if summary["required_for_release"]
    ]
    report = {
        "schema_version": 1,
        "purpose": "matrix15 stochastic interception release evaluation",
        "limitations": [
            "Point-mass dynamics and idealized first-order body-rate response are not a flight approval.",
            "Noise models are configured surrogates, not a fitted YOLO/ByteTrack error distribution.",
            "candidate_velocity_hold_variable_thrust is offline-only and never emits Betaflight RC/PWM.",
            "The candidate uses the production LOS filter and delayed/noisy own velocity, but its noise model is not fitted to flight data.",
        ],
        "source_config": str(Path(args.config).expanduser().resolve()),
        "base_seed": base_seed,
        "trials_per_case": trials_per_case,
        "worker_count": args.workers,
        "case_count": len(MATRIX15_CASES),
        "row_count": len(rows),
        "simulation": base_simulation,
        "acceptance": criteria.to_dict(),
        "scenarios": scenarios,
        "evaluations": evaluations,
        "summaries": summaries,
        "required_summary_count": len(required_summaries),
        "release_passed": bool(required_summaries)
        and all(bool(summary["passed"]) for summary in required_summaries),
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
) -> list[dict[str, object]]:
    tasks = []
    for scenario in scenarios:
        scenario_name = str(scenario["name"])
        scenario_seed = zlib.crc32(scenario_name.encode("utf-8")) & 0xFFFFFFFF
        scenario_values = {
            key: value for key, value in scenario.items() if key != "name"
        }
        simulation = _simulation_values({**base_simulation, **scenario_values})
        for evaluation in evaluations:
            evaluation_name = str(evaluation["name"])
            mode = str(evaluation.get("controller_mode", ""))
            start = str(evaluation.get("start_profile", ""))
            if mode not in CONTROLLER_MODES:
                raise ValueError(f"unsupported controller_mode for {evaluation_name}: {mode}")
            if start not in START_PROFILES:
                raise ValueError(f"unsupported start_profile for {evaluation_name}: {start}")
            for trial_index in range(trials_per_case):
                trial_seed = (base_seed + scenario_seed + trial_index) & 0xFFFFFFFF
                trial_simulation = {**simulation, "random_seed": trial_seed}
                for case in MATRIX15_CASES:
                    tasks.append(
                        {
                            "scenario_name": scenario_name,
                            "evaluation_name": evaluation_name,
                            "controller_mode": mode,
                            "start_profile": start,
                            "required_for_release": bool(
                                evaluation.get("required_for_release", False)
                            ),
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
    if int(data.get("schema_version", 0)) != 1:
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
