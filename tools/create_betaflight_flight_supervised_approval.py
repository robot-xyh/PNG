#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from create_betaflight_noprop_approval import (  # noqa: E402
    _configured_override_mode_cli_id,
    _read_json,
    _sha256,
    _validate_camera_extrinsic,
    _validate_guidance_command_frames,
    _validate_rate_profile,
    _validate_raw_imu_gyro_binding,
    _validate_snapshot,
)
from vision_guidance.betaflight_intercept_controller import (  # noqa: E402
    VelocityEstablishingPngConfig,
)
from vision_guidance.betaflight_runtime import MspRuntimeConfig  # noqa: E402
from vision_guidance.betaflight_sitl import (  # noqa: E402
    SITL_OFFICIAL_BETAFLIGHT_COMMIT,
    SITL_OFFICIAL_BETAFLIGHT_ELF_SHA256,
    validate_sitl_audit_evidence,
)
from vision_guidance.flight_control import (  # noqa: E402
    AccelerationTiltRateConfig,
    RcMappingConfig,
)
from vision_guidance.thrust_model import VoltageThrottleThrustModel  # noqa: E402


NONCOLLISION_SCOPE = "flight_noncollision_short_supervised_v3"
CONTACT_SCOPE = "flight_contact_short_supervised_v2"
ACTIVE_SCOPES = frozenset({NONCOLLISION_SCOPE, CONTACT_SCOPE})
OVERRIDE_CHANNELS_MASK = 15
MAX_RATE_DEG_S = 60.0
MAX_TILT_DEG = 35.0
MAX_GUIDANCE_ACCEL_MPS2 = 7.0
MAX_TAKEOVER_DURATION_S = 0.9
THROTTLE_MIN_US = 1200
THROTTLE_HOVER_US = 1275
THROTTLE_MAX_US = 1500
THROTTLE_SLEW_US_PER_S = 600.0
MIN_VBAT_V = 22.0
MIN_GPS_SATELLITES = 6
REQUIRED_THRUST_VOLTAGE_MIN_V = 22.0
REQUIRED_THRUST_VOLTAGE_MAX_V = 25.2
MIN_THRUST_VALIDATION_SAMPLES = 100
MIN_FINALIZED_RUN_ROWS = 100
MIN_FINALIZED_EVIDENCE_FRAMES = 25
RELEASE_HIT_RATE_MIN = 0.80
RELEASE_FOV_HIT_RATE_MIN = 0.80
RELEASE_TRIALS_PER_CASE_MIN = 100
RELEASE_CASE_COUNT_MIN = 30
RELEASE_CONTACT_ROW_COUNT_MIN = 18000
RELEASE_NONCOLLISION_ROW_COUNT_MIN = 27000
RELEASE_NONCOLLISION_TIMELY_ABORT_RATE_MIN = 0.99
RELEASE_NONCOLLISION_UNSAFE_CONTACT_RATE_MAX = 0.01
RELEASE_NONCOLLISION_ABORT_LEAD_TIME_S = 0.75
RELEASE_SPEED_SATURATION_FRACTION_MAX = 0.40
RELEASE_TOTAL_SATURATION_FRACTION_MAX = 0.40
RELEASE_REQUIRED_SCENARIOS = {
    "final_chain_software_p95",
    "observed_active_flight_p95",
    "conservative_physical_p95_budget",
}
RELEASE_SOURCE_PATHS = {
    "controller": ROOT / "vision_guidance" / "betaflight_intercept_controller.py",
    "flight_control": ROOT / "vision_guidance" / "flight_control.py",
    "simulation": ROOT / "vision_guidance" / "betaflight_png_sim.py",
    "thrust_model": ROOT / "vision_guidance" / "thrust_model.py",
    "monte_carlo_runner": ROOT / "tools" / "run_betaflight_intercept_monte_carlo.py",
}
EXPECTED_POLL_HZ = {
    "status_poll_hz": 5.0,
    "attitude_poll_hz": 20.0,
    "raw_imu_poll_hz": 5.0,
    "raw_gps_poll_hz": 5.0,
    "altitude_poll_hz": 5.0,
    "motor_poll_hz": 0.0,
    "rc_poll_hz": 5.0,
    "analog_poll_hz": 1.0,
}
NOPROP_TIMING_EXPECTED_POLL_HZ = {
    **EXPECTED_POLL_HZ,
    "motor_poll_hz": 2.0,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a hash-bound approval for supervised velocity-PNG flight."
    )
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--release-evidence", required=True)
    parser.add_argument(
        "--rc-interlock-evidence",
        default="",
        help="Optional legacy EdgeTX/ANGLE release evidence; not required by short-supervised scopes.",
    )
    parser.add_argument("--finalized-run-evidence", required=True)
    parser.add_argument("--noprop-timing-evidence", required=True)
    parser.add_argument(
        "--sitl-evidence",
        action="append",
        required=True,
        help=(
            "Passing policy-specific Gazebo SIL audit. Pass once for projected and once "
            "for rendered detector mode. SIL never grants hardware authorization."
        ),
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--acknowledge-supervised-flight", action="store_true")
    parser.add_argument(
        "--acknowledge-no-automatic-msp-fallback",
        action="store_true",
        help=(
            "Acknowledge that Orange Pi/process/UART failure may hold the last MSP "
            "frame until the pilot lowers RC7."
        ),
    )
    parser.add_argument("--acknowledge-intentional-contact", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.acknowledge_supervised_flight:
        raise RuntimeError("--acknowledge-supervised-flight is required")
    if not args.acknowledge_no_automatic_msp_fallback:
        raise RuntimeError("--acknowledge-no-automatic-msp-fallback is required")

    snapshot_path = Path(args.snapshot).expanduser().resolve()
    config_path = Path(args.config).expanduser().resolve()
    release_evidence_path = Path(args.release_evidence).expanduser().resolve()
    rc_interlock_path = (
        None
        if not str(args.rc_interlock_evidence).strip()
        else Path(args.rc_interlock_evidence).expanduser().resolve()
    )
    finalized_run_path = Path(args.finalized_run_evidence).expanduser().resolve()
    noprop_timing_path = Path(args.noprop_timing_evidence).expanduser().resolve()
    sitl_evidence_paths = [
        Path(value).expanduser().resolve() for value in args.sitl_evidence
    ]
    output_path = Path(args.output).expanduser().resolve()
    snapshot = _read_json(snapshot_path)
    config = _read_json(config_path)
    config_sha256 = _sha256(config_path)
    scope = _configured_scope(config)
    if scope == CONTACT_SCOPE and not args.acknowledge_intentional_contact:
        raise RuntimeError("--acknowledge-intentional-contact is required")
    rc_interlock_evidence = (
        None
        if rc_interlock_path is None
        else validate_rc_interlock_evidence(
            _read_json(rc_interlock_path),
            rc_interlock_path,
            runtime_config_sha256=config_sha256,
        )
    )
    finalized_run_evidence = validate_finalized_run_evidence(
        _read_json(finalized_run_path),
        finalized_run_path,
        runtime_config_sha256=config_sha256,
    )
    noprop_timing_evidence = validate_noprop_timing_evidence(
        _read_json(noprop_timing_path),
        noprop_timing_path,
    )
    repository_commit, repository_dirty = _repository_state()
    if repository_dirty:
        raise RuntimeError("supervised approval requires a clean Git worktree")
    if finalized_run_evidence["repository_commit"] != repository_commit:
        raise RuntimeError(
            "finalized LOG_ONLY evidence commit does not match the approval build"
        )
    if noprop_timing_evidence["repository_commit"] != repository_commit:
        raise RuntimeError(
            "no-prop timing evidence commit does not match the approval build"
        )
    expected_engagement_policy = "contact" if scope == CONTACT_SCOPE else "noncollision"
    sitl_evidence = validate_sitl_evidence(
        [(_read_json(path), path) for path in sitl_evidence_paths],
        runtime_config_sha256=config_sha256,
        expected_engagement_policy=expected_engagement_policy,
        repository_commit=repository_commit,
    )
    parsed_cli = _validate_snapshot(
        snapshot,
        snapshot_path,
        expected_override_mode_cli_id=_configured_override_mode_cli_id(config),
        expected_override_channels_mask=OVERRIDE_CHANNELS_MASK,
    )
    gps_evidence = validate_snapshot_flight_state(snapshot, snapshot_path)
    evidence = validate_flight_supervised_config(
        config,
        output_path=output_path,
        parsed_cli=parsed_cli,
        fc_identity=dict(snapshot["fc_identity"]),
        config_path=config_path,
    )
    release_evidence = validate_release_evidence(
        _read_json(release_evidence_path),
        release_evidence_path,
        runtime_config_sha256=config_sha256,
        runtime_thrust_model=evidence["guidance_command"]["thrust_model"],
        expected_engagement_policy=expected_engagement_policy,
    )
    approval = {
        "schema_version": 5,
        "approved": True,
        "scope": scope,
        "created_unix_s": time.time(),
        "created_local": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "operator": str(args.operator),
        "operator_acknowledgement": (
            "professional pilot, short supervised flight, immediate RC7 release "
            "on anomaly"
        ),
        "manual_msp_loss_waiver": {
            "acknowledged": True,
            "scope": scope,
            "parameters_sha256": config_sha256,
            "risk": (
                "Orange Pi, process, or UART failure may hold the last MSP frame "
                "until the pilot lowers RC7"
            ),
        },
        "source_conflicts_resolved": True,
        "snapshot_manifest": str(snapshot_path),
        "snapshot_sha256": _sha256(snapshot_path),
        "expected_fc_identity": dict(snapshot["fc_identity"]),
        "parameters_path": str(config_path),
        "parameters_sha256": config_sha256,
        "release_evidence": release_evidence,
        "finalized_run_evidence": finalized_run_evidence,
        "noprop_timing_evidence": noprop_timing_evidence,
        "sitl_evidence": sitl_evidence,
        "software_binding": {
            "repository_commit": repository_commit,
            "repository_dirty": False,
        },
        "limits": {
            "override_channels_mask": OVERRIDE_CHANNELS_MASK,
            "operator_target_takeover_duration_s": 0.5,
            "actual_algorithm_publication_limit_s": MAX_TAKEOVER_DURATION_S,
            "duration_interlock_enabled": True,
            "max_takeovers_per_arm": 1,
            "roll_pitch_rate_deg_s": MAX_RATE_DEG_S,
            "tilt_envelope_deg": MAX_TILT_DEG,
            "total_guidance_accel_mps2": MAX_GUIDANCE_ACCEL_MPS2,
            "throttle_us": [THROTTLE_MIN_US, THROTTLE_HOVER_US, THROTTLE_MAX_US],
            "throttle_slew_us_per_s": THROTTLE_SLEW_US_PER_S,
            "minimum_vbat_v": MIN_VBAT_V,
            "minimum_gps_satellites": MIN_GPS_SATELLITES,
            "acro_rate_mode_required": True,
        },
        "snapshot_flight_state": gps_evidence,
        "thrust_model_evidence": evidence["guidance_command"]["thrust_model"],
        **evidence,
    }
    if scope == CONTACT_SCOPE:
        approval["intentional_contact_acknowledgement"] = {
            "acknowledged": True,
            "scope": scope,
            "parameters_sha256": config_sha256,
        }
    if rc_interlock_evidence is not None:
        approval["rc_interlock_evidence"] = rc_interlock_evidence
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(approval, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output_path)
    print(f"approval_sha256={_sha256(output_path)}")


def validate_sitl_evidence(
    reports: list[tuple[dict[str, Any], Path]],
    *,
    runtime_config_sha256: str,
    expected_engagement_policy: str,
    repository_commit: str,
) -> list[dict[str, Any]]:
    return validate_sitl_audit_evidence(
        reports,
        runtime_config_sha256=runtime_config_sha256,
        expected_engagement_policy=expected_engagement_policy,
        repository_commit=repository_commit,
        official_betaflight_commit=SITL_OFFICIAL_BETAFLIGHT_COMMIT,
        official_betaflight_elf_sha256=SITL_OFFICIAL_BETAFLIGHT_ELF_SHA256,
    )


def validate_release_evidence(
    report: dict[str, Any],
    report_path: Path,
    *,
    runtime_config_sha256: str,
    runtime_thrust_model: dict[str, Any],
    expected_engagement_policy: str = "noncollision",
) -> dict[str, Any]:
    if expected_engagement_policy not in {"noncollision", "contact"}:
        raise RuntimeError("unsupported release engagement policy")
    if report.get("schema_version") != 3:
        raise RuntimeError("release evidence schema_version must be 3")
    if report.get("purpose") != "stochastic interception release evaluation":
        raise RuntimeError("release evidence purpose mismatch")
    if report.get("release_passed") is not True:
        raise RuntimeError("release evidence did not pass")

    runtime_binding = _release_mapping(report.get("runtime_binding"), "runtime_binding")
    if runtime_binding.get("sha256") != runtime_config_sha256:
        raise RuntimeError("release evidence runtime config SHA256 mismatch")
    _validate_release_source_bindings(report)
    _validate_release_thrust_binding(
        report,
        runtime_thrust_model=runtime_thrust_model,
    )
    acceptance = _release_mapping(report.get("acceptance"), "acceptance")
    try:
        hit_rate_min = float(acceptance["initially_visible_hit_rate_min"])
        fov_hit_rate_min = float(acceptance["initially_visible_fov_hit_rate_min"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("release evidence acceptance is invalid") from exc
    if (
        hit_rate_min != RELEASE_HIT_RATE_MIN
        or fov_hit_rate_min != RELEASE_FOV_HIT_RATE_MIN
        or float(
            acceptance.get(
                "mean_speed_hold_accel_saturation_fraction_max", math.nan
            )
        )
        != RELEASE_SPEED_SATURATION_FRACTION_MAX
        or float(
            acceptance.get("mean_total_accel_saturation_fraction_max", math.nan)
        )
        != RELEASE_TOTAL_SATURATION_FRACTION_MAX
        or acceptance.get("worst_minimum_range_m_max", "missing") is not None
    ):
        raise RuntimeError("release evidence must use the formal 80% probabilistic policy")
    paired = _release_mapping(report.get("paired_screening"), "paired_screening")
    if paired.get("passed") is not True:
        raise RuntimeError("release evidence paired screening did not pass")
    try:
        trials_per_case = int(report["trials_per_case"])
        case_count = int(report["case_count"])
        row_count = int(report["row_count"])
        required_summary_count = int(report["required_summary_count"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("release evidence Monte Carlo coverage is invalid") from exc
    minimum_row_count = (
        RELEASE_NONCOLLISION_ROW_COUNT_MIN
        if expected_engagement_policy == "noncollision"
        else RELEASE_CONTACT_ROW_COUNT_MIN
    )
    if (
        trials_per_case < RELEASE_TRIALS_PER_CASE_MIN
        or case_count < RELEASE_CASE_COUNT_MIN
        or row_count < minimum_row_count
    ):
        raise RuntimeError("release evidence Monte Carlo coverage is insufficient")

    raw_summaries = report.get("summaries")
    if not isinstance(raw_summaries, list) or any(
        not isinstance(summary, dict) for summary in raw_summaries
    ):
        raise RuntimeError("release evidence summaries must be a list of objects")
    required_summaries = [
        summary
        for summary in raw_summaries
        if summary.get("required_for_release") is True
    ]
    summaries = [
        summary
        for summary in required_summaries
        if summary.get("evidence_role") == "contact_performance"
    ]
    noncollision_summaries = [
        summary
        for summary in required_summaries
        if summary.get("evidence_role") == "noncollision_safety"
    ]
    scenario_names = {summary.get("scenario_name") for summary in summaries}
    selected_evaluation = paired.get("selected_evaluation")
    expected_noncollision_count = (
        len(RELEASE_REQUIRED_SCENARIOS)
        if expected_engagement_policy == "noncollision"
        else 0
    )
    if (
        len(summaries) != len(RELEASE_REQUIRED_SCENARIOS)
        or len(noncollision_summaries) != expected_noncollision_count
        or len(required_summaries) != required_summary_count
        or scenario_names != RELEASE_REQUIRED_SCENARIOS
        or (
            expected_engagement_policy == "noncollision"
            and {
                summary.get("scenario_name") for summary in noncollision_summaries
            }
            != RELEASE_REQUIRED_SCENARIOS
        )
        or not isinstance(selected_evaluation, str)
        or not selected_evaluation
        or any(
            summary.get("evaluation_name") != selected_evaluation
            for summary in summaries
        )
    ):
        raise RuntimeError("release evidence required scenario coverage is incomplete")
    for summary in summaries:
        if (
            summary.get("passed") is not True
            or summary.get("engagement_policy") != "contact"
        ):
            raise RuntimeError("release evidence contains a failed required scenario")
        try:
            hit_rate = float(summary["initially_visible_hit_rate"])
            fov_hit_rate = float(summary["initially_visible_fov_hit_rate"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("release evidence scenario rates are invalid") from exc
        if hit_rate < RELEASE_HIT_RATE_MIN or fov_hit_rate < RELEASE_FOV_HIT_RATE_MIN:
            raise RuntimeError("release evidence scenario is below the formal 80% policy")
        checks = _release_mapping(summary.get("checks"), "summary checks")
        range_check = _release_mapping(
            checks.get("worst_minimum_range_m"), "worst range check"
        )
        if (
            range_check.get("operator") != "report_only"
            or range_check.get("threshold") is not None
            or range_check.get("required") is not False
        ):
            raise RuntimeError("release evidence worst range must remain report-only")
    _validate_release_policy_results(
        report,
        selected_evaluation=selected_evaluation,
        expected_engagement_policy=expected_engagement_policy,
    )
    return {
        "path": str(report_path),
        "sha256": _sha256(report_path),
        "schema_version": 3,
        "runtime_config_sha256": runtime_config_sha256,
        "formal_hit_rate_min": RELEASE_HIT_RATE_MIN,
        "formal_fov_hit_rate_min": RELEASE_FOV_HIT_RATE_MIN,
        "trials_per_case": trials_per_case,
        "case_count": case_count,
        "row_count": row_count,
        "required_scenario_count": len(summaries),
        "required_noncollision_scenario_count": len(noncollision_summaries),
        "source_bindings": dict(report["source_bindings"]),
        "thrust_model_binding": dict(report["thrust_model_binding"]),
    }


def _validate_release_source_bindings(report: dict[str, Any]) -> None:
    bindings = _release_mapping(report.get("source_bindings"), "source_bindings")
    if set(bindings) != set(RELEASE_SOURCE_PATHS):
        raise RuntimeError("release evidence source binding set is incomplete")
    for name, expected_path in RELEASE_SOURCE_PATHS.items():
        binding = _release_mapping(bindings.get(name), f"source binding {name}")
        expected_repository_path = str(expected_path.resolve().relative_to(ROOT))
        if binding.get("repository_path") != expected_repository_path:
            raise RuntimeError(f"release evidence {name} source path mismatch")
        if binding.get("sha256") != _sha256(expected_path.resolve()):
            raise RuntimeError(f"release evidence {name} source SHA256 mismatch")


def _validate_release_thrust_binding(
    report: dict[str, Any],
    *,
    runtime_thrust_model: dict[str, Any],
) -> None:
    binding = _release_mapping(
        report.get("thrust_model_binding"), "thrust_model_binding"
    )
    runtime_binding = _release_mapping(report.get("runtime_binding"), "runtime_binding")
    nested = _release_mapping(
        runtime_binding.get("thrust_model"), "runtime thrust_model"
    )
    for key in ("sha256", "calibration_id"):
        expected = runtime_thrust_model.get(key)
        if binding.get(key) != expected or nested.get(key) != expected:
            raise RuntimeError(f"release evidence thrust LUT {key} mismatch")
    for key in ("voltage_coverage_v", "throttle_coverage_us"):
        expected = runtime_thrust_model.get(key)
        if binding.get(key) != expected or nested.get(key) != expected:
            raise RuntimeError(f"release evidence thrust LUT {key} mismatch")
    simulation = _release_mapping(report.get("simulation"), "simulation")
    if (
        simulation.get("thrust_model_sha256") != runtime_thrust_model.get("sha256")
        or simulation.get("thrust_model_calibration_id")
        != runtime_thrust_model.get("calibration_id")
    ):
        raise RuntimeError("release simulation thrust LUT binding mismatch")
    voltage_coverage = runtime_thrust_model.get("voltage_coverage_v")
    if not isinstance(voltage_coverage, list) or len(voltage_coverage) != 2:
        raise RuntimeError("runtime thrust LUT voltage coverage is invalid")
    minimum_voltage = float(voltage_coverage[0])
    maximum_voltage = float(voltage_coverage[1])
    scenarios = report.get("scenarios")
    if not isinstance(scenarios, list) or not scenarios:
        raise RuntimeError("release evidence scenarios are missing")
    try:
        base_voltage = float(simulation["battery_voltage_v"])
        scenario_voltages = [
            float(scenario.get("battery_voltage_v", base_voltage))
            for scenario in scenarios
            if isinstance(scenario, dict)
        ]
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("release simulation battery voltage is invalid") from exc
    if len(scenario_voltages) != len(scenarios) or any(
        not minimum_voltage <= voltage <= maximum_voltage
        for voltage in scenario_voltages
    ):
        raise RuntimeError("release simulation voltage is outside thrust LUT coverage")
    if min(scenario_voltages) > REQUIRED_THRUST_VOLTAGE_MIN_V or max(
        scenario_voltages
    ) < REQUIRED_THRUST_VOLTAGE_MAX_V:
        raise RuntimeError("release simulation must exercise the 22.0-25.2 V endpoints")


def _validate_release_policy_results(
    report: dict[str, Any],
    *,
    selected_evaluation: str,
    expected_engagement_policy: str,
) -> None:
    policy = _release_mapping(report.get("policy_results"), "policy_results")
    if (
        policy.get("passed") is not True
        or policy.get("runtime_engagement_policy") != expected_engagement_policy
    ):
        raise RuntimeError("release evidence policy separation did not pass")
    contact = _release_mapping(
        policy.get("contact_performance"), "contact performance"
    )
    if (
        contact.get("passed") is not True
        or contact.get("engagement_policy") != "contact"
        or contact.get("evaluation_name") != selected_evaluation
        or contact.get("authorizes_contact_flight") is not False
        or set(contact.get("scenario_names", [])) != RELEASE_REQUIRED_SCENARIOS
    ):
        raise RuntimeError("release contact-performance evidence is invalid")
    if expected_engagement_policy == "contact":
        if policy.get("contact_mc_is_performance_evidence_only") is not True:
            raise RuntimeError("contact release must remain performance evidence only")
        if policy.get("noncollision_safety") is not None:
            raise RuntimeError("contact release must not reuse non-collision authority")
        return
    if policy.get("contact_evidence_is_not_noncollision_flight_authority") is not True:
        raise RuntimeError("release evidence policy separation did not pass")
    noncollision = _release_mapping(
        policy.get("noncollision_safety"), "noncollision safety"
    )
    acceptance = _release_mapping(
        noncollision.get("acceptance"), "noncollision acceptance"
    )
    if (
        noncollision.get("passed") is not True
        or noncollision.get("engagement_policy") != "noncollision"
        or noncollision.get("requires_pilot_action_after_abort") is not True
        or float(acceptance.get("timely_abort_rate_min", math.nan))
        != RELEASE_NONCOLLISION_TIMELY_ABORT_RATE_MIN
        or float(acceptance.get("unsafe_contact_rate_max", math.nan))
        != RELEASE_NONCOLLISION_UNSAFE_CONTACT_RATE_MAX
        or float(acceptance.get("minimum_abort_lead_time_s", math.nan))
        != RELEASE_NONCOLLISION_ABORT_LEAD_TIME_S
    ):
        raise RuntimeError("release noncollision policy or acceptance is invalid")
    summaries = noncollision.get("summaries")
    if not isinstance(summaries, list) or any(
        not isinstance(summary, dict) for summary in summaries
    ):
        raise RuntimeError("release noncollision summaries are invalid")
    if {
        summary.get("scenario_name") for summary in summaries
    } != RELEASE_REQUIRED_SCENARIOS:
        raise RuntimeError("release noncollision scenario coverage is incomplete")
    for summary in summaries:
        if (
            summary.get("passed") is not True
            or summary.get("engagement_policy") != "noncollision"
            or float(summary.get("timely_abort_rate", -1.0))
            < RELEASE_NONCOLLISION_TIMELY_ABORT_RATE_MIN
            or float(summary.get("unsafe_contact_rate", math.inf))
            > RELEASE_NONCOLLISION_UNSAFE_CONTACT_RATE_MAX
        ):
            raise RuntimeError("release noncollision scenario failed")


def validate_rc_interlock_evidence(
    report: dict[str, Any],
    report_path: Path,
    *,
    runtime_config_sha256: str,
) -> dict[str, Any]:
    if report.get("schema_version") != 1 or report.get("passed") is not True:
        raise RuntimeError("RC interlock evidence must be a passing schema v1 report")
    binding = _release_mapping(report.get("runtime_binding"), "runtime_binding")
    if binding.get("sha256") != runtime_config_sha256:
        raise RuntimeError("RC interlock evidence runtime config SHA256 mismatch")
    checks = _release_mapping(report.get("checks"), "checks")
    required = (
        "override_seen",
        "release_mode_seen",
        "rc7_low_seen",
        "override_cleared",
    )
    if any(checks.get(name) is not True for name in required):
        raise RuntimeError("RC interlock evidence is incomplete")
    latency_ms = float(report.get("max_release_latency_ms", math.inf))
    if not math.isfinite(latency_ms) or latency_ms > 200.0:
        raise RuntimeError("RC interlock release latency exceeds 200 ms")
    return {
        "path": str(report_path),
        "sha256": _sha256(report_path),
        "schema_version": 1,
        "max_release_latency_ms": latency_ms,
        "checks": {name: True for name in required},
    }


def validate_noprop_timing_evidence(
    report: dict[str, Any],
    report_path: Path,
) -> dict[str, Any]:
    if (
        report.get("audit_schema_version") != 1
        or report.get("passed") is not True
        or report.get("violations") != []
    ):
        raise RuntimeError("no-prop timing evidence must be a passing schema v1 audit")
    bindings = _release_mapping(report.get("source_bindings"), "timing source bindings")
    if set(bindings) != {"csv", "meta"}:
        raise RuntimeError("no-prop timing evidence source bindings are incomplete")
    resolved_paths: dict[str, Path] = {}
    for name in ("csv", "meta"):
        binding = _release_mapping(bindings.get(name), f"timing {name} binding")
        path = Path(str(binding.get("path", ""))).expanduser().resolve()
        if not path.is_file() or _sha256(path) != str(binding.get("sha256", "")):
            raise RuntimeError(f"no-prop timing {name} evidence changed or is missing")
        resolved_paths[name] = path

    meta = _read_json(resolved_paths["meta"])
    commit = str(meta.get("repository_commit", ""))
    config = _release_mapping(meta.get("config"), "timing source config")
    bench = _release_mapping(config.get("bench_profile"), "timing bench profile")
    logging = _release_mapping(config.get("logging"), "timing logging")
    frames = _release_mapping(logging.get("evidence_frames"), "timing evidence frames")
    runtime = _release_mapping(config.get("msp_runtime"), "timing MSP runtime")
    if (
        len(commit) != 40
        or meta.get("repository_dirty") is not False
        or bench.get("scope") != "noprop_bench"
        or meta.get("allow_control") is not True
        or meta.get("control_mode") != "msp_raw_rc"
        or frames.get("enabled") is not True
        or float(runtime.get("control_publish_hz", 0.0)) != 50.0
    ):
        raise RuntimeError(
            "no-prop timing evidence must use clean, active noprop_bench code with JPEG recording"
        )
    for key, expected in NOPROP_TIMING_EXPECTED_POLL_HZ.items():
        try:
            actual = float(runtime[key])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"no-prop timing msp_runtime.{key} is missing") from exc
        if not math.isclose(actual, expected, abs_tol=1.0e-9):
            raise RuntimeError(
                f"no-prop timing msp_runtime.{key} must be exactly {expected:g} Hz"
            )

    metrics = _release_mapping(report.get("metrics"), "timing metrics")
    try:
        write_rate_hz = float(metrics["set_raw_rc_write_rate_hz"])
        p999_interval_s = float(metrics["set_raw_rc_write_p999_interval_s"])
        maximum_gap_s = float(metrics["max_send_gap_s"])
        write_count = int(metrics["set_raw_rc_write_success_count"])
        evidence_frame_count = int(metrics["evidence_frame_write_count"])
        evidence_frame_errors = int(metrics["evidence_frame_error_count"])
        transport_errors = sum(
            int(metrics[name])
            for name in (
                "set_raw_rc_error_count",
                "set_raw_rc_write_error_count",
                "msp_rx_checksum_error_count",
                "msp_rx_parser_error_count",
            )
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("no-prop timing metrics are incomplete") from exc
    if (
        write_rate_hz < 49.0
        or p999_interval_s > 0.040
        or maximum_gap_s > 0.060
        or write_count < 100
        or evidence_frame_count < MIN_FINALIZED_EVIDENCE_FRAMES
        or evidence_frame_errors != 0
        or transport_errors != 0
    ):
        raise RuntimeError("no-prop timing evidence does not satisfy the 50 Hz contract")
    return {
        "path": str(report_path),
        "sha256": _sha256(report_path),
        "audit_schema_version": 1,
        "repository_commit": commit,
        "set_raw_rc_write_rate_hz": write_rate_hz,
        "set_raw_rc_write_p999_interval_s": p999_interval_s,
        "max_send_gap_s": maximum_gap_s,
        "set_raw_rc_write_success_count": write_count,
        "evidence_frame_count": evidence_frame_count,
        "source_bindings": dict(bindings),
    }


def validate_finalized_run_evidence(
    manifest: dict[str, Any],
    manifest_path: Path,
    *,
    runtime_config_sha256: str,
) -> dict[str, Any]:
    if manifest.get("schema_version") != 2 or manifest.get("finalized") is not True:
        raise RuntimeError("run evidence must be a finalized schema v2 manifest")
    completion = _release_mapping(manifest.get("completion"), "run completion")
    if completion.get("complete") is not True:
        raise RuntimeError("finalized run evidence is incomplete")
    if manifest.get("missing_runtime_artifacts"):
        raise RuntimeError("finalized run evidence has missing runtime artifacts")
    pairing = _release_mapping(manifest.get("pairing"), "run pairing")
    if pairing.get("confidence") != "unique":
        raise RuntimeError("finalized run evidence must have unique pairing")
    visual = _release_mapping(manifest.get("visual_evidence"), "visual evidence")
    if (
        visual.get("enabled") is not True
        or int(visual.get("frame_count", 0)) < MIN_FINALIZED_EVIDENCE_FRAMES
    ):
        raise RuntimeError(
            f"finalized run evidence requires at least {MIN_FINALIZED_EVIDENCE_FRAMES} frames"
        )
    external = _release_mapping(
        manifest.get("external_artifacts"), "external artifacts"
    )
    if "blackbox" not in external:
        raise RuntimeError("finalized run evidence requires a paired Blackbox artifact")
    blackbox = _release_mapping(
        manifest.get("blackbox_interpretation"), "Blackbox interpretation"
    )
    if (
        blackbox.get("authoritative_mode_source") != "host_msp_status_box_ids"
        or blackbox.get("decoder_labels_used_for_mode_decisions") is not False
    ):
        raise RuntimeError("finalized run evidence has an unsafe Blackbox mode interpretation")

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, list):
        raise RuntimeError("finalized run evidence artifacts are invalid")
    meta_path = _single_artifact_path(artifacts, "_meta.json")
    csv_path = _single_artifact_path(artifacts, ".csv")
    meta = _read_json(meta_path)
    if meta.get("config_sha256") != runtime_config_sha256:
        raise RuntimeError("finalized run evidence runtime config SHA256 mismatch")
    if meta.get("allow_control") is not False or meta.get("control_mode") != "log_only":
        raise RuntimeError("finalized run evidence must come from LOG_ONLY")
    commit = str(meta.get("repository_commit", ""))
    if len(commit) != 40 or meta.get("repository_dirty") is not False:
        raise RuntimeError("finalized run evidence requires a clean recorded Git commit")
    source_files = meta.get("source_files")
    if not isinstance(source_files, list) or not source_files:
        raise RuntimeError("finalized run evidence source hashes are missing")
    if any(
        not isinstance(entry, dict)
        or len(str(entry.get("sha256", ""))) != 64
        or not entry.get("path")
        for entry in source_files
    ):
        raise RuntimeError("finalized run evidence source hashes are invalid")

    row_count = 0
    valid_flight_state_rows = 0
    maximum_set_raw_rc_attempts = 0
    maximum_evidence_errors = 0
    maximum_evidence_writes = 0
    with csv_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required_fields = {
            "msp_set_raw_rc_attempt_count",
            "evidence_frame_write_count",
            "evidence_frame_error_count",
            "kinematics_valid",
            "gps_fix",
            "gps_satellites",
            "vbat_v",
        }
        missing = sorted(required_fields - set(reader.fieldnames or ()))
        if missing:
            raise RuntimeError(
                "finalized run CSV is missing fields: " + ", ".join(missing)
            )
        for row in reader:
            row_count += 1
            maximum_set_raw_rc_attempts = max(
                maximum_set_raw_rc_attempts,
                _csv_int(row, "msp_set_raw_rc_attempt_count"),
            )
            maximum_evidence_writes = max(
                maximum_evidence_writes,
                _csv_int(row, "evidence_frame_write_count"),
            )
            maximum_evidence_errors = max(
                maximum_evidence_errors,
                _csv_int(row, "evidence_frame_error_count"),
            )
            if (
                _csv_int(row, "kinematics_valid") == 1
                and _csv_int(row, "gps_fix") >= 1
                and _csv_int(row, "gps_satellites") >= MIN_GPS_SATELLITES
                and MIN_VBAT_V
                <= _csv_float(row, "vbat_v")
                <= REQUIRED_THRUST_VOLTAGE_MAX_V
            ):
                valid_flight_state_rows += 1
    if row_count < MIN_FINALIZED_RUN_ROWS:
        raise RuntimeError("finalized run evidence is too short")
    if maximum_set_raw_rc_attempts != 0:
        raise RuntimeError("finalized LOG_ONLY evidence attempted MSP_SET_RAW_RC")
    if maximum_evidence_errors != 0:
        raise RuntimeError("finalized run evidence contains frame recording errors")
    if maximum_evidence_writes < MIN_FINALIZED_EVIDENCE_FRAMES:
        raise RuntimeError("finalized run CSV does not confirm enough evidence frames")
    if valid_flight_state_rows < 3:
        raise RuntimeError("finalized run evidence lacks valid GPS/voltage/kinematics samples")
    return {
        "path": str(manifest_path),
        "sha256": _sha256(manifest_path),
        "schema_version": 2,
        "runtime_config_sha256": runtime_config_sha256,
        "row_count": row_count,
        "evidence_frame_count": int(visual["frame_count"]),
        "valid_flight_state_rows": valid_flight_state_rows,
        "set_raw_rc_attempt_count": maximum_set_raw_rc_attempts,
        "pairing_confidence": "unique",
        "repository_commit": commit,
    }


def _single_artifact_path(artifacts: list[Any], suffix: str) -> Path:
    matches = [
        entry
        for entry in artifacts
        if isinstance(entry, dict) and str(entry.get("path", "")).endswith(suffix)
    ]
    if len(matches) != 1:
        raise RuntimeError(f"finalized run evidence requires exactly one {suffix} artifact")
    path = Path(str(matches[0]["path"])).expanduser().resolve()
    if not path.is_file() or _sha256(path) != matches[0].get("sha256"):
        raise RuntimeError(f"finalized run artifact changed or is missing: {path}")
    return path


def _csv_int(row: dict[str, str], field: str) -> int:
    try:
        return int(float(row[field]))
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"finalized run CSV field {field} is invalid") from exc


def _csv_float(row: dict[str, str], field: str) -> float:
    try:
        value = float(row[field])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"finalized run CSV field {field} is invalid") from exc
    if not math.isfinite(value):
        raise RuntimeError(f"finalized run CSV field {field} is non-finite")
    return value


def _release_mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise RuntimeError(f"release evidence {name} must be an object")
    return value


def validate_snapshot_flight_state(
    snapshot: dict[str, Any],
    snapshot_path: Path,
) -> dict[str, Any]:
    capture = dict(snapshot.get("capture", {}))
    if capture.get("include_kinematics") is not True:
        raise RuntimeError("fresh flight snapshot requires --include-kinematics")
    telemetry_name = "telemetry.csv"
    telemetry_path = snapshot_path.parent / telemetry_name
    expected_hash = str(snapshot.get("artifacts", {}).get(telemetry_name, ""))
    if not telemetry_path.is_file() or not expected_hash or _sha256(telemetry_path) != expected_hash:
        raise RuntimeError("snapshot telemetry.csv is missing or changed")

    valid_rows: list[tuple[int, int, float]] = []
    with telemetry_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            try:
                fix = int(row.get("gps_fix", ""))
                satellites = int(row.get("gps_satellites", ""))
                voltage = float(row.get("vbat_v", ""))
            except (TypeError, ValueError):
                continue
            if (
                fix >= 1
                and satellites >= MIN_GPS_SATELLITES
                and MIN_VBAT_V <= voltage <= REQUIRED_THRUST_VOLTAGE_MAX_V
            ):
                valid_rows.append((fix, satellites, voltage))
    if len(valid_rows) < 3:
        raise RuntimeError(
            "snapshot needs at least three samples with GPS >=6 satellites and VBAT 22.0-25.2 V"
        )
    return {
        "valid_sample_count": len(valid_rows),
        "minimum_fix": min(value[0] for value in valid_rows),
        "minimum_satellites": min(value[1] for value in valid_rows),
        "minimum_vbat_v": min(value[2] for value in valid_rows),
        "maximum_vbat_v": max(value[2] for value in valid_rows),
        "telemetry_sha256": expected_hash,
    }


def _configured_scope(config: dict[str, Any]) -> str:
    candidate_scope = str(dict(config.get("candidate_profile", {})).get("scope", ""))
    flight_scope = str(dict(config.get("flight_profile", {})).get("scope", ""))
    if candidate_scope != flight_scope or flight_scope not in ACTIVE_SCOPES:
        raise RuntimeError(
            "candidate_profile and flight_profile must use the same supported short-supervised scope"
        )
    return flight_scope


def validate_flight_supervised_config(
    config: dict[str, Any],
    *,
    output_path: Path,
    parsed_cli: dict[str, Any],
    fc_identity: dict[str, Any],
    config_path: Path | None = None,
) -> dict[str, Any]:
    candidate = dict(config.get("candidate_profile", {}))
    profile = dict(config.get("flight_profile", {}))
    policy = dict(config.get("runtime_policy", {}))
    authorization = dict(config.get("control_authorization", {}))
    safety = dict(config.get("safety", {}))
    logging = dict(config.get("logging", {}))
    scope = _configured_scope(config)

    for key in ("propellers_installed", "professional_pilot_required", "acro_rate_mode_required"):
        if profile.get(key) is not True:
            raise RuntimeError(f"flight_profile.{key}=true is required")
    if profile.get("controlled_axes") != ["roll", "pitch", "throttle", "yaw"]:
        raise RuntimeError("supervised profile must control Roll/Pitch/Throttle/Yaw")
    if profile.get("yaw_policy") != "neutral_1500_us":
        raise RuntimeError("supervised profile must hold Yaw neutral")
    if int(profile.get("override_channels_mask", -1)) != OVERRIDE_CHANNELS_MASK:
        raise RuntimeError("supervised profile must use mask 15")
    if (
        float(profile.get("operator_target_takeover_duration_s", math.nan)) != 0.5
        or float(profile.get("max_takeover_duration_s", math.nan))
        != MAX_TAKEOVER_DURATION_S
        or profile.get("takeover_time_basis") != "actual_algorithm_publication"
        or int(profile.get("max_takeovers_per_arm", 0)) != 1
        or float(profile.get("rc7_release_rearm_s", math.nan)) != 0.0
    ):
        raise RuntimeError("short supervised profile must declare one 0.9 s takeover per ARM")

    if policy.get("required_authorization_scope") != scope:
        raise RuntimeError("runtime policy scope mismatch")
    if policy.get("allowed_control_modes") != ["msp_raw_rc"]:
        raise RuntimeError("active flight profile must reject LOG_ONLY startup")
    if policy.get("allow_control_flag_permitted") is not True:
        raise RuntimeError("runtime policy must permit --allow-control")
    if policy.get("msp_set_raw_rc_permitted") is not True:
        raise RuntimeError("runtime policy must permit MSP_SET_RAW_RC")
    if authorization.get("enabled") is not True or authorization.get("required_scope") != scope:
        raise RuntimeError("control_authorization scope mismatch")
    if int(authorization.get("minimum_approval_schema_version", 0)) < 5:
        raise RuntimeError("control_authorization must require approval schema v5")
    if authorization.get("manual_msp_loss_waiver_required") is not True:
        raise RuntimeError("control_authorization must require manual MSP-loss waiver")
    if authorization.get("rc_interlock_evidence_required") is True:
        raise RuntimeError(
            "short supervised scopes use manual RC7 return, not mandatory EdgeTX release interlock"
        )
    if authorization.get("release_evidence_required") is not True:
        raise RuntimeError("control_authorization must require release evidence")
    if authorization.get("thrust_model_evidence_required") is not True:
        raise RuntimeError("control_authorization must require thrust-model evidence")
    if authorization.get("finalized_run_evidence_required") is not True:
        raise RuntimeError("control_authorization must require finalized run evidence")
    if authorization.get("noprop_timing_evidence_required") is not True:
        raise RuntimeError("control_authorization must require no-prop timing evidence")
    if authorization.get("sitl_evidence_required") is not True:
        raise RuntimeError("control_authorization must require policy-bound SIL evidence")
    if authorization.get("software_binding_required") is not True:
        raise RuntimeError("control_authorization must require a clean software binding")
    configured_output = Path(str(authorization.get("approval_manifest", ""))).expanduser().resolve()
    if configured_output != output_path:
        raise RuntimeError("approval output must match control_authorization.approval_manifest")

    try:
        runtime = MspRuntimeConfig.from_mapping(dict(config.get("msp_runtime", {})))
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid msp_runtime: {exc}") from exc
    if not runtime.io_worker_enabled or not runtime.prefill_enabled:
        raise RuntimeError("MSP worker and physical-RC prefill are required")
    if runtime.transport_mode != "async_pipeline":
        raise RuntimeError("asynchronous MSP pipeline is required")
    if runtime.override_channels_mask != OVERRIDE_CHANNELS_MASK:
        raise RuntimeError("msp_runtime.override_channels_mask must be mask 15")
    if runtime.set_raw_rc_channel_map != "AETR1234":
        raise RuntimeError("SET_RAW_RC wire order must be AETR1234")
    if runtime.aux_arm_channel_zero_based != 4 or runtime.throttle_channel_zero_based != 2:
        raise RuntimeError("ARM and Throttle channel indices do not match AETR1234")
    if runtime.control_publish_hz != 50.0:
        raise RuntimeError("control_publish_hz must be exactly 50 Hz")
    for key, expected in EXPECTED_POLL_HZ.items():
        if not math.isclose(float(getattr(runtime, key)), expected, abs_tol=1.0e-9):
            raise RuntimeError(f"msp_runtime.{key} must be exactly {expected:g} Hz")
    if runtime.prefill_min_frames < 10 or runtime.response_stale_s > 0.25:
        raise RuntimeError("prefill or MSP acknowledgement freshness is too weak")
    if runtime.throttle_relative_limit_us != 0:
        raise RuntimeError("supervised profile must disable relative throttle limiting")
    if (
        runtime.throttle_reference_min_us != THROTTLE_MIN_US
        or runtime.throttle_reference_max_us != 1400
        or runtime.throttle_command_min_us != THROTTLE_MIN_US
        or runtime.throttle_command_max_us != THROTTLE_MAX_US
        or not math.isclose(
            runtime.throttle_slew_limit_us_per_s,
            THROTTLE_SLEW_US_PER_S,
            abs_tol=1.0e-9,
        )
    ):
        raise RuntimeError("supervised throttle runtime envelope mismatch")

    takeover = dict(safety.get("takeover_duration_interlock", {}))
    if (
        takeover.get("enabled") is not True
        or takeover.get("latch_until_disarm") is not True
        or float(takeover.get("max_duration_s", math.nan))
        != MAX_TAKEOVER_DURATION_S
        or int(takeover.get("max_takeovers_per_arm", 0)) != 1
        or float(takeover.get("rearm_release_s", math.nan)) != 0.0
    ):
        raise RuntimeError("takeover interlock must enforce one 0.9 s pulse per ARM")
    if safety.get("require_acro_rate_mode") is not True:
        raise RuntimeError("Acro/Rate mode is required")
    if float(safety.get("min_vbat_v", 0.0)) < MIN_VBAT_V:
        raise RuntimeError("minimum battery gate must be at least 22 V")
    if float(safety.get("max_vbat_v", math.inf)) > REQUIRED_THRUST_VOLTAGE_MAX_V:
        raise RuntimeError("maximum battery gate must be at most 25.2 V")
    aux = dict(safety.get("aux_enable", {}))
    if (
        int(aux.get("channel_index", -1)) != 7
        or int(aux.get("min_us", 0)) != 1700
        or int(aux.get("max_us", 0)) != 2100
        or aux.get("satisfied_by_override_mode") is not True
    ):
        raise RuntimeError("takeover gate must be MSP OVERRIDE on RC7/AUX3 high")

    try:
        rc_values = dict(config.get("rc_mapping", {}))
        rc = RcMappingConfig(
            **{
                key: value
                for key, value in rc_values.items()
                if key in RcMappingConfig.__dataclass_fields__
            }
        )
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid rc_mapping: {exc}") from exc
    if rc.channel_map != "AETR1234" or rc.rate_mapping_type != "betaflight":
        raise RuntimeError("RC mapping must use AETR1234 Betaflight rate inversion")
    if tuple(rc.betaflight_rc_rate) != (1.0, 1.0, 1.0):
        raise RuntimeError("RC Rate must be 1.0 on all axes")
    if tuple(rc.betaflight_super_rate) != (0.7, 0.7, 0.7):
        raise RuntimeError("Super Rate must be 0.7 on all axes")
    if tuple(rc.betaflight_expo) != (0.0, 0.0, 0.0):
        raise RuntimeError("Expo must be zero on all axes")
    if (
        rc.roll_command_limit_deg_s != MAX_RATE_DEG_S
        or rc.pitch_command_limit_deg_s != MAX_RATE_DEG_S
        or rc.yaw_command_limit_deg_s != 0.0
    ):
        raise RuntimeError("Roll/Pitch/Yaw command limits must be 60/60/0 deg/s")
    if (
        rc.throttle_min_us != THROTTLE_MIN_US
        or rc.throttle_hover_us != THROTTLE_HOVER_US
        or rc.throttle_max_us != THROTTLE_MAX_US
        or rc.neutral_throttle_us != THROTTLE_MIN_US
        or rc.max_delta_us_per_s != THROTTLE_SLEW_US_PER_S
    ):
        raise RuntimeError("RC measured throttle mapping must be 1200/1275/1500 at 600 us/s")
    _validate_rate_profile(dict(config.get("rc_mapping", {})), parsed_cli)

    guidance = _validate_guidance(config, expected_scope=scope)
    command = _validate_guidance_command(config, config_path=config_path)
    kinematics = dict(config.get("kinematics", {}))
    if int(kinematics.get("minimum_satellites", 0)) < MIN_GPS_SATELLITES:
        raise RuntimeError("at least six GPS satellites are required")
    if int(kinematics.get("origin_lock_samples", 0)) < 3:
        raise RuntimeError("at least three stable origin samples are required")
    if not math.isclose(float(logging.get("csv_flush_interval_s", math.nan)), 1.0):
        raise RuntimeError("logging.csv_flush_interval_s must be exactly 1 s")
    evidence_frames = dict(logging.get("evidence_frames", {}))
    if (
        evidence_frames.get("enabled") is not True
        or not math.isclose(float(evidence_frames.get("max_fps", 0.0)), 5.0)
        or int(evidence_frames.get("jpeg_quality", 0)) != 80
    ):
        raise RuntimeError("supervised flight requires 5 Hz JPEG-80 evidence frames")

    rknn = dict(config.get("rknn_detector", {}))
    torch_runtime = dict(config.get("torch_runtime", {}))
    if not str(rknn.get("model", "")).endswith(".rknn") or not str(
        rknn.get("library", "")
    ).endswith(".so"):
        raise RuntimeError("real RKNN model and native detector library are required")
    if torch_runtime.get("allow_cpu_inference") is not False:
        raise RuntimeError("CPU detector fallback must remain disabled")

    if scope == CONTACT_SCOPE:
        contact_risk = dict(config.get("contact_risk_policy", {}))
        if (
            authorization.get("intentional_contact_acknowledgement_required") is not True
            or contact_risk.get("intentional_contact") is not True
            or contact_risk.get("explicit_risk_waiver_required") is not True
            or contact_risk.get("bounded_takeover_required") is not True
        ):
            raise RuntimeError(
                "contact scope requires bounded intentional-contact acknowledgement"
            )

    return {
        "scope": scope,
        "camera_extrinsic": _validate_camera_extrinsic(config),
        "guidance": guidance,
        "guidance_command": command,
        "guidance_command_frames": _validate_guidance_command_frames(config),
        "msp_runtime": {
            "poll_total_hz": sum(EXPECTED_POLL_HZ.values()),
            "control_publish_hz": runtime.control_publish_hz,
            "throttle_slew_limit_us_per_s": runtime.throttle_slew_limit_us_per_s,
        },
        "msp_raw_imu_gyro": _validate_raw_imu_gyro_binding(
            dict(config.get("msp_runtime", {})),
            fc_identity=fc_identity,
        ),
    }


def _validate_guidance(
    config: dict[str, Any],
    *,
    expected_scope: str,
) -> dict[str, Any]:
    guidance = dict(config.get("guidance", {}))
    if guidance.get("law") != "velocity_establishing_png":
        raise RuntimeError("guidance.law must be velocity_establishing_png")
    if guidance.get("velocity_source") != "msp_kinematics":
        raise RuntimeError("guidance.velocity_source must be msp_kinematics")
    if not math.isclose(
        float(guidance.get("max_guidance_accel_mps2", math.nan)),
        MAX_GUIDANCE_ACCEL_MPS2,
    ):
        raise RuntimeError("guidance.max_guidance_accel_mps2 must be exactly 7")
    raw = dict(guidance.get("velocity_establishing_png", {}))
    try:
        value = VelocityEstablishingPngConfig(**raw)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid velocity-establishing PNG config: {exc}") from exc
    expected_policy = "contact" if expected_scope == CONTACT_SCOPE else "noncollision"
    if value.engagement_policy != expected_policy:
        raise RuntimeError("guidance engagement policy does not match approval scope")
    if value.acquire_consecutive_frames != 3:
        raise RuntimeError("short supervised guidance requires three acquisition frames")
    if not math.isclose(value.detection_result_age_limit_s, 0.20):
        raise RuntimeError(
            "short supervised guidance requires a 0.20 s detection result-age limit"
        )
    if not math.isclose(value.fixed_vm_m_s, 10.0) or not math.isclose(
        value.navigation_constant, 3.0
    ):
        raise RuntimeError("supervised candidate requires Vm=10 m/s and N=3")
    for key in (
        "speed_accel_limit_m_s2",
        "png_accel_limit_m_s2",
        "fov_centering_accel_limit_m_s2",
        "total_accel_limit_m_s2",
    ):
        if float(getattr(value, key)) > MAX_GUIDANCE_ACCEL_MPS2:
            raise RuntimeError(f"{key} exceeds 7 m/s2")
    if not math.isclose(value.total_accel_limit_m_s2, MAX_GUIDANCE_ACCEL_MPS2):
        raise RuntimeError("total_accel_limit_m_s2 must be exactly 7 m/s2")
    return {"law": guidance["law"], "velocity_source": guidance["velocity_source"], **asdict(value)}


def _validate_guidance_command(
    config: dict[str, Any],
    *,
    config_path: Path | None,
) -> dict[str, Any]:
    command = dict(config.get("guidance_command", {}))
    if not math.isclose(float(command.get("hover_thrust", math.nan)), 0.5):
        raise RuntimeError("hover_thrust must map to measured 1275 us")
    try:
        accel = AccelerationTiltRateConfig.from_mapping(
            dict(command.get("accel_tilt_rate", {}))
        )
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid accel_tilt_rate config: {exc}") from exc
    if (
        accel.max_roll_tilt_deg != MAX_TILT_DEG
        or accel.max_pitch_tilt_deg != MAX_TILT_DEG
        or accel.max_roll_rate_deg_s != MAX_RATE_DEG_S
        or accel.max_pitch_rate_deg_s != MAX_RATE_DEG_S
        or accel.roll_rate_sign != 1.0
        or accel.pitch_rate_sign != -1.0
    ):
        raise RuntimeError("accel_tilt_rate limits/signs must be 35/35 deg, 60/60 deg/s, +1/-1")
    thrust = accel.thrust_feedforward
    if (
        not thrust.enabled
        or thrust.model != "voltage_throttle_lut"
        or not math.isclose(thrust.hover_load_factor_g, 1.0)
        or not math.isclose(thrust.max_load_factor_g, 2.37)
        or not math.isclose(thrust.minimum_tilt_cosine, 0.5)
    ):
        raise RuntimeError("voltage/throttle thrust feedforward binding mismatch")
    thrust_model = _validate_thrust_model(thrust, config_path=config_path)
    tilt = dict(command.get("tilt_envelope", {}))
    if (
        tilt.get("enabled") is not True
        or float(tilt.get("max_roll_angle_deg", math.nan)) != MAX_TILT_DEG
        or float(tilt.get("max_pitch_angle_deg", math.nan)) != MAX_TILT_DEG
        or float(tilt.get("hardcap_max_level_rate_deg_s", math.nan)) != MAX_RATE_DEG_S
    ):
        raise RuntimeError("tilt envelope must be enabled at 35 deg with 60 deg/s leveling")
    entry = dict(command.get("entry_handoff", {}))
    if (
        entry.get("enabled") is not True
        or entry.get("rate_source") != "gyro"
        or not math.isclose(
            float(entry.get("duration_s", math.nan)), 0.8, abs_tol=1.0e-9
        )
        or float(entry.get("gyro_max_age_s", math.inf)) > 0.25
    ):
        raise RuntimeError("gyro-based entry handoff must remain enabled and fresh")
    return {
        "mapping_type": command.get("mapping_type"),
        "hover_thrust": command.get("hover_thrust"),
        "accel_tilt_rate": asdict(accel),
        "thrust_model": thrust_model,
    }


def _validate_thrust_model(
    thrust: Any,
    *,
    config_path: Path | None,
) -> dict[str, Any]:
    if str(thrust.calibration_id).strip().upper().startswith("PENDING"):
        raise RuntimeError("pending thrust LUT calibration cannot be approved")
    raw_path = Path(str(thrust.model_path)).expanduser()
    if raw_path.is_absolute():
        model_path = raw_path.resolve()
    else:
        repository_candidate = (ROOT / raw_path).resolve()
        config_candidate = (
            (config_path.resolve().parent / raw_path).resolve()
            if config_path is not None
            else repository_candidate
        )
        model_path = (
            repository_candidate
            if repository_candidate.is_file() or not config_candidate.is_file()
            else config_candidate
        )
    try:
        model = VoltageThrottleThrustModel.from_file(
            model_path,
            expected_sha256=thrust.model_sha256,
            expected_calibration_id=thrust.calibration_id,
        )
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"invalid voltage/throttle thrust LUT: {exc}") from exc
    if (
        model.minimum_voltage_v > REQUIRED_THRUST_VOLTAGE_MIN_V
        or model.maximum_voltage_v < REQUIRED_THRUST_VOLTAGE_MAX_V
    ):
        raise RuntimeError("thrust LUT does not cover the full 22.0-25.2 V flight range")
    try:
        sample_count = int(model.validation["sample_count"])
        median_error = float(model.validation["median_relative_error"])
        p95_error = float(model.validation["p95_relative_error"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("thrust LUT validation metrics are incomplete") from exc
    if (
        model.validation.get("passed") is not True
        or not math.isfinite(median_error)
        or not math.isfinite(p95_error)
        or median_error > 0.10
        or p95_error > 0.20
    ):
        raise RuntimeError(
            "thrust LUT held-out validation does not pass release thresholds"
        )
    if sample_count < MIN_THRUST_VALIDATION_SAMPLES:
        raise RuntimeError("thrust LUT held-out validation sample count is insufficient")
    try:
        effective_rate_hz = float(model.validation["effective_sample_rate_hz"])
        coverage_counts = model.validation["three_by_five_sample_counts"]
        minimum_cell_samples = int(model.validation["minimum_cell_samples"])
        filter_counts = model.validation["filter_counts"]
        time_constant_s = float(model.dynamics["first_order_time_constant_s"])
        dynamics_sample_count = int(model.dynamics["fit_sample_count"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("thrust LUT filtering, coverage, or dynamics evidence is incomplete") from exc
    if (
        not math.isclose(effective_rate_hz, 10.0)
        or not _is_three_by_five_matrix(coverage_counts)
        or not isinstance(filter_counts, dict)
        or not {
            "armed_edge_takeoff_landing_trim",
            "collision_or_force_outlier",
            "high_angular_rate",
            "motor_saturation",
        }.issubset(filter_counts)
        or not 0.0 < time_constant_s <= 0.5
        or dynamics_sample_count < 500
    ):
        raise RuntimeError("thrust LUT does not satisfy filtering and dynamics gates")

    coverage_policy = str(
        model.validation.get("coverage_policy", "full_three_by_five_grid_v1")
    )
    if coverage_policy == "bounded_physics_constrained_sparse_surface_v1":
        try:
            voltage_extrapolation_v = [
                float(value) for value in model.validation["voltage_extrapolation_v"]
            ]
            maximum_voltage_extrapolation_v = float(
                model.validation["maximum_voltage_extrapolation_v"]
            )
            throttle_band_counts = [
                int(value)
                for value in model.validation["throttle_band_sample_counts"]
            ]
            high_throttle_sample_count = int(
                model.validation["high_throttle_sample_count"]
            )
            minimum_high_throttle_samples = int(
                model.validation["minimum_high_throttle_samples"]
            )
            maximum_high_throttle_p95_error = float(
                model.validation["maximum_high_throttle_p95_relative_error"]
            )
            high_throttle_median_error = float(
                model.validation[
                    "high_throttle_support_median_relative_error"
                ]
            )
            high_throttle_p95_error = float(
                model.validation["high_throttle_support_p95_relative_error"]
            )
            high_throttle_holdout_sample_count = int(
                model.validation["high_throttle_holdout_sample_count"]
            )
            high_throttle_holdout_p95_error = float(
                model.validation["high_throttle_holdout_p95_relative_error"]
            )
            holdout_counts = model.validation[
                "holdout_three_by_five_sample_counts"
            ]
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                "physics-constrained thrust LUT evidence is incomplete"
            ) from exc
        if (
            model.fit.get("method")
            != "voltage_scaled_effective_input_quadratic_v1"
            or model.fit.get("effective_input")
            != "voltage_v * (throttle_us - effective_zero_throttle_us)"
            or len(voltage_extrapolation_v) != 2
            or any(
                not math.isfinite(value) or value < 0.0
                for value in voltage_extrapolation_v
            )
            or not math.isfinite(maximum_voltage_extrapolation_v)
            or maximum_voltage_extrapolation_v < 0.0
            or maximum_voltage_extrapolation_v > 0.30
            or max(voltage_extrapolation_v) > maximum_voltage_extrapolation_v
            or len(throttle_band_counts) != 5
            or any(value <= 0 for value in throttle_band_counts)
            or minimum_high_throttle_samples < 5
            or high_throttle_sample_count < minimum_high_throttle_samples
            or not math.isfinite(maximum_high_throttle_p95_error)
            or maximum_high_throttle_p95_error < 0.0
            or maximum_high_throttle_p95_error > 0.25
            or not math.isfinite(high_throttle_median_error)
            or not math.isfinite(high_throttle_p95_error)
            or high_throttle_median_error > 0.10
            or high_throttle_p95_error > maximum_high_throttle_p95_error
            or high_throttle_holdout_sample_count < 1
            or not math.isfinite(high_throttle_holdout_p95_error)
            or high_throttle_holdout_p95_error
            > maximum_high_throttle_p95_error
            or not _is_three_by_five_matrix(holdout_counts)
        ):
            raise RuntimeError(
                "thrust LUT does not satisfy bounded physics-constrained coverage gates"
            )
    elif coverage_policy == "full_three_by_five_grid_v1":
        if (
            minimum_cell_samples < 5
            or any(
                int(value) < minimum_cell_samples
                for row in coverage_counts
                for value in row
            )
        ):
            raise RuntimeError(
                "thrust LUT does not satisfy filtered 3x5 coverage gates"
            )
    else:
        raise RuntimeError(f"unsupported thrust LUT coverage policy: {coverage_policy}")
    return {
        "path": str(model_path),
        "sha256": model.source_sha256,
        "calibration_id": model.calibration_id,
        "voltage_coverage_v": [model.minimum_voltage_v, model.maximum_voltage_v],
        "throttle_coverage_us": [
            float(model.throttle_us[0]),
            float(model.throttle_us[-1]),
        ],
        "validation": {
            "passed": True,
            "sample_count": sample_count,
            "median_relative_error": median_error,
            "p95_relative_error": p95_error,
            "effective_sample_rate_hz": effective_rate_hz,
            "coverage_policy": coverage_policy,
            "three_by_five_sample_counts": coverage_counts,
            "minimum_cell_samples": minimum_cell_samples,
        },
        "dynamics": dict(model.dynamics),
    }


def _is_three_by_five_matrix(value: object) -> bool:
    return bool(
        isinstance(value, list)
        and len(value) == 3
        and all(isinstance(row, list) and len(row) == 5 for row in value)
    )


def _repository_state() -> tuple[str, bool]:
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
        status = subprocess.check_output(
            ["git", "status", "--porcelain", "--untracked-files=no"],
            cwd=ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError("cannot determine repository state for supervised approval") from exc
    if len(commit) != 40:
        raise RuntimeError("repository commit is invalid")
    return commit, bool(status.strip())


if __name__ == "__main__":
    main()
