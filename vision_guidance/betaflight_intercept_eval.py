from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Iterable, Mapping, Sequence


@dataclass(frozen=True)
class InterceptionAcceptanceCriteria:
    initially_visible_hit_rate_min: float = 0.95
    initially_visible_fov_hit_rate_min: float = 0.90
    target_stale_failure_rate_max: float = 0.01
    mean_measurement_valid_fraction_min: float = 0.90
    mean_kinematic_valid_fraction_min: float = 0.90
    worst_minimum_range_m_max: float | None = 1.0
    mean_tilt_saturation_fraction_max: float = 0.10
    mean_rate_saturation_fraction_max: float = 0.10

    def __post_init__(self) -> None:
        for name in (
            "initially_visible_hit_rate_min",
            "initially_visible_fov_hit_rate_min",
            "target_stale_failure_rate_max",
            "mean_measurement_valid_fraction_min",
            "mean_kinematic_valid_fraction_min",
            "mean_tilt_saturation_fraction_max",
            "mean_rate_saturation_fraction_max",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if self.worst_minimum_range_m_max is not None:
            if (
                not math.isfinite(float(self.worst_minimum_range_m_max))
                or self.worst_minimum_range_m_max <= 0.0
            ):
                raise ValueError(
                    "worst_minimum_range_m_max must be null or finite and positive"
                )

    def to_dict(self) -> dict[str, float | None]:
        return asdict(self)


def evaluate_interception_results(
    rows: Sequence[Mapping[str, object]],
    criteria: InterceptionAcceptanceCriteria,
) -> dict[str, object]:
    if not rows:
        raise ValueError("cannot evaluate empty interception results")
    visible = [row for row in rows if bool(row.get("initial_target_in_fov"))]
    visible_count = len(visible)
    visible_hit_count = sum(bool(row.get("hit")) for row in visible)
    visible_fov_hit_count = sum(bool(row.get("fov_feasible_hit")) for row in visible)
    target_stale_count = sum(_is_target_stale_failure(row) for row in visible)

    hit_rate = _fraction(visible_hit_count, visible_count)
    fov_hit_rate = _fraction(visible_fov_hit_count, visible_count)
    target_stale_rate = _fraction(target_stale_count, visible_count)
    measurement_valid_fraction = _mean(
        _finite_float(row, "measurement_valid_fraction") for row in visible
    )
    kinematic_valid_fraction = _mean(
        _finite_float(row, "kinematic_valid_fraction") for row in visible
    )
    worst_minimum_range_m = max(
        (_finite_float(row, "minimum_range_m") for row in visible),
        default=float("inf"),
    )
    mean_tilt_saturation_fraction = _mean(
        _finite_float(row, "tilt_saturation_fraction") for row in visible
    )
    mean_rate_saturation_fraction = _mean(
        _finite_float(row, "rate_saturation_fraction") for row in visible
    )

    checks = {
        "initially_visible_hit_rate": _minimum_check(
            hit_rate, criteria.initially_visible_hit_rate_min
        ),
        "initially_visible_fov_hit_rate": _minimum_check(
            fov_hit_rate, criteria.initially_visible_fov_hit_rate_min
        ),
        "target_stale_failure_rate": _maximum_check(
            target_stale_rate, criteria.target_stale_failure_rate_max
        ),
        "mean_measurement_valid_fraction": _minimum_check(
            measurement_valid_fraction,
            criteria.mean_measurement_valid_fraction_min,
        ),
        "mean_kinematic_valid_fraction": _minimum_check(
            kinematic_valid_fraction,
            criteria.mean_kinematic_valid_fraction_min,
        ),
        "worst_minimum_range_m": _optional_maximum_check(
            worst_minimum_range_m, criteria.worst_minimum_range_m_max
        ),
        "mean_tilt_saturation_fraction": _maximum_check(
            mean_tilt_saturation_fraction,
            criteria.mean_tilt_saturation_fraction_max,
        ),
        "mean_rate_saturation_fraction": _maximum_check(
            mean_rate_saturation_fraction,
            criteria.mean_rate_saturation_fraction_max,
        ),
    }
    outcome_counts = {
        reason: sum(str(row.get("outcome_reason", "")) == reason for row in rows)
        for reason in sorted({str(row.get("outcome_reason", "")) for row in rows})
    }
    return {
        "row_count": len(rows),
        "case_count": len({str(row.get("case_id", "")) for row in rows}),
        "trial_count": len({int(row.get("trial_index", 0)) for row in rows}),
        "initially_visible_count": visible_count,
        "initially_visible_hit_count": visible_hit_count,
        "initially_visible_fov_hit_count": visible_fov_hit_count,
        "target_stale_failure_count": target_stale_count,
        "initially_visible_hit_rate": hit_rate,
        "initially_visible_hit_rate_wilson95": _wilson_interval(
            visible_hit_count, visible_count
        ),
        "initially_visible_fov_hit_rate": fov_hit_rate,
        "initially_visible_fov_hit_rate_wilson95": _wilson_interval(
            visible_fov_hit_count, visible_count
        ),
        "target_stale_failure_rate": target_stale_rate,
        "mean_measurement_valid_fraction": measurement_valid_fraction,
        "mean_kinematic_valid_fraction": kinematic_valid_fraction,
        "worst_minimum_range_m": worst_minimum_range_m,
        "mean_tilt_saturation_fraction": mean_tilt_saturation_fraction,
        "mean_rate_saturation_fraction": mean_rate_saturation_fraction,
        "outcome_counts": outcome_counts,
        "checks": checks,
        "passed": visible_count > 0 and all(
            bool(check["passed"]) for check in checks.values()
        ),
    }


def _finite_float(row: Mapping[str, object], key: str) -> float:
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"result field {key} must be numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"result field {key} must be finite")
    return value


def _is_target_stale_failure(row: Mapping[str, object]) -> bool:
    if bool(row.get("hit")):
        return False
    if str(row.get("outcome_reason", "")) == "target_stale":
        return True
    return str(row.get("controller_final_reason", "")) in {
        "detection_stale",
        "tracking_invalid",
    }


def _fraction(numerator: int, denominator: int) -> float:
    if denominator <= 0:
        return 0.0
    return numerator / denominator


def _mean(values: Iterable[float]) -> float:
    selected = list(values)
    if not selected:
        return 0.0
    return math.fsum(selected) / len(selected)


def _minimum_check(observed: float, threshold: float) -> dict[str, object]:
    return {
        "observed": observed,
        "operator": ">=",
        "threshold": threshold,
        "passed": observed >= threshold,
    }


def _maximum_check(observed: float, threshold: float) -> dict[str, object]:
    return {
        "observed": observed,
        "operator": "<=",
        "threshold": threshold,
        "passed": observed <= threshold,
    }


def _optional_maximum_check(
    observed: float,
    threshold: float | None,
) -> dict[str, object]:
    if threshold is None:
        return {
            "observed": observed,
            "operator": "report_only",
            "threshold": None,
            "passed": True,
            "required": False,
        }
    return {**_maximum_check(observed, threshold), "required": True}


def _wilson_interval(successes: int, samples: int) -> list[float]:
    if samples <= 0:
        return [0.0, 0.0]
    z = 1.959963984540054
    proportion = successes / samples
    denominator = 1.0 + z * z / samples
    center = (proportion + z * z / (2.0 * samples)) / denominator
    margin = (
        z
        * math.sqrt(
            proportion * (1.0 - proportion) / samples
            + z * z / (4.0 * samples * samples)
        )
        / denominator
    )
    return [max(0.0, center - margin), min(1.0, center + margin)]
