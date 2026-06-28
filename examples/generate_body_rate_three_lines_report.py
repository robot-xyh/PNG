from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAJECTORY_DIR = PROJECT_ROOT / "logs" / "body_rate_three_lines"
DEFAULT_REPORT_PATH = PROJECT_ROOT / "完整方案" / "BodyRate_三问题线实施实验报告.md"
DEFAULT_ASSET_DIR = PROJECT_ROOT / "完整方案" / "assets" / "BodyRate_三问题线实施实验报告"

EXPERIMENT_LABELS = {
    "A_truth_actor": "A truth+actor",
    "A_vehicle_pair": "A truth+vehicle-pair",
    "B0": "B0 gimbal baseline",
    "B1": "B1 gimbal no LOS filter",
    "B2": "B2 gimbal yaw feedback",
    "B3": "B3 gimbal frame guard",
    "B4": "B4 gimbal TTC fallback",
    "B5": "B5 gimbal blind push",
    "C0": "C0 strapdown AirSim detect",
    "C1": "C1 strapdown YOLO no LOS filter",
    "C2": "C2 strapdown YOLO relaxed LOS",
    "C3": "C3 strapdown YOLO hard LOS gate",
    "C4": "C4 strapdown saturation guard",
    "C5": "C5 strapdown YOLO 12Hz",
    "C6": "C6 strapdown YOLO 20Hz",
    "C7": "C7 strapdown high authority",
    "C8": "C8 strapdown high thrust limited accel",
}


@dataclass
class Case:
    experiment: str
    law: str
    range_m: float
    csv_path: Path
    rows: list[dict[str, str]]
    hit: bool
    geometric_hit_1m: bool
    geometric_hit_15m: bool
    geometric_hit_2m: bool
    collision_raw_hit: bool
    collision_accepted: bool
    hit_t: float
    min_range: float
    min_range_t: float
    final_range: float
    frames: int
    detected_rate: float
    valid_rate: float
    body_rate_active_rate: float
    thrust_sat_rate: float
    los_reject_count: int
    avg_wall_fps: float
    avg_sim_fps: float
    avg_detector_fps: float
    max_n_cmd_g: float
    max_actual_g: float
    target_body_bearing_pre1_abs_mean: float
    gimbal_yaw_pre1_abs_mean: float
    bbox_top_first_s: float
    bbox_bottom_first_s: float
    terminal_lost_first_s: float
    terminal_state_at_min: str
    reject_reason_at_min: str


def _float(value: object, default: float = math.nan) -> float:
    if value is None or value == "":
        return default
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _finite(values: list[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def _mean(values: list[float]) -> float:
    finite = _finite(values)
    return float(sum(finite) / len(finite)) if finite else math.nan


def _max(values: list[float]) -> float:
    finite = _finite(values)
    return max(finite) if finite else math.nan


def _truthy(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _rate(rows: list[dict[str, str]], key: str) -> float:
    if not rows:
        return math.nan
    return sum(1 for row in rows if _truthy(row.get(key, ""))) / len(rows)


def _first_time(rows: list[dict[str, str]], predicate) -> float:
    for row in rows:
        if predicate(row):
            return _float(row.get("t"))
    return math.nan


def _row_at_min_range(rows: list[dict[str, str]]) -> dict[str, str]:
    best_row: dict[str, str] = {}
    best_range = math.inf
    for row in rows:
        range_m = _float(row.get("range"))
        if math.isfinite(range_m) and range_m < best_range:
            best_range = range_m
            best_row = row
    return best_row


def _window_abs_mean(rows: list[dict[str, str]], key: str, center_t: float, before_s: float = 1.0) -> float:
    if not math.isfinite(center_t):
        return math.nan
    values = [
        abs(_float(row.get(key)))
        for row in rows
        if center_t - before_s <= _float(row.get("t")) <= center_t
    ]
    return _mean(values)


def _count_los_reject(rows: list[dict[str, str]]) -> int:
    count = 0
    for row in rows:
        text = " ".join(
            str(row.get(key, ""))
            for key in ("reject_reason", "image_kf_reject_reason", "terminal_reason")
        )
        if "los_innovation_reject" in text or "los_invalid" in text:
            count += 1
    return count


def _case_from_csv(path: Path, stamp: str) -> Case | None:
    pattern = re.compile(rf"^body_rate_three_(?P<experiment>.+)_(?P<law>TTC|VM)_{re.escape(stamp)}_r(?P<range>[0-9.]+)_h")
    match = pattern.match(path.stem)
    if not match:
        return None
    rows = _read_csv(path)
    if not rows:
        return None
    times = [_float(row.get("t")) for row in rows]
    ranges = [_float(row.get("range")) for row in rows]
    hit_rows = [row for row in rows if _truthy(row.get("hit", ""))]
    hit = bool(hit_rows)
    min_row = _row_at_min_range(rows)
    min_range = _float(min_row.get("range"))
    min_range_t = _float(min_row.get("t"))
    detector_fps = [_float(row.get("detector_fps")) for row in rows]
    if not _finite(detector_fps):
        detector_fps = [_float(row.get("yolo_fps")) for row in rows]
    return Case(
        experiment=match.group("experiment"),
        law=match.group("law"),
        range_m=float(match.group("range")),
        csv_path=path,
        rows=rows,
        hit=hit,
        geometric_hit_1m=any(_truthy(row.get("geometric_hit_1m", "")) for row in rows),
        geometric_hit_15m=any(_truthy(row.get("geometric_hit_15m", "")) for row in rows),
        geometric_hit_2m=any(_truthy(row.get("geometric_hit_2m", "")) for row in rows),
        collision_raw_hit=any(_truthy(row.get("collision_raw_hit", "")) for row in rows),
        collision_accepted=any(_truthy(row.get("collision_accepted", row.get("hit", ""))) for row in rows),
        hit_t=_float(hit_rows[0].get("t")) if hit_rows else math.nan,
        min_range=min_range,
        min_range_t=min_range_t,
        final_range=next((value for value in reversed(ranges) if math.isfinite(value)), math.nan),
        frames=len(rows),
        detected_rate=_rate(rows, "detected"),
        valid_rate=_rate(rows, "valid"),
        body_rate_active_rate=_rate(rows, "body_rate_control_active"),
        thrust_sat_rate=_rate(rows, "body_rate_thrust_saturated"),
        los_reject_count=_count_los_reject(rows),
        avg_wall_fps=_mean([_float(row.get("wall_fps")) for row in rows]),
        avg_sim_fps=_mean([_float(row.get("sim_sample_fps")) for row in rows]),
        avg_detector_fps=_mean(detector_fps),
        max_n_cmd_g=_max([_float(row.get("n_cmd_g")) for row in rows]),
        max_actual_g=_max([_float(row.get("load_factor_fd_g")) for row in rows]),
        target_body_bearing_pre1_abs_mean=_window_abs_mean(rows, "target_body_bearing_deg", min_range_t),
        gimbal_yaw_pre1_abs_mean=_window_abs_mean(rows, "gimbal_yaw_deg", min_range_t),
        bbox_top_first_s=_first_time(rows, lambda row: _truthy(row.get("bbox_top_clipped", ""))),
        bbox_bottom_first_s=_first_time(rows, lambda row: _truthy(row.get("bbox_bottom_clipped", ""))),
        terminal_lost_first_s=_first_time(rows, lambda row: str(row.get("terminal_reason", "")) == "terminal_lost"),
        terminal_state_at_min=str(min_row.get("terminal_state", "")),
        reject_reason_at_min=str(min_row.get("reject_reason", "")),
    )


def _collect_cases(trajectory_dir: Path, stamp: str) -> list[Case]:
    cases: list[Case] = []
    for path in sorted(trajectory_dir.glob(f"body_rate_three_*_{stamp}_r*.csv")):
        case = _case_from_csv(path, stamp)
        if case is not None:
            cases.append(case)
    return sorted(cases, key=lambda case: (case.experiment, case.law, case.range_m))


def _rel(path: Path, report_path: Path) -> str:
    try:
        return path.relative_to(report_path.parent).as_posix()
    except ValueError:
        return path.as_posix()


def _label(experiment: str) -> str:
    return EXPERIMENT_LABELS.get(experiment, experiment)


def _plot_summary(cases: list[Case], asset_dir: Path) -> dict[str, Path]:
    asset_dir.mkdir(parents=True, exist_ok=True)
    outputs: dict[str, Path] = {}
    if not cases:
        return outputs

    experiments = sorted({case.experiment for case in cases})
    laws = sorted({case.law for case in cases})
    ranges = sorted({case.range_m for case in cases})

    fig, axes = plt.subplots(len(laws), 1, figsize=(12, max(4, 2.6 * len(laws))), squeeze=False)
    for row_index, law in enumerate(laws):
        ax = axes[row_index][0]
        matrix = np.full((len(experiments), len(ranges)), np.nan)
        for case in cases:
            if case.law == law:
                matrix[experiments.index(case.experiment), ranges.index(case.range_m)] = 1.0 if case.hit else 0.0
        image = ax.imshow(matrix, vmin=0.0, vmax=1.0, cmap="RdYlGn", aspect="auto")
        ax.set_xticks(range(len(ranges)), [f"{value:.0f}" for value in ranges])
        ax.set_yticks(range(len(experiments)), [_label(name) for name in experiments])
        ax.set_title(f"Collision Hit Matrix - {law}")
        for y in range(len(experiments)):
            for x in range(len(ranges)):
                value = matrix[y, x]
                if math.isfinite(float(value)):
                    ax.text(x, y, "hit" if value > 0.5 else "miss", ha="center", va="center", fontsize=8)
        fig.colorbar(image, ax=ax, fraction=0.025, pad=0.02)
    fig.tight_layout()
    outputs["hit_matrix"] = asset_dir / "hit_matrix.png"
    fig.savefig(outputs["hit_matrix"], dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(12, 6))
    for key in sorted({(case.experiment, case.law) for case in cases}):
        subset = sorted([case for case in cases if (case.experiment, case.law) == key], key=lambda item: item.range_m)
        ax.plot([case.range_m for case in subset], [case.min_range for case in subset], marker="o", label=f"{_label(key[0])} {key[1]}")
    ax.axhline(1.0, color="tab:red", linestyle="--", linewidth=1.0, label="1m")
    ax.axhline(1.5, color="tab:orange", linestyle=":", linewidth=1.0, label="1.5m")
    ax.set_xlabel("Start horizontal range / m")
    ax.set_ylabel("Minimum true range / m")
    ax.set_title("Minimum Range")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    outputs["min_range"] = asset_dir / "min_range.png"
    fig.savefig(outputs["min_range"], dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(12, 6))
    for key in sorted({(case.experiment, case.law) for case in cases if case.experiment.startswith("B")}):
        subset = sorted([case for case in cases if (case.experiment, case.law) == key], key=lambda item: item.range_m)
        ax.plot(
            [case.range_m for case in subset],
            [case.target_body_bearing_pre1_abs_mean for case in subset],
            marker="o",
            label=f"{_label(key[0])} {key[1]} target bearing",
        )
    ax.axhline(10.0, color="tab:red", linestyle="--", linewidth=1.0, label="10 deg target")
    ax.set_xlabel("Start horizontal range / m")
    ax.set_ylabel("Mean abs target body bearing before closest point / deg")
    ax.set_title("Gimbal/Body Coupling Diagnostic")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    outputs["gimbal_body_bearing"] = asset_dir / "gimbal_body_bearing.png"
    fig.savefig(outputs["gimbal_body_bearing"], dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for key in sorted({(case.experiment, case.law) for case in cases if case.experiment.startswith("C")}):
        subset = sorted([case for case in cases if (case.experiment, case.law) == key], key=lambda item: item.range_m)
        label = f"{_label(key[0])} {key[1]}"
        axes[0].plot([case.range_m for case in subset], [100.0 * case.detected_rate for case in subset], marker="o", label=label)
        axes[1].plot([case.range_m for case in subset], [case.los_reject_count for case in subset], marker="o", label=label)
        axes[2].plot([case.range_m for case in subset], [100.0 * case.thrust_sat_rate for case in subset], marker="o", label=label)
    axes[0].set_ylabel("Detected / %")
    axes[1].set_ylabel("LOS reject count")
    axes[2].set_ylabel("Thrust saturation / %")
    axes[2].set_xlabel("Start horizontal range / m")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, ncol=2)
    axes[0].set_title("CDE Quality Diagnostics")
    fig.tight_layout()
    outputs["cde_quality"] = asset_dir / "cde_quality.png"
    fig.savefig(outputs["cde_quality"], dpi=180)
    plt.close(fig)
    return outputs


def _fmt(value: float, precision: int = 2) -> str:
    return "" if not math.isfinite(value) else f"{value:.{precision}f}"


def _case_table(cases: list[Case]) -> list[str]:
    lines = [
        "|实验|导引|距离m|collision|geom<1m|geom<1.5m|geom<2m|min m|final m|检测率|有效率|body-rate率|推力饱和|LOS拒绝|最近点前body bearing|最近点状态|CSV|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    for case in cases:
        lines.append(
            "|"
            + "|".join(
                [
                    _label(case.experiment),
                    case.law,
                    f"{case.range_m:.0f}",
                    "1" if case.hit else "0",
                    "1" if case.geometric_hit_1m else "0",
                    "1" if case.geometric_hit_15m else "0",
                    "1" if case.geometric_hit_2m else "0",
                    _fmt(case.min_range),
                    _fmt(case.final_range),
                    _fmt(100.0 * case.detected_rate, 1) + ("%" if math.isfinite(case.detected_rate) else ""),
                    _fmt(100.0 * case.valid_rate, 1) + ("%" if math.isfinite(case.valid_rate) else ""),
                    _fmt(100.0 * case.body_rate_active_rate, 1) + ("%" if math.isfinite(case.body_rate_active_rate) else ""),
                    _fmt(100.0 * case.thrust_sat_rate, 1) + ("%" if math.isfinite(case.thrust_sat_rate) else ""),
                    str(case.los_reject_count),
                    _fmt(case.target_body_bearing_pre1_abs_mean, 1),
                    f"{case.terminal_state_at_min}/{case.reject_reason_at_min}".strip("/"),
                    f"`{case.csv_path.name}`",
                ]
            )
            + "|"
        )
    return lines


def _summary_lines(cases: list[Case]) -> list[str]:
    lines = []
    for experiment in sorted({case.experiment for case in cases}):
        subset = [case for case in cases if case.experiment == experiment]
        if not subset:
            continue
        hit_count = sum(1 for case in subset if case.hit)
        geom15_count = sum(1 for case in subset if case.geometric_hit_15m)
        los_reject = sum(case.los_reject_count for case in subset)
        thrust_sat = _mean([case.thrust_sat_rate for case in subset])
        lines.append(
            f"- `{experiment}` {_label(experiment)}: collision `{hit_count}/{len(subset)}`, "
            f"geometric<1.5m `{geom15_count}/{len(subset)}`, LOS reject `{los_reject}`, "
            f"mean thrust saturation `{_fmt(100.0 * thrust_sat, 1)}%`."
        )
    return lines


def _write_report(cases: list[Case], plots: dict[str, Path], report_path: Path, stamp: str, trajectory_dir: Path) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# BodyRate 三问题线实施实验报告",
        "",
        f"- stamp: `{stamp}`",
        f"- trajectory_dir: `{trajectory_dir}`",
        f"- cases: `{len(cases)}`",
        "",
        "## 1. 总览结论",
        "",
        *(_summary_lines(cases) or ["- 未找到匹配的实验 CSV。"]),
        "",
        "## 2. 汇总图",
        "",
    ]
    for key, path in plots.items():
        lines.append(f"![{key}]({_rel(path, report_path)})")
        lines.append("")
    lines.extend(
        [
            "## 3. 实验明细",
            "",
            *_case_table(cases),
            "",
            "## 4. 判读口径",
            "",
            "- `collision` 仍然是 AirSim collision 判据。",
            "- `geom<1m/1.5m/2m` 是独立几何评价，不改写 `hit`。",
            "- B 线重点看最近点前 `target_body_bearing_deg` 是否压到 `10deg` 内。",
            "- CDE 线重点看检测率、LOS reject 和推力饱和是否与未命中同步出现。",
        ]
    )
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate BodyRate three-line diagnostic report.")
    parser.add_argument("--stamp", required=True)
    parser.add_argument("--trajectory-dir", type=Path, default=DEFAULT_TRAJECTORY_DIR)
    parser.add_argument("--report-path", type=Path, default=DEFAULT_REPORT_PATH)
    parser.add_argument("--asset-dir", type=Path, default=DEFAULT_ASSET_DIR)
    args = parser.parse_args()

    cases = _collect_cases(args.trajectory_dir, args.stamp)
    plots = _plot_summary(cases, args.asset_dir)
    _write_report(cases, plots, args.report_path, args.stamp, args.trajectory_dir)
    print(f"body_rate_three_lines_report={args.report_path}")


if __name__ == "__main__":
    main()
