from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = PROJECT_ROOT / "logs" / "strapdown_accuracy"
SUMMARY_PATH = LOG_DIR / "strapdown_headless_all_summary.csv"
REPORT_PATH = PROJECT_ROOT / "完整方案" / "捷联无头距离测试报告.md"
ASSET_DIR = PROJECT_ROOT / "完整方案" / "assets" / "捷联无头距离测试报告"
CLOCK02_PREFIX = "strapdown_clock0p2_simtime_"
CLOCK02_SUMMARY_PATH = LOG_DIR / "strapdown_clock0p2_all_summary.csv"
CLOCK02_REPORT_PATH = PROJECT_ROOT / "完整方案" / "捷联ClockSpeed0p2距离过载测试报告.md"
CLOCK02_ASSET_DIR = PROJECT_ROOT / "完整方案" / "assets" / "捷联ClockSpeed0p2距离过载测试报告"
TRUTH_REQUIRED_SUMMARY_PATH = LOG_DIR / "truth_required_load" / "strapdown_clock0p2_truth_theory_N3_summary.csv"
CLOCK02_NO_LOS_PREFIX = "strapdown_clock0p2_no_los_filter_"
CLOCK02_NO_LOS_SUMMARY_PATH = LOG_DIR / "strapdown_clock0p2_no_los_filter_all_summary.csv"
CLOCK02_NO_LOS_REPORT_PATH = PROJECT_ROOT / "完整方案" / "捷联ClockSpeed0p2无LOS滤波距离过载测试报告.md"
CLOCK02_NO_LOS_ASSET_DIR = PROJECT_ROOT / "完整方案" / "assets" / "捷联ClockSpeed0p2无LOS滤波距离过载测试报告"
TRUTH_REQUIRED_NO_LOS_SUMMARY_PATH = (
    LOG_DIR / "truth_required_load" / "strapdown_clock0p2_no_los_filter_truth_theory_N3_summary.csv"
)
GRAVITY_MPS2 = 9.80665
AGGREGATE_SUMMARY_NAMES = {
    "strapdown_headless_all_summary.csv",
    "strapdown_headless_combined_summary.csv",
    "strapdown_clock0p2_all_summary.csv",
    "strapdown_clock0p2_no_los_filter_all_summary.csv",
}


@dataclass
class SummaryRow:
    case: str
    start_range_m: float
    altitude_offset_m: float
    lateral_offset_m: float
    hit: bool
    hit_t_s: float | None
    min_range_m: float
    final_range_m: float
    frames: int
    detected_frames: int
    valid_frames: int
    avg_wall_fps: float
    avg_sim_sample_fps: float
    avg_sim_clock_ratio: float
    avg_load_factor_g: float
    max_load_factor_g: float
    avg_load_factor_fd_g: float
    max_load_factor_fd_g: float
    csv_path: Path
    meta_path: Path


def _float(value: str | None, default: float = math.nan) -> float:
    if value is None or value == "":
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _int(value: str | None, default: int = 0) -> int:
    if value is None or value == "":
        return default
    try:
        return int(float(value))
    except ValueError:
        return default


def _bool(value: str | bool | None) -> bool:
    if isinstance(value, bool):
        return value
    return str(value or "").strip().lower() in {"1", "true", "yes"}


def read_summary(path: Path) -> list[SummaryRow]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = []
        for item in csv.DictReader(stream):
            rows.append(
                SummaryRow(
                    case=item["case"],
                    start_range_m=_float(item.get("start_horizontal_range_m")),
                    altitude_offset_m=_float(item.get("intruder_altitude_offset_m")),
                    lateral_offset_m=_float(item.get("lateral_offset_m")),
                    hit=_bool(item.get("hit")),
                    hit_t_s=None if item.get("hit_t_s", "") == "" else _float(item.get("hit_t_s")),
                    min_range_m=_float(item.get("min_range_m")),
                    final_range_m=_float(item.get("final_range_m")),
                    frames=_int(item.get("frames")),
                    detected_frames=_int(item.get("detected_frames")),
                    valid_frames=_int(item.get("valid_frames")),
                    avg_wall_fps=_float(item.get("avg_wall_fps")),
                    avg_sim_sample_fps=_float(item.get("avg_sim_sample_fps")),
                    avg_sim_clock_ratio=_float(item.get("avg_sim_clock_ratio")),
                    avg_load_factor_g=_float(item.get("avg_load_factor_g")),
                    max_load_factor_g=_float(item.get("max_load_factor_g")),
                    avg_load_factor_fd_g=_float(item.get("avg_load_factor_fd_g")),
                    max_load_factor_fd_g=_float(item.get("max_load_factor_fd_g")),
                    csv_path=PROJECT_ROOT / item["csv_path"],
                    meta_path=PROJECT_ROOT / item["meta_path"],
                )
            )
    return sorted(rows, key=lambda row: (row.start_range_m, row.altitude_offset_m))


def _path_for_summary(path: Path) -> str:
    absolute = path if path.is_absolute() else PROJECT_ROOT / path
    try:
        return absolute.relative_to(PROJECT_ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def write_summary(rows: list[SummaryRow], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "case",
        "start_horizontal_range_m",
        "intruder_altitude_offset_m",
        "lateral_offset_m",
        "hit",
        "hit_t_s",
        "min_range_m",
        "final_range_m",
        "frames",
        "detected_frames",
        "valid_frames",
        "avg_wall_fps",
        "avg_sim_sample_fps",
        "avg_sim_clock_ratio",
        "avg_load_factor_g",
        "max_load_factor_g",
        "avg_load_factor_fd_g",
        "max_load_factor_fd_g",
        "returncode",
        "csv_path",
        "meta_path",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "case": row.case,
                    "start_horizontal_range_m": row.start_range_m,
                    "intruder_altitude_offset_m": row.altitude_offset_m,
                    "lateral_offset_m": row.lateral_offset_m,
                    "hit": row.hit,
                    "hit_t_s": "" if row.hit_t_s is None else row.hit_t_s,
                    "min_range_m": row.min_range_m,
                    "final_range_m": row.final_range_m,
                    "frames": row.frames,
                    "detected_frames": row.detected_frames,
                    "valid_frames": row.valid_frames,
                    "avg_wall_fps": row.avg_wall_fps,
                    "avg_sim_sample_fps": row.avg_sim_sample_fps,
                    "avg_sim_clock_ratio": row.avg_sim_clock_ratio,
                    "avg_load_factor_g": row.avg_load_factor_g,
                    "max_load_factor_g": row.max_load_factor_g,
                    "avg_load_factor_fd_g": row.avg_load_factor_fd_g,
                    "max_load_factor_fd_g": row.max_load_factor_fd_g,
                    "returncode": "",
                    "csv_path": _path_for_summary(row.csv_path),
                    "meta_path": _path_for_summary(row.meta_path),
                }
            )


def read_all_batch_summaries(prefix: str = "strapdown_headless_", summary_path: Path = SUMMARY_PATH) -> tuple[list[SummaryRow], list[Path]]:
    summary_files = [
        path
        for path in sorted(LOG_DIR.glob(f"{prefix}*_summary.csv"))
        if path.name not in AGGREGATE_SUMMARY_NAMES
    ]
    rows_by_case: dict[str, SummaryRow] = {}
    for path in summary_files:
        for row in read_summary(path):
            rows_by_case[row.case] = row
    rows = sorted(rows_by_case.values(), key=lambda row: (row.start_range_m, row.altitude_offset_m, row.case))
    write_summary(rows, summary_path)
    return rows, summary_files


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _project_path(path_text: str) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else PROJECT_ROOT / path


def read_truth_required_index(path: Path = TRUTH_REQUIRED_SUMMARY_PATH) -> dict[str, dict[str, str]]:
    if not path.exists():
        return {}
    with path.open(newline="", encoding="utf-8") as stream:
        return {row["case"]: row for row in csv.DictReader(stream)}


def _truth_required_summary(row: SummaryRow, index: dict[str, dict[str, str]]) -> dict[str, str] | None:
    return index.get(row.case)


def _truth_required_rows(row: SummaryRow, index: dict[str, dict[str, str]]) -> list[dict[str, str]]:
    summary = _truth_required_summary(row, index)
    if not summary:
        return []
    output_csv = summary.get("output_csv", "")
    if not output_csv:
        return []
    path = _project_path(output_csv)
    if not path.exists():
        return []
    return read_csv_rows(path)


def _truth_required_stat(
    row: SummaryRow,
    index: dict[str, dict[str, str]],
    key: str,
    default: float = math.nan,
) -> float:
    summary = _truth_required_summary(row, index)
    if not summary:
        return default
    return _float(summary.get(key), default)


def read_meta(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def _series(rows: Iterable[dict[str, str]], key: str) -> list[float]:
    return [_float(row.get(key)) for row in rows]


def _finite_values(values: Iterable[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def _percentile(values: Iterable[float], ratio: float) -> float:
    finite = sorted(_finite_values(values))
    if not finite:
        return math.nan
    ratio = max(0.0, min(1.0, ratio))
    index = int(round((len(finite) - 1) * ratio))
    return finite[index]


def _vector_from_row(row: dict[str, str], keys: tuple[str, str, str]) -> tuple[float, float, float] | None:
    values = tuple(_float(row.get(key)) for key in keys)
    if not all(math.isfinite(value) for value in values):
        return None
    return values


def _normalize_vector(vector: tuple[float, float, float] | None) -> tuple[float, float, float] | None:
    if vector is None:
        return None
    norm = math.sqrt(sum(value * value for value in vector))
    if not math.isfinite(norm) or norm <= 1.0e-9:
        return None
    return tuple(value / norm for value in vector)


def _los_angles_deg(vector: tuple[float, float, float] | None) -> tuple[float, float] | None:
    los = _normalize_vector(vector)
    if los is None:
        return None
    x, y, z = los
    azimuth = math.degrees(math.atan2(y, x))
    horizontal = math.hypot(x, y)
    # AirSim uses NED coordinates; negative Z means upward.
    elevation = math.degrees(math.atan2(-z, horizontal))
    return azimuth, elevation


def _angle_delta_deg(a: float, b: float) -> float:
    if not math.isfinite(a) or not math.isfinite(b):
        return math.nan
    return (a - b + 180.0) % 360.0 - 180.0


def _los_separation_deg(a: tuple[float, float, float] | None, b: tuple[float, float, float] | None) -> float:
    lhs = _normalize_vector(a)
    rhs = _normalize_vector(b)
    if lhs is None or rhs is None:
        return math.nan
    dot = max(-1.0, min(1.0, sum(lhs[index] * rhs[index] for index in range(3))))
    return math.degrees(math.acos(dot))


def _camera_world_position_from_row(row: dict[str, str]) -> tuple[float, float, float] | None:
    logged = _vector_from_row(row, ("camera_world_x", "camera_world_y", "camera_world_z"))
    if logged is not None:
        return logged
    interceptor = _vector_from_row(row, ("interceptor_x", "interceptor_y", "interceptor_z"))
    if interceptor is None:
        return None
    offset = _vector_from_row(row, ("camera_x", "camera_y", "camera_z"))
    if offset is None:
        return interceptor
    yaw_deg = _float(row.get("body_yaw_deg"), 0.0)
    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    # Existing logs only stored body yaw, not full roll/pitch. This fallback is
    # exact for yaw-only motion and still accounts for the fixed camera lever arm.
    dx = c * offset[0] - s * offset[1]
    dy = s * offset[0] + c * offset[1]
    dz = offset[2]
    return interceptor[0] + dx, interceptor[1] + dy, interceptor[2] + dz


def _unwrap_degree_series(values: list[float]) -> list[float]:
    result: list[float] = []
    previous: float | None = None
    offset = 0.0
    for value in values:
        if not math.isfinite(value):
            result.append(math.nan)
            previous = None
            offset = 0.0
            continue
        adjusted = value + offset
        if previous is not None:
            while adjusted - previous > 180.0:
                offset -= 360.0
                adjusted = value + offset
            while adjusted - previous < -180.0:
                offset += 360.0
                adjusted = value + offset
        result.append(adjusted)
        previous = adjusted
    return result


def required_load_series(rows: list[dict[str, str]]) -> list[float]:
    """Velocity-command equivalent required overload, computed in simulation time."""
    required: list[float] = []
    previous_t: float | None = None
    previous_v: tuple[float, float, float] | None = None
    for row in rows:
        t = _float(row.get("t"))
        v_cmd = _vector_from_row(row, ("v_cmd_x", "v_cmd_y", "v_cmd_z"))
        if previous_t is None or previous_v is None or v_cmd is None or not math.isfinite(t):
            required.append(0.0 if v_cmd is not None and math.isfinite(t) else math.nan)
        else:
            dt = t - previous_t
            if dt <= 1.0e-6:
                required.append(math.nan)
            else:
                dv = math.sqrt(sum((v_cmd[index] - previous_v[index]) ** 2 for index in range(3)))
                required.append(dv / dt / GRAVITY_MPS2)
        if v_cmd is not None and math.isfinite(t):
            previous_t = t
            previous_v = v_cmd
    return required


def required_load_stats(rows: list[dict[str, str]]) -> dict[str, float]:
    values = _finite_values(required_load_series(rows))
    if not values:
        return {"avg": math.nan, "max": math.nan, "p95": math.nan}
    return {
        "avg": sum(values) / len(values),
        "max": max(values),
        "p95": _percentile(values, 0.95),
    }


def _format_g(value: float) -> str:
    return "-" if not _finite_summary(value) else f"{value:.2f}g"


def _format_range(values: list[float], suffix: str = "") -> str:
    finite = [value for value in values if _finite_summary(value)]
    if not finite:
        return "-"
    return f"{min(finite):.2f}{suffix} - {max(finite):.2f}{suffix}"


def _format_meter_range(values: list[float]) -> str:
    finite = [value for value in values if _finite_summary(value)]
    if not finite:
        return "-"
    return f"{min(finite):.3f}m - {max(finite):.3f}m"


def _min_range_row(rows: list[dict[str, str]]) -> dict[str, str]:
    return min(rows, key=lambda row: _float(row.get("range"), default=1.0e9))


def _rel_asset(path: Path) -> str:
    return path.relative_to(REPORT_PATH.parent).as_posix()


def _rel_asset_for(path: Path, report_path: Path) -> str:
    return path.relative_to(report_path.parent).as_posix()


def _finite_summary(value: float) -> bool:
    return math.isfinite(value)


def plot_all_summary(rows: list[SummaryRow], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    ax_min, ax_quality = axes

    altitudes = sorted({row.altitude_offset_m for row in rows})
    colors = {alt: color for alt, color in zip(altitudes, ["tab:blue", "tab:orange", "tab:green", "tab:red"])}
    for alt in altitudes:
        group = [row for row in rows if row.altitude_offset_m == alt]
        ax_min.plot(
            [row.start_range_m for row in group],
            [row.min_range_m for row in group],
            marker="o",
            linewidth=1.8,
            color=colors[alt],
            label=f"alt offset {alt:.0f}m",
        )
        for row in group:
            marker = "*" if row.hit else "x"
            ax_min.scatter(row.start_range_m, row.min_range_m, s=110, marker=marker, color=colors[alt])
            ax_min.annotate("hit" if row.hit else "miss", (row.start_range_m, row.min_range_m), xytext=(3, 6), textcoords="offset points", fontsize=8)
    ax_min.axhline(0.5, color="0.35", linestyle="--", linewidth=1, label="0.5m reference")
    ax_min.set_title("Minimum range by start geometry")
    ax_min.set_xlabel("Initial horizontal range / m")
    ax_min.set_ylabel("Minimum truth range / m")
    ax_min.grid(True, alpha=0.3)
    ax_min.legend(fontsize=8)

    x = list(range(len(rows)))
    labels = [f"{row.start_range_m:.0f}/{row.altitude_offset_m:.0f}" for row in rows]
    detect_rate = [row.detected_frames / row.frames if row.frames else 0.0 for row in rows]
    valid_rate = [row.valid_frames / row.frames if row.frames else 0.0 for row in rows]
    width = 0.38
    ax_quality.bar([i - width / 2 for i in x], detect_rate, width=width, label="detected/frame", color="tab:cyan")
    ax_quality.bar([i + width / 2 for i in x], valid_rate, width=width, label="valid/frame", color="tab:purple")
    ax_quality.set_title("Vision availability")
    ax_quality.set_xlabel("Initial range / altitude offset")
    ax_quality.set_ylabel("Frame ratio")
    ax_quality.set_xticks(x)
    ax_quality.set_xticklabels(labels, rotation=35, ha="right")
    ax_quality.set_ylim(0.0, 1.05)
    ax_quality.grid(True, axis="y", alpha=0.3)
    ax_quality.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_clock02_summary(rows: list[SummaryRow], output: Path, truth_index: dict[str, dict[str, str]] | None = None) -> None:
    truth_index = truth_index or {}
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(15, 9))
    ax_min, ax_hit, ax_load, ax_fps = axes.flat
    labels = [f"{row.start_range_m:.0f}" for row in rows]
    x = list(range(len(rows)))
    visual_req_stats = [required_load_stats(read_csv_rows(row.csv_path)) for row in rows]
    theory_p95 = [_truth_required_stat(row, truth_index, "p95_theory_n_req_g") for row in rows]
    theory_max = [_truth_required_stat(row, truth_index, "max_theory_n_req_g") for row in rows]

    ax_min.plot([row.start_range_m for row in rows], [row.min_range_m for row in rows], marker="o", linewidth=1.8)
    for row in rows:
        ax_min.scatter(row.start_range_m, row.min_range_m, marker="*" if row.hit else "x", s=110)
    ax_min.axhline(0.5, color="0.35", linestyle="--", linewidth=1, label="0.5m reference")
    ax_min.set_title("Minimum range")
    ax_min.set_xlabel("Initial horizontal range / m")
    ax_min.set_ylabel("Minimum truth range / m")
    ax_min.grid(True, alpha=0.3)
    ax_min.legend(fontsize=8)

    hit_values = [1 if row.hit else 0 for row in rows]
    ax_hit.bar(x, hit_values, color=["tab:green" if value else "tab:red" for value in hit_values])
    ax_hit.set_title("AirSim collision result")
    ax_hit.set_xlabel("Initial range / m")
    ax_hit.set_ylabel("Hit")
    ax_hit.set_ylim(0.0, 1.15)
    ax_hit.set_xticks(x)
    ax_hit.set_xticklabels(labels)
    ax_hit.grid(True, axis="y", alpha=0.3)

    width = 0.18
    ax_load.bar([i - 1.5 * width for i in x], [row.max_load_factor_fd_g for row in rows], width=width, label="max actual load")
    ax_load.bar([i - 0.5 * width for i in x], [item["max"] for item in visual_req_stats], width=width, label="max visual command equivalent")
    ax_load.bar([i + 0.5 * width for i in x], theory_max, width=width, label="max truth-PNG theory")
    ax_load.bar([i + 1.5 * width for i in x], theory_p95, width=width, label="P95 truth-PNG theory")
    ax_load.set_title("Actual, visual-command equivalent, and truth-PNG theoretical overload")
    ax_load.set_xlabel("Initial range / m")
    ax_load.set_ylabel("g")
    ax_load.set_xticks(x)
    ax_load.set_xticklabels(labels)
    ax_load.grid(True, axis="y", alpha=0.3)
    ax_load.legend(fontsize=8)

    ax_fps.bar([i - width / 2 for i in x], [row.avg_wall_fps for row in rows], width=width, label="wall FPS")
    ax_fps.bar([i + width / 2 for i in x], [row.avg_sim_sample_fps for row in rows], width=width, label="sim sample FPS")
    ax_fps.set_title("Frame-rate summary")
    ax_fps.set_xlabel("Initial range / m")
    ax_fps.set_ylabel("Hz")
    ax_fps.set_xticks(x)
    ax_fps.set_xticklabels(labels)
    ax_fps.grid(True, axis="y", alpha=0.3)
    ax_fps.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_range_curves(rows: list[SummaryRow], output: Path, title: str) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    ax_full, ax_zoom = axes
    for item in rows:
        samples = read_csv_rows(item.csv_path)
        t = _series(samples, "t")
        r = _series(samples, "range")
        min_row = _min_range_row(samples)
        min_t = _float(min_row.get("t"))
        min_r = _float(min_row.get("range"))
        label = f"{item.start_range_m:.0f}m {'hit' if item.hit else 'miss'}"
        ax_full.plot(t, r, linewidth=1.8, label=label)
        ax_full.scatter([min_t], [min_r], s=45)
        ax_zoom.plot(t, r, linewidth=1.8, label=label)
        ax_zoom.scatter([min_t], [min_r], s=45)
        if item.hit_t_s is not None:
            ax_full.axvline(item.hit_t_s, color="0.4", linestyle=":", linewidth=0.8)
            ax_zoom.axvline(item.hit_t_s, color="0.4", linestyle=":", linewidth=0.8)
    ax_full.set_title(f"{title}, full run")
    ax_full.set_xlabel("Time / s")
    ax_full.set_ylabel("Truth range / m")
    ax_full.grid(True, alpha=0.3)
    ax_full.legend(fontsize=8)

    ax_zoom.set_title(f"{title}, terminal zoom")
    ax_zoom.set_xlabel("Time / s")
    ax_zoom.set_ylabel("Truth range / m")
    ax_zoom.set_ylim(0.0, 5.0)
    ax_zoom.grid(True, alpha=0.3)
    ax_zoom.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_load_factor_curves(rows: list[SummaryRow], output: Path, truth_index: dict[str, dict[str, str]] | None = None) -> None:
    truth_index = truth_index or {}
    fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=False)
    ax_actual, ax_required, ax_fps = axes
    for item in rows:
        samples = read_csv_rows(item.csv_path)
        t = _series(samples, "t")
        actual_load = _series(samples, "load_factor_fd_g")
        visual_required_load = required_load_series(samples)
        theory_samples = _truth_required_rows(item, truth_index)
        theory_t = _series(theory_samples, "t")
        theory_required_load = _series(theory_samples, "n_req_g")
        wall_fps = _series(samples, "wall_fps")
        sim_fps = _series(samples, "sim_sample_fps")
        label = f"{item.start_range_m:.0f}m {'hit' if item.hit else 'miss'}"
        ax_actual.plot(t, actual_load, linewidth=1.5, label=label)
        ax_required.plot(t, visual_required_load, linewidth=1.1, alpha=0.55, label=f"{item.start_range_m:.0f}m visual cmd")
        if theory_samples:
            ax_required.plot(theory_t, theory_required_load, linewidth=1.5, label=f"{item.start_range_m:.0f}m truth theory")
        ax_fps.plot(t, sim_fps, linewidth=1.2, label=f"{item.start_range_m:.0f}m sim")
        ax_fps.plot(t, wall_fps, linewidth=0.9, linestyle="--", alpha=0.75, label=f"{item.start_range_m:.0f}m wall")
    ax_actual.set_title("Actual overload over time, truth-velocity finite-difference estimate")
    ax_actual.set_xlabel("Time / s")
    ax_actual.set_ylabel("Actual overload / g")
    ax_actual.grid(True, alpha=0.3)
    ax_actual.legend(fontsize=7, ncol=2)

    ax_required.set_title("Required overload over time, visual velocity-command equivalent vs truth-PNG theory")
    ax_required.set_xlabel("Time / s")
    ax_required.set_ylabel("Required overload / g")
    ax_required.grid(True, alpha=0.3)
    ax_required.legend(fontsize=7, ncol=2)

    ax_fps.set_title("Per-flight frame rate over time")
    ax_fps.set_xlabel("Time / s")
    ax_fps.set_ylabel("Hz")
    ax_fps.grid(True, alpha=0.3)
    ax_fps.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_clock02_per_distance_loads(
    rows: list[SummaryRow],
    output_dir: Path,
    truth_index: dict[str, dict[str, str]] | None = None,
    image_prefix: str = "strapdown_clock0p2",
) -> dict[str, Path]:
    truth_index = truth_index or {}
    output_dir.mkdir(parents=True, exist_ok=True)
    images: dict[str, Path] = {}
    for item in rows:
        samples = read_csv_rows(item.csv_path)
        t = _series(samples, "t")
        image_png_required_load = required_load_series(samples)
        strapdown_actual_load = _series(samples, "load_factor_fd_g")
        theory_samples = _truth_required_rows(item, truth_index)
        theory_t = _series(theory_samples, "t")
        shadow_truth_required_load = _series(theory_samples, "n_req_g")

        fig, axes = plt.subplots(2, 1, figsize=(11, 7.2), sharex=True)
        ax_load, ax_range = axes
        ax_load.plot(
            t,
            image_png_required_load,
            color="tab:orange",
            linewidth=1.0,
            alpha=0.75,
            label="image-PNG theoretical/equivalent load",
        )
        if theory_samples:
            ax_load.plot(
                theory_t,
                shadow_truth_required_load,
                color="tab:blue",
                linewidth=1.7,
                label="shadow truth-position theoretical load",
            )
        ax_load.plot(
            t,
            strapdown_actual_load,
            color="tab:green",
            linewidth=1.2,
            alpha=0.9,
            label="strapdown UAV actual load",
        )
        ax_load.set_title(f"{item.start_range_m:.0f}m overload comparison")
        ax_load.set_ylabel("Overload / g")
        ax_load.grid(True, alpha=0.3)
        ax_load.legend(fontsize=8)

        ax_range.plot(t, _series(samples, "range"), color="0.25", linewidth=1.4, label="truth range")
        if item.hit_t_s is not None:
            ax_range.axvline(item.hit_t_s, color="tab:red", linestyle=":", linewidth=1.0, label="collision")
        ax_range.set_xlabel("Time / s")
        ax_range.set_ylabel("Range / m")
        ax_range.grid(True, alpha=0.3)
        ax_range.legend(fontsize=8)

        fig.tight_layout()
        path = output_dir / f"{image_prefix}_load_compare_{int(round(item.start_range_m)):03d}m.png"
        fig.savefig(path, dpi=170)
        plt.close(fig)
        images[item.case] = path
    return images


def _los_comparison_series(rows: list[dict[str, str]]) -> dict[str, list[float]]:
    t: list[float] = []
    strap_az: list[float] = []
    strap_el: list[float] = []
    truth_az: list[float] = []
    truth_el: list[float] = []
    sep: list[float] = []
    az_err: list[float] = []
    el_err: list[float] = []
    valid_mask: list[float] = []

    for row in rows:
        time_s = _float(row.get("t"))
        strap_los = _vector_from_row(row, ("lambda_x", "lambda_y", "lambda_z"))
        camera_world = _camera_world_position_from_row(row)
        intruder = _vector_from_row(row, ("intruder_x", "intruder_y", "intruder_z"))
        truth_los: tuple[float, float, float] | None = None
        if camera_world is not None and intruder is not None:
            truth_los = tuple(intruder[index] - camera_world[index] for index in range(3))
        strap_angles = _los_angles_deg(strap_los)
        truth_angles = _los_angles_deg(truth_los)

        t.append(time_s)
        if strap_angles is None:
            strap_az.append(math.nan)
            strap_el.append(math.nan)
        else:
            strap_az.append(strap_angles[0])
            strap_el.append(strap_angles[1])
        if truth_angles is None:
            truth_az.append(math.nan)
            truth_el.append(math.nan)
        else:
            truth_az.append(truth_angles[0])
            truth_el.append(truth_angles[1])
        separation = _los_separation_deg(strap_los, truth_los)
        sep.append(separation)
        if strap_angles is None or truth_angles is None:
            az_err.append(math.nan)
            el_err.append(math.nan)
        else:
            az_err.append(abs(_angle_delta_deg(strap_angles[0], truth_angles[0])))
            el_err.append(abs(strap_angles[1] - truth_angles[1]))
        valid_mask.append(1.0 if _bool(row.get("valid")) and math.isfinite(separation) else 0.0)

    return {
        "t": t,
        "strap_az": _unwrap_degree_series(strap_az),
        "strap_el": strap_el,
        "truth_az": _unwrap_degree_series(truth_az),
        "truth_el": truth_el,
        "separation": sep,
        "az_error": az_err,
        "el_error": el_err,
        "valid_mask": valid_mask,
    }


def _masked_values(values: list[float], mask: list[float] | None = None) -> list[float]:
    if mask is None:
        return _finite_values(values)
    return [
        value
        for value, keep in zip(values, mask)
        if keep > 0.5 and math.isfinite(value)
    ]


def _los_stats(values: dict[str, list[float]], *, valid_only: bool = False) -> dict[str, float]:
    mask = values["valid_mask"] if valid_only else None
    separation = _masked_values(values["separation"], mask)
    az_error = _masked_values(values["az_error"], mask)
    el_error = _masked_values(values["el_error"], mask)
    if not separation:
        return {
            "frames": 0,
            "avg_separation": math.nan,
            "p95_separation": math.nan,
            "max_separation": math.nan,
            "avg_az_error": math.nan,
            "p95_az_error": math.nan,
            "avg_el_error": math.nan,
            "p95_el_error": math.nan,
        }
    return {
        "frames": len(separation),
        "avg_separation": sum(separation) / len(separation),
        "p95_separation": _percentile(separation, 0.95),
        "max_separation": max(separation),
        "avg_az_error": sum(az_error) / len(az_error) if az_error else math.nan,
        "p95_az_error": _percentile(az_error, 0.95),
        "avg_el_error": sum(el_error) / len(el_error) if el_error else math.nan,
        "p95_el_error": _percentile(el_error, 0.95),
    }


def plot_clock02_los_comparisons(
    rows: list[SummaryRow],
    output_dir: Path,
    image_prefix: str = "strapdown_clock0p2",
) -> tuple[dict[str, Path], list[dict[str, float | str]]]:
    output_dir.mkdir(parents=True, exist_ok=True)
    images: dict[str, Path] = {}
    summaries: list[dict[str, float | str]] = []

    for item in rows:
        samples = read_csv_rows(item.csv_path)
        series = _los_comparison_series(samples)
        valid_stats = _los_stats(series, valid_only=True)
        full_stats = _los_stats(series, valid_only=False)
        summaries.append(
            {
                "case": item.case,
                "range": item.start_range_m,
                "total_frames": len(samples),
                "valid_los_frames": valid_stats["frames"],
                "valid_avg_separation": valid_stats["avg_separation"],
                "valid_p95_separation": valid_stats["p95_separation"],
                "valid_avg_az_error": valid_stats["avg_az_error"],
                "valid_p95_az_error": valid_stats["p95_az_error"],
                "valid_avg_el_error": valid_stats["avg_el_error"],
                "valid_p95_el_error": valid_stats["p95_el_error"],
                "full_max_separation": full_stats["max_separation"],
            }
        )

        fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
        ax_az, ax_el, ax_err = axes
        t = series["t"]
        ax_az.plot(t, series["truth_az"], color="tab:blue", linewidth=1.6, label="shadow truth LOS azimuth")
        ax_az.plot(t, series["strap_az"], color="tab:orange", linewidth=1.2, alpha=0.9, label="strapdown image LOS azimuth")
        ax_az.set_title(f"{item.start_range_m:.0f}m LOS angle comparison")
        ax_az.set_ylabel("Azimuth / deg")
        ax_az.grid(True, alpha=0.3)
        ax_az.legend(fontsize=8)

        ax_el.plot(t, series["truth_el"], color="tab:blue", linewidth=1.6, label="shadow truth LOS elevation")
        ax_el.plot(t, series["strap_el"], color="tab:orange", linewidth=1.2, alpha=0.9, label="strapdown image LOS elevation")
        ax_el.set_ylabel("Elevation / deg")
        ax_el.grid(True, alpha=0.3)
        ax_el.legend(fontsize=8)

        ax_err.plot(t, series["separation"], color="tab:red", linewidth=1.3, label="3D LOS separation")
        ax_err.plot(t, series["az_error"], color="tab:purple", linewidth=1.0, alpha=0.8, label="azimuth abs error")
        ax_err.plot(t, series["el_error"], color="tab:green", linewidth=1.0, alpha=0.8, label="elevation abs error")
        ax_err.set_xlabel("Time / s")
        ax_err.set_ylabel("Angle error / deg")
        ax_err.grid(True, alpha=0.3)
        ax_err.legend(fontsize=8)

        fig.tight_layout()
        path = output_dir / f"{image_prefix}_los_compare_{int(round(item.start_range_m)):03d}m.png"
        fig.savefig(path, dpi=170)
        plt.close(fig)
        images[item.case] = path

    return images, summaries


def plot_topdown(rows: list[SummaryRow], output: Path, title: str) -> None:
    cols = 2 if len(rows) > 1 else 1
    grid_rows = max(1, math.ceil(len(rows) / cols))
    fig, axes = plt.subplots(grid_rows, cols, figsize=(5.8 * cols, 4.8 * grid_rows), squeeze=False)
    for ax in axes.flat:
        ax.axis("off")
    for ax, item in zip(axes.flat, rows):
        ax.axis("on")
        samples = read_csv_rows(item.csv_path)
        min_row = _min_range_row(samples)
        ax.plot(_series(samples, "interceptor_x"), _series(samples, "interceptor_y"), color="tab:blue", linewidth=1.8, label="Interceptor")
        ax.plot(_series(samples, "intruder_x"), _series(samples, "intruder_y"), color="tab:red", linewidth=1.8, label="Intruder")
        ax.scatter(_float(samples[0]["interceptor_x"]), _float(samples[0]["interceptor_y"]), color="tab:blue", marker="o", s=25)
        ax.scatter(_float(samples[0]["intruder_x"]), _float(samples[0]["intruder_y"]), color="tab:red", marker="o", s=25)
        ax.scatter(_float(min_row["interceptor_x"]), _float(min_row["interceptor_y"]), color="black", marker="x", s=70, label="Min range")
        ax.set_title(f"{item.start_range_m:.0f}m / {'hit' if item.hit else 'miss'} / min {item.min_range_m:.3f}m")
        ax.set_xlabel("World X / m")
        ax.set_ylabel("World Y / m")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    fig.suptitle(title, y=0.995)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_terminal_errors(rows: list[SummaryRow], output: Path) -> list[dict[str, float | str | bool]]:
    diagnostics = []
    for item in rows:
        samples = read_csv_rows(item.csv_path)
        min_row = _min_range_row(samples)
        diagnostics.append(
            {
                "case": item.case,
                "range": item.start_range_m,
                "hit": item.hit,
                "min_t": _float(min_row.get("t")),
                "min_range": _float(min_row.get("range")),
                "horizontal": _float(min_row.get("horizontal_range")),
                "vertical": _float(min_row.get("vertical_error")),
                "px_x": _float(min_row.get("pixel_error_x")),
                "px_y": _float(min_row.get("pixel_error_y")),
                "terminal_state": min_row.get("terminal_state", ""),
                "guidance_mode": min_row.get("guidance_mode", ""),
                "reject_reason": min_row.get("reject_reason", ""),
            }
        )

    x = list(range(len(diagnostics)))
    labels = [f"{item['range']:.0f}m" for item in diagnostics]
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.8))
    ax_residual, ax_pixel = axes
    width = 0.26
    ax_residual.bar([i - width for i in x], [item["min_range"] for item in diagnostics], width=width, label="3D min range", color="tab:green")
    ax_residual.bar(x, [item["horizontal"] for item in diagnostics], width=width, label="horizontal residual", color="tab:blue")
    ax_residual.bar([i + width for i in x], [abs(item["vertical"]) for item in diagnostics], width=width, label="vertical residual abs", color="tab:orange")
    ax_residual.set_title("Residual geometry at minimum range")
    ax_residual.set_ylabel("Meters")
    ax_residual.set_xticks(x)
    ax_residual.set_xticklabels(labels)
    ax_residual.grid(True, axis="y", alpha=0.3)
    ax_residual.legend(fontsize=8)

    ax_pixel.bar([i - width / 2 for i in x], [item["px_x"] for item in diagnostics], width=width, label="pixel error x", color="tab:purple")
    ax_pixel.bar([i + width / 2 for i in x], [item["px_y"] for item in diagnostics], width=width, label="pixel error y", color="tab:pink")
    ax_pixel.axhline(0.0, color="0.3", linewidth=0.8)
    ax_pixel.set_title("Image-plane error at minimum range")
    ax_pixel.set_ylabel("Pixels")
    ax_pixel.set_xticks(x)
    ax_pixel.set_xticklabels(labels)
    ax_pixel.grid(True, axis="y", alpha=0.3)
    ax_pixel.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)
    return diagnostics


def _markdown_table(rows: list[SummaryRow]) -> str:
    lines = [
        "| 初始水平距离 | 高度差 | 侧向偏置 | 是否碰撞 | 碰撞时间 | 最小距离 | 末端距离 | 检测帧 | 有效帧 |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        hit_t = "-" if row.hit_t_s is None else f"{row.hit_t_s:.2f}s"
        lines.append(
            f"| {row.start_range_m:.0f}m | {row.altitude_offset_m:.0f}m | {row.lateral_offset_m:.0f}m | "
            f"{'是' if row.hit else '否'} | {hit_t} | {row.min_range_m:.3f}m | {row.final_range_m:.3f}m | "
            f"{row.detected_frames}/{row.frames} | {row.valid_frames}/{row.frames} |"
        )
    return "\n".join(lines)


def _terminal_table(diagnostics: list[dict[str, float | str | bool]]) -> str:
    lines = [
        "| 初始水平距离 | 是否碰撞 | 最小距离时刻 | 最小距离 | 水平残差 | 垂直残差 | 像面误差 x/y | 末端状态 | 原因 |",
        "|---:|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    for item in diagnostics:
        lines.append(
            f"| {float(item['range']):.0f}m | {'是' if item['hit'] else '否'} | {float(item['min_t']):.2f}s | "
            f"{float(item['min_range']):.3f}m | {float(item['horizontal']):.3f}m | {float(item['vertical']):.3f}m | "
            f"{float(item['px_x']):.1f}/{float(item['px_y']):.1f}px | {item['terminal_state']} | {item['reject_reason']} |"
        )
    return "\n".join(lines)


def _clock02_table(rows: list[SummaryRow], truth_index: dict[str, dict[str, str]] | None = None) -> str:
    truth_index = truth_index or {}
    lines = [
        "| 初始水平距离 | 是否碰撞 | 碰撞时间 | 最小距离 | 检测帧 | 有效帧 | 平均仿真FPS | 最大实际过载 | 视觉指令最大等效过载 | 视觉指令P95等效过载 | 理论最大需用过载 | 理论P95需用过载 |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        hit_t = "-" if row.hit_t_s is None else f"{row.hit_t_s:.2f}s"
        sim_fps = "-" if not _finite_summary(row.avg_sim_sample_fps) else f"{row.avg_sim_sample_fps:.2f}"
        visual_required = required_load_stats(read_csv_rows(row.csv_path))
        theory_max = _truth_required_stat(row, truth_index, "max_theory_n_req_g")
        theory_p95 = _truth_required_stat(row, truth_index, "p95_theory_n_req_g")
        lines.append(
            f"| {row.start_range_m:.0f}m | {'是' if row.hit else '否'} | {hit_t} | "
            f"{row.min_range_m:.3f}m | "
            f"{row.detected_frames}/{row.frames} | {row.valid_frames}/{row.frames} | "
            f"{sim_fps} | {_format_g(row.max_load_factor_fd_g)} | "
            f"{_format_g(visual_required['max'])} | {_format_g(visual_required['p95'])} | "
            f"{_format_g(theory_max)} | {_format_g(theory_p95)} |"
        )
    return "\n".join(lines)


def _clock02_theory_log_files(
    truth_index: dict[str, dict[str, str]],
    truth_summary_path: Path = TRUTH_REQUIRED_SUMMARY_PATH,
) -> str:
    if not truth_index:
        return "- 理论需用过载导出：未找到"
    output_files = sorted({row.get("output_csv", "") for row in truth_index.values() if row.get("output_csv")})
    lines = [f"- 理论需用过载汇总：`{truth_summary_path.relative_to(PROJECT_ROOT).as_posix()}`"]
    if output_files:
        lines.append("- 理论需用过载逐帧文件：")
        for item in output_files:
            lines.append(f"  - `{item}`")
    return "\n".join(lines)


def _per_distance_image_list(rows: list[SummaryRow], images: dict[str, Path]) -> str:
    lines: list[str] = []
    for row in rows:
        path = images.get(row.case)
        if path is None:
            continue
        lines.append(f"### {row.start_range_m:.0f}m")
        lines.append("")
        lines.append(f"![{row.start_range_m:.0f}m 过载对比]({_rel_asset_for(path, CLOCK02_REPORT_PATH)})")
        lines.append("")
    return "\n".join(lines)


def _per_case_curve_sections(
    rows: list[SummaryRow],
    load_images: dict[str, Path],
    los_images: dict[str, Path],
    report_path: Path = CLOCK02_REPORT_PATH,
) -> str:
    lines: list[str] = []
    for row in rows:
        lines.append(f"### {row.start_range_m:.0f}m 工况")
        lines.append("")
        load_path = load_images.get(row.case)
        if load_path is not None:
            lines.append("过载曲线：")
            lines.append("")
            lines.append(f"![{row.start_range_m:.0f}m 过载曲线]({_rel_asset_for(load_path, report_path)})")
            lines.append("")
        los_path = los_images.get(row.case)
        if los_path is not None:
            lines.append("视线角曲线：")
            lines.append("")
            lines.append(f"![{row.start_range_m:.0f}m 视线角曲线]({_rel_asset_for(los_path, report_path)})")
            lines.append("")
    return "\n".join(lines)


def _los_summary_table(summaries: list[dict[str, float | str]]) -> str:
    lines = [
        "| 初始水平距离 | 有效LOS帧 | 有效平均LOS夹角误差 | 有效P95 LOS夹角误差 | 有效平均方位误差 | 有效P95方位误差 | 有效平均俯仰误差 | 有效P95俯仰误差 | 全程最大LOS夹角误差 |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in sorted(summaries, key=lambda row: float(row["range"])):
        lines.append(
            f"| {float(item['range']):.0f}m | "
            f"{int(float(item['valid_los_frames']))}/{int(float(item['total_frames']))} | "
            f"{float(item['valid_avg_separation']):.2f}deg | {float(item['valid_p95_separation']):.2f}deg | "
            f"{float(item['valid_avg_az_error']):.2f}deg | {float(item['valid_p95_az_error']):.2f}deg | "
            f"{float(item['valid_avg_el_error']):.2f}deg | {float(item['valid_p95_el_error']):.2f}deg | "
            f"{float(item['full_max_separation']):.2f}deg |"
        )
    return "\n".join(lines)


def _source_count_text(samples: list[dict[str, str]]) -> str:
    counts: dict[str, int] = {}
    for row in samples:
        source = row.get("los_source", "")
        if source:
            counts[source] = counts.get(source, 0) + 1
    if not counts:
        return "-"
    return ", ".join(f"{key}:{value}" for key, value in sorted(counts.items()))


def _raw_los_diagnostic_table(rows: list[SummaryRow]) -> str:
    has_raw_fields = False
    stats: list[dict[str, float | str]] = []
    for row in rows:
        samples = read_csv_rows(row.csv_path)
        if any("omega_raw_norm_rad_s" in sample for sample in samples):
            has_raw_fields = True
        raw_omega = _finite_values(_float(sample.get("omega_raw_norm_rad_s")) for sample in samples)
        effective_omega = _finite_values(_float(sample.get("omega_effective_norm_rad_s")) for sample in samples)
        angle_step = _finite_values(_float(sample.get("los_angle_step_deg")) for sample in samples)
        los_dt = _finite_values(_float(sample.get("los_dt_s")) for sample in samples)
        stats.append(
            {
                "range": row.start_range_m,
                "sources": _source_count_text(samples),
                "max_raw_omega": max(raw_omega) if raw_omega else math.nan,
                "p95_raw_omega": _percentile(raw_omega, 0.95),
                "max_effective_omega": max(effective_omega) if effective_omega else math.nan,
                "max_angle_step": max(angle_step) if angle_step else math.nan,
                "p95_angle_step": _percentile(angle_step, 0.95),
                "median_los_dt_ms": 1000.0 * _percentile(los_dt, 0.50),
            }
        )
    if not has_raw_fields:
        return ""

    lines = [
        "### 原始 LOS 差分诊断",
        "",
        "本节直接读取日志中的 `omega_raw_norm_rad_s`、`los_angle_step_deg` 和 `los_dt_s`。在关闭 6D LOS Kalman 滤波后，`omega_raw_norm_rad_s` 就是图像 PNG 实际使用的 LOS 角速度模长，因此它能直接暴露有限差分在末端穿越、裁切、失锁和重捕获时产生的尖峰。",
        "",
        "| 初始水平距离 | LOS来源帧数 | 最大原始LOS角速度 | P95原始LOS角速度 | 最大有效LOS角速度 | 最大单帧LOS角步长 | P95单帧LOS角步长 | 中位LOS dt |",
        "|---:|---|---:|---:|---:|---:|---:|---:|",
    ]
    for item in sorted(stats, key=lambda value: float(value["range"])):
        lines.append(
            f"| {float(item['range']):.0f}m | {item['sources']} | "
            f"{float(item['max_raw_omega']):.2f}rad/s | {float(item['p95_raw_omega']):.2f}rad/s | "
            f"{float(item['max_effective_omega']):.2f}rad/s | "
            f"{float(item['max_angle_step']):.2f}deg | {float(item['p95_angle_step']):.2f}deg | "
            f"{float(item['median_los_dt_ms']):.1f}ms |"
        )
    lines.extend(
        [
            "",
            "可以看到，即使 AirSim detection 给出的是理论检测框，原始 LOS 差分仍会在末端产生很高的角速度尖峰。根本原因不是检测框抖动，而是几何本身在近距穿越时 LOS 方向变化极快；再叠加固定相机出视场、bbox 裁切、`valid/invalid` 状态切换以及仿真离散采样，有限差分会把单帧角步长直接放大成 `rad/s` 量级的角速度脉冲。6D LOS Kalman 滤波的作用之一正是限制这种离散微分噪声，但代价是会引入相位滞后。",
            "",
        ]
    )
    return "\n".join(lines)


def _hit_rate(rows: list[SummaryRow]) -> str:
    if not rows:
        return "0/0"
    return f"{sum(1 for row in rows if row.hit)}/{len(rows)}"


def _range_text(rows: list[SummaryRow]) -> str:
    return ", ".join(f"{row.start_range_m:.0f}m" for row in rows) if rows else "-"


def _summary_file_list(summary_files: list[Path]) -> str:
    lines = []
    for path in summary_files:
        lines.append(f"  - `{path.relative_to(PROJECT_ROOT).as_posix()}`")
    return "\n".join(lines)


def write_report(
    all_rows: list[SummaryRow],
    near_rows: list[SummaryRow],
    far_rows: list[SummaryRow],
    near_diagnostics: list[dict[str, float | str | bool]],
    far_diagnostics: list[dict[str, float | str | bool]],
    images: dict[str, Path],
    summary_files: list[Path],
) -> None:
    first_meta = read_meta((near_rows or far_rows or all_rows)[0].meta_path)
    args = first_meta.get("args", {})
    near_command = (
        "python3 examples/batch_strapdown_accuracy.py --ranges 30 40 50 "
        "--altitude-offsets 20 --duration-s 18 --intruder-speed 5 --speed-ratio 2 "
        "--rate-hz 20 --print-every-n 0 --trajectory-dir logs/strapdown_accuracy"
    )
    far_command = (
        "python3 examples/batch_strapdown_accuracy.py --ranges 130 140 150 160 "
        "--altitude-offsets 20 --duration-s 32 --intruder-speed 5 --speed-ratio 2 "
        "--rate-hz 20 --print-every-n 0 --trajectory-dir logs/strapdown_accuracy"
    )
    report = f"""# 捷联视觉 PNG 无头距离测试报告

## 1. 测试目的

本报告整理 AirSim Blocks 中捷联视觉 PNG 的无头批量测试结果。测试只使用捷联相机方案，不打开 OpenCV 界面，不保存检测截图，不生成仿真窗口录屏；本文图片均由 CSV 实验日志离线绘制。

成功判据采用 AirSim 双机碰撞检测；真值距离、水平残差和垂直残差只用于算法评价，不参与 PNG 内部导引。

## 2. 测试条件

- 测试对象：`examples/run_airsim_strapdown_vision_png.py`
- 批量脚本：`examples/batch_strapdown_accuracy.py`
- AirSim 配置：`config/airsim_blocks_settings.json`，`ViewMode=NoDisplay`
- 相机视场角：`{float(args.get('fov_deg', 120.0)):.0f} deg`
- 拦截机起始高度：`{float(args.get('intercept_altitude_m', 50.0)):.0f} m`
- 入侵机速度：`{float(args.get('intruder_speed', 5.0)):.1f} m/s`
- 速度比：`{float(args.get('speed_ratio', 2.0)):.1f}`
- 侧向偏置：`-20 m`
- 测试距离组 A：`30m, 40m, 50m`
- 测试距离组 B：`130m, 140m, 150m, 160m`

距离组 A 运行命令：

```bash
{near_command}
```

距离组 B 运行命令：

```bash
{far_command}
```

## 3. 结果总览

![总体结果]({_rel_asset(images['all_summary'])})

{_markdown_table(all_rows)}

## 4. 30-50m 工况

![30-50m 距离曲线]({_rel_asset(images['near_range'])})

![30-50m 俯视轨迹]({_rel_asset(images['near_topdown'])})

{_markdown_table(near_rows)}

![30-50m 末端误差]({_rel_asset(images['near_terminal_errors'])})

{_terminal_table(near_diagnostics)}

本距离组用于观察捷联固定相机在对应初始距离下的 LOS+TTC 收敛、检测连续性和末端外推表现。当前命中率为 `{_hit_rate(near_rows)}`，测试距离为 `{_range_text(near_rows)}`。未碰撞工况需要结合最小距离、检测帧比例、末端状态和残差共同判断；如果最小距离已经进入亚米级，主要问题通常集中在末端裁切、盲推时长、垂直偏置和 AirSim 碰撞体交叠判据。

## 5. 130-160m 工况

![130-160m 距离曲线]({_rel_asset(images['far_range'])})

![130-160m 俯视轨迹]({_rel_asset(images['far_topdown'])})

{_markdown_table(far_rows)}

![130-160m 末端误差]({_rel_asset(images['far_terminal_errors'])})

{_terminal_table(far_diagnostics)}

本距离组用于观察捷联固定相机在对应初始距离下的 LOS+TTC 收敛、检测连续性和末端外推表现。当前命中率为 `{_hit_rate(far_rows)}`，测试距离为 `{_range_text(far_rows)}`。未碰撞工况需要结合最小距离、检测帧比例、末端状态和残差共同判断；如果最小距离已经进入亚米级，主要问题通常集中在末端裁切、盲推时长、垂直偏置和 AirSim 碰撞体交叠判据。

## 6. 结论和后续调参方向

从现有批量测试看，捷联视觉 PNG 在多数距离下可以把目标导入亚米级最小距离。未触发碰撞的工况需要结合最小距离、水平残差、垂直残差和末端状态一起判断；如果最小距离已经接近或小于 `0.5m`，失败更可能来自末端视觉裁切后的 BlindPush 外推、上下方向偏置不足或碰撞体交叠不足，而不是 LOS 中段估计完全失败。

后续调参优先级：

1. 先调 `BlindPush` 的持续时间和衰减，避免过早退出或保持过久。
2. 再调末端 pitch-up / 垂直方向偏置，使最小距离时刻的垂直残差进一步收敛。
3. 最后调 TTC 增益和 LOS 像面 KF 外推，降低 `bbox_clipped` 后的像面残差。

## 7. 日志文件

- 总汇总：`logs/strapdown_accuracy/strapdown_headless_all_summary.csv`
自动纳入的批次汇总：
{_summary_file_list(summary_files)}
"""
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(report, encoding="utf-8")


def write_clock02_report(
    rows: list[SummaryRow],
    diagnostics: list[dict[str, float | str | bool]],
    images: dict[str, Path],
    per_distance_images: dict[str, Path],
    los_images: dict[str, Path],
    los_summaries: list[dict[str, float | str]],
    summary_files: list[Path],
    truth_index: dict[str, dict[str, str]],
    *,
    report_path: Path = CLOCK02_REPORT_PATH,
    summary_path: Path = CLOCK02_SUMMARY_PATH,
    truth_summary_path: Path = TRUTH_REQUIRED_SUMMARY_PATH,
    title: str = "捷联视觉 PNG ClockSpeed 0.2 距离与过载测试报告",
    command: str | None = None,
    los_filter_condition: str = "开启（6D Kalman）",
    los_method_note: str = (
        "本报告中的捷联图像 LOS 经过 6D LOS Kalman 滤波后再用于导引，"
        "因此 `lambda_x/lambda_y/lambda_z` 和 `omega_x/omega_y/omega_z` 均代表滤波后的有效导引量。"
    ),
) -> None:
    first_meta = read_meta(rows[0].meta_path)
    args = first_meta.get("args", {})
    command_text = command or (
        "python3 examples/batch_strapdown_accuracy.py --ranges 30 40 50 60 70 80 90 100 110 120 130 140 150 160 "
        "--altitude-offsets 20 --duration-s 35 --intruder-speed 5 --speed-ratio 2 "
        "--rate-hz 20 --print-every-n 0 --prefix strapdown_clock0p2_simtime_$(date +%Y%m%d_%H%M%S) "
        "--trajectory-dir logs/strapdown_accuracy"
    )
    hit_count = sum(1 for row in rows if row.hit)
    finite_min_ranges = [row.min_range_m for row in rows if _finite_summary(row.min_range_m)]
    finite_loads = [row.max_load_factor_fd_g for row in rows if _finite_summary(row.max_load_factor_fd_g)]
    finite_sim_fps = [row.avg_sim_sample_fps for row in rows if _finite_summary(row.avg_sim_sample_fps)]
    visual_required_stats = [required_load_stats(read_csv_rows(row.csv_path)) for row in rows]
    finite_visual_required_max = [item["max"] for item in visual_required_stats if _finite_summary(item["max"])]
    finite_visual_required_p95 = [item["p95"] for item in visual_required_stats if _finite_summary(item["p95"])]
    finite_theory_required_max = [
        _truth_required_stat(row, truth_index, "max_theory_n_req_g") for row in rows
    ]
    finite_theory_required_max = [value for value in finite_theory_required_max if _finite_summary(value)]
    finite_theory_required_p95 = [
        _truth_required_stat(row, truth_index, "p95_theory_n_req_g") for row in rows
    ]
    finite_theory_required_p95 = [value for value in finite_theory_required_p95 if _finite_summary(value)]
    report = f"""# {title}

## 1. 测试目的

本报告整理 AirSim Blocks 在 `ClockSpeed=0.2` 下的捷联视觉 PNG 批量仿真结果。测试距离覆盖 `30-160m`，每次飞行记录真值距离、检测可用性、AirSim 碰撞结果、视觉速度指令等效过载、真值 PNG 理论需用过载、实际过载和仿真帧率。

成功判据仍采用 AirSim 双机碰撞检测；真值位置、最小距离和过载只用于离线评价，不参与捷联视觉 PNG 内部导引。真值 PNG 理论需用过载是假设“已知真实相对位置和速度”后事后计算得到，用于和视觉控制输出进行对照。

## 2. 测试条件

- 测试对象：`examples/run_airsim_strapdown_vision_png.py`
- 批量脚本：`examples/batch_strapdown_accuracy.py`
- AirSim 配置：`config/airsim_blocks_settings.json`
- 仿真时钟：`ClockSpeed=0.2`
- 显示模式：`ViewMode=NoDisplay`
- LOS Kalman 滤波：{los_filter_condition}
- 相机视场角：`{float(args.get('fov_deg', 120.0)):.0f} deg`
- 拦截机起始高度：`{float(args.get('intercept_altitude_m', 50.0)):.0f} m`
- 入侵机高度差：`20 m`
- 入侵机速度：`{float(args.get('intruder_speed', 5.0)):.1f} m/s`
- 速度比：`{float(args.get('speed_ratio', 2.0)):.1f}`
- 侧向偏置：`-20 m`

运行命令：

```bash
{command_text}
```

## 3. 结果总览

![总体结果]({_rel_asset_for(images['summary'], report_path)})

{_clock02_table(rows, truth_index)}

## 4. 距离与轨迹

![距离曲线]({_rel_asset_for(images['range'], report_path)})

![俯视轨迹]({_rel_asset_for(images['topdown'], report_path)})

## 5. 需用过载、理论过载、实际过载与帧率

![过载和帧率]({_rel_asset_for(images['load_fps'], report_path)})

本报告区分三类过载：

- 视觉速度指令等效过载：由速度指令变化量离线计算，`n_cmd = ||Δv_cmd / Δt_sim|| / g`。因为当前捷联程序向 AirSim 发送速度设定值，所以该指标用于评价视觉 PNG、LossHold 和末端外推状态机对飞控提出的瞬时速度变化需求。
- 真值 PNG 理论需用过载：假设已知目标真实相对位置和速度，用经典三维 PNG 公式计算，`n_theory = ||N * Vc * lambda_dot|| / g`，本报告使用 `N=3.0`。该指标不使用视觉检测框，也不使用视觉状态机输出，用于表示同一几何轨迹下理想比例导引本身需要的机动强度。
- 实际过载：由拦截机真值速度有限差分计算，`n_act = ||Δv_truth / Δt_sim|| / g`。`load_factor_g` 也被记录，但 SimpleFlight 在部分版本中真值线加速度字段可能长期为零或更新不稳定，所以表格和曲线优先使用 `load_factor_fd_g`。

视觉速度指令最大等效过载用于暴露 BlindPush 进入/退出、LossHold、Complete 和速度指令重置带来的单帧尖峰；P95 更适合评价大部分飞行过程中的持续控制需求。真值 PNG 理论过载则用于回答“如果目标位置和速度完全已知，比例导引理论上需要多大过载”。

## 6. 各工况曲线汇总

本节按初始水平距离组织曲线。同一个工况下依次放置过载曲线和视线角曲线，便于直接对照该工况中的导引需求、无人机响应和 LOS 偏差。

过载曲线包含三条过载曲线；图下方的距离曲线只用于标注该工况的交会进程，不是轨迹图：

- `image-PNG theoretical/equivalent load`：捷联图像 PNG 输出速度指令的一阶差分等效过载，代表图像 PNG 对飞控提出的理论机动需求。
- `strapdown UAV actual load`：捷联视觉程序实际驱动无人机后，由无人机真值速度有限差分得到的实际过载。
- `shadow truth-position theoretical load`：影子测试曲线，在同一捷联实验轨迹上假设已知入侵机真实位置和速度，离线计算经典 PNG 理论需用过载。

视线角曲线包含两条 LOS 角度曲线和误差曲线：

- 捷联图像 LOS：来自捷联视觉程序日志中的 `lambda_x/lambda_y/lambda_z`，代表图像 PNG 实际用于导引的惯性系视线方向。
- 影子真值 LOS：使用同一时刻 `intruder_position - camera_world_position` 计算，即从拦截机相机光心指向入侵机原点，只用于离线评价，不参与捷联导引。新日志直接读取 `camera_world_x/y/z`；旧日志没有相机世界坐标时，用 `interceptor_position + yaw(camera_x,camera_y,camera_z)` 回算固定相机光心，因此仍能扣除主要的相机安装偏移。

{los_method_note}

方位角按惯性系水平面 `atan2(y, x)` 计算，俯仰角按 NED 坐标中的 `atan2(-z, horizontal)` 计算。`LOS夹角误差` 是两条三维单位视线向量的夹角。表格中的平均值和 P95 只统计 `valid=1` 的有效导引帧；全程最大值保留失锁、裁切和穿越后的极端偏离，作为异常诊断参考。

{_los_summary_table(los_summaries)}

{_raw_los_diagnostic_table(rows)}

{_per_case_curve_sections(rows, per_distance_images, los_images, report_path)}

## 7. 视觉速度指令等效过载尖峰原因分析

本批日志中，视觉速度指令最大等效过载明显高于真值 PNG 理论需用过载。真值 PNG 理论最大值约为 `{_format_range(finite_theory_required_max, "g")}`，而视觉速度指令最大等效过载达到 `{_format_range(finite_visual_required_max, "g")}`。这说明高尖峰主要不是比例导引理论本身造成的，而是少数离散帧上的速度指令阶跃造成的。

由于视觉速度指令等效过载按 `n_cmd = ||Δv_cmd / Δt_sim|| / g` 计算，在 `ClockSpeed=0.2` 的日志中相邻仿真时间步约为 `0.01s`，即使 `v_cmd` 只发生 `0.5m/s` 量级变化，也会被换算为约 `5g` 的瞬时等效过载；如果状态切换导致 `v_cmd` 一帧内变化 `2-5m/s`，表格中的最大值就会达到几十 g。

本批测试使用 AirSim 内置 detection 函数输出理论检测框，因此检测框本身不代表真实 YOLO 噪声模型。报告中的 `detected=0` 主要表示固定相机几何下目标离开视场、目标框被严重裁切、穿越后相机几何失效，或 AirSim detection 在该帧没有返回匹配目标，不应简单理解为神经网络随机漏检。

对于关闭 LOS 滤波的无噪声仿真，速度指令尖峰还会来自原始 LOS 有限差分本身：近距穿越时 LOS 单位向量的单帧角步长会快速增大，`omega_LOS = lambda x lambda_dot` 会把这种角步长除以约 `10ms` 的仿真采样间隔，形成几十 `rad/s` 的瞬时角速度。也就是说，检测框不抖并不等于微分后的 LOS 角速度没有尖峰；滤波关闭后，尖峰会更直接地进入 `g_eval` 和速度指令。

从日志抽查看，尖峰主要集中在以下场景：

1. 初始接管阶跃。部分工况第 0 帧仍是默认前向速度，例如 `[10, 0, 0]`，第 1 帧进入视觉导引后变成带横向和垂向分量的速度，例如 `[8.29, -3.96, -3.95]`。两帧相隔约 `0.009s`，因此会出现 `60g` 量级的速度指令等效过载。
2. `Tracking / LossHold / BlindPush / Complete` 之间切换。典型情况是 `ttc_png -> invalid`、`invalid -> ttc_png`、`blind_push -> invalid` 或 `invalid -> blind_push`，上层速度指令从视觉 PNG 输出、末端盲推输出和保持输出之间切换，造成 `v_cmd` 不连续。
3. 固定相机末端出视场和严重裁切。重捕获或切换到 LossHold/BlindPush 时，速度指令容易发生单帧台阶。
4. `BlindPush` 结束后的回退。部分工况出现 `BlindPush -> Complete -> invalid` 后又因末端几何状态重新进入 `BlindPush` 的往复过程，横向速度和垂直速度分量会在短时间内反复切换。
5. 垂向速度限幅和末端偏置切换。捷联系统为了补偿高度差加入了垂直速度项，末端裁切后该项可能从正常限幅值跳到保持/盲推状态的历史值，`v_cmd_z` 的突变会显著抬高 `n_cmd`。

因此，视觉速度指令最大等效过载应被视为“指令连续性诊断指标”，不应直接等同于飞机真实承受的机体过载。后续优化应优先在上层指令输出端加入连续化处理：对 `v_cmd` 做 slew-rate limiter；初始接管阶段用 `0.2-0.5s` 淡入视觉修正；`LossHold` 保留上一帧完整速度并指数衰减；`BlindPush` 退出时平滑过渡到默认/重捕获控制；检测重获后用 `0.1-0.2s` 淡入 `ttc_png` 修正量。

## 8. 实际捷联过载偏大的原因

实际过载来自拦截机真值速度有限差分，定义为 `n_act = ||Δv_truth / Δt_sim|| / g`。多数距离工况的最大实际过载约为 `0.9g`，但 40m 和 50m 工况出现了 `20g+` 的离散尖峰。结合日志看，原因主要有三类：

1. 碰撞/近距接触导致速度状态不连续。40m 工况的最大实际过载出现在碰撞帧附近，最小距离约 `1.03m`，AirSim 碰撞体接触和物理求解会让真值速度在有限差分中出现尖峰。
2. AirSim SimpleFlight / RPC 状态离散更新造成有限差分放大。实际过载不是飞控内部连续加速度，而是用相邻日志帧的真值速度差计算。若 AirSim 在某几帧对位置或速度做了离散修正，相邻 `0.01s` 的差分会把这种修正放大成十几到几十 g。
3. 速度控制模式与物理状态不同步。脚本发送的是 `moveByVelocityAsync` 速度设定值，SimpleFlight 会用内部控制器追踪速度。若上一段命令、碰撞接触、姿态/高度控制或位置修正造成速度状态突然变化，有限差分实际过载会出现孤立尖峰，即使当前 `v_cmd` 已保持不变。

因此，实际过载中的 `20g+` 峰值更适合作为“仿真物理/离散采样异常点”排查指标，而不是直接等同于真实穿越机可承受或实际产生的持续机动过载。评价持续机动强度时，应优先同时查看 P95、峰值所在帧、是否碰撞、是否近距穿越以及该帧前后的速度状态。

## 9. 末端误差

![末端误差]({_rel_asset_for(images['terminal'], report_path)})

{_terminal_table(diagnostics)}

## 10. 结论

- 本批共 `{len(rows)}` 组，AirSim 碰撞命中 `{hit_count}` 组。
- 最小距离范围：`{_format_meter_range(finite_min_ranges)}`。
- 最大有限差分过载范围：`{_format_range(finite_loads, "g")}`。
- 视觉速度指令最大等效过载范围：`{_format_range(finite_visual_required_max, "g")}`。
- 视觉速度指令 P95 等效过载范围：`{_format_range(finite_visual_required_p95, "g")}`。
- 真值 PNG 理论最大需用过载范围：`{_format_range(finite_theory_required_max, "g")}`。
- 真值 PNG 理论 P95 需用过载范围：`{_format_range(finite_theory_required_p95, "g")}`。
- 平均仿真采样 FPS 范围：`{_format_range(finite_sim_fps, "Hz")}`。

在 `ClockSpeed=0.2` 下，仿真以更慢的时钟推进，日志中的 `avg_wall_fps` 反映 Python 控制循环实际刷新率，`avg_sim_sample_fps` 反映 AirSim 状态时间戳推进后的采样频率。两者需要一起看：如果墙钟 FPS 正常但仿真 FPS 较低，说明仿真时钟确实被减速；如果墙钟 FPS 也明显下降，则要优先检查渲染、检测调用和 RPC 延迟。

## 11. 日志文件

- 总汇总：`{summary_path.relative_to(PROJECT_ROOT).as_posix()}`
自动纳入的批次汇总：
{_summary_file_list(summary_files)}

{_clock02_theory_log_files(truth_index, truth_summary_path)}
"""
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(report, encoding="utf-8")


def generate_clock02_report_variant(
    *,
    prefix: str,
    summary_path: Path,
    asset_dir: Path,
    report_path: Path,
    truth_summary_path: Path,
    image_prefix: str,
    title: str,
    command: str,
    los_filter_condition: str,
    los_method_note: str,
) -> None:
    asset_dir.mkdir(parents=True, exist_ok=True)
    rows, summary_files = read_all_batch_summaries(prefix, summary_path)
    rows = [row for row in rows if row.altitude_offset_m == 20.0 and 30.0 <= row.start_range_m <= 160.0]
    if not rows:
        raise SystemExit(f"no rows found for prefix {prefix!r}")
    truth_index = read_truth_required_index(truth_summary_path)
    images = {
        "summary": asset_dir / f"{image_prefix}_summary.png",
        "range": asset_dir / f"{image_prefix}_range_curves.png",
        "topdown": asset_dir / f"{image_prefix}_topdown.png",
        "load_fps": asset_dir / f"{image_prefix}_load_fps.png",
        "terminal": asset_dir / f"{image_prefix}_terminal_errors.png",
    }
    plot_clock02_summary(rows, images["summary"], truth_index)
    plot_range_curves(rows, images["range"], "ClockSpeed 0.2 range curves")
    plot_topdown(rows, images["topdown"], "Top-down trajectories, ClockSpeed 0.2")
    plot_load_factor_curves(rows, images["load_fps"], truth_index)
    per_distance_images = plot_clock02_per_distance_loads(rows, asset_dir, truth_index, image_prefix=image_prefix)
    los_images, los_summaries = plot_clock02_los_comparisons(rows, asset_dir, image_prefix=image_prefix)
    diagnostics = plot_terminal_errors(rows, images["terminal"])
    write_clock02_report(
        rows,
        diagnostics,
        images,
        per_distance_images,
        los_images,
        los_summaries,
        summary_files,
        truth_index,
        report_path=report_path,
        summary_path=summary_path,
        truth_summary_path=truth_summary_path,
        title=title,
        command=command,
        los_filter_condition=los_filter_condition,
        los_method_note=los_method_note,
    )
    print(f"report={report_path}")
    for image in images.values():
        print(f"image={image}")
    for image in per_distance_images.values():
        print(f"image={image}")
    for image in los_images.values():
        print(f"image={image}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate strapdown AirSim accuracy reports.")
    parser.add_argument("--clock02", action="store_true", help="Generate the ClockSpeed=0.2 30-160m report.")
    parser.add_argument(
        "--clock02-no-los-filter",
        action="store_true",
        help="Generate the ClockSpeed=0.2 report for strapdown runs with --no-los-filter.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.clock02:
        generate_clock02_report_variant(
            prefix=CLOCK02_PREFIX,
            summary_path=CLOCK02_SUMMARY_PATH,
            asset_dir=CLOCK02_ASSET_DIR,
            report_path=CLOCK02_REPORT_PATH,
            truth_summary_path=TRUTH_REQUIRED_SUMMARY_PATH,
            image_prefix="strapdown_clock0p2",
            title="捷联视觉 PNG ClockSpeed 0.2 距离与过载测试报告",
            command=(
                "python3 examples/batch_strapdown_accuracy.py --ranges 30 40 50 60 70 80 90 100 110 120 130 140 150 160 "
                "--altitude-offsets 20 --duration-s 35 --intruder-speed 5 --speed-ratio 2 "
                "--rate-hz 20 --print-every-n 0 --prefix strapdown_clock0p2_simtime_$(date +%Y%m%d_%H%M%S) "
                "--trajectory-dir logs/strapdown_accuracy"
            ),
            los_filter_condition="开启（6D Kalman）",
            los_method_note=(
                "本报告中的捷联图像 LOS 经过 6D LOS Kalman 滤波后再用于导引，"
                "因此 `lambda_x/lambda_y/lambda_z` 和 `omega_x/omega_y/omega_z` 均代表滤波后的有效导引量。"
            ),
        )
        return
    if args.clock02_no_los_filter:
        generate_clock02_report_variant(
            prefix=CLOCK02_NO_LOS_PREFIX,
            summary_path=CLOCK02_NO_LOS_SUMMARY_PATH,
            asset_dir=CLOCK02_NO_LOS_ASSET_DIR,
            report_path=CLOCK02_NO_LOS_REPORT_PATH,
            truth_summary_path=TRUTH_REQUIRED_NO_LOS_SUMMARY_PATH,
            image_prefix="strapdown_clock0p2_no_los_filter",
            title="捷联视觉 PNG ClockSpeed 0.2 无LOS滤波距离与过载测试报告",
            command=(
                "python3 examples/batch_strapdown_accuracy.py --ranges 30 40 50 60 70 80 90 100 110 120 130 140 150 160 "
                "--altitude-offsets 20 --duration-s 28 --intruder-speed 5 --speed-ratio 2 "
                "--rate-hz 20 --print-every-n 0 --prefix strapdown_clock0p2_no_los_filter_$(date +%Y%m%d_%H%M%S) "
                "--trajectory-dir logs/strapdown_accuracy -- --no-los-filter"
            ),
            los_filter_condition="关闭；`lambda_x/lambda_y/lambda_z` 直接使用时间对齐后的原始测量 LOS",
            los_method_note=(
                "本报告关闭 6D LOS Kalman 滤波。捷联图像 LOS 中的 `lambda_x/lambda_y/lambda_z` "
                "等同于姿态去旋转后的原始测量 LOS；`omega_x/omega_y/omega_z` 由相邻原始 LOS 单位向量按仿真时间有限差分得到，"
                "并投影到垂直 LOS 的切平面。日志额外保留 `lambda_raw_*`、`omega_raw_*`、`los_dt_s` 和 "
                "`los_angle_step_deg`，用于检查无噪声仿真下原始视线角速度是否仍存在离散采样尖峰。"
            ),
        )
        return

    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    all_rows, summary_files = read_all_batch_summaries()
    near_rows = [
        row for row in all_rows if row.altitude_offset_m == 20.0 and 30.0 <= row.start_range_m <= 50.0
    ]
    far_rows = [
        row for row in all_rows if row.altitude_offset_m == 20.0 and 130.0 <= row.start_range_m <= 160.0
    ]
    images = {
        "all_summary": ASSET_DIR / "strapdown_all_summary.png",
        "near_range": ASSET_DIR / "strapdown_near_range_curves.png",
        "near_topdown": ASSET_DIR / "strapdown_near_topdown.png",
        "near_terminal_errors": ASSET_DIR / "strapdown_near_terminal_errors.png",
        "far_range": ASSET_DIR / "strapdown_far_range_curves.png",
        "far_topdown": ASSET_DIR / "strapdown_far_topdown.png",
        "far_terminal_errors": ASSET_DIR / "strapdown_far_terminal_errors.png",
    }
    plot_all_summary(all_rows, images["all_summary"])
    plot_range_curves(near_rows, images["near_range"], "30-50m range curves")
    plot_topdown(near_rows, images["near_topdown"], "Top-down trajectories for 30-50m runs")
    near_diagnostics = plot_terminal_errors(near_rows, images["near_terminal_errors"])
    plot_range_curves(far_rows, images["far_range"], "130-160m range curves")
    plot_topdown(far_rows, images["far_topdown"], "Top-down trajectories for 130-160m runs")
    far_diagnostics = plot_terminal_errors(far_rows, images["far_terminal_errors"])
    write_report(all_rows, near_rows, far_rows, near_diagnostics, far_diagnostics, images, summary_files)
    print(f"report={REPORT_PATH}")
    for image in images.values():
        print(f"image={image}")


if __name__ == "__main__":
    main()
