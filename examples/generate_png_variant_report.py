from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = PROJECT_ROOT / "logs" / "png_accuracy"
REPORT_PATH = PROJECT_ROOT / "完整方案" / "真值与云台ClockSpeed0p2过载测试报告.md"
ASSET_DIR = PROJECT_ROOT / "完整方案" / "assets" / "真值与云台ClockSpeed0p2过载测试报告"
GRAVITY_MPS2 = 9.80665


@dataclass
class Row:
    scenario: str
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


def _finite(value: float) -> bool:
    return math.isfinite(value)


def _finite_values(values: Iterable[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def _percentile(values: Iterable[float], ratio: float) -> float:
    finite = sorted(_finite_values(values))
    if not finite:
        return math.nan
    ratio = max(0.0, min(1.0, ratio))
    index = int(round((len(finite) - 1) * ratio))
    return finite[index]


def read_summary(path: Path) -> list[Row]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = []
        for item in csv.DictReader(stream):
            scenario = item.get("scenario") or ("truth" if "truth" in item.get("case", "") else "gimbal")
            rows.append(
                Row(
                    scenario=scenario,
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
    return sorted(rows, key=lambda row: (row.scenario, row.start_range_m))


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _series(rows: Iterable[dict[str, str]], key: str) -> list[float]:
    return [_float(row.get(key)) for row in rows]


def _vector_from_row(row: dict[str, str], keys: tuple[str, str, str]) -> tuple[float, float, float] | None:
    values = tuple(_float(row.get(key)) for key in keys)
    if not all(math.isfinite(value) for value in values):
        return None
    return values


def required_load_series(rows: list[dict[str, str]]) -> list[float]:
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
    return {"avg": sum(values) / len(values), "max": max(values), "p95": _percentile(values, 0.95)}


def _stats_for(row: Row) -> dict[str, float]:
    return required_load_stats(read_csv_rows(row.csv_path))


def _fmt(value: float, suffix: str = "") -> str:
    return "-" if not _finite(value) else f"{value:.2f}{suffix}"


def _fmt3(value: float, suffix: str = "") -> str:
    return "-" if not _finite(value) else f"{value:.3f}{suffix}"


def _rel(path: Path) -> str:
    return path.relative_to(REPORT_PATH.parent).as_posix()


def _group(rows: list[Row], scenario: str) -> list[Row]:
    return [row for row in rows if row.scenario == scenario]


def plot_summary(rows: list[Row], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(15, 9))
    ax_min, ax_hit, ax_req, ax_act = axes.flat
    markers = {"truth": "o", "gimbal": "s"}
    colors = {"truth": "tab:blue", "gimbal": "tab:orange"}

    for scenario in ["truth", "gimbal"]:
        group = _group(rows, scenario)
        if not group:
            continue
        ax_min.plot(
            [row.start_range_m for row in group],
            [row.min_range_m for row in group],
            marker=markers[scenario],
            color=colors[scenario],
            linewidth=1.8,
            label=scenario,
        )
        x = [row.start_range_m + (-1.4 if scenario == "truth" else 1.4) for row in group]
        ax_hit.bar(x, [1 if row.hit else 0 for row in group], width=2.4, color=colors[scenario], label=scenario)
        req = [_stats_for(row) for row in group]
        ax_req.plot([row.start_range_m for row in group], [item["p95"] for item in req], marker=markers[scenario], color=colors[scenario], label=f"{scenario} P95")
        ax_req.plot([row.start_range_m for row in group], [item["max"] for item in req], linestyle="--", marker=markers[scenario], color=colors[scenario], alpha=0.75, label=f"{scenario} max")
        ax_act.plot(
            [row.start_range_m for row in group],
            [row.max_load_factor_fd_g for row in group],
            marker=markers[scenario],
            color=colors[scenario],
            linewidth=1.8,
            label=scenario,
        )

    ax_min.axhline(0.5, color="0.35", linestyle="--", linewidth=1, label="0.5m reference")
    ax_min.set_title("Minimum truth range")
    ax_min.set_xlabel("Initial horizontal range / m")
    ax_min.set_ylabel("Meters")
    ax_min.grid(True, alpha=0.3)
    ax_min.legend(fontsize=8)

    ax_hit.set_title("AirSim collision result")
    ax_hit.set_xlabel("Initial horizontal range / m")
    ax_hit.set_ylabel("Hit")
    ax_hit.set_ylim(0.0, 1.15)
    ax_hit.grid(True, axis="y", alpha=0.3)
    ax_hit.legend(fontsize=8)

    ax_req.set_title("Required overload from velocity command")
    ax_req.set_xlabel("Initial horizontal range / m")
    ax_req.set_ylabel("g")
    ax_req.grid(True, alpha=0.3)
    ax_req.legend(fontsize=8)

    ax_act.set_title("Actual overload, truth velocity finite difference")
    ax_act.set_xlabel("Initial horizontal range / m")
    ax_act.set_ylabel("g")
    ax_act.grid(True, alpha=0.3)
    ax_act.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_load_curves(rows: list[Row], output: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=False)
    ax_req, ax_act = axes
    for row in rows:
        samples = read_csv_rows(row.csv_path)
        t = _series(samples, "t")
        req = required_load_series(samples)
        act = _series(samples, "load_factor_fd_g")
        label = f"{row.scenario} {row.start_range_m:.0f}m"
        linestyle = "-" if row.scenario == "truth" else "--"
        ax_req.plot(t, req, linewidth=1.1, linestyle=linestyle, label=label)
        ax_act.plot(t, act, linewidth=1.1, linestyle=linestyle, label=label)
    ax_req.set_title("Required overload over time")
    ax_req.set_xlabel("Time / s")
    ax_req.set_ylabel("Required overload / g")
    ax_req.grid(True, alpha=0.3)
    ax_req.legend(fontsize=6, ncol=2)
    ax_act.set_title("Actual overload over time")
    ax_act.set_xlabel("Time / s")
    ax_act.set_ylabel("Actual overload / g")
    ax_act.grid(True, alpha=0.3)
    ax_act.legend(fontsize=6, ncol=2)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_range_curves(rows: list[Row], output: Path) -> None:
    fig, ax = plt.subplots(1, 1, figsize=(14, 6))
    for row in rows:
        samples = read_csv_rows(row.csv_path)
        t = _series(samples, "t")
        r = _series(samples, "range")
        linestyle = "-" if row.scenario == "truth" else "--"
        ax.plot(t, r, linewidth=1.3, linestyle=linestyle, label=f"{row.scenario} {row.start_range_m:.0f}m {'hit' if row.hit else 'miss'}")
    ax.set_title("Truth range over time")
    ax.set_xlabel("Time / s")
    ax.set_ylabel("Range / m")
    ax.set_ylim(bottom=0.0)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=6, ncol=2)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def _table(rows: list[Row]) -> str:
    lines = [
        "| 方案 | 初始水平距离 | 是否碰撞 | 碰撞时间 | 最小距离 | 末端距离 | 检测帧 | 有效帧 | 平均仿真FPS | 平均墙钟FPS | 最大实际过载 | 平均实际过载 | 最大需用过载 | P95需用过载 | 平均需用过载 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in sorted(rows, key=lambda item: (item.start_range_m, item.scenario)):
        req = _stats_for(row)
        hit_t = "-" if row.hit_t_s is None else f"{row.hit_t_s:.2f}s"
        lines.append(
            f"| {row.scenario} | {row.start_range_m:.0f}m | {'是' if row.hit else '否'} | {hit_t} | "
            f"{_fmt3(row.min_range_m, 'm')} | {_fmt3(row.final_range_m, 'm')} | "
            f"{row.detected_frames}/{row.frames} | {row.valid_frames}/{row.frames} | "
            f"{_fmt(row.avg_sim_sample_fps)} | {_fmt(row.avg_wall_fps)} | "
            f"{_fmt(row.max_load_factor_fd_g, 'g')} | {_fmt(row.avg_load_factor_fd_g, 'g')} | "
            f"{_fmt(req['max'], 'g')} | {_fmt(req['p95'], 'g')} | {_fmt(req['avg'], 'g')} |"
        )
    return "\n".join(lines)


def write_report(rows: list[Row], images: dict[str, Path], summary_paths: list[Path]) -> None:
    truth = _group(rows, "truth")
    gimbal = _group(rows, "gimbal")
    hit_truth = sum(1 for row in truth if row.hit)
    hit_gimbal = sum(1 for row in gimbal if row.hit)
    report = f"""# 真值 PNG 与云台视觉 PNG ClockSpeed 0.2 过载测试报告

## 1. 测试目的

本报告在与捷联 ClockSpeed 0.2 测试相同的初始工况下，对比两类拦截程序：

- `truth`：已知入侵无人机真实位置和速度的经典比例导引基线。
- `gimbal`：云台相机视觉 PNG，导引内部不使用入侵机真实位置，真值只用于离线评价。

两组测试均记录需用过载和实际过载。需用过载由速度指令变化量计算：`n_req = ||Δv_cmd / Δt_sim|| / g`；实际过载由拦截机真值速度有限差分计算：`n_act = ||Δv_truth / Δt_sim|| / g`。

## 2. 测试条件

- AirSim 配置：`config/airsim_blocks_settings.json`
- 仿真时钟：`ClockSpeed=0.2`
- 显示模式：`ViewMode=NoDisplay`
- 测试距离：`30m, 40m, 50m, 60m, 70m, 80m, 90m, 100m, 110m, 120m, 130m, 140m, 150m, 160m`
- 入侵机高度差：`20m`
- 侧向偏置：`-20m`
- 入侵机速度：`5m/s`
- 速度比：`2`
- 拦截机起始高度：`50m`

## 3. 结果总览

![总体对比]({_rel(images['summary'])})

{_table(rows)}

## 4. 距离曲线

![距离曲线]({_rel(images['range'])})

## 5. 需用过载与实际过载曲线

![过载曲线]({_rel(images['loads'])})

## 6. 结果解读

- 真值 PNG 命中 `{hit_truth}/{len(truth)}` 组；云台视觉 PNG 命中 `{hit_gimbal}/{len(gimbal)}` 组。
- 真值 PNG 的需用过载来自经典比例导引加速度经过速度指令接口后的等效变化量，因此可作为“理想测量条件下”的基准。
- 云台视觉 PNG 的需用过载同时包含视觉检测、LOS/TTC 估计、云台跟踪、末端 BlindPush 和重获目标状态切换的影响。若最大需用过载明显高于 P95，通常说明是少数帧状态切换或指令跳变，而不是持续导引需求。
- 实际过载仍以 `load_factor_fd_g` 为主，因为 AirSim SimpleFlight 的线加速度字段在部分版本中可能不稳定。

## 7. 日志文件

自动纳入的批次汇总：
{chr(10).join(f'- `{path.relative_to(PROJECT_ROOT).as_posix()}`' for path in summary_paths)}
"""
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(report, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate truth/gimbal ClockSpeed 0.2 report.")
    parser.add_argument("--truth-summary", required=True)
    parser.add_argument("--gimbal-summary", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    summary_paths = [PROJECT_ROOT / args.truth_summary, PROJECT_ROOT / args.gimbal_summary]
    rows = []
    for path in summary_paths:
        rows.extend(read_summary(path))
    rows = [row for row in rows if row.altitude_offset_m == 20.0 and 30.0 <= row.start_range_m <= 160.0]
    images = {
        "summary": ASSET_DIR / "truth_gimbal_clock0p2_summary.png",
        "range": ASSET_DIR / "truth_gimbal_clock0p2_range.png",
        "loads": ASSET_DIR / "truth_gimbal_clock0p2_loads.png",
    }
    plot_summary(rows, images["summary"])
    plot_range_curves(rows, images["range"])
    plot_load_curves(rows, images["loads"])
    write_report(rows, images, summary_paths)
    print(f"report={REPORT_PATH}")
    for image in images.values():
        print(f"image={image}")


if __name__ == "__main__":
    main()
