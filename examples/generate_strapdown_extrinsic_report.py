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
LOG_DIR = PROJECT_ROOT / "logs" / "strapdown_accuracy"
TRUTH_DIR = LOG_DIR / "truth_required_load"
REPORT_PATH = PROJECT_ROOT / "完整方案" / "捷联ClockSpeed0p2四工况相机外参测试报告.md"
ASSET_DIR = PROJECT_ROOT / "完整方案" / "assets" / "捷联ClockSpeed0p2四工况相机外参测试报告"
GRAVITY_MPS2 = 9.80665

CASE_LABELS = {
    "A": "A z+0.5m",
    "B": "B z+0.5m pitch-up15",
    "C": "C z+0.5m early-coast top-gate",
    "D": "D z+0.5m blind up-bias",
}


@dataclass
class CaseRow:
    label: str
    case: str
    start_range_m: float
    hit: bool
    hit_t_s: float
    min_range_m: float
    final_range_m: float
    frames: int
    detected_frames: int
    valid_frames: int
    avg_sim_sample_fps: float
    avg_sim_clock_ratio: float
    max_load_factor_fd_g: float
    csv_path: Path
    meta_path: Path


def _float(value: object, default: float = math.nan) -> float:
    if value is None or value == "":
        return default
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _int(value: object, default: int = 0) -> int:
    number = _float(value)
    return default if not math.isfinite(number) else int(number)


def _bool(value: object) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes"}


def _rel(path: Path) -> str:
    return path.relative_to(REPORT_PATH.parent).as_posix()


def _resolve(path_text: str) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else PROJECT_ROOT / path


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _series(rows: Iterable[dict[str, str]], key: str) -> list[float]:
    return [_float(row.get(key)) for row in rows]


def _finite(values: Iterable[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def _percentile(values: Iterable[float], q: float) -> float:
    finite = sorted(_finite(values))
    if not finite:
        return math.nan
    if len(finite) == 1:
        return finite[0]
    position = (len(finite) - 1) * max(0.0, min(1.0, q))
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return finite[lower]
    fraction = position - lower
    return finite[lower] * (1.0 - fraction) + finite[upper] * fraction


def _vector(row: dict[str, str], keys: tuple[str, str, str]) -> tuple[float, float, float] | None:
    values = tuple(_float(row.get(key)) for key in keys)
    if not all(math.isfinite(value) for value in values):
        return None
    return values


def _normalize(vec: tuple[float, float, float]) -> tuple[float, float, float] | None:
    norm = math.sqrt(sum(value * value for value in vec))
    if norm <= 1.0e-9:
        return None
    return tuple(value / norm for value in vec)


def _los_sep_deg(a: tuple[float, float, float] | None, b: tuple[float, float, float] | None) -> float:
    if a is None or b is None:
        return math.nan
    an = _normalize(a)
    bn = _normalize(b)
    if an is None or bn is None:
        return math.nan
    dot = max(-1.0, min(1.0, sum(an[i] * bn[i] for i in range(3))))
    return math.degrees(math.acos(dot))


def _load_rows(stamp: str) -> list[CaseRow]:
    rows: list[CaseRow] = []
    for label in "ABCD":
        summary_path = LOG_DIR / f"strapdown_clock0p2_ext{label}_{stamp}_summary.csv"
        if not summary_path.exists():
            continue
        for item in _read_csv(summary_path):
            rows.append(
                CaseRow(
                    label=label,
                    case=item.get("case", ""),
                    start_range_m=_float(item.get("start_horizontal_range_m")),
                    hit=_bool(item.get("hit")),
                    hit_t_s=_float(item.get("hit_t_s")),
                    min_range_m=_float(item.get("min_range_m")),
                    final_range_m=_float(item.get("final_range_m")),
                    frames=_int(item.get("frames")),
                    detected_frames=_int(item.get("detected_frames")),
                    valid_frames=_int(item.get("valid_frames")),
                    avg_sim_sample_fps=_float(item.get("avg_sim_sample_fps")),
                    avg_sim_clock_ratio=_float(item.get("avg_sim_clock_ratio")),
                    max_load_factor_fd_g=_float(item.get("max_load_factor_fd_g")),
                    csv_path=_resolve(item.get("csv_path", "")),
                    meta_path=_resolve(item.get("meta_path", "")),
                )
            )
    return sorted(rows, key=lambda row: (row.start_range_m, row.label))


def _truth_rows(row: CaseRow, stamp: str) -> list[dict[str, str]]:
    path = TRUTH_DIR / f"strapdown_clock0p2_ext{row.label}_truth_N3_{stamp}_{row.case}.csv"
    return _read_csv(path) if path.exists() else []


def _required_load(rows: list[dict[str, str]]) -> list[float]:
    result: list[float] = []
    prev_t: float | None = None
    prev_v: tuple[float, float, float] | None = None
    for item in rows:
        t = _float(item.get("t"))
        v = _vector(item, ("v_cmd_x", "v_cmd_y", "v_cmd_z"))
        if not math.isfinite(t) or v is None or prev_t is None or prev_v is None:
            result.append(0.0 if math.isfinite(t) and v is not None else math.nan)
        else:
            dt = t - prev_t
            if dt <= 1.0e-6:
                result.append(math.nan)
            else:
                dv = math.sqrt(sum((v[index] - prev_v[index]) ** 2 for index in range(3)))
                result.append(dv / dt / GRAVITY_MPS2)
        if math.isfinite(t) and v is not None:
            prev_t = t
            prev_v = v
    return result


def _clip_stats(rows: list[dict[str, str]]) -> dict[str, float]:
    total = max(1, len(rows))
    return {
        "top": sum(_bool(row.get("bbox_top_clipped")) for row in rows) / total,
        "bottom": sum(_bool(row.get("bbox_bottom_clipped")) for row in rows) / total,
        "left": sum(_bool(row.get("bbox_left_clipped")) for row in rows) / total,
        "right": sum(_bool(row.get("bbox_right_clipped")) for row in rows) / total,
        "pitch_rejected": sum(_bool(row.get("pitch_measurement_rejected")) for row in rows) / total,
    }


def _los_errors(rows: list[dict[str, str]]) -> list[float]:
    errors: list[float] = []
    for row in rows:
        visual = _vector(row, ("lambda_x", "lambda_y", "lambda_z"))
        camera = _vector(row, ("camera_world_x", "camera_world_y", "camera_world_z"))
        intruder = _vector(row, ("intruder_x", "intruder_y", "intruder_z"))
        truth = None
        if camera is not None and intruder is not None:
            truth = tuple(intruder[index] - camera[index] for index in range(3))
        errors.append(_los_sep_deg(visual, truth))
    return errors


def plot_summary(rows: list[CaseRow], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(15, 9))
    ax_min, ax_hit, ax_load, ax_fps = axes.flat
    for label in "ABCD":
        group = [row for row in rows if row.label == label]
        if not group:
            continue
        x = [row.start_range_m for row in group]
        ax_min.plot(x, [row.min_range_m for row in group], marker="o", linewidth=1.6, label=CASE_LABELS[label])
        ax_hit.plot(x, [1 if row.hit else 0 for row in group], marker="o", linewidth=1.4, label=CASE_LABELS[label])
        ax_load.plot(x, [row.max_load_factor_fd_g for row in group], marker="o", linewidth=1.4, label=CASE_LABELS[label])
        ax_fps.plot(x, [row.avg_sim_sample_fps for row in group], marker="o", linewidth=1.4, label=CASE_LABELS[label])
    ax_min.axhline(0.5, color="0.35", linestyle="--", linewidth=1)
    ax_min.set_title("Minimum true range")
    ax_min.set_ylabel("m")
    ax_hit.set_title("Collision result")
    ax_hit.set_ylabel("hit=1")
    ax_load.set_title("Max actual overload")
    ax_load.set_ylabel("g")
    ax_fps.set_title("Average simulation sample FPS")
    ax_fps.set_ylabel("Hz")
    for ax in axes.flat:
        ax.set_xlabel("Initial horizontal range / m")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_clips(rows: list[CaseRow], stamp: str, output: Path) -> dict[str, dict[str, float]]:
    stats: dict[str, dict[str, float]] = {}
    labels: list[str] = []
    values = {"top": [], "bottom": [], "left": [], "right": [], "pitch_rejected": []}
    for label in "ABCD":
        group = [row for row in rows if row.label == label]
        if not group:
            continue
        merged: list[dict[str, str]] = []
        for row in group:
            merged.extend(_read_csv(row.csv_path))
        item = _clip_stats(merged)
        stats[label] = item
        labels.append(label)
        for key in values:
            values[key].append(100.0 * item[key])

    fig, ax = plt.subplots(figsize=(11, 5.5))
    bottoms = [0.0 for _ in labels]
    colors = {
        "top": "tab:red",
        "bottom": "tab:blue",
        "left": "tab:orange",
        "right": "tab:green",
        "pitch_rejected": "tab:purple",
    }
    for key in ("top", "bottom", "left", "right", "pitch_rejected"):
        ax.bar(labels, values[key], bottom=bottoms, label=key, color=colors[key], alpha=0.75)
        bottoms = [bottoms[index] + values[key][index] for index in range(len(labels))]
    ax.set_title("BBox clipping and top-clipped pitch rejection ratio")
    ax.set_xlabel("Case")
    ax.set_ylabel("Frame ratio / %")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)
    return stats


def plot_per_distance(rows: list[CaseRow], stamp: str, output_dir: Path) -> dict[float, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    images: dict[float, Path] = {}
    for range_m in sorted({row.start_range_m for row in rows if math.isfinite(row.start_range_m)}):
        group = [row for row in rows if row.start_range_m == range_m]
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
        ax_load, ax_los, ax_range = axes
        for item in group:
            samples = _read_csv(item.csv_path)
            truth = _truth_rows(item, stamp)
            t = _series(samples, "t")
            label = CASE_LABELS[item.label]
            ax_load.plot(t, _required_load(samples), linewidth=1.0, alpha=0.55, label=f"{label} image cmd")
            ax_load.plot(t, _series(samples, "load_factor_fd_g"), linewidth=1.2, alpha=0.85, label=f"{label} actual")
            if truth:
                ax_load.plot(_series(truth, "t"), _series(truth, "n_req_g"), linewidth=1.4, linestyle="--", label=f"{label} truth theory")
            ax_los.plot(t, _los_errors(samples), linewidth=1.1, label=label)
            ax_range.plot(t, _series(samples, "range"), linewidth=1.3, label=f"{label} {'hit' if item.hit else 'miss'}")
        ax_load.set_title(f"{range_m:.0f}m overload comparison")
        ax_load.set_ylabel("Overload / g")
        ax_los.set_title("Visual LOS vs camera-origin truth LOS")
        ax_los.set_ylabel("LOS separation / deg")
        ax_range.set_title("True range")
        ax_range.set_xlabel("Time / s")
        ax_range.set_ylabel("Range / m")
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=6, ncol=2)
        fig.tight_layout()
        path = output_dir / f"strapdown_ext_compare_{int(round(range_m)):03d}m.png"
        fig.savefig(path, dpi=170)
        plt.close(fig)
        images[range_m] = path
    return images


def _stats_table(rows: list[CaseRow], stamp: str) -> str:
    lines = [
        "|工况|距离m|是否碰撞|碰撞时间s|最小距离m|检测帧/总帧|有效帧|实际过载max g|视觉指令P95 g|影子真值P95 g|LOS误差P95 deg|sim FPS|ClockRatio|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        samples = _read_csv(row.csv_path)
        truth = _truth_rows(row, stamp)
        visual_req = _required_load(samples)
        truth_req = _series(truth, "n_req_g")
        los_err = _los_errors(samples)
        lines.append(
            f"|{row.label}|{row.start_range_m:.0f}|{1 if row.hit else 0}|"
            f"{'-' if not math.isfinite(row.hit_t_s) else f'{row.hit_t_s:.2f}'}|"
            f"{row.min_range_m:.3f}|{row.detected_frames}/{row.frames}|{row.valid_frames}|"
            f"{row.max_load_factor_fd_g:.2f}|{_percentile(visual_req, 0.95):.2f}|"
            f"{_percentile(truth_req, 0.95):.2f}|{_percentile(los_err, 0.95):.2f}|"
            f"{row.avg_sim_sample_fps:.2f}|{row.avg_sim_clock_ratio:.3f}|"
        )
    return "\n".join(lines)


def _latest_stamp() -> str:
    candidates = sorted(LOG_DIR.glob("strapdown_clock0p2_extA_*_summary.csv"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise SystemExit("no extA summary found; pass --stamp or run run_strapdown_extrinsic_clock02_batch.sh")
    name = candidates[-1].name
    return name.removeprefix("strapdown_clock0p2_extA_").removesuffix("_summary.csv")


def write_report(rows: list[CaseRow], stamp: str, images: dict[str, Path], per_distance: dict[float, Path], clip_stats: dict[str, dict[str, float]]) -> None:
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    clip_lines = [
        "|工况|top裁切%|bottom裁切%|left裁切%|right裁切%|top俯仰拒绝%|",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for label in "ABCD":
        item = clip_stats.get(label, {})
        clip_lines.append(
            f"|{label}|{100.0 * item.get('top', math.nan):.2f}|{100.0 * item.get('bottom', math.nan):.2f}|"
            f"{100.0 * item.get('left', math.nan):.2f}|{100.0 * item.get('right', math.nan):.2f}|"
            f"{100.0 * item.get('pitch_rejected', math.nan):.2f}|"
        )

    image_lines = "\n".join(
        f"![{int(range_m)}m]({_rel(path)})" for range_m, path in sorted(per_distance.items())
    )
    report = f"""# 捷联视觉 PNG ClockSpeed 0.2 四工况相机外参测试报告

## 1. 实验设置

- 仿真：AirSim Blocks，`ClockSpeed=0.2`，无头运行。
- 算法：捷联视觉 PNG，`--no-los-filter`，检测使用 AirSim detect，算法内部不使用入侵机真实位置。
- 批量距离：`30-160m`，高度差 `20m`，入侵机速度 `5m/s`，拦截机速度上限为其 `2x`。
- 相机坐标：AirSim NED 机体系，`camera_z=-0.5` 表示相机相对机架上移 `0.5m`。
- 俯仰约定：`camera_pitch_deg` 为正向下，所以上仰 `15deg` 使用 `--camera-pitch-deg -15`。

## 2. 四种工况

- A：相机上移 `0.5m`，俯仰 `0deg`。
- B：相机上移 `0.5m`，相机上仰 `15deg`。
- C：相机上移 `0.5m`，更早进入外推，并在 `bbox_top_clipped=True` 后不再信任 bbox center 的俯仰测量。
- D：相机上移 `0.5m`，进入外推后保留更强向上零偏。

## 3. 总览图

![summary]({_rel(images['summary'])})

![clip]({_rel(images['clip'])})

## 4. 汇总表

{_stats_table(rows, stamp)}

## 5. 裁切统计

{chr(10).join(clip_lines)}

## 6. 单距离曲线

每张图包含视觉 PNG 速度指令等效需用过载、无人机实际过载、影子真值 PNG 理论需用过载、视觉 LOS 与相机光心真值 LOS 的误差，以及真实距离曲线。

{image_lines}

## 7. 结论读取方法

- 如果 B 组 LOS 误差和最小距离优于 A，说明固定上仰相机外参能缓解目标从画面上边界饱和导致的俯仰偏低。
- 如果 C 组在 top 裁切后过载尖峰下降，说明 top clipped 俯仰门控和更早外推有效；若最小距离变差，则外推进入过早或 KF 预测时间过长。
- 如果 D 组垂直残差改善但实际过载升高，说明向上零偏方向正确但幅值需要回调。
- 影子真值 PNG 曲线只用于评价，不参与视觉导引。
"""
    REPORT_PATH.write_text(report, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate the strapdown four-case extrinsic report.")
    parser.add_argument("--stamp", default="", help="Batch stamp used by run_strapdown_extrinsic_clock02_batch.sh")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    stamp = args.stamp or _latest_stamp()
    rows = _load_rows(stamp)
    if not rows:
        raise SystemExit(f"no rows found for stamp {stamp}")
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    images = {
        "summary": ASSET_DIR / f"strapdown_ext_{stamp}_summary.png",
        "clip": ASSET_DIR / f"strapdown_ext_{stamp}_clip_stats.png",
    }
    plot_summary(rows, images["summary"])
    clip_stats = plot_clips(rows, stamp, images["clip"])
    per_distance = plot_per_distance(rows, stamp, ASSET_DIR)
    write_report(rows, stamp, images, per_distance, clip_stats)
    print(f"report={REPORT_PATH}")
    print(f"stamp={stamp}")


if __name__ == "__main__":
    main()
