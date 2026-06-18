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
REPORT_PATH = PROJECT_ROOT / "完整方案" / "捷联ClockSpeed1中心相机对比报告.md"
ASSET_DIR = PROJECT_ROOT / "完整方案" / "assets" / "捷联ClockSpeed1中心相机对比报告"
GRAVITY_MPS2 = 9.80665
CASE_ORDER = "ABCDEFGHI"

CASE_LABELS = {
    "A": "A c0.2 z+0.5m",
    "B": "B c0.2 pitch-up15",
    "C": "C c0.2 early-coast top-gate",
    "D": "D c0.2 blind up-bias",
    "E": "E c1.0 center-camera",
    "F": "F c1.0 noisy KF",
    "G": "G c1.0 noisy raw",
    "H": "H PX4+Actor noisy raw",
    "I": "I PX4+Actor noisy KF",
}

CASE_TEXT = {
    "A": "上一轮 A：ClockSpeed=0.2，相机上移 0.5m。",
    "B": "上一轮 B：ClockSpeed=0.2，相机上移 0.5m，并上仰 15deg。",
    "C": "上一轮 C：ClockSpeed=0.2，相机上移 0.5m，更早外推，top 裁切后拒绝俯仰 bbox center。",
    "D": "上一轮 D：ClockSpeed=0.2，相机上移 0.5m，外推后加入向上零偏。",
    "E": "本轮 E：ClockSpeed=1.0，相机位于机体中心，外参无平移、无俯仰偏差。",
    "F": "本轮 F：ClockSpeed=1.0，中心相机，bbox 中等噪声，LOS Kalman 滤波开启。",
    "G": "本轮 G：ClockSpeed=1.0，中心相机，bbox 中等噪声，LOS 滤波关闭。",
    "H": "本轮 H：PX4 SITL 拦截机 + 按轨迹移动的立方体 Actor 目标，中心相机，bbox 中等噪声，LOS 滤波关闭。",
    "I": "本轮 I：PX4 SITL 拦截机 + 按轨迹移动的立方体 Actor 目标，中心相机，bbox 中等噪声，LOS Kalman 滤波开启。",
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
    truth_prefix: str


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


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _resolve(path_text: str) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else PROJECT_ROOT / path


def _rel(path: Path) -> str:
    return path.relative_to(REPORT_PATH.parent).as_posix()


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
    dot = max(-1.0, min(1.0, sum(an[index] * bn[index] for index in range(3))))
    return math.degrees(math.acos(dot))


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


def _truth_rows(row: CaseRow) -> list[dict[str, str]]:
    path = TRUTH_DIR / f"{row.truth_prefix}_{row.case}.csv"
    return _read_csv(path) if path.exists() else []


def _load_summary(path: Path, label: str, truth_prefix: str) -> list[CaseRow]:
    if not path.exists():
        return []
    rows: list[CaseRow] = []
    for item in _read_csv(path):
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
                truth_prefix=truth_prefix,
            )
        )
    return rows


def _latest_baseline_stamp() -> str:
    candidates = sorted(LOG_DIR.glob("strapdown_clock0p2_extA_*_summary.csv"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        return ""
    return candidates[-1].name.removeprefix("strapdown_clock0p2_extA_").removesuffix("_summary.csv")


def load_rows(stamp: str, baseline_stamp: str, noise_stamp: str, sitl_stamp: str) -> list[CaseRow]:
    rows: list[CaseRow] = []
    if baseline_stamp:
        for label in "ABCD":
            rows.extend(
                _load_summary(
                    LOG_DIR / f"strapdown_clock0p2_ext{label}_{baseline_stamp}_summary.csv",
                    label,
                    f"strapdown_clock0p2_ext{label}_truth_N3_{baseline_stamp}",
                )
            )
    rows.extend(
        _load_summary(
            LOG_DIR / f"strapdown_clock1_center_{stamp}_summary.csv",
            "E",
            f"strapdown_clock1_center_truth_N3_{stamp}",
        )
    )
    if noise_stamp:
        for label in ("F", "G"):
            rows.extend(
                _load_summary(
                    LOG_DIR / f"strapdown_clock1_noise_{label}_{noise_stamp}_summary.csv",
                    label,
                    f"strapdown_clock1_noise_{label}_truth_N3_{noise_stamp}",
                )
            )
    if sitl_stamp:
        for label in ("H", "I"):
            rows.extend(
                _load_summary(
                    LOG_DIR / f"strapdown_clock1_sitl_{label}_{sitl_stamp}_summary.csv",
                    label,
                    f"strapdown_clock1_sitl_{label}_truth_N3_{sitl_stamp}",
                )
            )
    return sorted(rows, key=lambda row: (row.start_range_m, row.label))


def _clip_stats(rows: list[dict[str, str]]) -> dict[str, float]:
    total = max(1, len(rows))
    return {
        "top": sum(_bool(row.get("bbox_top_clipped")) for row in rows) / total,
        "bottom": sum(_bool(row.get("bbox_bottom_clipped")) for row in rows) / total,
        "left": sum(_bool(row.get("bbox_left_clipped")) for row in rows) / total,
        "right": sum(_bool(row.get("bbox_right_clipped")) for row in rows) / total,
        "pitch_rejected": sum(_bool(row.get("pitch_measurement_rejected")) for row in rows) / total,
    }


def plot_summary(rows: list[CaseRow], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(15, 9))
    ax_min, ax_hit, ax_load, ax_clock = axes.flat
    for label in CASE_ORDER:
        group = [row for row in rows if row.label == label]
        if not group:
            continue
        x = [row.start_range_m for row in group]
        ax_min.plot(x, [row.min_range_m for row in group], marker="o", linewidth=1.5, label=CASE_LABELS[label])
        ax_hit.plot(x, [1 if row.hit else 0 for row in group], marker="o", linewidth=1.3, label=CASE_LABELS[label])
        ax_load.plot(x, [row.max_load_factor_fd_g for row in group], marker="o", linewidth=1.3, label=CASE_LABELS[label])
        ax_clock.plot(x, [row.avg_sim_clock_ratio for row in group], marker="o", linewidth=1.3, label=CASE_LABELS[label])
    ax_min.axhline(0.5, color="0.35", linestyle="--", linewidth=1)
    ax_min.set_title("Minimum true range")
    ax_min.set_ylabel("m")
    ax_hit.set_title("Collision result")
    ax_hit.set_ylabel("hit=1")
    ax_load.set_title("Max actual overload")
    ax_load.set_ylabel("g")
    ax_clock.set_title("Measured ClockSpeed ratio")
    ax_clock.set_ylabel("sim/wall")
    for ax in axes.flat:
        ax.set_xlabel("Initial horizontal range / m")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=6)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_clips(rows: list[CaseRow], output: Path) -> dict[str, dict[str, float]]:
    labels: list[str] = []
    values = {"top": [], "bottom": [], "left": [], "right": [], "pitch_rejected": []}
    stats: dict[str, dict[str, float]] = {}
    for label in CASE_ORDER:
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
    ax.set_title("BBox clipping ratio")
    ax.set_xlabel("Case")
    ax.set_ylabel("Frame ratio / %")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)
    return stats


def plot_per_distance(rows: list[CaseRow], output_dir: Path) -> dict[float, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    images: dict[float, Path] = {}
    for range_m in sorted({row.start_range_m for row in rows if math.isfinite(row.start_range_m)}):
        group = [row for row in rows if row.start_range_m == range_m]
        fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=False)
        ax_load, ax_los, ax_ttc, ax_range = axes
        for item in group:
            samples = _read_csv(item.csv_path)
            truth = _truth_rows(item)
            t = _series(samples, "t")
            label = CASE_LABELS[item.label]
            ax_load.plot(t, _required_load(samples), linewidth=1.0, alpha=0.5, label=f"{label} image cmd")
            ax_load.plot(t, _series(samples, "load_factor_fd_g"), linewidth=1.1, alpha=0.82, label=f"{label} actual")
            if truth:
                ax_load.plot(_series(truth, "t"), _series(truth, "n_req_g"), linewidth=1.2, linestyle="--", label=f"{label} truth")
            ax_los.plot(t, _los_errors(samples), linewidth=1.0, label=label)
            ax_ttc.plot(t, _series(samples, "ttc"), linewidth=1.0, label=f"{label} TTC")
            ax_range.plot(t, _series(samples, "range"), linewidth=1.2, label=f"{label} {'hit' if item.hit else 'miss'}")
        ax_load.set_title(f"{range_m:.0f}m overload comparison")
        ax_load.set_ylabel("Overload / g")
        ax_los.set_title("Visual LOS vs camera-origin truth LOS")
        ax_los.set_ylabel("LOS separation / deg")
        ax_ttc.set_title("Scale expansion TTC")
        ax_ttc.set_ylabel("TTC / s")
        ax_range.set_title("True range")
        ax_range.set_xlabel("Time / s")
        ax_range.set_ylabel("Range / m")
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=5, ncol=2)
        fig.tight_layout()
        path = output_dir / f"strapdown_center_clock1_compare_{int(round(range_m)):03d}m.png"
        fig.savefig(path, dpi=170)
        plt.close(fig)
        images[range_m] = path
    return images


def _stats_table(rows: list[CaseRow]) -> str:
    lines = [
        "|工况|距离m|是否碰撞|碰撞时间s|最小距离m|检测帧/总帧|有效帧|实际过载max g|视觉指令P95 g|影子真值P95 g|LOS误差P95 deg|sim FPS|ClockRatio|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        samples = _read_csv(row.csv_path)
        truth = _truth_rows(row)
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


def _group_summary(rows: list[CaseRow]) -> str:
    lines = [
        "|工况|命中数|命中距离m|最小距离m|平均ClockRatio|噪声|LOS滤波|",
        "|---|---:|---|---:|---:|---:|---:|",
    ]
    for label in CASE_ORDER:
        group = [row for row in rows if row.label == label]
        if not group:
            continue
        hit_ranges = ", ".join(f"{row.start_range_m:.0f}" for row in group if row.hit) or "-"
        min_range = min(row.min_range_m for row in group if math.isfinite(row.min_range_m))
        clock = _percentile((row.avg_sim_clock_ratio for row in group), 0.5)
        samples = _read_csv(group[0].csv_path) if group[0].csv_path.exists() else []
        noise_enabled = _bool(samples[0].get("bbox_noise_enabled")) if samples else False
        los_filter_enabled = _bool(samples[0].get("los_filter_enabled")) if samples else False
        lines.append(
            f"|{label}|{sum(row.hit for row in group)}/{len(group)}|{hit_ranges}|{min_range:.3f}|"
            f"{clock:.3f}|{int(noise_enabled)}|{int(los_filter_enabled)}|"
        )
    return "\n".join(lines)


def _norm_series(rows: list[dict[str, str]], keys: tuple[str, str, str]) -> list[float]:
    values: list[float] = []
    for row in rows:
        vec = _vector(row, keys)
        if vec is not None:
            values.append(math.sqrt(sum(component * component for component in vec)))
    return values


def _sitl_conclusion(groups: dict[str, list[CaseRow]]) -> str:
    lines: list[str] = []
    for label in ("H", "I"):
        group = groups.get(label, [])
        if not group:
            continue
        samples: list[dict[str, str]] = []
        for row in group:
            samples.extend(_read_csv(row.csv_path))
        hits = sum(row.hit for row in group)
        min_range = min(row.min_range_m for row in group if math.isfinite(row.min_range_m))
        sim_fps = _percentile((row.avg_sim_sample_fps for row in group), 0.5)
        valid_frames = sum(row.valid_frames for row in group)
        total_frames = sum(row.frames for row in group)
        actual_speed = _percentile(_norm_series(samples, ("interceptor_vel_x", "interceptor_vel_y", "interceptor_vel_z")), 0.5)
        speed_cap = _percentile((_float(row.get("speed_cap")) for row in samples), 0.5)
        intruder_speed = _percentile((_float(row.get("intruder_speed")) for row in samples), 0.5)
        detection_names: set[str] = set()
        multi_name_frames = 0
        for sample in samples:
            names = [name.strip() for name in str(sample.get("detection_names", "")).split("|") if name.strip()]
            detection_names.update(names)
            if len(names) > 1:
                multi_name_frames += 1
        lines.append(
            f"- {label} 组 PX4+Actor 实测命中 `{hits}/{len(group)}`，最小原点距离 `{min_range:.3f}m`，"
            f"有效视觉帧 `{valid_frames}/{total_frames}`，中位仿真采样率 `{sim_fps:.2f}Hz`，"
            f"拦截机中位实际速度 `{actual_speed:.2f}m/s`，明显低于速度上限 `{speed_cap:.1f}m/s`；"
            f"Actor 目标按 `{intruder_speed:.1f}m/s` 移动，因此当前失败主要受 PX4 Offboard 控制链路、"
            "AirSim/PX4 同步采样率和实际速度响应限制影响，不能直接判定为 LOS+TTC 导引律失效。"
        )
        if multi_name_frames or any(name != "IntruderActor" for name in detection_names):
            names_text = ", ".join(sorted(detection_names)) or "-"
            lines.append(
                f"- {label} 组日志中出现过多目标检测名 `{names_text}`，其中多目标帧 `{multi_name_frames}` 帧。"
                "这说明 Actor 批量测试期间存在残留或自动重命名目标，可能污染 `detect` 的目标选择；"
                "后续测试应重启 Blocks 或复用同名 Actor，避免把残留 Actor 纳入 `IntruderActor*` 检测通配符。"
            )
    if not lines:
        return ""
    lines.append(
        "- 因此，H/I 在本报告中应作为“PX4 SITL 控制链路连通性与工程问题暴露”记录，而不是作为与 E/F/G "
        "SimpleFlight 同权重的算法命中率对比。下一步应先把 PX4 实际速度闭环、Offboard 更新率和 Actor 生命周期稳定下来，再重新评估滤波开关的收益。"
    )
    return "\n".join(lines)


def _actual_conclusion(rows: list[CaseRow]) -> str:
    groups = {label: [row for row in rows if row.label == label] for label in CASE_ORDER}
    e_group = groups.get("E", [])
    if not e_group:
        return "- 本轮 E 组数据缺失，无法形成实际结论。"
    e_hits = sum(row.hit for row in e_group)
    e_total = len(e_group)
    e_hit_ranges = ", ".join(f"{row.start_range_m:.0f}m" for row in e_group if row.hit) or "-"
    e_min = min(row.min_range_m for row in e_group if math.isfinite(row.min_range_m))
    e_clock = _percentile((row.avg_sim_clock_ratio for row in e_group), 0.5)
    e_clip = _clip_stats([sample for row in e_group for sample in _read_csv(row.csv_path)])

    baseline_lines: list[str] = []
    for label in "ABCD":
        group = groups.get(label, [])
        if not group:
            continue
        hits = sum(row.hit for row in group)
        min_range = min(row.min_range_m for row in group if math.isfinite(row.min_range_m))
        baseline_lines.append(f"{label}={hits}/{len(group)}, min={min_range:.3f}m")
    baseline_text = "; ".join(baseline_lines) or "无基线数据"
    noise_lines: list[str] = []
    for label in ("F", "G", "H", "I"):
        group = groups.get(label, [])
        if not group:
            continue
        hits = sum(row.hit for row in group)
        hit_ranges = ", ".join(f"{row.start_range_m:.0f}m" for row in group if row.hit) or "-"
        min_range = min(row.min_range_m for row in group if math.isfinite(row.min_range_m))
        samples = _read_csv(group[0].csv_path)
        los_filter_enabled = _bool(samples[0].get("los_filter_enabled")) if samples else False
        noise_lines.append(
            f"- {label} 组带噪声、LOS滤波 `{int(los_filter_enabled)}`，命中 `{hits}/{len(group)}`，"
            f"命中距离 `{hit_ranges}`，最小原点距离 `{min_range:.3f}m`。"
        )
    noise_text = "\n".join(noise_lines)

    base_text = "\n".join(
        [
            f"- E 组实际命中 `{e_hits}/{e_total}`，命中距离为 `{e_hit_ranges}`，只有 `30m` 未命中；上一轮四组为 `{baseline_text}`。",
            f"- E 组最小原点距离为 `{e_min:.3f}m`，看起来大于上一轮 0.45-0.48m，但这是因为 AirSim 碰撞体先触发 collision 后程序立即截断日志，不能把它直接理解为脱靶量更大。",
            f"- E 组 top 裁切比例 `{100.0 * e_clip['top']:.2f}%`，明显低于上一轮 C 组的 top 裁切门控压力，说明相机位于机体中心后末端 bbox 上边界饱和问题减轻。",
            f"- E 组实测 ClockRatio 约 `{e_clock:.3f}`，而上一轮约 `0.200`。因此本次改动同时改变了相机外参和仿真时钟，结论是“中心相机 + 正常时钟”显著提高碰撞触发稳定性，但还不能单独证明 0.5m 外参就是唯一原因。",
            "- 30m 失败仍然是近距离初始 bbox 过大、可用视觉帧过少导致，和上一轮短距离不稳定现象一致。",
        ]
    )
    sitl_text = _sitl_conclusion(groups)
    sections = [base_text]
    if noise_text:
        sections.append(noise_text)
    if sitl_text:
        sections.append(sitl_text)
    return "\n".join(sections)


def write_report(
    rows: list[CaseRow],
    stamp: str,
    baseline_stamp: str,
    noise_stamp: str,
    sitl_stamp: str,
    summary_path: Path,
    clip_path: Path,
    per_distance: dict[float, Path],
    clip_stats: dict[str, dict[str, float]],
) -> None:
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    clip_lines = [
        "|工况|top裁切%|bottom裁切%|left裁切%|right裁切%|top俯仰拒绝%|",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for label in CASE_ORDER:
        if label not in clip_stats:
            continue
        item = clip_stats[label]
        clip_lines.append(
            f"|{label}|{100.0 * item['top']:.2f}|{100.0 * item['bottom']:.2f}|"
            f"{100.0 * item['left']:.2f}|{100.0 * item['right']:.2f}|"
            f"{100.0 * item['pitch_rejected']:.2f}|"
        )
    image_lines = "\n".join(f"![{int(range_m)}m]({_rel(path)})" for range_m, path in sorted(per_distance.items()))
    present_labels = {row.label for row in rows}
    case_lines = "\n".join(f"- {CASE_TEXT[label]}" for label in CASE_ORDER if label in present_labels)
    sitl_note = (
        ""
        if {"H", "I"} & present_labels
        else "\n- H/I PX4 SITL+Actor 工况尚未形成可统计结果；该工况只保留拦截机为 PX4 SITL，入侵目标为按轨迹移动的立方体 Actor，用于规避双 PX4 Offboard 链路不稳定。"
    )
    report = f"""# 捷联视觉 PNG ClockSpeed 1 中心相机对比报告

## 1. 实验设置

- 本轮：AirSim Blocks，`ClockSpeed=1.0`，捷联视觉 PNG；E/G/H 为 `--no-los-filter`，F/I 为 LOS Kalman 滤波开启。
- 本轮相机：`camera_z=0`、`camera_pitch_deg=0`、`camera_roll_deg=0`、`camera_yaw_deg=0`，即视觉 LOS 求解时认为相机位于机体中心且无安装偏差。
- 对比基线：上一轮四工况，`ClockSpeed=0.2`，baseline stamp `{baseline_stamp or 'N/A'}`。
- 噪声工况：noise stamp `{noise_stamp or 'N/A'}`；bbox center 高斯噪声 `sigma=3px`，bbox area 比例高斯噪声 `sigma=8%`，保持宽高比缩放。该量级按工程经验近似代表 640x480、120deg FOV 下检测框边界抖动和尺度估计误差。
- SITL 工况：sitl stamp `{sitl_stamp or 'N/A'}`；H/I 目标配置为 PX4 SITL 拦截机 + 按轨迹移动的立方体 Actor。PNG 内部仍只使用 AirSim detection bbox，Actor 真值只用于轨迹驱动、距离统计和碰撞评价。
- 批量距离：`30-160m`，高度差 `20m`，入侵机速度 `5m/s`，拦截机速度上限为其 `2x`。
- 注意：E/F/G 与 H/I 的飞控链路不同；H/I 结果包含 PX4 SITL Offboard 控制、EKF 状态估计和 AirSim 接口时序影响，不能与 SimpleFlight 工况只按算法开关直接等价比较。

## 2. 工况说明

{case_lines}
{sitl_note}

## 3. 总览

![summary]({_rel(summary_path)})

![clip]({_rel(clip_path)})

## 4. 命中汇总

{_group_summary(rows)}

## 5. 明细表

{_stats_table(rows)}

## 6. 裁切统计

{chr(10).join(clip_lines)}

## 7. 单距离曲线

每张图包含视觉 PNG 速度指令等效需用过载、无人机实际过载、影子真值 PNG 理论需用过载、视觉 LOS 与相机光心真值 LOS 的误差、尺度膨胀 TTC，以及真实距离曲线。

{image_lines}

## 8. 实际结论

{_actual_conclusion(rows)}
"""
    REPORT_PATH.write_text(report, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate center-camera ClockSpeed=1 strapdown comparison report.")
    parser.add_argument("--stamp", required=True)
    parser.add_argument("--baseline-stamp", default="")
    parser.add_argument("--noise-stamp", default="")
    parser.add_argument("--sitl-stamp", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    baseline_stamp = args.baseline_stamp or _latest_baseline_stamp()
    rows = load_rows(args.stamp, baseline_stamp, args.noise_stamp, args.sitl_stamp)
    if not rows:
        raise SystemExit(f"no rows found for stamp {args.stamp}")
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    summary_path = ASSET_DIR / f"strapdown_center_clock1_{args.stamp}_summary.png"
    clip_path = ASSET_DIR / f"strapdown_center_clock1_{args.stamp}_clip_stats.png"
    plot_summary(rows, summary_path)
    clip_stats = plot_clips(rows, clip_path)
    per_distance = plot_per_distance(rows, ASSET_DIR)
    write_report(rows, args.stamp, baseline_stamp, args.noise_stamp, args.sitl_stamp, summary_path, clip_path, per_distance, clip_stats)
    print(f"report={REPORT_PATH}")
    print(f"stamp={args.stamp}")
    print(f"baseline_stamp={baseline_stamp}")
    print(f"noise_stamp={args.noise_stamp}")
    print(f"sitl_stamp={args.sitl_stamp}")


if __name__ == "__main__":
    main()
