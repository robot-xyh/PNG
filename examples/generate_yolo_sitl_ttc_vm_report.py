from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

LOG_DIR = PROJECT_ROOT / "logs" / "yolo_sitl_ttc_vm"
REPORT_PATH = PROJECT_ROOT / "完整方案" / "YOLO_SITL_TTC_VM拦截对比报告.md"
ASSET_DIR = PROJECT_ROOT / "完整方案" / "assets" / "YOLO_SITL_TTC_VM拦截对比报告"
DEFAULT_TITLE = "YOLO + ByteTrack PX4 SITL TTC / V_m 拦截对比报告"
DEFAULT_RANGE_NOTE = "两组均测试 50m、60m、70m、80m、90m、100m，每个工况重启 PX4 SITL 和 Blocks。"
GRAVITY_MPS2 = 9.80665

LABELS = {
    "TTC": "TTC + LOS/Vm soft guidance",
    "VM": "fixed Vm PNG",
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
    avg_wall_fps: float
    avg_sim_sample_fps: float
    avg_detector_fps: float
    max_load_factor_fd_g: float
    csv_path: Path
    meta_path: Path


def _float(value: object, default: float = math.nan) -> float:
    if value is None or value == "":
        return default
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _int(value: object, default: int = 0) -> int:
    number = _float(value)
    return default if not math.isfinite(number) else int(number)


def _bool(value: object) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes"}


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _read_json(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def _resolve(path_text: str) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else PROJECT_ROOT / path


def _rel(path: Path) -> str:
    return path.relative_to(REPORT_PATH.parent).as_posix()


def _series(rows: list[dict[str, str]], key: str) -> list[float]:
    return [_float(row.get(key)) for row in rows]


def _finite(values: list[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def _percentile(values: list[float], q: float) -> float:
    finite = sorted(_finite(values))
    if not finite:
        return math.nan
    if len(finite) == 1:
        return finite[0]
    pos = (len(finite) - 1) * max(0.0, min(1.0, q))
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return finite[lo]
    frac = pos - lo
    return finite[lo] * (1.0 - frac) + finite[hi] * frac


def _vector(row: dict[str, str], keys: tuple[str, str, str]) -> tuple[float, float, float] | None:
    values = tuple(_float(row.get(key)) for key in keys)
    if not all(math.isfinite(value) for value in values):
        return None
    return values


def _norm_series(rows: list[dict[str, str]], keys: tuple[str, str, str]) -> list[float]:
    result: list[float] = []
    for row in rows:
        vec = _vector(row, keys)
        if vec is not None:
            result.append(math.sqrt(sum(component * component for component in vec)))
    return result


def _required_load(rows: list[dict[str, str]]) -> list[float]:
    result: list[float] = []
    prev_t: float | None = None
    prev_v: tuple[float, float, float] | None = None
    for row in rows:
        t = _float(row.get("t"))
        v = _vector(row, ("v_cmd_x", "v_cmd_y", "v_cmd_z"))
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


def _guidance_load(rows: list[dict[str, str]]) -> list[float]:
    result: list[float] = []
    for row in rows:
        n_cmd = _float(row.get("n_cmd_g"))
        if math.isfinite(n_cmd):
            result.append(n_cmd)
            continue
        vec = _vector(row, ("g_eval_x", "g_eval_y", "g_eval_z"))
        if vec is None:
            result.append(math.nan)
        else:
            result.append(math.sqrt(sum(component * component for component in vec)) / GRAVITY_MPS2)
    return result


def _load_summary(path: Path, label: str) -> list[CaseRow]:
    if not path.exists():
        return []
    rows: list[CaseRow] = []
    for item in _read_csv(path):
        csv_path = _resolve(item.get("csv_path", ""))
        meta_path = _resolve(item.get("meta_path", "")) if item.get("meta_path") else csv_path.with_name(f"{csv_path.stem}_meta.json")
        meta = _read_json(meta_path)
        derived = meta.get("derived", {}) if isinstance(meta, dict) else {}
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
                avg_wall_fps=_float(item.get("avg_wall_fps")),
                avg_sim_sample_fps=_float(item.get("avg_sim_sample_fps")),
                avg_detector_fps=_float(derived.get("avg_detector_fps")),
                max_load_factor_fd_g=_float(item.get("max_load_factor_fd_g")),
                csv_path=csv_path,
                meta_path=meta_path,
            )
        )
    return rows


def load_rows(stamp: str) -> list[CaseRow]:
    rows: list[CaseRow] = []
    for label in ("TTC", "VM"):
        rows.extend(_load_summary(LOG_DIR / f"yolo_sitl_{label}_{stamp}_summary.csv", label))
    return sorted(rows, key=lambda row: (row.start_range_m, row.label))


def _first_sample(rows: list[CaseRow]) -> dict[str, str]:
    for row in rows:
        if row.csv_path.exists():
            data = _read_csv(row.csv_path)
            if data:
                return data[0]
    return {}


def _first_meta(rows: list[CaseRow]) -> dict:
    for row in rows:
        meta = _read_json(row.meta_path)
        if meta:
            return meta
    return {}


def _summary_table(rows: list[CaseRow]) -> str:
    lines = [
        "|组别|命中数|命中距离m|未命中距离m|最小中心距离m|检测帧/总帧|有效帧/总帧|平均检测FPS|",
        "|---|---:|---|---|---:|---:|---:|---:|",
    ]
    for label in ("TTC", "VM"):
        group = [row for row in rows if row.label == label]
        if not group:
            continue
        hit_ranges = ", ".join(f"{row.start_range_m:.0f}" for row in group if row.hit) or "-"
        miss_ranges = ", ".join(f"{row.start_range_m:.0f}" for row in group if not row.hit) or "-"
        min_range = min([row.min_range_m for row in group if math.isfinite(row.min_range_m)] or [math.nan])
        frames = sum(row.frames for row in group)
        detected = sum(row.detected_frames for row in group)
        valid = sum(row.valid_frames for row in group)
        fps = [row.avg_detector_fps for row in group if math.isfinite(row.avg_detector_fps)]
        lines.append(
            f"|{label}|{sum(row.hit for row in group)}/{len(group)}|{hit_ranges}|{miss_ranges}|"
            f"{min_range:.3f}|{detected}/{frames}|{valid}/{frames}|"
            f"{(sum(fps) / len(fps) if fps else math.nan):.2f}|"
        )
    return "\n".join(lines)


def _detail_table(rows: list[CaseRow]) -> str:
    lines = [
        "|组别|距离m|碰撞|碰撞时间s|最小距离m|终点距离m|检测帧率|有效帧率|YOLO FPS|sim FPS|实际过载max g|速度指令差分P95 g|需用过载P95 g|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        samples = _read_csv(row.csv_path) if row.csv_path.exists() else []
        det_ratio = 100.0 * row.detected_frames / max(1, row.frames)
        valid_ratio = 100.0 * row.valid_frames / max(1, row.frames)
        lines.append(
            f"|{row.label}|{row.start_range_m:.0f}|{1 if row.hit else 0}|"
            f"{'-' if not math.isfinite(row.hit_t_s) else f'{row.hit_t_s:.2f}'}|"
            f"{row.min_range_m:.3f}|{row.final_range_m:.3f}|{det_ratio:.1f}%|{valid_ratio:.1f}%|"
            f"{row.avg_detector_fps:.2f}|{row.avg_sim_sample_fps:.2f}|{row.max_load_factor_fd_g:.2f}|"
            f"{_percentile(_required_load(samples), 0.95):.2f}|{_percentile(_guidance_load(samples), 0.95):.2f}|"
        )
    return "\n".join(lines)


def _settings_markdown(rows: list[CaseRow], stamp: str) -> str:
    sample = _first_sample(rows)
    meta = _first_meta(rows).get("args", {})
    return "\n".join(
        [
            "|参数|值|",
            "|---|---|",
            f"|stamp|`{stamp}`|",
            f"|settings|`{meta.get('settings_path', sample.get('settings_path', ''))}`|",
            f"|拦截机|`PX4 SITL / {sample.get('px4_command_mode', '')}`|",
            f"|目标 actor|`{sample.get('intruder_actor_name', '')}`|",
            f"|actor asset|`{sample.get('intruder_actor_asset', '')}`|",
            f"|actor scale|`{sample.get('intruder_actor_scale', '')}`|",
            f"|检测源|`{sample.get('detector_source', '')}`|",
            f"|YOLO model|`{sample.get('yolo_model', '')}`|",
            f"|YOLO device|`{sample.get('yolo_device', '')}` runtime `{sample.get('yolo_runtime_device', '')}`|",
            f"|YOLO conf / iou / imgsz|`{sample.get('yolo_conf', '')}` / `{sample.get('yolo_iou', '')}` / `{sample.get('yolo_imgsz', '')}`|",
            f"|tracker|`{sample.get('yolo_tracker', '')}`，single target `{sample.get('yolo_single_target_mode', '')}`|",
            f"|相机外参|`x={sample.get('camera_x', '')}, y={sample.get('camera_y', '')}, z={sample.get('camera_z', '')}`|",
            f"|FOV / resolution|`{sample.get('fov_deg', '')} deg`, `{sample.get('image_width_runtime', sample.get('width', ''))}x{sample.get('image_height_runtime', sample.get('height', ''))}`|",
            f"|高度差|`{sample.get('intruder_altitude_offset_m', '')} m`|",
            f"|目标速度 / speed ratio|`{sample.get('intruder_speed', '')} m/s` / `{sample.get('speed_ratio', '')}`|",
            f"|rate_hz|`{sample.get('rate_hz', '')}`|",
            f"|guidance output|`{sample.get('guidance_output_mode', '')}`|",
            f"|max guidance accel|`{sample.get('max_guidance_accel_mps2', '')} m/s^2`|",
            f"|min speed ratio|`{sample.get('min_speed_ratio', '')}`|",
            f"|body-rate tilt / attitude P|`{sample.get('body_rate_max_tilt_deg', '')} deg` / `{sample.get('body_rate_attitude_p', '')}`|",
            f"|body-rate roll/pitch max rate|`{sample.get('body_rate_max_roll_rate_deg', '')}` / `{sample.get('body_rate_max_pitch_rate_deg', '')} deg/s`|",
            f"|body-rate thrust|min/hover/max `{sample.get('body_rate_min_thrust', '')}` / `{sample.get('body_rate_hover_thrust', '')}` / `{sample.get('body_rate_max_thrust', '')}`|",
            f"|body-rate speed hold|gain `{sample.get('body_rate_speed_hold_gain', '')}`, max accel `{sample.get('body_rate_speed_hold_max_accel_mps2', '')} m/s^2`, total limit `{sample.get('body_rate_total_accel_limit_mps2', '')} m/s^2`|",
            f"|LOS filter|`{sample.get('los_filter_enabled', '')}`|",
            f"|frame_guard|`{meta.get('frame_guard', '')}`|",
            f"|bbox noise|`{sample.get('bbox_noise_enabled', '')}`|",
        ]
    )


def plot_summary(rows: list[CaseRow], output: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    ax_min, ax_hit, ax_detect, ax_fps = axes.flat
    for label in ("TTC", "VM"):
        group = [row for row in rows if row.label == label]
        if not group:
            continue
        x = [row.start_range_m for row in group]
        ax_min.plot(x, [row.min_range_m for row in group], marker="o", label=LABELS[label])
        ax_hit.plot(x, [1 if row.hit else 0 for row in group], marker="o", label=LABELS[label])
        ax_detect.plot(x, [100.0 * row.detected_frames / max(1, row.frames) for row in group], marker="o", label=LABELS[label])
        ax_fps.plot(x, [row.avg_detector_fps for row in group], marker="o", label=LABELS[label])
    ax_min.set_title("Minimum true center range")
    ax_min.set_ylabel("m")
    ax_hit.set_title("Collision accepted")
    ax_hit.set_ylabel("hit")
    ax_detect.set_title("YOLO detection frame ratio")
    ax_detect.set_ylabel("%")
    ax_fps.set_title("Detector FPS")
    ax_fps.set_ylabel("FPS")
    for ax in axes.flat:
        ax.set_xlabel("Initial horizontal range / m")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_per_distance(rows: list[CaseRow], output_dir: Path) -> dict[float, Path]:
    images: dict[float, Path] = {}
    for range_m in sorted({row.start_range_m for row in rows if math.isfinite(row.start_range_m)}):
        group = [row for row in rows if row.start_range_m == range_m]
        fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=False)
        ax_range, ax_bbox, ax_ttc, ax_load, ax_fps = axes
        for row in group:
            samples = _read_csv(row.csv_path) if row.csv_path.exists() else []
            t = _series(samples, "t")
            label = LABELS[row.label]
            ax_range.plot(t, _series(samples, "range"), linewidth=1.1, label=f"{label} {'hit' if row.hit else 'miss'}")
            ax_bbox.plot(t, _series(samples, "bbox_area"), linewidth=1.0, label=label)
            ax_ttc.plot(t, _series(samples, "ttc"), linewidth=1.0, label=label)
            ax_load.plot(t, _series(samples, "load_factor_fd_g"), linewidth=1.0, label=f"{label} actual")
            ax_load.plot(t, _guidance_load(samples), linewidth=0.9, linestyle="--", label=f"{label} required")
            ax_fps.plot(t, _series(samples, "detector_fps"), linewidth=1.0, label=label)
        ax_range.set_title(f"{range_m:.0f}m true center range")
        ax_range.set_ylabel("m")
        ax_bbox.set_title("BBox area ratio")
        ax_bbox.set_ylabel("area")
        ax_ttc.set_title("TTC estimate")
        ax_ttc.set_ylabel("s")
        ax_load.set_title("Actual overload and required overload")
        ax_load.set_ylabel("g")
        ax_fps.set_title("Detector FPS")
        ax_fps.set_xlabel("Time / s")
        ax_fps.set_ylabel("FPS")
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=6, ncol=2)
        fig.tight_layout()
        path = output_dir / f"yolo_sitl_ttc_vm_{int(round(range_m)):03d}m.png"
        fig.savefig(path, dpi=170)
        plt.close(fig)
        images[range_m] = path
    return images


def _case_notes(rows: list[CaseRow]) -> str:
    lines: list[str] = []
    for label in ("TTC", "VM"):
        group = [row for row in rows if row.label == label]
        if not group:
            continue
        hit_ranges = ", ".join(f"{row.start_range_m:.0f}m" for row in group if row.hit) or "-"
        miss_ranges = ", ".join(f"{row.start_range_m:.0f}m" for row in group if not row.hit) or "-"
        det = sum(row.detected_frames for row in group) / max(1, sum(row.frames for row in group))
        valid = sum(row.valid_frames for row in group) / max(1, sum(row.frames for row in group))
        fps = [row.avg_detector_fps for row in group if math.isfinite(row.avg_detector_fps)]
        lines.append(
            f"- {label}: 命中 `{sum(row.hit for row in group)}/{len(group)}`，命中距离 `{hit_ranges}`，"
            f"未命中 `{miss_ranges}`，检测帧比例 `{100.0 * det:.1f}%`，有效导引帧比例 `{100.0 * valid:.1f}%`，"
            f"平均检测 FPS `{(sum(fps) / len(fps) if fps else math.nan):.2f}`。"
        )
    lines.append(
        "- 本轮使用真实 YOLOv8 + ByteTrack，因此检测连续性和 GPU 推理速度会直接进入闭环；结果不能和 AirSim detect 函数的理想 bbox 直接等价比较。"
    )
    lines.append(
        "- `accel_integral` 模式的 `n_cmd_g` 来自导引层 `a_cmd`，底层仍通过 PX4/AirSim 速度 setpoint 闭环；实际过载由真实速度差分估计，因此会同时受 PX4 响应、速度限幅和视觉帧率影响。"
    )
    lines.append(
        "- `accel_body_rate` 模式下 `n_cmd_g` 仍表示纯 PNG 需用过载；实际发送给 PX4 的是 `SET_ATTITUDE_TARGET` 机体系 `p/q/r` 角速度和归一化 thrust，日志中的 `body_rate_control_accel_*` 额外包含沿 LOS 的速度保持加速度。"
    )
    return "\n".join(lines)


def write_report(
    rows: list[CaseRow],
    stamp: str,
    summary_img: Path,
    per_distance: dict[float, Path],
    *,
    title: str,
    range_note: str,
) -> None:
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    image_lines = "\n".join(f"![{int(range_m)}m]({_rel(path)})" for range_m, path in sorted(per_distance.items()))
    report = f"""# {title}

## 1. 实验目的

按照此前已命中的 YOLO 案例配置，改用真正 PX4 SITL actor 场景，比较两种捷联视觉比例导引。本报告优先使用 `n_cmd_g` 作为需用过载；旧日志没有该字段时才回退到 `g_eval` 等效过载。

- `TTC` 组：`ttc_png`，TTC 只参与增益调度，并保留 LOS/Vm soft guidance。
- `VM` 组：`fixed_vm_png`，不使用 TTC，固定 `N * V_m` 导引增益。
- `accel_integral` 输出模式：导引律先计算 `a_cmd` / `n_cmd_g`，再按当前仿真步长积分为速度 setpoint；这不是直接向 PX4 发送加速度 setpoint。
- `accel_body_rate` 输出模式：导引律先计算 PNG 需用加速度，再转换为 PX4 `SET_ATTITUDE_TARGET` 机体系角速度 `p/q/r` 和 thrust；速度只作为沿 LOS 保速参考，不再把 PNG 横向修正直接加到速度指令上。

{range_note}

## 2. 基准条件

{_settings_markdown(rows, stamp)}

## 3. 总览图

![summary]({_rel(summary_img)})

## 4. 汇总表

{_summary_table(rows)}

## 5. 明细表

{_detail_table(rows)}

## 6. 分距离曲线

每个距离一张图，包含真实中心距离、bbox 面积、TTC 估计、实际过载/需用过载和 YOLO 检测 FPS。

{image_lines}

## 7. 结论

{_case_notes(rows)}
"""
    REPORT_PATH.write_text(report, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate YOLO SITL TTC/Vm report.")
    parser.add_argument("--stamp", required=True)
    parser.add_argument("--report-path", default=str(REPORT_PATH))
    parser.add_argument("--asset-dir", default=str(ASSET_DIR))
    parser.add_argument("--title", default=DEFAULT_TITLE)
    parser.add_argument("--range-note", default=DEFAULT_RANGE_NOTE)
    return parser.parse_args()


def main() -> None:
    global REPORT_PATH, ASSET_DIR
    args = parse_args()
    REPORT_PATH = _resolve(args.report_path)
    ASSET_DIR = _resolve(args.asset_dir)
    rows = load_rows(args.stamp)
    if not rows:
        raise SystemExit(f"no rows found for stamp {args.stamp}")
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    summary_img = ASSET_DIR / f"summary_{args.stamp}.png"
    plot_summary(rows, summary_img)
    per_distance = plot_per_distance(rows, ASSET_DIR)
    write_report(rows, args.stamp, summary_img, per_distance, title=args.title, range_note=args.range_note)
    print(f"report={REPORT_PATH}")
    print(f"stamp={args.stamp}")


if __name__ == "__main__":
    main()
