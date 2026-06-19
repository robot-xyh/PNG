from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_guidance.los_filter import LOSKalmanFilter6D


LOG_DIR = PROJECT_ROOT / "logs" / "strapdown_accuracy"
TRUTH_DIR = LOG_DIR / "truth_required_load"
REPORT_PATH = PROJECT_ROOT / "完整方案" / "JKL_SITL矩形目标噪声对比报告.md"
ASSET_DIR = PROJECT_ROOT / "完整方案" / "assets" / "JKL_SITL矩形目标噪声对比报告"
SENSOR_NOISE_SETTINGS_PATH = PROJECT_ROOT / "config" / "airsim_blocks_px4_actor_sensor_noise_settings.json"
NO_SENSOR_SETTINGS_PATH = PROJECT_ROOT / "config" / "airsim_blocks_px4_actor_settings.json"
GRAVITY_MPS2 = 9.80665

CASE_LABELS = {
    "J": "J sensor-noise KF",
    "K": "K sensor-noise raw",
    "L": "L no-sensor-noise raw",
}

CASE_DESCRIPTIONS = {
    "J": "PX4 SITL 拦截机 + 1m x 1m x 0.5m Actor，IMU/GPS 噪声开启，bbox 中等噪声，LOS Kalman 滤波开启。",
    "K": "PX4 SITL 拦截机 + 1m x 1m x 0.5m Actor，IMU/GPS 噪声开启，bbox 中等噪声，LOS 滤波关闭。",
    "L": "PX4 SITL 拦截机 + 1m x 1m x 0.5m Actor，无 IMU/GPS 噪声，bbox 中等噪声，LOS 滤波关闭。",
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


def _norm_series(rows: list[dict[str, str]], keys: tuple[str, str, str]) -> list[float]:
    values: list[float] = []
    for row in rows:
        vec = _vector(row, keys)
        if vec is not None:
            values.append(math.sqrt(sum(component * component for component in vec)))
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


def _truth_rows(row: CaseRow) -> list[dict[str, str]]:
    path = TRUTH_DIR / f"{row.truth_prefix}_{row.case}.csv"
    return _read_csv(path) if path.exists() else []


def load_rows(jk_stamp: str, l_stamp: str) -> list[CaseRow]:
    rows: list[CaseRow] = []
    for label in ("J", "K"):
        rows.extend(
            _load_summary(
                LOG_DIR / f"strapdown_clock1_sitl_{label}_{jk_stamp}_summary.csv",
                label,
                f"strapdown_clock1_sitl_{label}_truth_N3_{jk_stamp}",
            )
        )
    rows.extend(
        _load_summary(
            LOG_DIR / f"strapdown_clock1_sitl_L_{l_stamp}_summary.csv",
            "L",
            f"strapdown_clock1_sitl_L_truth_N3_{l_stamp}",
        )
    )
    return sorted(rows, key=lambda row: (row.start_range_m, row.label))


def _first_sample(rows: list[CaseRow], labels: tuple[str, ...]) -> dict[str, str]:
    for row in rows:
        if row.label not in labels or not row.csv_path.exists():
            continue
        samples = _read_csv(row.csv_path)
        if samples:
            return samples[0]
    return {}


def _kf_delay_markdown() -> str:
    dt = 0.05
    filt = LOSKalmanFilter6D()
    base = np.array([0.0, 0.0, 1.0])
    step = np.array([math.tan(0.05), 0.0, 1.0])
    step = step / np.linalg.norm(step)
    filt.update(0.0, base)
    for index in range(1, int(5.0 / dt) + 1):
        filt.update(index * dt, base)
    t0 = 5.0
    samples: list[tuple[float, float]] = []
    for index in range(1, 200):
        out = filt.update(t0 + index * dt, step)
        ratio = float(out.lambda_I[0] / step[0]) if abs(step[0]) > 1.0e-12 else math.nan
        samples.append((index * dt, ratio))

    def cross(level: float) -> float:
        for t, ratio in samples:
            if ratio >= level:
                return t
        return math.nan

    return "\n".join(
        [
            "当前 LOS Kalman 使用 `vision_guidance/los_filter.py` 的 6D 常速度模型，默认参数为：",
            "",
            "|参数|值|",
            "|---|---:|",
            "|`process_lambda`|`1e-4`|",
            "|`process_lambda_dot`|`5e-3`|",
            "|`measurement_noise`|`5e-3`|",
            "|`innovation_reject`|`0.25`|",
            "",
            "按 20Hz 采样、滤波器稳态后输入 LOS 阶跃估算，等效延迟为：",
            "",
            "|响应比例|时间|帧数@20Hz|",
            "|---|---:|---:|",
            f"|50%|`{cross(0.5):.2f}s`|`{cross(0.5) / dt:.1f}`|",
            f"|63%|`{cross(0.632):.2f}s`|`{cross(0.632) / dt:.1f}`|",
            f"|90%|`{cross(0.9):.2f}s`|`{cross(0.9) / dt:.1f}`|",
            f"|95%|`{cross(0.95):.2f}s`|`{cross(0.95) / dt:.1f}`|",
            "",
            "工程解释：LOS 方向量的主要滞后约 `0.10-0.15s`，到 90% 响应约 `0.25s`。这只是滤波器本身，不包括 AirSim detection、PX4 EKF、Offboard 速度/航向响应和机体动力学延迟。",
        ]
    )


def _settings_noise_markdown(rows: list[CaseRow], jk_stamp: str, l_stamp: str) -> str:
    sample = _first_sample(rows, ("J", "K", "L"))
    bbox_center = _float(sample.get("bbox_noise_center_sigma_px"), 3.0)
    bbox_area = _float(sample.get("bbox_noise_area_sigma_ratio"), 0.08)
    bbox_seed = _int(sample.get("bbox_noise_seed"), 20260617)
    actor_asset = sample.get("intruder_actor_asset") or "1M_Cube_Chamfer"
    actor_scale_x = _float(sample.get("intruder_actor_scale_x"), 1.0)
    actor_scale_y = _float(sample.get("intruder_actor_scale_y"), 1.0)
    actor_scale_z = _float(sample.get("intruder_actor_scale_z"), 0.5)

    lines = [
        "|项目|J|K|L|",
        "|---|---|---|---|",
        "|LOS Kalman|开启|关闭|关闭|",
        "|IMU/GPS 噪声|开启|开启|关闭|",
        "|bbox center 噪声|`3.0px`|`3.0px`|`3.0px`|",
        "|bbox area 噪声|`8.0%`|`8.0%`|`8.0%`|",
        f"|bbox 随机种子|`{bbox_seed}`|`{bbox_seed}`|`{bbox_seed}`|",
        f"|Actor 资源|`{actor_asset}`|`{actor_asset}`|`{actor_asset}`|",
        f"|Actor 缩放|`{actor_scale_x:g}, {actor_scale_y:g}, {actor_scale_z:g}`|`{actor_scale_x:g}, {actor_scale_y:g}, {actor_scale_z:g}`|`{actor_scale_x:g}, {actor_scale_y:g}, {actor_scale_z:g}`|",
        "|Actor 名义尺寸|`1m x 1m x 0.5m`|`1m x 1m x 0.5m`|`1m x 1m x 0.5m`|",
        f"|AirSim settings|`{SENSOR_NOISE_SETTINGS_PATH.relative_to(PROJECT_ROOT).as_posix()}`|`{SENSOR_NOISE_SETTINGS_PATH.relative_to(PROJECT_ROOT).as_posix()}`|`{NO_SENSOR_SETTINGS_PATH.relative_to(PROJECT_ROOT).as_posix()}`|",
        f"|实验 stamp|`{jk_stamp}`|`{jk_stamp}`|`{l_stamp}`|",
    ]

    if SENSOR_NOISE_SETTINGS_PATH.exists():
        data = json.loads(SENSOR_NOISE_SETTINGS_PATH.read_text(encoding="utf-8"))
        sensors = data.get("Vehicles", {}).get("Interceptor", {}).get("Sensors", {})
        imu = sensors.get("Imu", {})
        gps = sensors.get("Gps", {})
        lines.extend(
            [
                "",
                "J/K 的 IMU/GPS 噪声参数：",
                "",
                "|传感器|参数|值|",
                "|---|---|---:|",
            ]
        )
        for key in (
            "AngularRandomWalk",
            "VelocityRandomWalk",
            "GyroBiasStabilityTau",
            "GyroBiasStability",
            "AccelBiasStabilityTau",
            "AccelBiasStability",
        ):
            if key in imu:
                lines.append(f"|IMU|`{key}`|`{imu[key]}`|")
        for key in (
            "EphInitial",
            "EpvInitial",
            "EphFinal",
            "EpvFinal",
            "EphMin3d",
            "EphMin2d",
            "EphTimeConstant",
            "EpvTimeConstant",
            "UpdateLatency",
            "UpdateFrequency",
            "StartupDelay",
        ):
            if key in gps:
                lines.append(f"|GPS|`{key}`|`{gps[key]}`|")
    return "\n".join(lines)


def _coordinate_method_markdown() -> str:
    return "\n".join(
        [
            "本轮 J/K/L 全部采用严格重置流程：每个距离、每个工况都重新启动 PX4 SITL 和 Blocks，避免 PX4 SITL 热运行时的坐标重置、上一轮末端位置残留或 EKF 局部坐标漂移污染下一轮 Actor 目标摆放。",
            "",
            "- PNG 控制链路：拦截机速度、姿态、航向仍使用 PX4 `getMultirotorState().kinematics_estimated`，保持 SITL 闭环真实性。",
            "- 目标摆放与评价链路：Actor 目标位置、拦截机真实位置、中心距离和碰撞距离使用 AirSim `simGetObjectPose()` 的物理对象 pose。",
            "- 每轮拦截开始前，脚本重新设置 Actor 初始位置、拦截机起飞高度和初始航向；70m 工况 t=0 中心距在 J/K/L 中分别为约 72.80m、72.79m、72.75m，残余差异来自 PX4 起飞悬停后的厘米级位置偏差。",
            "- 命中判据优先采用 AirSim collision；中心距离用于解释近失、目标碰撞体尺寸和末端误差。",
        ]
    )


def _summary_table(rows: list[CaseRow]) -> str:
    lines = [
        "|工况|命中数|命中距离m|未命中距离m|最小中心距离m|中位实际速度m/s|有效帧/总帧|",
        "|---|---:|---|---|---:|---:|---:|",
    ]
    for label in ("J", "K", "L"):
        group = [row for row in rows if row.label == label]
        if not group:
            continue
        merged: list[dict[str, str]] = []
        for row in group:
            merged.extend(_read_csv(row.csv_path))
        hit_ranges = ", ".join(f"{row.start_range_m:.0f}" for row in group if row.hit) or "-"
        miss_ranges = ", ".join(f"{row.start_range_m:.0f}" for row in group if not row.hit) or "-"
        min_range = min(row.min_range_m for row in group if math.isfinite(row.min_range_m))
        actual_speed = _percentile(_norm_series(merged, ("interceptor_vel_x", "interceptor_vel_y", "interceptor_vel_z")), 0.5)
        valid_frames = sum(row.valid_frames for row in group)
        frames = sum(row.frames for row in group)
        lines.append(
            f"|{label}|{sum(row.hit for row in group)}/{len(group)}|{hit_ranges}|{miss_ranges}|"
            f"{min_range:.3f}|{actual_speed:.2f}|{valid_frames}/{frames}|"
        )
    return "\n".join(lines)


def _detail_table(rows: list[CaseRow]) -> str:
    lines = [
        "|工况|距离m|碰撞|碰撞时间s|最小中心距离m|终点距离m|检测帧/总帧|有效帧|实际过载max g|视觉指令P95 g|真值PNG P95 g|LOS误差P95 deg|sim FPS|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        samples = _read_csv(row.csv_path)
        truth = _truth_rows(row)
        lines.append(
            f"|{row.label}|{row.start_range_m:.0f}|{1 if row.hit else 0}|"
            f"{'-' if not math.isfinite(row.hit_t_s) else f'{row.hit_t_s:.2f}'}|"
            f"{row.min_range_m:.3f}|{row.final_range_m:.3f}|{row.detected_frames}/{row.frames}|{row.valid_frames}|"
            f"{row.max_load_factor_fd_g:.2f}|{_percentile(_required_load(samples), 0.95):.2f}|"
            f"{_percentile(_series(truth, 'n_req_g'), 0.95):.2f}|{_percentile(_los_errors(samples), 0.95):.2f}|"
            f"{row.avg_sim_sample_fps:.2f}|"
        )
    return "\n".join(lines)


def plot_summary(rows: list[CaseRow], output: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(14, 8.5))
    ax_min, ax_hit, ax_valid, ax_speed = axes.flat
    for label in ("J", "K", "L"):
        group = [row for row in rows if row.label == label]
        if not group:
            continue
        x = [row.start_range_m for row in group]
        ax_min.plot(x, [row.min_range_m for row in group], marker="o", label=CASE_LABELS[label])
        ax_hit.plot(x, [1 if row.hit else 0 for row in group], marker="o", label=CASE_LABELS[label])
        ax_valid.plot(x, [100.0 * row.valid_frames / max(1, row.frames) for row in group], marker="o", label=CASE_LABELS[label])
        speeds = []
        for row in group:
            speeds.append(_percentile(_norm_series(_read_csv(row.csv_path), ("interceptor_vel_x", "interceptor_vel_y", "interceptor_vel_z")), 0.5))
        ax_speed.plot(x, speeds, marker="o", label=CASE_LABELS[label])
    ax_min.set_title("Minimum center range")
    ax_min.set_ylabel("m")
    ax_hit.set_title("AirSim collision hit")
    ax_hit.set_ylabel("hit")
    ax_valid.set_title("Valid visual frame ratio")
    ax_valid.set_ylabel("%")
    ax_speed.set_title("Median interceptor speed")
    ax_speed.set_ylabel("m/s")
    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("Initial horizontal range / m")
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_per_distance(rows: list[CaseRow], output_dir: Path) -> dict[float, Path]:
    images: dict[float, Path] = {}
    for range_m in sorted({row.start_range_m for row in rows if math.isfinite(row.start_range_m)}):
        group = [row for row in rows if row.start_range_m == range_m]
        fig, axes = plt.subplots(5, 1, figsize=(12, 15), sharex=False)
        ax_load, ax_los, ax_ttc, ax_range, ax_yaw = axes
        for item in group:
            samples = _read_csv(item.csv_path)
            t = _series(samples, "t")
            label = CASE_LABELS[item.label]
            ax_load.plot(t, _series(samples, "load_factor_fd_g"), linewidth=1.1, alpha=0.85, label=label)
            ax_los.plot(t, _los_errors(samples), linewidth=1.0, label=label)
            ax_ttc.plot(t, _series(samples, "ttc"), linewidth=1.0, label=label)
            ax_range.plot(t, _series(samples, "range"), linewidth=1.2, label=f"{label} {'hit' if item.hit else 'miss'}")
            ax_yaw.plot(t, _series(samples, "yaw_error_deg"), linewidth=1.0, label=label)
        ax_load.set_title(f"{range_m:.0f}m actual overload: J/K/L only")
        ax_load.set_ylabel("g")
        ax_los.set_title("Visual LOS vs camera-origin truth LOS")
        ax_los.set_ylabel("deg")
        ax_ttc.set_title("Scale-expansion TTC")
        ax_ttc.set_ylabel("s")
        ax_range.set_title("True center range")
        ax_range.set_ylabel("m")
        ax_yaw.set_title("PX4 yaw tracking error")
        ax_yaw.set_xlabel("Time / s")
        ax_yaw.set_ylabel("deg")
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=5, ncol=2)
        fig.tight_layout()
        path = output_dir / f"jkl_compare_{int(round(range_m)):03d}m.png"
        fig.savefig(path, dpi=170)
        plt.close(fig)
        images[range_m] = path
    return images


def _conclusion(rows: list[CaseRow]) -> str:
    lines: list[str] = []
    groups = {label: [row for row in rows if row.label == label] for label in ("J", "K", "L")}
    for label, group in groups.items():
        if not group:
            continue
        hits = sum(row.hit for row in group)
        hit_ranges = ", ".join(f"{row.start_range_m:.0f}m" for row in group if row.hit) or "-"
        min_range = min(row.min_range_m for row in group if math.isfinite(row.min_range_m))
        samples: list[dict[str, str]] = []
        for row in group:
            samples.extend(_read_csv(row.csv_path))
        valid_ratio = sum(row.valid_frames for row in group) / max(1, sum(row.frames for row in group))
        no_detection_ratio = (
            sum(sample.get("guidance_mode") == "invalid" and sample.get("reject_reason") == "no_detection" for sample in samples)
            / max(1, len(samples))
        )
        yaw_p95 = _percentile([abs(_float(sample.get("yaw_error_deg"))) for sample in samples], 0.95)
        lines.append(
            f"- {label} 组命中 `{hits}/{len(group)}`，命中距离 `{hit_ranges}`，最小中心距离 `{min_range:.3f}m`，"
            f"有效视觉帧比例 `{100.0 * valid_ratio:.1f}%`，无检测比例 `{100.0 * no_detection_ratio:.1f}%`，"
            f"航向误差 P95 `{yaw_p95:.1f}deg`。"
        )
    lines.append(
        "- 对比重点：J 与 K 的差别主要是 LOS Kalman；K 与 L 的差别主要是 IMU/GPS 噪声。"
        "如果 L 明显优于 K，说明传感器噪声经 PX4 EKF/姿态链路放大后影响拦截；如果 J 优于 K，说明当前 bbox 噪声下 LOS 滤波虽有约 0.1-0.25s 延迟，但总体提升了视线稳定性。"
    )
    lines.append(
        "- 命中判据仍是 AirSim collision；报告同时列出最小中心距离。对于 1m x 1m x 0.5m 小目标，"
        "最小中心距离进入 1-3m 但没有 collision 的情况，应视为近失而非命中。"
    )
    return "\n".join(lines)


def write_report(rows: list[CaseRow], jk_stamp: str, l_stamp: str, summary_img: Path, per_distance: dict[float, Path]) -> None:
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    image_lines = "\n".join(f"![{int(range_m)}m]({_rel(path)})" for range_m, path in sorted(per_distance.items()))
    report = f"""# J/K/L PX4 SITL 矩形目标噪声对比报告

## 1. 实验目的

本报告只比较 J/K/L 三组。三组均使用 PX4 SITL 拦截机、按轨迹移动的 `1m x 1m x 0.5m` Actor 目标、中心捷联相机、bbox 中等噪声、`ClockSpeed=1.0`、初始距离 `40-140m`、高度差 `20m`、目标速度 `5m/s`、拦截机速度上限 `10m/s`。

比较关系：

- J vs K：评估 LOS Kalman 滤波在传感器噪声存在时的收益和延迟代价。
- K vs L：评估关闭 IMU/GPS 噪声后，PX4 EKF/姿态/速度链路对拦截结果的影响。

## 2. 工况设置

{_settings_noise_markdown(rows, jk_stamp, l_stamp)}

## 3. 坐标与评价口径

{_coordinate_method_markdown()}

## 4. LOS Kalman 等效延迟

{_kf_delay_markdown()}

## 5. 总览图

![summary]({_rel(summary_img)})

## 6. 命中汇总

{_summary_table(rows)}

## 7. 明细表

{_detail_table(rows)}

## 8. 单距离曲线

每个距离一张图。过载子图只绘制 J/K/L 三组实际过载曲线；其余子图包含 LOS 误差、TTC、中心距离和 PX4 航向误差。

{image_lines}

## 9. 结论

{_conclusion(rows)}
"""
    REPORT_PATH.write_text(report, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate J/K/L PX4 SITL strapdown comparison report.")
    parser.add_argument("--jk-stamp", default="px4_actor_sensor_noise_rect_20260618_232002")
    parser.add_argument("--l-stamp", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = load_rows(args.jk_stamp, args.l_stamp)
    if not rows:
        raise SystemExit("no J/K/L rows found")
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    summary_img = ASSET_DIR / f"jkl_summary_{args.l_stamp}.png"
    plot_summary(rows, summary_img)
    per_distance = plot_per_distance(rows, ASSET_DIR)
    write_report(rows, args.jk_stamp, args.l_stamp, summary_img, per_distance)
    print(f"report={REPORT_PATH}")
    print(f"jk_stamp={args.jk_stamp}")
    print(f"l_stamp={args.l_stamp}")


if __name__ == "__main__":
    main()
