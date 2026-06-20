from __future__ import annotations

import argparse
import csv
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

LOG_DIR = PROJECT_ROOT / "logs" / "strapdown_accuracy"
TRUTH_DIR = LOG_DIR / "truth_required_load"
REPORT_PATH = PROJECT_ROOT / "完整方案" / "JKL_MNP_目标识别噪声消融12组对比报告.md"
ASSET_DIR = PROJECT_ROOT / "完整方案" / "assets" / "JKL_MNP_目标识别噪声消融12组对比报告"
GRAVITY_MPS2 = 9.80665

CASE_ORDER = ("J", "J0", "K", "K0", "L", "L0", "M", "M0", "N", "N0", "P", "P0")
PAIR_ORDER = (("J", "J0"), ("K", "K0"), ("L", "L0"), ("M", "M0"), ("N", "N0"), ("P", "P0"))

CASE_LABELS = {
    "J": "J TTC sensor KF bbox",
    "J0": "J0 TTC sensor KF no-bbox",
    "K": "K TTC sensor raw bbox",
    "K0": "K0 TTC sensor raw no-bbox",
    "L": "L TTC clean raw bbox",
    "L0": "L0 TTC clean raw no-bbox",
    "M": "M Vm sensor KF bbox",
    "M0": "M0 Vm sensor KF no-bbox",
    "N": "N Vm sensor raw bbox",
    "N0": "N0 Vm sensor raw no-bbox",
    "P": "P Vm clean raw bbox",
    "P0": "P0 Vm clean raw no-bbox",
}

CASE_CONFIG = {
    "J": ("TTC", "on", "on", "on"),
    "J0": ("TTC", "on", "on", "off"),
    "K": ("TTC", "on", "off", "on"),
    "K0": ("TTC", "on", "off", "off"),
    "L": ("TTC", "off", "off", "on"),
    "L0": ("TTC", "off", "off", "off"),
    "M": ("fixed Vm", "on", "on", "on"),
    "M0": ("fixed Vm", "on", "on", "off"),
    "N": ("fixed Vm", "on", "off", "on"),
    "N0": ("fixed Vm", "on", "off", "off"),
    "P": ("fixed Vm", "off", "off", "on"),
    "P0": ("fixed Vm", "off", "off", "off"),
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
    max_load_factor_fd_g: float
    csv_path: Path
    truth_prefix: str


@dataclass
class CaseMetric:
    label: str
    start_range_m: float
    hit: bool
    hit_t_s: float
    min_range_m: float
    final_range_m: float
    frames: int
    detected_frames: int
    valid_frames: int
    detected_ratio: float
    valid_ratio: float
    max_load_factor_fd_g: float
    command_load_p95_g: float
    truth_or_guidance_p95_g: float
    los_error_p95_deg: float
    ttc_p50_s: float
    ttc_valid_ratio: float
    avg_sim_sample_fps: float


def _float(value: object, default: float = math.nan) -> float:
    if value is None or value == "":
        return default
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _int(value: object, default: int = 0) -> int:
    value_float = _float(value)
    return default if not math.isfinite(value_float) else int(value_float)


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


def _series(rows: list[dict[str, str]], key: str) -> list[float]:
    return [_float(row.get(key)) for row in rows]


def _finite(values) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def _mean(values) -> float:
    finite = _finite(values)
    return sum(finite) / len(finite) if finite else math.nan


def _percentile(values, q: float) -> float:
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


def _fmt(value: float, digits: int = 2) -> str:
    return "-" if not math.isfinite(value) else f"{value:.{digits}f}"


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


def _g_eval_load(rows: list[dict[str, str]]) -> list[float]:
    result: list[float] = []
    for row in rows:
        vec = _vector(row, ("g_eval_x", "g_eval_y", "g_eval_z"))
        if vec is None:
            result.append(math.nan)
            continue
        result.append(math.sqrt(sum(value * value for value in vec)) / GRAVITY_MPS2)
    return result


def _truth_rows(row: CaseRow) -> list[dict[str, str]]:
    if not row.truth_prefix:
        return []
    path = TRUTH_DIR / f"{row.truth_prefix}_{row.case}.csv"
    return _read_csv(path) if path.exists() else []


def _summary_path(label: str, stamp: str) -> Path:
    return LOG_DIR / f"strapdown_clock1_sitl_{label}_{stamp}_summary.csv"


def _load_summary(label: str, stamp: str, truth_prefix: str = "") -> list[CaseRow]:
    path = _summary_path(label, stamp)
    if not path.exists():
        print(f"warning: missing summary for {label}: {path}")
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
                max_load_factor_fd_g=_float(item.get("max_load_factor_fd_g")),
                csv_path=_resolve(item.get("csv_path", "")),
                truth_prefix=truth_prefix,
            )
        )
    return rows


def load_rows(jkl_noise_stamp: str, mnp_noise_stamp: str, no_bbox_stamp: str) -> list[CaseRow]:
    rows: list[CaseRow] = []
    for label in ("J", "K", "L"):
        rows.extend(
            _load_summary(label, jkl_noise_stamp, f"strapdown_clock1_sitl_{label}_truth_N3_{jkl_noise_stamp}")
        )
    for label in ("J0", "K0", "L0"):
        rows.extend(_load_summary(label, no_bbox_stamp, f"strapdown_clock1_sitl_{label}_truth_N3_{no_bbox_stamp}"))
    for label in ("M", "N", "P"):
        rows.extend(_load_summary(label, mnp_noise_stamp))
    for label in ("M0", "N0", "P0"):
        rows.extend(_load_summary(label, no_bbox_stamp))
    return sorted(rows, key=lambda row: (row.start_range_m, CASE_ORDER.index(row.label)))


def _metric(row: CaseRow) -> CaseMetric:
    samples = _read_csv(row.csv_path) if row.csv_path.exists() else []
    truth = _truth_rows(row)
    if row.label.startswith(("J", "K", "L")):
        truth_or_guidance = _percentile(_series(truth, "n_req_g"), 0.95)
    else:
        truth_or_guidance = _percentile(_g_eval_load(samples), 0.95)
    ttc_values = [value for value in _series(samples, "ttc") if math.isfinite(value) and value > 0.0]
    ttc_valid = [
        1.0
        for sample in samples
        if math.isfinite(_float(sample.get("ttc"))) and _float(sample.get("ttc")) > 0.0
    ]
    return CaseMetric(
        label=row.label,
        start_range_m=row.start_range_m,
        hit=row.hit,
        hit_t_s=row.hit_t_s,
        min_range_m=row.min_range_m,
        final_range_m=row.final_range_m,
        frames=row.frames,
        detected_frames=row.detected_frames,
        valid_frames=row.valid_frames,
        detected_ratio=row.detected_frames / max(1, row.frames),
        valid_ratio=row.valid_frames / max(1, row.frames),
        max_load_factor_fd_g=row.max_load_factor_fd_g,
        command_load_p95_g=_percentile(_required_load(samples), 0.95),
        truth_or_guidance_p95_g=truth_or_guidance,
        los_error_p95_deg=_percentile(_los_errors(samples), 0.95),
        ttc_p50_s=_percentile(ttc_values, 0.50),
        ttc_valid_ratio=len(ttc_valid) / max(1, len(samples)),
        avg_sim_sample_fps=row.avg_sim_sample_fps,
    )


def build_metrics(rows: list[CaseRow]) -> list[CaseMetric]:
    return [_metric(row) for row in rows]


def write_metric_csv(metrics: list[CaseMetric], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(CaseMetric.__dataclass_fields__.keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for item in metrics:
            writer.writerow({field: getattr(item, field) for field in fields})


def _group(metrics: list[CaseMetric], label: str) -> list[CaseMetric]:
    return [item for item in metrics if item.label == label]


def _aggregate_table(metrics: list[CaseMetric]) -> str:
    lines = [
        "|工况|导引|IMU/GPS噪声|LOS滤波|bbox噪声|命中数|最小中心距m|平均最小距m|检测帧率%|有效帧率%|LOS误差P95中位deg|指令P95中位g|sim FPS|",
        "|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for label in CASE_ORDER:
        group = _group(metrics, label)
        if not group:
            continue
        guidance, sensor, los_filter, bbox = CASE_CONFIG[label]
        lines.append(
            f"|{label}|{guidance}|{sensor}|{los_filter}|{bbox}|"
            f"{sum(item.hit for item in group)}/{len(group)}|"
            f"{_fmt(min(item.min_range_m for item in group if math.isfinite(item.min_range_m)), 3)}|"
            f"{_fmt(_mean(item.min_range_m for item in group), 3)}|"
            f"{_fmt(100.0 * _mean(item.detected_ratio for item in group), 1)}|"
            f"{_fmt(100.0 * _mean(item.valid_ratio for item in group), 1)}|"
            f"{_fmt(_percentile([item.los_error_p95_deg for item in group], 0.5), 2)}|"
            f"{_fmt(_percentile([item.command_load_p95_g for item in group], 0.5), 2)}|"
            f"{_fmt(_mean(item.avg_sim_sample_fps for item in group), 2)}|"
        )
    return "\n".join(lines)


def _pair_table(metrics: list[CaseMetric]) -> str:
    lines = [
        "|成对比较|导引|IMU/GPS噪声|LOS滤波|有噪声命中|无噪声命中|平均最小距变化m|LOS误差P95变化deg|有效帧率变化百分点|说明|",
        "|---|---|---|---|---:|---:|---:|---:|---:|---|",
    ]
    for noisy, clean in PAIR_ORDER:
        left = _group(metrics, noisy)
        right = _group(metrics, clean)
        if not left or not right:
            continue
        guidance, sensor, los_filter, _ = CASE_CONFIG[noisy]
        delta_min = _mean(item.min_range_m for item in right) - _mean(item.min_range_m for item in left)
        delta_los = _percentile([item.los_error_p95_deg for item in right], 0.5) - _percentile(
            [item.los_error_p95_deg for item in left], 0.5
        )
        delta_valid = 100.0 * (_mean(item.valid_ratio for item in right) - _mean(item.valid_ratio for item in left))
        note = "负值代表无噪声更近/误差更小" if delta_min < 0 or delta_los < 0 else "正值代表无噪声指标更大"
        lines.append(
            f"|{noisy} -> {clean}|{guidance}|{sensor}|{los_filter}|"
            f"{sum(item.hit for item in left)}/{len(left)}|{sum(item.hit for item in right)}/{len(right)}|"
            f"{_fmt(delta_min, 3)}|{_fmt(delta_los, 2)}|{_fmt(delta_valid, 1)}|{note}|"
        )
    return "\n".join(lines)


def _distance_table(metrics: list[CaseMetric]) -> str:
    lines = [
        "|距离m|J|J0|K|K0|L|L0|M|M0|N|N0|P|P0|",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    ranges = sorted({item.start_range_m for item in metrics if math.isfinite(item.start_range_m)})
    for range_m in ranges:
        cells = [f"{range_m:.0f}"]
        for label in CASE_ORDER:
            found = [item for item in metrics if item.label == label and item.start_range_m == range_m]
            if not found:
                cells.append("-")
            else:
                item = found[0]
                cells.append(("Y" if item.hit else "N") + f"/{item.min_range_m:.2f}")
        lines.append("|" + "|".join(cells) + "|")
    return "\n".join(lines)


def _style_for(label: str) -> tuple[str, str]:
    colors = {
        "J": "#1f77b4",
        "K": "#ff7f0e",
        "L": "#2ca02c",
        "M": "#9467bd",
        "N": "#8c564b",
        "P": "#e377c2",
    }
    base = label.rstrip("0")
    return colors.get(base, "#333333"), "--" if label.endswith("0") else "-"


def plot_summary(metrics: list[CaseMetric], output: Path) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(17, 9.5))
    ax_min, ax_hit, ax_valid, ax_detect, ax_los, ax_cmd = axes.flat
    for label in CASE_ORDER:
        group = _group(metrics, label)
        if not group:
            continue
        color, linestyle = _style_for(label)
        x = [item.start_range_m for item in group]
        kwargs = {"marker": "o", "linewidth": 1.0, "markersize": 3.0, "label": label, "color": color, "linestyle": linestyle}
        ax_min.plot(x, [item.min_range_m for item in group], **kwargs)
        ax_hit.plot(x, [1.0 if item.hit else 0.0 for item in group], **kwargs)
        ax_valid.plot(x, [100.0 * item.valid_ratio for item in group], **kwargs)
        ax_detect.plot(x, [100.0 * item.detected_ratio for item in group], **kwargs)
        ax_los.plot(x, [item.los_error_p95_deg for item in group], **kwargs)
        ax_cmd.plot(x, [item.command_load_p95_g for item in group], **kwargs)
    ax_min.set_title("Minimum center range")
    ax_min.set_ylabel("m")
    ax_hit.set_title("Collision hit")
    ax_hit.set_ylabel("hit")
    ax_valid.set_title("Valid LOS frame ratio")
    ax_valid.set_ylabel("%")
    ax_detect.set_title("Detection frame ratio")
    ax_detect.set_ylabel("%")
    ax_los.set_title("LOS error P95")
    ax_los.set_ylabel("deg")
    ax_cmd.set_title("Velocity-command equivalent overload P95")
    ax_cmd.set_ylabel("g")
    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("Initial horizontal range / m")
        ax.legend(fontsize=6, ncol=2)
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_per_distance(rows: list[CaseRow], output_dir: Path) -> dict[float, Path]:
    images: dict[float, Path] = {}
    ranges = sorted({row.start_range_m for row in rows if math.isfinite(row.start_range_m)})
    for range_m in ranges:
        group = [row for row in rows if row.start_range_m == range_m]
        fig, axes = plt.subplots(5, 2, figsize=(17, 17), sharex=False)
        group_sets = [("TTC: J/K/L/J0/K0/L0", ("J", "J0", "K", "K0", "L", "L0")), ("fixed Vm: M/N/P/M0/N0/P0", ("M", "M0", "N", "N0", "P", "P0"))]
        for col, (title, labels) in enumerate(group_sets):
            ax_actual, ax_cmd, ax_los, ax_range, ax_ttc = axes[:, col]
            ax_actual.set_title(f"{range_m:.0f}m {title}: actual overload")
            ax_cmd.set_title("command/theory equivalent overload")
            ax_los.set_title("visual LOS vs camera-origin truth LOS")
            ax_range.set_title("true center range")
            ax_ttc.set_title("TTC / bbox area")
            for label in labels:
                row_list = [row for row in group if row.label == label]
                if not row_list:
                    continue
                row = row_list[0]
                samples = _read_csv(row.csv_path)
                t = _series(samples, "t")
                color, linestyle = _style_for(label)
                plot_args = {"linewidth": 1.0, "label": f"{label} {'hit' if row.hit else 'miss'}", "color": color, "linestyle": linestyle}
                ax_actual.plot(t, _series(samples, "load_factor_fd_g"), **plot_args)
                if label.startswith(("J", "K", "L")):
                    truth = _truth_rows(row)
                    ax_cmd.plot(_series(truth, "t"), _series(truth, "n_req_g"), **plot_args)
                else:
                    ax_cmd.plot(t, _g_eval_load(samples), **plot_args)
                ax_los.plot(t, _los_errors(samples), **plot_args)
                ax_range.plot(t, _series(samples, "range"), **plot_args)
                if label.startswith(("J", "K", "L")):
                    ax_ttc.plot(t, _series(samples, "ttc"), **plot_args)
                    ax_ttc.set_ylabel("s")
                else:
                    ax_ttc.plot(t, _series(samples, "bbox_area"), **plot_args)
                    ax_ttc.set_ylabel("area")
            for ax in (ax_actual, ax_cmd, ax_los, ax_range, ax_ttc):
                ax.grid(True, alpha=0.3)
                ax.legend(fontsize=5, ncol=2)
        axes[0, 0].set_ylabel("g")
        axes[1, 0].set_ylabel("g")
        axes[2, 0].set_ylabel("deg")
        axes[3, 0].set_ylabel("m")
        axes[4, 0].set_xlabel("Time / s")
        axes[4, 1].set_xlabel("Time / s")
        fig.tight_layout()
        path = output_dir / f"bbox_noise_ablation_{int(round(range_m)):03d}m.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        images[range_m] = path
    return images


def _config_table() -> str:
    lines = [
        "|标签|导引律|IMU/GPS噪声|LOS Kalman|bbox目标识别噪声|",
        "|---|---|---|---|---|",
    ]
    for label in CASE_ORDER:
        guidance, sensor, los_filter, bbox = CASE_CONFIG[label]
        lines.append(f"|{label}|{guidance}|{sensor}|{los_filter}|{bbox}|")
    return "\n".join(lines)


def write_report(
    metrics: list[CaseMetric],
    rows: list[CaseRow],
    args: argparse.Namespace,
    metric_csv: Path,
    summary_img: Path,
    per_distance: dict[float, Path],
) -> None:
    image_lines = "\n".join(f"![{int(range_m)}m]({_rel(path)})" for range_m, path in sorted(per_distance.items()))
    missing = [label for label in CASE_ORDER if not _group(metrics, label)]
    missing_text = "无。" if not missing else ", ".join(missing)
    report = f"""# J/K/L/M/N/P 目标识别噪声消融 12 组对比报告

## 1. 实验目的

本报告把已有 `J/K/L`、`M/N/P` 六组与新跑的 `J0/K0/L0`、`M0/N0/P0` 六组放在同一口径下比较。新组唯一变化是关闭目标识别框噪声：不注入 bbox 中心点随机噪声，也不注入 bbox 面积随机噪声。

对比目标：

- `J/K/L` 与 `J0/K0/L0`：评估 6D LOS + TTC 视觉比例导引对目标识别噪声的敏感性。
- `M/N/P` 与 `M0/N0/P0`：评估 6D LOS + 固定 `V_m` 比例导引对目标识别噪声的敏感性。
- 同时保留 IMU/GPS 噪声、LOS Kalman 开关、导引律差异，用于判断误差来源。

## 2. 数据来源

- 有 bbox 噪声的 J/K/L stamp：`{args.jkl_noise_stamp}`
- 有 bbox 噪声的 M/N/P stamp：`{args.mnp_noise_stamp}`
- 无 bbox 噪声的新实验 stamp：`{args.no_bbox_stamp}`
- 缺失标签：{missing_text}
- 明细指标 CSV：[{metric_csv.name}]({_rel(metric_csv)})

## 3. 工况定义

{_config_table()}

共同条件：PX4 SITL 拦截机，按轨迹移动的 `1m x 1m x 0.5m` Actor 目标，中心捷联相机，`ClockSpeed=1.0`，初始水平距离 `40-140m`，高度差 `20m`，目标速度 `5m/s`，拦截机速度上限约 `10m/s`。

有 bbox 噪声组使用 `bbox center sigma = 3.0px`、`bbox area sigma = 8.0%`。无 bbox 噪声组显式记录 `bbox_noise_enabled=0`，且中心/面积噪声幅值均为 `0`。

## 4. 总览图

实线为有 bbox 噪声组，虚线为无 bbox 噪声组；颜色按对应基准组保持一致。

![summary]({_rel(summary_img)})

## 5. 汇总表

{_aggregate_table(metrics)}

## 6. 成对消融比较

{_pair_table(metrics)}

## 7. 各距离命中和最小距离

单元格格式为 `Y/最小中心距` 或 `N/最小中心距`。

{_distance_table(metrics)}

## 8. 单距离曲线

每个距离一张图，左列为 TTC 方案，右列为固定 `V_m` 方案。TTC 组的指令/理论子图绘制影子真值 PNG 需用过载；固定 `V_m` 组绘制 `g_eval` 等效过载。

{image_lines}

## 9. 解释口径

- 如果无 bbox 噪声组 LOS 误差明显下降，但命中率没有同步提升，主要瓶颈更可能在 PX4 速度/航向响应、末端裁切/盲推或几何可达性，而不是检测框噪声。
- 如果无 bbox 噪声组 TTC 有效帧率明显提高，说明面积通道对 bbox 噪声敏感；此时实机应优先加强 bbox 面积低通、裁切门控和目标姿态变化剔除。
- 如果固定 `V_m` 组比 TTC 组对 bbox 噪声更稳，说明当前主要问题来自 TTC 面积通道；如果两者都明显受影响，问题更可能来自 bbox center 对 LOS 角速度的扰动。
- `速度指令P95 g` 是上层速度指令变化率的等效过载，不等同于机体真实过载；真实过载以 `实际过载max g` 和曲线中的实际过载为准。
"""
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(report, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate 12-case JKL/MNP bbox-noise ablation report.")
    parser.add_argument("--jkl-noise-stamp", default="strict_reset_20260618_0528")
    parser.add_argument("--mnp-noise-stamp", default="mnp_fixed_vm_20260618_0636")
    parser.add_argument("--no-bbox-stamp", required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = load_rows(args.jkl_noise_stamp, args.mnp_noise_stamp, args.no_bbox_stamp)
    if not rows:
        raise SystemExit("no rows found")
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    metrics = build_metrics(rows)
    metric_csv = ASSET_DIR / f"case_metrics_{args.no_bbox_stamp}.csv"
    write_metric_csv(metrics, metric_csv)
    summary_img = ASSET_DIR / f"bbox_noise_ablation_summary_{args.no_bbox_stamp}.png"
    plot_summary(metrics, summary_img)
    per_distance = plot_per_distance(rows, ASSET_DIR)
    write_report(metrics, rows, args, metric_csv, summary_img, per_distance)
    print(f"report={REPORT_PATH}")
    print(f"metric_csv={metric_csv}")
    print(f"rows={len(rows)}")


if __name__ == "__main__":
    main()
