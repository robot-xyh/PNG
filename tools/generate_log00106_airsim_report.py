#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

matplotlib.rcParams["font.sans-serif"] = [
    "Noto Sans CJK JP",
    "Droid Sans Fallback",
    "DejaVu Sans",
]
matplotlib.rcParams["axes.unicode_minus"] = False


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "logs/analysis/LOG00106_airsim_log_only"
DEFAULT_FIGURE_DIR = PROJECT_ROOT / "doc/figures/log00106_airsim_log_only"
DEFAULT_REPORT = PROJECT_ROOT / "doc/BETAFLIGHT_LOG00106_AIRSIM_LOG_ONLY_TREND_REPORT.md"
PROFILE_LABELS = {
    "ideal": "理想基线 1.000",
    "measured_0.809": "实测敏感性 0.809",
    "measured_0.847": "乐观边界 0.847",
}
PROFILE_COLORS = {
    "ideal": "#1f77b4",
    "measured_0.809": "#d62728",
    "measured_0.847": "#2ca02c",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate the LOG00106 AirSim LOG_ONLY trend report.")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--run-dir", default="")
    parser.add_argument("--figure-dir", default=str(DEFAULT_FIGURE_DIR))
    parser.add_argument("--report", default=str(DEFAULT_REPORT))
    return parser.parse_args()


def _latest_run(output_root: Path) -> Path:
    marker = output_root / "latest_run.txt"
    if marker.exists():
        candidate = output_root / marker.read_text(encoding="utf-8").strip()
        if candidate.is_dir():
            return candidate
    candidates = sorted(path for path in output_root.glob("run_*") if path.is_dir())
    if not candidates:
        raise FileNotFoundError(f"no LOG00106 AirSim runs under {output_root}")
    return candidates[-1]


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _number(row: Mapping[str, Any], key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def _series(rows: Sequence[Mapping[str, Any]], key: str) -> np.ndarray:
    return np.asarray([_number(row, key) for row in rows], dtype=float)


def _profile_key(metrics: Mapping[str, Any]) -> str:
    if metrics["timing_profile"] == "ideal":
        return "ideal"
    return f"measured_{float(metrics['force_ratio']):.3f}"


def _style(metrics: Mapping[str, Any]) -> tuple[str, str]:
    key = _profile_key(metrics)
    return PROFILE_LABELS.get(key, key), PROFILE_COLORS.get(key, "#555555")


def _event_lines(ax: Any, rows: Sequence[Mapping[str, Any]]) -> None:
    for row in rows:
        time_s = _number(row, "t_algorithm_s")
        if _number(row, "algorithm_exit_event") == 1.0:
            ax.axvline(time_s, color="#555555", linestyle=":", linewidth=1.0)
        if _number(row, "contact_event") == 1.0:
            ax.axvline(time_s, color="#000000", linestyle="--", linewidth=1.0)


def _finish_figure(fig: Any, path: Path, title: str) -> None:
    fig.suptitle(title, fontsize=12)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def _plot_rates(cases: Sequence[tuple[dict[str, Any], list[dict[str, str]]]], path: Path, title: str) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    for metrics, rows in cases:
        label, color = _style(metrics)
        t = _series(rows, "t_algorithm_s")
        axes[0, 0].plot(t, _series(rows, "roll_rate_setpoint_deg_s"), color=color, label=f"{label} setpoint")
        axes[0, 0].plot(t, _series(rows, "roll_rate_actual_deg_s"), color=color, linestyle="--", label=f"{label} actual")
        axes[0, 1].plot(t, _series(rows, "pitch_rate_setpoint_deg_s"), color=color, label=f"{label} setpoint")
        axes[0, 1].plot(t, _series(rows, "pitch_rate_actual_deg_s"), color=color, linestyle="--", label=f"{label} actual")
        axes[1, 0].plot(t, _series(rows, "desired_roll_frd_deg"), color=color, label=f"{label} desired")
        axes[1, 0].plot(t, _series(rows, "roll_frd_deg"), color=color, linestyle="--", label=f"{label} actual")
        axes[1, 1].plot(t, _series(rows, "desired_pitch_frd_deg"), color=color, label=f"{label} desired")
        axes[1, 1].plot(t, _series(rows, "pitch_frd_deg"), color=color, linestyle="--", label=f"{label} actual")
        for ax in axes.flat:
            _event_lines(ax, rows)
    for ax, ylabel in zip(axes.flat, ("Roll rate (deg/s)", "Pitch rate (deg/s)", "Roll FRD (deg)", "Pitch FRD (deg)")):
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=7, ncol=2)
    axes[1, 0].set_xlabel("Algorithm time (s)")
    axes[1, 1].set_xlabel("Algorithm time (s)")
    _finish_figure(fig, path, title)


def _plot_throttle(cases: Sequence[tuple[dict[str, Any], list[dict[str, str]]]], path: Path, title: str) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    for metrics, rows in cases:
        label, color = _style(metrics)
        t = _series(rows, "t_algorithm_s")
        axes[0, 0].plot(t, _series(rows, "throttle_model_target_us"), color=color, linestyle=":", label=f"{label} model")
        axes[0, 0].plot(t, _series(rows, "throttle_handover_output_us"), color=color, label=f"{label} output")
        axes[0, 1].plot(t, _series(rows, "thrust_model_load_factor_g"), color=color, label=f"{label} model")
        axes[0, 1].plot(t, _series(rows, "specific_force_actual_g"), color=color, linestyle="--", label=f"{label} actual")
        axes[1, 0].plot(t, _series(rows, "thrust_model_ratio"), color=color, label=label)
        axes[1, 1].plot(t, _series(rows, "airsim_throttle_command_0_1"), color=color, label=label)
        for ax in axes.flat:
            _event_lines(ax, rows)
    axes[0, 0].set_ylabel("Throttle (us)")
    axes[0, 1].set_ylabel("Specific force / load (g)")
    axes[1, 0].set_ylabel("Plant force ratio")
    axes[1, 1].set_ylabel("AirSim throttle (0-1)")
    for ax in axes.flat:
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=7, ncol=2)
    axes[1, 0].set_xlabel("Algorithm time (s)")
    axes[1, 1].set_xlabel("Algorithm time (s)")
    _finish_figure(fig, path, title)


def _plot_velocity(cases: Sequence[tuple[dict[str, Any], list[dict[str, str]]]], path: Path, title: str) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    for metrics, rows in cases:
        label, color = _style(metrics)
        t = _series(rows, "t_algorithm_s")
        for axis_name, ax in zip(("n", "e", "d"), axes):
            ax.plot(t, _series(rows, f"velocity_reference_{axis_name}_m_s"), color=color, linestyle=":", label=f"{label} ref")
            ax.plot(t, _series(rows, f"interceptor_velocity_{axis_name}_m_s"), color=color, label=f"{label} truth")
            ax.plot(t, _series(rows, f"interceptor_velocity_observed_{axis_name}_m_s"), color=color, linestyle="--", label=f"{label} observed")
            _event_lines(ax, rows)
    for ax, name in zip(axes, ("N", "E", "D")):
        ax.set_ylabel(f"Velocity {name} (m/s)")
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=7, ncol=3)
    axes[-1].set_xlabel("Algorithm time (s)")
    _finish_figure(fig, path, title)


def _plot_acceleration(cases: Sequence[tuple[dict[str, Any], list[dict[str, str]]]], path: Path, title: str) -> None:
    terms = ("speed_accel", "png_accel", "fov_accel", "total_accel")
    columns = ("n_m_s2", "e_m_s2", "d_m_s2", "norm_m_s2")
    fig, axes = plt.subplots(4, 4, figsize=(17, 13), sharex=True)
    for metrics, rows in cases:
        label, color = _style(metrics)
        t = _series(rows, "t_algorithm_s")
        for row_index, term in enumerate(terms):
            for column_index, suffix in enumerate(columns):
                ax = axes[row_index, column_index]
                ax.plot(t, _series(rows, f"{term}_{suffix}"), color=color, label=label)
                _event_lines(ax, rows)
    for row_index, term in enumerate(("Speed", "PNG", "FOV", "Total")):
        axes[row_index, 0].set_ylabel(f"{term} (m/s2)")
    for column_index, component in enumerate(("N", "E", "D", "Norm")):
        axes[0, column_index].set_title(component)
    for ax in axes.flat:
        ax.grid(True, alpha=0.25)
    for ax in axes[-1, :]:
        ax.set_xlabel("Algorithm time (s)")
    axes[0, 0].legend(fontsize=7)
    _finish_figure(fig, path, title)


def _plot_los_bbox(cases: Sequence[tuple[dict[str, Any], list[dict[str, str]]]], path: Path, title: str) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), sharex=True)
    for metrics, rows in cases:
        label, color = _style(metrics)
        t = _series(rows, "t_algorithm_s")
        for component, ax in zip(("n", "e", "d"), (axes[0, 0], axes[0, 1], axes[1, 0])):
            ax.plot(t, _series(rows, f"lambda_truth_{component}"), color=color, linewidth=1.5, label=f"{label} truth")
            ax.plot(t, _series(rows, f"lambda_measured_{component}"), color=color, linestyle=":", label=f"{label} measured")
            ax.plot(t, _series(rows, f"lambda_filtered_{component}"), color=color, linestyle="--", label=f"{label} filtered")
        rate_norm = np.sqrt(sum(_series(rows, f"lambda_dot_{component}_s") ** 2 for component in ("n", "e", "d")))
        axes[1, 1].plot(t, rate_norm, color=color, label=label)
        axes[2, 0].plot(t, _series(rows, "bbox_center_u_px"), color=color, label=f"{label} u")
        axes[2, 0].plot(t, _series(rows, "bbox_center_v_px"), color=color, linestyle="--", label=f"{label} v")
        axes[2, 1].plot(t, _series(rows, "bbox_area_ratio"), color=color, label=label)
        for ax in axes.flat:
            _event_lines(ax, rows)
    for ax, ylabel in zip(
        axes.flat,
        ("LOS N", "LOS E", "LOS D", "LOS rate norm (1/s)", "BBox center (px)", "BBox area ratio"),
    ):
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=6, ncol=2)
    axes[2, 0].set_xlabel("Algorithm time (s)")
    axes[2, 1].set_xlabel("Algorithm time (s)")
    _finish_figure(fig, path, title)


def _plot_state(cases: Sequence[tuple[dict[str, Any], list[dict[str, str]]]], path: Path, title: str) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    for metrics, rows in cases:
        label, color = _style(metrics)
        t = _series(rows, "t_algorithm_s")
        saturation = (
            _series(rows, "speed_saturated")
            + 2.0 * _series(rows, "png_saturated")
            + 4.0 * _series(rows, "fov_saturated")
            + 8.0 * _series(rows, "total_saturated")
        )
        axes[0, 0].step(t, saturation, where="post", color=color, label=label)
        axes[0, 1].step(t, _series(rows, "algorithm_active"), where="post", color=color, label=f"{label} ACTIVE")
        axes[0, 1].step(t, _series(rows, "contact_detected"), where="post", color=color, linestyle="--", label=f"{label} contact")
        axes[1, 0].plot(t, _series(rows, "relative_range_m"), color=color, label=label)
        axes[1, 1].plot(t, _series(rows, "closing_speed_m_s"), color=color, label=label)
        for ax in axes.flat:
            _event_lines(ax, rows)
    axes[0, 0].set_ylabel("Saturation bitmask")
    axes[0, 1].set_ylabel("State")
    axes[1, 0].set_ylabel("Truth range (m)")
    axes[1, 1].set_ylabel("Closing speed (m/s)")
    for ax in axes.flat:
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=7)
    axes[1, 0].set_xlabel("Algorithm time (s)")
    axes[1, 1].set_xlabel("Algorithm time (s)")
    _finish_figure(fig, path, title)


def _format(value: Any, digits: int = 3) -> str:
    if value is None:
        return "-"
    if isinstance(value, bool):
        return "是" if value else "否"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(number):
        return "-"
    return f"{number:.{digits}f}"


def _trend_comparisons(metrics: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], dict[str, Mapping[str, Any]]] = defaultdict(dict)
    for item in metrics:
        grouped[(str(item["distance_label"]), str(item["exit_mode"]))][_profile_key(item)] = item
    results = []
    for (distance, exit_mode), group in sorted(grouped.items()):
        ideal = group.get("ideal")
        measured = group.get("measured_0.809")
        optimistic = group.get("measured_0.847")
        result: dict[str, Any] = {"distance_label": distance, "exit_mode": exit_mode}
        if ideal and measured:
            ideal_force = ideal["specific_force_actual_g_pre_contact"]["p50"]
            measured_force = measured["specific_force_actual_g_pre_contact"]["p50"]
            result["measured_force_lower_than_ideal"] = (
                ideal_force is not None and measured_force is not None and measured_force < ideal_force
            )
            result["measured_contact_not_earlier_than_ideal"] = _not_earlier(measured, ideal)
            result["measured_minimum_range_not_smaller_than_ideal"] = (
                float(measured["minimum_truth_range_m"]) >= float(ideal["minimum_truth_range_m"]) - 0.05
            )
        if measured and optimistic:
            result["optimistic_force_not_lower_than_main"] = (
                float(optimistic["specific_force_actual_g_pre_contact"]["p50"])
                >= float(measured["specific_force_actual_g_pre_contact"]["p50"]) - 0.02
            )
        results.append(result)
    return results


def _not_earlier(candidate: Mapping[str, Any], baseline: Mapping[str, Any]) -> bool:
    candidate_time = candidate.get("contact_time_s")
    baseline_time = baseline.get("contact_time_s")
    if baseline_time is None:
        return candidate_time is None
    return candidate_time is None or float(candidate_time) >= float(baseline_time) - 0.05


def _report_text(
    run_dir: Path,
    summary: Mapping[str, Any],
    metrics: Sequence[Mapping[str, Any]],
    figures: Mapping[tuple[str, str], Sequence[Path]],
    comparisons: Sequence[Mapping[str, Any]],
    report_path: Path,
) -> str:
    input_hash_rows = [
        f"|`{name}`|`{digest}`|"
        for name, digest in sorted(summary["input_sha256"].items())
    ]
    measured_metrics = [item for item in metrics if item["timing_profile"] == "measured"]
    roll_lags = [float(item["rate_tracking"]["roll"]["lag_ms"]) for item in measured_metrics]
    pitch_lags = [float(item["rate_tracking"]["pitch"]["lag_ms"]) for item in measured_metrics]
    rate_correlations = [
        float(item["rate_tracking"][axis]["correlation"])
        for item in measured_metrics
        for axis in ("roll", "pitch")
    ]
    control_periods = [float(item["control_timing"]["sample_period_mean_ms"]) for item in metrics]
    hard_checks_passed = all(all(item["trend_checks"].values()) for item in metrics)
    rows = []
    for item in sorted(metrics, key=lambda value: (value["distance_label"], value["exit_mode"], -float(value["force_ratio"]))):
        rows.append(
            "|{case}|{distance}|{exit_mode}|{timing}|{ratio}|{outcome}|{minimum}|{contact}|{exit_range}|{accel}|{throttle}|".format(
                case=item["case_id"],
                distance=item["distance_label"],
                exit_mode="提前退出" if item["exit_mode"] == "early" else "持续闭环",
                timing="即时" if item["timing_profile"] == "ideal" else "40 组实测配对时序",
                ratio=_format(item["force_ratio"]),
                outcome="接触" if item["contact_detected"] else "未接触",
                minimum=_format(item["minimum_truth_range_m"]),
                contact=_format(item["contact_time_s"]),
                exit_range=_format(item["algorithm_exit_remaining_range_m"]),
                accel=_format(item["maximum_total_accel_m_s2"]),
                throttle=_format(item["maximum_throttle_us_pre_contact"], 1),
            )
        )
    figure_sections = []
    descriptions = (
        "角速度设定与 AirSim 响应；点线为算法退出，虚线为 Actor 接触。",
        "模型目标油门、交接输出、AirSim 油门及有限差分比力。",
        "NED 速度参考、SimpleFlight 真值速度及 5 Hz 滤波观测速度。",
        "速度建立、PNG、FOV 和总加速度的 N/E/D 分量与模长。",
        "真值 LOS、检测框针孔 LOS、Kalman LOS/LOS rate 和框尺度。",
        "饱和、ACTIVE/退出、Actor 接触、真值距离与闭合速度。",
    )
    for key, paths in sorted(figures.items()):
        distance, exit_mode = key
        figure_sections.append(f"### {distance} / {'提前退出' if exit_mode == 'early' else '持续闭环'}")
        for path, description in zip(paths, descriptions):
            relative = path.relative_to(report_path.parent)
            figure_sections.append(f"![{description}]({relative.as_posix()})\n\n{description}")
    comparison_lines = []
    for item in comparisons:
        checks = [value for key, value in item.items() if key not in {"distance_label", "exit_mode"}]
        status = "通过" if checks and all(checks) else "警告"
        comparison_lines.append(
            f"- `{item['distance_label']} / {item['exit_mode']}`：推力偏差方向性检查为 **{status}**；"
            "该判定只比较当前 AirSim 三条曲线，不回填或平移实测数据。"
        )
    airsim = summary["airsim"]
    return f"""# Betaflight LOG00106 AirSim LOG_ONLY 趋势对比报告

## 1. 证据边界

本报告基于 LOG00106 单次真实物理接触样本建立 AirSim 趋势复现。`simGetDetections` 提供
`airsim_truth_box`，框中心经 AirSim 渲染内参归一化后映射到实机内参，再进入生产 LOS Kalman、
固定速度 PNG、`accel_tilt_rate` 和 0.8 s 角速度/油门交接。没有运行 YOLO/RKNN，也没有访问
真实飞控。理想针孔表示无额外畸变和噪声，不表示绕过检测函数。

真实数据、仿真数据和推断量严格区分：真实距离仅引用接触锚定位置增量；表内最近距离全部为
AirSim 双机真值；有限差分比力和趋势判定属于推断量。

## 2. 配置与复现

|项目|值|
|---|---|
|run ID|`{summary['run_id']}`|
|AirSim|Python `{airsim['python_version']}`，client/server `{airsim['client_version']}/{airsim['server_version']}`|
|模式|`{airsim['mode']}`，RPC `{airsim['host']}:{airsim['port']}`|
|seed|`{summary['seed']}`|
|图像/相机|`640x512`，`fx=530.8443`、`fy=532.2955`、固定上视 `R_BC`|
|导引|`N=3`、`fixed_vm=10 m/s`、总加速度 `7 m/s2`|
|目标|静止 Actor；调整尺寸仅是仿真视觉代理，不代表真实靶机尺寸|
|框来源|AirSim `simGetDetections` 真值框；未运行 YOLO/RKNN|
|运行命令|`{summary['run_command']}`|
|一键复现|`./run_log00106_airsim_log_only.sh test`，再依次运行 `smoke` 和 `full`|
|配置 SHA256|`{summary['config_sha256']}`|
|Settings SHA256|`{summary['settings_sha256']}`|
|原始输出|`{run_dir.relative_to(PROJECT_ROOT)}`|

理想组使用 30 Hz 即时框和高频真值速度。两组敏感性使用主 CSV 的 40 组
`sample/available/result_age/fusion_wait` 配对时序，结果年龄已经包含融合等待；速度观测为 5 Hz、
`tau=0.25 s`，角速度命令延迟 15 ms。推力比 0.809 为主工况，0.847 为乐观边界，电压只保留
`22.65--22.95 V` 标签。

输入文件校验如下；`joint_*` 是接触锚定派生产物，其余为主 CSV/meta 和原始飞控日志。

|输入标识|SHA256|
|---|---|
{chr(10).join(input_hash_rows)}

## 3. 实测参考

|实测量|LOG00106 结果|
|---|---:|
|算法发布时长|1.670804 s|
|接触锚定初始剩余位移|约 5.06 m；气压高度敏感性约 6.34 m|
|算法退出时剩余位移|约 3.27--3.39 m|
|退出至物理接触|约 0.902 s|
|速度建立/总加速度饱和|65.48% / 72.62%|
|Rate 同钟滞后和相关|15 ms；Roll/Pitch 0.991/0.994|
|交接后实测/模型比力|P50 0.809；P95 边界 0.847|

以上距离不是两机绝对 GPS 差。碰撞后的 963 deg/s、电机 158/2047 和 82.06 A 已从 PNG 对比窗
排除。

## 4. 仿真结果

|工况|距离|退出|感知时序|比力比例|结果|AirSim 最近距离 (m)|接触时间 (s)|退出剩余距离 (m)|最大总加速度 (m/s2)|最大油门 (us)|
|---|---:|---|---|---:|---|---:|---:|---:|---:|---:|
{chr(10).join(rows)}

提前退出和持续闭环是两类独立结论，不合并计算命中率。`contact` 只接受 AirSim 返回且对象名匹配
目标 Actor 的碰撞；未接触组只报告最近点或安全超时。

## 5. 趋势验收

{chr(10).join(comparison_lines)}

每组 `metrics.json` 还给出上视 D 轴、7 m/s2 上限、1500 us 上限、速度建立主导和 Rate 相关性
检查。本轮全部硬检查为 **{'通过' if hard_checks_passed else '警告'}**；12 组平均控制周期范围为
`{min(control_periods):.3f}--{max(control_periods):.3f} ms`。敏感性组额外注入 15 ms 命令延迟后，
AirSim 飞控端到端最优滞后为 Roll `{min(roll_lags):.0f}--{max(roll_lags):.0f} ms`、Pitch
`{min(pitch_lags):.0f}--{max(pitch_lags):.0f} ms`，最低相关系数 `{min(rate_correlations):.3f}`；该值包含
SimpleFlight 动态，不能解释成只有 15 ms 的纯延迟。`t_contact_s` 对接触组以 Actor 接触为零，
未接触组以 AirSim 真值最近点为零。LOS rate 不要求单调归零；静止目标下仍受拦截机平移、姿态变化、
感知年龄和终端几何影响。

## 6. 曲线

{chr(10).join(figure_sections)}

## 7. 结论限制

本轮只能说明指定初始几何和单个 seed 下，理想/敏感性模型相对 LOG00106 的方向、时序、幅值及
饱和趋势。LOG00106 只有一个真实接触样本；无论 AirSim 某组是否接触，都不能宣称真实命中率达到
80%。概率命中率需要预定义场景分布、多个 seed 和独立真实飞行样本另行验证。
"""


def main() -> int:
    args = parse_args()
    output_root = Path(args.output_root).expanduser().resolve()
    run_dir = Path(args.run_dir).expanduser().resolve() if args.run_dir else _latest_run(output_root)
    summary = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
    if summary.get("smoke"):
        raise RuntimeError("a full 12-case run is required before report generation")
    cases: list[tuple[dict[str, Any], list[dict[str, str]]]] = []
    for item in summary["cases"]:
        case_dir = run_dir / item["case_id"]
        metrics = json.loads((case_dir / "metrics.json").read_text(encoding="utf-8"))
        rows = _read_csv(case_dir / "timeseries.csv")
        cases.append((metrics, rows))
    if len(cases) != 12:
        raise RuntimeError(f"expected 12 complete cases, got {len(cases)}")
    grouped: dict[tuple[str, str], list[tuple[dict[str, Any], list[dict[str, str]]]]] = defaultdict(list)
    for metrics, rows in cases:
        grouped[(str(metrics["distance_label"]), str(metrics["exit_mode"]))].append((metrics, rows))
    figure_dir = Path(args.figure_dir).expanduser().resolve()
    figures: dict[tuple[str, str], list[Path]] = {}
    plotters: tuple[tuple[str, Callable[..., None]], ...] = (
        ("01_rates", _plot_rates),
        ("02_throttle_force", _plot_throttle),
        ("03_velocity", _plot_velocity),
        ("04_acceleration", _plot_acceleration),
        ("05_los_bbox", _plot_los_bbox),
        ("06_state_range", _plot_state),
    )
    for (distance, exit_mode), selected in sorted(grouped.items()):
        stem = f"{distance.replace('.', 'p').replace('m', 'm')}_{exit_mode}"
        paths = []
        for suffix, plotter in plotters:
            path = figure_dir / f"{stem}_{suffix}.png"
            plotter(selected, path, f"LOG00106 AirSim {distance} / {exit_mode}")
            paths.append(path)
        figures[(distance, exit_mode)] = paths
    metrics = [item for item, _ in cases]
    comparisons = _trend_comparisons(metrics)
    comparison_output = {
        "run_id": summary["run_id"],
        "statistics_boundary": "simulation only; real range remains contact-anchored and separate",
        "trend_comparisons": comparisons,
    }
    (run_dir / "comparison_metrics.json").write_text(
        json.dumps(comparison_output, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    report_path = Path(args.report).expanduser().resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        _report_text(run_dir, summary, metrics, figures, comparisons, report_path),
        encoding="utf-8",
    )
    print(f"report={report_path}")
    print(f"figures={figure_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
