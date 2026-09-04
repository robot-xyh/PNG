#!/usr/bin/env python3
"""Generate control-response figures for the final LOG00106 flight.

The host and Blackbox curves are aligned at the first algorithm command seen by
each logger. This alignment is suitable for waveform and amplitude comparison;
latency is calculated only between Blackbox setpoint and gyro signals, which
share the same flight-controller clock.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib import font_manager


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HOST = ROOT / "logs/flight_active_supervised/FLIGHT_ACTIVE_05S_VIDEO_20260904_183721_20260904_183722.csv"
DEFAULT_BLACKBOX_BFL = ROOT / "logs/blackbox_import/LOG00106.BFL"
DEFAULT_BLACKBOX = Path("/tmp/log00106_decode_raw_20260904_0402/LOG00106.01.csv")
DEFAULT_EVENT = Path("/tmp/log00106_decode_raw_20260904_0402/LOG00106.01.event")
DEFAULT_TARGET_ULOG = ROOT / "logs/target-log/10_35_19.ulg"
DEFAULT_JOINT = ROOT / "logs/analysis/LOG00106_target_joint/joint_timeseries_50hz.csv"
DEFAULT_JOINT_METRICS = ROOT / "logs/analysis/LOG00106_target_joint/metrics.json"
DEFAULT_OUTPUT = ROOT / "doc/figures/log00106_control_response"
DEFAULT_METRICS_OUTPUT = ROOT / "doc/evidence/BETAFLIGHT_LOG00106_CONTROL_RESPONSE_metrics.json"


COLORS = {
    "expected": "#0072B2",
    "setpoint": "#D55E00",
    "actual": "#009E73",
    "secondary": "#CC79A7",
    "warning": "#B23A48",
    "neutral": "#555555",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-csv", type=Path, default=DEFAULT_HOST)
    parser.add_argument("--blackbox-bfl", type=Path, default=DEFAULT_BLACKBOX_BFL)
    parser.add_argument("--blackbox-csv", type=Path, default=DEFAULT_BLACKBOX)
    parser.add_argument("--blackbox-event", type=Path, default=DEFAULT_EVENT)
    parser.add_argument("--target-ulog", type=Path, default=DEFAULT_TARGET_ULOG)
    parser.add_argument("--joint-csv", type=Path, default=DEFAULT_JOINT)
    parser.add_argument("--joint-metrics", type=Path, default=DEFAULT_JOINT_METRICS)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--metrics-output", type=Path, default=DEFAULT_METRICS_OUTPUT)
    parser.add_argument(
        "--blackbox-search-start-s",
        type=float,
        default=17.8,
        help="Blackbox time after Sync beep where the algorithm onset search begins",
    )
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def display_path(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


def numeric(frame: pd.DataFrame, column: str) -> np.ndarray:
    return pd.to_numeric(frame[column], errors="coerce").to_numpy(dtype=float)


def finite_summary(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return {
        "min": float(np.min(values)),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def configure_matplotlib() -> None:
    cjk_font_path = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if cjk_font_path.is_file():
        font_manager.fontManager.addfont(cjk_font_path)
        cjk_family = font_manager.FontProperties(fname=cjk_font_path).get_name()
    else:
        cjk_family = "Droid Sans Fallback"
    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": [cjk_family, "DejaVu Sans"],
            "axes.unicode_minus": False,
            "axes.grid": True,
            "grid.alpha": 0.25,
            "grid.linewidth": 0.7,
            "legend.frameon": False,
            "figure.dpi": 130,
            "savefig.dpi": 180,
        }
    )


def read_sync_beep_us(path: Path) -> int:
    match = re.search(r'"name":"Sync beep",\s*"time":(\d+)', path.read_text())
    if not match:
        raise ValueError(f"Sync beep not found in {path}")
    return int(match.group(1))


def detect_blackbox_algorithm_onset(bb: pd.DataFrame, bb_time_s: np.ndarray, search_start_s: float) -> float:
    roll = numeric(bb, " rcCommand[0]")
    pitch = numeric(bb, " rcCommand[1]")
    candidates = np.flatnonzero(
        (bb_time_s >= search_start_s)
        & (bb_time_s <= search_start_s + 1.0)
        & ((np.abs(roll) >= 2.0) | (np.abs(pitch) >= 2.0))
    )
    if len(candidates) == 0:
        raise ValueError("Could not locate the first algorithm command in Blackbox")
    return float(bb_time_s[candidates[0]])


def add_active_span(ax: plt.Axes, duration_s: float) -> None:
    ax.axvspan(0.0, duration_s, color="#DCEFD8", alpha=0.45, zorder=0)
    ax.axvline(0.0, color=COLORS["neutral"], linewidth=1.0, linestyle="--")
    ax.axvline(duration_s, color=COLORS["warning"], linewidth=1.0, linestyle="--")


def plot_rate_tracking(
    host_active: pd.DataFrame,
    host_time_s: np.ndarray,
    bb_window: pd.DataFrame,
    bb_time_s: np.ndarray,
    duration_s: float,
    output: Path,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True, constrained_layout=True)
    axis_specs = [
        ("Roll", "sp_roll_rate_deg_s", " setpoint[0]", " gyroADC[0]"),
        ("Pitch", "sp_pitch_rate_deg_s", " setpoint[1]", " gyroADC[1]"),
        ("Yaw", "sp_yaw_rate_deg_s", " setpoint[2]", " gyroADC[2]"),
    ]
    for ax, (name, host_col, setpoint_col, gyro_col) in zip(axes, axis_specs):
        add_active_span(ax, duration_s)
        ax.plot(
            host_time_s,
            numeric(host_active, host_col),
            color=COLORS["expected"],
            linewidth=1.9,
            label="PNG期望角速度",
        )
        ax.plot(
            bb_time_s,
            numeric(bb_window, setpoint_col),
            color=COLORS["setpoint"],
            linewidth=1.25,
            label="Betaflight setpoint",
        )
        ax.plot(
            bb_time_s,
            numeric(bb_window, gyro_col),
            color=COLORS["actual"],
            linewidth=1.0,
            alpha=0.9,
            label="实际gyro",
        )
        ax.set_ylabel(f"{name}\n(deg/s)")
        ax.set_xlim(-0.05, duration_s + 0.08)
        ax.legend(loc="upper left", ncols=3)
    axes[0].set_title("LOG00106：期望角速度、飞控设定值与实际角速度")
    axes[-1].set_xlabel("相对PNG首次算法指令时间 (s)")
    fig.text(
        0.995,
        0.005,
        "绿色区：PNG发布窗口；红色虚线：aux_disabled。两设备按各自算法起点归一化，仅比较波形与幅值。",
        ha="right",
        va="bottom",
        fontsize=9,
        color=COLORS["neutral"],
    )
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def plot_throttle_response(
    host_active: pd.DataFrame,
    host_time_s: np.ndarray,
    bb_window: pd.DataFrame,
    bb_time_s: np.ndarray,
    duration_s: float,
    output: Path,
) -> None:
    acc_norm_g = np.sqrt(
        sum((numeric(bb_window, f" accSmooth[{axis}]") / 2048.0) ** 2 for axis in range(3))
    )
    fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True, constrained_layout=True)

    add_active_span(axes[0], duration_s)
    axes[0].plot(
        host_time_s,
        numeric(host_active, "command_thrust_load_factor_raw_g"),
        color=COLORS["expected"],
        linewidth=2.0,
        label="推力模型期望载荷",
    )
    axes[0].plot(
        bb_time_s,
        acc_norm_g,
        color=COLORS["actual"],
        linewidth=1.1,
        label="Blackbox实测比力模长",
    )
    axes[0].axhline(1.0, color=COLORS["neutral"], linestyle=":", linewidth=1.0, label="1 g")
    axes[0].set_ylabel("载荷/比力 (g)")
    axes[0].legend(loc="upper left", ncols=3)

    add_active_span(axes[1], duration_s)
    axes[1].plot(
        host_time_s,
        numeric(host_active, "throttle_handover_requested_target_us"),
        color=COLORS["expected"],
        linewidth=1.8,
        label="模型目标油门",
    )
    axes[1].plot(
        host_time_s,
        numeric(host_active, "throttle_handover_output_us"),
        color=COLORS["setpoint"],
        linewidth=1.8,
        label="实际发送油门",
    )
    source = numeric(host_active, "throttle_handover_source_us")
    if np.isfinite(source).any():
        axes[1].axhline(
            float(np.nanmedian(source)),
            color=COLORS["neutral"],
            linestyle=":",
            linewidth=1.0,
            label="切入前油门",
        )
    axes[1].set_ylabel("RC油门 (us)")
    axes[1].legend(loc="upper left", ncols=3)

    add_active_span(axes[2], duration_s)
    axes[2].plot(
        bb_time_s,
        numeric(bb_window, " rcCommand[3]"),
        color=COLORS["secondary"],
        linewidth=1.5,
        label="飞控内部throttle",
    )
    for axis, color in zip(range(4), ["#0072B2", "#D55E00", "#009E73", "#CC79A7"]):
        axes[2].plot(
            bb_time_s,
            numeric(bb_window, f" motor[{axis}]"),
            color=color,
            linewidth=0.9,
            alpha=0.85,
            label=f"电机{axis + 1} raw",
        )
    axes[2].set_ylabel("飞控内部值")
    axes[2].legend(loc="upper left", ncols=5)

    add_active_span(axes[3], duration_s)
    axes[3].plot(
        bb_time_s,
        numeric(bb_window, " amperageLatest (A)"),
        color=COLORS["warning"],
        linewidth=1.3,
        label="电流",
    )
    axes[3].set_ylabel("电流 (A)")
    voltage_axis = axes[3].twinx()
    voltage_axis.plot(
        bb_time_s,
        numeric(bb_window, " vbatLatest (V)"),
        color=COLORS["neutral"],
        linewidth=1.2,
        label="电压",
    )
    voltage_axis.set_ylabel("电压 (V)")
    handles_a, labels_a = axes[3].get_legend_handles_labels()
    handles_b, labels_b = voltage_axis.get_legend_handles_labels()
    axes[3].legend(handles_a + handles_b, labels_a + labels_b, loc="upper left", ncols=2)
    axes[3].set_xlim(-0.05, duration_s + 0.08)
    axes[3].set_xlabel("相对PNG首次算法指令时间 (s)")
    axes[0].set_title("LOG00106：油门交接、实际比力与动力系统响应")
    fig.text(
        0.995,
        0.005,
        "实测比力为机体加速度计模长；可用于本段响应比较，但不等同于推力台静态推力。",
        ha="right",
        va="bottom",
        fontsize=9,
        color=COLORS["neutral"],
    )
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def add_event_lines(ax: plt.Axes, duration_s: float, passthrough_s: float, impact_s: float) -> None:
    ax.axvline(0.0, color=COLORS["expected"], linewidth=1.0, linestyle="--")
    ax.axvline(duration_s, color=COLORS["warning"], linewidth=1.0, linestyle="--")
    ax.axvline(passthrough_s, color=COLORS["secondary"], linewidth=1.0, linestyle=":")
    ax.axvline(impact_s, color="#000000", linewidth=1.25, linestyle="-")


def plot_guidance_and_closure(
    host: pd.DataFrame,
    algorithm_start_s: float,
    duration_s: float,
    joint: pd.DataFrame,
    joint_metrics: dict,
    output: Path,
) -> None:
    impact_from_start_s = -float(joint_metrics["events"]["algorithm_start"]["relative_to_impact_s"])
    passthrough_from_start_s = (
        float(joint_metrics["events"]["passthrough_after_aux"]["relative_to_impact_s"])
        + impact_from_start_s
    )
    joint_time_s = numeric(joint, "relative_to_impact_s") + impact_from_start_s
    host_time_s = numeric(host, "elapsed_s") - algorithm_start_s
    total_accel = np.sqrt(
        sum(numeric(host, f"intercept_total_accel_{axis}") ** 2 for axis in ("n", "e", "d"))
    )

    fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True, constrained_layout=True)
    for ax in axes:
        ax.axvspan(0.0, duration_s, color="#DCEFD8", alpha=0.45, zorder=0)
        add_event_lines(ax, duration_s, passthrough_from_start_s, impact_from_start_s)

    axes[0].plot(
        joint_time_s,
        numeric(joint, "contact_anchored_relative_norm_m"),
        color=COLORS["expected"],
        linewidth=2.0,
        label="接触锚定剩余位移",
    )
    axes[0].set_ylabel("剩余位移 (m)")
    axes[0].legend(loc="upper right")

    axes[1].plot(
        joint_time_s,
        -numeric(joint, "interceptor_velocity_raw_d_m_s"),
        color=COLORS["setpoint"],
        linewidth=1.25,
        label="拦截机向上速度（GPS raw）",
    )
    axes[1].plot(
        joint_time_s,
        -numeric(joint, "interceptor_velocity_filtered_d_m_s"),
        color=COLORS["actual"],
        linewidth=1.5,
        label="拦截机向上速度（滤波）",
    )
    axes[1].plot(
        joint_time_s,
        -numeric(joint, "target_velocity_d_m_s"),
        color=COLORS["neutral"],
        linewidth=1.1,
        label="靶机向上速度",
    )
    axes[1].axhline(0.0, color="#777777", linewidth=0.8)
    axes[1].set_ylabel("向上速度 (m/s)")
    axes[1].legend(loc="upper left", ncols=3)

    axes[2].plot(host_time_s, total_accel, color=COLORS["warning"], linewidth=1.6, label="候选总加速度")
    axes[2].axhline(7.0, color=COLORS["neutral"], linestyle=":", linewidth=1.0, label="7 m/s²上限")
    axes[2].set_ylabel("导引加速度 (m/s²)")
    axes[2].legend(loc="upper left", ncols=2)

    axes[3].plot(
        host_time_s,
        numeric(host, "bbox_area_ratio"),
        color=COLORS["secondary"],
        linewidth=1.5,
        label="目标框面积比",
    )
    axes[3].set_ylabel("框面积/画面")
    axes[3].set_xlabel("相对PNG首次算法指令时间 (s)")
    axes[3].legend(loc="upper left")
    axes[3].set_xlim(-0.3, impact_from_start_s + 0.12)
    axes[0].set_title("LOG00106：导引建立、退出后惯性闭合与物理接触")

    legend_lines = [
        (COLORS["expected"], "PNG开始"),
        (COLORS["warning"], "aux_disabled"),
        (COLORS["secondary"], "passthrough"),
        ("#000000", "物理接触"),
    ]
    handles = [plt.Line2D([0], [0], color=color, linewidth=1.5) for color, _ in legend_lines]
    labels = [label for _, label in legend_lines]
    axes[0].legend(handles + axes[0].get_legend_handles_labels()[0], labels + axes[0].get_legend_handles_labels()[1], loc="upper right", ncols=2)
    fig.savefig(output, bbox_inches="tight")
    plt.close(fig)


def tracking_metrics(bb: pd.DataFrame, bb_time_s: np.ndarray, onset_s: float, duration_s: float) -> dict:
    end_s = onset_s + max(0.4, duration_s - 0.2)
    mask = (bb_time_s >= onset_s) & (bb_time_s <= end_s)
    source_time = bb_time_s[mask]
    sample_time = np.arange(source_time[0], source_time[-1], 0.002)
    result: dict[str, dict[str, float]] = {}
    for axis, name in ((0, "roll"), (1, "pitch")):
        setpoint = np.interp(sample_time, source_time, numeric(bb.loc[mask], f" setpoint[{axis}]"))
        gyro = np.interp(sample_time, source_time, numeric(bb.loc[mask], f" gyroADC[{axis}]"))
        best: tuple[float, float, np.ndarray, np.ndarray] | None = None
        for lag_s in np.arange(0.0, 0.101, 0.001):
            shifted = np.interp(sample_time + lag_s, sample_time, gyro, left=np.nan, right=np.nan)
            valid = np.isfinite(shifted)
            x = setpoint[valid]
            y = shifted[valid]
            correlation = float(np.corrcoef(x, y)[0, 1])
            if best is None or correlation > best[0]:
                best = (correlation, float(lag_s), x, y)
        assert best is not None
        correlation, lag_s, x, y = best
        gain, intercept = np.polyfit(x, y, 1)
        result[name] = {
            "lag_ms": lag_s * 1000.0,
            "correlation": correlation,
            "gain": float(gain),
            "intercept_deg_s": float(intercept),
            "absolute_error_p95_deg_s": float(np.percentile(np.abs(y - x), 95)),
        }
    return result


def main() -> None:
    args = parse_args()
    for path in (
        args.host_csv,
        args.blackbox_bfl,
        args.blackbox_csv,
        args.blackbox_event,
        args.target_ulog,
        args.joint_csv,
        args.joint_metrics,
    ):
        if not path.is_file():
            raise FileNotFoundError(path)

    configure_matplotlib()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.metrics_output.parent.mkdir(parents=True, exist_ok=True)

    host = pd.read_csv(args.host_csv, low_memory=False)
    bb = pd.read_csv(args.blackbox_csv, low_memory=False)
    joint = pd.read_csv(args.joint_csv, low_memory=False)
    joint_metrics = json.loads(args.joint_metrics.read_text())

    host_active = host.loc[host["msp_publish_mode"].eq("algorithm")].copy()
    if host_active.empty:
        raise ValueError("No msp_publish_mode=algorithm rows in host CSV")
    algorithm_start_s = float(host_active["elapsed_s"].iloc[0])
    algorithm_end_s = float(host_active["elapsed_s"].iloc[-1])
    duration_s = algorithm_end_s - algorithm_start_s
    host_time_s = numeric(host_active, "elapsed_s") - algorithm_start_s

    sync_beep_us = read_sync_beep_us(args.blackbox_event)
    bb_time_absolute_s = (numeric(bb, " time (us)") - sync_beep_us) / 1e6
    onset_s = detect_blackbox_algorithm_onset(bb, bb_time_absolute_s, args.blackbox_search_start_s)
    bb_mask = (bb_time_absolute_s >= onset_s - 0.05) & (bb_time_absolute_s <= onset_s + duration_s + 0.08)
    bb_window = bb.loc[bb_mask].copy()
    bb_time_s = bb_time_absolute_s[bb_mask] - onset_s
    bb_active_mask = (bb_time_absolute_s >= onset_s) & (bb_time_absolute_s <= onset_s + duration_s)
    bb_active = bb.loc[bb_active_mask].copy()

    rate_figure = args.output_dir / "01_angle_rate_tracking.png"
    throttle_figure = args.output_dir / "02_throttle_and_load_response.png"
    closure_figure = args.output_dir / "03_guidance_closure_timeline.png"
    plot_rate_tracking(host_active, host_time_s, bb_window, bb_time_s, duration_s, rate_figure)
    plot_throttle_response(host_active, host_time_s, bb_window, bb_time_s, duration_s, throttle_figure)
    plot_guidance_and_closure(host, algorithm_start_s, duration_s, joint, joint_metrics, closure_figure)

    acc_norm_g = np.sqrt(
        sum((numeric(bb_active, f" accSmooth[{axis}]") / 2048.0) ** 2 for axis in range(3))
    )
    aligned_acc_norm_g = np.interp(host_time_s, bb_time_absolute_s[bb_active_mask] - onset_s, acc_norm_g)
    expected_load_g = numeric(host_active, "command_thrust_load_factor_raw_g")
    load_ratio = aligned_acc_norm_g / expected_load_g
    post_handover = host_time_s >= 0.8
    motors = np.concatenate([numeric(bb_active, f" motor[{axis}]") for axis in range(4)])
    motor_matrix = np.column_stack([numeric(bb_active, f" motor[{axis}]") for axis in range(4)])
    evidence = {
        "scope": "Only the final FLIGHT_ACTIVE_05S_VIDEO_20260904_183721 / LOG00106 flight",
        "alignment": {
            "method": "Host and Blackbox curves normalized to their own first algorithm command",
            "blackbox_sync_beep_us": sync_beep_us,
            "blackbox_algorithm_onset_s": onset_s,
            "algorithm_publish_duration_s": duration_s,
            "latency_rule": "Only Blackbox setpoint-to-gyro lag is treated as a latency measurement",
        },
        "host_expected": {
            "roll_rate_deg_s": finite_summary(numeric(host_active, "sp_roll_rate_deg_s")),
            "pitch_rate_deg_s": finite_summary(numeric(host_active, "sp_pitch_rate_deg_s")),
            "yaw_rate_deg_s": finite_summary(numeric(host_active, "sp_yaw_rate_deg_s")),
            "load_factor_g": finite_summary(numeric(host_active, "command_thrust_load_factor_raw_g")),
            "handover_source_us": finite_summary(numeric(host_active, "throttle_handover_source_us")),
            "target_throttle_us": finite_summary(numeric(host_active, "throttle_handover_requested_target_us")),
            "sent_throttle_us": finite_summary(numeric(host_active, "throttle_handover_output_us")),
        },
        "blackbox_actual": {
            "roll_setpoint_deg_s": finite_summary(numeric(bb_active, " setpoint[0]")),
            "pitch_setpoint_deg_s": finite_summary(numeric(bb_active, " setpoint[1]")),
            "yaw_setpoint_deg_s": finite_summary(numeric(bb_active, " setpoint[2]")),
            "roll_gyro_deg_s": finite_summary(numeric(bb_active, " gyroADC[0]")),
            "pitch_gyro_deg_s": finite_summary(numeric(bb_active, " gyroADC[1]")),
            "yaw_gyro_deg_s": finite_summary(numeric(bb_active, " gyroADC[2]")),
            "specific_force_norm_g": finite_summary(acc_norm_g),
            "internal_throttle": finite_summary(numeric(bb_active, " rcCommand[3]")),
            "motor_raw": finite_summary(motors),
            "motor_spread_raw": finite_summary(np.max(motor_matrix, axis=1) - np.min(motor_matrix, axis=1)),
            "battery_voltage_v": finite_summary(numeric(bb_active, " vbatLatest (V)")),
            "battery_current_a": finite_summary(numeric(bb_active, " amperageLatest (A)")),
            "specific_force_to_model_ratio": finite_summary(load_ratio),
            "specific_force_to_model_ratio_after_0p8s_handover": finite_summary(load_ratio[post_handover]),
        },
        "same_clock_tracking": tracking_metrics(bb, bb_time_absolute_s, onset_s, duration_s),
        "interpretation": {
            "rate_chain": "Direction, mapping, and closed-loop rate tracking are consistent",
            "throttle_chain": "Throttle handover and FC reception are consistent; no 1500 us limit violation",
            "thrust_model": "Measured specific force is about 15-20 percent below model demand in this transient",
            "collision_samples_excluded": True,
        },
        "inputs": {
            display_path(args.host_csv): sha256(args.host_csv),
            display_path(args.blackbox_bfl): sha256(args.blackbox_bfl),
            display_path(args.target_ulog): sha256(args.target_ulog),
            display_path(args.joint_csv): sha256(args.joint_csv),
            display_path(args.joint_metrics): sha256(args.joint_metrics),
        },
        "figures": [display_path(rate_figure), display_path(throttle_figure), display_path(closure_figure)],
    }
    args.metrics_output.write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"figures": evidence["figures"], "metrics": str(args.metrics_output)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
