from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable, Sequence


GRAVITY_MPS2 = 9.80665


def _number(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _values(rows: Sequence[dict[str, str]], field: str, *, positive: bool = False) -> list[float]:
    values = []
    for row in rows:
        value = _number(row.get(field))
        if value is not None and (not positive or value > 0.0):
            values.append(value)
    return values


def _mean(values: Sequence[float]) -> float | None:
    return statistics.fmean(values) if values else None


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    position = (len(ordered) - 1) * min(1.0, max(0.0, percentile))
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _truth_los_errors_deg(rows: Sequence[dict[str, str]]) -> list[float]:
    errors = []
    for row in rows:
        interceptor = [_number(row.get(f"interceptor_{axis}")) for axis in "xyz"]
        intruder = [_number(row.get(f"intruder_{axis}")) for axis in "xyz"]
        estimate = [_number(row.get(f"lambda_{axis}")) for axis in "xyz"]
        if any(value is None for value in interceptor + intruder + estimate):
            continue
        relative = [float(intruder[i]) - float(interceptor[i]) for i in range(3)]
        relative_norm = math.sqrt(sum(value * value for value in relative))
        estimate_norm = math.sqrt(sum(float(value) ** 2 for value in estimate))
        if relative_norm <= 1.0e-9 or estimate_norm <= 1.0e-9:
            continue
        dot = sum(relative[i] * float(estimate[i]) for i in range(3)) / (relative_norm * estimate_norm)
        errors.append(math.degrees(math.acos(max(-1.0, min(1.0, dot)))))
    return errors


def _contains_failure(log_text: str, keywords: Sequence[str]) -> bool:
    lowered = log_text.lower()
    return any(keyword.lower() in lowered for keyword in keywords)


def read_case_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def evaluate_case(
    csv_path: Path,
    *,
    case_info: dict[str, Any],
    thresholds: dict[str, Any],
    return_code: int,
    timed_out: bool,
    log_path: Path,
    simulator_alive: bool,
) -> dict[str, Any]:
    rows = read_case_csv(csv_path) if csv_path.exists() else []
    log_text = log_path.read_text(encoding="utf-8", errors="replace") if log_path.exists() else ""
    ranges = _values(rows, "range")
    detector_fps = _values(rows, "detector_fps", positive=True)
    loop_fps = _values(rows, "wall_fps", positive=True)
    clock_ratios = _values(rows, "sim_clock_ratio", positive=True)
    requested_load = _values(rows, "n_cmd_g")
    actual_load = _values(rows, "load_factor_fd_g") or _values(rows, "load_factor_g")
    deadline_misses = sum(str(row.get("deadline_miss", "0")) == "1" for row in rows)
    deadline_miss_ratio = deadline_misses / len(rows) if rows else 1.0
    detected_frames = sum(str(row.get("detected", "0")) == "1" for row in rows)
    valid_frames = sum(str(row.get("valid", "0")) == "1" for row in rows)
    hit_rows = [row for row in rows if str(row.get("collision_accepted", row.get("hit", "0"))) == "1"]
    los_errors = _truth_los_errors_deg(rows)

    rpc_failure = _contains_failure(
        log_text,
        ("failed to connect to airsim rpc", "rpc error", "msgpackrpc", "connection reset", "connection refused"),
    )
    mavlink_failure = _contains_failure(
        log_text,
        ("mavlink connection", "heartbeat timeout", "offboard rejected", "mavlink send failed"),
    )
    reasons: list[str] = []
    if timed_out:
        reasons.append("case_timeout")
    if return_code != 0:
        reasons.append(f"runner_return_code_{return_code}")
    if not simulator_alive:
        reasons.append("simulator_exited")
    if not csv_path.exists():
        reasons.append("csv_missing")
    if len(rows) < int(thresholds.get("min_frames", 1)):
        reasons.append("too_few_frames")
    detector_mean = _mean(detector_fps)
    loop_mean = _mean(loop_fps)
    clock_mean = _mean(clock_ratios)
    if detector_mean is None or detector_mean < float(thresholds.get("min_detector_fps", 0.0)):
        reasons.append("detector_fps_below_gate")
    if loop_mean is None or loop_mean < float(thresholds.get("min_loop_fps", 0.0)):
        reasons.append("loop_fps_below_gate")
    if clock_mean is None or clock_mean < float(thresholds.get("min_sim_clock_ratio", 0.0)):
        reasons.append("sim_clock_ratio_below_gate")
    if deadline_miss_ratio > float(thresholds.get("max_deadline_miss_ratio", 1.0)):
        reasons.append("deadline_miss_ratio_above_gate")
    if rpc_failure:
        reasons.append("rpc_failure")
    if mavlink_failure:
        reasons.append("mavlink_failure")
    cuda_values = {str(row.get("yolo_cuda_available", "")) for row in rows}
    half_values = {str(row.get("yolo_half_effective", "")) for row in rows}
    if rows and "1" not in cuda_values:
        reasons.append("cuda_not_active")
    if rows and "1" not in half_values:
        reasons.append("fp16_not_active")

    result = dict(case_info)
    result.update(
        {
            "status": "infra_invalid" if reasons else "completed",
            "infrastructure_valid": not reasons,
            "infra_invalid_reasons": reasons,
            "collision_hit": bool(hit_rows),
            "hit_time_s": _number(hit_rows[0].get("t")) if hit_rows else None,
            "near_hit": any(str(row.get("near_hit", "0")) == "1" for row in rows),
            "min_distance_m": min(ranges) if ranges else None,
            "final_distance_m": ranges[-1] if ranges else None,
            "frames": len(rows),
            "detection_rate": detected_frames / len(rows) if rows else None,
            "guidance_valid_rate": valid_frames / len(rows) if rows else None,
            "detector_fps_mean": detector_mean,
            "loop_fps_mean": loop_mean,
            "sim_clock_ratio_mean": clock_mean,
            "deadline_miss_ratio": deadline_miss_ratio,
            "truth_los_error_p95_deg": _percentile(los_errors, 0.95),
            "requested_load_p95_g": _percentile(requested_load, 0.95),
            "actual_load_max_g": max(actual_load) if actual_load else None,
            "rpc_failure_count": int(rpc_failure),
            "mavlink_failure_count": int(mavlink_failure),
            "return_code": int(return_code),
            "timed_out": bool(timed_out),
            "csv_path": str(csv_path),
            "meta_path": str(csv_path.with_name(f"{csv_path.stem}_meta.json")),
            "log_path": str(log_path),
        }
    )
    return result


def wilson_interval(hits: int, total: int, z: float = 1.959963984540054) -> tuple[float, float]:
    if total <= 0:
        return 0.0, 0.0
    proportion = hits / total
    denominator = 1.0 + z * z / total
    center = (proportion + z * z / (2.0 * total)) / denominator
    margin = z * math.sqrt(proportion * (1.0 - proportion) / total + z * z / (4.0 * total * total)) / denominator
    return max(0.0, center - margin), min(1.0, center + margin)


def aggregate_results(results: Sequence[dict[str, Any]]) -> dict[str, Any]:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for result in results:
        if result.get("infrastructure_valid"):
            groups[(str(result.get("tier")), str(result.get("guidance_law")))].append(result)

    grouped = []
    for (tier, law), items in sorted(groups.items()):
        hits = sum(bool(item.get("collision_hit")) for item in items)
        low, high = wilson_interval(hits, len(items))
        grouped.append(
            {
                "tier": tier,
                "guidance_law": law,
                "valid_cases": len(items),
                "hits": hits,
                "hit_rate": hits / len(items) if items else None,
                "hit_rate_ci95_low": low,
                "hit_rate_ci95_high": high,
                "hit_time_mean_s": _mean([float(item["hit_time_s"]) for item in items if item.get("hit_time_s") is not None]),
                "min_distance_mean_m": _mean([float(item["min_distance_m"]) for item in items if item.get("min_distance_m") is not None]),
                "detector_fps_mean": _mean([float(item["detector_fps_mean"]) for item in items if item.get("detector_fps_mean") is not None]),
                "loop_fps_mean": _mean([float(item["loop_fps_mean"]) for item in items if item.get("loop_fps_mean") is not None]),
                "sim_clock_ratio_mean": _mean([float(item["sim_clock_ratio_mean"]) for item in items if item.get("sim_clock_ratio_mean") is not None]),
                "deadline_miss_ratio_mean": _mean([float(item["deadline_miss_ratio"]) for item in items if item.get("deadline_miss_ratio") is not None]),
                "detection_rate_mean": _mean([float(item["detection_rate"]) for item in items if item.get("detection_rate") is not None]),
                "guidance_valid_rate_mean": _mean([float(item["guidance_valid_rate"]) for item in items if item.get("guidance_valid_rate") is not None]),
                "truth_los_error_p95_mean_deg": _mean([float(item["truth_los_error_p95_deg"]) for item in items if item.get("truth_los_error_p95_deg") is not None]),
                "requested_load_p95_mean_g": _mean([float(item["requested_load_p95_g"]) for item in items if item.get("requested_load_p95_g") is not None]),
                "actual_load_max_mean_g": _mean([float(item["actual_load_max_g"]) for item in items if item.get("actual_load_max_g") is not None]),
            }
        )
    invalid = [result for result in results if not result.get("infrastructure_valid")]
    return {
        "total_cases": len(results),
        "algorithm_valid_cases": len(results) - len(invalid),
        "infrastructure_invalid_cases": len(invalid),
        "groups": grouped,
        "infra_invalid_reason_counts": _reason_counts(invalid),
    }


def _reason_counts(results: Sequence[dict[str, Any]]) -> dict[str, int]:
    counts: dict[str, int] = defaultdict(int)
    for result in results:
        for reason in result.get("infra_invalid_reasons", []):
            counts[str(reason)] += 1
    return dict(sorted(counts.items()))


def _fmt(value: Any, digits: int = 2) -> str:
    number = _number(value)
    return "-" if number is None else f"{number:.{digits}f}"


def _fmt_percent(value: Any, digits: int = 1) -> str:
    number = _number(value)
    return "-" if number is None else f"{100.0 * number:.{digits}f}%"


def _write_cases_csv(results: Sequence[dict[str, Any]], path: Path) -> None:
    fields = [
        "case_key", "tier", "scenario_id", "guidance_law", "repeat", "attempt", "range_m", "lateral_m",
        "height_m", "target_speed_mps", "maneuver", "status",
        "infrastructure_valid", "collision_hit", "hit_time_s", "near_hit", "min_distance_m",
        "final_distance_m", "frames", "detection_rate", "guidance_valid_rate", "detector_fps_mean",
        "loop_fps_mean", "sim_clock_ratio_mean", "deadline_miss_ratio", "truth_los_error_p95_deg",
        "requested_load_p95_g", "actual_load_max_g", "rpc_failure_count", "mavlink_failure_count",
        "infra_invalid_reasons", "csv_path", "meta_path", "log_path"
    ]
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for item in results:
            row = dict(item)
            row["infra_invalid_reasons"] = ";".join(item.get("infra_invalid_reasons", []))
            writer.writerow(row)


def _write_plots(summary: dict[str, Any], results: Sequence[dict[str, Any]], plot_dir: Path) -> list[Path]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return []
    plot_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    paths = []

    groups = summary.get("groups", [])
    if groups:
        labels = [f"{item['tier']}-{item['guidance_law']}" for item in groups]
        rates = [100.0 * float(item["hit_rate"]) for item in groups]
        lower = [100.0 * (float(item["hit_rate"]) - float(item["hit_rate_ci95_low"])) for item in groups]
        upper = [100.0 * (float(item["hit_rate_ci95_high"]) - float(item["hit_rate"])) for item in groups]
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.bar(labels, rates, color=["#287271", "#D17C42", "#4F6D9B", "#A44A3F"][: len(labels)], yerr=[lower, upper], capsize=5)
        ax.set_ylabel("Collision hit rate (%)")
        ax.set_ylim(0, 105)
        ax.grid(axis="y", alpha=0.25)
        fig.tight_layout()
        path = plot_dir / "hit_rate_ci95.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths.append(path)

    valid = [item for item in results if item.get("infrastructure_valid")]
    if valid:
        labels = [f"{item['tier']}-{item['guidance_law']}" for item in valid]
        distance = [float(item.get("min_distance_m") or math.nan) for item in valid]
        colors = ["#287271" if item.get("collision_hit") else "#A44A3F" for item in valid]
        fig, ax = plt.subplots(figsize=(max(10, len(valid) * 0.22), 5))
        ax.scatter(range(len(valid)), distance, c=colors, s=24)
        ax.set_ylabel("Minimum center distance (m)")
        ax.set_xlabel("Valid case index")
        ax.grid(axis="y", alpha=0.25)
        fig.tight_layout()
        path = plot_dir / "minimum_distance_cases.png"
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths.append(path)
    return paths


def write_aggregate(results: Sequence[dict[str, Any]], output_dir: Path, *, include_plots: bool = True) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    ordered = sorted(results, key=lambda item: str(item.get("case_key", "")))
    summary = aggregate_results(ordered)
    _write_cases_csv(ordered, output_dir / "cases.csv")
    with (output_dir / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
    plots = _write_plots(summary, ordered, output_dir / "plots") if include_plots else []
    _write_chinese_report(summary, ordered, output_dir / "Windows仿真批量测试报告.md", plots)
    return summary


def _write_chinese_report(summary: dict[str, Any], results: Sequence[dict[str, Any]], path: Path, plots: Sequence[Path]) -> None:
    lines = [
        "# Windows YOLO+ByteTrack 上视相机 PNG 批量仿真报告",
        "",
        "## 判据与数据处理",
        "",
        "闭环输入为 YOLO+ByteTrack 检测框，经 LOS 滤波后进入 TTC 或固定 V_m 比例导引。命中只由 AirSim collision 判定；near-hit 和最小中心距离仅作诊断。主测试未启用 AirSim detect 影子链路，真实 LOS 由无人机与目标真值位置离线计算，并与滤波 LOS 比较。",
        "",
        f"总任务数 `{summary['total_cases']}`，基础设施有效 `{summary['algorithm_valid_cases']}`，基础设施无效 `{summary['infrastructure_invalid_cases']}`。无效任务不计入命中率。",
        "",
        "场景参数来自 `config/windows_scenarios.json`。`M01-M15` 覆盖距离、侧向、高度差和目标速度组合；`S30-S50` 使用幅值 4 m、周期 8 s 的 S 机动。逐 case 参数及结果见 `cases.csv`。",
        "",
        "## 分层结果",
        "",
        "|动力学层|算法|有效组数|碰撞命中|命中率及95%CI|平均命中时间s|平均最小距离m|检测FPS|闭环FPS|时钟比|deadline miss|检测率|有效导引率|LOS误差P95 deg|需用过载P95 g|实际过载max g|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in summary.get("groups", []):
        lines.append(
            f"|{item['tier']}|{item['guidance_law']}|{item['valid_cases']}|{item['hits']}|"
            f"{100.0 * item['hit_rate']:.1f}% [{100.0 * item['hit_rate_ci95_low']:.1f}%, {100.0 * item['hit_rate_ci95_high']:.1f}%]|"
            f"{_fmt(item['hit_time_mean_s'])}|{_fmt(item['min_distance_mean_m'])}|{_fmt(item['detector_fps_mean'])}|{_fmt(item['loop_fps_mean'])}|"
            f"{_fmt(item['sim_clock_ratio_mean'])}|{_fmt_percent(item['deadline_miss_ratio_mean'])}|"
            f"{_fmt_percent(item['detection_rate_mean'])}|{_fmt_percent(item['guidance_valid_rate_mean'])}|"
            f"{_fmt(item['truth_los_error_p95_mean_deg'])}|{_fmt(item['requested_load_p95_mean_g'])}|"
            f"{_fmt(item['actual_load_max_mean_g'])}|"
        )
    for plot in plots:
        lines.extend(["", f"![{plot.stem}]({plot.relative_to(path.parent).as_posix()})"])
    lines.extend(["", "## 基础设施无效任务", ""])
    reason_counts = summary.get("infra_invalid_reason_counts", {})
    if reason_counts:
        lines.extend(["|原因|次数|", "|---|---:|"])
        lines.extend(f"|`{reason}`|{count}|" for reason, count in reason_counts.items())
    else:
        lines.append("无。")
    lines.extend(
        [
            "",
            "## 说明",
            "",
            "`fast` 为 Windows 原生 SimpleFlight 快速统计层；`sitl` 为 Windows AirSim 与专用 WSL1 PX4 v1.11.3 软件在环层。两层控制器和动力学不同，因此分别统计，不直接合并命中率。每个 case 的 CSV、meta、控制台日志和判定 JSON 位于 `cases/`。环境版本记录见 `environment.json`。",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _load_results(output_dir: Path) -> list[dict[str, Any]]:
    results = []
    for path in sorted((output_dir / "cases").glob("*/result.json")):
        with path.open("r", encoding="utf-8") as stream:
            results.append(json.load(stream))
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description="Aggregate Windows AirSim/PX4 case outputs.")
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    results = _load_results(args.output_dir)
    summary = write_aggregate(results, args.output_dir)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
