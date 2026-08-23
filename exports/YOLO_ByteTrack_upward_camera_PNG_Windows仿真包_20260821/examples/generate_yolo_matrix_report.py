from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

LOG_DIR = PROJECT_ROOT / "logs" / "yolo_sitl_ttc_vm"
GRAVITY_MPS2 = 9.80665


@dataclass
class MatrixCase:
    case_id: str
    range_m: float
    lateral_m: float
    altitude_m: float
    intruder_speed_mps: float
    speed_ratio: float
    stamp: str
    status: str


@dataclass
class ResultRow:
    case: MatrixCase
    label: str
    hit: bool
    near_hit: bool
    hit_t_s: float
    min_range_m: float
    final_range_m: float
    frames: int
    detected_frames: int
    valid_frames: int
    avg_detector_fps: float
    avg_wall_fps: float
    los_p95_deg: float
    required_p95_g: float
    max_load_fd_g: float
    yolo_raw_frames: int
    yolo_selected_frames: int
    shadow_enabled_frames: int
    detector_sources: str
    reject_top: str
    csv_path: Path | None


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
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _read_json(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def _percentile(values: list[float], q: float) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
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


def _unit(vector: np.ndarray) -> np.ndarray | None:
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= 1.0e-9:
        return None
    return vector / norm


def _angle_deg(left: np.ndarray | None, right: np.ndarray | None) -> float:
    if left is None or right is None:
        return math.nan
    if not np.all(np.isfinite(left)) or not np.all(np.isfinite(right)):
        return math.nan
    dot = float(np.clip(np.dot(left, right), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def _np_vector(row: dict[str, str], keys: tuple[str, str, str]) -> np.ndarray | None:
    values = np.array([_float(row.get(key)) for key in keys], dtype=float)
    if not np.all(np.isfinite(values)):
        return None
    return values


def _los_error_p95(samples: list[dict[str, str]]) -> float:
    errors: list[float] = []
    for row in samples:
        camera = _np_vector(row, ("camera_world_x", "camera_world_y", "camera_world_z"))
        intruder = _np_vector(row, ("intruder_x", "intruder_y", "intruder_z"))
        visual = _np_vector(row, ("lambda_x", "lambda_y", "lambda_z"))
        truth = None if camera is None or intruder is None else _unit(intruder - camera)
        errors.append(_angle_deg(_unit(visual) if visual is not None else None, truth))
    return _percentile(errors, 0.95)


def _guidance_load_p95(samples: list[dict[str, str]]) -> float:
    values: list[float] = []
    for row in samples:
        n_cmd = _float(row.get("n_cmd_g"))
        if math.isfinite(n_cmd):
            values.append(n_cmd)
            continue
        vec = _np_vector(row, ("g_eval_x", "g_eval_y", "g_eval_z"))
        values.append(float(np.linalg.norm(vec)) / GRAVITY_MPS2 if vec is not None else math.nan)
    return _percentile(values, 0.95)


def _load_manifest(path: Path) -> list[MatrixCase]:
    cases: list[MatrixCase] = []
    for row in _read_csv(path):
        cases.append(
            MatrixCase(
                case_id=str(row.get("case_id", "")).strip(),
                range_m=_float(row.get("start_horizontal_range_m")),
                lateral_m=_float(row.get("start_lateral_offset_m")),
                altitude_m=_float(row.get("altitude_offset_m")),
                intruder_speed_mps=_float(row.get("intruder_speed_mps")),
                speed_ratio=_float(row.get("speed_ratio")),
                stamp=str(row.get("stamp", "")).strip(),
                status=str(row.get("status", "") or "ok").strip(),
            )
        )
    return cases


def _summary_path(label: str, stamp: str) -> Path:
    return LOG_DIR / f"yolo_sitl_{label}_{stamp}_summary.csv"


def _result_for(case: MatrixCase, label: str) -> ResultRow:
    summary_rows = _read_csv(_summary_path(label, case.stamp))
    item = summary_rows[0] if summary_rows else {}
    csv_path = Path(item.get("csv_path", "")) if item.get("csv_path") else None
    if csv_path is not None and not csv_path.is_absolute():
        csv_path = PROJECT_ROOT / csv_path
    samples = _read_csv(csv_path) if csv_path is not None else []
    meta_path = csv_path.with_name(f"{csv_path.stem}_meta.json") if csv_path is not None else None
    meta = _read_json(meta_path) if meta_path is not None else {}
    derived = meta.get("derived", {}) if isinstance(meta, dict) else {}
    detector_sources = sorted({str(row.get("detector_source", "")) for row in samples if row.get("detector_source", "")})
    reject_counts: dict[str, int] = {}
    for row in samples:
        reason = str(row.get("reject_reason", "") or "valid")
        reject_counts[reason] = reject_counts.get(reason, 0) + 1
    common_rejects = ", ".join(
        f"{reason}:{count}"
        for reason, count in sorted(reject_counts.items(), key=lambda entry: entry[1], reverse=True)[:3]
    )
    return ResultRow(
        case=case,
        label=label,
        hit=_bool(item.get("hit")),
        near_hit=_bool(item.get("near_hit")),
        hit_t_s=_float(item.get("hit_t_s")),
        min_range_m=_float(item.get("min_range_m")),
        final_range_m=_float(item.get("final_range_m")),
        frames=_int(item.get("frames")),
        detected_frames=_int(item.get("detected_frames")),
        valid_frames=_int(item.get("valid_frames")),
        avg_detector_fps=_float(derived.get("avg_detector_fps")),
        avg_wall_fps=_float(item.get("avg_wall_fps")),
        los_p95_deg=_los_error_p95(samples),
        required_p95_g=_guidance_load_p95(samples),
        max_load_fd_g=_float(item.get("max_load_factor_fd_g")),
        yolo_raw_frames=sum(1 for row in samples if _float(row.get("yolo_raw_count"), 0.0) > 0.0),
        yolo_selected_frames=sum(1 for row in samples if str(row.get("yolo_selected_source", "")).strip()),
        shadow_enabled_frames=sum(1 for row in samples if _bool(row.get("shadow_airsim_enabled"))),
        detector_sources=", ".join(detector_sources) or "-",
        reject_top=common_rejects or "-",
        csv_path=csv_path,
    )


def _rel(path: Path, report_path: Path) -> str:
    return path.relative_to(report_path.parent).as_posix()


def _fmt(value: float, digits: int = 2) -> str:
    return "-" if not math.isfinite(value) else f"{value:.{digits}f}"


def _ratio(numerator: int, denominator: int) -> str:
    if denominator <= 0:
        return "-"
    return f"{100.0 * numerator / denominator:.1f}%"


def _summary_table(rows: list[ResultRow]) -> str:
    lines = [
        "|算法|collision命中|near-hit|完全未命中|最小距离范围m|平均检测率|平均YOLO FPS|",
        "|---|---:|---:|---:|---|---:|---:|",
    ]
    for label in ("TTC", "VM"):
        group = [row for row in rows if row.label == label and row.case.status == "ok"]
        hit = sum(1 for row in group if row.hit)
        near = sum(1 for row in group if row.near_hit)
        miss = sum(1 for row in group if not row.hit and not row.near_hit)
        min_values = [row.min_range_m for row in group if math.isfinite(row.min_range_m)]
        det_ratio = (
            100.0 * sum(row.detected_frames for row in group) / max(1, sum(row.frames for row in group))
            if group
            else math.nan
        )
        fps = _percentile([row.avg_detector_fps for row in group], 0.50)
        min_range = f"{min(min_values):.2f}-{max(min_values):.2f}" if min_values else "-"
        lines.append(f"|{label}|{hit}/{len(group)}|{near}/{len(group)}|{miss}|{min_range}|{_fmt(det_ratio, 1)}|{_fmt(fps, 2)}|")
    return "\n".join(lines)


def _detail_table(rows: list[ResultRow]) -> str:
    lines = [
        "|case|算法|距离|侧向|高度差|目标速度|碰撞|near|最小m|终点m|检测率|有效率|YOLO FPS|LOS P95 deg|需用P95 g|主要状态|",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in sorted(rows, key=lambda item: (item.case.case_id, item.label)):
        lines.append(
            "|"
            + "|".join(
                [
                    row.case.case_id,
                    row.label,
                    _fmt(row.case.range_m, 0),
                    _fmt(row.case.lateral_m, 0),
                    _fmt(row.case.altitude_m, 0),
                    _fmt(row.case.intruder_speed_mps, 1),
                    "1" if row.hit else "0",
                    "1" if row.near_hit else "0",
                    _fmt(row.min_range_m, 2),
                    _fmt(row.final_range_m, 2),
                    _ratio(row.detected_frames, row.frames),
                    _ratio(row.valid_frames, row.frames),
                    _fmt(row.avg_detector_fps, 2),
                    _fmt(row.los_p95_deg, 1),
                    _fmt(row.required_p95_g, 2),
                    row.reject_top,
                ]
            )
            + "|"
        )
    return "\n".join(lines)


def _manifest_table(cases: list[MatrixCase]) -> str:
    lines = [
        "|case|距离m|侧向m|高度差m|目标速度m/s|speed ratio|stamp|status|",
        "|---|---:|---:|---:|---:|---:|---|---|",
    ]
    for case in cases:
        lines.append(
            f"|{case.case_id}|{_fmt(case.range_m, 0)}|{_fmt(case.lateral_m, 0)}|{_fmt(case.altitude_m, 0)}|"
            f"{_fmt(case.intruder_speed_mps, 1)}|{_fmt(case.speed_ratio, 1)}|`{case.stamp}`|`{case.status}`|"
        )
    return "\n".join(lines)


def _validation_table(rows: list[ResultRow]) -> str:
    lines = [
        "|检查项|结果|",
        "|---|---|",
        f"|CSV 数量|`{sum(1 for row in rows if row.csv_path and row.csv_path.exists())}/30`|",
        f"|detector_source|`{', '.join(sorted({row.detector_sources for row in rows}))}`|",
        f"|shadow enabled frames|`{sum(row.shadow_enabled_frames for row in rows)}`|",
        f"|YOLO raw frames|`{sum(row.yolo_raw_frames for row in rows)}`|",
        f"|YOLO selected frames|`{sum(row.yolo_selected_frames for row in rows)}`|",
    ]
    return "\n".join(lines)


def _plot_matrix(rows: list[ResultRow], asset_dir: Path) -> tuple[Path, Path]:
    asset_dir.mkdir(parents=True, exist_ok=True)
    cases = sorted({row.case.case_id for row in rows})
    x = np.arange(len(cases))
    width = 0.36
    labels = ("TTC", "VM")

    fig, axes = plt.subplots(2, 2, figsize=(15, 9), sharex=True)
    ax_hit, ax_range, ax_det, ax_los = axes.flat
    for offset, label in [(-width / 2, labels[0]), (width / 2, labels[1])]:
        group = {row.case.case_id: row for row in rows if row.label == label}
        ordered = [group.get(case) for case in cases]
        ax_hit.bar(x + offset, [1 if row and row.hit else 0 for row in ordered], width, label=label)
        ax_range.plot(x, [row.min_range_m if row else math.nan for row in ordered], marker="o", label=label)
        ax_det.plot(x, [100.0 * row.detected_frames / max(1, row.frames) if row else math.nan for row in ordered], marker="o", label=label)
        ax_los.plot(x, [row.los_p95_deg if row else math.nan for row in ordered], marker="o", label=label)
    ax_hit.set_title("Collision hit")
    ax_hit.set_ylabel("hit")
    ax_range.set_title("Minimum range")
    ax_range.set_ylabel("m")
    ax_det.set_title("Detection frame ratio")
    ax_det.set_ylabel("%")
    ax_los.set_title("Visual LOS error P95")
    ax_los.set_ylabel("deg")
    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_xticks(x, cases, rotation=45)
    fig.tight_layout()
    summary_path = asset_dir / "matrix15_summary.png"
    fig.savefig(summary_path, dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(15, 9), sharex=True)
    ax_range_p, ax_lat, ax_alt, ax_speed = axes.flat
    case_map = {row.case.case_id: row.case for row in rows}
    ordered_cases = [case_map[case] for case in cases if case in case_map]
    ax_range_p.bar(np.arange(len(ordered_cases)), [case.range_m for case in ordered_cases])
    ax_lat.bar(np.arange(len(ordered_cases)), [case.lateral_m for case in ordered_cases])
    ax_alt.bar(np.arange(len(ordered_cases)), [case.altitude_m for case in ordered_cases])
    ax_speed.bar(np.arange(len(ordered_cases)), [case.intruder_speed_mps for case in ordered_cases])
    for ax, title, ylabel in [
        (ax_range_p, "Initial horizontal range", "m"),
        (ax_lat, "Lateral offset", "m"),
        (ax_alt, "Altitude offset", "m"),
        (ax_speed, "Intruder speed", "m/s"),
    ]:
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True, axis="y", alpha=0.3)
        ax.set_xticks(np.arange(len(ordered_cases)), [case.case_id for case in ordered_cases], rotation=45)
    fig.tight_layout()
    params_path = asset_dir / "matrix15_parameters.png"
    fig.savefig(params_path, dpi=170)
    plt.close(fig)
    return summary_path, params_path


def _export_docx(report_path: Path) -> None:
    module_path = PROJECT_ROOT / "examples" / "generate_yolo_sitl_ttc_vm_report.py"
    spec = importlib.util.spec_from_file_location("gen_yolo_report", module_path)
    if spec is None or spec.loader is None:
        return
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    module.REPORT_PATH = report_path.resolve()
    module._export_docx(report_path.with_suffix(".docx"))


def write_report(
    cases: list[MatrixCase],
    rows: list[ResultRow],
    report_path: Path,
    asset_dir: Path,
    title: str,
    *,
    export_docx: bool = True,
) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    summary_img, params_img = _plot_matrix(rows, asset_dir)
    infra_failed = [case.case_id for case in cases if case.status != "ok"]
    near_misses = [row for row in rows if not row.hit and row.near_hit]
    misses = [row for row in rows if not row.hit and not row.near_hit]
    report = f"""# {title}

## 1. 实验目的

本轮测试 YOLO+ByteTrack 固定上视相机闭环在 15 个距离、侧向、高度差和目标速度组合下的拦截性能。每个工况分别运行 TTC 与 VM，共 30 个 case；AirSim detect shadow 关闭，collision 作为成功标准。

## 2. 工况矩阵

{_manifest_table(cases)}

![matrix15_parameters]({_rel(params_img, report_path)})

## 3. 总体结果

{_summary_table(rows)}

![matrix15_summary]({_rel(summary_img, report_path)})

## 4. 明细结果

{_detail_table(rows)}

## 5. 验证

{_validation_table(rows)}

## 6. 诊断要点

- infrastructure failed case: `{', '.join(infra_failed) if infra_failed else '-'}`。
- near-hit but no collision: `{', '.join(f'{row.case.case_id}-{row.label}({row.min_range_m:.2f}m)' for row in near_misses) if near_misses else '-'}`。
- full miss: `{', '.join(f'{row.case.case_id}-{row.label}({row.min_range_m:.2f}m)' for row in misses) if misses else '-'}`。
- 本轮 `shadow_airsim_enabled` 总帧数应为 0；若非 0，说明实验不满足“无影子测试”条件。
- 低检测率或 LOS P95 高的失败工况优先分析 YOLO/ByteTrack 连续性、frame-centering、terminal image KF 和末端 bbox 裁切。
"""
    report_path.write_text(report, encoding="utf-8")
    if export_docx:
        _export_docx(report_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate YOLO upward matrix report.")
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--report-path", required=True)
    parser.add_argument("--asset-dir", required=True)
    parser.add_argument("--title", default="YOLO+ByteTrack upward-camera matrix15 performance report")
    parser.add_argument("--no-docx", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    manifest_path = Path(args.manifest).resolve()
    report_path = Path(args.report_path).resolve()
    asset_dir = Path(args.asset_dir).resolve()
    cases = _load_manifest(manifest_path)
    rows = [_result_for(case, label) for case in cases for label in ("TTC", "VM")]
    write_report(cases, rows, report_path, asset_dir, args.title, export_docx=not args.no_docx)
    print(f"manifest={manifest_path}")
    print(f"report={report_path}")
    print(f"rows={len(rows)}")


if __name__ == "__main__":
    main()
