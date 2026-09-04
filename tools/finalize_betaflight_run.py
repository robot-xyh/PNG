#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision_guidance.runtime_evidence import (  # noqa: E402
    file_metadata,
    validate_blackbox_mode_binding,
    verify_evidence_frame_index,
)


FORBIDDEN_MISS_DISTANCE_METHOD = "independent_absolute_gnss"
ALLOWED_MISS_DISTANCE_METHODS = {
    "not_evaluated",
    "contact_anchor",
    "shared_local_frame",
    "external_truth",
    FORBIDDEN_MISS_DISTANCE_METHOD,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Finalize one Betaflight run manifest with immutable external evidence."
    )
    parser.add_argument("--runtime-manifest", required=True)
    parser.add_argument(
        "--artifact",
        action="append",
        default=[],
        metavar="ROLE=PATH",
        help="Add console, blackbox, target_log, video, or another named artifact.",
    )
    parser.add_argument(
        "--pairing-confidence",
        choices=("unique", "time_only", "unpaired"),
        required=True,
    )
    parser.add_argument("--clock-uncertainty-ms", type=float, required=True)
    parser.add_argument(
        "--blackbox-mode-binding",
        default="",
        help="Firmware-specific Blackbox decoder/mode contract; required with a blackbox artifact.",
    )
    parser.add_argument(
        "--miss-distance-method",
        choices=tuple(sorted(ALLOWED_MISS_DISTANCE_METHODS)),
        default="not_evaluated",
    )
    parser.add_argument("--output", default="")
    parser.add_argument("--acknowledge-incomplete-run", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    runtime_path = Path(args.runtime_manifest).expanduser().resolve()
    output_path = (
        Path(args.output).expanduser().resolve()
        if args.output
        else runtime_path.with_name(runtime_path.name.replace("_runtime_manifest", "_manifest"))
    )
    result = finalize(
        runtime_path,
        artifacts=_parse_artifacts(args.artifact),
        pairing_confidence=args.pairing_confidence,
        clock_uncertainty_ms=float(args.clock_uncertainty_ms),
        miss_distance_method=args.miss_distance_method,
        acknowledge_incomplete_run=bool(args.acknowledge_incomplete_run),
        blackbox_mode_binding_path=(
            Path(args.blackbox_mode_binding).expanduser().resolve()
            if args.blackbox_mode_binding
            else None
        ),
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(f".{output_path.name}.tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, output_path)
    _fsync_directory(output_path.parent)
    print(output_path)


def finalize(
    runtime_manifest_path: Path,
    *,
    artifacts: dict[str, Path],
    pairing_confidence: str,
    clock_uncertainty_ms: float,
    miss_distance_method: str,
    acknowledge_incomplete_run: bool,
    blackbox_mode_binding_path: Path | None = None,
) -> dict[str, Any]:
    if not math.isfinite(clock_uncertainty_ms) or clock_uncertainty_ms < 0.0:
        raise ValueError("clock_uncertainty_ms must be finite and non-negative")
    if miss_distance_method not in ALLOWED_MISS_DISTANCE_METHODS:
        raise ValueError("unsupported miss-distance method")
    if miss_distance_method == FORBIDDEN_MISS_DISTANCE_METHOD:
        raise RuntimeError(
            "independent absolute GNSS positions cannot be used as interception miss distance"
        )
    runtime_manifest_path = runtime_manifest_path.expanduser().resolve()
    runtime = json.loads(runtime_manifest_path.read_text(encoding="utf-8"))
    if runtime.get("schema_version") != 1:
        raise RuntimeError("unsupported runtime manifest schema")
    completion = runtime.get("completion", {})
    if not isinstance(completion, dict):
        raise RuntimeError("runtime manifest completion is invalid")
    if completion.get("complete") is not True and not acknowledge_incomplete_run:
        raise RuntimeError("incomplete run requires --acknowledge-incomplete-run")
    runtime_artifacts = runtime.get("artifacts", [])
    for recorded in runtime_artifacts:
        if not isinstance(recorded, dict):
            raise RuntimeError("runtime artifact entry is invalid")
        path = Path(str(recorded.get("path", "")))
        if not path.is_file() or file_metadata(path)["sha256"] != recorded.get("sha256"):
            raise RuntimeError(f"runtime artifact changed or is missing: {path}")

    evidence_indexes = [
        Path(str(recorded.get("path", "")))
        for recorded in runtime_artifacts
        if isinstance(recorded, dict)
        and str(recorded.get("path", "")).endswith("_evidence_frames.jsonl")
    ]
    visual_evidence = (
        {"enabled": False, "frame_count": 0}
        if not evidence_indexes
        else {"enabled": True, **verify_evidence_frame_index(evidence_indexes[0])}
    )
    external = {
        role: {"role": role, **file_metadata(path)}
        for role, path in sorted(artifacts.items())
    }
    blackbox_interpretation = None
    if "blackbox" in artifacts:
        if blackbox_mode_binding_path is None:
            raise RuntimeError(
                "a blackbox artifact requires --blackbox-mode-binding"
            )
        blackbox_interpretation = validate_blackbox_mode_binding(
            blackbox_mode_binding_path,
            fc_identity=_runtime_fc_identity(runtime_artifacts),
        )
    elif blackbox_mode_binding_path is not None:
        raise RuntimeError("Blackbox mode binding was supplied without a blackbox artifact")
    return {
        **runtime,
        "schema_version": 2,
        "finalized": True,
        "finalized_unix_s": time.time(),
        "runtime_manifest": file_metadata(runtime_manifest_path),
        "external_artifacts": external,
        "visual_evidence": visual_evidence,
        "blackbox_interpretation": blackbox_interpretation,
        "pairing": {
            "confidence": pairing_confidence,
            "clock_uncertainty_ms": clock_uncertainty_ms,
            "hardware_latency_claim_allowed": clock_uncertainty_ms <= 5.0,
        },
        "miss_distance": {
            "method": miss_distance_method,
            "independent_absolute_gnss_allowed": False,
        },
    }


def _runtime_fc_identity(artifacts: list[Any]) -> dict[str, Any]:
    meta_paths = [
        Path(str(record.get("path", "")))
        for record in artifacts
        if isinstance(record, dict) and str(record.get("path", "")).endswith("_meta.json")
    ]
    if len(meta_paths) != 1:
        raise RuntimeError("runtime manifest must contain exactly one run meta artifact")
    value = json.loads(meta_paths[0].read_text(encoding="utf-8"))
    identity = value.get("fc_identity") if isinstance(value, dict) else None
    if not isinstance(identity, dict) or not identity:
        raise RuntimeError("run meta does not contain a firmware identity")
    return identity


def _parse_artifacts(values: list[str]) -> dict[str, Path]:
    result: dict[str, Path] = {}
    for value in values:
        role, separator, raw_path = value.partition("=")
        if not separator or not role.strip() or not raw_path.strip():
            raise ValueError(f"artifact must use ROLE=PATH: {value}")
        if role in result:
            raise ValueError(f"duplicate artifact role: {role}")
        result[role] = Path(raw_path).expanduser().resolve()
    return result


def _fsync_directory(path: Path) -> None:
    try:
        directory_fd = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


if __name__ == "__main__":
    main()
