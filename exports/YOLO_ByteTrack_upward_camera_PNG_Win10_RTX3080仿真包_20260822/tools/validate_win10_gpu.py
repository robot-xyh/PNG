from __future__ import annotations

import argparse
import json
import re
import subprocess
import time
from pathlib import Path
from typing import Any


MIN_DRIVER = (560, 76)
MIN_COMPUTE_CAPABILITY = (8, 6)
MIN_VRAM_BYTES = 8 * 1024**3


def parse_version(value: str) -> tuple[int, ...]:
    numbers = tuple(int(item) for item in re.findall(r"\d+", value))
    if not numbers:
        raise ValueError(f"invalid version: {value!r}")
    return numbers


def version_at_least(actual: str, minimum: tuple[int, ...]) -> bool:
    parsed = parse_version(actual)
    width = max(len(parsed), len(minimum))
    return parsed + (0,) * (width - len(parsed)) >= minimum + (0,) * (width - len(minimum))


def evaluate_gpu_profile(
    *,
    name: str,
    driver: str,
    vram_bytes: int,
    compute_capability: tuple[int, int],
) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []
    if not version_at_least(driver, MIN_DRIVER):
        errors.append(f"NVIDIA driver {driver} is older than required 560.76")
    if vram_bytes < MIN_VRAM_BYTES:
        errors.append(f"GPU memory {vram_bytes / 1024**3:.1f} GiB is below the required 8 GiB")
    if compute_capability < MIN_COMPUTE_CAPABILITY:
        errors.append(
            f"compute capability {compute_capability[0]}.{compute_capability[1]} "
            "is below the RTX 3080 baseline 8.6"
        )
    if not re.search(r"RTX\s+3080", name, re.IGNORECASE):
        warnings.append(f"GPU name {name!r} does not match the RTX 3080 target profile")
    return errors, warnings


def _nvidia_smi() -> dict[str, Any]:
    completed = subprocess.run(
        [
            "nvidia-smi.exe",
            "--query-gpu=name,driver_version,memory.total",
            "--format=csv,noheader,nounits",
        ],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if completed.returncode != 0 or not completed.stdout.strip():
        raise RuntimeError(f"nvidia-smi failed: {completed.stderr.strip()}")
    fields = [item.strip() for item in completed.stdout.splitlines()[0].split(",")]
    if len(fields) != 3:
        raise RuntimeError(f"unexpected nvidia-smi output: {completed.stdout.strip()}")
    return {
        "name": fields[0],
        "driver": fields[1],
        "vram_mib": int(fields[2]),
    }


def validate_runtime(model_path: Path, *, benchmark_iterations: int = 10) -> dict[str, Any]:
    import numpy as np
    import torch
    from ultralytics import YOLO

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable in PyTorch")
    device = torch.device("cuda:0")
    properties = torch.cuda.get_device_properties(device)
    smi = _nvidia_smi()
    profile_errors, profile_warnings = evaluate_gpu_profile(
        name=properties.name,
        driver=str(smi["driver"]),
        vram_bytes=int(properties.total_memory),
        compute_capability=(int(properties.major), int(properties.minor)),
    )
    if profile_errors:
        raise RuntimeError("; ".join(profile_errors))

    left = torch.randn((1024, 1024), device=device, dtype=torch.float16)
    right = torch.randn((1024, 1024), device=device, dtype=torch.float16)
    product = left @ right
    torch.cuda.synchronize()
    if not bool(torch.isfinite(product).all().item()):
        raise RuntimeError("CUDA FP16 matrix validation produced non-finite values")

    if not model_path.is_file() or model_path.stat().st_size == 0:
        raise RuntimeError(f"YOLO model is missing: {model_path}")
    model = YOLO(str(model_path))
    image = np.zeros((640, 640, 3), dtype=np.uint8)
    for _ in range(3):
        model.predict(image, imgsz=640, device=0, half=True, verbose=False)
    torch.cuda.synchronize()
    started = time.perf_counter()
    for _ in range(max(1, benchmark_iterations)):
        model.predict(image, imgsz=640, device=0, half=True, verbose=False)
    torch.cuda.synchronize()
    elapsed = max(time.perf_counter() - started, 1e-9)

    return {
        "profile": "win10_rtx3080",
        "gpu": properties.name,
        "driver": str(smi["driver"]),
        "vram_mib": int(smi["vram_mib"]),
        "compute_capability": f"{properties.major}.{properties.minor}",
        "torch": str(torch.__version__),
        "torch_cuda": str(torch.version.cuda),
        "fp16": True,
        "synthetic_yolo_fps": benchmark_iterations / elapsed,
        "warnings": profile_warnings,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate the Win10 RTX 3080 CUDA and YOLO runtime.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--benchmark-iterations", type=int, default=10)
    args = parser.parse_args()
    result = validate_runtime(args.model.resolve(), benchmark_iterations=args.benchmark_iterations)
    payload = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)


if __name__ == "__main__":
    main()
