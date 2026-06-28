#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export an Ultralytics YOLO model to TensorRT engine.")
    parser.add_argument("--model", required=True, help="Input .pt model path.")
    parser.add_argument("--imgsz", type=int, default=640, help="Export image size.")
    parser.add_argument("--device", default="0", help="Ultralytics device string, for example 0 or cpu.")
    parser.add_argument("--half", action=argparse.BooleanOptionalAction, default=True, help="Export FP16 engine when supported.")
    parser.add_argument("--int8", action=argparse.BooleanOptionalAction, default=False, help="Export INT8 engine. Requires Ultralytics calibration support.")
    parser.add_argument("--workspace", type=float, default=None, help="Optional TensorRT workspace size in GiB.")
    parser.add_argument("--dynamic", action=argparse.BooleanOptionalAction, default=False, help="Export dynamic-shape engine.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model_path = Path(args.model).expanduser()
    if not model_path.exists():
        raise SystemExit(f"model not found: {model_path}")

    from ultralytics import YOLO

    model = YOLO(str(model_path))
    kwargs = {
        "format": "engine",
        "imgsz": int(args.imgsz),
        "device": str(args.device),
        "half": bool(args.half),
        "int8": bool(args.int8),
        "dynamic": bool(args.dynamic),
    }
    if args.workspace is not None:
        kwargs["workspace"] = float(args.workspace)
    output = model.export(**kwargs)
    print(f"tensorrt_engine={output}")


if __name__ == "__main__":
    main()
