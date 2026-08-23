from __future__ import annotations

import argparse
import ast
import hashlib
import json
import py_compile
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EXCLUDED_PARTS = {".venv", "runtime", "outputs", "__pycache__", ".pytest_cache"}


def _required_paths() -> list[str]:
    return [
        "install_windows.bat",
        "run_experiments.bat",
        "requirements-windows.txt",
        "README_资料包说明.md",
        "config/windows_scenarios.json",
        "config/airsim_blocks_simpleflight_actor_upward_windows.json",
        "config/airsim_blocks_px4_actor_upward_windows.json",
        "examples/run_airsim_gimbal_vision_png.py",
        "examples/run_airsim_strapdown_vision_png.py",
        "tools/install_windows.ps1",
        "tools/setup_px4_wsl.sh",
        "tools/run_px4_wsl.sh",
        "tools/windows_experiments.py",
        "tools/windows_report.py",
        "vision_guidance/best.pt",
        "vision_guidance/yolo_bytetrack_detector.py",
    ]


def _python_files() -> list[Path]:
    return [
        path
        for path in PACKAGE_ROOT.rglob("*.py")
        if not any(part in EXCLUDED_PARTS for part in path.relative_to(PACKAGE_ROOT).parts)
    ]


def _check_import_closure(paths: list[Path]) -> list[str]:
    errors = []
    for path in paths:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            module = ""
            if isinstance(node, ast.ImportFrom):
                module = str(node.module or "")
            elif isinstance(node, ast.Import):
                for alias in node.names:
                    if alias.name.startswith("vision_guidance"):
                        module_path = PACKAGE_ROOT / (alias.name.replace(".", "/") + ".py")
                        package_path = PACKAGE_ROOT / alias.name.replace(".", "/") / "__init__.py"
                        if not module_path.exists() and not package_path.exists():
                            errors.append(f"{path.relative_to(PACKAGE_ROOT)} imports missing {alias.name}")
                continue
            if module.startswith("vision_guidance"):
                module_path = PACKAGE_ROOT / (module.replace(".", "/") + ".py")
                package_path = PACKAGE_ROOT / module.replace(".", "/") / "__init__.py"
                if not module_path.exists() and not package_path.exists():
                    errors.append(f"{path.relative_to(PACKAGE_ROOT)} imports missing {module}")
            if module == "run_airsim_gimbal_vision_png" and not (PACKAGE_ROOT / "examples" / "run_airsim_gimbal_vision_png.py").exists():
                errors.append(f"{path.relative_to(PACKAGE_ROOT)} imports missing gimbal helper")
    return errors


def _load_json(relative: str) -> dict:
    with (PACKAGE_ROOT / relative).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _expanded_count(config: dict, preset: str, tier: str) -> int:
    definition = config["presets"][preset][tier]
    return len(definition["scenario_ids"]) * int(definition["repeats"]) * 2


def _manifest_files() -> list[Path]:
    files = []
    for path in PACKAGE_ROOT.rglob("*"):
        if not path.is_file():
            continue
        relative = path.relative_to(PACKAGE_ROOT)
        if any(part in EXCLUDED_PARTS for part in relative.parts):
            continue
        if relative.name in {"MANIFEST.sha256"} or relative.suffix.lower() == ".zip":
            continue
        files.append(path)
    return sorted(files, key=lambda item: item.relative_to(PACKAGE_ROOT).as_posix())


def write_manifest() -> Path:
    target = PACKAGE_ROOT / "MANIFEST.sha256"
    lines = []
    for path in _manifest_files():
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        lines.append(f"{digest}  {path.relative_to(PACKAGE_ROOT).as_posix()}")
    target.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return target


def validate(runtime: bool = False) -> list[str]:
    errors = []
    for relative in _required_paths():
        path = PACKAGE_ROOT / relative
        if not path.is_file() or path.stat().st_size == 0:
            errors.append(f"missing or empty: {relative}")
    for relative in (
        "config/windows_scenarios.json",
        "config/airsim_blocks_simpleflight_actor_upward_windows.json",
        "config/airsim_blocks_px4_actor_upward_windows.json",
    ):
        try:
            _load_json(relative)
        except Exception as exc:
            errors.append(f"invalid JSON {relative}: {exc}")
    paths = _python_files()
    for path in paths:
        try:
            py_compile.compile(str(path), doraise=True)
        except py_compile.PyCompileError as exc:
            errors.append(f"compile failed {path.relative_to(PACKAGE_ROOT)}: {exc.msg}")
    errors.extend(_check_import_closure(paths))

    try:
        config = _load_json("config/windows_scenarios.json")
        if _expanded_count(config, "standard", "fast") != 120:
            errors.append("standard fast preset must expand to 120 cases")
        if _expanded_count(config, "standard", "sitl") != 24:
            errors.append("standard sitl preset must expand to 24 cases")
        common = config["runner_common_args"]
        for flag in ("--detector-source", "--airsim-image-transport", "--yolo-half", "--no-shadow-airsim-detect"):
            if flag not in common:
                errors.append(f"runner_common_args missing {flag}")
        if str(config.get("airsim_rpc_host")) != "127.0.0.2":
            errors.append("AirSim RPC host must be 127.0.0.2")
        if str(config.get("px4_tag")) != "v1.11.3":
            errors.append("PX4 tag must remain v1.11.3")
    except Exception as exc:
        errors.append(f"scenario validation failed: {exc}")

    if runtime:
        for module in ("airsim", "cv2", "lap", "matplotlib", "numpy", "pymavlink", "torch", "ultralytics"):
            try:
                __import__(module)
            except Exception as exc:
                errors.append(f"runtime import failed {module}: {exc}")
        try:
            import torch

            if not torch.cuda.is_available():
                errors.append("runtime CUDA validation failed")
        except Exception:
            pass
    return errors


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate the standalone Windows simulation package.")
    parser.add_argument("--runtime", action="store_true", help="Also import pinned third-party dependencies and require CUDA.")
    parser.add_argument("--write-manifest", action="store_true")
    args = parser.parse_args()
    errors = validate(runtime=args.runtime)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
    if args.write_manifest:
        print(f"manifest={write_manifest()}")
    print(f"package_validation=ok python_files={len(_python_files())}")


if __name__ == "__main__":
    main()
