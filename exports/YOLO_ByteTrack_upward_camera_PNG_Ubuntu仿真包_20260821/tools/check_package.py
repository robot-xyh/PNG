from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import py_compile
import re
import subprocess
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EXCLUDED_PARTS = {".venv", "runtime", "outputs", "__pycache__", ".pytest_cache"}


def _required_paths() -> list[str]:
    return [
        "install_ubuntu.sh",
        "run_experiments.sh",
        "requirements-ubuntu.txt",
        "requirements-px4-v1.11.3.txt",
        "README_资料包说明.md",
        "config/ubuntu_scenarios.json",
        "config/airsim_blocks_simpleflight_actor_upward_ubuntu.json",
        "config/airsim_blocks_px4_actor_upward_ubuntu.json",
        "examples/run_airsim_gimbal_vision_png.py",
        "examples/run_airsim_strapdown_vision_png.py",
        "patches/px4-v1.11.3-ubuntu24-stacksize.patch",
        "tools/ubuntu_experiments.py",
        "tools/ubuntu_report.py",
        "vision_guidance/best.pt",
        "vision_guidance/yolo_bytetrack_detector.py",
        "doc/YOLO_ByteTrack_upward_camera_PNG算法说明.md",
        "doc/YOLO_ByteTrack_upward_camera_PNG算法说明.docx",
        "完整方案/YOLO_ByteTrack_upward_baseline_S机动_30_50测试报告.md",
        "完整方案/YOLO_ByteTrack_upward_matrix15_多工况性能测试报告.md",
        "完整方案/YOLO_ByteTrack_upward_仿真结果汇总报告.md",
        "logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_upward_baseline_s_maneuver_30_50_20260701_231523_r35_h30.csv",
        "logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_upward_final_yolo_35_40_20260628_173525_r35_h30.csv",
        "logs/yolo_sitl_ttc_vm/yolo_sitl_TTC_upward_yolo_matrix15_20260701_202024_M05_r40_h30.csv",
    ]


def _forbidden_paths() -> list[str]:
    return [
        "install_windows.bat",
        "run_experiments.bat",
        "requirements-windows.txt",
        "config/windows_scenarios.json",
        "config/airsim_blocks_simpleflight_actor_upward_windows.json",
        "config/airsim_blocks_px4_actor_upward_windows.json",
        "tools/install_windows.ps1",
        "tools/setup_px4_wsl.sh",
        "tools/run_px4_wsl.sh",
        "tools/windows_experiments.py",
        "tools/windows_report.py",
        "tests/test_windows_experiment_tools.py",
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


def _check_markdown_assets() -> list[str]:
    errors = []
    documents = list((PACKAGE_ROOT / "doc").glob("*.md")) + list((PACKAGE_ROOT / "完整方案").glob("*.md"))
    for path in documents:
        text = path.read_text(encoding="utf-8")
        for target in re.findall(r"!\[[^]]*\]\(([^)]+)\)", text):
            relative = target.strip().strip("<>").split("#", 1)[0]
            if "://" in relative or relative.startswith("data:"):
                continue
            asset = (path.parent / relative).resolve()
            if not asset.is_file():
                errors.append(f"missing Markdown asset: {path.relative_to(PACKAGE_ROOT)} -> {target}")
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
    for relative in _forbidden_paths():
        if (PACKAGE_ROOT / relative).exists():
            errors.append(f"Windows-only file remains in Ubuntu package: {relative}")
    for relative in ("install_ubuntu.sh", "run_experiments.sh"):
        path = PACKAGE_ROOT / relative
        if path.is_file() and not os.access(path, os.X_OK):
            errors.append(f"not executable: {relative}")
        completed = subprocess.run(["bash", "-n", str(path)], capture_output=True, text=True, check=False)
        if completed.returncode != 0:
            errors.append(f"shell syntax failed {relative}: {completed.stderr.strip()}")
    for relative in (
        "config/ubuntu_scenarios.json",
        "config/airsim_blocks_simpleflight_actor_upward_ubuntu.json",
        "config/airsim_blocks_px4_actor_upward_ubuntu.json",
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
    errors.extend(_check_markdown_assets())

    try:
        config = _load_json("config/ubuntu_scenarios.json")
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
        fast_settings = _load_json(str(config["settings"]["fast"]))
        sitl_settings = _load_json(str(config["settings"]["sitl"]))
        if fast_settings.get("Vehicles", {}).get("Interceptor", {}).get("VehicleType") != "SimpleFlight":
            errors.append("fast settings must use SimpleFlight")
        sitl_vehicle = sitl_settings.get("Vehicles", {}).get("Interceptor", {})
        if sitl_vehicle.get("VehicleType") != "PX4Multirotor":
            errors.append("sitl settings must use PX4Multirotor")
        if sitl_vehicle.get("LocalHostIp") != "127.0.0.1" or sitl_vehicle.get("ControlIp") != "127.0.0.1":
            errors.append("PX4 SITL transport must use native 127.0.0.1 loopback")
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
        blocks_marker = PACKAGE_ROOT / "runtime" / "blocks_path.txt"
        px4_marker = PACKAGE_ROOT / "runtime" / "px4_path.txt"
        if not blocks_marker.is_file():
            errors.append("runtime Blocks marker is missing; rerun ./install_ubuntu.sh")
        else:
            blocks = Path(blocks_marker.read_text(encoding="utf-8-sig").strip())
            if not blocks.is_file() or not os.access(blocks, os.X_OK):
                errors.append(f"runtime Blocks launcher is unavailable: {blocks}")
        if not px4_marker.is_file():
            errors.append("runtime PX4 marker is missing; rerun ./install_ubuntu.sh")
        else:
            px4_dir = Path(px4_marker.read_text(encoding="utf-8-sig").strip())
            px4_binary = px4_dir / "build" / "px4_sitl_default" / "bin" / "px4"
            if not px4_binary.is_file() or not os.access(px4_binary, os.X_OK):
                errors.append(f"runtime PX4 SITL binary is unavailable: {px4_binary}")
            completed = subprocess.run(
                ["git", "-C", str(px4_dir), "describe", "--tags", "--exact-match", "HEAD"],
                capture_output=True,
                text=True,
                check=False,
            )
            if completed.returncode != 0 or completed.stdout.strip() != "v1.11.3":
                errors.append(f"runtime PX4 tag mismatch: {completed.stdout.strip() or 'unavailable'}")
    return errors


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate the standalone Ubuntu simulation package.")
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
