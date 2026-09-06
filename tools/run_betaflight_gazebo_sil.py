#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time
from typing import Any, Callable, Sequence
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.audit_betaflight_gazebo_sil import audit_sil_run  # noqa: E402
from tools.configure_betaflight_sitl import configure  # noqa: E402
from tools.materialize_betaflight_sitl_config import (  # noqa: E402
    materialize_sitl_config,
)
from vision_guidance.betaflight_sitl import SITL_SCOPE  # noqa: E402


DEFAULT_BETAFLIGHT_SOURCE = Path("/home/linux/betaflight-2025.12.2-sitl")
DEFAULT_BETAFLIGHT_BINARY = (
    DEFAULT_BETAFLIGHT_SOURCE / "obj/main/betaflight_SITL.elf"
)
SIL_PORTS = ((socket.SOCK_STREAM, 5761),) + tuple(
    (socket.SOCK_DGRAM, port) for port in (9001, 9002, 9003, 9004)
)
GAZEBO_WORLD_NAME = "png_betaflight_sitl"
RUNNER_READY_MARKER = "MSP RAW_IMU gyro:"
TARGET_APPROACH_BY_SCENARIO = {
    ("noncollision", "projected"): (7.5, 10.0, 5.95),
    ("noncollision", "rendered"): (7.1, 2.5, 6.2),
    ("contact", "projected"): (7.5, 10.0, 5.95),
    ("contact", "rendered"): (7.5, 10.0, 5.95),
}


@dataclass
class ManagedProcess:
    name: str
    process: subprocess.Popen
    console_path: Path
    console_stream: Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one isolated official Betaflight 2025.12.2 + Gazebo SIL audit."
    )
    parser.add_argument("--policy", choices=("noncollision", "contact"), required=True)
    parser.add_argument(
        "--detector-mode", choices=("projected", "rendered"), default="projected"
    )
    parser.add_argument("--base-config", default="")
    parser.add_argument("--betaflight-source", default=str(DEFAULT_BETAFLIGHT_SOURCE))
    parser.add_argument("--betaflight-binary", default=str(DEFAULT_BETAFLIGHT_BINARY))
    parser.add_argument(
        "--cli", default=str(ROOT / "sitl/betaflight/sitl_cli_2025_12_2.txt")
    )
    parser.add_argument(
        "--world", default=str(ROOT / "sitl/gazebo/worlds/png_betaflight_sitl.sdf")
    )
    parser.add_argument("--run-root", default=str(ROOT / "logs/betaflight_sitl"))
    parser.add_argument("--duration-s", type=float, default=14.0)
    parser.add_argument("--stop-after-disarm-s", type=float, default=4.0)
    parser.add_argument("--yolo-model", default=str(ROOT / "vision_guidance/best.pt"))
    parser.add_argument("--yolo-class-id", type=int, default=0)
    parser.add_argument(
        "--yolo-device",
        default="0",
        help="Ultralytics device for rendered SIL; use cpu only if it meets the flight detection deadline.",
    )
    parser.add_argument("--startup-wait-s", type=float, default=2.0)
    return parser.parse_args()


def sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _artifact(path: Path) -> dict[str, Any]:
    resolved = path.expanduser().resolve()
    return {
        "path": str(resolved),
        "sha256": sha256_path(resolved),
        "bytes": resolved.stat().st_size,
    }


def materialize_target_model(
    source_path: Path, resource_root: Path, policy: str, detector_mode: str
) -> Path:
    try:
        approach_start_s, approach_speed_m_s, maximum_approach_m = (
            TARGET_APPROACH_BY_SCENARIO[(policy, detector_mode)]
        )
    except KeyError as exc:
        raise ValueError(
            f"unsupported SIL scenario: policy={policy}, detector={detector_mode}"
        ) from exc
    tree = ET.parse(source_path)
    plugin = tree.getroot().find(
        ".//plugin[@name='png::sitl::DeterministicTargetMotion']"
    )
    if plugin is None:
        raise RuntimeError("target model is missing DeterministicTargetMotion")
    start = plugin.find("verticalApproachStartS")
    speed = plugin.find("verticalApproachSpeedMps")
    maximum = plugin.find("maximumVerticalApproachM")
    if start is None or speed is None or maximum is None:
        raise RuntimeError("target model is missing vertical approach parameters")
    start.text = str(approach_start_s)
    speed.text = str(approach_speed_m_s)
    maximum.text = str(maximum_approach_m)
    output_path = resource_root / "target_uav" / "model.sdf"
    output_path.parent.mkdir(parents=True, exist_ok=False)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    shutil.copy2(
        source_path.with_name("model.config"), output_path.with_name("model.config")
    )
    return output_path


def _repository_state() -> dict[str, Any]:
    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip()
    status = subprocess.check_output(
        ["git", "status", "--porcelain", "--untracked-files=normal"],
        cwd=ROOT,
        text=True,
    )
    return {"repository_commit": commit, "repository_dirty": bool(status.strip())}


def _probe_ports_available() -> tuple[bool, list[int]]:
    sockets: list[socket.socket] = []
    occupied: list[int] = []
    try:
        for socket_type, port in SIL_PORTS:
            candidate = socket.socket(socket.AF_INET, socket_type)
            sockets.append(candidate)
            try:
                candidate.bind(("127.0.0.1", port))
                if socket_type == socket.SOCK_STREAM:
                    candidate.listen(1)
            except OSError:
                occupied.append(port)
    finally:
        for candidate in sockets:
            candidate.close()
    return not occupied, occupied


def _wait_for_tcp_listener(process: subprocess.Popen, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    last_error = ""
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                f"Betaflight exited before MSP listener startup: {process.returncode}"
            )
        candidate = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            candidate.bind(("127.0.0.1", 5761))
        except OSError as exc:
            # Avoid a probe connection: official SITL treats every accepted
            # socket as its single UART client and may not recover before the
            # real MSP runner connects.
            if exc.errno in {98, 48, 10048}:
                return
            last_error = str(exc)
        finally:
            candidate.close()
        time.sleep(0.05)
    raise TimeoutError(f"Betaflight MSP listener did not open: {last_error}")


def _wait_process_alive(process: subprocess.Popen, wait_s: float, name: str) -> None:
    deadline = time.monotonic() + max(0.0, wait_s)
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"{name} exited during startup: {process.returncode}")
        time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))


def _wait_for_log_marker(
    process: subprocess.Popen,
    path: Path,
    marker: str,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                f"runner exited before readiness marker: {process.returncode}"
            )
        if path.is_file() and marker in path.read_text(
            encoding="utf-8", errors="replace"
        ):
            return
        time.sleep(0.05)
    raise TimeoutError(f"runner readiness marker not observed: {marker}")


def _wait_for_bound_port(
    process: subprocess.Popen,
    socket_type: int,
    port: int,
    timeout_s: float,
    name: str,
) -> None:
    deadline = time.monotonic() + timeout_s
    last_error = ""
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"{name} exited before binding port {port}")
        candidate = socket.socket(socket.AF_INET, socket_type)
        try:
            candidate.bind(("127.0.0.1", port))
        except OSError as exc:
            if exc.errno in {98, 48, 10048}:
                return
            last_error = str(exc)
        finally:
            candidate.close()
        time.sleep(0.05)
    raise TimeoutError(f"{name} did not bind port {port}: {last_error}")


def _run_checked(
    command: Sequence[str],
    *,
    cwd: Path,
    console_path: Path,
    env: dict[str, str] | None = None,
) -> None:
    with console_path.open("wb") as stream:
        result = subprocess.run(
            list(command),
            cwd=cwd,
            env=env,
            stdout=stream,
            stderr=subprocess.STDOUT,
            check=False,
        )
    if result.returncode != 0:
        raise RuntimeError(
            f"command failed with {result.returncode}; see {console_path}: {' '.join(command)}"
        )


def _start_logged_process(
    name: str,
    command: Sequence[str],
    *,
    cwd: Path,
    console_path: Path,
    env: dict[str, str] | None = None,
) -> ManagedProcess:
    stream = console_path.open("wb")
    try:
        process = subprocess.Popen(
            list(command),
            cwd=cwd,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except BaseException:
        stream.close()
        raise
    return ManagedProcess(name, process, console_path, stream)


def stop_processes(processes: Sequence[ManagedProcess]) -> dict[str, int | None]:
    returncodes: dict[str, int | None] = {}
    for managed in reversed(processes):
        process = managed.process
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        returncodes[managed.name] = process.returncode
        managed.console_stream.close()
    return returncodes


def start_runtime_stack(
    *,
    binary: Path,
    eeprom_dir: Path,
    gazebo_command: Sequence[str],
    runner_command: Sequence[str],
    root: Path,
    run_dir: Path,
    gazebo_env: dict[str, str],
    startup_wait_s: float,
    start_process: Callable[..., ManagedProcess] = _start_logged_process,
    wait_for_listener: Callable[[subprocess.Popen, float], None] = _wait_for_tcp_listener,
    wait_alive: Callable[[subprocess.Popen, float, str], None] = _wait_process_alive,
    wait_for_gazebo: Callable[[subprocess.Popen, int, int, float, str], None] = (
        _wait_for_bound_port
    ),
) -> tuple[list[ManagedProcess], list[str]]:
    processes: list[ManagedProcess] = []
    sequence: list[str] = []
    try:
        betaflight = start_process(
            "betaflight",
            [str(binary), "127.0.0.1"],
            cwd=eeprom_dir,
            console_path=run_dir / "betaflight_runtime_console.log",
        )
        processes.append(betaflight)
        sequence.append("start_betaflight")
        wait_for_listener(betaflight.process, 10.0)
        wait_alive(betaflight.process, startup_wait_s, "Betaflight")

        gazebo = start_process(
            "gazebo",
            gazebo_command,
            cwd=root,
            console_path=run_dir / "gazebo_console.log",
            env=gazebo_env,
        )
        processes.append(gazebo)
        sequence.append("start_gazebo")
        wait_for_gazebo(
            gazebo.process, socket.SOCK_DGRAM, 9002, max(10.0, startup_wait_s), "Gazebo"
        )
        wait_alive(gazebo.process, max(4.0, startup_wait_s), "Gazebo")

        runner = start_process(
            "runner",
            runner_command,
            cwd=root,
            console_path=run_dir / "runner_console.log",
        )
        processes.append(runner)
        sequence.append("start_runner")
        return processes, sequence
    except BaseException:
        stop_processes(processes)
        raise


def _base_config(policy: str, value: str) -> Path:
    if value:
        return Path(value).expanduser().resolve()
    filename = (
        "betaflight.rk3588.velocity_png.flight_supervised.json"
        if policy == "noncollision"
        else "betaflight.rk3588.velocity_png.flight_contact_supervised.json"
    )
    return (ROOT / "config" / filename).resolve()


def _runner_artifacts(runner_dir: Path) -> dict[str, Path]:
    matches = {
        "runner_csv": sorted(runner_dir.glob("*.csv")),
        "runner_meta": sorted(runner_dir.glob("*_meta.json")),
        "runner_manifest": sorted(runner_dir.glob("*_runtime_manifest.json")),
    }
    resolved: dict[str, Path] = {}
    for name, paths in matches.items():
        if len(paths) != 1:
            raise RuntimeError(
                f"expected exactly one {name} artifact in {runner_dir}, found {len(paths)}"
            )
        resolved[name] = paths[0]
    return resolved


def run_sil(args: argparse.Namespace) -> tuple[Path, Path]:
    if args.duration_s < 12.0:
        raise ValueError("SIL duration must be at least 12 seconds")
    ports_available, occupied = _probe_ports_available()
    if not ports_available:
        raise RuntimeError(f"SIL ports are already occupied: {occupied}")

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    run_dir = (
        Path(args.run_root).expanduser().resolve()
        / f"{args.policy}_{args.detector_mode}_{timestamp}"
    )
    run_dir.mkdir(parents=True, exist_ok=False)
    runner_dir = run_dir / "runner"
    runner_dir.mkdir()
    eeprom_dir = run_dir / "betaflight_eeprom"
    build_dir = run_dir / "gazebo_build"

    binary = Path(args.betaflight_binary).expanduser().resolve()
    source_tree = Path(args.betaflight_source).expanduser().resolve()
    cli_path = Path(args.cli).expanduser().resolve()
    world_path = Path(args.world).expanduser().resolve()
    base_config = _base_config(args.policy, args.base_config)
    yolo_model = Path(args.yolo_model).expanduser().resolve()
    required_paths = [binary, source_tree, cli_path, world_path, base_config]
    if args.detector_mode == "rendered":
        required_paths.append(yolo_model)
    for required in required_paths:
        if not required.exists():
            raise FileNotFoundError(required)

    base_bytes = base_config.read_bytes()
    generated_config = materialize_sitl_config(
        json.loads(base_bytes),
        policy=args.policy,
        source_path=base_config,
        source_sha256=hashlib.sha256(base_bytes).hexdigest(),
        simulated_vbat_v=23.6,
    )
    sitl_config = run_dir / "sitl_runtime_config.json"
    sitl_config.write_text(
        json.dumps(generated_config, indent=2, sort_keys=False) + "\n",
        encoding="utf-8",
    )

    startup_sequence = ["configure_betaflight"]
    configuration_manifest = configure(
        binary=binary,
        source_tree=source_tree,
        cli_path=cli_path,
        run_dir=eeprom_dir,
        timeout_s=20.0,
    )

    gazebo_source = ROOT / "sitl/gazebo"
    gazebo_resource_root = run_dir / "gazebo_models"
    target_model = materialize_target_model(
        gazebo_source / "models/target_uav/model.sdf",
        gazebo_resource_root,
        args.policy,
        args.detector_mode,
    )
    build_console = run_dir / "gazebo_build_console.log"
    with build_console.open("wb") as stream:
        configure_result = subprocess.run(
            ["cmake", "-S", str(gazebo_source), "-B", str(build_dir)],
            cwd=ROOT,
            stdout=stream,
            stderr=subprocess.STDOUT,
            check=False,
        )
        build_result = (
            subprocess.run(
                ["cmake", "--build", str(build_dir), "--parallel", "2"],
                cwd=ROOT,
                stdout=stream,
                stderr=subprocess.STDOUT,
                check=False,
            )
            if configure_result.returncode == 0
            else None
        )
    if configure_result.returncode != 0 or build_result is None or build_result.returncode != 0:
        raise RuntimeError(f"Gazebo bridge build failed; see {build_console}")
    libraries = list(build_dir.rglob("libPngBetaflightSilBridge.so"))
    if len(libraries) != 1:
        raise RuntimeError("Gazebo bridge build did not produce one shared library")
    bridge_library = libraries[0].resolve()

    gazebo_env = dict(os.environ)
    gazebo_env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(
        filter(
            None,
            (str(bridge_library.parent), gazebo_env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")),
        )
    )
    gazebo_env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        filter(
            None,
            (
                str(gazebo_resource_root),
                str(gazebo_source / "models"),
                gazebo_env.get("GZ_SIM_RESOURCE_PATH", ""),
            ),
        )
    )
    gazebo_command = ["gz", "sim", "-s", str(world_path)]
    detector_source = (
        "sitl_projected" if args.detector_mode == "projected" else "gazebo_yolo_bytetrack"
    )
    prefix = f"SIL_{args.detector_mode.upper()}_{args.policy.upper()}_{timestamp}"
    runner_command = [
        sys.executable,
        "-u",
        str(ROOT / "examples/run_betaflight_log_only.py"),
        "--config",
        str(sitl_config),
        "--allow-control",
        "--control-mode",
        "msp_raw_rc",
        "--duration-s",
        str(args.duration_s),
        "--stop-after-disarm-s",
        str(args.stop_after_disarm_s),
        "--rate-hz",
        "50",
        "--log-dir",
        str(runner_dir),
        "--log-prefix",
        prefix,
        "--detector-source",
        detector_source,
        "--sitl-loopback",
        "--disable-web-preview",
    ]
    if args.detector_mode == "rendered":
        runner_command.extend(
            [
                "--yolo-model",
                str(yolo_model),
                "--yolo-class-id",
                str(args.yolo_class_id),
                "--yolo-device",
                str(args.yolo_device),
                "--gazebo-camera-topic",
                "/world/png_betaflight_sitl/model/interceptor/link/base_link/sensor/upward_camera/image",
            ]
        )

    processes: list[ManagedProcess] = []
    process_returncodes: dict[str, int | None] = {}
    completed = False
    failure = ""
    try:
        processes, runtime_sequence = start_runtime_stack(
            binary=binary,
            eeprom_dir=eeprom_dir,
            gazebo_command=gazebo_command,
            runner_command=runner_command,
            root=ROOT,
            run_dir=run_dir,
            gazebo_env=gazebo_env,
            startup_wait_s=float(args.startup_wait_s),
        )
        startup_sequence.extend(runtime_sequence)
        runner = next(item for item in processes if item.name == "runner")
        _wait_for_log_marker(
            runner.process,
            runner.console_path,
            RUNNER_READY_MARKER,
            30.0,
        )
        _run_checked(
            [
                "gz",
                "service",
                "-s",
                f"/world/{GAZEBO_WORLD_NAME}/control",
                "--reqtype",
                "gz.msgs.WorldControl",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                "pause: false",
            ],
            cwd=ROOT,
            console_path=run_dir / "gazebo_unpause_console.log",
            env=gazebo_env,
        )
        startup_sequence.append("unpause_gazebo_after_runner_ready")
        try:
            runner_returncode = runner.process.wait(timeout=float(args.duration_s) + 45.0)
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError("SIL runner exceeded its duration and cleanup allowance") from exc
        process_returncodes["runner"] = runner_returncode
        if runner_returncode != 0:
            raise RuntimeError(
                f"SIL runner exited with {runner_returncode}; see {runner.console_path}"
            )
        completed = True
    except BaseException as exc:
        failure = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        process_returncodes.update(stop_processes(processes))
        deadline = time.monotonic() + 5.0
        released = False
        occupied_after: list[int] = []
        while time.monotonic() < deadline:
            released, occupied_after = _probe_ports_available()
            if released:
                break
            time.sleep(0.1)

        artifacts: dict[str, Any] = {}
        fixed_artifacts = {
            "flight_config": base_config,
            "sitl_config": sitl_config,
            "configuration_manifest": configuration_manifest,
            "betaflight_binary": binary,
            "betaflight_cli": cli_path,
            "eeprom": eeprom_dir / "eeprom.bin",
            "gazebo_world": world_path,
            "gazebo_bridge_source": gazebo_source / "PngBetaflightSilBridge.cc",
            "gazebo_bridge_library": bridge_library,
            "interceptor_model": gazebo_source / "models/interceptor/model.sdf",
            "target_model": target_model,
            "target_model_source": gazebo_source / "models/target_uav/model.sdf",
            "gazebo_build_console": build_console,
            "betaflight_configure_console": eeprom_dir / "betaflight_configure_console.log",
            "betaflight_console": run_dir / "betaflight_runtime_console.log",
            "gazebo_console": run_dir / "gazebo_console.log",
            "gazebo_unpause_console": run_dir / "gazebo_unpause_console.log",
            "runner_console": run_dir / "runner_console.log",
        }
        if args.detector_mode == "rendered":
            fixed_artifacts["yolo_model"] = yolo_model
        for name, path in fixed_artifacts.items():
            if path.is_file():
                artifacts[name] = _artifact(path)
        try:
            for name, path in _runner_artifacts(runner_dir).items():
                artifacts[name] = _artifact(path)
        except RuntimeError as exc:
            if completed:
                completed = False
                failure = str(exc)

        orchestration_manifest = run_dir / "betaflight_gazebo_sil_run_manifest.json"
        manifest = {
            "schema_version": 1,
            "evidence_type": "betaflight_gazebo_sil_run",
            "scope": SITL_SCOPE,
            "created_unix_s": time.time(),
            "policy": args.policy,
            "detector_mode": args.detector_mode,
            "detector_configuration": {
                "yolo_class_id": (
                    int(args.yolo_class_id)
                    if args.detector_mode == "rendered"
                    else None
                ),
                "yolo_device": (
                    str(args.yolo_device)
                    if args.detector_mode == "rendered"
                    else None
                ),
            },
            "completed": completed,
            "failure": failure,
            "startup_sequence": startup_sequence,
            "software_binding": _repository_state(),
            "commands": {
                "gazebo": gazebo_command,
                "runner": runner_command,
            },
            "process_returncodes": process_returncodes,
            "cleanup": {
                "ports_released": released,
                "occupied_ports": occupied_after,
            },
            "artifacts": artifacts,
        }
        orchestration_manifest.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    audit_report = audit_sil_run(orchestration_manifest)
    audit_path = run_dir / "betaflight_gazebo_sil_audit.json"
    audit_path.write_text(
        json.dumps(audit_report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if not audit_report["passed"]:
        raise RuntimeError(
            f"SIL audit failed: {', '.join(audit_report['violations'])}; see {audit_path}"
        )
    return orchestration_manifest, audit_path


def main() -> None:
    args = parse_args()
    manifest_path, audit_path = run_sil(args)
    print(f"run_manifest={manifest_path}")
    print(f"audit_evidence={audit_path}")


if __name__ == "__main__":
    main()
