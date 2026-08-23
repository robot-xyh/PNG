from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import json
import os
import platform
import re
import shutil
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, IO, Sequence

try:
    from windows_report import evaluate_case, write_aggregate
except ImportError:  # Imported as tools.windows_experiments by unit tests.
    from tools.windows_report import evaluate_case, write_aggregate

try:
    from validate_win10_gpu import evaluate_gpu_profile
except ImportError:  # Imported as tools.windows_experiments by unit tests.
    from tools.validate_win10_gpu import evaluate_gpu_profile


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PACKAGE_ROOT / "config" / "windows_scenarios.json"
RUNNER = PACKAGE_ROOT / "examples" / "run_airsim_strapdown_vision_png.py"
MODEL = PACKAGE_ROOT / "vision_guidance" / "best.pt"
PORTS = (41451, 4560, 14540, 14550, 14580)
FAILURE_RETRY_COUNT = 1


@dataclass(frozen=True)
class CaseSpec:
    tier: str
    scenario: dict[str, Any]
    guidance_law: str
    repeat: int

    @property
    def key(self) -> str:
        return f"{self.tier}_{self.scenario['id']}_{self.guidance_law}_r{self.repeat:02d}"


@dataclass
class OwnedProcess:
    name: str
    process: subprocess.Popen[str]
    log_stream: IO[str]

    @property
    def pid(self) -> int:
        return int(self.process.pid)

    def alive(self) -> bool:
        return self.process.poll() is None

    def stop(self, timeout_s: float = 10.0) -> None:
        if self.alive():
            if os.name == "nt":
                subprocess.run(
                    ["taskkill.exe", "/PID", str(self.pid), "/T", "/F"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )
            else:
                self.process.terminate()
            try:
                self.process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5.0)
        self.log_stream.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run repeatable Windows AirSim YOLO+ByteTrack PNG experiments.")
    parser.add_argument("--preset", choices=("smoke", "standard", "overnight"), default="standard")
    parser.add_argument("--tier", choices=("fast", "sitl", "all"), default="all")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run-id", default="")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--max-cases", type=int, default=0, help="Limit expanded cases for launcher validation.")
    parser.add_argument("--dry-run", action="store_true", help="Validate and print the plan without starting AirSim.")
    return parser.parse_args()


def _load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    if int(config.get("schema_version", 0)) != 1:
        raise SystemExit("Unsupported windows_scenarios.json schema_version")
    return config


def _config_hash(config_path: Path) -> str:
    digest = hashlib.sha256()
    for path in (config_path, RUNNER, PACKAGE_ROOT / "vision_guidance" / "yolo_bytetrack_detector.py", MODEL):
        digest.update(path.name.encode("utf-8"))
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    return digest.hexdigest()


def expand_cases(config: dict[str, Any], preset: str, tier: str) -> list[CaseSpec]:
    selected_tiers = ("fast", "sitl") if tier == "all" else (tier,)
    scenarios = {str(item["id"]): item for item in config["scenarios"]}
    cases: list[CaseSpec] = []
    for tier_name in selected_tiers:
        definition = config["presets"][preset][tier_name]
        for repeat in range(1, int(definition["repeats"]) + 1):
            for scenario_id in definition["scenario_ids"]:
                if scenario_id not in scenarios:
                    raise SystemExit(f"Preset references unknown scenario: {scenario_id}")
                for guidance_law in ("TTC", "VM"):
                    cases.append(CaseSpec(tier_name, scenarios[scenario_id], guidance_law, repeat))
    return cases


def _run_id(args: argparse.Namespace) -> str:
    if args.run_id:
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", args.run_id):
            raise SystemExit("--run-id may contain only letters, digits, dot, underscore, and hyphen")
        return args.run_id
    return f"{args.preset}_{args.tier}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"


def _command_text(command: Sequence[str]) -> str:
    return subprocess.list2cmdline([str(item) for item in command])


def _start_owned(name: str, command: Sequence[str], log_path: Path, *, cwd: Path, env: dict[str, str]) -> OwnedProcess:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    stream = log_path.open("w", encoding="utf-8", errors="replace", buffering=1)
    stream.write(f"command={_command_text(command)}\n")
    stream.flush()
    flags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0) if os.name == "nt" else 0
    try:
        process = subprocess.Popen(
            [str(item) for item in command],
            cwd=str(cwd),
            env=env,
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            creationflags=flags,
        )
    except Exception:
        stream.close()
        raise
    return OwnedProcess(name, process, stream)


def _netstat_owners() -> dict[int, set[int]]:
    if os.name != "nt":
        return {}
    completed = subprocess.run(["netstat.exe", "-ano"], capture_output=True, text=True, errors="replace", check=False)
    owners: dict[int, set[int]] = {}
    for line in completed.stdout.splitlines():
        fields = line.split()
        if len(fields) < 4 or fields[0].upper() not in {"TCP", "UDP"}:
            continue
        if fields[0].upper() == "TCP" and fields[-2].upper() != "LISTENING":
            continue
        match = re.search(r":(\d+)$", fields[1].strip("[]"))
        try:
            pid = int(fields[-1])
        except ValueError:
            continue
        if match:
            owners.setdefault(int(match.group(1)), set()).add(pid)
    return owners


def _assert_ports_free(ports: Sequence[int] = PORTS) -> None:
    owners = _netstat_owners()
    occupied = {port: sorted(owners[port]) for port in ports if port in owners}
    if occupied:
        raise RuntimeError(f"AirSim/PX4 ports are already occupied; no process was killed: {occupied}")


def _wait_ports_free(timeout_s: float = 20.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        owners = _netstat_owners()
        if not any(port in owners for port in PORTS):
            return
        time.sleep(0.5)
    _assert_ports_free()


def _wait_tcp(host: str, port: int, process: OwnedProcess, timeout_s: float = 60.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not process.alive():
            raise RuntimeError(f"{process.name} exited before {host}:{port} became ready")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.5)
            if sock.connect_ex((host, port)) == 0:
                return
        time.sleep(0.5)
    raise TimeoutError(f"Timed out waiting for {process.name} at {host}:{port}")


def _resolve_blocks_exe(config: dict[str, Any]) -> Path:
    marker = PACKAGE_ROOT / "runtime" / "blocks_path.txt"
    if marker.exists():
        candidate = Path(marker.read_text(encoding="utf-8-sig").strip())
        if candidate.exists():
            return candidate
    candidate = PACKAGE_ROOT / str(config["blocks_relative_path"])
    if candidate.exists():
        return candidate
    matches = list((PACKAGE_ROOT / "runtime" / "Blocks").rglob("Blocks.exe"))
    if matches:
        return matches[0]
    raise RuntimeError("Blocks.exe is missing. Run install_windows.bat first.")


def _start_blocks(config: dict[str, Any], tier: str, log_path: Path, env: dict[str, str]) -> OwnedProcess:
    _assert_ports_free((41451,))
    executable = _resolve_blocks_exe(config)
    settings = (PACKAGE_ROOT / config["settings"][tier]).resolve()
    command = [str(executable), f"-settings={settings}", *[str(item) for item in config["blocks_args"]]]
    process = _start_owned("AirSim Blocks", command, log_path, cwd=executable.parent, env=env)
    try:
        _wait_tcp(str(config["airsim_rpc_host"]), 41451, process, timeout_s=90.0)
    except Exception:
        process.stop()
        raise
    return process


def _wsl_command(distro: str, *arguments: str, capture: bool = False) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["wsl.exe", "-d", distro, "-u", "root", "--", *arguments],
        capture_output=capture,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )


def _wsl_path(distro: str, path: Path) -> str:
    completed = _wsl_command(distro, "wslpath", "-a", str(path.resolve()), capture=True)
    if completed.returncode != 0 or not completed.stdout.strip():
        raise RuntimeError(f"wslpath failed: {completed.stderr.strip()}")
    return completed.stdout.strip()


def parse_wsl_distribution_version(listing: str, distro: str) -> int | None:
    for line in listing.replace("\x00", "").splitlines():
        fields = line.lstrip("* ").split()
        if len(fields) >= 3 and fields[0].casefold() == distro.casefold() and fields[-1] in {"1", "2"}:
            return int(fields[-1])
    return None


def _preflight_sitl(config: dict[str, Any]) -> None:
    distro = str(config["wsl_distribution"])
    listing = subprocess.run(["wsl.exe", "--list", "--verbose"], capture_output=True, text=True, errors="replace", check=False)
    version = parse_wsl_distribution_version(listing.stdout, distro)
    if listing.returncode != 0 or version != 1:
        raise RuntimeError(f"{distro} must be the dedicated WSL1 distribution; found version {version or 'unavailable'}")
    completed = _wsl_command(
        distro,
        "bash",
        "-lc",
        "cd /opt/png-px4/PX4-Autopilot && git describe --tags --exact-match HEAD && test -x build/px4_sitl_default/bin/px4",
        capture=True,
    )
    actual = completed.stdout.strip().splitlines()[-1] if completed.stdout.strip() else ""
    if completed.returncode != 0 or actual != str(config["px4_tag"]):
        raise RuntimeError(f"PX4 SITL preflight failed: expected {config['px4_tag']}, found {actual or 'unavailable'}")
    settings = json.loads((PACKAGE_ROOT / config["settings"]["sitl"]).read_text(encoding="utf-8"))
    vehicle = settings["Vehicles"]["Interceptor"]
    if any(vehicle.get(key) != "127.0.0.1" for key in ("LocalHostIp", "ControlIp", "UdpIp")):
        raise RuntimeError("PX4/AirSim SITL settings must retain 127.0.0.1 loopback under WSL1")


def _wait_wsl_px4_ready(distro: str, process: OwnedProcess, timeout_s: float = 45.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not process.alive():
            raise RuntimeError("PX4 WSL process exited before opening TCP 4560")
        completed = _wsl_command(distro, "bash", "-lc", "ss -ltn | awk '{print $4}' | grep -Eq '(^|:)4560$'", capture=True)
        if completed.returncode == 0:
            return
        time.sleep(0.5)
    raise TimeoutError("Timed out waiting for PX4 SITL TCP 4560 in WSL1")


def _start_px4(config: dict[str, Any], session_name: str, log_path: Path, env: dict[str, str]) -> tuple[OwnedProcess, str]:
    distro = str(config["wsl_distribution"])
    script = _wsl_path(distro, PACKAGE_ROOT / "tools" / "run_px4_wsl.sh")
    pid_file = f"/tmp/png_windows_{session_name}.pid"
    command = ["wsl.exe", "-d", distro, "-u", "root", "--", "bash", script, pid_file]
    process = _start_owned("PX4 SITL WSL1", command, log_path, cwd=PACKAGE_ROOT, env=env)
    try:
        _wait_wsl_px4_ready(distro, process)
    except Exception:
        _stop_px4(config, process, pid_file)
        raise
    return process, pid_file


def _stop_px4(config: dict[str, Any], process: OwnedProcess | None, pid_file: str | None) -> None:
    if pid_file:
        distro = str(config["wsl_distribution"])
        script = (
            f"if test -f {pid_file}; then pgid=$(cat {pid_file}); "
            "kill -TERM -- -$pgid 2>/dev/null || true; sleep 2; "
            "kill -KILL -- -$pgid 2>/dev/null || true; rm -f " + pid_file + "; fi"
        )
        _wsl_command(distro, "bash", "-lc", script, capture=True)
    if process is not None:
        process.stop()


def _build_runner_command(spec: CaseSpec, config: dict[str, Any], case_dir: Path) -> tuple[list[str], Path]:
    scenario = spec.scenario
    settings = (PACKAGE_ROOT / config["settings"][spec.tier]).resolve()
    prefix = "trajectory"
    csv_path = case_dir / f"{prefix}.csv"
    command = [
        sys.executable,
        str(RUNNER),
        *[str(item) for item in config["runner_common_args"]],
        *[str(item) for item in config["runner_tier_args"][spec.tier]],
        "--duration-s", str(float(scenario.get("duration_s", 36.0))),
        "--intruder-speed", str(float(scenario["speed_mps"])),
        "--intruder-maneuver", str(scenario.get("maneuver", "straight")),
        "--intruder-maneuver-amplitude-m", str(float(scenario.get("amplitude_m", 0.0))),
        "--intruder-maneuver-period-s", str(float(scenario.get("period_s", 8.0))),
        "--start-horizontal-range-m", str(float(scenario["range_m"])),
        "--start-lateral-offset-m", str(float(scenario["lateral_m"])),
        "--intruder-altitude-offset-m", str(float(scenario["height_m"])),
        "--trajectory-dir", str(case_dir),
        "--trajectory-prefix", prefix,
        "--settings-path", str(settings),
        "--yolo-model", str(MODEL.resolve()),
    ]
    if spec.guidance_law == "TTC":
        command.extend(("--guidance-law", "ttc_png"))
    else:
        command.extend(("--guidance-law", "fixed_vm_png", "--navigation-constant", "3.0"))
    return command, csv_path


def _run_runner(command: Sequence[str], log_path: Path, timeout_s: float, env: dict[str, str]) -> tuple[int, bool]:
    process = _start_owned("case runner", command, log_path, cwd=PACKAGE_ROOT, env=env)
    timed_out = False
    try:
        process.process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        timed_out = True
        process.stop()
        return 124, timed_out
    return_code = int(process.process.returncode or 0)
    process.stop()
    return return_code, timed_out


def _case_info(spec: CaseSpec, attempt: int, config_digest: str) -> dict[str, Any]:
    scenario = spec.scenario
    return {
        "case_key": spec.key,
        "tier": spec.tier,
        "scenario_id": str(scenario["id"]),
        "guidance_law": spec.guidance_law,
        "repeat": spec.repeat,
        "attempt": attempt,
        "range_m": float(scenario["range_m"]),
        "lateral_m": float(scenario["lateral_m"]),
        "height_m": float(scenario["height_m"]),
        "target_speed_mps": float(scenario["speed_mps"]),
        "maneuver": str(scenario.get("maneuver", "straight")),
        "config_hash": config_digest,
    }


def _write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)


def _completed_resume_result(case_root: Path, config_digest: str) -> dict[str, Any] | None:
    result_path = case_root / "result.json"
    if not result_path.exists():
        return None
    with result_path.open("r", encoding="utf-8") as stream:
        result = json.load(stream)
    if result.get("infrastructure_valid") and result.get("config_hash") == config_digest:
        return result
    return None


def _next_attempt(case_root: Path) -> int:
    numbers = []
    for path in case_root.glob("attempt_*"):
        match = re.fullmatch(r"attempt_(\d+)", path.name)
        if match:
            numbers.append(int(match.group(1)))
    return max(numbers, default=0) + 1


def _package_versions() -> dict[str, str]:
    versions = {}
    for name in ("airsim", "torch", "torchvision", "ultralytics", "numpy", "opencv-python", "lapx", "matplotlib", "pymavlink"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = "missing"
    return versions


def _environment_manifest(args: argparse.Namespace, config: dict[str, Any], digest: str) -> dict[str, Any]:
    manifest: dict[str, Any] = {
        "created_local_time": datetime.now().isoformat(timespec="seconds"),
        "platform": platform.platform(),
        "python": sys.version,
        "executable": sys.executable,
        "preset": args.preset,
        "tier": args.tier,
        "config_hash": digest,
        "airsim_rpc_host": os.environ.get("AIRSIM_RPC_HOST", ""),
        "airsim_blocks_release": "1.8.1",
        "wsl_distribution": str(config.get("wsl_distribution", "")),
        "px4_expected_tag": str(config.get("px4_tag", "")),
        "packages": _package_versions(),
    }
    try:
        import torch

        manifest["cuda"] = {
            "available": bool(torch.cuda.is_available()),
            "torch_cuda": str(torch.version.cuda),
            "gpu": torch.cuda.get_device_name(0) if torch.cuda.is_available() else "",
            "compute_capability": ".".join(str(item) for item in torch.cuda.get_device_capability(0)) if torch.cuda.is_available() else "",
            "vram_bytes": int(torch.cuda.get_device_properties(0).total_memory) if torch.cuda.is_available() else 0,
            "fp16_required": True,
        }
    except Exception as exc:
        manifest["cuda"] = {"available": False, "error": str(exc)}
    if os.name == "nt":
        completed = subprocess.run(["nvidia-smi.exe", "--query-gpu=name,driver_version", "--format=csv,noheader"], capture_output=True, text=True, check=False)
        manifest["nvidia_smi"] = completed.stdout.strip()
        manifest["windows_build"] = int(sys.getwindowsversion().build)  # type: ignore[attr-defined]
        gpu_validation = PACKAGE_ROOT / "runtime" / "gpu_validation.json"
        if gpu_validation.is_file():
            try:
                manifest["install_gpu_validation"] = json.loads(gpu_validation.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                manifest["install_gpu_validation"] = {"error": str(exc)}
        if args.tier in {"sitl", "all"}:
            listing = subprocess.run(["wsl.exe", "--list", "--verbose"], capture_output=True, text=True, errors="replace", check=False)
            manifest["wsl_version"] = parse_wsl_distribution_version(listing.stdout, str(config["wsl_distribution"]))
            completed = _wsl_command(
                str(config["wsl_distribution"]),
                "bash",
                "-lc",
                "cd /opt/png-px4/PX4-Autopilot && git describe --tags --exact-match HEAD",
                capture=True,
            )
            manifest["px4_actual_tag"] = completed.stdout.strip() if completed.returncode == 0 else "unavailable"
    return manifest


def _preflight_runtime(args: argparse.Namespace, config: dict[str, Any], cases: Sequence[CaseSpec]) -> None:
    if os.name != "nt":
        raise RuntimeError("This launcher must run on Windows; use --dry-run for package validation on other platforms.")
    requirements = config.get("environment_requirements", {})
    minimum_build = int(requirements.get("windows_min_build", 19045))
    if sys.getwindowsversion().build < minimum_build:  # type: ignore[attr-defined]
        raise RuntimeError(f"Windows 10 22H2 build {minimum_build} or newer is required")
    os.environ["AIRSIM_RPC_HOST"] = "127.0.0.2"
    os.environ["AIRSIM_REWRITE_HOST_IPS"] = "0"
    if str(config["airsim_rpc_host"]) != "127.0.0.2":
        raise RuntimeError("windows_scenarios.json must use AIRSIM RPC host 127.0.0.2")
    if not MODEL.exists() or MODEL.stat().st_size == 0:
        raise RuntimeError("YOLO model is missing")
    import torch

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA PyTorch is required; rerun install_windows.bat after fixing the NVIDIA driver")
    properties = torch.cuda.get_device_properties(0)
    completed = subprocess.run(
        ["nvidia-smi.exe", "--query-gpu=driver_version", "--format=csv,noheader,nounits"],
        capture_output=True,
        text=True,
        errors="replace",
        check=False,
    )
    driver = completed.stdout.strip().splitlines()[0] if completed.returncode == 0 and completed.stdout.strip() else "0"
    profile_errors, profile_warnings = evaluate_gpu_profile(
        name=str(properties.name),
        driver=driver,
        vram_bytes=int(properties.total_memory),
        compute_capability=(int(properties.major), int(properties.minor)),
    )
    if profile_errors:
        raise RuntimeError("RTX 3080 runtime validation failed: " + "; ".join(profile_errors))
    for warning in profile_warnings:
        print(f"WARNING: {warning}")
    probe = torch.ones((32, 32), device="cuda", dtype=torch.float16) @ torch.ones((32, 32), device="cuda", dtype=torch.float16)
    torch.cuda.synchronize()
    if not bool(torch.isfinite(probe).all().item()):
        raise RuntimeError("CUDA FP16 runtime validation failed")
    _resolve_blocks_exe(config)
    _assert_ports_free()
    if any(case.tier == "sitl" for case in cases):
        _preflight_sitl(config)


def main() -> None:
    args = parse_args()
    config_path = args.config.resolve()
    config = _load_config(config_path)
    cases = expand_cases(config, args.preset, args.tier)
    if args.max_cases > 0:
        cases = cases[: args.max_cases]
    digest = _config_hash(config_path)
    run_id = _run_id(args)
    output_dir = PACKAGE_ROOT / "outputs" / run_id
    print(f"run_id={run_id} preset={args.preset} tier={args.tier} cases={len(cases)}")
    print(f"config_hash={digest}")
    for case in cases:
        print(f"plan {case.key}: range={case.scenario['range_m']} lateral={case.scenario['lateral_m']} height={case.scenario['height_m']} speed={case.scenario['speed_mps']}")
    if args.dry_run:
        return

    if output_dir.exists() and not args.resume:
        raise SystemExit(f"Output already exists: {output_dir}. Pass --resume or choose another --run-id.")
    output_dir.mkdir(parents=True, exist_ok=True)
    _preflight_runtime(args, config, cases)
    env = dict(os.environ)
    env.update({"AIRSIM_RPC_HOST": "127.0.0.2", "AIRSIM_REWRITE_HOST_IPS": "0", "PYTHONUTF8": "1"})
    _write_json(output_dir / "environment.json", _environment_manifest(args, config, digest))
    _write_json(output_dir / "run_config.json", config)
    _write_json(output_dir / "case_plan.json", [_case_info(case, 0, digest) for case in cases])

    results: list[dict[str, Any]] = []
    fast_blocks: OwnedProcess | None = None
    try:
        for index, spec in enumerate(cases, start=1):
            case_root = output_dir / "cases" / spec.key
            resumed = _completed_resume_result(case_root, digest) if args.resume else None
            if resumed is not None:
                print(f"[{index}/{len(cases)}] resume_skip={spec.key}")
                results.append(resumed)
                continue
            first_attempt = _next_attempt(case_root)
            final_result: dict[str, Any] | None = None
            for retry in range(FAILURE_RETRY_COUNT + 1):
                attempt = first_attempt + retry
                attempt_dir = case_root / f"attempt_{attempt:02d}"
                attempt_dir.mkdir(parents=True, exist_ok=True)
                print(f"[{index}/{len(cases)}] case={spec.key} attempt={attempt}")
                blocks: OwnedProcess | None = None
                px4: OwnedProcess | None = None
                px4_pid_file: str | None = None
                try:
                    if spec.tier == "fast":
                        if fast_blocks is None or not fast_blocks.alive():
                            if fast_blocks is not None:
                                fast_blocks.stop()
                                _wait_ports_free()
                            fast_blocks = _start_blocks(config, "fast", output_dir / "fast_blocks.log", env)
                        _assert_ports_free((4560, 14540, 14550, 14580))
                        _wait_tcp(str(config["airsim_rpc_host"]), 41451, fast_blocks, timeout_s=5.0)
                        blocks = fast_blocks
                    else:
                        _assert_ports_free()
                        px4, px4_pid_file = _start_px4(config, f"{run_id}_{spec.key}_{attempt}", attempt_dir / "px4.log", env)
                        blocks = _start_blocks(config, "sitl", attempt_dir / "blocks.log", env)

                    command, csv_path = _build_runner_command(spec, config, attempt_dir)
                    _write_json(attempt_dir / "command.json", {"argv": command, "command_line": _command_text(command)})
                    threshold = config["thresholds"][spec.tier]
                    return_code, timed_out = _run_runner(command, attempt_dir / "runner.log", float(threshold["timeout_s"]), env)
                    simulator_alive = bool(blocks and blocks.alive()) and (spec.tier != "sitl" or bool(px4 and px4.alive()))
                    final_result = evaluate_case(
                        csv_path,
                        case_info=_case_info(spec, attempt, digest),
                        thresholds=threshold,
                        return_code=return_code,
                        timed_out=timed_out,
                        log_path=attempt_dir / "runner.log",
                        simulator_alive=simulator_alive,
                    )
                except Exception as exc:
                    final_result = _case_info(spec, attempt, digest)
                    final_result.update(
                        {
                            "status": "infra_invalid",
                            "infrastructure_valid": False,
                            "infra_invalid_reasons": [f"orchestration_error:{type(exc).__name__}:{exc}"],
                            "collision_hit": False,
                            "return_code": 125,
                            "timed_out": False,
                            "csv_path": "",
                            "meta_path": "",
                            "log_path": str(attempt_dir / "runner.log"),
                        }
                    )
                finally:
                    if spec.tier == "sitl":
                        if blocks is not None:
                            blocks.stop()
                        _stop_px4(config, px4, px4_pid_file)
                        _wait_ports_free()

                _write_json(attempt_dir / "result.json", final_result)
                _write_json(case_root / "result.json", final_result)
                if final_result.get("infrastructure_valid"):
                    break
                print(f"infra_invalid={spec.key} reasons={final_result.get('infra_invalid_reasons')}")
                if spec.tier == "fast" and fast_blocks is not None:
                    fast_blocks.stop()
                    fast_blocks = None
                    _wait_ports_free()
            assert final_result is not None
            results.append(final_result)
            write_aggregate(results, output_dir, include_plots=False)
    finally:
        if fast_blocks is not None:
            fast_blocks.stop()
            _wait_ports_free()

    summary = write_aggregate(results, output_dir)
    _write_json(output_dir / "run_state.json", {"status": "complete", "completed_local_time": datetime.now().isoformat(timespec="seconds"), "summary": summary})
    print(f"cases_csv={output_dir / 'cases.csv'}")
    print(f"summary_json={output_dir / 'summary.json'}")
    print(f"report={output_dir / 'Windows仿真批量测试报告.md'}")


if __name__ == "__main__":
    main()
