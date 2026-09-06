#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import socket
import subprocess
import time


OFFICIAL_BETAFLIGHT_COMMIT = "79065c96ba0bb5cdc675e67d7093e05dab8b330e"
OFFICIAL_BETAFLIGHT_ELF_SHA256 = "f4e4456aae4f079d1349dc7bc4037211897260eeeb8cc9c4e5691949996212be"
OFFICIAL_BETAFLIGHT_BUILD_FLAGS = ["-DUSE_GPS"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Configure an isolated official Betaflight SITL eeprom over loopback CLI."
    )
    parser.add_argument("--binary", required=True)
    parser.add_argument("--source-tree", required=True)
    parser.add_argument("--cli", required=True)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    return parser.parse_args()


def sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_commit(source_tree: Path) -> str:
    return subprocess.check_output(
        ["git", "rev-parse", "HEAD"],
        cwd=source_tree,
        text=True,
    ).strip()


def _wait_for_cli_port(process: subprocess.Popen, timeout_s: float) -> socket.socket:
    deadline = time.monotonic() + timeout_s
    last_error = ""
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Betaflight SITL exited before CLI startup: {process.returncode}")
        candidate = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        candidate.settimeout(0.25)
        try:
            candidate.connect(("127.0.0.1", 5761))
            candidate.settimeout(0.1)
            return candidate
        except OSError as exc:
            last_error = str(exc)
            candidate.close()
            time.sleep(0.05)
    raise TimeoutError(f"Betaflight SITL CLI did not open on loopback: {last_error}")


def _receive_until(
    sock: socket.socket,
    token: bytes,
    timeout_s: float,
    *,
    require_suffix: bool = False,
) -> bytes:
    deadline = time.monotonic() + timeout_s
    response = bytearray()
    def complete() -> bool:
        return response.endswith(token) if require_suffix else token in response

    while not complete() and time.monotonic() < deadline:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        response.extend(chunk)
    if not complete():
        raise TimeoutError(
            f"SITL CLI response did not contain {token!r}: {bytes(response[-500:])!r}"
        )
    return bytes(response)


def _cli_commands(path: Path) -> list[str]:
    commands = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        command = raw.strip()
        if not command or command.startswith("#"):
            continue
        if command.lower() in {"save", "exit", "defaults"}:
            raise RuntimeError(f"CLI file contains lifecycle command controlled by this tool: {command}")
        commands.append(command)
    if not commands:
        raise RuntimeError("SITL CLI file has no commands")
    return commands


def _verify_saved_configuration(process: subprocess.Popen, timeout_s: float) -> None:
    sock = _wait_for_cli_port(process, timeout_s)
    with sock:
        sock.sendall(b"#")
        _receive_until(sock, b"\r\n# ", timeout_s, require_suffix=True)
        responses = []
        for command in (
            "feature",
            "serial",
            "get gps_provider",
            "get serial_update_rate_hz",
            "aux",
            "get msp_override_channels_mask",
            "get msp_override_failsafe",
            "get p_pitch",
            "get p_roll",
            "get roll_rc_rate",
            "get pitch_rc_rate",
        ):
            sock.sendall(command.encode("ascii") + b"\r\n")
            responses.append(
                _receive_until(sock, b"\r\n# ", timeout_s, require_suffix=True)
            )
        combined = b"\n".join(responses).decode("ascii", errors="replace")
        required = (
            "Enabled:  GPS",
            "serial UART2 2 115200 57600 0 115200",
            "gps_provider = VIRTUAL",
            "serial_update_rate_hz = 2000",
            "aux 2 50 2 1700 2100 0 0",
            "msp_override_channels_mask = 15",
            "msp_override_failsafe = OFF",
            "p_pitch = 54",
            "p_roll = 51",
            "roll_rc_rate = 100",
            "pitch_rc_rate = 100",
        )
        missing = [value for value in required if value not in combined]
        if missing:
            raise RuntimeError(
                "saved SITL configuration verification failed: " + ", ".join(missing)
            )
        sock.sendall(b"exit\r\n")


def _start_sitl(
    *,
    binary: Path,
    run_dir: Path,
    console,
) -> subprocess.Popen:
    return subprocess.Popen(
        [str(binary), "127.0.0.1"],
        cwd=run_dir,
        stdin=subprocess.DEVNULL,
        stdout=console,
        stderr=subprocess.STDOUT,
    )


def _stop_sitl(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)


def configure(
    *,
    binary: Path,
    source_tree: Path,
    cli_path: Path,
    run_dir: Path,
    timeout_s: float,
) -> Path:
    binary = binary.resolve()
    source_tree = source_tree.resolve()
    cli_path = cli_path.resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    eeprom_path = run_dir / "eeprom.bin"
    if eeprom_path.exists():
        raise RuntimeError(f"refusing to replace existing SITL eeprom: {eeprom_path}")
    binary_hash = sha256_path(binary)
    source_commit = _git_commit(source_tree)
    if binary_hash != OFFICIAL_BETAFLIGHT_ELF_SHA256:
        raise RuntimeError(f"unexpected Betaflight SITL ELF SHA256: {binary_hash}")
    if source_commit != OFFICIAL_BETAFLIGHT_COMMIT:
        raise RuntimeError(f"unexpected Betaflight source commit: {source_commit}")
    commands = _cli_commands(cli_path)
    console_path = run_dir / "betaflight_configure_console.log"

    with console_path.open("wb") as console:
        process = _start_sitl(binary=binary, run_dir=run_dir, console=console)
        try:
            sock = _wait_for_cli_port(process, timeout_s)
            with sock:
                sock.sendall(b"#")
                _receive_until(sock, b"\r\n# ", timeout_s, require_suffix=True)
                for command in commands:
                    sock.sendall(command.encode("ascii") + b"\r\n")
                    response = _receive_until(
                        sock, b"\r\n# ", timeout_s, require_suffix=True
                    )
                    if b"###ERROR" in response:
                        raise RuntimeError(
                            f"SITL rejected CLI command {command!r}: {response.decode(errors='replace')}"
                        )
                sock.sendall(b"save\r\n")
                _receive_until(sock, b"Rebooting", timeout_s)
            try:
                returncode = process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                raise TimeoutError("Betaflight SITL did not exit after save")
            if returncode != 0:
                raise RuntimeError(f"Betaflight SITL save process exited with {returncode}")

            # Official SITL terminates after `save`; launch a fresh process in the
            # same run directory so it loads and verifies the persisted eeprom.
            process = _start_sitl(binary=binary, run_dir=run_dir, console=console)
            _verify_saved_configuration(process, timeout_s)
        except BaseException:
            _stop_sitl(process)
            raise
        finally:
            _stop_sitl(process)

    if not eeprom_path.is_file():
        raise RuntimeError("Betaflight SITL did not create eeprom.bin")
    manifest = {
        "schema_version": 1,
        "scope": "betaflight_sitl_configuration_v1",
        "official_source_commit": source_commit,
        "official_binary": {"path": str(binary), "sha256": binary_hash},
        "official_build_flags": OFFICIAL_BETAFLIGHT_BUILD_FLAGS,
        "cli": {"path": str(cli_path), "sha256": sha256_path(cli_path)},
        "eeprom": {"path": str(eeprom_path), "sha256": sha256_path(eeprom_path)},
        "console": {"path": str(console_path), "sha256": sha256_path(console_path)},
        "commands": commands,
    }
    manifest_path = run_dir / "betaflight_sitl_configuration_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest_path


def main() -> None:
    args = parse_args()
    path = configure(
        binary=Path(args.binary),
        source_tree=Path(args.source_tree),
        cli_path=Path(args.cli),
        run_dir=Path(args.run_dir),
        timeout_s=float(args.timeout_s),
    )
    print(path)


if __name__ == "__main__":
    main()
