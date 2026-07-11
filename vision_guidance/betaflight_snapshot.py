from __future__ import annotations

import csv
import hashlib
import json
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

from .betaflight_msp import BetaflightMSPAdapter, BetaflightTelemetry


MSP_OVERRIDE_PERMANENT_ID = 50
CLI_CATEGORIES = (
    "serial_ports",
    "receiver",
    "modes",
    "failsafe",
    "pid_profile",
    "rate_profile",
    "blackbox",
    "battery",
)


def classify_cli_export(text: str) -> dict[str, bool]:
    categories = {name: False for name in CLI_CATEGORIES}
    for raw_line in text.splitlines():
        line = raw_line.strip().lower()
        if not line or line.startswith("#"):
            continue
        if line.startswith("serial "):
            categories["serial_ports"] = True
        if line.startswith("map ") or "serialrx_provider" in line or "rx_spi_protocol" in line:
            categories["receiver"] = True
        if line.startswith("aux "):
            categories["modes"] = True
        if "failsafe_" in line:
            categories["failsafe"] = True
        if line.startswith("profile ") or any(token in line for token in ("set p_roll", "set p_pitch", "set p_yaw")):
            categories["pid_profile"] = True
        if line.startswith("rateprofile ") or any(
            token in line for token in ("rc_rate", "rc_expo", "roll_srate", "pitch_srate", "yaw_srate")
        ):
            categories["rate_profile"] = True
        if "blackbox_" in line:
            categories["blackbox"] = True
        if any(token in line for token in ("vbat_", "battery_", "current_meter")):
            categories["battery"] = True
    return categories


def capture_betaflight_snapshot(
    adapter: BetaflightMSPAdapter,
    output_root: str | Path,
    *,
    duration_s: float = 5.0,
    rate_hz: float = 5.0,
    cli_export: str | Path | None = None,
    source_reference: str | Path | None = None,
    sleep_fn=time.sleep,
    monotonic_fn=time.monotonic,
) -> Path:
    if duration_s <= 0.0 or rate_hz <= 0.0:
        raise ValueError("duration_s and rate_hz must be positive")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    directory = Path(output_root).expanduser() / f"betaflight_snapshot_{stamp}"
    directory.mkdir(parents=True, exist_ok=False)

    errors: list[str] = []
    identity = _read_identity(adapter, errors)
    box_ids: tuple[int, ...] = ()
    try:
        box_ids = adapter.read_box_ids()
    except Exception as exc:
        errors.append(f"box_ids: {exc}")

    telemetry_path = directory / "telemetry.csv"
    sample_count, telemetry_errors = _capture_telemetry(
        adapter,
        telemetry_path,
        duration_s=duration_s,
        rate_hz=rate_hz,
        sleep_fn=sleep_fn,
        monotonic_fn=monotonic_fn,
    )
    errors.extend(telemetry_errors)

    cli_path: Path | None = None
    cli_categories = {name: False for name in CLI_CATEGORIES}
    if cli_export:
        source = Path(cli_export).expanduser().resolve()
        if not source.is_file():
            raise FileNotFoundError(f"Betaflight CLI export not found: {source}")
        cli_path = directory / "betaflight_cli.txt"
        shutil.copyfile(source, cli_path)
        cli_categories = classify_cli_export(cli_path.read_text(encoding="utf-8", errors="replace"))

    artifacts = {telemetry_path.name: _sha256(telemetry_path)}
    if cli_path is not None:
        artifacts[cli_path.name] = _sha256(cli_path)
    reference_info: dict[str, Any] = {}
    if source_reference:
        reference = Path(source_reference).expanduser().resolve()
        reference_info = {"path": str(reference), "sha256": _sha256(reference) if reference.is_file() else ""}

    blockers = []
    if not identity or "error" in identity:
        blockers.append("fc_identity_missing")
    if MSP_OVERRIDE_PERMANENT_ID not in box_ids:
        blockers.append("msp_override_box_missing")
    if not all(cli_categories.values()):
        blockers.append("cli_configuration_incomplete")
    blockers.extend(("source_parameter_conflicts_unresolved", "manual_control_approval_required"))

    manifest = {
        "schema_version": 1,
        "created_unix_s": time.time(),
        "created_local": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "capture": {
            "duration_s": float(duration_s),
            "rate_hz": float(rate_hz),
            "sample_count": sample_count,
            "error_count": len(errors),
            "errors": errors,
        },
        "serial": {"port": adapter.port, "baud": adapter.baudrate, "timeout_s": adapter.timeout_s},
        "fc_identity": identity,
        "box_ids": list(box_ids),
        "msp_override_permanent_id": MSP_OVERRIDE_PERMANENT_ID,
        "msp_override_available": MSP_OVERRIDE_PERMANENT_ID in box_ids,
        "cli_export": {"provided": cli_path is not None, "categories": cli_categories},
        "source_reference": reference_info,
        "repository_commit": _git_commit(Path(__file__).resolve().parents[1]),
        "artifacts": artifacts,
        "readiness": {
            "log_only_ready": bool(identity and "error" not in identity and sample_count > 0),
            "control_ready": False,
            "control_blockers": sorted(set(blockers)),
        },
    }
    manifest_path = directory / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest_path


def _read_identity(adapter: BetaflightMSPAdapter, errors: list[str]) -> dict[str, Any]:
    try:
        api = adapter.read_api_version()
        version = adapter.read_fc_version()
        return {
            "api_protocol_version": api.protocol_version,
            "api_major": api.api_major,
            "api_minor": api.api_minor,
            "fc_variant": adapter.read_fc_variant(),
            "fc_version_major": version.major,
            "fc_version_minor": version.minor,
            "fc_version_patch": version.patch,
        }
    except Exception as exc:
        errors.append(f"fc_identity: {exc}")
        return {"error": str(exc)}


def _capture_telemetry(
    adapter: BetaflightMSPAdapter,
    path: Path,
    *,
    duration_s: float,
    rate_hz: float,
    sleep_fn,
    monotonic_fn,
) -> tuple[int, list[str]]:
    fields = (
        "sample", "monotonic_s", "cycle_time_us", "i2c_error_count", "sensor_flags", "mode_flags",
        "profile", "roll_deg", "pitch_deg", "yaw_deg", "vbat_v", "mah_drawn", "rssi",
        "amperage_a", "rc_channels",
    )
    errors: list[str] = []
    start = monotonic_fn()
    count = 0
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        while monotonic_fn() - start < duration_s:
            loop_start = monotonic_fn()
            try:
                telemetry = adapter.read_telemetry()
                writer.writerow(_telemetry_row(count + 1, telemetry))
                count += 1
            except Exception as exc:
                errors.append(str(exc))
            sleep_fn(max(0.0, 1.0 / rate_hz - (monotonic_fn() - loop_start)))
    return count, errors


def _telemetry_row(sample: int, telemetry: BetaflightTelemetry) -> dict[str, Any]:
    status = telemetry.status
    attitude = telemetry.attitude
    analog = telemetry.analog
    return {
        "sample": sample,
        "monotonic_s": f"{telemetry.timestamp:.9f}",
        "cycle_time_us": "" if status is None else status.cycle_time_us,
        "i2c_error_count": "" if status is None else status.i2c_error_count,
        "sensor_flags": "" if status is None else status.sensor_flags,
        "mode_flags": "" if status is None else status.mode_flags,
        "profile": "" if status is None or status.profile is None else status.profile,
        "roll_deg": "" if attitude is None else attitude.roll_deg,
        "pitch_deg": "" if attitude is None else attitude.pitch_deg,
        "yaw_deg": "" if attitude is None else attitude.yaw_deg,
        "vbat_v": "" if analog is None else analog.vbat_v,
        "mah_drawn": "" if analog is None or analog.mah_drawn is None else analog.mah_drawn,
        "rssi": "" if analog is None or analog.rssi is None else analog.rssi,
        "amperage_a": "" if analog is None or analog.amperage_a is None else analog.amperage_a,
        "rc_channels": ";".join(str(value) for value in telemetry.rc_channels),
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_commit(repository: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repository, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""
