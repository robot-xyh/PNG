from __future__ import annotations

import csv
import hashlib
import json
import re
import shutil
import subprocess
import time
from itertools import zip_longest
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
_SET_RE = re.compile(r"^set\s+([^\s=]+)\s*=\s*(.*?)\s*$", re.IGNORECASE)
_KNOWN_PASSTHROUGH_COMMANDS = {
    "adjrange", "batch", "beeper", "color", "defaults", "dma", "feature", "led",
    "master", "mixer", "mmix", "mode_color", "resource", "rxrange", "save", "servo",
    "smix", "timer", "vtxtable",
}


def classify_cli_export(text: str) -> dict[str, bool]:
    return dict(parse_cli_export(text)["categories"])


def parse_cli_export(text: str) -> dict[str, Any]:
    categories = {name: False for name in CLI_CATEGORIES}
    serial_ports: list[dict[str, Any]] = []
    aux_ranges: list[dict[str, Any]] = []
    rx_failsafe: list[dict[str, Any]] = []
    channel_map = ""
    global_settings: dict[str, str] = {}
    pid_profiles: dict[str, dict[str, str]] = {}
    rate_profiles: dict[str, dict[str, str]] = {}
    duplicate_assignments: list[dict[str, str]] = []
    malformed_commands: list[str] = []
    unparsed_commands: list[str] = []
    firmware_header = ""
    context = "global"
    profile_index = ""
    command_count = 0

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("#"):
            comment = line.lstrip("# ").strip()
            comment_lower = comment.lower()
            if "betaflight" in comment_lower and not firmware_header:
                firmware_header = line.lstrip("# ")
            if comment_lower == "master":
                context = "global"
                profile_index = ""
            elif re.fullmatch(r"profile\s+\d+", comment_lower):
                context = "pid"
                profile_index = comment_lower.split()[1]
            elif re.fullmatch(r"rateprofile\s+\d+", comment_lower):
                context = "rate"
                profile_index = comment_lower.split()[1]
            continue
        command_count += 1
        lower = line.lower()
        tokens = line.split()
        command = tokens[0].lower()

        if command == "batch" and len(tokens) > 1 and tokens[1].lower() == "start":
            context = "global"
            profile_index = ""
            continue
        if command == "defaults":
            context = "global"
            profile_index = ""
            continue

        if command == "serial":
            categories["serial_ports"] = True
            values = _parse_int_tokens(tokens[2:])
            if len(tokens) < 3 or values is None or len(values) < 1:
                malformed_commands.append(line)
            else:
                serial_ports.append({
                    "identifier": _parse_identifier(tokens[1]),
                    "function_mask": values[0],
                    "baud_values": values[1:],
                    "raw": line,
                })
            continue
        if command == "map":
            categories["receiver"] = True
            if len(tokens) == 2:
                channel_map = tokens[1].upper()
            else:
                malformed_commands.append(line)
            continue
        if command == "aux":
            categories["modes"] = True
            values = _parse_int_tokens(tokens[1:])
            if values is None or len(values) < 5:
                malformed_commands.append(line)
            else:
                aux_ranges.append({
                    "index": values[0],
                    "mode_id": values[1],
                    "aux_channel_index": values[2],
                    "range_start_us": values[3],
                    "range_end_us": values[4],
                    "logic": values[5] if len(values) > 5 else None,
                    "linked_to": values[6] if len(values) > 6 else None,
                    "raw": line,
                })
            continue
        if command == "rxfail":
            categories["failsafe"] = True
            if len(tokens) not in {3, 4} or not tokens[1].isdigit():
                malformed_commands.append(line)
            else:
                rx_failsafe.append({
                    "channel_index": int(tokens[1]),
                    "mode": tokens[2].lower(),
                    "value": int(tokens[3]) if len(tokens) == 4 and tokens[3].isdigit() else None,
                    "raw": line,
                })
            continue
        if command in {"profile", "rateprofile"}:
            if len(tokens) != 2 or not tokens[1].isdigit():
                malformed_commands.append(line)
                continue
            context = "pid" if command == "profile" else "rate"
            profile_index = tokens[1]
            categories["pid_profile" if context == "pid" else "rate_profile"] = True
            continue
        if command == "master":
            context = "global"
            profile_index = ""
            continue

        setting_match = _SET_RE.match(line)
        if setting_match:
            name = setting_match.group(1).lower()
            value = setting_match.group(2)
            target, scope = _setting_target(
                context, profile_index, global_settings, pid_profiles, rate_profiles
            )
            if name in target and target[name] != value:
                duplicate_assignments.append({
                    "scope": scope, "name": name, "previous": target[name], "value": value,
                })
            target[name] = value
            _mark_setting_categories(name, categories)
            continue

        if command in _KNOWN_PASSTHROUGH_COMMANDS:
            if command == "rxrange":
                categories["receiver"] = True
            if command == "feature" and "blackbox" in lower:
                categories["blackbox"] = True
            continue
        unparsed_commands.append(line)

    return {
        "line_count": len(text.splitlines()),
        "command_count": command_count,
        "firmware_header": firmware_header,
        "categories": categories,
        "serial_ports": serial_ports,
        "receiver": {"channel_map": channel_map},
        "aux_ranges": aux_ranges,
        "rx_failsafe": rx_failsafe,
        "settings": global_settings,
        "pid_profiles": pid_profiles,
        "rate_profiles": rate_profiles,
        "duplicate_assignments": duplicate_assignments,
        "malformed_commands": malformed_commands,
        "unparsed_commands": unparsed_commands,
    }


def review_cli_exports(exports: dict[str, dict[str, Any]]) -> dict[str, Any]:
    categories = {name: False for name in CLI_CATEGORIES}
    for parsed in exports.values():
        for name, present in parsed["categories"].items():
            categories[name] = categories[name] or bool(present)

    conflicts: list[dict[str, Any]] = []
    diff = exports.get("diff_all")
    dump = exports.get("dump_all")
    if diff is not None and dump is not None:
        diff_values = _comparable_cli_values(diff)
        dump_values = _comparable_cli_values(dump)
        for key in sorted(diff_values.keys() & dump_values.keys()):
            if diff_values[key] != dump_values[key]:
                conflicts.append({"key": key, "diff_all": diff_values[key], "dump_all": dump_values[key]})

    malformed_count = sum(len(parsed["malformed_commands"]) for parsed in exports.values())
    duplicate_count = sum(len(parsed["duplicate_assignments"]) for parsed in exports.values())
    missing_categories = sorted(name for name, present in categories.items() if not present)
    missing_structures = _missing_cli_structures(exports)
    evidence_complete = bool(
        "diff_all" in exports
        and "dump_all" in exports
        and not missing_categories
        and not missing_structures
        and not conflicts
        and malformed_count == 0
        and duplicate_count == 0
    )
    return {
        "exports_provided": {"diff_all": "diff_all" in exports, "dump_all": "dump_all" in exports},
        "categories": categories,
        "missing_categories": missing_categories,
        "missing_structures": missing_structures,
        "cross_export_conflicts": conflicts,
        "malformed_command_count": malformed_count,
        "duplicate_assignment_count": duplicate_count,
        "configuration_evidence_complete": evidence_complete,
        "control_ready": False,
    }


def capture_betaflight_snapshot(
    adapter: BetaflightMSPAdapter,
    output_root: str | Path,
    *,
    duration_s: float = 5.0,
    rate_hz: float = 5.0,
    cli_export: str | Path | None = None,
    cli_diff_all: str | Path | None = None,
    cli_dump_all: str | Path | None = None,
    source_reference: str | Path | None = None,
    include_kinematics: bool = False,
    sleep_fn=time.sleep,
    monotonic_fn=time.monotonic,
) -> Path:
    if duration_s <= 0.0 or rate_hz <= 0.0:
        raise ValueError("duration_s and rate_hz must be positive")
    if cli_export and (cli_diff_all or cli_dump_all):
        raise ValueError("cli_export cannot be combined with cli_diff_all or cli_dump_all")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    directory = Path(output_root).expanduser() / f"betaflight_snapshot_{stamp}"
    directory.mkdir(parents=True, exist_ok=False)

    errors: list[str] = []
    identity = _read_identity(adapter, errors)
    box_ids = _read_box_ids(adapter, errors)
    box_names = _read_box_names(adapter, errors)
    box_modes = _pair_box_modes(box_ids, box_names)

    telemetry_path = directory / "telemetry.csv"
    sample_count, telemetry_errors = _capture_telemetry(
        adapter,
        telemetry_path,
        duration_s=duration_s,
        rate_hz=rate_hz,
        include_kinematics=include_kinematics,
        sleep_fn=sleep_fn,
        monotonic_fn=monotonic_fn,
    )
    errors.extend(telemetry_errors)

    export_sources: dict[str, str | Path] = {}
    if cli_export:
        export_sources["legacy"] = cli_export
    if cli_diff_all:
        export_sources["diff_all"] = cli_diff_all
    if cli_dump_all:
        export_sources["dump_all"] = cli_dump_all

    parsed_exports: dict[str, dict[str, Any]] = {}
    export_files: dict[str, str] = {}
    for kind, source_value in export_sources.items():
        source = Path(source_value).expanduser().resolve()
        if not source.is_file():
            raise FileNotFoundError(f"Betaflight CLI export not found: {source}")
        filename = {
            "legacy": "betaflight_cli.txt",
            "diff_all": "betaflight_diff_all.txt",
            "dump_all": "betaflight_dump_all.txt",
        }[kind]
        destination = directory / filename
        shutil.copyfile(source, destination)
        parsed_exports[kind] = parse_cli_export(destination.read_text(encoding="utf-8", errors="replace"))
        export_files[kind] = filename

    review_exports = dict(parsed_exports)
    if "legacy" in review_exports:
        review_exports["diff_all"] = review_exports.pop("legacy")
    review = review_cli_exports(review_exports)
    review["exports"] = parsed_exports
    review_path = directory / "configuration_review.json"
    review_path.write_text(json.dumps(review, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    artifacts = {
        telemetry_path.name: _sha256(telemetry_path),
        review_path.name: _sha256(review_path),
    }
    for filename in export_files.values():
        artifacts[filename] = _sha256(directory / filename)

    reference_info: dict[str, Any] = {}
    if source_reference:
        reference = Path(source_reference).expanduser().resolve()
        reference_info = {"path": str(reference), "sha256": _sha256(reference) if reference.is_file() else ""}

    blockers = []
    if not identity or "error" in identity:
        blockers.append("fc_identity_missing")
    if not box_names:
        blockers.append("box_names_missing")
    if len(box_ids) != len(box_names):
        blockers.append("box_id_name_count_mismatch")
    if MSP_OVERRIDE_PERMANENT_ID not in box_ids:
        blockers.append("msp_override_box_missing")
    if not review["exports_provided"]["diff_all"]:
        blockers.append("cli_diff_all_missing")
    if not review["exports_provided"]["dump_all"]:
        blockers.append("cli_dump_all_missing")
    if review["missing_categories"] or review["missing_structures"]:
        blockers.append("cli_configuration_incomplete")
    if review["cross_export_conflicts"]:
        blockers.append("cli_export_conflict")
    if review["malformed_command_count"]:
        blockers.append("cli_export_malformed")
    if review["duplicate_assignment_count"]:
        blockers.append("cli_duplicate_assignment")
    clock_status = _read_clock_status()
    if not clock_status["ntp_synchronized"]:
        blockers.append("clock_not_synchronized")
    if not clock_status["rtc_matches_system_date"]:
        blockers.append("rtc_not_aligned")
    blockers.extend(("source_parameter_conflicts_unresolved", "manual_control_approval_required"))

    override_mode = next(
        (mode for mode in box_modes if mode["permanent_id"] == MSP_OVERRIDE_PERMANENT_ID), None
    )
    manifest = {
        "schema_version": 2,
        "created_unix_s": time.time(),
        "created_local": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "clock": clock_status,
        "capture": {
            "duration_s": float(duration_s),
            "rate_hz": float(rate_hz),
            "sample_count": sample_count,
            "include_kinematics": bool(include_kinematics),
            "error_count": len(errors),
            "errors": errors,
        },
        "serial": {"port": adapter.port, "baud": adapter.baudrate, "timeout_s": adapter.timeout_s},
        "fc_identity": identity,
        "box_ids": list(box_ids),
        "box_names": list(box_names),
        "box_modes": box_modes,
        "msp_override_permanent_id": MSP_OVERRIDE_PERMANENT_ID,
        "msp_override_available": override_mode is not None,
        "msp_override_mode": override_mode,
        "cli_configuration": {
            "files": export_files,
            "review_artifact": review_path.name,
            **{key: value for key, value in review.items() if key != "exports"},
        },
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


def _parse_int_tokens(tokens: list[str]) -> list[int] | None:
    try:
        return [int(value, 0) for value in tokens]
    except ValueError:
        return None


def _parse_identifier(value: str) -> int | str:
    try:
        return int(value, 0)
    except ValueError:
        return value.upper()


def _setting_target(
    context: str,
    profile_index: str,
    global_settings: dict[str, str],
    pid_profiles: dict[str, dict[str, str]],
    rate_profiles: dict[str, dict[str, str]],
) -> tuple[dict[str, str], str]:
    if context == "pid":
        return pid_profiles.setdefault(profile_index, {}), f"profile:{profile_index}"
    if context == "rate":
        return rate_profiles.setdefault(profile_index, {}), f"rateprofile:{profile_index}"
    return global_settings, "global"


def _mark_setting_categories(name: str, categories: dict[str, bool]) -> None:
    if any(token in name for token in ("serialrx", "rx_spi", "receiver", "rssi_channel")):
        categories["receiver"] = True
    if name.startswith("failsafe_"):
        categories["failsafe"] = True
    if name.startswith(("p_", "i_", "d_", "f_")) or name in {"pid_process_denom"}:
        categories["pid_profile"] = True
    if any(token in name for token in ("rc_rate", "rc_expo", "_srate", "rates_type", "rate_limit")):
        categories["rate_profile"] = True
    if name.startswith("blackbox_"):
        categories["blackbox"] = True
    if any(token in name for token in ("vbat_", "battery_", "current_meter", "ibat_")):
        categories["battery"] = True


def _comparable_cli_values(parsed: dict[str, Any]) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for port in parsed["serial_ports"]:
        identifier = str(port["identifier"])
        values[f"serial_ports.{identifier}"] = {
            "function_mask": port["function_mask"],
            "baud_values": port["baud_values"],
        }
    if parsed["receiver"]["channel_map"]:
        values["receiver.channel_map"] = parsed["receiver"]["channel_map"]
    for aux in parsed["aux_ranges"]:
        values[f"aux_ranges.{aux['index']}"] = {
            key: value for key, value in aux.items() if key not in {"index", "raw"}
        }
    for channel in parsed["rx_failsafe"]:
        values[f"rx_failsafe.{channel['channel_index']}"] = {
            key: value for key, value in channel.items() if key not in {"channel_index", "raw"}
        }
    for name, value in parsed["settings"].items():
        values[f"setting.{name}"] = value
    for profile, settings in parsed["pid_profiles"].items():
        for name, value in settings.items():
            values[f"profile.{profile}.{name}"] = value
    for profile, settings in parsed["rate_profiles"].items():
        for name, value in settings.items():
            values[f"rateprofile.{profile}.{name}"] = value
    return values


def _missing_cli_structures(exports: dict[str, dict[str, Any]]) -> list[str]:
    parsed_values = list(exports.values())
    checks = {
        "serial_ports": any(parsed["serial_ports"] for parsed in parsed_values),
        "receiver.channel_map": any(parsed["receiver"]["channel_map"] for parsed in parsed_values),
        "modes.aux_ranges": any(parsed["aux_ranges"] for parsed in parsed_values),
        "pid_profiles.settings": any(parsed["pid_profiles"] for parsed in parsed_values),
        "rate_profiles.settings": any(parsed["rate_profiles"] for parsed in parsed_values),
    }
    return sorted(name for name, present in checks.items() if not present)


def _read_box_ids(adapter: BetaflightMSPAdapter, errors: list[str]) -> tuple[int, ...]:
    try:
        return adapter.read_box_ids()
    except Exception as exc:
        errors.append(f"box_ids: {exc}")
        return ()


def _read_box_names(adapter: BetaflightMSPAdapter, errors: list[str]) -> tuple[str, ...]:
    try:
        return adapter.read_box_names()
    except Exception as exc:
        errors.append(f"box_names: {exc}")
        return ()


def _pair_box_modes(box_ids: tuple[int, ...], box_names: tuple[str, ...]) -> list[dict[str, Any]]:
    return [
        {"index": index, "permanent_id": permanent_id, "name": name}
        for index, (permanent_id, name) in enumerate(zip_longest(box_ids, box_names))
    ]


def _read_clock_status() -> dict[str, Any]:
    command = [
        "timedatectl", "show", "--property=NTPSynchronized", "--property=TimeUSec",
        "--property=RTCTimeUSec", "--property=Timezone",
    ]
    try:
        lines = subprocess.check_output(command, text=True, stderr=subprocess.DEVNULL, timeout=2.0).splitlines()
        values = dict(line.split("=", 1) for line in lines if "=" in line)
        return {
            "ntp_synchronized": values.get("NTPSynchronized", "").strip().lower() == "yes",
            "time": values.get("TimeUSec", "").strip(),
            "rtc_time": values.get("RTCTimeUSec", "").strip(),
            "rtc_matches_system_date": _clock_dates_match(
                values.get("TimeUSec", ""), values.get("RTCTimeUSec", "")
            ),
            "timezone": values.get("Timezone", "").strip(),
            "error": "",
        }
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        return {
            "ntp_synchronized": False, "time": "", "rtc_time": "", "rtc_matches_system_date": False,
            "timezone": "", "error": str(exc),
        }


def _clock_dates_match(system_time: str, rtc_time: str) -> bool:
    system_date = re.search(r"\b\d{4}-\d{2}-\d{2}\b", system_time)
    rtc_date = re.search(r"\b\d{4}-\d{2}-\d{2}\b", rtc_time)
    return bool(system_date and rtc_date and system_date.group(0) == rtc_date.group(0))


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
    include_kinematics: bool,
    sleep_fn,
    monotonic_fn,
) -> tuple[int, list[str]]:
    fields = (
        "sample", "monotonic_s", "cycle_time_us", "i2c_error_count", "sensor_flags", "mode_flags",
        "profile", "roll_deg", "pitch_deg", "yaw_deg", "vbat_v", "mah_drawn", "rssi",
        "amperage_a", "rc_channels", "gps_fix", "gps_satellites", "gps_hdop",
        "gps_latitude_deg", "gps_longitude_deg", "gps_altitude_m",
        "gps_ground_speed_m_s", "gps_ground_course_deg", "baro_altitude_m",
        "baro_vertical_speed_m_s",
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
                telemetry = (
                    adapter.read_telemetry(
                        include_raw_gps=True,
                        include_altitude=True,
                    )
                    if include_kinematics
                    else adapter.read_telemetry()
                )
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
    gps = telemetry.raw_gps
    altitude = telemetry.altitude
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
        "gps_fix": "" if gps is None else gps.fix,
        "gps_satellites": "" if gps is None else gps.satellites,
        "gps_hdop": "" if gps is None or gps.hdop is None else gps.hdop,
        "gps_latitude_deg": "" if gps is None else gps.latitude_deg,
        "gps_longitude_deg": "" if gps is None else gps.longitude_deg,
        "gps_altitude_m": "" if gps is None else gps.altitude_m,
        "gps_ground_speed_m_s": "" if gps is None else gps.ground_speed_m_s,
        "gps_ground_course_deg": "" if gps is None else gps.ground_course_deg,
        "baro_altitude_m": "" if altitude is None else altitude.altitude_m,
        "baro_vertical_speed_m_s": "" if altitude is None else altitude.vertical_speed_m_s,
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
