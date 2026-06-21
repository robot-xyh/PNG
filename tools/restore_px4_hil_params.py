#!/usr/bin/env python3
"""Restore PX4 parameters used by the AirSim HIL bench tests.

This script writes parameters through MAVLink using PX4 bytewise encoding for
integer parameters. It is intended for recovery after HIL estimator experiments
or after QGC parameter edits.
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ParamSpec:
    value: float | int
    reboot_required: bool = False
    optional: bool = False


PX4_HIL_PARAMS: dict[str, ParamSpec] = {
    "SYS_AUTOSTART": ParamSpec(1001, reboot_required=True),
    "SYS_HITL": ParamSpec(1, reboot_required=True),
    "SYS_MC_EST_GROUP": ParamSpec(2, reboot_required=True),
    "SYS_HAS_BARO": ParamSpec(0, reboot_required=True),
    "SYS_HAS_MAG": ParamSpec(0, reboot_required=True),
    "MAV_USEHILGPS": ParamSpec(1),
    "COM_RC_IN_MODE": ParamSpec(1),
    "COM_RC_LOSS_T": ParamSpec(60.0),
    "NAV_RCL_ACT": ParamSpec(0),
    "NAV_DLL_ACT": ParamSpec(0),
    "COM_OBL_ACT": ParamSpec(1),
    "COM_OBL_RC_ACT": ParamSpec(5),
    "COM_OF_LOSS_T": ParamSpec(5.0),
    "COM_DISARM_PRFLT": ParamSpec(0.0),
    # Bench HIL only: relax IMU consistency/range checks caused by mixed HIL + board sensors.
    "COM_ARM_IMU_ACC": ParamSpec(5.0),
    "COM_ARM_IMU_GYR": ParamSpec(2.0),
    "EKF2_AID_MASK": ParamSpec(1, reboot_required=True),
    "EKF2_HGT_MODE": ParamSpec(1, reboot_required=True),
    "EKF2_MAG_TYPE": ParamSpec(5, reboot_required=True),
    "EKF2_GPS_CHECK": ParamSpec(0),
    "CBRK_SUPPLY_CHK": ParamSpec(894281, optional=True),
    "CBRK_USB_CHK": ParamSpec(197848, optional=True),
    "CBRK_IO_SAFETY": ParamSpec(22027, optional=True),
    "CBRK_VELPOSERR": ParamSpec(201607),
}


def _param_name(param_id: Any) -> str:
    if isinstance(param_id, bytes):
        return param_id.decode(errors="ignore").rstrip("\x00")
    return str(param_id).rstrip("\x00")


def _is_int_type(mavutil: Any, param_type: int) -> bool:
    mavlink = mavutil.mavlink
    return param_type in {
        mavlink.MAV_PARAM_TYPE_UINT8,
        mavlink.MAV_PARAM_TYPE_INT8,
        mavlink.MAV_PARAM_TYPE_UINT16,
        mavlink.MAV_PARAM_TYPE_INT16,
        mavlink.MAV_PARAM_TYPE_UINT32,
        mavlink.MAV_PARAM_TYPE_INT32,
    }


def _is_uint_type(mavutil: Any, param_type: int) -> bool:
    mavlink = mavutil.mavlink
    return param_type in {
        mavlink.MAV_PARAM_TYPE_UINT8,
        mavlink.MAV_PARAM_TYPE_UINT16,
        mavlink.MAV_PARAM_TYPE_UINT32,
    }


def _pack_param_value(mavutil: Any, param_type: int, value: float | int) -> float:
    if not _is_int_type(mavutil, param_type):
        return float(value)
    if _is_uint_type(mavutil, param_type):
        raw = struct.pack("<I", int(value) & 0xFFFFFFFF)
    else:
        raw = struct.pack("<i", int(value))
    return struct.unpack("<f", raw)[0]


def _decode_param_value(mavutil: Any, param_type: int, param_value: float) -> float | int:
    if not _is_int_type(mavutil, param_type):
        return float(param_value)
    raw = struct.pack("<f", float(param_value))
    if _is_uint_type(mavutil, param_type):
        return struct.unpack("<I", raw)[0]
    return struct.unpack("<i", raw)[0]


def _request_param(master: Any, mavutil: Any, name: str, timeout_s: float = 2.0) -> Any | None:
    master.mav.param_request_read_send(master.target_system, master.target_component, name.encode(), -1)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg and _param_name(msg.param_id) == name:
            return msg
    return None


def _set_param(master: Any, mavutil: Any, name: str, value: float | int, timeout_s: float) -> bool:
    current = None
    for _ in range(4):
        current = _request_param(master, mavutil, name, timeout_s=timeout_s)
        if current is not None:
            break
    if current is None:
        print(f"{name}=missing")
        return False

    packed_value = _pack_param_value(mavutil, current.param_type, value)
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode(),
        packed_value,
        current.param_type,
    )

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if not msg or _param_name(msg.param_id) != name:
            continue
        decoded = _decode_param_value(mavutil, msg.param_type, msg.param_value)
        ok = abs(float(decoded) - float(value)) < 1e-4
        status = "ok" if ok else "mismatch"
        print(f"{name}={decoded} target={value} type={msg.param_type} {status}")
        return ok

    print(f"{name}=no_ack target={value}")
    return False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout-s", type=float, default=3.0)
    parser.add_argument("--reboot", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--read-only", action="store_true")
    args = parser.parse_args()

    try:
        from pymavlink import mavutil
    except ImportError as exc:
        raise SystemExit("Install pymavlink first.") from exc

    master = mavutil.mavlink_connection(
        args.port,
        baud=args.baud,
        source_system=250,
        source_component=1,
        autoreconnect=False,
    )
    heartbeat = master.wait_heartbeat(timeout=max(1.0, args.timeout_s * 4.0))
    if heartbeat is None:
        raise SystemExit(f"No MAVLink heartbeat from {args.port} at {args.baud} baud.")

    print(f"heartbeat={heartbeat}")
    failed = 0
    for name, spec in PX4_HIL_PARAMS.items():
        if args.read_only:
            msg = _request_param(master, mavutil, name, timeout_s=args.timeout_s)
            if msg is None:
                print(f"{name}=missing")
                if not spec.optional:
                    failed += 1
                continue
            decoded = _decode_param_value(mavutil, msg.param_type, msg.param_value)
            print(f"{name}={decoded} type={msg.param_type}")
        elif not _set_param(master, mavutil, name, spec.value, args.timeout_s) and not spec.optional:
            failed += 1

    if failed:
        raise SystemExit(f"Failed parameters: {failed}")

    if args.reboot:
        print("rebooting_px4=true")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )


if __name__ == "__main__":
    main()
