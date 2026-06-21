#!/usr/bin/env python3
"""Minimal AirSim/PX4 HIL arming and Offboard probe.

This intentionally avoids the vision guidance pipeline. It is meant to answer:

* Can AirSim RPC talk to the PX4-backed vehicle?
* Does programmatic arming work without an RC?
* Does PX4 publish finite local position/velocity estimates?
* Does moveByVelocityAsync keep Offboard long enough to move the vehicle?
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Any


def _finite(value: float) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _fmt(value: float) -> str:
    if not _finite(value):
        return "nan"
    return f"{float(value):.3f}"


def _state_row(client: Any, vehicle: str, t0: float, label: str) -> dict[str, float | str]:
    state = client.getMultirotorState(vehicle_name=vehicle)
    kin = state.kinematics_estimated
    pos = kin.position
    vel = kin.linear_velocity
    acc = kin.linear_acceleration
    row = {
        "label": label,
        "t": time.monotonic() - t0,
        "landed": int(getattr(state, "landed_state", -1)),
        "px": float(pos.x_val),
        "py": float(pos.y_val),
        "pz": float(pos.z_val),
        "vx": float(vel.x_val),
        "vy": float(vel.y_val),
        "vz": float(vel.z_val),
        "ax": float(acc.x_val),
        "ay": float(acc.y_val),
        "az": float(acc.z_val),
    }
    print(
        f"{row['label']},t={row['t']:.2f},landed={row['landed']},"
        f"pos=[{_fmt(row['px'])},{_fmt(row['py'])},{_fmt(row['pz'])}],"
        f"vel=[{_fmt(row['vx'])},{_fmt(row['vy'])},{_fmt(row['vz'])}],"
        f"acc=[{_fmt(row['ax'])},{_fmt(row['ay'])},{_fmt(row['az'])}]"
    )
    return row


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", default="Interceptor")
    parser.add_argument("--observe-s", type=float, default=8.0)
    parser.add_argument("--command-s", type=float, default=6.0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--vz", type=float, default=-0.5, help="NED vertical speed; negative climbs.")
    parser.add_argument("--yaw-rate-deg", type=float, default=0.0)
    parser.add_argument("--force-arm", action="store_true", help="Arm through MAV_CMD_COMPONENT_ARM_DISARM param2=21196.")
    parser.add_argument("--mavlink-url", default="/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00")
    parser.add_argument("--mavlink-baud", type=int, default=115200)
    parser.add_argument("--mavlink-only", action="store_true", help="Only send MAVLink arm/disarm commands; do not connect to AirSim.")
    parser.add_argument("--no-arm", action="store_true")
    parser.add_argument("--no-api-control", action="store_true")
    parser.add_argument("--disarm-at-end", action=argparse.BooleanOptionalAction, default=False)
    args = parser.parse_args()

    if args.rate_hz <= 0:
        raise SystemExit("--rate-hz must be positive")

    if args.mavlink_only:
        if not args.force_arm:
            raise SystemExit("--mavlink-only currently requires --force-arm")
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise SystemExit("Install pymavlink first.") from exc

        master = mavutil.mavlink_connection(
            args.mavlink_url,
            baud=args.mavlink_baud,
            source_system=251,
            source_component=1,
            autoreconnect=False,
        )
        heartbeat = master.wait_heartbeat(timeout=8.0)
        if heartbeat is None:
            raise SystemExit("mavlink_only_error=no_mavlink_heartbeat")
        print(f"mavlink_only_heartbeat={heartbeat}")
        for armed in (True, False):
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1 if armed else 0,
                21196,
                0,
                0,
                0,
                0,
                0,
            )
            ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3.0)
            print(f"mavlink_only_{'arm' if armed else 'disarm'}_ack={ack}")
        return

    try:
        import airsim
    except ImportError as exc:
        raise SystemExit("Install the AirSim Python package first.") from exc

    client = airsim.MultirotorClient()
    client.confirmConnection()
    print(f"vehicles={client.listVehicles()}")

    t0 = time.monotonic()
    observe_deadline = t0 + max(0.0, args.observe_s)
    rows = []
    while time.monotonic() < observe_deadline:
        rows.append(_state_row(client, args.vehicle, t0, "observe"))
        time.sleep(1.0)

    finite_z = any(_finite(float(row["pz"])) for row in rows)
    if not finite_z:
        print("warning=no_finite_local_z_before_command")

    if not args.no_api_control:
        try:
            client.enableApiControl(True, vehicle_name=args.vehicle)
            print("api_control=enabled")
        except Exception as exc:
            print(f"api_control_error={exc!r}")

    if not args.no_arm:
        if args.force_arm:
            try:
                from pymavlink import mavutil

                master = mavutil.mavlink_connection(
                    args.mavlink_url,
                    baud=args.mavlink_baud,
                    source_system=251,
                    source_component=1,
                    autoreconnect=False,
                )
                heartbeat = master.wait_heartbeat(timeout=5.0)
                if heartbeat is None:
                    print("force_arm_error=no_mavlink_heartbeat")
                else:
                    master.mav.command_long_send(
                        master.target_system,
                        master.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,
                        1,
                        21196,
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
                    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3.0)
                    print(f"force_arm_ack={ack}")
            except Exception as exc:
                print(f"force_arm_error={exc!r}")
        else:
            try:
                result = client.armDisarm(True, vehicle_name=args.vehicle)
                print(f"arm_result={result}")
            except Exception as exc:
                print(f"arm_error={exc!r}")

    dt = max(0.02, 1.0 / args.rate_hz)
    command_duration = max(0.05, min(0.5, dt * 1.5))
    yaw_mode = airsim.YawMode(is_rate=True, yaw_or_rate=float(args.yaw_rate_deg))
    deadline = time.monotonic() + max(0.0, args.command_s)
    frame = 0
    while time.monotonic() < deadline:
        try:
            client.moveByVelocityAsync(
                float(args.vx),
                float(args.vy),
                float(args.vz),
                duration=command_duration,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=yaw_mode,
                vehicle_name=args.vehicle,
            )
        except Exception as exc:
            print(f"command_error frame={frame} error={exc!r}")
        if frame % max(1, int(args.rate_hz)) == 0:
            _state_row(client, args.vehicle, t0, "command")
        frame += 1
        time.sleep(dt)

    try:
        client.moveByVelocityAsync(0.0, 0.0, 0.0, duration=0.2, vehicle_name=args.vehicle).join()
        print("stop_command=sent")
    except Exception as exc:
        print(f"stop_command_error={exc!r}")

    _state_row(client, args.vehicle, t0, "final")

    if args.disarm_at_end:
        try:
            result = client.armDisarm(False, vehicle_name=args.vehicle)
            print(f"disarm_result={result}")
        except Exception as exc:
            print(f"disarm_error={exc!r}")


if __name__ == "__main__":
    main()
