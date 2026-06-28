#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4/PX4-Autopilot}"
PX4_MODEL="${PX4_MODEL:-none_iris}"
PX4_TARGET="${PX4_TARGET:-px4_sitl_default}"
PX4_INSTANCE="${PX4_INSTANCE:-}"

if [[ -z "$PX4_INSTANCE" && -n "${PX4_SIM_TCP_PORT:-}" ]]; then
  if [[ "$PX4_SIM_TCP_PORT" =~ ^[0-9]+$ && "$PX4_SIM_TCP_PORT" -ge 4560 ]]; then
    PX4_INSTANCE="$((PX4_SIM_TCP_PORT - 4560))"
  fi
fi

if [[ ! -d "$PX4_DIR" ]]; then
  cat >&2 <<EOF
PX4-Autopilot not found: $PX4_DIR

Install PX4 SITL first, for example:
  mkdir -p "$HOME/PX4"
  cd "$HOME/PX4"
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
  bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
  cd PX4-Autopilot
  git checkout v1.11.3

Then run this script again. You can also set PX4_DIR=/path/to/PX4-Autopilot.
EOF
  exit 1
fi

if [[ ! -f "$PX4_DIR/Makefile" ]]; then
  echo "PX4_DIR does not look like PX4-Autopilot: $PX4_DIR" >&2
  exit 1
fi

cd "$PX4_DIR"
echo "Starting PX4 SITL from: $PX4_DIR"
echo "Target/model: ${PX4_TARGET} ${PX4_MODEL}"
if [[ -n "$PX4_INSTANCE" && "$PX4_INSTANCE" != "0" ]]; then
  if [[ ! -x "build/${PX4_TARGET}/bin/px4" ]]; then
    echo "PX4 binary not found; building ${PX4_TARGET} first."
    make "$PX4_TARGET"
  fi
  BUILD_DIR="$PX4_DIR/build/$PX4_TARGET"
  ROOTFS="$PX4_DIR/ROMFS/px4fmu_common"
  WORKDIR="$BUILD_DIR/instance_$PX4_INSTANCE"
  mkdir -p "$WORKDIR"
  export PX4_SIM_MODEL="${PX4_SIM_MODEL:-iris}"
  echo "PX4 instance: $PX4_INSTANCE"
  echo "Expected AirSim simulator TCP port: $((4560 + PX4_INSTANCE))"
  echo "Keep this terminal open while AirSim Blocks is running."
  cd "$WORKDIR"
  exec "$BUILD_DIR/bin/px4" -i "$PX4_INSTANCE" -d "$ROOTFS" -s etc/init.d-posix/rcS
fi

echo "Keep this terminal open; start AirSim Blocks after PX4 prints that it is waiting on TCP port 4560."
exec make "$PX4_TARGET" "$PX4_MODEL"
