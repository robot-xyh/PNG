#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4/PX4-Autopilot}"

if [[ ! -d "$PX4_DIR" || ! -f "$PX4_DIR/Makefile" ]]; then
  echo "PX4_DIR does not look like PX4-Autopilot: $PX4_DIR" >&2
  exit 1
fi

cd "$PX4_DIR"
if [[ ! -x build/px4_sitl_default/bin/px4 ]]; then
  echo "PX4 SITL binary not found; building px4_sitl_default first." >&2
  make px4_sitl_default
fi

BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
ROOTFS="$PX4_DIR/ROMFS/px4fmu_common"

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup INT TERM EXIT

pkill -x px4 2>/dev/null || true
rm -f /tmp/px4_lock-* /tmp/px4-sock-*

echo "Starting two PX4 SITL instances for AirSim from: $PX4_DIR"
echo "Expected simulator TCP ports: 4560 and 4561."
export PX4_SIM_MODEL=iris

PIDS=()
for instance in 0 1; do
  workdir="$BUILD_DIR/instance_$instance"
  mkdir -p "$workdir"
  (
    cd "$workdir"
    exec "$BUILD_DIR/bin/px4" -i "$instance" -d "$ROOTFS" -s etc/init.d-posix/rcS
  ) &
  PIDS+=("$!")
  echo "PX4 instance ${instance} pid=${PIDS[-1]}"
  sleep 1
done

echo "Keep this terminal open while AirSim Blocks is running."
wait -n "${PIDS[@]}"
