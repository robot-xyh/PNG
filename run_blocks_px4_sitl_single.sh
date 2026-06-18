#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETTINGS_PATH="$SCRIPT_DIR/config/airsim_blocks_px4_sitl_single_settings.json"

if [[ ! -f "$SETTINGS_PATH" ]]; then
  echo "PX4 single-vehicle AirSim settings not found: $SETTINGS_PATH" >&2
  exit 1
fi

if command -v ss >/dev/null 2>&1; then
  PORT_USERS="$(ss -H -ltnp 2>/dev/null | awk '$4 ~ /:4560$/ {print}' || true)"
  if [[ -n "$PORT_USERS" ]]; then
    cat >&2 <<EOF
PX4 TCP port 4560 is already in use, so Blocks cannot bind it.

$PORT_USERS

Close the existing Blocks/AirSim instance before starting another one. If a previous
run crashed, stop both PX4 and Blocks, then restart:
  pkill -f 'Blocks/Binaries/Linux/Blocks' || true
  pkill -f 'px4_sitl_default|PX4-Autopilot.*px4|px4-simulator' || true
  ./run_px4_sitl.sh
  ./run_blocks_px4_sitl_single.sh
EOF
    exit 1
  fi
fi

if [[ "$#" -eq 0 ]]; then
  set -- -RenderOffscreen -NoSplash -NoVSync -BENCHMARK -FPS=20
fi

exec "$SCRIPT_DIR/run_blocks_nvidia.sh" "-settings=$SETTINGS_PATH" "$@"
