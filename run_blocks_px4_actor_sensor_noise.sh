#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETTINGS_PATH="$SCRIPT_DIR/config/airsim_blocks_px4_actor_sensor_noise_settings.json"

if [[ ! -f "$SETTINGS_PATH" ]]; then
  echo "PX4 actor-target sensor-noise AirSim settings not found: $SETTINGS_PATH" >&2
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
  pkill -x px4 || true
  ./run_px4_sitl.sh
  ./run_blocks_px4_actor_sensor_noise.sh
EOF
    exit 1
  fi
fi

if [[ "$#" -eq 0 ]]; then
  set -- -RenderOffscreen -NoSplash -NoVSync -BENCHMARK -FPS=20
fi

exec "$SCRIPT_DIR/run_blocks_nvidia.sh" "-settings=$SETTINGS_PATH" "$@"
