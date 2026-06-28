#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETTINGS_PATH="$SCRIPT_DIR/config/airsim_blocks_px4_sitl_settings.json"

if [[ ! -f "$SETTINGS_PATH" ]]; then
  echo "PX4 SITL AirSim settings not found: $SETTINGS_PATH" >&2
  exit 1
fi

if [[ "$#" -eq 0 ]]; then
  set -- -RenderOffscreen -NoSplash -NoVSync -BENCHMARK -FPS=20
fi

exec "$SCRIPT_DIR/run_blocks_nvidia.sh" "-settings=$SETTINGS_PATH" "$@"
