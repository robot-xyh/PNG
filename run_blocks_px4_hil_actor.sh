#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETTINGS_PATH="${PX4_HIL_SETTINGS_PATH:-$SCRIPT_DIR/config/airsim_blocks_px4_hil_actor_settings.json}"
SERIAL_PORT="${PX4_HIL_SERIAL_PORT:-/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00}"

if [[ ! -f "$SETTINGS_PATH" ]]; then
  echo "PX4 HIL actor-target AirSim settings not found: $SETTINGS_PATH" >&2
  exit 1
fi

if [[ ! -e "$SERIAL_PORT" ]]; then
  echo "PX4 serial port not found: $SERIAL_PORT" >&2
  echo "Set PX4_HIL_SERIAL_PORT=/dev/ttyACM* or update $SETTINGS_PATH." >&2
  exit 1
fi

if [[ "$#" -eq 0 ]]; then
  set -- -NoSplash -nohmd -NoVSync -BENCHMARK -FPS=20 -windowed -ResX=640 -ResY=480
fi

echo "Starting AirSim Blocks HIL actor setup."
echo "Settings: $SETTINGS_PATH"
echo "PX4 serial port expected by settings: $SERIAL_PORT"
exec "$SCRIPT_DIR/run_blocks_nvidia.sh" "-settings=$SETTINGS_PATH" "$@"
