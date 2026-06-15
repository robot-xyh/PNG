#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETTINGS_PATH="$SCRIPT_DIR/config/airsim_blocks_recording_settings.json"

if [[ ! -f "$SETTINGS_PATH" ]]; then
  echo "Recording settings not found: $SETTINGS_PATH" >&2
  exit 1
fi

ARGS=(
  "-settings=$SETTINGS_PATH"
  "-nohmd"
  "-windowed"
  "-ResX=${AIRSIM_RECORDING_RESX:-1600}"
  "-ResY=${AIRSIM_RECORDING_RESY:-900}"
)

echo "Starting Blocks recording scene with settings: $SETTINGS_PATH"
exec "$SCRIPT_DIR/run_blocks_nvidia.sh" "${ARGS[@]}" "$@"
