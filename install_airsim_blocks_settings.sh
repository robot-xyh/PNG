#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE="$SCRIPT_DIR/config/airsim_blocks_settings.json"
TARGET="${AIRSIM_SETTINGS:-$HOME/Documents/AirSim/settings.json}"
TARGET_DIR="$(dirname "$TARGET")"

if [[ ! -f "$SOURCE" ]]; then
  echo "Settings template not found: $SOURCE" >&2
  exit 1
fi

mkdir -p "$TARGET_DIR"
if [[ -f "$TARGET" ]]; then
  BACKUP="$TARGET.backup.$(date +%Y%m%d-%H%M%S)"
  cp "$TARGET" "$BACKUP"
  echo "Backed up existing AirSim settings to: $BACKUP"
fi

cp "$SOURCE" "$TARGET"
echo "Installed AirSim Blocks settings to: $TARGET"
echo "Restart Blocks before running examples/run_airsim_blocks.py."
