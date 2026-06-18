#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETTINGS_PATH="$SCRIPT_DIR/config/airsim_blocks_px4_sitl_dual_settings.json"

if [[ ! -f "$SETTINGS_PATH" ]]; then
  echo "Dual PX4 AirSim settings not found: $SETTINGS_PATH" >&2
  exit 1
fi

if command -v ss >/dev/null 2>&1; then
  PORT_USERS="$(ss -H -ltnp 2>/dev/null | awk '$4 ~ /:4560$/ || $4 ~ /:4561$/ {print}' || true)"
  if [[ -n "$PORT_USERS" ]]; then
    cat >&2 <<EOF
PX4 TCP port 4560 or 4561 is already in use, so Blocks cannot bind it.

$PORT_USERS

Stop existing Blocks/PX4 processes before starting another dual-SITL session.
EOF
    exit 1
  fi
fi

if [[ "$#" -eq 0 ]]; then
  set -- -RenderOffscreen -NoSplash -NoVSync -BENCHMARK -FPS=20
fi

exec "$SCRIPT_DIR/run_blocks_nvidia.sh" "-settings=$SETTINGS_PATH" "$@"
