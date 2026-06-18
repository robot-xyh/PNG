#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETTINGS_PATH="$SCRIPT_DIR/config/airsim_blocks_px4_sitl_settings.json"

if [[ ! -f "$SETTINGS_PATH" ]]; then
  echo "PX4 mixed AirSim settings not found: $SETTINGS_PATH" >&2
  exit 1
fi

python3 "$SCRIPT_DIR/examples/run_airsim_truth_png.py" \
  --interceptor Interceptor \
  --intruder Intruder \
  --settings-path "$SETTINGS_PATH" \
  --px4-interceptor \
  --enable-motion \
  --duration-s "${PX4_TRUTH_DURATION_S:-25}" \
  --rate-hz "${PX4_TRUTH_RATE_HZ:-20}" \
  --intruder-speed "${PX4_TRUTH_INTRUDER_SPEED:-5}" \
  --speed-ratio "${PX4_TRUTH_SPEED_RATIO:-2.0}" \
  --navigation-constant "${PX4_TRUTH_NAVIGATION_CONSTANT:-3.0}" \
  --max-accel "${PX4_TRUTH_MAX_ACCEL:-10.0}" \
  --intercept-altitude-m "${PX4_TRUTH_INTERCEPT_ALTITUDE_M:-50}" \
  --intruder-altitude-offset-m "${PX4_TRUTH_INTRUDER_ALTITUDE_OFFSET_M:-0}" \
  --start-horizontal-range-m "${PX4_TRUTH_START_RANGE_M:-80}" \
  --start-lateral-offset-m "${PX4_TRUTH_START_LATERAL_M:--20}" \
  --climb-speed "${PX4_TRUTH_CLIMB_SPEED:-3}" \
  --climb-timeout-s "${PX4_TRUTH_CLIMB_TIMEOUT_S:-80}" \
  --settle-s "${PX4_TRUTH_SETTLE_S:-3}" \
  --settle-speed "${PX4_TRUTH_SETTLE_SPEED:-0.8}" \
  --settle-timeout-s "${PX4_TRUTH_SETTLE_TIMEOUT_S:-12}" \
  --vertical-kp "${PX4_TRUTH_VERTICAL_KP:-1.0}" \
  --vertical-speed-limit "${PX4_TRUTH_VERTICAL_SPEED_LIMIT:-2.0}" \
  --trajectory-prefix "${PX4_TRUTH_TRAJECTORY_PREFIX:-px4_truth_png_demo}" \
  --print-every-n "${PX4_TRUTH_PRINT_EVERY_N:-5}" \
  "$@"
