#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETTINGS_PATH="${SETTINGS_PATH:-$SCRIPT_DIR/config/airsim_blocks_px4_actor_settings.json}"

python3 "$SCRIPT_DIR/examples/run_airsim_strapdown_vision_png.py" \
  --settings-path "$SETTINGS_PATH" \
  --interceptor Interceptor \
  --intruder IntruderActor \
  --intruder-actor \
  --intruder-actor-name IntruderActor \
  --intruder-actor-asset "${INTRUDER_ACTOR_ASSET:-1M_Cube_Chamfer}" \
  --intruder-actor-scale "${INTRUDER_ACTOR_SCALE:-2.0}" \
  --mesh "${INTRUDER_ACTOR_MESH:-IntruderActor}" \
  --px4-interceptor \
  --enable-motion \
  --duration-s "${DURATION_S:-25}" \
  --rate-hz "${RATE_HZ:-20}" \
  --intruder-speed "${INTRUDER_SPEED:-5}" \
  --speed-ratio "${SPEED_RATIO:-2}" \
  --no-px4-command-join \
  --min-command-duration-s "${MIN_COMMAND_DURATION_S:-0.10}" \
  --command-duration-margin-s "${COMMAND_DURATION_MARGIN_S:-0.05}" \
  --max-command-duration-s "${MAX_COMMAND_DURATION_S:-0.20}" \
  --start-horizontal-range-m "${START_RANGE_M:-100}" \
  --start-lateral-offset-m "${START_LATERAL_M:--20}" \
  --intruder-altitude-offset-m "${ALTITUDE_OFFSET_M:-30}" \
  --intercept-altitude-m "${INTERCEPT_ALTITUDE_M:-50}" \
  --no-los-filter \
  --print-every-n "${PRINT_EVERY_N:-10}" \
  --trajectory-dir "$SCRIPT_DIR/logs/px4_actor" \
  --trajectory-prefix "${TRAJECTORY_PREFIX:-strapdown_px4_actor_demo_$(date +%Y%m%d_%H%M%S)}" \
  "$@"
