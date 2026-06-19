#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

STAMP="${STAMP:-strict_reset_$(date +%Y%m%d_%H%M%S)}"
RANGES=(${RANGES:-40 50 60 70 80 90 100 110 120 130 140})
ALTITUDE_OFFSET="${ALTITUDE_OFFSET:-20}"
DURATION_S="${DURATION_S:-35}"
RATE_HZ="${RATE_HZ:-20}"
INTRUDER_SPEED="${INTRUDER_SPEED:-5}"
SPEED_RATIO="${SPEED_RATIO:-2}"
INTERCEPT_ALTITUDE_M="${INTERCEPT_ALTITUDE_M:-50}"
NOISE_SEED="${NOISE_SEED:-20260617}"
NOISE_CENTER_PX="${NOISE_CENTER_PX:-3.0}"
NOISE_AREA_RATIO="${NOISE_AREA_RATIO:-0.08}"
INTRUDER_ACTOR_ASSET="${INTRUDER_ACTOR_ASSET:-1M_Cube_Chamfer}"
INTRUDER_ACTOR_SCALE="${INTRUDER_ACTOR_SCALE:-1.0}"
INTRUDER_ACTOR_SCALE_X="${INTRUDER_ACTOR_SCALE_X:-1.0}"
INTRUDER_ACTOR_SCALE_Y="${INTRUDER_ACTOR_SCALE_Y:-1.0}"
INTRUDER_ACTOR_SCALE_Z="${INTRUDER_ACTOR_SCALE_Z:-0.5}"
LOG_DIR="$SCRIPT_DIR/logs/strict_reset"
TRAJECTORY_DIR="$SCRIPT_DIR/logs/strapdown_accuracy"

mkdir -p "$LOG_DIR" "$TRAJECTORY_DIR"

stop_sim() {
  pkill -f 'Blocks/Binaries/Linux/Blocks|Blocks.sh|px4-simulator|PX4-Autopilot.*px4|sitl_run.sh|make px4_sitl_default|cmake --build .*/px4_sitl_default' 2>/dev/null || true
  sleep 2
}

wait_for_px4() {
  local log_file="$1"
  local deadline=$((SECONDS + 90))
  while (( SECONDS < deadline )); do
    if grep -q 'Waiting for simulator to accept connection on TCP port 4560' "$log_file" 2>/dev/null; then
      return 0
    fi
    if ! ps -p "$PX4_PID" >/dev/null 2>&1; then
      echo "PX4 exited before waiting for simulator. See $log_file" >&2
      return 1
    fi
    sleep 1
  done
  echo "Timed out waiting for PX4 TCP 4560. See $log_file" >&2
  return 1
}

wait_for_airsim() {
  local deadline=$((SECONDS + 90))
  while (( SECONDS < deadline )); do
    if python3 - <<'PY' >/dev/null 2>&1
import airsim
c = airsim.MultirotorClient(timeout_value=2)
c.confirmConnection()
vehicles = c.listVehicles()
raise SystemExit(0 if "Interceptor" in vehicles else 1)
PY
    then
      return 0
    fi
    if ! ps -p "$BLOCKS_PID" >/dev/null 2>&1; then
      echo "Blocks exited before AirSim RPC became ready. See $BLOCKS_LOG" >&2
      return 1
    fi
    sleep 1
  done
  echo "Timed out waiting for AirSim RPC. See $BLOCKS_LOG" >&2
  return 1
}

start_stack() {
  local label="$1"
  local settings_kind="$2"
  local run_tag="$3"
  PX4_LOG="$LOG_DIR/px4_${label}_${run_tag}.log"
  BLOCKS_LOG="$LOG_DIR/blocks_${label}_${run_tag}.log"

  stop_sim
  echo "Starting PX4 for ${label} ${run_tag}"
  script -q -f -c "$SCRIPT_DIR/run_px4_sitl.sh" "$PX4_LOG" >/dev/null 2>&1 &
  PX4_PID=$!
  wait_for_px4 "$PX4_LOG"

  echo "Starting Blocks for ${label} ${run_tag}"
  if [[ "$settings_kind" == "sensor_noise" ]]; then
    "$SCRIPT_DIR/run_blocks_px4_actor_sensor_noise.sh" >"$BLOCKS_LOG" 2>&1 &
  else
    "$SCRIPT_DIR/run_blocks_px4_actor.sh" >"$BLOCKS_LOG" 2>&1 &
  fi
  BLOCKS_PID=$!
  wait_for_airsim
}

run_one() {
  local label="$1"
  local settings_kind="$2"
  local range_m="$3"
  shift 3
  local extra_args=("$@")
  local run_tag="r${range_m}_h${ALTITUDE_OFFSET}_${STAMP}"
  local prefix="strapdown_clock1_sitl_${label}_${STAMP}_r${range_m}_h${ALTITUDE_OFFSET}"
  local settings_path
  if [[ "$settings_kind" == "sensor_noise" ]]; then
    settings_path="$SCRIPT_DIR/config/airsim_blocks_px4_actor_sensor_noise_settings.json"
  else
    settings_path="$SCRIPT_DIR/config/airsim_blocks_px4_actor_settings.json"
  fi

  start_stack "$label" "$settings_kind" "$run_tag"
  echo "Running ${label}: range=${range_m}m, altitude_offset=${ALTITUDE_OFFSET}m"
  python3 examples/run_airsim_strapdown_vision_png.py \
    --enable-motion \
    --duration-s "$DURATION_S" \
    --rate-hz "$RATE_HZ" \
    --intruder-speed "$INTRUDER_SPEED" \
    --speed-ratio "$SPEED_RATIO" \
    --intercept-altitude-m "$INTERCEPT_ALTITUDE_M" \
    --intruder-altitude-offset-m "$ALTITUDE_OFFSET" \
    --start-horizontal-range-m "$range_m" \
    --start-lateral-offset-m -20 \
    --trajectory-dir "$TRAJECTORY_DIR" \
    --trajectory-prefix "$prefix" \
    --settings-path "$settings_path" \
    --no-show-window \
    --no-record-preview \
    --no-plot \
    --print-every-n 0 \
    --reset \
    "${extra_args[@]}" \
    --px4-interceptor \
    --intruder IntruderActor \
    --intruder-actor \
    --intruder-actor-name IntruderActor \
    --intruder-actor-asset "$INTRUDER_ACTOR_ASSET" \
    --intruder-actor-scale "$INTRUDER_ACTOR_SCALE" \
    --intruder-actor-scale-x "$INTRUDER_ACTOR_SCALE_X" \
    --intruder-actor-scale-y "$INTRUDER_ACTOR_SCALE_Y" \
    --intruder-actor-scale-z "$INTRUDER_ACTOR_SCALE_Z" \
    --intruder-actor-respawn \
    --mesh 'IntruderActor' \
    --bbox-noise \
    --bbox-center-noise-px "$NOISE_CENTER_PX" \
    --bbox-area-noise-ratio "$NOISE_AREA_RATIO" \
    --bbox-noise-seed "$NOISE_SEED" \
    --climb-timeout-s 90 \
    --no-px4-command-join \
    --px4-command-mode velocity_yaw_rate \
    --min-command-duration-s 0.10 \
    --command-duration-margin-s 0.05 \
    --max-command-duration-s 0.20 \
    --camera-z 0 \
    --camera-pitch-deg 0 \
    --camera-roll-deg 0 \
    --camera-yaw-deg 0
  stop_sim
}

summarize_label() {
  local label="$1"
  local prefix="strapdown_clock1_sitl_${label}_${STAMP}"
  python3 examples/batch_strapdown_accuracy.py \
    --trajectory-dir "$TRAJECTORY_DIR" \
    --summarize-prefix "$prefix"
  python3 examples/export_strapdown_truth_required_load.py \
    --summary-csv "$TRAJECTORY_DIR/${prefix}_summary.csv" \
    --prefix "strapdown_clock1_sitl_${label}_truth_N3_${STAMP}" \
    --output-dir "$TRAJECTORY_DIR/truth_required_load"
}

trap stop_sim EXIT

echo "Strict-reset J/K/L stamp: ${STAMP}"
echo "Ranges: ${RANGES[*]}"
echo "Every case restarts PX4 SITL and Blocks."

for range_m in "${RANGES[@]}"; do
  run_one J sensor_noise "$range_m"
done
summarize_label J

for range_m in "${RANGES[@]}"; do
  run_one K sensor_noise "$range_m" --no-los-filter
done
summarize_label K

for range_m in "${RANGES[@]}"; do
  run_one L no_sensor_noise "$range_m" --no-los-filter
done
summarize_label L

python3 examples/generate_jkl_strapdown_report.py \
  --jk-stamp "$STAMP" \
  --l-stamp "$STAMP"

echo
echo "strict_reset_stamp=${STAMP}"
echo "report=完整方案/JKL_SITL矩形目标噪声对比报告.md"
