#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

STAMP="${STAMP:-body_rate_diag_$(date +%Y%m%d_%H%M%S)}"
RANGES=(${RANGES:-50 60 70 80 90 100})
EXPERIMENT_GROUPS=(${EXPERIMENT_GROUPS:-A_truth B_gimbal_detect C_strapdown_detect D_strapdown_detect_high_authority E_strapdown_detect_high_authority_20hz})
LAWS=(${LAWS:-TTC VM})
ALTITUDE_OFFSET="${ALTITUDE_OFFSET:-20}"
INTERCEPT_ALTITUDE_M="${INTERCEPT_ALTITUDE_M:-50}"
INTRUDER_SPEED="${INTRUDER_SPEED:-5}"
SPEED_RATIO="${SPEED_RATIO:-2}"
NAVIGATION_CONSTANT="${NAVIGATION_CONSTANT:-3.0}"
RATE_HZ="${RATE_HZ:-8}"
RATE_HZ_FAST="${RATE_HZ_FAST:-20}"
CASE_TIMEOUT_S="${CASE_TIMEOUT_S:-180}"
SETTINGS_BASE="${SETTINGS_BASE:-$SCRIPT_DIR/config/airsim_blocks_px4_actor_settings.json}"
SETTINGS_HIGH_AUTHORITY="${SETTINGS_HIGH_AUTHORITY:-$SCRIPT_DIR/config/airsim_blocks_px4_actor_high_authority_settings.json}"
LOG_DIR="$SCRIPT_DIR/logs/body_rate_diagnostic_stack"
TRAJECTORY_DIR="${TRAJECTORY_DIR:-$SCRIPT_DIR/logs/body_rate_diagnostic}"
REPORT_PATH="${REPORT_PATH:-$SCRIPT_DIR/完整方案/BodyRate_五组诊断实验报告.md}"
ASSET_DIR="${ASSET_DIR:-$SCRIPT_DIR/完整方案/assets/BodyRate_五组诊断实验报告}"

INTRUDER_ACTOR_ASSET="${INTRUDER_ACTOR_ASSET:-Quadrotor1}"
INTRUDER_ACTOR_SCALE="${INTRUDER_ACTOR_SCALE:-1.0}"
START_LATERAL_OFFSET_M="${START_LATERAL_OFFSET_M:--20}"

BODY_RATE_CONTROL_PROFILE="${BODY_RATE_CONTROL_PROFILE:-legacy}"
BODY_RATE_MAX_TILT_DEG="${BODY_RATE_MAX_TILT_DEG:-20}"
BODY_RATE_MAX_ROLL_RATE_DEG="${BODY_RATE_MAX_ROLL_RATE_DEG:-60}"
BODY_RATE_MAX_PITCH_RATE_DEG="${BODY_RATE_MAX_PITCH_RATE_DEG:-60}"
BODY_RATE_ATTITUDE_P="${BODY_RATE_ATTITUDE_P:-4.0}"
BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2="${BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2:-18}"
BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2="${BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2:-6}"
BODY_RATE_MAX_THRUST="${BODY_RATE_MAX_THRUST:-0.95}"

HIGH_BODY_RATE_MAX_TILT_DEG="${HIGH_BODY_RATE_MAX_TILT_DEG:-35}"
HIGH_BODY_RATE_MAX_ROLL_RATE_DEG="${HIGH_BODY_RATE_MAX_ROLL_RATE_DEG:-120}"
HIGH_BODY_RATE_MAX_PITCH_RATE_DEG="${HIGH_BODY_RATE_MAX_PITCH_RATE_DEG:-120}"
HIGH_MAX_YAW_RATE_DEG="${HIGH_MAX_YAW_RATE_DEG:-60}"
HIGH_BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2="${HIGH_BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2:-24}"
HIGH_BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2="${HIGH_BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2:-8}"
HIGH_BODY_RATE_MAX_THRUST="${HIGH_BODY_RATE_MAX_THRUST:-1.0}"

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
  local env_path="${1:-}"
  local deadline=$((SECONDS + 90))
  while (( SECONDS < deadline )); do
    if [[ -n "$env_path" && -f "$env_path" ]]; then
      set -a
      # shellcheck disable=SC1090
      source "$env_path"
      set +a
    fi
    if python3 - <<'PY' >/dev/null 2>&1
import os
import airsim
host = os.environ.get("AIRSIM_RPC_HOST", "")
port = int(os.environ.get("AIRSIM_RPC_PORT", "41451"))
c = airsim.MultirotorClient(ip=host, port=port, timeout_value=2)
c.confirmConnection()
raise SystemExit(0 if "Interceptor" in c.listVehicles() else 1)
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
  local group="$1"
  local range_m="$2"
  local settings_path="$3"
  local tag="${group}_r${range_m}_${STAMP}"
  PX4_LOG="$LOG_DIR/px4_${tag}.log"
  BLOCKS_LOG="$LOG_DIR/blocks_${tag}.log"
  AIRSIM_RUNTIME_ENV="$LOG_DIR/airsim_${tag}.env"
  CURRENT_AIRSIM_SETTINGS_PATH="$settings_path"
  stop_sim
  echo "Starting PX4 SITL for ${tag}"
  script -q -f -c "$SCRIPT_DIR/run_px4_sitl.sh" "$PX4_LOG" >/dev/null 2>&1 &
  PX4_PID=$!
  wait_for_px4 "$PX4_LOG"
  echo "Starting Blocks for ${tag} with settings=${settings_path}"
  AIRSIM_PORT_ENV_PATH="$AIRSIM_RUNTIME_ENV" AIRSIM_INSTANCE_LABEL="$tag" SETTINGS_PATH="$settings_path" "$SCRIPT_DIR/run_blocks_px4_actor.sh" >"$BLOCKS_LOG" 2>&1 &
  BLOCKS_PID=$!
  wait_for_airsim "$AIRSIM_RUNTIME_ENV"
  if [[ -f "$AIRSIM_RUNTIME_ENV" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$AIRSIM_RUNTIME_ENV"
    set +a
    CURRENT_AIRSIM_SETTINGS_PATH="${AIRSIM_SETTINGS_PATH_RESOLVED:-$settings_path}"
    echo "Resolved AirSim RPC: ${AIRSIM_RPC_HOST:-127.0.0.1}:${AIRSIM_RPC_PORT:-41451}"
    echo "Resolved AirSim settings: $CURRENT_AIRSIM_SETTINGS_PATH"
  fi
}

duration_for_range() {
  python3 - "$1" <<'PY'
import sys
r = float(sys.argv[1])
print(f"{max(24.0, min(42.0, r * 0.36 + 8.0)):.1f}")
PY
}

law_args() {
  case "$1" in
    TTC) echo "--guidance-law ttc_png" ;;
    VM) echo "--guidance-law fixed_vm_png --navigation-constant $NAVIGATION_CONSTANT" ;;
    *) echo "unknown law $1" >&2; return 1 ;;
  esac
}

common_actor_args=(
  --intruder Intruder
  --intruder-actor
  --intruder-actor-name IntruderActor
  --intruder-actor-asset "$INTRUDER_ACTOR_ASSET"
  --intruder-actor-scale "$INTRUDER_ACTOR_SCALE"
  --intruder-actor-respawn
  --intruder-speed "$INTRUDER_SPEED"
  --speed-ratio "$SPEED_RATIO"
  --intercept-altitude-m "$INTERCEPT_ALTITUDE_M"
  --intruder-altitude-offset-m "$ALTITUDE_OFFSET"
  --start-lateral-offset-m "$START_LATERAL_OFFSET_M"
  --px4-interceptor
  --no-px4-command-join
  --reset
  --enable-motion
)

body_rate_args_base=(
  --guidance-output-mode accel_body_rate
  --px4-command-mode mavlink_body_rate
  --body-rate-control-profile "$BODY_RATE_CONTROL_PROFILE"
  --body-rate-max-tilt-deg "$BODY_RATE_MAX_TILT_DEG"
  --body-rate-max-roll-rate-deg "$BODY_RATE_MAX_ROLL_RATE_DEG"
  --body-rate-max-pitch-rate-deg "$BODY_RATE_MAX_PITCH_RATE_DEG"
  --body-rate-attitude-p "$BODY_RATE_ATTITUDE_P"
  --body-rate-total-accel-limit-mps2 "$BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2"
  --body-rate-speed-hold-max-accel-mps2 "$BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2"
  --body-rate-max-thrust "$BODY_RATE_MAX_THRUST"
  --max-guidance-accel-mps2 15
  --ttc-soft-guidance
)

body_rate_args_high=(
  --guidance-output-mode accel_body_rate
  --px4-command-mode mavlink_body_rate
  --body-rate-control-profile "$BODY_RATE_CONTROL_PROFILE"
  --body-rate-max-tilt-deg "$HIGH_BODY_RATE_MAX_TILT_DEG"
  --body-rate-max-roll-rate-deg "$HIGH_BODY_RATE_MAX_ROLL_RATE_DEG"
  --body-rate-max-pitch-rate-deg "$HIGH_BODY_RATE_MAX_PITCH_RATE_DEG"
  --body-rate-attitude-p "$BODY_RATE_ATTITUDE_P"
  --body-rate-total-accel-limit-mps2 "$HIGH_BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2"
  --body-rate-speed-hold-max-accel-mps2 "$HIGH_BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2"
  --body-rate-max-thrust "$HIGH_BODY_RATE_MAX_THRUST"
  --max-yaw-rate-deg "$HIGH_MAX_YAW_RATE_DEG"
  --max-guidance-accel-mps2 15
  --ttc-soft-guidance
)

run_case() {
  local group="$1"
  local law="$2"
  local range_m="$3"
  local duration_s
  duration_s="$(duration_for_range "$range_m")"
  local prefix="body_rate_diag_${group}_${law}_${STAMP}_r${range_m}_h${ALTITUDE_OFFSET}"
  local settings_path="$SETTINGS_BASE"
  local rate_hz="$RATE_HZ"
  local min_command_duration="0.12"
  local command_margin="0.04"
  local max_command_duration="0.25"
  local script_path=""
  local detector_args=()
  local control_args=("${body_rate_args_base[@]}")

  case "$group" in
    A_truth)
      script_path="examples/run_airsim_truth_png.py"
      ;;
    B_gimbal_detect)
      script_path="examples/run_airsim_gimbal_vision_png.py"
      detector_args=(--detector-source airsim --mesh IntruderActor --no-show-window --no-record-preview)
      ;;
    C_strapdown_detect)
      script_path="examples/run_airsim_strapdown_vision_png.py"
      detector_args=(--detector-source airsim --mesh IntruderActor --no-show-window --no-record-preview --no-los-filter)
      ;;
    D_strapdown_detect_high_authority)
      script_path="examples/run_airsim_strapdown_vision_png.py"
      settings_path="$SETTINGS_HIGH_AUTHORITY"
      control_args=("${body_rate_args_high[@]}")
      detector_args=(--detector-source airsim --mesh IntruderActor --no-show-window --no-record-preview --no-los-filter)
      ;;
    E_strapdown_detect_high_authority_20hz)
      script_path="examples/run_airsim_strapdown_vision_png.py"
      settings_path="$SETTINGS_HIGH_AUTHORITY"
      control_args=("${body_rate_args_high[@]}")
      detector_args=(--detector-source airsim --mesh IntruderActor --no-show-window --no-record-preview --no-los-filter)
      rate_hz="$RATE_HZ_FAST"
      min_command_duration="0.05"
      command_margin="0.02"
      max_command_duration="0.12"
      ;;
    *)
      echo "Unknown group: $group" >&2
      return 1
      ;;
  esac

  start_stack "$group" "$range_m" "$settings_path"
  echo "Running group=${group}, law=${law}, range=${range_m}m, duration=${duration_s}s"
  local rc=0
  read -r -a guidance_args <<<"$(law_args "$law")"
  timeout --kill-after=10s "$CASE_TIMEOUT_S" \
  python3 "$script_path" \
    "${common_actor_args[@]}" \
    "${guidance_args[@]}" \
    "${control_args[@]}" \
    "${detector_args[@]}" \
    --duration-s "$duration_s" \
    --rate-hz "$rate_hz" \
    --start-horizontal-range-m "$range_m" \
    --trajectory-dir "$TRAJECTORY_DIR" \
    --trajectory-prefix "$prefix" \
    --settings-path "${CURRENT_AIRSIM_SETTINGS_PATH:-$settings_path}" \
    --min-command-duration-s "$min_command_duration" \
    --command-duration-margin-s "$command_margin" \
    --max-command-duration-s "$max_command_duration" \
    --no-plot \
    --print-every-n 0 || rc=$?
  stop_sim
  if [[ "$rc" -ne 0 ]]; then
    echo "case_failed group=${group} law=${law} range=${range_m} rc=${rc}" >&2
  fi
  return 0
}

trap stop_sim EXIT

echo "body_rate_diagnostic_stamp=${STAMP}"
echo "groups=${EXPERIMENT_GROUPS[*]}"
echo "laws=${LAWS[*]}"
echo "ranges=${RANGES[*]}"

for group in "${EXPERIMENT_GROUPS[@]}"; do
  for law in "${LAWS[@]}"; do
    for range_m in "${RANGES[@]}"; do
      run_case "$group" "$law" "$range_m"
    done
  done
done

python3 examples/generate_body_rate_diagnostic_report.py \
  --stamp "$STAMP" \
  --trajectory-dir "$TRAJECTORY_DIR" \
  --report-path "$REPORT_PATH" \
  --asset-dir "$ASSET_DIR"

echo "body_rate_diagnostic_report=${REPORT_PATH}"
