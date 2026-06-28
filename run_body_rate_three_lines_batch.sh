#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

STAMP="${STAMP:-body_rate_three_$(date +%Y%m%d_%H%M%S)}"
PHASE="${PHASE:-smoke}"
ALTITUDE_OFFSET="${ALTITUDE_OFFSET:-20}"
INTERCEPT_ALTITUDE_M="${INTERCEPT_ALTITUDE_M:-50}"
INTRUDER_SPEED="${INTRUDER_SPEED:-5}"
SPEED_RATIO="${SPEED_RATIO:-2}"
NAVIGATION_CONSTANT="${NAVIGATION_CONSTANT:-3.0}"
RATE_HZ="${RATE_HZ:-8}"
RATE_HZ_12="${RATE_HZ_12:-12}"
RATE_HZ_FAST="${RATE_HZ_FAST:-20}"
CASE_TIMEOUT_S="${CASE_TIMEOUT_S:-180}"
SETTINGS_BASE="${SETTINGS_BASE:-$SCRIPT_DIR/config/airsim_blocks_px4_actor_settings.json}"
SETTINGS_HIGH_AUTHORITY="${SETTINGS_HIGH_AUTHORITY:-$SCRIPT_DIR/config/airsim_blocks_px4_actor_high_authority_settings.json}"
LOG_DIR="${LOG_DIR:-$SCRIPT_DIR/logs/body_rate_three_lines_stack}"
TRAJECTORY_DIR="${TRAJECTORY_DIR:-$SCRIPT_DIR/logs/body_rate_three_lines}"
REPORT_PATH="${REPORT_PATH:-$SCRIPT_DIR/完整方案/BodyRate_三问题线实施实验报告.md}"
ASSET_DIR="${ASSET_DIR:-$SCRIPT_DIR/完整方案/assets/BodyRate_三问题线实施实验报告}"

A_RANGES=(${A_RANGES:-50 80 100})
B_RANGES=(${B_RANGES:-70 90 100})
C_RANGES=(${C_RANGES:-50 60 70 80 90 100})
ALL_RANGES=(${ALL_RANGES:-50 60 70 80 90 100})

INTRUDER_ACTOR_NAME="${INTRUDER_ACTOR_NAME:-IntruderActor}"
INTRUDER_ACTOR_ASSET="${INTRUDER_ACTOR_ASSET:-Quadrotor1}"
INTRUDER_ACTOR_SCALE="${INTRUDER_ACTOR_SCALE:-1.0}"
START_LATERAL_OFFSET_M="${START_LATERAL_OFFSET_M:--20}"
YOLO_MODEL="${YOLO_MODEL:-$SCRIPT_DIR/vision_guidance/best.pt}"
YOLO_CLASS_ID="${YOLO_CLASS_ID:-0}"
YOLO_IMGSZ="${YOLO_IMGSZ:-640}"
YOLO_CONF="${YOLO_CONF:-0.1}"
YOLO_DEVICE="${YOLO_DEVICE:-0}"

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

case "$PHASE" in
  smoke)
    EXPERIMENTS=(${EXPERIMENTS:-A_truth_actor B0 B1 B2 C0 C1 C2})
    ;;
  full)
    EXPERIMENTS=(${EXPERIMENTS:-A_truth_actor B0 B1 B2 B3 B4 B5 C0 C1 C2})
    A_RANGES=(${FULL_A_RANGES:-${ALL_RANGES[*]}})
    B_RANGES=(${FULL_B_RANGES:-${ALL_RANGES[*]}})
    ;;
  control)
    EXPERIMENTS=(${EXPERIMENTS:-C4 C5 C6 C7 C8})
    ;;
  *)
    EXPERIMENTS=(${EXPERIMENTS:-$PHASE})
    ;;
esac

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
  local experiment="$1"
  local range_m="$2"
  local settings_path="$3"
  local tag="${experiment}_r${range_m}_${STAMP}"
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

base_args=(
  --intruder Intruder
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

actor_args=(
  --intruder-actor
  --intruder-actor-name "$INTRUDER_ACTOR_NAME"
  --intruder-actor-asset "$INTRUDER_ACTOR_ASSET"
  --intruder-actor-scale "$INTRUDER_ACTOR_SCALE"
  --intruder-actor-respawn
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

yolo_args=(
  --detector-source yolo_bytetrack_async
  --yolo-model "$YOLO_MODEL"
  --yolo-class-id "$YOLO_CLASS_ID"
  --yolo-imgsz "$YOLO_IMGSZ"
  --yolo-conf "$YOLO_CONF"
  --yolo-device "$YOLO_DEVICE"
  --yolo-allow-untracked-fallback
  --yolo-single-target-mode
  --no-show-window
  --no-record-preview
)

airsim_detector_args=(
  --detector-source airsim
  --mesh "$INTRUDER_ACTOR_NAME"
  --no-show-window
  --no-record-preview
)

case_ranges() {
  case "$1" in
    A_*) echo "${A_RANGES[*]}" ;;
    B*) echo "${B_RANGES[*]}" ;;
    *) echo "${C_RANGES[*]}" ;;
  esac
}

case_laws() {
  case "$PHASE:$1" in
    smoke:C*) echo "TTC" ;;
    *) echo "TTC VM" ;;
  esac
}

run_case() {
  local experiment="$1"
  local law="$2"
  local range_m="$3"
  local script_path=""
  local settings_path="$SETTINGS_BASE"
  local rate_hz="$RATE_HZ"
  local min_command_duration="0.12"
  local command_margin="0.04"
  local max_command_duration="0.25"
  local -a target_args=("${actor_args[@]}")
  local -a detector_args=()
  local -a control_args=("${body_rate_args_base[@]}")
  local -a experiment_args=()

  case "$experiment" in
    A_truth_actor)
      script_path="examples/run_airsim_truth_png.py"
      ;;
    A_vehicle_pair)
      script_path="examples/run_airsim_truth_png.py"
      target_args=()
      ;;
    B0)
      script_path="examples/run_airsim_gimbal_vision_png.py"
      detector_args=("${airsim_detector_args[@]}")
      ;;
    B1)
      script_path="examples/run_airsim_gimbal_vision_png.py"
      detector_args=("${airsim_detector_args[@]}" --no-los-filter)
      ;;
    B2)
      script_path="examples/run_airsim_gimbal_vision_png.py"
      detector_args=("${airsim_detector_args[@]}" --no-los-filter)
      experiment_args=(--gimbal-body-yaw-feedback)
      ;;
    B3)
      script_path="examples/run_airsim_gimbal_vision_png.py"
      detector_args=("${airsim_detector_args[@]}" --no-los-filter)
      experiment_args=(--gimbal-body-yaw-feedback --terminal-gimbal-gain-scale 0.25 --terminal-cutoff-area-ratio 0.55)
      ;;
    B4)
      script_path="examples/run_airsim_gimbal_vision_png.py"
      detector_args=("${airsim_detector_args[@]}" --no-los-filter)
      experiment_args=(--gimbal-body-yaw-feedback --ttc-soft-guidance --ttc-soft-min-gain-scale 0.65)
      ;;
    B5)
      script_path="examples/run_airsim_gimbal_vision_png.py"
      detector_args=("${airsim_detector_args[@]}" --no-los-filter)
      experiment_args=(--gimbal-body-yaw-feedback --terminal-blind-duration-s 0.45 --terminal-cutoff-miss-count 2)
      ;;
    C0)
      script_path="examples/run_airsim_strapdown_vision_png.py"
      detector_args=("${airsim_detector_args[@]}" --no-los-filter)
      ;;
    C1)
      script_path="examples/run_airsim_strapdown_vision_png.py"
      detector_args=("${yolo_args[@]}" --no-los-filter)
      ;;
    C2)
      script_path="examples/run_airsim_strapdown_vision_png.py"
      detector_args=("${yolo_args[@]}" --los-filter --los-filter-innovation-reject 0.60 --los-filter-terminal-innovation-reject 1.20 --terminal-los-reject-policy raw_capped)
      ;;
    C3)
      script_path="examples/run_airsim_strapdown_vision_png.py"
      detector_args=("${yolo_args[@]}" --los-filter --terminal-los-reject-policy strict)
      ;;
    C4)
      script_path="examples/run_airsim_strapdown_vision_png.py"
      detector_args=("${yolo_args[@]}" --no-los-filter)
      control_args=("${body_rate_args_base[@]}" --body-rate-control-profile hybrid_v2 --body-rate-v2-thrust-reserve 0.15 --body-rate-v2-guard-png-scale 0.60 --body-rate-v2-guard-speed-hold-scale 0.45)
      ;;
    C5)
      script_path="examples/run_airsim_strapdown_vision_png.py"
      detector_args=("${yolo_args[@]}" --no-los-filter)
      rate_hz="$RATE_HZ_12"
      min_command_duration="0.08"
      command_margin="0.03"
      max_command_duration="0.16"
      ;;
    C6)
      script_path="examples/run_airsim_strapdown_vision_png.py"
      detector_args=("${yolo_args[@]}" --no-los-filter)
      rate_hz="$RATE_HZ_FAST"
      min_command_duration="0.05"
      command_margin="0.02"
      max_command_duration="0.12"
      ;;
    C7)
      script_path="examples/run_airsim_strapdown_vision_png.py"
      settings_path="$SETTINGS_HIGH_AUTHORITY"
      control_args=("${body_rate_args_high[@]}")
      detector_args=("${yolo_args[@]}" --no-los-filter)
      ;;
    C8)
      script_path="examples/run_airsim_strapdown_vision_png.py"
      settings_path="$SETTINGS_HIGH_AUTHORITY"
      control_args=("${body_rate_args_high[@]}" --max-guidance-accel-mps2 12 --body-rate-total-accel-limit-mps2 18)
      detector_args=("${yolo_args[@]}" --no-los-filter)
      ;;
    *)
      echo "Unknown experiment: $experiment" >&2
      return 1
      ;;
  esac

  local duration_s
  duration_s="$(duration_for_range "$range_m")"
  local prefix="body_rate_three_${experiment}_${law}_${STAMP}_r${range_m}_h${ALTITUDE_OFFSET}"
  read -r -a guidance_args <<<"$(law_args "$law")"

  start_stack "$experiment" "$range_m" "$settings_path"
  echo "Running experiment=${experiment}, law=${law}, range=${range_m}m, duration=${duration_s}s"
  local rc=0
  timeout --kill-after=10s "$CASE_TIMEOUT_S" \
  python3 "$script_path" \
    "${base_args[@]}" \
    "${target_args[@]}" \
    "${guidance_args[@]}" \
    "${control_args[@]}" \
    "${detector_args[@]}" \
    "${experiment_args[@]}" \
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
    echo "case_failed experiment=${experiment} law=${law} range=${range_m} rc=${rc}" >&2
  fi
  return 0
}

trap stop_sim EXIT

echo "body_rate_three_stamp=${STAMP}"
echo "phase=${PHASE}"
echo "experiments=${EXPERIMENTS[*]}"

for experiment in "${EXPERIMENTS[@]}"; do
  read -r -a ranges <<<"$(case_ranges "$experiment")"
  read -r -a laws <<<"$(case_laws "$experiment")"
  for law in "${laws[@]}"; do
    for range_m in "${ranges[@]}"; do
      run_case "$experiment" "$law" "$range_m"
    done
  done
done

python3 examples/generate_body_rate_three_lines_report.py \
  --stamp "$STAMP" \
  --trajectory-dir "$TRAJECTORY_DIR" \
  --report-path "$REPORT_PATH" \
  --asset-dir "$ASSET_DIR"

echo "body_rate_three_lines_report=${REPORT_PATH}"
