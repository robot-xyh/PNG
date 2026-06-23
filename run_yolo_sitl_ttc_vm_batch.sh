#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

STAMP="${STAMP:-yolo_sitl_ttc_vm_$(date +%Y%m%d_%H%M%S)}"
RANGES=(${RANGES:-50 60 70 80 90 100})
ALTITUDE_OFFSET="${ALTITUDE_OFFSET:-20}"
INTERCEPT_ALTITUDE_M="${INTERCEPT_ALTITUDE_M:-50}"
INTRUDER_SPEED="${INTRUDER_SPEED:-5}"
SPEED_RATIO="${SPEED_RATIO:-2}"
NAVIGATION_CONSTANT="${NAVIGATION_CONSTANT:-3.0}"
GUIDANCE_OUTPUT_MODE="${GUIDANCE_OUTPUT_MODE:-accel_integral}"
MAX_GUIDANCE_ACCEL_MPS2="${MAX_GUIDANCE_ACCEL_MPS2:-15.0}"
MIN_SPEED_RATIO="${MIN_SPEED_RATIO:-0.60}"
ACCEL_INTEGRAL_RESET_ON_INVALID="${ACCEL_INTEGRAL_RESET_ON_INVALID:-0}"
BODY_RATE_MAX_TILT_DEG="${BODY_RATE_MAX_TILT_DEG:-20}"
BODY_RATE_ROLL_GAIN="${BODY_RATE_ROLL_GAIN:-1.0}"
BODY_RATE_PITCH_GAIN="${BODY_RATE_PITCH_GAIN:-1.0}"
BODY_RATE_ATTITUDE_P="${BODY_RATE_ATTITUDE_P:-4.0}"
BODY_RATE_MAX_ROLL_RATE_DEG="${BODY_RATE_MAX_ROLL_RATE_DEG:-60}"
BODY_RATE_MAX_PITCH_RATE_DEG="${BODY_RATE_MAX_PITCH_RATE_DEG:-60}"
BODY_RATE_HOVER_THRUST="${BODY_RATE_HOVER_THRUST:-0.50}"
BODY_RATE_THRUST_GAIN="${BODY_RATE_THRUST_GAIN:-0.50}"
BODY_RATE_MIN_THRUST="${BODY_RATE_MIN_THRUST:-0.25}"
BODY_RATE_MAX_THRUST="${BODY_RATE_MAX_THRUST:-0.75}"
BODY_RATE_SPEED_HOLD_GAIN="${BODY_RATE_SPEED_HOLD_GAIN:-1.2}"
BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2="${BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2:-6.0}"
BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2="${BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2:-18.0}"
RATE_HZ="${RATE_HZ:-8}"
YAW_ERROR_GAIN="${YAW_ERROR_GAIN:-1.6}"
MAX_YAW_RATE_DEG="${MAX_YAW_RATE_DEG:-45}"
if [[ -z "${PX4_COMMAND_MODE:-}" && "$GUIDANCE_OUTPUT_MODE" == "accel_body_rate" ]]; then
  PX4_COMMAND_MODE="mavlink_body_rate"
else
  PX4_COMMAND_MODE="${PX4_COMMAND_MODE:-velocity_yaw_rate}"
fi
YOLO_MODEL="${YOLO_MODEL:-vision_guidance/best.pt}"
YOLO_DEVICE="${YOLO_DEVICE:-0}"
YOLO_CONF="${YOLO_CONF:-0.1}"
YOLO_IOU="${YOLO_IOU:-0.7}"
YOLO_IMGSZ="${YOLO_IMGSZ:-640}"
YOLO_SINGLE_TARGET_MAX_CENTER_JUMP_PX="${YOLO_SINGLE_TARGET_MAX_CENTER_JUMP_PX:-260}"
SHADOW_AIRSIM_DETECT="${SHADOW_AIRSIM_DETECT:-1}"
LOS_FILTER="${LOS_FILTER:-1}"
REJECT_TOP_CLIPPED_PITCH="${REJECT_TOP_CLIPPED_PITCH:-0}"
FRAME_CENTERING="${FRAME_CENTERING:-1}"
FRAME_CENTERING_SPEED_RATIO="${FRAME_CENTERING_SPEED_RATIO:-1.45}"
TERMINAL_CAPTURE_SPEED_RATIO="${TERMINAL_CAPTURE_SPEED_RATIO:-1.20}"
FRAME_CENTERING_MAX_LATERAL_SPEED="${FRAME_CENTERING_MAX_LATERAL_SPEED:-1.20}"
TERMINAL_CAPTURE_MAX_LATERAL_SPEED="${TERMINAL_CAPTURE_MAX_LATERAL_SPEED:-0.55}"
FRAME_CENTERING_LATERAL_SCALE="${FRAME_CENTERING_LATERAL_SCALE:-0.20}"
TERMINAL_CAPTURE_LATERAL_SCALE="${TERMINAL_CAPTURE_LATERAL_SCALE:-0.08}"
FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY="${FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY:-0}"
CAMERA_X="${CAMERA_X:-0.5}"
CAMERA_Y="${CAMERA_Y:-0}"
CAMERA_Z="${CAMERA_Z:-0}"
CAMERA_PITCH_DEG="${CAMERA_PITCH_DEG:-0}"
CAMERA_ROLL_DEG="${CAMERA_ROLL_DEG:-0}"
CAMERA_YAW_DEG="${CAMERA_YAW_DEG:-0}"
INTRUDER_ACTOR_ASSET="${INTRUDER_ACTOR_ASSET:-Quadrotor1}"
INTRUDER_ACTOR_SCALE="${INTRUDER_ACTOR_SCALE:-1.0}"
SETTINGS_PATH="${SETTINGS_PATH:-$SCRIPT_DIR/config/airsim_blocks_px4_actor_settings.json}"
LOG_DIR="$SCRIPT_DIR/logs/strict_reset"
TRAJECTORY_DIR="$SCRIPT_DIR/logs/yolo_sitl_ttc_vm"
REPORT_PATH="${REPORT_PATH:-$SCRIPT_DIR/完整方案/YOLO_SITL_AccelPNG_TTC_VM拦截对比报告.md}"
ASSET_DIR="${ASSET_DIR:-$SCRIPT_DIR/完整方案/assets/YOLO_SITL_AccelPNG_TTC_VM拦截对比报告}"
REPORT_TITLE="${REPORT_TITLE:-YOLO + ByteTrack PX4 SITL 加速度过载 PNG TTC / V_m 拦截对比报告}"
RANGE_NOTE="${RANGE_NOTE:-两组均测试 50m、60m、70m、80m、90m、100m，每个工况重启 PX4 SITL 和 Blocks。}"

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
  local range_m="$2"
  local run_tag="${label}_r${range_m}_h${ALTITUDE_OFFSET}_${STAMP}"
  PX4_LOG="$LOG_DIR/px4_${run_tag}.log"
  BLOCKS_LOG="$LOG_DIR/blocks_${run_tag}.log"

  stop_sim
  echo "Starting PX4 SITL for ${run_tag}"
  script -q -f -c "$SCRIPT_DIR/run_px4_sitl.sh" "$PX4_LOG" >/dev/null 2>&1 &
  PX4_PID=$!
  wait_for_px4 "$PX4_LOG"

  echo "Starting Blocks for ${run_tag}"
  "$SCRIPT_DIR/run_blocks_px4_actor.sh" >"$BLOCKS_LOG" 2>&1 &
  BLOCKS_PID=$!
  wait_for_airsim
}

duration_for_range() {
  local range_m="$1"
  python3 - "$range_m" <<'PY'
import sys
r = float(sys.argv[1])
print(f"{max(28.0, min(45.0, r * 0.45 + 8.0)):.1f}")
PY
}

run_case() {
  local label="$1"
  local guidance="$2"
  local range_m="$3"
  local duration_s
  duration_s="$(duration_for_range "$range_m")"
  local prefix="yolo_sitl_${label}_${STAMP}_r${range_m}_h${ALTITUDE_OFFSET}"

  start_stack "$label" "$range_m"
  echo "Running ${label}: guidance=${guidance}, range=${range_m}m, duration=${duration_s}s"

  local guidance_args=()
  if [[ "$guidance" == "fixed_vm_png" ]]; then
    guidance_args=(--guidance-law fixed_vm_png --navigation-constant "$NAVIGATION_CONSTANT")
  else
    guidance_args=(--guidance-law ttc_png)
  fi
  local shadow_args=(--no-shadow-airsim-detect)
  if [[ "$SHADOW_AIRSIM_DETECT" == "1" || "$SHADOW_AIRSIM_DETECT" == "true" || "$SHADOW_AIRSIM_DETECT" == "TRUE" ]]; then
    shadow_args=(--shadow-airsim-detect)
  fi
  local los_filter_args=(--no-los-filter)
  if [[ "$LOS_FILTER" == "1" || "$LOS_FILTER" == "true" || "$LOS_FILTER" == "TRUE" ]]; then
    los_filter_args=(--los-filter)
  fi
  local top_clip_args=(--no-reject-top-clipped-pitch)
  if [[ "$REJECT_TOP_CLIPPED_PITCH" == "1" || "$REJECT_TOP_CLIPPED_PITCH" == "true" || "$REJECT_TOP_CLIPPED_PITCH" == "TRUE" ]]; then
    top_clip_args=(--reject-top-clipped-pitch)
  fi
  local accel_reset_args=(--no-accel-integral-reset-on-invalid)
  if [[ "$ACCEL_INTEGRAL_RESET_ON_INVALID" == "1" || "$ACCEL_INTEGRAL_RESET_ON_INVALID" == "true" || "$ACCEL_INTEGRAL_RESET_ON_INVALID" == "TRUE" ]]; then
    accel_reset_args=(--accel-integral-reset-on-invalid)
  fi
  local frame_centering_args=(--no-frame-centering)
  if [[ "$FRAME_CENTERING" == "1" || "$FRAME_CENTERING" == "true" || "$FRAME_CENTERING" == "TRUE" ]]; then
    frame_centering_args=(
      --frame-centering
      --frame-centering-speed-ratio "$FRAME_CENTERING_SPEED_RATIO"
      --terminal-capture-speed-ratio "$TERMINAL_CAPTURE_SPEED_RATIO"
      --frame-centering-max-lateral-speed "$FRAME_CENTERING_MAX_LATERAL_SPEED"
      --terminal-capture-max-lateral-speed "$TERMINAL_CAPTURE_MAX_LATERAL_SPEED"
      --frame-centering-lateral-scale "$FRAME_CENTERING_LATERAL_SCALE"
      --terminal-capture-lateral-scale "$TERMINAL_CAPTURE_LATERAL_SCALE"
    )
    if [[ "$FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY" == "1" || "$FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY" == "true" || "$FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY" == "TRUE" ]]; then
      frame_centering_args+=(--frame-centering-loss-hold-last-velocity)
    else
      frame_centering_args+=(--no-frame-centering-loss-hold-last-velocity)
    fi
  fi

  python3 examples/run_airsim_strapdown_vision_png.py \
    --enable-motion \
    --duration-s "$duration_s" \
    --rate-hz "$RATE_HZ" \
    --intruder-speed "$INTRUDER_SPEED" \
    --speed-ratio "$SPEED_RATIO" \
    "${guidance_args[@]}" \
    --guidance-output-mode "$GUIDANCE_OUTPUT_MODE" \
    --max-guidance-accel-mps2 "$MAX_GUIDANCE_ACCEL_MPS2" \
    --min-speed-ratio "$MIN_SPEED_RATIO" \
    "${accel_reset_args[@]}" \
    --body-rate-max-tilt-deg "$BODY_RATE_MAX_TILT_DEG" \
    --body-rate-roll-gain "$BODY_RATE_ROLL_GAIN" \
    --body-rate-pitch-gain "$BODY_RATE_PITCH_GAIN" \
    --body-rate-attitude-p "$BODY_RATE_ATTITUDE_P" \
    --body-rate-max-roll-rate-deg "$BODY_RATE_MAX_ROLL_RATE_DEG" \
    --body-rate-max-pitch-rate-deg "$BODY_RATE_MAX_PITCH_RATE_DEG" \
    --body-rate-hover-thrust "$BODY_RATE_HOVER_THRUST" \
    --body-rate-thrust-gain "$BODY_RATE_THRUST_GAIN" \
    --body-rate-min-thrust "$BODY_RATE_MIN_THRUST" \
    --body-rate-max-thrust "$BODY_RATE_MAX_THRUST" \
    --body-rate-speed-hold-gain "$BODY_RATE_SPEED_HOLD_GAIN" \
    --body-rate-speed-hold-max-accel-mps2 "$BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2" \
    --body-rate-total-accel-limit-mps2 "$BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2" \
    --intercept-altitude-m "$INTERCEPT_ALTITUDE_M" \
    --intruder-altitude-offset-m "$ALTITUDE_OFFSET" \
    --start-horizontal-range-m "$range_m" \
    --start-lateral-offset-m -20 \
    --trajectory-dir "$TRAJECTORY_DIR" \
    --trajectory-prefix "$prefix" \
    --settings-path "$SETTINGS_PATH" \
    --no-show-window \
    --no-record-preview \
    --preview-max-frames 0 \
    --no-plot \
    --print-every-n 0 \
    --reset \
    --px4-interceptor \
    --intruder Intruder \
    --intruder-actor \
    --intruder-actor-name IntruderActor \
    --intruder-actor-asset "$INTRUDER_ACTOR_ASSET" \
    --intruder-actor-scale "$INTRUDER_ACTOR_SCALE" \
    --intruder-actor-respawn \
    --mesh 'IntruderActor' \
    --detector-source yolo_bytetrack \
    --yolo-model "$YOLO_MODEL" \
    --yolo-class-id 0 \
    --yolo-conf "$YOLO_CONF" \
    --yolo-iou "$YOLO_IOU" \
    --yolo-imgsz "$YOLO_IMGSZ" \
    --yolo-device "$YOLO_DEVICE" \
    --yolo-tracker bytetrack.yaml \
    --yolo-allow-untracked-fallback \
    --yolo-single-target-mode \
    --yolo-single-target-max-center-jump-px "$YOLO_SINGLE_TARGET_MAX_CENTER_JUMP_PX" \
    "${shadow_args[@]}" \
    "${los_filter_args[@]}" \
    --frame-guard \
    "${frame_centering_args[@]}" \
    --yaw-control \
    --yaw-error-gain "$YAW_ERROR_GAIN" \
    --max-yaw-rate-deg "$MAX_YAW_RATE_DEG" \
    --ttc-soft-guidance \
    --terminal-extrapolation \
    --terminal-image-kf \
    --terminal-image-kf-guidance \
    --climb-timeout-s 90 \
    --no-px4-command-join \
    --px4-command-mode "$PX4_COMMAND_MODE" \
    --min-command-duration-s 0.12 \
    --command-duration-margin-s 0.04 \
    --max-command-duration-s 0.25 \
    --camera-x "$CAMERA_X" \
    --camera-y "$CAMERA_Y" \
    --camera-z "$CAMERA_Z" \
    --camera-pitch-deg "$CAMERA_PITCH_DEG" \
    --camera-roll-deg "$CAMERA_ROLL_DEG" \
    --camera-yaw-deg "$CAMERA_YAW_DEG" \
    "${top_clip_args[@]}"

  stop_sim
}

summarize_label() {
  local label="$1"
  local prefix="yolo_sitl_${label}_${STAMP}"
  python3 examples/batch_strapdown_accuracy.py \
    --trajectory-dir "$TRAJECTORY_DIR" \
    --summarize-prefix "$prefix"
}

trap stop_sim EXIT

echo "YOLO SITL TTC/Vm stamp: ${STAMP}"
echo "Ranges: ${RANGES[*]}"
echo "Detector: ${YOLO_MODEL}, device=${YOLO_DEVICE}, conf=${YOLO_CONF}, tracker=bytetrack.yaml"
echo "Actor: ${INTRUDER_ACTOR_ASSET}, scale=${INTRUDER_ACTOR_SCALE}"
echo "PX4 command mode: ${PX4_COMMAND_MODE}"
echo "Guidance output: ${GUIDANCE_OUTPUT_MODE}; max_accel=${MAX_GUIDANCE_ACCEL_MPS2}; min_speed_ratio=${MIN_SPEED_RATIO}; reset_on_invalid=${ACCEL_INTEGRAL_RESET_ON_INVALID}"
echo "Body-rate: tilt=${BODY_RATE_MAX_TILT_DEG}deg rates=${BODY_RATE_MAX_ROLL_RATE_DEG}/${BODY_RATE_MAX_PITCH_RATE_DEG}deg/s thrust=${BODY_RATE_MIN_THRUST}/${BODY_RATE_HOVER_THRUST}/${BODY_RATE_MAX_THRUST}"
echo "Body-rate speed hold: gain=${BODY_RATE_SPEED_HOLD_GAIN}; max_accel=${BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2}; total_limit=${BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2}"
echo "Camera: x=${CAMERA_X}, y=${CAMERA_Y}, z=${CAMERA_Z}, pitch=${CAMERA_PITCH_DEG}, roll=${CAMERA_ROLL_DEG}, yaw=${CAMERA_YAW_DEG}"
echo "Shadow AirSim detect: ${SHADOW_AIRSIM_DETECT}; LOS filter: ${LOS_FILTER}; reject top clipped pitch: ${REJECT_TOP_CLIPPED_PITCH}; yolo-imgsz=${YOLO_IMGSZ}"
echo "Frame centering: ${FRAME_CENTERING}; speed_ratio=${FRAME_CENTERING_SPEED_RATIO}; terminal_speed_ratio=${TERMINAL_CAPTURE_SPEED_RATIO}; lateral=${FRAME_CENTERING_MAX_LATERAL_SPEED}/${TERMINAL_CAPTURE_MAX_LATERAL_SPEED}"
echo "Frame centering loss hold last velocity: ${FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY}"
echo "Every case restarts PX4 SITL and Blocks."

for range_m in "${RANGES[@]}"; do
  run_case TTC ttc_png "$range_m"
done
summarize_label TTC

for range_m in "${RANGES[@]}"; do
  run_case VM fixed_vm_png "$range_m"
done
summarize_label VM

python3 examples/generate_yolo_sitl_ttc_vm_report.py \
  --stamp "$STAMP" \
  --report-path "$REPORT_PATH" \
  --asset-dir "$ASSET_DIR" \
  --title "$REPORT_TITLE" \
  --range-note "$RANGE_NOTE"

echo
echo "yolo_sitl_stamp=${STAMP}"
echo "report=${REPORT_PATH}"
