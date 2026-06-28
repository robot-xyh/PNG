#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

STAMP="${STAMP:-yolo_sitl_ttc_vm_$(date +%Y%m%d_%H%M%S)}"
export AIRSIM_RPC_HOST="${AIRSIM_RPC_HOST:-127.0.0.2}"
RANGES=(${RANGES:-50 60 70 80 90 100})
RUN_TTC="${RUN_TTC:-1}"
RUN_VM="${RUN_VM:-1}"
ALTITUDE_OFFSET="${ALTITUDE_OFFSET:-20}"
START_LATERAL_OFFSET="${START_LATERAL_OFFSET:--20}"
INTERCEPT_ALTITUDE_M="${INTERCEPT_ALTITUDE_M:-50}"
INTRUDER_SPEED="${INTRUDER_SPEED:-5}"
SPEED_RATIO="${SPEED_RATIO:-2}"
NAVIGATION_CONSTANT="${NAVIGATION_CONSTANT:-3.0}"
DURATION_MIN_S="${DURATION_MIN_S:-28.0}"
DURATION_MAX_S="${DURATION_MAX_S:-45.0}"
DURATION_RANGE_SCALE="${DURATION_RANGE_SCALE:-0.45}"
DURATION_RANGE_OFFSET="${DURATION_RANGE_OFFSET:-8.0}"
GUIDANCE_OUTPUT_MODE="${GUIDANCE_OUTPUT_MODE:-accel_attitude}"
TERMINAL_PROFILE="${TERMINAL_PROFILE:-legacy}"
USER_SHADOW_AIRSIM_DETECT_SET="${SHADOW_AIRSIM_DETECT+x}"
USER_TERMINAL_IMAGE_KF_MAX_PREDICT_S_SET="${TERMINAL_IMAGE_KF_MAX_PREDICT_S+x}"
USER_TERMINAL_IMAGE_KF_ACCEL_NOISE_RAD_S2_SET="${TERMINAL_IMAGE_KF_ACCEL_NOISE_RAD_S2+x}"
USER_TERMINAL_IMAGE_KF_INNOVATION_REJECT_RAD_SET="${TERMINAL_IMAGE_KF_INNOVATION_REJECT_RAD+x}"
USER_TERMINAL_IMAGE_KF_MAX_RATE_RAD_S_SET="${TERMINAL_IMAGE_KF_MAX_RATE_RAD_S+x}"
USER_TERMINAL_IMAGE_KF_SOFT_REJECT_PREDICT_SET="${TERMINAL_IMAGE_KF_SOFT_REJECT_PREDICT+x}"
USER_TERMINAL_VELOCITY_BLIND_PUSH_SET="${TERMINAL_VELOCITY_BLIND_PUSH+x}"
USER_TERMINAL_ACCEL_HOLD_SET="${TERMINAL_ACCEL_HOLD+x}"
USER_FRAME_CENTERING_ENTER_ERROR_RATIO_SET="${FRAME_CENTERING_ENTER_ERROR_RATIO+x}"
USER_FRAME_CENTERING_TERMINAL_ERROR_RATIO_SET="${FRAME_CENTERING_TERMINAL_ERROR_RATIO+x}"
USER_FRAME_CENTERING_AREA_RATIO_SET="${FRAME_CENTERING_AREA_RATIO+x}"
USER_FRAME_CENTERING_LOSS_HOLD_S_SET="${FRAME_CENTERING_LOSS_HOLD_S+x}"
USER_FRAME_CENTERING_SPEED_RATIO_SET="${FRAME_CENTERING_SPEED_RATIO+x}"
USER_TERMINAL_CAPTURE_SPEED_RATIO_SET="${TERMINAL_CAPTURE_SPEED_RATIO+x}"
USER_FRAME_CENTERING_LATERAL_SCALE_SET="${FRAME_CENTERING_LATERAL_SCALE+x}"
USER_TERMINAL_CAPTURE_LATERAL_SCALE_SET="${TERMINAL_CAPTURE_LATERAL_SCALE+x}"
USER_FRAME_CENTERING_MAX_LATERAL_SPEED_SET="${FRAME_CENTERING_MAX_LATERAL_SPEED+x}"
USER_TERMINAL_CAPTURE_MAX_LATERAL_SPEED_SET="${TERMINAL_CAPTURE_MAX_LATERAL_SPEED+x}"
USER_FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY_SET="${FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY+x}"
USER_ATTITUDE_MAX_TILT_DEG_SET="${ATTITUDE_MAX_TILT_DEG+x}"
USER_ATTITUDE_SPEED_HOLD_MAX_ACCEL_MPS2_SET="${ATTITUDE_SPEED_HOLD_MAX_ACCEL_MPS2+x}"
USER_ATTITUDE_TOTAL_ACCEL_LIMIT_MPS2_SET="${ATTITUDE_TOTAL_ACCEL_LIMIT_MPS2+x}"
MAX_GUIDANCE_ACCEL_MPS2="${MAX_GUIDANCE_ACCEL_MPS2:-15.0}"
MIN_SPEED_RATIO="${MIN_SPEED_RATIO:-0.60}"
ACCEL_INTEGRAL_RESET_ON_INVALID="${ACCEL_INTEGRAL_RESET_ON_INVALID:-0}"
NEAR_HIT_RADIUS_M="${NEAR_HIT_RADIUS_M:-1.5}"
BODY_RATE_MAX_TILT_DEG="${BODY_RATE_MAX_TILT_DEG:-20}"
BODY_RATE_ROLL_GAIN="${BODY_RATE_ROLL_GAIN:-1.0}"
BODY_RATE_PITCH_GAIN="${BODY_RATE_PITCH_GAIN:-1.0}"
BODY_RATE_ATTITUDE_P="${BODY_RATE_ATTITUDE_P:-4.0}"
BODY_RATE_MAX_ROLL_RATE_DEG="${BODY_RATE_MAX_ROLL_RATE_DEG:-60}"
BODY_RATE_MAX_PITCH_RATE_DEG="${BODY_RATE_MAX_PITCH_RATE_DEG:-60}"
BODY_RATE_CONTROL_PROFILE="${BODY_RATE_CONTROL_PROFILE:-legacy}"
BODY_RATE_V2_KP_ROLL="${BODY_RATE_V2_KP_ROLL:-5.0}"
BODY_RATE_V2_KP_PITCH="${BODY_RATE_V2_KP_PITCH:-5.0}"
BODY_RATE_V2_KP_YAW="${BODY_RATE_V2_KP_YAW:-3.0}"
BODY_RATE_V2_MAX_PQ_RATE_DEG_S="${BODY_RATE_V2_MAX_PQ_RATE_DEG_S:-120.0}"
BODY_RATE_V2_SLEW_PQ_DEG_S2="${BODY_RATE_V2_SLEW_PQ_DEG_S2:-720.0}"
BODY_RATE_V2_SLEW_R_DEG_S2="${BODY_RATE_V2_SLEW_R_DEG_S2:-540.0}"
BODY_RATE_V2_THRUST_RESERVE="${BODY_RATE_V2_THRUST_RESERVE:-0.15}"
BODY_RATE_V2_GUARD_ERROR_RATIO="${BODY_RATE_V2_GUARD_ERROR_RATIO:-0.55}"
BODY_RATE_V2_GUARD_PNG_SCALE="${BODY_RATE_V2_GUARD_PNG_SCALE:-0.60}"
BODY_RATE_V2_GUARD_SPEED_HOLD_SCALE="${BODY_RATE_V2_GUARD_SPEED_HOLD_SCALE:-0.45}"
THRUST_MODEL="${THRUST_MODEL:-airsim_generic_quad}"
VEHICLE_MASS_KG="${VEHICLE_MASS_KG:-1.0}"
VEHICLE_MAX_TOTAL_THRUST_N="${VEHICLE_MAX_TOTAL_THRUST_N:-16.717785072}"
BODY_RATE_HOVER_THRUST="${BODY_RATE_HOVER_THRUST:-0.5865998371}"
BODY_RATE_THRUST_GAIN="${BODY_RATE_THRUST_GAIN:-0.5865998371}"
BODY_RATE_MIN_THRUST="${BODY_RATE_MIN_THRUST:-0.25}"
BODY_RATE_MAX_THRUST="${BODY_RATE_MAX_THRUST:-0.95}"
BODY_RATE_SPEED_HOLD_GAIN="${BODY_RATE_SPEED_HOLD_GAIN:-1.2}"
BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2="${BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2:-6.0}"
BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2="${BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2:-18.0}"
ATTITUDE_MAX_TILT_DEG="${ATTITUDE_MAX_TILT_DEG:-25}"
ATTITUDE_YAW_LOOKAHEAD_S="${ATTITUDE_YAW_LOOKAHEAD_S:-0.25}"
ATTITUDE_HOVER_THRUST="${ATTITUDE_HOVER_THRUST:-0.5865998371}"
ATTITUDE_THRUST_GAIN="${ATTITUDE_THRUST_GAIN:-0.5865998371}"
ATTITUDE_MIN_THRUST="${ATTITUDE_MIN_THRUST:-0.25}"
ATTITUDE_MAX_THRUST="${ATTITUDE_MAX_THRUST:-0.95}"
ATTITUDE_SPEED_HOLD_GAIN="${ATTITUDE_SPEED_HOLD_GAIN:-1.2}"
ATTITUDE_SPEED_HOLD_MAX_ACCEL_MPS2="${ATTITUDE_SPEED_HOLD_MAX_ACCEL_MPS2:-6.0}"
ATTITUDE_TOTAL_ACCEL_LIMIT_MPS2="${ATTITUDE_TOTAL_ACCEL_LIMIT_MPS2:-18.0}"
RATE_HZ="${RATE_HZ:-8}"
CASE_TIMEOUT_S="${CASE_TIMEOUT_S:-180}"
PX4_MAX_VERTICAL_SPEED="${PX4_MAX_VERTICAL_SPEED:-2.0}"
MAX_VISION_LATERAL_SPEED="${MAX_VISION_LATERAL_SPEED:-4.0}"
MAX_VISION_VERTICAL_SPEED="${MAX_VISION_VERTICAL_SPEED:-3.0}"
YAW_ERROR_GAIN="${YAW_ERROR_GAIN:-1.6}"
MAX_YAW_RATE_DEG="${MAX_YAW_RATE_DEG:-45}"
if [[ -z "${PX4_COMMAND_MODE:-}" && "$GUIDANCE_OUTPUT_MODE" == "accel_body_rate" ]]; then
  PX4_COMMAND_MODE="mavlink_body_rate"
elif [[ -z "${PX4_COMMAND_MODE:-}" && "$GUIDANCE_OUTPUT_MODE" == "accel_attitude" ]]; then
  PX4_COMMAND_MODE="mavlink_attitude"
else
  PX4_COMMAND_MODE="${PX4_COMMAND_MODE:-velocity_yaw_rate}"
fi
DETECTOR_SOURCE="${DETECTOR_SOURCE:-yolo_bytetrack}"
YOLO_MODEL="${YOLO_MODEL:-vision_guidance/best.pt}"
YOLO_DEVICE="${YOLO_DEVICE:-0}"
YOLO_CONF="${YOLO_CONF:-0.1}"
YOLO_IOU="${YOLO_IOU:-0.7}"
YOLO_IMGSZ="${YOLO_IMGSZ:-640}"
YOLO_SINGLE_TARGET_MAX_CENTER_JUMP_PX="${YOLO_SINGLE_TARGET_MAX_CENTER_JUMP_PX:-260}"
KCF_YOLO_PERIOD_N="${KCF_YOLO_PERIOD_N:-8}"
KCF_YOLO_PERIOD_S="${KCF_YOLO_PERIOD_S:-0.5}"
KCF_MAX_COAST_S="${KCF_MAX_COAST_S:-0.8}"
KCF_MIN_YOLO_IOU="${KCF_MIN_YOLO_IOU:-0.25}"
KCF_MAX_CENTER_JUMP_PX="${KCF_MAX_CENTER_JUMP_PX:-180}"
KCF_AREA_RATIO_MIN="${KCF_AREA_RATIO_MIN:-0.35}"
KCF_AREA_RATIO_MAX="${KCF_AREA_RATIO_MAX:-2.8}"
KCF_RESET_ON_YOLO_DRIFT="${KCF_RESET_ON_YOLO_DRIFT:-1}"
SHADOW_AIRSIM_DETECT="${SHADOW_AIRSIM_DETECT:-1}"
LOS_FILTER="${LOS_FILTER:-1}"
LOS_FILTER_PROCESS_LAMBDA="${LOS_FILTER_PROCESS_LAMBDA:-5e-4}"
LOS_FILTER_PROCESS_LAMBDA_DOT="${LOS_FILTER_PROCESS_LAMBDA_DOT:-2e-2}"
LOS_FILTER_MEASUREMENT_NOISE="${LOS_FILTER_MEASUREMENT_NOISE:-8e-3}"
LOS_FILTER_INNOVATION_REJECT="${LOS_FILTER_INNOVATION_REJECT:-0.75}"
LOS_FILTER_TERMINAL_INNOVATION_REJECT="${LOS_FILTER_TERMINAL_INNOVATION_REJECT:-1.20}"
LOS_FILTER_TERMINAL_AREA_RATIO="${LOS_FILTER_TERMINAL_AREA_RATIO:-0.01}"
LOS_FILTER_TERMINAL_ERROR_RATIO="${LOS_FILTER_TERMINAL_ERROR_RATIO:-0.55}"
LOS_DELAY_COMPENSATION_S="${LOS_DELAY_COMPENSATION_S:-0.18}"
REJECT_TOP_CLIPPED_PITCH="${REJECT_TOP_CLIPPED_PITCH:-0}"
FRAME_GUARD_ENTER_ERROR_RATIO="${FRAME_GUARD_ENTER_ERROR_RATIO:-0.42}"
FRAME_GUARD_EXIT_ERROR_RATIO="${FRAME_GUARD_EXIT_ERROR_RATIO:-0.28}"
FRAME_GUARD_AREA_MID_RATIO="${FRAME_GUARD_AREA_MID_RATIO:-0.0012}"
FRAME_GUARD_AREA_RATIO="${FRAME_GUARD_AREA_RATIO:-0.004}"
FRAME_GUARD_TTC_MID_S="${FRAME_GUARD_TTC_MID_S:-2.5}"
FRAME_GUARD_TTC_TERMINAL_S="${FRAME_GUARD_TTC_TERMINAL_S:-1.2}"
FRAME_GUARD_HOLD_S="${FRAME_GUARD_HOLD_S:-0.65}"
FRAME_GUARD_MIN_SPEED_RATIO="${FRAME_GUARD_MIN_SPEED_RATIO:-1.30}"
FRAME_GUARD_MID_SPEED_RATIO="${FRAME_GUARD_MID_SPEED_RATIO:-1.60}"
FRAME_GUARD_LATERAL_SCALE="${FRAME_GUARD_LATERAL_SCALE:-0.55}"
FRAME_GUARD_VERTICAL_SCALE="${FRAME_GUARD_VERTICAL_SCALE:-0.85}"
FRAME_GUARD_YAW_GAIN_SCALE="${FRAME_GUARD_YAW_GAIN_SCALE:-1.45}"
FRAME_GUARD_MAX_YAW_RATE_DEG="${FRAME_GUARD_MAX_YAW_RATE_DEG:-60.0}"
FRAME_GUARD_VERTICAL_ERROR_GAIN="${FRAME_GUARD_VERTICAL_ERROR_GAIN:-0.012}"
FRAME_CENTERING="${FRAME_CENTERING:-1}"
FRAME_CENTERING_ENTER_ERROR_RATIO="${FRAME_CENTERING_ENTER_ERROR_RATIO:-0.52}"
FRAME_CENTERING_TERMINAL_ERROR_RATIO="${FRAME_CENTERING_TERMINAL_ERROR_RATIO:-0.68}"
FRAME_CENTERING_AREA_RATIO="${FRAME_CENTERING_AREA_RATIO:-0.006}"
FRAME_CENTERING_LOSS_HOLD_S="${FRAME_CENTERING_LOSS_HOLD_S:-0.80}"
FRAME_CENTERING_SPEED_RATIO="${FRAME_CENTERING_SPEED_RATIO:-1.35}"
TERMINAL_CAPTURE_SPEED_RATIO="${TERMINAL_CAPTURE_SPEED_RATIO:-1.10}"
FRAME_CENTERING_MIN_FORWARD_RATIO="${FRAME_CENTERING_MIN_FORWARD_RATIO:-0.55}"
FRAME_CENTERING_MAX_LATERAL_SPEED="${FRAME_CENTERING_MAX_LATERAL_SPEED:-1.20}"
TERMINAL_CAPTURE_MAX_LATERAL_SPEED="${TERMINAL_CAPTURE_MAX_LATERAL_SPEED:-0.40}"
FRAME_CENTERING_LATERAL_SCALE="${FRAME_CENTERING_LATERAL_SCALE:-0.20}"
TERMINAL_CAPTURE_LATERAL_SCALE="${TERMINAL_CAPTURE_LATERAL_SCALE:-0.05}"
FRAME_CENTERING_YAW_GAIN_SCALE="${FRAME_CENTERING_YAW_GAIN_SCALE:-1.20}"
TERMINAL_CAPTURE_YAW_GAIN_SCALE="${TERMINAL_CAPTURE_YAW_GAIN_SCALE:-1.45}"
FRAME_CENTERING_MAX_YAW_RATE_DEG="${FRAME_CENTERING_MAX_YAW_RATE_DEG:-70.0}"
TERMINAL_CAPTURE_MAX_YAW_RATE_DEG="${TERMINAL_CAPTURE_MAX_YAW_RATE_DEG:-60.0}"
FRAME_CENTERING_YAW_HOLD_WINDOW_S="${FRAME_CENTERING_YAW_HOLD_WINDOW_S:-0.20}"
FRAME_CENTERING_YAW_HOLD_DECAY_TAU_S="${FRAME_CENTERING_YAW_HOLD_DECAY_TAU_S:-0.30}"
FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY="${FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY:-0}"
UPWARD_CENTERING="${UPWARD_CENTERING:-0}"
UPWARD_CENTERING_GAIN="${UPWARD_CENTERING_GAIN:-8.0}"
UPWARD_CENTERING_MAX_ACCEL_MPS2="${UPWARD_CENTERING_MAX_ACCEL_MPS2:-4.0}"
CAMERA_X="${CAMERA_X:-0.5}"
CAMERA_Y="${CAMERA_Y:-0}"
CAMERA_Z="${CAMERA_Z:-0}"
CAMERA_PITCH_DEG="${CAMERA_PITCH_DEG:-0}"
CAMERA_ROLL_DEG="${CAMERA_ROLL_DEG:-0}"
CAMERA_YAW_DEG="${CAMERA_YAW_DEG:-0}"
TERMINAL_IMAGE_KF_MAX_PREDICT_S="${TERMINAL_IMAGE_KF_MAX_PREDICT_S:-0.35}"
TERMINAL_IMAGE_KF_MEAS_NOISE_RAD="${TERMINAL_IMAGE_KF_MEAS_NOISE_RAD:-0.006}"
TERMINAL_IMAGE_KF_ACCEL_NOISE_RAD_S2="${TERMINAL_IMAGE_KF_ACCEL_NOISE_RAD_S2:-8.0}"
TERMINAL_IMAGE_KF_INNOVATION_REJECT_RAD="${TERMINAL_IMAGE_KF_INNOVATION_REJECT_RAD:-0.20}"
TERMINAL_IMAGE_KF_MAX_ANGLE_RAD="${TERMINAL_IMAGE_KF_MAX_ANGLE_RAD:-1.0}"
TERMINAL_IMAGE_KF_MAX_RATE_RAD_S="${TERMINAL_IMAGE_KF_MAX_RATE_RAD_S:-8.0}"
TERMINAL_IMAGE_KF_SOFT_REJECT_PREDICT="${TERMINAL_IMAGE_KF_SOFT_REJECT_PREDICT:-0}"
TERMINAL_EXTRAPOLATION="${TERMINAL_EXTRAPOLATION:-1}"
TERMINAL_ENTER_AREA_RATIO="${TERMINAL_ENTER_AREA_RATIO:-0.20}"
TERMINAL_SOFT_ENTER_AREA_RATIO="${TERMINAL_SOFT_ENTER_AREA_RATIO:-0.05}"
TERMINAL_CUTOFF_AREA_RATIO="${TERMINAL_CUTOFF_AREA_RATIO:-0.60}"
TERMINAL_GIMBAL_LIMIT_AREA_RATIO="${TERMINAL_GIMBAL_LIMIT_AREA_RATIO:-0.05}"
TERMINAL_CUTOFF_MISS_COUNT="${TERMINAL_CUTOFF_MISS_COUNT:-3}"
TERMINAL_MIN_TRACKING_TIME_S="${TERMINAL_MIN_TRACKING_TIME_S:-0.20}"
TERMINAL_CONFIDENCE_MIN_SCORE="${TERMINAL_CONFIDENCE_MIN_SCORE:-0.35}"
TERMINAL_MAX_MEASUREMENT_AGE_S="${TERMINAL_MAX_MEASUREMENT_AGE_S:-0.12}"
TERMINAL_BLIND_DURATION_S="${TERMINAL_BLIND_DURATION_S:-0.30}"
TERMINAL_COMMAND_AVERAGE_WINDOW_S="${TERMINAL_COMMAND_AVERAGE_WINDOW_S:-0.10}"
TERMINAL_COMMAND_DECAY_TAU_S="${TERMINAL_COMMAND_DECAY_TAU_S:-0.18}"
UPWARD_ACCEL_TERMINAL="$(python3 -c 'import sys; mode=sys.argv[1]; pitch=float(sys.argv[2]); print("1" if mode in {"accel_body_rate", "accel_attitude"} and pitch <= -75.0 else "0")' "$GUIDANCE_OUTPUT_MODE" "$CAMERA_PITCH_DEG")"
if [[ -z "$USER_TERMINAL_VELOCITY_BLIND_PUSH_SET" ]]; then
  if [[ "$UPWARD_ACCEL_TERMINAL" == "1" ]]; then
    TERMINAL_VELOCITY_BLIND_PUSH=0
  else
    TERMINAL_VELOCITY_BLIND_PUSH=1
  fi
fi
if [[ -z "$USER_TERMINAL_ACCEL_HOLD_SET" ]]; then
  if [[ "$UPWARD_ACCEL_TERMINAL" == "1" ]]; then
    TERMINAL_ACCEL_HOLD=1
  else
    TERMINAL_ACCEL_HOLD=0
  fi
fi
TERMINAL_ACCEL_HOLD_WINDOW_S="${TERMINAL_ACCEL_HOLD_WINDOW_S:-0.35}"
TERMINAL_ACCEL_DECAY_TAU_S="${TERMINAL_ACCEL_DECAY_TAU_S:-0.60}"
TERMINAL_ACCEL_HOLD_MAX_MPS2="${TERMINAL_ACCEL_HOLD_MAX_MPS2:-0.0}"
TERMINAL_TREND_BIAS_GAIN="${TERMINAL_TREND_BIAS_GAIN:-0.10}"
TERMINAL_TREND_BIAS_MAX_MPS="${TERMINAL_TREND_BIAS_MAX_MPS:-1.5}"
TERMINAL_PITCH_UP_BIAS_MPS="${TERMINAL_PITCH_UP_BIAS_MPS:-0.8}"
TERMINAL_ABORT_ON_TILT_HARDCAP="${TERMINAL_ABORT_ON_TILT_HARDCAP:-1}"
TERMINAL_YAW_RATE_EXTRAPOLATION="${TERMINAL_YAW_RATE_EXTRAPOLATION:-1}"
TERMINAL_YAW_RATE_AVERAGE_WINDOW_S="${TERMINAL_YAW_RATE_AVERAGE_WINDOW_S:-0.10}"
TERMINAL_YAW_RATE_DECAY_TAU_S="${TERMINAL_YAW_RATE_DECAY_TAU_S:-0.18}"
TERMINAL_YAW_RATE_SCALE="${TERMINAL_YAW_RATE_SCALE:-0.70}"
INTRUDER_ACTOR_ASSET="${INTRUDER_ACTOR_ASSET:-Quadrotor1}"
INTRUDER_ACTOR_SCALE="${INTRUDER_ACTOR_SCALE:-1.0}"
INTRUDER_ACTOR_SCALE_X="${INTRUDER_ACTOR_SCALE_X:-}"
INTRUDER_ACTOR_SCALE_Y="${INTRUDER_ACTOR_SCALE_Y:-}"
INTRUDER_ACTOR_SCALE_Z="${INTRUDER_ACTOR_SCALE_Z:-}"
SETTINGS_PATH="${SETTINGS_PATH:-$SCRIPT_DIR/config/airsim_blocks_px4_actor_settings.json}"
LOG_DIR="$SCRIPT_DIR/logs/strict_reset"
TRAJECTORY_DIR="$SCRIPT_DIR/logs/yolo_sitl_ttc_vm"
REPORT_PATH="${REPORT_PATH:-$SCRIPT_DIR/完整方案/YOLO_SITL_AccelPNG_TTC_VM拦截对比报告.md}"
ASSET_DIR="${ASSET_DIR:-$SCRIPT_DIR/完整方案/assets/YOLO_SITL_AccelPNG_TTC_VM拦截对比报告}"
REPORT_TITLE="${REPORT_TITLE:-YOLO + ByteTrack PX4 SITL 加速度过载 PNG TTC / V_m 拦截对比报告}"
RANGE_NOTE="${RANGE_NOTE:-两组均测试 50m、60m、70m、80m、90m、100m，每个工况重启 PX4 SITL 和 Blocks。}"

if [[ "$TERMINAL_PROFILE" == "terminal_v2" ]]; then
  [[ -z "$USER_SHADOW_AIRSIM_DETECT_SET" ]] && SHADOW_AIRSIM_DETECT=0
  [[ -z "$USER_TERMINAL_IMAGE_KF_MAX_PREDICT_S_SET" ]] && TERMINAL_IMAGE_KF_MAX_PREDICT_S=0.55
  [[ -z "$USER_TERMINAL_IMAGE_KF_ACCEL_NOISE_RAD_S2_SET" ]] && TERMINAL_IMAGE_KF_ACCEL_NOISE_RAD_S2=12.0
  [[ -z "$USER_TERMINAL_IMAGE_KF_INNOVATION_REJECT_RAD_SET" ]] && TERMINAL_IMAGE_KF_INNOVATION_REJECT_RAD=0.35
  [[ -z "$USER_TERMINAL_IMAGE_KF_MAX_RATE_RAD_S_SET" ]] && TERMINAL_IMAGE_KF_MAX_RATE_RAD_S=12.0
  [[ -z "$USER_TERMINAL_IMAGE_KF_SOFT_REJECT_PREDICT_SET" ]] && TERMINAL_IMAGE_KF_SOFT_REJECT_PREDICT=1
  [[ -z "$USER_FRAME_CENTERING_ENTER_ERROR_RATIO_SET" ]] && FRAME_CENTERING_ENTER_ERROR_RATIO=0.40
  [[ -z "$USER_FRAME_CENTERING_TERMINAL_ERROR_RATIO_SET" ]] && FRAME_CENTERING_TERMINAL_ERROR_RATIO=0.55
  [[ -z "$USER_FRAME_CENTERING_AREA_RATIO_SET" ]] && FRAME_CENTERING_AREA_RATIO=0.004
  [[ -z "$USER_FRAME_CENTERING_LOSS_HOLD_S_SET" ]] && FRAME_CENTERING_LOSS_HOLD_S=1.20
  [[ -z "$USER_FRAME_CENTERING_SPEED_RATIO_SET" ]] && FRAME_CENTERING_SPEED_RATIO=1.10
  [[ -z "$USER_TERMINAL_CAPTURE_SPEED_RATIO_SET" ]] && TERMINAL_CAPTURE_SPEED_RATIO=0.85
  [[ -z "$USER_FRAME_CENTERING_LATERAL_SCALE_SET" ]] && FRAME_CENTERING_LATERAL_SCALE=0.12
  [[ -z "$USER_TERMINAL_CAPTURE_LATERAL_SCALE_SET" ]] && TERMINAL_CAPTURE_LATERAL_SCALE=0.02
  [[ -z "$USER_FRAME_CENTERING_MAX_LATERAL_SPEED_SET" ]] && FRAME_CENTERING_MAX_LATERAL_SPEED=0.80
  [[ -z "$USER_TERMINAL_CAPTURE_MAX_LATERAL_SPEED_SET" ]] && TERMINAL_CAPTURE_MAX_LATERAL_SPEED=0.25
  [[ -z "$USER_FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY_SET" ]] && FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY=1
  [[ -z "$USER_ATTITUDE_MAX_TILT_DEG_SET" ]] && ATTITUDE_MAX_TILT_DEG=35
  [[ -z "$USER_ATTITUDE_SPEED_HOLD_MAX_ACCEL_MPS2_SET" ]] && ATTITUDE_SPEED_HOLD_MAX_ACCEL_MPS2=4.0
  [[ -z "$USER_ATTITUDE_TOTAL_ACCEL_LIMIT_MPS2_SET" ]] && ATTITUDE_TOTAL_ACCEL_LIMIT_MPS2=24.0
fi

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
  AIRSIM_RUNTIME_ENV="$LOG_DIR/airsim_${run_tag}.env"
  CURRENT_AIRSIM_SETTINGS_PATH="$SETTINGS_PATH"

  stop_sim
  echo "Starting PX4 SITL for ${run_tag}"
  script -q -f -c "$SCRIPT_DIR/run_px4_sitl.sh" "$PX4_LOG" >/dev/null 2>&1 &
  PX4_PID=$!
  wait_for_px4 "$PX4_LOG"

  echo "Starting Blocks for ${run_tag}"
  AIRSIM_PORT_ENV_PATH="$AIRSIM_RUNTIME_ENV" AIRSIM_INSTANCE_LABEL="$run_tag" SETTINGS_PATH="$SETTINGS_PATH" "$SCRIPT_DIR/run_blocks_px4_actor.sh" >"$BLOCKS_LOG" 2>&1 &
  BLOCKS_PID=$!
  wait_for_airsim "$AIRSIM_RUNTIME_ENV"
  if [[ -f "$AIRSIM_RUNTIME_ENV" ]]; then
    set -a
    # shellcheck disable=SC1090
    source "$AIRSIM_RUNTIME_ENV"
    set +a
    CURRENT_AIRSIM_SETTINGS_PATH="${AIRSIM_SETTINGS_PATH_RESOLVED:-$SETTINGS_PATH}"
    echo "Resolved AirSim RPC: ${AIRSIM_RPC_HOST:-127.0.0.2}:${AIRSIM_RPC_PORT:-41451}"
    echo "Resolved AirSim settings: $CURRENT_AIRSIM_SETTINGS_PATH"
  fi
}

duration_for_range() {
  local range_m="$1"
  python3 - "$range_m" "$DURATION_MIN_S" "$DURATION_MAX_S" "$DURATION_RANGE_SCALE" "$DURATION_RANGE_OFFSET" <<'PY'
import sys
r = float(sys.argv[1])
minimum = float(sys.argv[2])
maximum = float(sys.argv[3])
scale = float(sys.argv[4])
offset = float(sys.argv[5])
print(f"{max(minimum, min(maximum, r * scale + offset)):.1f}")
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
  local image_kf_soft_reject_args=(--no-terminal-image-kf-soft-reject-predict)
  if [[ "$TERMINAL_IMAGE_KF_SOFT_REJECT_PREDICT" == "1" || "$TERMINAL_IMAGE_KF_SOFT_REJECT_PREDICT" == "true" || "$TERMINAL_IMAGE_KF_SOFT_REJECT_PREDICT" == "TRUE" ]]; then
    image_kf_soft_reject_args=(--terminal-image-kf-soft-reject-predict)
  fi
  local terminal_extrapolation_args=(--no-terminal-extrapolation)
  if [[ "$TERMINAL_EXTRAPOLATION" == "1" || "$TERMINAL_EXTRAPOLATION" == "true" || "$TERMINAL_EXTRAPOLATION" == "TRUE" ]]; then
    terminal_extrapolation_args=(--terminal-extrapolation)
  fi
  local terminal_abort_args=(--no-terminal-abort-on-tilt-hardcap)
  if [[ "$TERMINAL_ABORT_ON_TILT_HARDCAP" == "1" || "$TERMINAL_ABORT_ON_TILT_HARDCAP" == "true" || "$TERMINAL_ABORT_ON_TILT_HARDCAP" == "TRUE" ]]; then
    terminal_abort_args=(--terminal-abort-on-tilt-hardcap)
  fi
  local terminal_yaw_rate_args=(--no-terminal-yaw-rate-extrapolation)
  if [[ "$TERMINAL_YAW_RATE_EXTRAPOLATION" == "1" || "$TERMINAL_YAW_RATE_EXTRAPOLATION" == "true" || "$TERMINAL_YAW_RATE_EXTRAPOLATION" == "TRUE" ]]; then
    terminal_yaw_rate_args=(--terminal-yaw-rate-extrapolation)
  fi
  local terminal_velocity_blind_push_args=(--no-terminal-velocity-blind-push)
  if [[ "$TERMINAL_VELOCITY_BLIND_PUSH" == "1" || "$TERMINAL_VELOCITY_BLIND_PUSH" == "true" || "$TERMINAL_VELOCITY_BLIND_PUSH" == "TRUE" ]]; then
    terminal_velocity_blind_push_args=(--terminal-velocity-blind-push)
  fi
  local terminal_accel_hold_args=(--no-terminal-accel-hold)
  if [[ "$TERMINAL_ACCEL_HOLD" == "1" || "$TERMINAL_ACCEL_HOLD" == "true" || "$TERMINAL_ACCEL_HOLD" == "TRUE" ]]; then
    terminal_accel_hold_args=(--terminal-accel-hold)
  fi
  local top_clip_args=(--no-reject-top-clipped-pitch)
  if [[ "$REJECT_TOP_CLIPPED_PITCH" == "1" || "$REJECT_TOP_CLIPPED_PITCH" == "true" || "$REJECT_TOP_CLIPPED_PITCH" == "TRUE" ]]; then
    top_clip_args=(--reject-top-clipped-pitch)
  fi
  local accel_reset_args=(--no-accel-integral-reset-on-invalid)
  if [[ "$ACCEL_INTEGRAL_RESET_ON_INVALID" == "1" || "$ACCEL_INTEGRAL_RESET_ON_INVALID" == "true" || "$ACCEL_INTEGRAL_RESET_ON_INVALID" == "TRUE" ]]; then
    accel_reset_args=(--accel-integral-reset-on-invalid)
  fi
  local kcf_reset_args=(--no-kcf-reset-on-yolo-drift)
  if [[ "$KCF_RESET_ON_YOLO_DRIFT" == "1" || "$KCF_RESET_ON_YOLO_DRIFT" == "true" || "$KCF_RESET_ON_YOLO_DRIFT" == "TRUE" ]]; then
    kcf_reset_args=(--kcf-reset-on-yolo-drift)
  fi
  local upward_centering_args=(--no-upward-centering)
  if [[ "$UPWARD_CENTERING" == "1" || "$UPWARD_CENTERING" == "true" || "$UPWARD_CENTERING" == "TRUE" ]]; then
    upward_centering_args=(--upward-centering)
  fi
  local frame_centering_args=(--no-frame-centering)
  if [[ "$FRAME_CENTERING" == "1" || "$FRAME_CENTERING" == "true" || "$FRAME_CENTERING" == "TRUE" ]]; then
    frame_centering_args=(
      --frame-centering
      --frame-centering-enter-error-ratio "$FRAME_CENTERING_ENTER_ERROR_RATIO"
      --frame-centering-terminal-error-ratio "$FRAME_CENTERING_TERMINAL_ERROR_RATIO"
      --frame-centering-area-ratio "$FRAME_CENTERING_AREA_RATIO"
      --frame-centering-loss-hold-s "$FRAME_CENTERING_LOSS_HOLD_S"
      --frame-centering-speed-ratio "$FRAME_CENTERING_SPEED_RATIO"
      --terminal-capture-speed-ratio "$TERMINAL_CAPTURE_SPEED_RATIO"
      --frame-centering-min-forward-ratio "$FRAME_CENTERING_MIN_FORWARD_RATIO"
      --frame-centering-max-lateral-speed "$FRAME_CENTERING_MAX_LATERAL_SPEED"
      --terminal-capture-max-lateral-speed "$TERMINAL_CAPTURE_MAX_LATERAL_SPEED"
      --frame-centering-lateral-scale "$FRAME_CENTERING_LATERAL_SCALE"
      --terminal-capture-lateral-scale "$TERMINAL_CAPTURE_LATERAL_SCALE"
      --frame-centering-yaw-gain-scale "$FRAME_CENTERING_YAW_GAIN_SCALE"
      --terminal-capture-yaw-gain-scale "$TERMINAL_CAPTURE_YAW_GAIN_SCALE"
      --frame-centering-max-yaw-rate-deg "$FRAME_CENTERING_MAX_YAW_RATE_DEG"
      --terminal-capture-max-yaw-rate-deg "$TERMINAL_CAPTURE_MAX_YAW_RATE_DEG"
      --frame-centering-yaw-hold-window-s "$FRAME_CENTERING_YAW_HOLD_WINDOW_S"
      --frame-centering-yaw-hold-decay-tau-s "$FRAME_CENTERING_YAW_HOLD_DECAY_TAU_S"
    )
    if [[ "$FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY" == "1" || "$FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY" == "true" || "$FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY" == "TRUE" ]]; then
      frame_centering_args+=(--frame-centering-loss-hold-last-velocity)
    else
      frame_centering_args+=(--no-frame-centering-loss-hold-last-velocity)
    fi
  fi
  local actor_scale_args=(--intruder-actor-scale "$INTRUDER_ACTOR_SCALE")
  if [[ -n "$INTRUDER_ACTOR_SCALE_X" ]]; then
    actor_scale_args+=(--intruder-actor-scale-x "$INTRUDER_ACTOR_SCALE_X")
  fi
  if [[ -n "$INTRUDER_ACTOR_SCALE_Y" ]]; then
    actor_scale_args+=(--intruder-actor-scale-y "$INTRUDER_ACTOR_SCALE_Y")
  fi
  if [[ -n "$INTRUDER_ACTOR_SCALE_Z" ]]; then
    actor_scale_args+=(--intruder-actor-scale-z "$INTRUDER_ACTOR_SCALE_Z")
  fi

  local rc=0
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
    --near-hit-radius-m "$NEAR_HIT_RADIUS_M" \
    --thrust-model "$THRUST_MODEL" \
    --vehicle-mass-kg "$VEHICLE_MASS_KG" \
    --vehicle-max-total-thrust-n "$VEHICLE_MAX_TOTAL_THRUST_N" \
    --body-rate-max-tilt-deg "$BODY_RATE_MAX_TILT_DEG" \
    --body-rate-roll-gain "$BODY_RATE_ROLL_GAIN" \
    --body-rate-pitch-gain "$BODY_RATE_PITCH_GAIN" \
    --body-rate-attitude-p "$BODY_RATE_ATTITUDE_P" \
    --body-rate-max-roll-rate-deg "$BODY_RATE_MAX_ROLL_RATE_DEG" \
    --body-rate-max-pitch-rate-deg "$BODY_RATE_MAX_PITCH_RATE_DEG" \
    --body-rate-control-profile "$BODY_RATE_CONTROL_PROFILE" \
    --body-rate-v2-kp-roll "$BODY_RATE_V2_KP_ROLL" \
    --body-rate-v2-kp-pitch "$BODY_RATE_V2_KP_PITCH" \
    --body-rate-v2-kp-yaw "$BODY_RATE_V2_KP_YAW" \
    --body-rate-v2-max-pq-rate-deg-s "$BODY_RATE_V2_MAX_PQ_RATE_DEG_S" \
    --body-rate-v2-slew-pq-deg-s2 "$BODY_RATE_V2_SLEW_PQ_DEG_S2" \
    --body-rate-v2-slew-r-deg-s2 "$BODY_RATE_V2_SLEW_R_DEG_S2" \
    --body-rate-v2-thrust-reserve "$BODY_RATE_V2_THRUST_RESERVE" \
    --body-rate-v2-guard-error-ratio "$BODY_RATE_V2_GUARD_ERROR_RATIO" \
    --body-rate-v2-guard-png-scale "$BODY_RATE_V2_GUARD_PNG_SCALE" \
    --body-rate-v2-guard-speed-hold-scale "$BODY_RATE_V2_GUARD_SPEED_HOLD_SCALE" \
    --body-rate-hover-thrust "$BODY_RATE_HOVER_THRUST" \
    --body-rate-thrust-gain "$BODY_RATE_THRUST_GAIN" \
    --body-rate-min-thrust "$BODY_RATE_MIN_THRUST" \
    --body-rate-max-thrust "$BODY_RATE_MAX_THRUST" \
    --body-rate-speed-hold-gain "$BODY_RATE_SPEED_HOLD_GAIN" \
    --body-rate-speed-hold-max-accel-mps2 "$BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2" \
    --body-rate-total-accel-limit-mps2 "$BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2" \
    --attitude-max-tilt-deg "$ATTITUDE_MAX_TILT_DEG" \
    --attitude-yaw-lookahead-s "$ATTITUDE_YAW_LOOKAHEAD_S" \
    --attitude-hover-thrust "$ATTITUDE_HOVER_THRUST" \
    --attitude-thrust-gain "$ATTITUDE_THRUST_GAIN" \
    --attitude-min-thrust "$ATTITUDE_MIN_THRUST" \
    --attitude-max-thrust "$ATTITUDE_MAX_THRUST" \
    --attitude-speed-hold-gain "$ATTITUDE_SPEED_HOLD_GAIN" \
    --attitude-speed-hold-max-accel-mps2 "$ATTITUDE_SPEED_HOLD_MAX_ACCEL_MPS2" \
    --attitude-total-accel-limit-mps2 "$ATTITUDE_TOTAL_ACCEL_LIMIT_MPS2" \
    --intercept-altitude-m "$INTERCEPT_ALTITUDE_M" \
    --intruder-altitude-offset-m "$ALTITUDE_OFFSET" \
    --start-horizontal-range-m "$range_m" \
    --start-lateral-offset-m "$START_LATERAL_OFFSET" \
    --px4-max-vertical-speed "$PX4_MAX_VERTICAL_SPEED" \
    --max-vision-lateral-speed "$MAX_VISION_LATERAL_SPEED" \
    --max-vision-vertical-speed "$MAX_VISION_VERTICAL_SPEED" \
    --trajectory-dir "$TRAJECTORY_DIR" \
    --trajectory-prefix "$prefix" \
    --settings-path "${CURRENT_AIRSIM_SETTINGS_PATH:-$SETTINGS_PATH}" \
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
    "${actor_scale_args[@]}" \
    --intruder-actor-respawn \
    --mesh 'IntruderActor' \
    --detector-source "$DETECTOR_SOURCE" \
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
    --kcf-yolo-period-n "$KCF_YOLO_PERIOD_N" \
    --kcf-yolo-period-s "$KCF_YOLO_PERIOD_S" \
    --kcf-max-coast-s "$KCF_MAX_COAST_S" \
    --kcf-min-yolo-iou "$KCF_MIN_YOLO_IOU" \
    --kcf-max-center-jump-px "$KCF_MAX_CENTER_JUMP_PX" \
    --kcf-area-ratio-min "$KCF_AREA_RATIO_MIN" \
    --kcf-area-ratio-max "$KCF_AREA_RATIO_MAX" \
    "${kcf_reset_args[@]}" \
    "${shadow_args[@]}" \
    "${los_filter_args[@]}" \
    --los-filter-process-lambda "$LOS_FILTER_PROCESS_LAMBDA" \
    --los-filter-process-lambda-dot "$LOS_FILTER_PROCESS_LAMBDA_DOT" \
    --los-filter-measurement-noise "$LOS_FILTER_MEASUREMENT_NOISE" \
    --los-filter-innovation-reject "$LOS_FILTER_INNOVATION_REJECT" \
    --los-filter-terminal-innovation-reject "$LOS_FILTER_TERMINAL_INNOVATION_REJECT" \
    --los-filter-terminal-area-ratio "$LOS_FILTER_TERMINAL_AREA_RATIO" \
    --los-filter-terminal-error-ratio "$LOS_FILTER_TERMINAL_ERROR_RATIO" \
    --los-delay-compensation-s "$LOS_DELAY_COMPENSATION_S" \
    --frame-guard \
    --frame-guard-enter-error-ratio "$FRAME_GUARD_ENTER_ERROR_RATIO" \
    --frame-guard-exit-error-ratio "$FRAME_GUARD_EXIT_ERROR_RATIO" \
    --frame-guard-area-mid-ratio "$FRAME_GUARD_AREA_MID_RATIO" \
    --frame-guard-area-ratio "$FRAME_GUARD_AREA_RATIO" \
    --frame-guard-ttc-mid-s "$FRAME_GUARD_TTC_MID_S" \
    --frame-guard-ttc-terminal-s "$FRAME_GUARD_TTC_TERMINAL_S" \
    --frame-guard-hold-s "$FRAME_GUARD_HOLD_S" \
    --frame-guard-min-speed-ratio "$FRAME_GUARD_MIN_SPEED_RATIO" \
    --frame-guard-mid-speed-ratio "$FRAME_GUARD_MID_SPEED_RATIO" \
    --frame-guard-lateral-scale "$FRAME_GUARD_LATERAL_SCALE" \
    --frame-guard-vertical-scale "$FRAME_GUARD_VERTICAL_SCALE" \
    --frame-guard-yaw-gain-scale "$FRAME_GUARD_YAW_GAIN_SCALE" \
    --frame-guard-max-yaw-rate-deg "$FRAME_GUARD_MAX_YAW_RATE_DEG" \
    --frame-guard-vertical-error-gain "$FRAME_GUARD_VERTICAL_ERROR_GAIN" \
    "${frame_centering_args[@]}" \
    --yaw-control \
    --yaw-error-gain "$YAW_ERROR_GAIN" \
    --max-yaw-rate-deg "$MAX_YAW_RATE_DEG" \
    --ttc-soft-guidance \
    "${terminal_extrapolation_args[@]}" \
    --terminal-enter-area-ratio "$TERMINAL_ENTER_AREA_RATIO" \
    --terminal-soft-enter-area-ratio "$TERMINAL_SOFT_ENTER_AREA_RATIO" \
    --terminal-cutoff-area-ratio "$TERMINAL_CUTOFF_AREA_RATIO" \
    --terminal-gimbal-limit-area-ratio "$TERMINAL_GIMBAL_LIMIT_AREA_RATIO" \
    --terminal-cutoff-miss-count "$TERMINAL_CUTOFF_MISS_COUNT" \
    --terminal-min-tracking-time-s "$TERMINAL_MIN_TRACKING_TIME_S" \
    --terminal-confidence-min-score "$TERMINAL_CONFIDENCE_MIN_SCORE" \
    --terminal-max-measurement-age-s "$TERMINAL_MAX_MEASUREMENT_AGE_S" \
    --terminal-blind-duration-s "$TERMINAL_BLIND_DURATION_S" \
    --terminal-command-average-window-s "$TERMINAL_COMMAND_AVERAGE_WINDOW_S" \
    --terminal-command-decay-tau-s "$TERMINAL_COMMAND_DECAY_TAU_S" \
    "${terminal_velocity_blind_push_args[@]}" \
    "${terminal_accel_hold_args[@]}" \
    --terminal-accel-hold-window-s "$TERMINAL_ACCEL_HOLD_WINDOW_S" \
    --terminal-accel-decay-tau-s "$TERMINAL_ACCEL_DECAY_TAU_S" \
    --terminal-accel-hold-max-mps2 "$TERMINAL_ACCEL_HOLD_MAX_MPS2" \
    --terminal-trend-bias-gain "$TERMINAL_TREND_BIAS_GAIN" \
    --terminal-trend-bias-max-mps "$TERMINAL_TREND_BIAS_MAX_MPS" \
    --terminal-pitch-up-bias-mps "$TERMINAL_PITCH_UP_BIAS_MPS" \
    "${terminal_abort_args[@]}" \
    "${terminal_yaw_rate_args[@]}" \
    --terminal-yaw-rate-average-window-s "$TERMINAL_YAW_RATE_AVERAGE_WINDOW_S" \
    --terminal-yaw-rate-decay-tau-s "$TERMINAL_YAW_RATE_DECAY_TAU_S" \
    --terminal-yaw-rate-scale "$TERMINAL_YAW_RATE_SCALE" \
    --terminal-image-kf \
    --terminal-image-kf-max-predict-s "$TERMINAL_IMAGE_KF_MAX_PREDICT_S" \
    --terminal-image-kf-meas-noise-rad "$TERMINAL_IMAGE_KF_MEAS_NOISE_RAD" \
    --terminal-image-kf-accel-noise-rad-s2 "$TERMINAL_IMAGE_KF_ACCEL_NOISE_RAD_S2" \
    --terminal-image-kf-innovation-reject-rad "$TERMINAL_IMAGE_KF_INNOVATION_REJECT_RAD" \
    --terminal-image-kf-max-angle-rad "$TERMINAL_IMAGE_KF_MAX_ANGLE_RAD" \
    --terminal-image-kf-max-rate-rad-s "$TERMINAL_IMAGE_KF_MAX_RATE_RAD_S" \
    "${image_kf_soft_reject_args[@]}" \
    --terminal-image-kf-guidance \
    --climb-timeout-s 90 \
    --no-px4-command-join \
    --px4-command-mode "$PX4_COMMAND_MODE" \
    --min-command-duration-s 0.12 \
    --command-duration-margin-s 0.04 \
    --max-command-duration-s 0.25 \
    "${upward_centering_args[@]}" \
    --upward-centering-gain "$UPWARD_CENTERING_GAIN" \
    --upward-centering-max-accel-mps2 "$UPWARD_CENTERING_MAX_ACCEL_MPS2" \
    --camera-x "$CAMERA_X" \
    --camera-y "$CAMERA_Y" \
    --camera-z "$CAMERA_Z" \
    --camera-pitch-deg "$CAMERA_PITCH_DEG" \
    --camera-roll-deg "$CAMERA_ROLL_DEG" \
    --camera-yaw-deg "$CAMERA_YAW_DEG" \
    "${top_clip_args[@]}" &
  local case_pid=$!
  local case_timeout_s_int
  case_timeout_s_int="$(python3 - "$CASE_TIMEOUT_S" <<'PY'
import math
import sys
print(max(1, int(math.ceil(float(sys.argv[1])))))
PY
)"
  local deadline=$((SECONDS + case_timeout_s_int))
  local blocks_crashed=0
  while kill -0 "$case_pid" 2>/dev/null; do
    if ! ps -p "$BLOCKS_PID" >/dev/null 2>&1; then
      blocks_crashed=1
      rc=125
      kill "$case_pid" 2>/dev/null || true
      break
    fi
    if (( SECONDS >= deadline )); then
      rc=124
      kill "$case_pid" 2>/dev/null || true
      break
    fi
    sleep 1
  done
  if [[ "$rc" -eq 124 || "$rc" -eq 125 ]]; then
    sleep 2
    kill -9 "$case_pid" 2>/dev/null || true
  fi
  local wait_rc=0
  wait "$case_pid" || wait_rc=$?
  if [[ "$wait_rc" -ne 0 && "$rc" -eq 0 ]]; then
    rc="$wait_rc"
  fi
  if [[ "$blocks_crashed" -eq 1 ]]; then
    echo "case_failed label=${label} range=${range_m}m rc=${rc} reason=blocks_crash log=${BLOCKS_LOG}" >&2
  fi

  stop_sim
  if [[ "$rc" -ne 0 ]]; then
    echo "case_failed label=${label} range=${range_m}m rc=${rc} timeout_s=${CASE_TIMEOUT_S}" >&2
  fi
  return 0
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
echo "Run groups: TTC=${RUN_TTC}; VM=${RUN_VM}"
echo "Detector: source=${DETECTOR_SOURCE}, model=${YOLO_MODEL}, device=${YOLO_DEVICE}, conf=${YOLO_CONF}, tracker=bytetrack.yaml"
echo "KCF: period_n=${KCF_YOLO_PERIOD_N}, period_s=${KCF_YOLO_PERIOD_S}, max_coast=${KCF_MAX_COAST_S}, min_iou=${KCF_MIN_YOLO_IOU}, center_jump=${KCF_MAX_CENTER_JUMP_PX}, area_ratio=${KCF_AREA_RATIO_MIN}/${KCF_AREA_RATIO_MAX}, reset_on_drift=${KCF_RESET_ON_YOLO_DRIFT}"
echo "Actor: ${INTRUDER_ACTOR_ASSET}, scale=${INTRUDER_ACTOR_SCALE}, scale_xyz=${INTRUDER_ACTOR_SCALE_X:-base}/${INTRUDER_ACTOR_SCALE_Y:-base}/${INTRUDER_ACTOR_SCALE_Z:-base}"
echo "PX4 command mode: ${PX4_COMMAND_MODE}"
echo "Guidance output: ${GUIDANCE_OUTPUT_MODE}; max_accel=${MAX_GUIDANCE_ACCEL_MPS2}; min_speed_ratio=${MIN_SPEED_RATIO}; reset_on_invalid=${ACCEL_INTEGRAL_RESET_ON_INVALID}"
echo "Near-hit radius: ${NEAR_HIT_RADIUS_M}m"
echo "Terminal profile: ${TERMINAL_PROFILE}; image_kf predict=${TERMINAL_IMAGE_KF_MAX_PREDICT_S}s reject=${TERMINAL_IMAGE_KF_INNOVATION_REJECT_RAD}rad soft_reject=${TERMINAL_IMAGE_KF_SOFT_REJECT_PREDICT} max_rate=${TERMINAL_IMAGE_KF_MAX_RATE_RAD_S}rad/s"
echo "Terminal extrapolation: ${TERMINAL_EXTRAPOLATION}; enter=${TERMINAL_ENTER_AREA_RATIO}; soft=${TERMINAL_SOFT_ENTER_AREA_RATIO}; cutoff=${TERMINAL_CUTOFF_AREA_RATIO}; miss=${TERMINAL_CUTOFF_MISS_COUNT}; blind=${TERMINAL_BLIND_DURATION_S}s"
echo "Terminal strapdown extrapolation: velocity_blind_push=${TERMINAL_VELOCITY_BLIND_PUSH}; accel_hold=${TERMINAL_ACCEL_HOLD}; accel_window=${TERMINAL_ACCEL_HOLD_WINDOW_S}s; accel_decay=${TERMINAL_ACCEL_DECAY_TAU_S}s; accel_max=${TERMINAL_ACCEL_HOLD_MAX_MPS2}"
echo "Terminal yaw-rate: extrapolation=${TERMINAL_YAW_RATE_EXTRAPOLATION}; scale=${TERMINAL_YAW_RATE_SCALE}; window=${TERMINAL_YAW_RATE_AVERAGE_WINDOW_S}s; decay=${TERMINAL_YAW_RATE_DECAY_TAU_S}s"
echo "Thrust model: ${THRUST_MODEL}; mass=${VEHICLE_MASS_KG}kg; max_total_thrust=${VEHICLE_MAX_TOTAL_THRUST_N}N"
echo "Body-rate: profile=${BODY_RATE_CONTROL_PROFILE}; tilt=${BODY_RATE_MAX_TILT_DEG}deg rates=${BODY_RATE_MAX_ROLL_RATE_DEG}/${BODY_RATE_MAX_PITCH_RATE_DEG}deg/s thrust=${BODY_RATE_MIN_THRUST}/${BODY_RATE_HOVER_THRUST}/${BODY_RATE_MAX_THRUST}"
echo "Body-rate v2: kp=${BODY_RATE_V2_KP_ROLL}/${BODY_RATE_V2_KP_PITCH}/${BODY_RATE_V2_KP_YAW}; max_pq=${BODY_RATE_V2_MAX_PQ_RATE_DEG_S}deg/s; slew=${BODY_RATE_V2_SLEW_PQ_DEG_S2}/${BODY_RATE_V2_SLEW_R_DEG_S2}deg/s^2; reserve=${BODY_RATE_V2_THRUST_RESERVE}; guard=${BODY_RATE_V2_GUARD_ERROR_RATIO}/${BODY_RATE_V2_GUARD_PNG_SCALE}/${BODY_RATE_V2_GUARD_SPEED_HOLD_SCALE}"
echo "Body-rate speed hold: gain=${BODY_RATE_SPEED_HOLD_GAIN}; max_accel=${BODY_RATE_SPEED_HOLD_MAX_ACCEL_MPS2}; total_limit=${BODY_RATE_TOTAL_ACCEL_LIMIT_MPS2}"
echo "Attitude: tilt=${ATTITUDE_MAX_TILT_DEG}deg yaw_lookahead=${ATTITUDE_YAW_LOOKAHEAD_S}s thrust=${ATTITUDE_MIN_THRUST}/${ATTITUDE_HOVER_THRUST}/${ATTITUDE_MAX_THRUST}"
echo "Attitude speed hold: gain=${ATTITUDE_SPEED_HOLD_GAIN}; max_accel=${ATTITUDE_SPEED_HOLD_MAX_ACCEL_MPS2}; total_limit=${ATTITUDE_TOTAL_ACCEL_LIMIT_MPS2}"
echo "Camera: x=${CAMERA_X}, y=${CAMERA_Y}, z=${CAMERA_Z}, pitch=${CAMERA_PITCH_DEG}, roll=${CAMERA_ROLL_DEG}, yaw=${CAMERA_YAW_DEG}"
echo "Upward centering: ${UPWARD_CENTERING}; gain=${UPWARD_CENTERING_GAIN}; max_accel=${UPWARD_CENTERING_MAX_ACCEL_MPS2}"
echo "Shadow AirSim detect: ${SHADOW_AIRSIM_DETECT}; LOS filter: ${LOS_FILTER}; LOS KF q=${LOS_FILTER_PROCESS_LAMBDA}/${LOS_FILTER_PROCESS_LAMBDA_DOT}, r=${LOS_FILTER_MEASUREMENT_NOISE}, reject=${LOS_FILTER_INNOVATION_REJECT}, terminal_reject=${LOS_FILTER_TERMINAL_INNOVATION_REJECT}; delay=${LOS_DELAY_COMPENSATION_S}s; reject top clipped pitch: ${REJECT_TOP_CLIPPED_PITCH}; yolo-imgsz=${YOLO_IMGSZ}"
echo "Frame centering: ${FRAME_CENTERING}; enter=${FRAME_CENTERING_ENTER_ERROR_RATIO}; terminal=${FRAME_CENTERING_TERMINAL_ERROR_RATIO}; loss_hold=${FRAME_CENTERING_LOSS_HOLD_S}s; speed_ratio=${FRAME_CENTERING_SPEED_RATIO}; terminal_speed_ratio=${TERMINAL_CAPTURE_SPEED_RATIO}; lateral=${FRAME_CENTERING_MAX_LATERAL_SPEED}/${TERMINAL_CAPTURE_MAX_LATERAL_SPEED}"
echo "Frame centering loss hold last velocity: ${FRAME_CENTERING_LOSS_HOLD_LAST_VELOCITY}"
echo "Case timeout: ${CASE_TIMEOUT_S}s"
echo "Every case restarts PX4 SITL and Blocks."

if [[ "$RUN_TTC" == "1" || "$RUN_TTC" == "true" || "$RUN_TTC" == "TRUE" ]]; then
  for range_m in "${RANGES[@]}"; do
    run_case TTC ttc_png "$range_m"
  done
  summarize_label TTC
fi

if [[ "$RUN_VM" == "1" || "$RUN_VM" == "true" || "$RUN_VM" == "TRUE" ]]; then
  for range_m in "${RANGES[@]}"; do
    run_case VM fixed_vm_png "$range_m"
  done
  summarize_label VM
fi

python3 examples/generate_yolo_sitl_ttc_vm_report.py \
  --stamp "$STAMP" \
  --report-path "$REPORT_PATH" \
  --asset-dir "$ASSET_DIR" \
  --title "$REPORT_TITLE" \
  --range-note "$RANGE_NOTE"

echo
echo "yolo_sitl_stamp=${STAMP}"
echo "report=${REPORT_PATH}"
