#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MATRIX_STAMP="${MATRIX_STAMP:-upward_yolo_matrix15_$(date +%Y%m%d_%H%M%S)}"
MANIFEST_PATH="${MANIFEST_PATH:-$SCRIPT_DIR/logs/yolo_sitl_ttc_vm/${MATRIX_STAMP}_manifest.csv}"
REPORT_PATH="${REPORT_PATH:-$SCRIPT_DIR/完整方案/YOLO_ByteTrack_upward_matrix15_多工况性能测试报告.md}"
ASSET_DIR="${ASSET_DIR:-$SCRIPT_DIR/完整方案/assets/YOLO_ByteTrack_upward_matrix15_多工况性能测试报告}"
CASE_REPORT_DIR="${CASE_REPORT_DIR:-$SCRIPT_DIR/logs/yolo_sitl_ttc_vm/${MATRIX_STAMP}_case_reports}"

export AIRSIM_RPC_HOST="${AIRSIM_RPC_HOST:-127.0.0.2}"
export AIRSIM_REWRITE_HOST_IPS="${AIRSIM_REWRITE_HOST_IPS:-0}"
export AIRSIM_PORT_POLICY="${AIRSIM_PORT_POLICY:-strict}"

mkdir -p "$(dirname "$MANIFEST_PATH")" "$CASE_REPORT_DIR" "$ASSET_DIR"

cat >"$MANIFEST_PATH" <<'CSV'
case_id,start_horizontal_range_m,start_lateral_offset_m,altitude_offset_m,intruder_speed_mps,speed_ratio,stamp,status
CSV

check_ports() {
  ss -ltnup | rg ':(41451|41452|4560|14540|14541|14550|1455[0-9])\b' || true
  ps -eo pid,ppid,stat,etime,cmd | rg -i 'Blocks|PX4|sitl|run_yolo|run_upward|python.*run_airsim|mavsdk|micrortps' || true
}

run_matrix_case() {
  local case_id="$1"
  local range_m="$2"
  local lateral_m="$3"
  local altitude_m="$4"
  local intruder_speed="$5"
  local speed_ratio="$6"
  local stamp="${MATRIX_STAMP}_${case_id}"
  local status="ok"
  local report_path="$CASE_REPORT_DIR/${case_id}.md"
  local asset_dir="$CASE_REPORT_DIR/${case_id}_assets"

  echo
  echo "matrix_case=${case_id} stamp=${stamp} range=${range_m} lateral=${lateral_m} altitude=${altitude_m} intruder_speed=${intruder_speed} speed_ratio=${speed_ratio}"
  check_ports

  if ! STAMP="$stamp" \
    RANGES="$range_m" \
    START_LATERAL_OFFSET="$lateral_m" \
    ALTITUDE_OFFSET="$altitude_m" \
    INTRUDER_SPEED="$intruder_speed" \
    SPEED_RATIO="$speed_ratio" \
    RUN_TTC=1 \
    RUN_VM=1 \
    DETECTOR_SOURCE=yolo_bytetrack \
    SHADOW_AIRSIM_DETECT=0 \
    TERMINAL_BLIND_REQUIRES_VISUAL_LOSS=1 \
    TERMINAL_CLIPPED_LOS_KF_PREDICT=1 \
    REPORT_PATH="$report_path" \
    ASSET_DIR="$asset_dir" \
    REPORT_TITLE="YOLO+ByteTrack upward matrix ${case_id}" \
    RANGE_NOTE="Matrix case ${case_id}: range=${range_m}m, lateral=${lateral_m}m, altitude=${altitude_m}m, intruder_speed=${intruder_speed}m/s, speed_ratio=${speed_ratio}; AirSim detect shadow disabled." \
    bash "$SCRIPT_DIR/run_upward_body_rate_ttc_vm_smoke.sh"; then
    status="infra_failed"
  fi

  printf '%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$case_id" "$range_m" "$lateral_m" "$altitude_m" "$intruder_speed" "$speed_ratio" "$stamp" "$status" \
    >>"$MANIFEST_PATH"
  check_ports
}

echo "YOLO upward matrix stamp: $MATRIX_STAMP"
echo "manifest=$MANIFEST_PATH"
echo "report=$REPORT_PATH"
echo "asset_dir=$ASSET_DIR"
echo "Detector: YOLO+ByteTrack closed-loop; AirSim detect shadow disabled."

run_matrix_case M01 40 -10 30 5 2.0
run_matrix_case M02 25 -10 30 5 2.0
run_matrix_case M03 55 -10 30 5 2.0
run_matrix_case M04 40 0 30 5 2.0
run_matrix_case M05 40 -20 30 5 2.0
run_matrix_case M06 40 20 30 5 2.0
run_matrix_case M07 40 -10 20 5 2.0
run_matrix_case M08 40 -10 40 5 2.0
run_matrix_case M09 30 -10 30 3 2.0
run_matrix_case M10 30 -10 30 7 2.0
run_matrix_case M11 50 -10 30 3 2.0
run_matrix_case M12 50 -10 30 7 2.0
run_matrix_case M13 45 -20 40 7 2.0
run_matrix_case M14 55 15 20 7 2.0
run_matrix_case M15 25 20 40 3 2.0

python3 "$SCRIPT_DIR/examples/generate_yolo_matrix_report.py" \
  --manifest "$MANIFEST_PATH" \
  --report-path "$REPORT_PATH" \
  --asset-dir "$ASSET_DIR" \
  --title "YOLO+ByteTrack upward-camera matrix15 performance report"

echo
echo "matrix_stamp=$MATRIX_STAMP"
echo "manifest=$MANIFEST_PATH"
echo "report=$REPORT_PATH"
