#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

STAMP="${STAMP:-upward_baseline_s_maneuver_30_50_$(date +%Y%m%d_%H%M%S)}"
REPORT_BASENAME="YOLO_ByteTrack_upward_baseline_S机动_30_50测试报告"

export AIRSIM_RPC_HOST="${AIRSIM_RPC_HOST:-127.0.0.2}"
export AIRSIM_REWRITE_HOST_IPS="${AIRSIM_REWRITE_HOST_IPS:-0}"
export AIRSIM_PORT_POLICY="${AIRSIM_PORT_POLICY:-strict}"

export STAMP
export RANGES="${RANGES:-30 35 40 45 50}"
export RUN_TTC="${RUN_TTC:-1}"
export RUN_VM="${RUN_VM:-1}"
export DETECTOR_SOURCE="${DETECTOR_SOURCE:-yolo_bytetrack}"
export SHADOW_AIRSIM_DETECT="${SHADOW_AIRSIM_DETECT:-1}"
export INTRUDER_SPEED="${INTRUDER_SPEED:-5}"
export ALTITUDE_OFFSET="${ALTITUDE_OFFSET:-30}"
export START_LATERAL_OFFSET="${START_LATERAL_OFFSET:--10}"
export SPEED_RATIO="${SPEED_RATIO:-2.0}"

export INTRUDER_MANEUVER="${INTRUDER_MANEUVER:-sine_s}"
export INTRUDER_MANEUVER_AMPLITUDE_M="${INTRUDER_MANEUVER_AMPLITUDE_M:-4.0}"
export INTRUDER_MANEUVER_PERIOD_S="${INTRUDER_MANEUVER_PERIOD_S:-8.0}"
export INTRUDER_MANEUVER_PHASE_DEG="${INTRUDER_MANEUVER_PHASE_DEG:-0.0}"

export TERMINAL_BLIND_REQUIRES_VISUAL_LOSS="${TERMINAL_BLIND_REQUIRES_VISUAL_LOSS:-1}"
export TERMINAL_CLIPPED_LOS_KF_PREDICT="${TERMINAL_CLIPPED_LOS_KF_PREDICT:-1}"
export REPORT_PATH="${REPORT_PATH:-$SCRIPT_DIR/完整方案/${REPORT_BASENAME}.md}"
export ASSET_DIR="${ASSET_DIR:-$SCRIPT_DIR/完整方案/assets/${REPORT_BASENAME}}"
export REPORT_TITLE="${REPORT_TITLE:-YOLO+ByteTrack upward-camera baseline S-maneuver 30-50m report}"
export RANGE_NOTE="${RANGE_NOTE:-Baseline upward-camera YOLO+ByteTrack closed-loop with target S maneuver: ranges 30/35/40/45/50m, nominal target speed 5m/s, altitude offset 30m, lateral offset -10m. S maneuver is sine_s perpendicular to nominal target velocity, amplitude ${INTRUDER_MANEUVER_AMPLITUDE_M}m, period ${INTRUDER_MANEUVER_PERIOD_S}s. AirSim detect shadow is diagnostic only and does not enter the guidance loop. Collision is the success criterion.}"

echo "Pre-run AirSim/PX4 port check:"
ss -ltnup | rg ':(41451|41452|4560|14540|14541|14550|1455[0-9])\b' || true
ps -eo pid,ppid,stat,etime,cmd | rg -i 'Blocks|PX4|sitl|run_yolo|run_upward|python.*run_airsim|mavsdk|micrortps' || true

bash "$SCRIPT_DIR/run_upward_body_rate_ttc_vm_smoke.sh"

echo "Post-run AirSim/PX4 port check:"
ss -ltnup | rg ':(41451|41452|4560|14540|14541|14550|1455[0-9])\b' || true
ps -eo pid,ppid,stat,etime,cmd | rg -i 'Blocks|PX4|sitl|run_yolo|run_upward|python.*run_airsim|mavsdk|micrortps' || true
