#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

STAMP="${STAMP:-center_clock1_$(date +%Y%m%d_%H%M%S)}"
BASELINE_STAMP="${BASELINE_STAMP:-}"
PREFIX="strapdown_clock1_center_${STAMP}"
SUMMARY_CSV="logs/strapdown_accuracy/${PREFIX}_summary.csv"
TRUTH_PREFIX="strapdown_clock1_center_truth_N3_${STAMP}"
SETTINGS_PATH="$SCRIPT_DIR/config/airsim_blocks_settings_clock1_center_camera.json"

echo "AirSim Blocks must already be running with ${SETTINGS_PATH}."
echo "Experiment stamp: ${STAMP}"
echo "Batch prefix: ${PREFIX}"

python3 examples/batch_strapdown_accuracy.py \
  --ranges 30 40 50 60 70 80 90 100 110 120 130 140 150 160 \
  --altitude-offsets 20 \
  --duration-s 28 \
  --intruder-speed 5 \
  --speed-ratio 2 \
  --rate-hz 20 \
  --print-every-n 0 \
  --prefix "$PREFIX" \
  --trajectory-dir logs/strapdown_accuracy \
  --settings-path "$SETTINGS_PATH" \
  -- --no-los-filter \
     --camera-z 0 \
     --camera-pitch-deg 0 \
     --camera-roll-deg 0 \
     --camera-yaw-deg 0

python3 examples/export_strapdown_truth_required_load.py \
  --summary-csv "$SUMMARY_CSV" \
  --prefix "$TRUTH_PREFIX" \
  --output-dir logs/strapdown_accuracy/truth_required_load

python3 examples/generate_strapdown_center_clock1_report.py \
  --stamp "$STAMP" \
  ${BASELINE_STAMP:+--baseline-stamp "$BASELINE_STAMP"}

echo
echo "summary_csv=${SUMMARY_CSV}"
echo "report=完整方案/捷联ClockSpeed1中心相机对比报告.md"
