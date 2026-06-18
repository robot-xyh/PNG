#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

STAMP="${STAMP:-center_clock1_noise_$(date +%Y%m%d_%H%M%S)}"
BASELINE_STAMP="${BASELINE_STAMP:-}"
CENTER_STAMP="${CENTER_STAMP:-}"
SETTINGS_PATH="$SCRIPT_DIR/config/airsim_blocks_settings_clock1_center_camera.json"
NOISE_SEED="${NOISE_SEED:-20260617}"
NOISE_CENTER_PX="${NOISE_CENTER_PX:-3.0}"
NOISE_AREA_RATIO="${NOISE_AREA_RATIO:-0.08}"

COMMON_ARGS=(
  --ranges 30 40 50 60 70 80 90 100 110 120 130 140 150 160
  --altitude-offsets 20
  --duration-s 28
  --intruder-speed 5
  --speed-ratio 2
  --rate-hz 20
  --print-every-n 0
  --trajectory-dir logs/strapdown_accuracy
  --settings-path "$SETTINGS_PATH"
)

run_case() {
  local label="$1"
  shift
  local prefix="strapdown_clock1_noise_${label}_${STAMP}"
  local summary_csv="logs/strapdown_accuracy/${prefix}_summary.csv"
  local truth_prefix="strapdown_clock1_noise_${label}_truth_N3_${STAMP}"

  echo
  echo "=== noise case ${label}: prefix=${prefix} ==="
  python3 examples/batch_strapdown_accuracy.py \
    "${COMMON_ARGS[@]}" \
    --prefix "$prefix" \
    -- "$@" \
       --bbox-noise \
       --bbox-center-noise-px "$NOISE_CENTER_PX" \
       --bbox-area-noise-ratio "$NOISE_AREA_RATIO" \
       --bbox-noise-seed "$NOISE_SEED" \
       --camera-z 0 \
       --camera-pitch-deg 0 \
       --camera-roll-deg 0 \
       --camera-yaw-deg 0

  python3 examples/export_strapdown_truth_required_load.py \
    --summary-csv "$summary_csv" \
    --prefix "$truth_prefix" \
    --output-dir logs/strapdown_accuracy/truth_required_load
}

echo "AirSim Blocks must already be running with ${SETTINGS_PATH}."
echo "Experiment stamp: ${STAMP}"
echo "Noise: center_sigma_px=${NOISE_CENTER_PX}, area_sigma_ratio=${NOISE_AREA_RATIO}, seed=${NOISE_SEED}"

if [[ -z "$CENTER_STAMP" ]]; then
  LATEST_CENTER_SUMMARY="$(find logs/strapdown_accuracy -maxdepth 1 -name 'strapdown_clock1_center_*_summary.csv' -printf '%T@ %f\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)"
  if [[ -n "$LATEST_CENTER_SUMMARY" ]]; then
    CENTER_STAMP="${LATEST_CENTER_SUMMARY#strapdown_clock1_center_}"
    CENTER_STAMP="${CENTER_STAMP%_summary.csv}"
  fi
fi
if [[ -z "$CENTER_STAMP" ]]; then
  echo "CENTER_STAMP is required when no existing E summary is available." >&2
  exit 1
fi
echo "Center/no-noise E stamp: ${CENTER_STAMP}"

run_case F
run_case G --no-los-filter

python3 examples/generate_strapdown_center_clock1_report.py \
  --stamp "$CENTER_STAMP" \
  --noise-stamp "$STAMP" \
  ${BASELINE_STAMP:+--baseline-stamp "$BASELINE_STAMP"}

echo
echo "report=完整方案/捷联ClockSpeed1中心相机对比报告.md"
