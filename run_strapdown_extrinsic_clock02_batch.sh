#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

STAMP="${STAMP:-$(date +%Y%m%d_%H%M%S)}"
RANGES=(30 40 50 60 70 80 90 100 110 120 130 140 150 160)
COMMON_ARGS=(
  --ranges "${RANGES[@]}"
  --altitude-offsets 20
  --duration-s 28
  --intruder-speed 5
  --speed-ratio 2
  --rate-hz 20
  --print-every-n 0
  --trajectory-dir logs/strapdown_accuracy
)

echo "AirSim Blocks must already be running with config/airsim_blocks_settings.json."
echo "Experiment stamp: ${STAMP}"

run_case() {
  local label="$1"
  shift
  local prefix="strapdown_clock0p2_ext${label}_${STAMP}"
  local summary_csv="logs/strapdown_accuracy/${prefix}_summary.csv"
  local truth_prefix="strapdown_clock0p2_ext${label}_truth_N3_${STAMP}"

  echo
  echo "=== extrinsic case ${label}: prefix=${prefix} ==="
  python3 examples/batch_strapdown_accuracy.py \
    "${COMMON_ARGS[@]}" \
    --prefix "$prefix" \
    -- --no-los-filter "$@"

  python3 examples/export_strapdown_truth_required_load.py \
    --summary-csv "$summary_csv" \
    --prefix "$truth_prefix" \
    --output-dir logs/strapdown_accuracy/truth_required_load
}

# A: camera 0.5m above frame, no pitch offset.
run_case A \
  --camera-z -0.5 \
  --camera-pitch-deg 0

# B: camera 0.5m above frame and pitched upward by 15 deg.
run_case B \
  --camera-z -0.5 \
  --camera-pitch-deg -15

# C: camera 0.5m above frame; earlier terminal prediction and top-clipped pitch rejection.
run_case C \
  --camera-z -0.5 \
  --camera-pitch-deg 0 \
  --reject-top-clipped-pitch \
  --terminal-enter-area-ratio 0.12 \
  --terminal-soft-enter-area-ratio 0.03 \
  --terminal-cutoff-area-ratio 0.35

# D: camera 0.5m above frame; stronger upward blind-push bias after extrapolation.
run_case D \
  --camera-z -0.5 \
  --camera-pitch-deg 0 \
  --terminal-pitch-up-bias-mps 1.2

python3 examples/generate_strapdown_extrinsic_report.py --stamp "$STAMP"

echo
echo "report=完整方案/捷联ClockSpeed0p2四工况相机外参测试报告.md"
