#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PREFIX="${PREFIX:-strapdown_clock0p2_no_los_filter_$(date +%Y%m%d_%H%M%S)}"
SUMMARY_CSV="logs/strapdown_accuracy/${PREFIX}_summary.csv"
TRUTH_PREFIX="strapdown_clock0p2_no_los_filter_truth_theory_N3"

echo "Using batch prefix: ${PREFIX}"
echo "AirSim Blocks must already be running with config/airsim_blocks_settings.json."

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
  -- --no-los-filter

python3 examples/export_strapdown_truth_required_load.py \
  --summary-csv "$SUMMARY_CSV" \
  --prefix "$TRUTH_PREFIX" \
  --output-dir logs/strapdown_accuracy/truth_required_load

python3 examples/generate_strapdown_accuracy_report.py --clock02-no-los-filter

echo "summary_csv=${SUMMARY_CSV}"
echo "report=完整方案/捷联ClockSpeed0p2无LOS滤波距离过载测试报告.md"
