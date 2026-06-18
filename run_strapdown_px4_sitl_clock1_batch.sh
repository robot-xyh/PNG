#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

STAMP="${STAMP:-px4_sitl_clock1_$(date +%Y%m%d_%H%M%S)}"
BASELINE_STAMP="${BASELINE_STAMP:-}"
CENTER_STAMP="${CENTER_STAMP:-}"
NOISE_STAMP="${NOISE_STAMP:-}"
SETTINGS_PATH="${SETTINGS_PATH:-$SCRIPT_DIR/config/airsim_blocks_px4_actor_settings.json}"
NOISE_SEED="${NOISE_SEED:-20260617}"
NOISE_CENTER_PX="${NOISE_CENTER_PX:-3.0}"
NOISE_AREA_RATIO="${NOISE_AREA_RATIO:-0.08}"
INTRUDER_ACTOR_ASSET="${INTRUDER_ACTOR_ASSET:-1M_Cube_Chamfer}"
INTRUDER_ACTOR_SCALE="${INTRUDER_ACTOR_SCALE:-2.0}"

COMMON_ARGS=(
  --ranges 40 80 120
  --altitude-offsets 20
  --duration-s 12
  --intruder-speed 5
  --speed-ratio 2
  --rate-hz 20
  --intercept-altitude-m 50
  --print-every-n 0
  --trajectory-dir logs/strapdown_accuracy
  --settings-path "$SETTINGS_PATH"
)

find_latest_stamp() {
  local glob="$1"
  local prefix="$2"
  local suffix="$3"
  local latest
  latest="$(find logs/strapdown_accuracy -maxdepth 1 -name "$glob" -printf '%T@ %f\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)"
  if [[ -n "$latest" ]]; then
    latest="${latest#"$prefix"}"
    latest="${latest%"$suffix"}"
    printf '%s' "$latest"
  fi
}

run_case() {
  local label="$1"
  shift
  local prefix="strapdown_clock1_sitl_${label}_${STAMP}"
  local summary_csv="logs/strapdown_accuracy/${prefix}_summary.csv"
  local truth_prefix="strapdown_clock1_sitl_${label}_truth_N3_${STAMP}"

  echo
  echo "=== PX4 SITL + actor case ${label}: prefix=${prefix} ==="
  python3 examples/batch_strapdown_accuracy.py \
    "${COMMON_ARGS[@]}" \
    --prefix "$prefix" \
       -- "$@" \
       --px4-interceptor \
       --intruder IntruderActor \
       --intruder-actor \
       --intruder-actor-name IntruderActor \
       --intruder-actor-asset "$INTRUDER_ACTOR_ASSET" \
       --intruder-actor-scale "$INTRUDER_ACTOR_SCALE" \
       --mesh 'IntruderActor' \
       --no-reset \
       --bbox-noise \
       --bbox-center-noise-px "$NOISE_CENTER_PX" \
       --bbox-area-noise-ratio "$NOISE_AREA_RATIO" \
       --bbox-noise-seed "$NOISE_SEED" \
       --climb-timeout-s 90 \
       --no-px4-command-join \
       --min-command-duration-s 0.10 \
       --command-duration-margin-s 0.05 \
       --max-command-duration-s 0.20 \
       --camera-z 0 \
       --camera-pitch-deg 0 \
       --camera-roll-deg 0 \
       --camera-yaw-deg 0

  python3 examples/export_strapdown_truth_required_load.py \
    --summary-csv "$summary_csv" \
    --prefix "$truth_prefix" \
    --output-dir logs/strapdown_accuracy/truth_required_load
}

echo "PX4 SITL and AirSim Blocks must already be running with ${SETTINGS_PATH}."
echo "Experiment stamp: ${STAMP}"
echo "Noise: center_sigma_px=${NOISE_CENTER_PX}, area_sigma_ratio=${NOISE_AREA_RATIO}, seed=${NOISE_SEED}"
echo "Actor target: asset=${INTRUDER_ACTOR_ASSET}, scale=${INTRUDER_ACTOR_SCALE}"

if [[ -z "$CENTER_STAMP" ]]; then
  CENTER_STAMP="$(find_latest_stamp 'strapdown_clock1_center_*_summary.csv' 'strapdown_clock1_center_' '_summary.csv')"
fi
if [[ -z "$CENTER_STAMP" ]]; then
  echo "CENTER_STAMP is required when no existing E summary is available." >&2
  exit 1
fi

if [[ -z "$NOISE_STAMP" ]]; then
  NOISE_STAMP="$(find_latest_stamp 'strapdown_clock1_noise_F_*_summary.csv' 'strapdown_clock1_noise_F_' '_summary.csv')"
fi

echo "Center/no-noise E stamp: ${CENTER_STAMP}"
echo "SimpleFlight noise F/G stamp: ${NOISE_STAMP:-N/A}"

run_case H --no-los-filter
run_case I

python3 examples/generate_strapdown_center_clock1_report.py \
  --stamp "$CENTER_STAMP" \
  --sitl-stamp "$STAMP" \
  ${NOISE_STAMP:+--noise-stamp "$NOISE_STAMP"} \
  ${BASELINE_STAMP:+--baseline-stamp "$BASELINE_STAMP"}

echo
echo "sitl_stamp=${STAMP}"
echo "report=完整方案/捷联ClockSpeed1中心相机对比报告.md"
