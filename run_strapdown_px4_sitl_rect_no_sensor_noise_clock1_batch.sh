#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

STAMP="${STAMP:-px4_actor_rect_no_sensor_noise_$(date +%Y%m%d_%H%M%S)}"
SETTINGS_PATH="${SETTINGS_PATH:-$SCRIPT_DIR/config/airsim_blocks_px4_actor_settings.json}"
NOISE_SEED="${NOISE_SEED:-20260617}"
NOISE_CENTER_PX="${NOISE_CENTER_PX:-3.0}"
NOISE_AREA_RATIO="${NOISE_AREA_RATIO:-0.08}"
INTRUDER_ACTOR_ASSET="${INTRUDER_ACTOR_ASSET:-1M_Cube_Chamfer}"
INTRUDER_ACTOR_SCALE="${INTRUDER_ACTOR_SCALE:-1.0}"
INTRUDER_ACTOR_SCALE_X="${INTRUDER_ACTOR_SCALE_X:-1.0}"
INTRUDER_ACTOR_SCALE_Y="${INTRUDER_ACTOR_SCALE_Y:-1.0}"
INTRUDER_ACTOR_SCALE_Z="${INTRUDER_ACTOR_SCALE_Z:-0.5}"

PREFIX="strapdown_clock1_sitl_L_${STAMP}"
SUMMARY_CSV="logs/strapdown_accuracy/${PREFIX}_summary.csv"
TRUTH_PREFIX="strapdown_clock1_sitl_L_truth_N3_${STAMP}"

echo "PX4 SITL and AirSim Blocks must already be running with ${SETTINGS_PATH}."
echo "Experiment stamp: ${STAMP}"
echo "Case L: no IMU/GPS noise, bbox noise enabled, LOS filter disabled."
echo "Noise: center_sigma_px=${NOISE_CENTER_PX}, area_sigma_ratio=${NOISE_AREA_RATIO}, seed=${NOISE_SEED}"
echo "Actor target: asset=${INTRUDER_ACTOR_ASSET}, scale=(${INTRUDER_ACTOR_SCALE_X}, ${INTRUDER_ACTOR_SCALE_Y}, ${INTRUDER_ACTOR_SCALE_Z})"

python3 examples/batch_strapdown_accuracy.py \
  --ranges 40 50 60 70 80 90 100 110 120 130 140 \
  --altitude-offsets 20 \
  --duration-s 35 \
  --intruder-speed 5 \
  --speed-ratio 2 \
  --rate-hz 20 \
  --intercept-altitude-m 50 \
  --print-every-n 0 \
  --trajectory-dir logs/strapdown_accuracy \
  --settings-path "$SETTINGS_PATH" \
  --no-reset-between-runs \
  --prefix "$PREFIX" \
  -- \
  --no-los-filter \
  --px4-interceptor \
  --intruder IntruderActor \
  --intruder-actor \
  --intruder-actor-name IntruderActor \
  --intruder-actor-asset "$INTRUDER_ACTOR_ASSET" \
  --intruder-actor-scale "$INTRUDER_ACTOR_SCALE" \
  --intruder-actor-scale-x "$INTRUDER_ACTOR_SCALE_X" \
  --intruder-actor-scale-y "$INTRUDER_ACTOR_SCALE_Y" \
  --intruder-actor-scale-z "$INTRUDER_ACTOR_SCALE_Z" \
  --mesh 'IntruderActor' \
  --no-reset \
  --bbox-noise \
  --bbox-center-noise-px "$NOISE_CENTER_PX" \
  --bbox-area-noise-ratio "$NOISE_AREA_RATIO" \
  --bbox-noise-seed "$NOISE_SEED" \
  --climb-timeout-s 90 \
  --no-px4-command-join \
  --px4-command-mode velocity_yaw_rate \
  --min-command-duration-s 0.10 \
  --command-duration-margin-s 0.05 \
  --max-command-duration-s 0.20 \
  --camera-z 0 \
  --camera-pitch-deg 0 \
  --camera-roll-deg 0 \
  --camera-yaw-deg 0

python3 examples/export_strapdown_truth_required_load.py \
  --summary-csv "$SUMMARY_CSV" \
  --prefix "$TRUTH_PREFIX" \
  --output-dir logs/strapdown_accuracy/truth_required_load

echo
echo "l_stamp=${STAMP}"
echo "summary=${SUMMARY_CSV}"
