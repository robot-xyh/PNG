#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PROFILE="${1:-gimbal}"
shift || true

COMMON_ARGS=(
  "--enable-motion"
  "--duration-s" "${AIRSIM_RECORDING_DURATION:-30}"
  "--intruder-speed" "${AIRSIM_RECORDING_INTRUDER_SPEED:-5}"
  "--intruder-altitude-offset-m" "${AIRSIM_RECORDING_ALTITUDE_OFFSET:-30}"
  "--terminal-blind-duration-s" "0.45"
  "--terminal-command-decay-tau-s" "0.30"
  "--terminal-image-kf-max-predict-s" "0.50"
  "--terminal-enter-area-ratio" "0.12"
  "--terminal-cutoff-area-ratio" "0.45"
  "--max-vision-vertical-speed" "4.0"
  "--terminal-pitch-up-bias-mps" "1.2"
  "--settings-path" "$SCRIPT_DIR/config/airsim_blocks_recording_settings.json"
  "--print-every-n" "20"
)

WINDOW_ARGS=()
if [[ "${AIRSIM_RECORDING_SHOW_WINDOW:-1}" != "0" ]]; then
  WINDOW_ARGS=(
    "--show-window"
    "--window-scale" "${AIRSIM_RECORDING_WINDOW_SCALE:-1.0}"
  )
fi

case "$PROFILE" in
  gimbal)
    python3 examples/run_airsim_gimbal_vision_png.py \
      "${COMMON_ARGS[@]}" \
      "${WINDOW_ARGS[@]}" \
      "--speed-ratio" "${AIRSIM_RECORDING_SPEED_RATIO:-1.6}" \
      "--terminal-gimbal-gain-scale" "0.60" \
      "--terminal-gimbal-limit-area-ratio" "0.10" \
      "--trajectory-prefix" "recording_gimbal_demo" \
      "$@"
    ;;
  strapdown)
    python3 examples/run_airsim_strapdown_vision_png.py \
      "${COMMON_ARGS[@]}" \
      "${WINDOW_ARGS[@]}" \
      "--speed-ratio" "${AIRSIM_RECORDING_SPEED_RATIO:-2.0}" \
      "--terminal-yaw-rate-decay-tau-s" "0.30" \
      "--terminal-yaw-rate-scale" "0.85" \
      "--trajectory-prefix" "recording_strapdown_demo" \
      "$@"
    ;;
  truth)
    python3 examples/run_airsim_truth_png.py \
      "--enable-motion" \
      "--duration-s" "${AIRSIM_RECORDING_DURATION:-30}" \
      "--intruder-speed" "${AIRSIM_RECORDING_INTRUDER_SPEED:-5}" \
      "--speed-ratio" "${AIRSIM_RECORDING_SPEED_RATIO:-2.0}" \
      "--settings-path" "$SCRIPT_DIR/config/airsim_blocks_recording_settings.json" \
      "--trajectory-prefix" "recording_truth_demo" \
      "$@"
    ;;
  *)
    echo "Usage: $0 [gimbal|strapdown|truth] [extra script args...]" >&2
    exit 2
    ;;
esac
