#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AEROLOOP_DIR="${AEROLOOP_DIR:-$HOME/aeroloop_gazebo}"
BETAFLIGHT_DIR="${BETAFLIGHT_DIR:-$HOME/betaflight}"
BETAFLIGHT_BIN="${BETAFLIGHT_BIN:-$BETAFLIGHT_DIR/obj/main/betaflight_SITL.elf}"
WORLD_PATH="${GAZEBO_WORLD_PATH:-$AEROLOOP_DIR/worlds/betaloop_iris_betaflight_demo_harmonic.sdf}"
CLI_PATH="${BETAFLIGHT_CLI_PATH:-$SCRIPT_DIR/config/betaflight_sitl_truth_png_msp_cli.txt}"
STAMP="${STAMP:-gazebo_bf_truth_png_$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$SCRIPT_DIR/logs/gazebo_betaflight_truth_png"
START_GAZEBO="${START_GAZEBO:-1}"
START_BETAFLIGHT="${START_BETAFLIGHT:-1}"
LOAD_BETAFLIGHT_CONFIG="${LOAD_BETAFLIGHT_CONFIG:-1}"
GAZEBO_SETTLE_S="${GAZEBO_SETTLE_S:-2}"
BETAFLIGHT_SETTLE_S="${BETAFLIGHT_SETTLE_S:-1}"
mkdir -p "$LOG_DIR"

require_file() {
  if [[ ! -f "$1" ]]; then
    echo "Required file not found: $1" >&2
    exit 1
  fi
}

check_ports_free() {
  local pattern='(:5761|:9001|:9002|:9003|:9004)\b'
  local busy
  busy="$(ss -lntup | rg "$pattern" || true)"
  if [[ -n "$busy" ]]; then
    echo "Betaflight/Gazebo ports are already in use; not starting another instance:" >&2
    echo "$busy" >&2
    exit 1
  fi
}

require_file "$BETAFLIGHT_BIN"
require_file "$WORLD_PATH"
require_file "$CLI_PATH"

if [[ "$START_GAZEBO" == "1" || "$START_BETAFLIGHT" == "1" ]]; then
  check_ports_free
fi

if [[ "$LOAD_BETAFLIGHT_CONFIG" == "1" ]]; then
  echo "Loading Betaflight MSP-RX config: $CLI_PATH"
  (cd "$BETAFLIGHT_DIR" && "$BETAFLIGHT_BIN" --config "$CLI_PATH") >"$LOG_DIR/betaflight_config_${STAMP}.log" 2>&1
fi

GZ_PID=""
BF_PID=""
cleanup() {
  if [[ -n "$BF_PID" ]] && kill -0 "$BF_PID" 2>/dev/null; then
    kill "$BF_PID" 2>/dev/null || true
    wait "$BF_PID" 2>/dev/null || true
  fi
  if [[ -n "$GZ_PID" ]] && kill -0 "$GZ_PID" 2>/dev/null; then
    kill "$GZ_PID" 2>/dev/null || true
    wait "$GZ_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

export SDF_PATH="$AEROLOOP_DIR/models:${SDF_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$AEROLOOP_DIR/worlds:$AEROLOOP_DIR/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$AEROLOOP_DIR/plugins/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

if [[ "$START_GAZEBO" == "1" ]]; then
  echo "Starting Gazebo Harmonic: $WORLD_PATH"
  gz sim -r -v 3 -s --headless-rendering "$WORLD_PATH" >"$LOG_DIR/gazebo_${STAMP}.log" 2>&1 &
  GZ_PID=$!
  for _ in $(seq 1 60); do
    if gz topic -l 2>/dev/null | rg -q '^/world/betaloop_demo/pose/info$'; then
      break
    fi
    sleep 1
  done
  if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo "Gazebo exited early. See $LOG_DIR/gazebo_${STAMP}.log" >&2
    exit 1
  fi
  sleep "$GAZEBO_SETTLE_S"
fi

if [[ "$START_BETAFLIGHT" == "1" ]]; then
  echo "Starting Betaflight SITL"
  if command -v stdbuf >/dev/null 2>&1; then
    (cd "$BETAFLIGHT_DIR" && stdbuf -oL -eL "$BETAFLIGHT_BIN" --ip 127.0.0.1) >"$LOG_DIR/betaflight_${STAMP}.log" 2>&1 &
  else
    (cd "$BETAFLIGHT_DIR" && "$BETAFLIGHT_BIN" --ip 127.0.0.1) >"$LOG_DIR/betaflight_${STAMP}.log" 2>&1 &
  fi
  BF_PID=$!
  for _ in $(seq 1 40); do
    if ss -lntup | rg -q ':5761\b'; then
      break
    fi
    sleep 0.25
  done
  if ! kill -0 "$BF_PID" 2>/dev/null; then
    echo "Betaflight exited early. See $LOG_DIR/betaflight_${STAMP}.log" >&2
    exit 1
  fi
  sleep "$BETAFLIGHT_SETTLE_S"
fi

cd "$SCRIPT_DIR"
PYTHONPATH="$SCRIPT_DIR:${PYTHONPATH:-}" python3 examples/run_gazebo_betaflight_truth_png.py \
  --trajectory-prefix "$STAMP" \
  "$@"

echo "gazebo_betaflight_truth_png_stamp=$STAMP"
