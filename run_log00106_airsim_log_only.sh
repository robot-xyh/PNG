#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ACTION="${1:-}"
CONFIG_PATH="${LOG00106_AIRSIM_CONFIG:-$SCRIPT_DIR/config/airsim_log00106_log_only_cases.json}"
SETTINGS_PATH="${LOG00106_AIRSIM_SETTINGS:-$SCRIPT_DIR/config/airsim_blocks_log00106_log_only_settings.json}"
OUTPUT_ROOT="${LOG00106_AIRSIM_OUTPUT_ROOT:-$SCRIPT_DIR/logs/analysis/LOG00106_airsim_log_only}"
BLOCKS_DIR="${BLOCKS_DIR:-/home/linux/Downloads/Blocks/LinuxBlocks1.8.1/LinuxNoEditor}"
export AIRSIM_RPC_HOST="127.0.0.2"
export AIRSIM_RPC_PORT="41451"
export AIRSIM_PORT_POLICY="strict"
export AIRSIM_INSTANCE_LABEL="log00106_log_only"
BLOCKS_PID=""

cleanup_blocks() {
  if [[ -n "$BLOCKS_PID" ]] && kill -0 "$BLOCKS_PID" 2>/dev/null; then
    kill "$BLOCKS_PID" 2>/dev/null || true
    wait "$BLOCKS_PID" 2>/dev/null || true
  fi
  BLOCKS_PID=""
}

usage() {
  echo "Usage: $0 {test|smoke|full|report}" >&2
}

run_tests() {
  python3 -m py_compile \
    "$SCRIPT_DIR/vision_guidance/airsim_log00106_log_only.py" \
    "$SCRIPT_DIR/examples/run_airsim_log00106_log_only.py" \
    "$SCRIPT_DIR/tools/generate_log00106_airsim_report.py"
  python3 -m unittest discover -s "$SCRIPT_DIR/tests" -p 'test_airsim_log00106_log_only.py' -v
  python3 -m unittest discover -s "$SCRIPT_DIR/tests" -v
}

run_with_blocks() {
  local mode="$1"
  local runtime_dir="$SCRIPT_DIR/.airsim_runtime/log00106_log_only"
  local blocks_log="$runtime_dir/blocks.log"
  mkdir -p "$runtime_dir"

  python3 "$SCRIPT_DIR/tools/airsim_port_guard.py" \
    --settings "$SETTINGS_PATH" \
    --output-dir "$runtime_dir/settings" \
    --env-path "$runtime_dir/port.env" \
    --label "log00106_preflight" \
    --host "$AIRSIM_RPC_HOST" \
    --policy strict >/dev/null

  BLOCKS_DIR="$BLOCKS_DIR" \
    AIRSIM_PORT_ENV_PATH="$runtime_dir/blocks.env" \
    "$SCRIPT_DIR/run_blocks_nvidia.sh" \
      -RenderOffscreen -NoSplash -NoVSync -BENCHMARK -FPS=60 \
      -settings="$SETTINGS_PATH" >"$blocks_log" 2>&1 &
  BLOCKS_PID=$!
  trap cleanup_blocks EXIT INT TERM

  local runner_args=(
    --config "$CONFIG_PATH"
    --output-root "$OUTPUT_ROOT"
    --connection-timeout-s 45
  )
  if [[ "$mode" == "smoke" ]]; then
    runner_args+=(--smoke)
  fi
  python3 "$SCRIPT_DIR/examples/run_airsim_log00106_log_only.py" "${runner_args[@]}"
  if [[ "$mode" == "full" ]]; then
    python3 "$SCRIPT_DIR/tools/generate_log00106_airsim_report.py" --output-root "$OUTPUT_ROOT"
  fi
  cleanup_blocks
  trap - EXIT INT TERM
}

case "$ACTION" in
  test)
    run_tests
    ;;
  smoke)
    run_with_blocks smoke
    ;;
  full)
    run_with_blocks full
    ;;
  report)
    python3 "$SCRIPT_DIR/tools/generate_log00106_airsim_report.py" --output-root "$OUTPUT_ROOT"
    ;;
  *)
    usage
    exit 2
    ;;
esac
