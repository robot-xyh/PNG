#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_AIRSIM_SETTINGS="$SCRIPT_DIR/config/airsim_blocks_settings.json"
DEFAULT_BLOCKS_DIR="$SCRIPT_DIR/Blocks/LinuxBlocks1.8.1/LinuxNoEditor"
if [[ -n "${BLOCKS_DIR:-}" ]]; then
  BLOCKS_DIR="$BLOCKS_DIR"
else
  BLOCKS_DIR="$DEFAULT_BLOCKS_DIR"
  if [[ ! -f "$BLOCKS_DIR/Blocks.sh" && -d "$SCRIPT_DIR/Blocks" ]]; then
    mapfile -t CANDIDATES < <(find "$SCRIPT_DIR/Blocks" -maxdepth 5 -path '*/LinuxNoEditor/Blocks.sh' -type f 2>/dev/null | sort)
    if [[ "${#CANDIDATES[@]}" -eq 1 ]]; then
      BLOCKS_DIR="$(cd "$(dirname "${CANDIDATES[0]}")" && pwd)"
    fi
  fi
fi
BLOCKS_SH="$BLOCKS_DIR/Blocks.sh"
BLOCKS_BIN="$BLOCKS_DIR/Blocks/Binaries/Linux/Blocks"

if [[ ! -f "$BLOCKS_SH" ]]; then
  echo "Blocks.sh not found: $BLOCKS_SH" >&2
  echo "Set BLOCKS_DIR to the directory that contains Blocks.sh." >&2
  exit 1
fi

chmod +x "$BLOCKS_SH"
if [[ -f "$BLOCKS_BIN" ]]; then
  chmod +x "$BLOCKS_BIN"
fi

export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"
export __VK_LAYER_NV_optimus="${__VK_LAYER_NV_optimus:-NVIDIA_only}"
export VK_ICD_FILENAMES="${VK_ICD_FILENAMES:-/usr/share/vulkan/icd.d/nvidia_icd.json}"
export VK_DRIVER_FILES="${VK_DRIVER_FILES:-$VK_ICD_FILENAMES}"

DEFAULT_ARGS=("-RenderOffscreen" "-NoSplash" "-NoVSync" "-BENCHMARK" "-FPS=20")
if [[ "$#" -gt 0 ]]; then
  ARGS=("$@")
else
  ARGS=("${DEFAULT_ARGS[@]}")
fi

HAS_NOHMD=0
SETTINGS_ARG=""
FILTERED_ARGS=()
INDEX=0
while [[ "$INDEX" -lt "${#ARGS[@]}" ]]; do
  ARG="${ARGS[$INDEX]}"
  if [[ "$ARG" == "-nohmd" || "$ARG" == "--nohmd" ]]; then
    HAS_NOHMD=1
  fi
  if [[ "$ARG" == -settings=* ]]; then
    SETTINGS_ARG="${ARG#-settings=}"
  elif [[ "$ARG" == "-settings" ]]; then
    INDEX=$((INDEX + 1))
    if [[ "$INDEX" -ge "${#ARGS[@]}" ]]; then
      echo "-settings requires a path argument" >&2
      exit 1
    fi
    SETTINGS_ARG="${ARGS[$INDEX]}"
  else
    FILTERED_ARGS+=("$ARG")
  fi
  INDEX=$((INDEX + 1))
done
ARGS=("${FILTERED_ARGS[@]}")

if [[ -z "$SETTINGS_ARG" && -f "$DEFAULT_AIRSIM_SETTINGS" ]]; then
  SETTINGS_ARG="$DEFAULT_AIRSIM_SETTINGS"
fi

if [[ -n "$SETTINGS_ARG" ]]; then
  PORT_ENV_PATH="${AIRSIM_PORT_ENV_PATH:-$SCRIPT_DIR/.airsim_runtime/latest.env}"
  PORT_GUARD_OUTPUT="$(
    python3 "$SCRIPT_DIR/tools/airsim_port_guard.py" \
      --settings "$SETTINGS_ARG" \
      --output-dir "$SCRIPT_DIR/.airsim_runtime/settings" \
      --env-path "$PORT_ENV_PATH" \
      --label "${AIRSIM_INSTANCE_LABEL:-blocks}" \
      --policy "${AIRSIM_PORT_POLICY:-auto}"
  )"
  eval "$PORT_GUARD_OUTPUT"
  export AIRSIM_SETTINGS_PATH_RESOLVED AIRSIM_RPC_HOST AIRSIM_RPC_PORT AIRSIM_PX4_TCP_PORTS AIRSIM_PORT_POLICY AIRSIM_PORT_REWRITTEN
  if [[ -n "${AIRSIM_PX4_TCP_PORT:-}" ]]; then
    export AIRSIM_PX4_TCP_PORT
  fi
  ARGS=("-settings=$AIRSIM_SETTINGS_PATH_RESOLVED" "${ARGS[@]}")
else
  AIRSIM_RPC_HOST="${AIRSIM_RPC_HOST:-127.0.0.1}"
  AIRSIM_RPC_PORT="${AIRSIM_RPC_PORT:-41451}"
  export AIRSIM_RPC_HOST AIRSIM_RPC_PORT
fi

if [[ "$HAS_NOHMD" -eq 0 ]]; then
  ARGS=("-nohmd" "${ARGS[@]}")
fi

cd "$BLOCKS_DIR"
echo "Launching Blocks from: $BLOCKS_DIR"
echo "Using Vulkan ICD: $VK_ICD_FILENAMES"
if [[ "$#" -eq 0 ]]; then
  echo "Using default offscreen args: ${DEFAULT_ARGS[*]}"
fi
echo "Using AirSim settings: ${AIRSIM_SETTINGS_PATH_RESOLVED:-$SETTINGS_ARG}"
echo "AirSim RPC endpoint: ${AIRSIM_RPC_HOST}:${AIRSIM_RPC_PORT}"
if [[ -n "${AIRSIM_PX4_TCP_PORTS:-}" ]]; then
  echo "PX4 simulator TCP ports in settings: ${AIRSIM_PX4_TCP_PORTS}"
fi
if [[ -n "${PORT_ENV_PATH:-}" ]]; then
  echo "AirSim runtime env: ${PORT_ENV_PATH}"
fi
exec ./Blocks.sh "${ARGS[@]}"
