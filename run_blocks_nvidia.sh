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

DEFAULT_ARGS=("-windowed" "-ResX=1280" "-ResY=720")
if [[ "$#" -gt 0 ]]; then
  ARGS=("$@")
else
  ARGS=("${DEFAULT_ARGS[@]}")
fi

HAS_NOHMD=0
HAS_SETTINGS=0
for ARG in "${ARGS[@]}"; do
  if [[ "$ARG" == "-nohmd" || "$ARG" == "--nohmd" ]]; then
    HAS_NOHMD=1
  fi
  if [[ "$ARG" == "-settings" || "$ARG" == -settings=* ]]; then
    HAS_SETTINGS=1
  fi
done
if [[ "$HAS_NOHMD" -eq 0 ]]; then
  ARGS=("-nohmd" "${ARGS[@]}")
fi
if [[ "$HAS_SETTINGS" -eq 0 && -f "$DEFAULT_AIRSIM_SETTINGS" ]]; then
  ARGS=("-settings=$DEFAULT_AIRSIM_SETTINGS" "${ARGS[@]}")
fi

cd "$BLOCKS_DIR"
echo "Launching Blocks from: $BLOCKS_DIR"
echo "Using Vulkan ICD: $VK_ICD_FILENAMES"
if [[ "$HAS_SETTINGS" -eq 0 && -f "$DEFAULT_AIRSIM_SETTINGS" ]]; then
  echo "Using AirSim settings: $DEFAULT_AIRSIM_SETTINGS"
fi
exec ./Blocks.sh "${ARGS[@]}"
