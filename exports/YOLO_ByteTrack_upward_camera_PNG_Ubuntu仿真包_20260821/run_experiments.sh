#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PRESET="${1:-standard}"
TIER="${2:-all}"

if [[ $# -gt 0 ]]; then
  shift
fi
if [[ $# -gt 0 ]]; then
  shift
fi

case "$PRESET" in
  smoke|standard|overnight) ;;
  *)
    echo "Preset must be smoke, standard, or overnight: $PRESET" >&2
    exit 2
    ;;
esac
case "$TIER" in
  fast|sitl|all) ;;
  *)
    echo "Tier must be fast, sitl, or all: $TIER" >&2
    exit 2
    ;;
esac

PYTHON="$SCRIPT_DIR/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  echo "Python environment is missing. Run ./install_ubuntu.sh first." >&2
  exit 1
fi

export AIRSIM_RPC_HOST="127.0.0.2"
export AIRSIM_REWRITE_HOST_IPS="0"
export PYTHONUTF8="1"
export __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-1}"
export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"

exec "$PYTHON" "$SCRIPT_DIR/tools/ubuntu_experiments.py" \
  --preset "$PRESET" --tier "$TIER" "$@"
