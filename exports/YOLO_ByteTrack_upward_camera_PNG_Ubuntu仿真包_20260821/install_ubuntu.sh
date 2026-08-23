#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_DIR="$SCRIPT_DIR/runtime"
VENV_DIR="$SCRIPT_DIR/.venv"
BLOCKS_URL="https://github.com/microsoft/AirSim/releases/download/v1.8.1/Blocks.zip"
BLOCKS_BYTES="142533463"
BLOCKS_SHA256="563e1da1b5d7303b9405c8736243ea92ab0a011b96a561f26f17563d105a404f"
PX4_URL="https://github.com/PX4/PX4-Autopilot.git"
PX4_TAG="v1.11.3"
PX4_PATCH="$SCRIPT_DIR/patches/px4-v1.11.3-ubuntu24-stacksize.patch"

log() {
  printf '[install] %s\n' "$*"
}

fail() {
  printf '[install] ERROR: %s\n' "$*" >&2
  exit 1
}

if [[ "$(uname -s)" != "Linux" || "$(uname -m)" != "x86_64" ]]; then
  fail "Ubuntu Linux x86_64 is required."
fi
if [[ ! -r /etc/os-release ]]; then
  fail "Cannot identify the operating system."
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || ( "${VERSION_ID:-}" != "22.04" && "${VERSION_ID:-}" != "24.04" ) ]]; then
  fail "Supported systems are Ubuntu 22.04 and 24.04; found ${PRETTY_NAME:-unknown}."
fi

if [[ $EUID -eq 0 ]]; then
  SUDO=()
else
  command -v sudo >/dev/null 2>&1 || fail "sudo is required to install apt dependencies."
  SUDO=(sudo)
fi

command -v nvidia-smi >/dev/null 2>&1 || fail "NVIDIA driver is missing. Install it and reboot before rerunning this script."
nvidia-smi >/dev/null 2>&1 || fail "The NVIDIA driver is installed but not operational. Reboot or repair the driver first."

log "Installing Ubuntu build, AirSim runtime, and PX4 SITL dependencies"
"${SUDO[@]}" apt-get update
DEBIAN_FRONTEND=noninteractive "${SUDO[@]}" apt-get install -y --no-install-recommends \
  astyle build-essential ca-certificates ccache cmake cppcheck curl file g++ gcc gdb genromfs git \
  libgl1 libglib2.0-0 libglu1-mesa libnss3 libsm6 libx11-6 libxcb-xinerama0 libxcursor1 \
  libxext6 libxi6 libxinerama1 libxrandr2 libxrender1 libxss1 libxtst6 libvulkan1 make \
  ninja-build python3 python3-dev python3-pip python3-venv python3-wheel rsync shellcheck unzip zip

PYTHON_MINOR="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
case "$PYTHON_MINOR" in
  3.10|3.11|3.12) ;;
  *) fail "Python 3.10-3.12 is required; found $PYTHON_MINOR." ;;
esac

log "Creating Python environment and installing pinned CUDA dependencies"
python3 -m venv "$VENV_DIR"
"$VENV_DIR/bin/python" -m pip install --upgrade pip==25.1.1 setuptools==80.9.0 wheel==0.45.1
"$VENV_DIR/bin/python" -m pip install -r "$SCRIPT_DIR/requirements-ubuntu.txt"
"$VENV_DIR/bin/python" -m pip install -r "$SCRIPT_DIR/requirements-px4-v1.11.3.txt"

mkdir -p "$RUNTIME_DIR"

find_blocks_launcher() {
  local candidate="$1"
  if [[ -f "$candidate" && "$(basename "$candidate")" == "Blocks.sh" ]]; then
    printf '%s\n' "$candidate"
    return 0
  fi
  if [[ ! -d "$candidate" ]]; then
    return 0
  fi
  find "$candidate" -maxdepth 5 -type f -name Blocks.sh -print -quit 2>/dev/null
}

BLOCKS_LAUNCHER=""
if [[ -n "${BLOCKS_DIR:-}" ]]; then
  BLOCKS_LAUNCHER="$(find_blocks_launcher "$BLOCKS_DIR")"
  [[ -n "$BLOCKS_LAUNCHER" ]] || fail "BLOCKS_DIR does not contain Blocks.sh: $BLOCKS_DIR"
elif [[ -s "$RUNTIME_DIR/blocks_path.txt" ]]; then
  MARKED_BLOCKS="$(tr -d '\r\n' < "$RUNTIME_DIR/blocks_path.txt")"
  if [[ -f "$MARKED_BLOCKS" ]]; then
    BLOCKS_LAUNCHER="$MARKED_BLOCKS"
  fi
fi
if [[ -z "$BLOCKS_LAUNCHER" && -d "$HOME/Downloads/Blocks" ]]; then
  BLOCKS_LAUNCHER="$(find_blocks_launcher "$HOME/Downloads/Blocks")"
fi

if [[ -z "$BLOCKS_LAUNCHER" ]]; then
  BLOCKS_ROOT="$RUNTIME_DIR/Blocks"
  BLOCKS_LAUNCHER="$(find_blocks_launcher "$BLOCKS_ROOT")"
  if [[ -z "$BLOCKS_LAUNCHER" ]]; then
    ARCHIVE="$RUNTIME_DIR/AirSim-Blocks-v1.8.1.zip"
    if [[ -f "$ARCHIVE" ]] && {
      [[ "$(stat -c '%s' "$ARCHIVE")" != "$BLOCKS_BYTES" ]] ||
      [[ "$(sha256sum "$ARCHIVE" | awk '{print $1}')" != "$BLOCKS_SHA256" ]];
    }; then
      rm -f "$ARCHIVE"
    fi
    if [[ ! -f "$ARCHIVE" ]]; then
      log "Downloading official AirSim Blocks 1.8.1 archive"
      curl -fL --retry 5 --retry-delay 2 --retry-all-errors -o "$ARCHIVE" "$BLOCKS_URL"
    fi
    [[ "$(stat -c '%s' "$ARCHIVE")" == "$BLOCKS_BYTES" ]] || fail "Blocks archive size validation failed."
    [[ "$(sha256sum "$ARCHIVE" | awk '{print $1}')" == "$BLOCKS_SHA256" ]] || fail "Blocks archive SHA256 validation failed."
    rm -rf "$RUNTIME_DIR/Blocks.extract" "$BLOCKS_ROOT"
    mkdir -p "$RUNTIME_DIR/Blocks.extract"
    unzip -q "$ARCHIVE" -d "$RUNTIME_DIR/Blocks.extract"
    mv "$RUNTIME_DIR/Blocks.extract" "$BLOCKS_ROOT"
    BLOCKS_LAUNCHER="$(find_blocks_launcher "$BLOCKS_ROOT")"
    [[ -n "$BLOCKS_LAUNCHER" ]] || fail "Blocks.sh was not found after extracting the official archive."
  fi
fi
BLOCKS_LAUNCHER="$(realpath "$BLOCKS_LAUNCHER")"
chmod +x "$BLOCKS_LAUNCHER"
BLOCKS_BINARY="$(dirname "$BLOCKS_LAUNCHER")/Blocks/Binaries/Linux/Blocks"
[[ -f "$BLOCKS_BINARY" ]] || fail "AirSim Blocks binary is missing beside $BLOCKS_LAUNCHER."
chmod +x "$BLOCKS_BINARY"
printf '%s\n' "$BLOCKS_LAUNCHER" > "$RUNTIME_DIR/blocks_path.txt"
log "AirSim Blocks: $BLOCKS_LAUNCHER"

PX4_SOURCE=""
if [[ -n "${PX4_DIR:-}" ]]; then
  PX4_SOURCE="$PX4_DIR"
elif [[ -s "$RUNTIME_DIR/px4_path.txt" ]]; then
  MARKED_PX4="$(tr -d '\r\n' < "$RUNTIME_DIR/px4_path.txt")"
  if [[ -f "$MARKED_PX4/Makefile" ]]; then
    PX4_SOURCE="$MARKED_PX4"
  fi
fi
if [[ -z "$PX4_SOURCE" && -f "$HOME/PX4/PX4-Autopilot/Makefile" ]]; then
  PX4_SOURCE="$HOME/PX4/PX4-Autopilot"
fi
if [[ -z "$PX4_SOURCE" ]]; then
  PX4_SOURCE="$RUNTIME_DIR/PX4-Autopilot"
fi

if [[ ! -f "$PX4_SOURCE/Makefile" ]]; then
  if [[ -n "${PX4_DIR:-}" ]]; then
    fail "PX4_DIR is not a PX4-Autopilot checkout: $PX4_SOURCE"
  fi
  log "Cloning PX4-Autopilot $PX4_TAG with submodules"
  git clone --branch "$PX4_TAG" --depth 1 --recurse-submodules --shallow-submodules "$PX4_URL" "$PX4_SOURCE"
fi
PX4_SOURCE="$(realpath "$PX4_SOURCE")"
PX4_ACTUAL_TAG="$(git -C "$PX4_SOURCE" describe --tags --exact-match HEAD 2>/dev/null || true)"
[[ "$PX4_ACTUAL_TAG" == "$PX4_TAG" ]] || fail "PX4 must be exactly $PX4_TAG; found ${PX4_ACTUAL_TAG:-unavailable} in $PX4_SOURCE."
if git -C "$PX4_SOURCE" submodule status --recursive | grep -q '^-'; then
  log "Initializing missing PX4 submodules"
  git -C "$PX4_SOURCE" submodule update --init --recursive
fi

if git -C "$PX4_SOURCE" apply --reverse --check "$PX4_PATCH" >/dev/null 2>&1; then
  log "PX4 Ubuntu compatibility patch is already applied"
elif git -C "$PX4_SOURCE" apply --check "$PX4_PATCH" >/dev/null 2>&1; then
  log "Applying PX4 v1.11.3 Ubuntu 24 compatibility patch"
  git -C "$PX4_SOURCE" apply "$PX4_PATCH"
else
  fail "PX4 compatibility patch cannot be applied cleanly. Use a clean $PX4_TAG checkout or an already patched tree."
fi

log "Building native PX4 software-in-the-loop target"
export PATH="$VENV_DIR/bin:$PATH"
make -C "$PX4_SOURCE" px4_sitl_default
[[ -x "$PX4_SOURCE/build/px4_sitl_default/bin/px4" ]] || fail "PX4 SITL build did not produce the expected executable."
printf '%s\n' "$PX4_SOURCE" > "$RUNTIME_DIR/px4_path.txt"

log "Validating Python runtime, CUDA, and package closure"
"$VENV_DIR/bin/python" "$SCRIPT_DIR/tools/check_package.py" --runtime
log "Installation complete. Start with: ./run_experiments.sh smoke fast"
