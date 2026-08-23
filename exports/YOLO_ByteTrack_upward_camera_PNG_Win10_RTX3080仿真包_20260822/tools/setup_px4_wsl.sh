#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
export LC_ALL=C.UTF-8
PX4_ROOT=/opt/png-px4/PX4-Autopilot
PX4_TAG=v1.11.3

source /etc/os-release
if [[ "${VERSION_ID:-}" != "20.04" ]]; then
  echo "Ubuntu 20.04 is required; found ${PRETTY_NAME:-unknown}" >&2
  exit 2
fi

apt-get update
apt-get install -y --no-install-recommends \
  build-essential ca-certificates ccache cmake git genromfs g++ gcc gdb iproute2 make ninja-build \
  python3 python3-cerberus python3-dev python3-empy python3-future python3-jinja2 python3-jsonschema \
  python3-kconfiglib python3-lxml python3-numpy python3-packaging python3-pip python3-pyserial \
  python3-setuptools python3-toml python3-wheel python3-yaml unzip wget zip

mkdir -p "$(dirname "$PX4_ROOT")"
if [[ ! -d "$PX4_ROOT/.git" ]]; then
  git clone --branch "$PX4_TAG" --depth 1 --recursive --shallow-submodules \
    https://github.com/PX4/PX4-Autopilot.git "$PX4_ROOT"
fi

cd "$PX4_ROOT"
git fetch --tags origin "$PX4_TAG"
git checkout --detach "$PX4_TAG"
git submodule sync --recursive
git submodule update --init --recursive

actual_tag="$(git describe --tags --exact-match HEAD)"
if [[ "$actual_tag" != "$PX4_TAG" ]]; then
  echo "PX4 tag mismatch: expected=$PX4_TAG actual=$actual_tag" >&2
  exit 2
fi

make px4_sitl_default
test -x build/px4_sitl_default/bin/px4
printf '%s\n' "$actual_tag" >/opt/png-px4/px4_version.txt
