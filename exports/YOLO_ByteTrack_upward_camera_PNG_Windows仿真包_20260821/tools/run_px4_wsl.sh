#!/usr/bin/env bash
set -euo pipefail

pid_file="${1:?pid file is required}"
px4_root="${PX4_ROOT:-/opt/png-px4/PX4-Autopilot}"

if [[ "${PNG_PX4_SESSION_CHILD:-0}" != "1" ]]; then
  exec setsid env PNG_PX4_SESSION_CHILD=1 "$0" "$@"
fi

cleanup() {
  rm -f "$pid_file"
}
trap cleanup EXIT
printf '%s\n' "$$" >"$pid_file"

cd "$px4_root"
actual_tag="$(git describe --tags --exact-match HEAD 2>/dev/null || true)"
if [[ "$actual_tag" != "v1.11.3" ]]; then
  echo "Refusing to start PX4: expected v1.11.3, found ${actual_tag:-unknown}" >&2
  exit 2
fi

make px4_sitl_default none_iris
