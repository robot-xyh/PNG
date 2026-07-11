#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="$(command -v python3)"
CONFIG="${ROOT}/config/betaflight.rk3588.example.json"
UNIT_NAME="png-betaflight-log-only.service"
UNIT_DIR="${HOME}/.config/systemd/user"
TEMPLATE="${ROOT}/deploy/systemd/${UNIT_NAME}.in"

usage() {
  echo "Usage: $0 [--project-root DIR] [--python PATH] [--config PATH]"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --project-root) ROOT="$(realpath "${2:?}")"; shift 2 ;;
    --python) PYTHON="$(realpath "${2:?}")"; shift 2 ;;
    --config) CONFIG="$(realpath "${2:?}")"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

TEMPLATE="${ROOT}/deploy/systemd/${UNIT_NAME}.in"
[[ -f "${TEMPLATE}" ]] || { echo "Missing service template: ${TEMPLATE}" >&2; exit 1; }
[[ -x "${PYTHON}" ]] || { echo "Python is not executable: ${PYTHON}" >&2; exit 1; }
[[ -f "${CONFIG}" ]] || { echo "Config does not exist: ${CONFIG}" >&2; exit 1; }
[[ "${ROOT}" != *" "* && "${PYTHON}" != *" "* && "${CONFIG}" != *" "* ]] || {
  echo "systemd installer does not support paths containing spaces" >&2
  exit 1
}

mkdir -p "${UNIT_DIR}" "${ROOT}/logs/service" "${ROOT}/logs/deployment"
UNIT_PATH="${UNIT_DIR}/${UNIT_NAME}"
sed \
  -e "s|@@PROJECT_ROOT@@|${ROOT}|g" \
  -e "s|@@PYTHON@@|${PYTHON}|g" \
  -e "s|@@CONFIG@@|${CONFIG}|g" \
  "${TEMPLATE}" >"${UNIT_PATH}"

grep -q -- '--control-mode log_only' "${UNIT_PATH}"
if grep -q -- '--allow-control' "${UNIT_PATH}"; then
  echo "Refusing to install a service containing --allow-control" >&2
  rm -f "${UNIT_PATH}"
  exit 1
fi

systemctl --user daemon-reload
systemctl --user disable --now "${UNIT_NAME}" >/dev/null 2>&1 || true

STAMP="$(date +%Y%m%d_%H%M%S)"
MANIFEST="${ROOT}/logs/deployment/systemd_${STAMP}.txt"
{
  echo "unit=${UNIT_PATH}"
  echo "unit_sha256=$(sha256sum "${UNIT_PATH}" | awk '{print $1}')"
  echo "project_root=${ROOT}"
  echo "python=${PYTHON}"
  echo "config=${CONFIG}"
  echo "enabled=$(systemctl --user is-enabled "${UNIT_NAME}" 2>/dev/null || true)"
  echo "active=$(systemctl --user is-active "${UNIT_NAME}" 2>/dev/null || true)"
} >"${MANIFEST}"

echo "Installed disabled/inactive unit: ${UNIT_PATH}"
echo "Deployment record: ${MANIFEST}"
