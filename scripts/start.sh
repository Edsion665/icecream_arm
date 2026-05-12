#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PARENT="$(cd "${REPO_ROOT}/.." && pwd)"
PKG_NAME="$(basename "${REPO_ROOT}")"
ENTRY_MODULE="${PKG_NAME}.main"

export ARM_CONTROL_RPI_UDP="${ARM_CONTROL_RPI_UDP:-1}"
export ARM_CONTROL_TAU_HZ="${ARM_CONTROL_TAU_HZ:-25}"
export ARM_CONTROL_RPI_UDP_PORT="${ARM_CONTROL_RPI_UDP_PORT:-9870}"
export ARM_CONTROL_COLD_HOLD_SEC="${ARM_CONTROL_COLD_HOLD_SEC:-0}"
export ARM_CONTROL_MIT_CMD_KP_FLOAT="${ARM_CONTROL_MIT_CMD_KP_FLOAT:-0}"
export ARM_CONTROL_GRAVITY_FF="${ARM_CONTROL_GRAVITY_FF:-1}"
export ARM_CONTROL_INIT_FEEDBACK_WAIT_SEC="${ARM_CONTROL_INIT_FEEDBACK_WAIT_SEC:-10}"

cd "${PARENT}"

if [[ -x "${REPO_ROOT}/.venv/bin/python" ]]; then
  exec "${REPO_ROOT}/.venv/bin/python" -m "${ENTRY_MODULE}"
fi

if command -v uv >/dev/null 2>&1; then
  exec uv run --project "${REPO_ROOT}" python -m "${ENTRY_MODULE}"
fi

exec python3 -m "${ENTRY_MODULE}"
