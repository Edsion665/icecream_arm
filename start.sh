#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_DIR="${ROOT_DIR}/icecreamPi"

if [[ ! -d "${PI_DIR}" ]]; then
  echo "icecreamPi 目录不存在: ${PI_DIR}" >&2
  exit 1
fi

exec "${PI_DIR}/scripts/start.sh" "$@"
