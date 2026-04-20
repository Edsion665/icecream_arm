#!/usr/bin/env bash
set -euo pipefail

# 用法：
#   ./start_pc_control.sh sim 192.168.1.100
#   ./start_pc_control.sh nosim 192.168.1.100

MODE="${1:-sim}"          # sim | nosim
RPI_IP="${2:-}"           # 必填：树莓派 IP
RPI_PORT="${RPI_PORT:-9870}"
LISTEN_HOST="${LISTEN_HOST:-0.0.0.0}"
TCP_PORT="${TCP_PORT:-9888}"
WEB_HOST="${WEB_HOST:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-8765}"

if [[ -z "${RPI_IP}" ]]; then
  echo "Usage: $0 <sim|nosim> <rpi_ip>"
  exit 1
fi

cd "$(dirname "$0")/.."

COMMON_ARGS=(
  --listen "${LISTEN_HOST}"
  --port "${TCP_PORT}"
  --rpi-ip "${RPI_IP}"
  --rpi-port "${RPI_PORT}"
  --udp-format v2
  --web-host "${WEB_HOST}"
  --web-port "${WEB_PORT}"
)

if [[ "${MODE}" == "sim" ]]; then
  echo "[start_pc_control] Launch SIM mode -> udp://${RPI_IP}:${RPI_PORT}"
  ~/isaac-sim/python.sh -m arm_control_bridge.run_control --sim "${COMMON_ARGS[@]}"
elif [[ "${MODE}" == "nosim" ]]; then
  echo "[start_pc_control] Launch NOSIM mode -> udp://${RPI_IP}:${RPI_PORT}"
  python3 -m arm_control_bridge.run_control "${COMMON_ARGS[@]}"
else
  echo "Invalid mode: ${MODE}. Use sim or nosim."
  exit 1
fi

