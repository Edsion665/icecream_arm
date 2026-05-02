#!/usr/bin/env bash
set -euo pipefail

# PC 端启动脚本（适配 PC_RPI_UDP_PROTOCOL.md）
# 用法：
#   ./start_pc_control.sh sim 192.168.1.100
#   ./start_pc_control.sh nosim 192.168.1.100

MODE="${1:-sim}"          # sim | nosim
RPI_IP="${2:-192.168.31.211}"           # 默认 head 地址，可被参数覆盖
RPI_PORT="${RPI_PORT:-9870}"
LISTEN_HOST="${LISTEN_HOST:-0.0.0.0}"
TCP_PORT="${TCP_PORT:-9888}"
WEB_HOST="${WEB_HOST:-0.0.0.0}"
WEB_PORT="${WEB_PORT:-8877}"

cd "$(dirname "$0")/.."

COMMON_ARGS=(
  --listen "${LISTEN_HOST}"
  --port "${TCP_PORT}"
  --rpi-ip "${RPI_IP}"
  --rpi-port "${RPI_PORT}"
  --web-host "${WEB_HOST}"
  --web-port "${WEB_PORT}"
)

if [[ "${MODE}" == "sim" ]]; then
  echo "[start_pc_control] Launch SIM mode -> udp://${RPI_IP}:${RPI_PORT}"
  ~/isaacsim/python.sh -m arm_control_bridge.run_control --sim "${COMMON_ARGS[@]}"
elif [[ "${MODE}" == "nosim" ]]; then
  echo "[start_pc_control] Launch NOSIM mode -> udp://${RPI_IP}:${RPI_PORT}"
  python3 -m arm_control_bridge.run_control "${COMMON_ARGS[@]}"
else
  echo "Invalid mode: ${MODE}. Use sim or nosim."
  exit 1
fi

