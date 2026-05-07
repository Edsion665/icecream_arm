#!/usr/bin/env bash
set -euo pipefail

# PC 端启动脚本（适配 PC_RPI_UDP_PROTOCOL.md）
# 仓库布局：icecreamarm/arm-control-bridge/start_pc_control.sh（本文件）
#          仓库根 = 本目录上一级；根目录下需有 arm_control_bridge -> arm-control-bridge 的符号链接，
#          以便 python -m arm_control_bridge.run_control 能解析包名。
# 用法：
#   ./start_pc_control.sh sim 192.168.1.100
#   ./start_pc_control.sh nosim 192.168.1.100
# sim 模式：依次尝试 $HOME/isaacsim/python.sh、$HOME/isaac-sim/python.sh（先找到先用）

MODE="${1:-sim}"          # sim | nosim
RPI_IP="${2:-192.168.31.211}"           # 默认 head 地址，可被参数覆盖
RPI_PORT="${RPI_PORT:-9870}"
LISTEN_HOST="${LISTEN_HOST:-0.0.0.0}"
TCP_PORT="${TCP_PORT:-9888}"

BRIDGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${BRIDGE_DIR}/.." && pwd)"
cd "${REPO_ROOT}"
export PYTHONPATH="${REPO_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

if [[ ! -d "${REPO_ROOT}/arm_control_bridge" ]]; then
  echo "[start_pc_control] 缺少包名目录 ${REPO_ROOT}/arm_control_bridge（应为指向 arm-control-bridge 的符号链接）。" >&2
  echo "  在仓库根执行: ln -sfn arm-control-bridge arm_control_bridge" >&2
  exit 1
fi

COMMON_ARGS=(
  --listen "${LISTEN_HOST}"
  --port "${TCP_PORT}"
  --rpi-ip "${RPI_IP}"
  --rpi-port "${RPI_PORT}"
)

if [[ "${MODE}" == "sim" ]]; then
  ISAAC_PY=""
  for d in "${HOME}/isaacsim" "${HOME}/isaac-sim"; do
    [[ -x "${d}/python.sh" ]] && ISAAC_PY="${d}/python.sh" && break
  done
  if [[ -z "${ISAAC_PY}" ]]; then
    echo "[start_pc_control] 未找到 ${HOME}/isaac-sim/python.sh 与 ${HOME}/isaacsim/python.sh" >&2
    exit 1
  fi
  echo "[start_pc_control] Launch SIM mode -> udp://${RPI_IP}:${RPI_PORT} (${ISAAC_PY})"
  "${ISAAC_PY}" -m arm_control_bridge.run_control --sim "${COMMON_ARGS[@]}"
elif [[ "${MODE}" == "nosim" ]]; then
  echo "[start_pc_control] Launch NOSIM mode -> udp://${RPI_IP}:${RPI_PORT}"
  python3 -m arm_control_bridge.run_control "${COMMON_ARGS[@]}"
else
  echo "Invalid mode: ${MODE}. Use sim or nosim."
  exit 1
fi
