#!/usr/bin/env bash
# 遥操作启动脚本：SSH 启动从臂 + 本地 teleop_runner
#
# 用法：
#   ./start_teleop.sh
#   ./start_teleop.sh 192.168.31.195 192.168.31.211
#
# 参数：
#   $1  从臂树莓派 IP（默认 192.168.31.195）
#   $2  主臂树莓派 IP（默认 192.168.31.211）

set -euo pipefail

SLAVE_IP="${1:-192.168.31.195}"
MASTER_IP="${2:-192.168.31.211}"
SLAVE_USER="${SLAVE_USER:-jianan}"
SLAVE_PASS="${SLAVE_PASS:-1}"
SLAVE_WS_PORT="${SLAVE_WS_PORT:-8765}"
MASTER_UDP_PORT="${MASTER_UDP_PORT:-9870}"

cd "$(dirname "$0")/.."

echo "[teleop] 从臂: ${SLAVE_USER}@${SLAVE_IP}  主臂: ${MASTER_IP}:${MASTER_UDP_PORT}"

# 启动从臂（后台 SSH，退出时自动终止）
sshpass -p "${SLAVE_PASS}" ssh \
    -o StrictHostKeyChecking=no \
    -o ServerAliveInterval=5 \
    "${SLAVE_USER}@${SLAVE_IP}" \
    "bash ~/icecreamPi/start.sh" &
SLAVE_PID=$!
echo "[teleop] 从臂 PID=${SLAVE_PID}，等待 WS 就绪…"
sleep 8

# 启动本地 teleop_runner
python3 -m arm_control_bridge.teleop.teleop_runner \
    --slave-ip "${SLAVE_IP}" \
    --slave-ws-port "${SLAVE_WS_PORT}" \
    --master-ip "${MASTER_IP}" \
    --master-port "${MASTER_UDP_PORT}" &
RUNNER_PID=$!
echo "[teleop] teleop_runner PID=${RUNNER_PID}"

# Ctrl-C 同时终止两个进程
trap 'echo "[teleop] 停止…"; kill ${RUNNER_PID} ${SLAVE_PID} 2>/dev/null; wait' INT TERM
wait ${RUNNER_PID}
kill ${SLAVE_PID} 2>/dev/null || true
