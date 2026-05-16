#!/usr/bin/env bash
# icecreamarm 根目录一键启动：先 arm_control_bridge，约 2 秒后再 head。
#
# 用法：
#   ./start.sh [sim|nosim] [RPI_IP] [-- head 传给 start_head.sh 的参数...]
#   ./start.sh 192.168.31.211              # 仅 IP，模式默认 nosim
#   ./start.sh sim 192.168.1.100
#   ./start.sh nosim 192.168.1.100 -- --config config.yaml
#
# 环境变量（可选）：
#   RPI_IP          未传位置参数时的默认树莓派 / head IP
#   BRIDGE_MODE     未传模式时的默认：nosim（可设为 sim）
#   HEAD_DELAY_S    启动 head 前等待秒数，默认 2

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_SCRIPT="${ROOT}/arm_control_bridge/start_pc_control.sh"
HEAD_SCRIPT="${ROOT}/head/start_head.sh"

MODE="${BRIDGE_MODE:-nosim}"
IP="${RPI_IP:-192.168.31.211}"
DELAY_S="${HEAD_DELAY_S:-2}"

if [[ ! -x "$BRIDGE_SCRIPT" ]]; then
  echo "[start] 未找到或可执行: $BRIDGE_SCRIPT" >&2
  exit 1
fi
if [[ ! -f "$HEAD_SCRIPT" ]]; then
  echo "[start] 未找到: $HEAD_SCRIPT（仓库内实际文件名为 start_head.sh）" >&2
  exit 1
fi

HEAD_ARGS=()
if [[ $# -gt 0 ]]; then
  if [[ "$1" == "sim" || "$1" == "nosim" ]]; then
    MODE="$1"
    shift
  elif [[ "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    MODE="nosim"
  fi
fi
if [[ $# -gt 0 && "$1" != "--" ]]; then
  IP="$1"
  shift
fi
if [[ $# -gt 0 && "$1" == "--" ]]; then
  shift
  HEAD_ARGS=("$@")
fi

cleanup() {
  if [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
    kill "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[start] arm_control_bridge: mode=${MODE} rpi_ip=${IP}"
"$BRIDGE_SCRIPT" "$MODE" "$IP" &
BRIDGE_PID=$!

echo "[start] 等待 ${DELAY_S}s 后启动 head …"
sleep "${DELAY_S}"

echo "[start] head: ${HEAD_SCRIPT} ${HEAD_ARGS[*]:-}"
bash "$HEAD_SCRIPT" "${HEAD_ARGS[@]}"
RET=$?
cleanup
trap - EXIT INT TERM
exit "$RET"
