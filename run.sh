#!/usr/bin/env bash
# icecreamarm 根目录：等待远端上线后，开 SSH 终端（相机、icecream_arm）+ 本地 icecream_vision。
#
# 用法：
#   ./run.sh
#   ICECREAM_SSH_PASS='你的密码' ./run.sh
#
# 环境变量（可选）：
#   REMOTE_HOST         默认 192.168.10.2
#   SSH_USER            默认 jianan
#   ICECREAM_SSH_PASS   SSH 密码（默认 1）
#   REMOTE_ROS_WS       远端 ros_ws（默认 ~/ros_ws）
#   REMOTE_ICECREAM     远端 icecream_arm（默认 ~/icecream_arm）
#   CAMERA_SCRIPT       相机启动脚本名（默认 run_ascamera_node.sh）
#   LOCAL_VISION_DIR         本地 icecream_vision 根目录（默认 ~/icecream_vision）
#   LOCAL_VISION_VENV        venv activate 脚本路径
#   LOCAL_VISION_RUN_SCRIPT  相对 LOCAL_VISION_DIR 的启动脚本（默认 robot-vision-ros2/scripts/run.sh）

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE_HOST="${REMOTE_HOST:-192.168.10.2}"
SSH_USER="${SSH_USER:-jianan}"
SSH_PASS="${ICECREAM_SSH_PASS:-1}"

REMOTE_ROS_WS="${REMOTE_ROS_WS:-~/ros_ws}"
REMOTE_ICECREAM="${REMOTE_ICECREAM:-~/icecream_arm}"
CAMERA_SCRIPT="${CAMERA_SCRIPT:-run_ascamera_node.sh}"

LOCAL_VISION_DIR="${LOCAL_VISION_DIR:-${HOME}/icecream_vision}"
LOCAL_VISION_VENV="${LOCAL_VISION_VENV:-/home/phoenix/icecream_vision/icecream_vision/bin/activate}"
LOCAL_VISION_RUN_SCRIPT="${LOCAL_VISION_RUN_SCRIPT:-robot-vision-ros2/scripts/run.sh}"

PING_INTERVAL_S="${PING_INTERVAL_S:-2}"
SSH_OPTS=(
  -o StrictHostKeyChecking=no
  -o UserKnownHostsFile=/dev/null
  -o ConnectTimeout=10
  -o ServerAliveInterval=30
  -o ServerAliveCountMax=3
)

log() { printf '[run] %s\n' "$*"; }
die() { printf '[run] 错误: %s\n' "$*" >&2; exit 1; }

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "未找到命令: $1（$2）"
}

wait_for_ping() {
  log "等待 ${REMOTE_HOST} 可 ping 通 …"
  while ! ping -c 1 -W 1 "${REMOTE_HOST}" >/dev/null 2>&1; do
    sleep "${PING_INTERVAL_S}"
  done
  log "${REMOTE_HOST} 已在线"
}

# 整条远端命令作为 ssh 的**一个**参数（勿写成 ssh host bash -lc cmd，会被拆参导致 cd 失效）
ssh_remote() {
  local remote_cmd="$1"
  sshpass -p "${SSH_PASS}" ssh "${SSH_OPTS[@]}" \
    "${SSH_USER}@${REMOTE_HOST}" "${remote_cmd}"
}

verify_ssh() {
  log "验证 SSH ${SSH_USER}@${REMOTE_HOST} …"
  if ! ssh_remote "echo ssh_ok" >/dev/null 2>&1; then
    die "SSH 登录失败，请检查用户名/密码/网络（可设置 ICECREAM_SSH_PASS）"
  fi
  log "SSH 连接正常"
}

# 启动前检查远端路径与脚本是否存在
preflight_remote() {
  log "检查远端目录与脚本 …"
  if ! ssh_remote "test -d ${REMOTE_ROS_WS}"; then
    die "远端不存在目录: ${REMOTE_ROS_WS}（可设置 REMOTE_ROS_WS）"
  fi
  if ! ssh_remote "test -f ${REMOTE_ROS_WS}/${CAMERA_SCRIPT}"; then
    die "远端未找到: ${REMOTE_ROS_WS}/${CAMERA_SCRIPT}"
  fi
  if ! ssh_remote "test -d ${REMOTE_ICECREAM}"; then
    die "远端不存在目录: ${REMOTE_ICECREAM}（可设置 REMOTE_ICECREAM）"
  fi
  if ! ssh_remote "test -f ${REMOTE_ICECREAM}/start.sh"; then
    die "远端未找到: ${REMOTE_ICECREAM}/start.sh"
  fi
  log "远端路径 OK"
}

build_camera_cmd() {
  printf 'cd %s && bash ./%s' "${REMOTE_ROS_WS}" "${CAMERA_SCRIPT}"
}

build_arm_cmd() {
  printf 'cd %s && bash ./start.sh' "${REMOTE_ICECREAM}"
}

# 整条远端命令一个参数传给 ssh（勿 ssh host bash -lc …，会拆参导致 cd 失效）
wrap_remote_ssh_shell() {
  local run_body="$1"
  printf '%s; echo; echo "[启动脚本已结束或被 Ctrl+C 中断] 已进入远端 shell，输入 exit 断开 SSH"; exec bash -i -l' \
    "${run_body}"
}

build_vision_local_cmd() {
  printf 'cd %s && source %s && bash %s' \
    "${LOCAL_VISION_DIR}" "${LOCAL_VISION_VENV}" "${LOCAL_VISION_RUN_SCRIPT}"
}

preflight_local_vision() {
  log "检查本地 icecream_vision …"
  if [[ ! -d "${LOCAL_VISION_DIR}" ]]; then
    die "本地不存在目录: ${LOCAL_VISION_DIR}（可设置 LOCAL_VISION_DIR）"
  fi
  if [[ ! -f "${LOCAL_VISION_VENV}" ]]; then
    die "未找到 venv: ${LOCAL_VISION_VENV}（可设置 LOCAL_VISION_VENV）"
  fi
  if [[ ! -f "${LOCAL_VISION_DIR}/${LOCAL_VISION_RUN_SCRIPT}" ]]; then
    die "未找到: ${LOCAL_VISION_DIR}/${LOCAL_VISION_RUN_SCRIPT}"
  fi
  log "本地 icecream_vision OK"
}

pick_terminal_launcher() {
  local title="$1"
  if command -v gnome-terminal >/dev/null 2>&1; then
    TERMINAL_LAUNCHER=(gnome-terminal --title="${title}" --)
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    TERMINAL_LAUNCHER=(xfce4-terminal --title="${title}" -x)
  elif command -v konsole >/dev/null 2>&1; then
    TERMINAL_LAUNCHER=(konsole -p tabtitle="${title}" -e)
  elif command -v xterm >/dev/null 2>&1; then
    TERMINAL_LAUNCHER=(xterm -T "${title}" -e)
  else
    die "未找到 gnome-terminal / xfce4-terminal / konsole / xterm"
  fi
}

# 本地新开终端执行命令（窗口保持）
open_local_terminal() {
  local title="$1"
  local local_cmd="$2"

  pick_terminal_launcher "${title}"

  "${TERMINAL_LAUNCHER[@]}" bash -lc "
set +e
echo '=== ${title} ===（本地）'
echo '${local_cmd}'
echo

${local_cmd}
st=\$?
echo
echo \"[退出码 \${st}] 按 Enter 关闭此窗口\"
read -r
" &
}

# 新开终端：ssh -tt 登录；自动跑启动脚本，中断后仍留在远端 shell
open_ssh_terminal() {
  local title="$1"
  local remote_cmd="$2"

  pick_terminal_launcher "${title}"

  local session_cmd session_quoted
  session_cmd=$(wrap_remote_ssh_shell "${remote_cmd}")
  session_quoted=$(printf '%q' "${session_cmd}")

  "${TERMINAL_LAUNCHER[@]}" bash -lc "
set +e
export ICECREAM_SSH_PASS=$(printf '%q' "${SSH_PASS}")
export REMOTE_HOST=$(printf '%q' "${REMOTE_HOST}")
export SSH_USER=$(printf '%q' "${SSH_USER}")
SSH_OPTS='-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ServerAliveInterval=30 -o ServerAliveCountMax=3'

echo '=== ${title} ==='
echo \"SSH -tt \${SSH_USER}@\${REMOTE_HOST}\"
echo \"先执行: ${remote_cmd}\"
echo \"脚本结束或 Ctrl+C 后 → 留在远端 shell（exit 断开 SSH）\"
echo

sshpass -p \"\${ICECREAM_SSH_PASS}\" ssh -tt \${SSH_OPTS} \"\${SSH_USER}@\${REMOTE_HOST}\" ${session_quoted}
st=\$?
echo
echo \"[SSH 已断开, 退出码 \${st}] 按 Enter 关闭本窗口\"
read -r
" &
}

main() {
  require_cmd ping "系统应自带 ping"
  require_cmd sshpass "请安装: sudo apt install -y sshpass"
  require_cmd ssh "请安装 openssh-client"

  preflight_local_vision

  wait_for_ping
  verify_ssh
  preflight_remote

  local cam_cmd arm_cmd vision_cmd
  cam_cmd=$(build_camera_cmd)
  arm_cmd=$(build_arm_cmd)
  vision_cmd=$(build_vision_local_cmd)

  log "新开终端 1/3（本地）：${vision_cmd}"
  open_local_terminal "icecream_vision（本地）" "${vision_cmd}"

  sleep 1

  log "新开终端 2/3（SSH）：${cam_cmd}"
  open_ssh_terminal "ascamera @ ${REMOTE_HOST}" "${cam_cmd}"

  sleep 1

  log "新开终端 3/3（SSH）：${arm_cmd}"
  open_ssh_terminal "icecream_arm @ ${REMOTE_HOST}" "${arm_cmd}"

  log "已启动 1 个本地 + 2 个 SSH 终端窗口"
}

main "$@"
