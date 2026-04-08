#!/usr/bin/env bash
set -euo pipefail

cd ~/icecream
source ./icecream/bin/activate

# 用法：
#   ./start_agent.sh home   # 归位后进入主循环
#   ./start_agent.sh work   # 到工作位并在主循环中保持(p=目标, v=0 + 重力补偿)
#   ./start_agent.sh run    # 不做启动姿态，直接进入主循环
MODE="${1:-run}"

export ARM_CONTROL_MODE=tau_ff
export ARM_CONTROL_MIT_UPLINK=hex68
export ARM_CONTROL_TAU_HZ=25

# 启动姿态与速率（两函数在 arm_control/init_pose_actions.py）
case "${MODE}" in
  home)
    export ARM_CONTROL_BOOT_POSE=home
    export ARM_CONTROL_MIT_CMD_KP_FLOAT=0
    ;;
  work)
    export ARM_CONTROL_BOOT_POSE=work
    export ARM_CONTROL_MIT_CMD_KP_FLOAT=0
    ;;
  run)
    export ARM_CONTROL_BOOT_POSE=none
    export ARM_CONTROL_MIT_CMD_KP_FLOAT=1
    ;;
  *)
    echo "未知模式: ${MODE}"
    echo "用法: $0 [home|work|run]"
    exit 2
    ;;
esac

# 与启动姿态配套参数
export ARM_CONTROL_BOOT_POSE_VMAX=0.4
# export ARM_CONTROL_BOOT_POSE_TOL_RAD=0.008
# export ARM_CONTROL_BOOT_POSE_WAIT_FB_SEC=45
# export ARM_CONTROL_BOOT_POSE_MAX_SEC=180

# 重要：这里不再使用旧的 RPI_UDP 预移动流程
export ARM_CONTROL_RPI_PREMOVE_SKIP=1

# 如需接收 PC UDP 流，取消下一行注释（work 模式若要持续保持，建议保持关闭）
# export ARM_CONTROL_RPI_UDP=1

python -m arm_control.main

