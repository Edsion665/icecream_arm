#!/usr/bin/env bash
set -euo pipefail
cd ~/icecream
source ./icecream/bin/activate
# 控制模式：data=WebSocket 发 DATA:；tau_ff=FB+重力 TAU:（二选一）
export ARM_CONTROL_MODE=tau_ff
# STM32 MIT 上行 34 字节（0xAA 0x55 + 4×8 原始 CAN）；hex68=一行 68 个 hex；binary=帧头同步（与 TAU 独立）
export ARM_CONTROL_MIT_UPLINK=hex68
# 力矩前馈四轴弧度：mit=hex 解码电机 p（默认）；fb=FB 行。无 MIT 帧时自动回退 FB
# export ARM_CONTROL_TAU_FF_INPUT=fb
python -m arm_control.main
