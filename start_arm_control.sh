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
# MIT kp：1=浮游(kp=0)；不设=正常(kp 为 config 中 MIT_CMD_FIXED_KP_NORMAL)
# export ARM_CONTROL_MIT_CMD_KP_FLOAT=1
#
# PC 仿真关节流（与 cartesian_ik_verify --rpi-ip 配套）：UDP 9870，p/v 驱动 MIT 下行；建议与 PC --ik-rate 一致提高 ARM_CONTROL_TAU_HZ
# export ARM_CONTROL_TAU_HZ=25
# export ARM_CONTROL_RPI_UDP=1
# export ARM_CONTROL_RPI_UDP_PORT=9870
# 注意：启用后勿再在同一端口运行 rpi_receiver.py（会抢 bind）。
# RPI UDP 默认先预移动到相对标定角(°) 18.99,-18.64,-121.12,-77.51（v_max=0.4 rad/s），到位后再 bind UDP：
# export ARM_CONTROL_RPI_PREMOVE_SKIP=1   # 跳过预移动
# export ARM_CONTROL_RPI_PREMOVE_VMAX=0.4
# export ARM_CONTROL_RPI_PREMOVE_REL_DEG=18.99,-18.64,-121.12,-77.51
#
# M2/M3 与重力前馈协同：下行 MIT 35B 中 M2、M3 走离散步进（默认 M2 +140°、M3 +130°），力矩 t 仍由 Pinocchio 按当前反馈算；
# M1、M4 命令 p 跟随当前反馈（保持不动）。需 MIT 上行（见上 ARM_CONTROL_MIT_UPLINK）。
# 关闭协同：不设置或设为 0 即可。
# export ARM_CONTROL_M23_GRAVITY_TRAJ=1
# export ARM_CONTROL_M23_DEG_M2=140
# export ARM_CONTROL_M23_DEG_M3=130
# export ARM_CONTROL_M23_V_MAX=0.3

export ARM_CONTROL_TAU_HZ=25          # 与 PC --ik-rate 对齐
export ARM_CONTROL_RPI_UDP=1

python -m arm_control.main
