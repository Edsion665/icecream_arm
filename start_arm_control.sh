#!/usr/bin/env bash
set -euo pipefail
cd ~/icecream
source ./icecream/bin/activate
# 控制模式：data=WebSocket 发 DATA:；tau_ff=FB+重力 TAU:（二选一）
export ARM_CONTROL_MODE=tau_ff
python -m arm_control.main
