"""默认网络、控制参数与标定加载。"""

from __future__ import annotations

import os
import re
from typing import List

CONTROL_HZ = 25.0
CONTROL_DT = 1.0 / CONTROL_HZ

DEFAULT_TCP_PORT = 9888
DEFAULT_UDP_PORT = 9870

APPROACH_GAIN = CONTROL_HZ * 0.1
MAX_JOINT_VEL_RAD_S = 0.6
MAX_TARGET_RATE_RAD_S = 1.5
POSE_VEL_MAX_M_S = 0.1


def frontend_pose_to_internal_m(x: float, y: float, z: float) -> tuple[float, float, float]:
    """前端输入的 link4 目标 (x,y,z) 米（父节点 local）→ IK local。

    约定：
    - 前端发送的是机械臂父节点坐标系（local）下的位置；
    - IK 也使用同一 local（URDF 基座 link0）求解；
    - 这里不做坐标变换，保持数值一一对应。
    """
    return (float(x), float(y), float(z))


# POSE 几何解耦（仅影响 q4）：q4 = rad(offset) + coeff*(q2+q3)。v8 常用 offset=0、coeff=-1 → q4=-(q2+q3)。
Q4_GEOMETRIC_OFFSET_DEG = -180.0
Q4_GEOMETRIC_Q23_COEFF = -1.0

# 笛卡尔下末节朝向：link4 位置 IK 与 q5 无关，在 q5_fixed 上叠加偏置可绕 joint5 翻转工具（通常改为朝 -Z）。
# 单位：度；实机/UDP 通过 JointFrame.joint5_rel_deg 下发第 5 轴相对角。
POSE_Q5_EXTRA_DEG = 0
Q4_BLEND_TIME_S = 2

IK_DAMPING = 1e-2
IK_MAX_ITER = 80
IK_POS_TOL = 1e-5

Q4_SAFE_MIN, Q4_SAFE_MAX = -2.8, 2.8
SV_WARN = 0.025
SV_CRIT = 0.005

# --sim：‖q_actual−q_cmd‖ 超阈值时把仿真关节对齐到 q_cmd（不把 q_cmd 拉回滞后仿真，避免「往回拽」）
SIM_TRACK_SNAP_THRESHOLD_RAD = 0.35

# 协议「相对标定」的基准角（度）：与 initial_position.md 一致。当前 v8 URDF 以关节全 0 为伸直/设计零位，不再单独做一组「姿态标定」变换。
DEFAULT_Q_CALIB_DEG_LIST = [0.0, 0.0, 0.0, 0.0, 0.0]

# 启动时主臂前四轴相对上述标定零点的初始角（度）；不改变标定本身，仅 reset_command() 首次姿态。
DEFAULT_INITIAL_JOINT_REL_DEG_4 = [0.0, 90.0, -180.0, -20.0]


def load_calibration_deg(calib_file: str | None) -> List[float]:
    """从 initial_position.md 或指定文件解析标定关节角（度）。"""
    if not calib_file:
        return list(DEFAULT_Q_CALIB_DEG_LIST)
    try:
        if not os.path.isfile(calib_file):
            return list(DEFAULT_Q_CALIB_DEG_LIST)
        with open(calib_file, "r", encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return list(DEFAULT_Q_CALIB_DEG_LIST)
    pattern = re.compile(r"joint(\d)\s*=\s*([+-]?\d+\.?\d*)\s*°", re.IGNORECASE)
    found = {}
    for m in pattern.finditer(text):
        idx = int(m.group(1))
        if 1 <= idx <= 5:
            found[idx] = float(m.group(2))
    if len(found) == 5:
        return [found[i] for i in range(1, 6)]
    return list(DEFAULT_Q_CALIB_DEG_LIST)

