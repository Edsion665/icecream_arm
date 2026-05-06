"""关节空间模式：相对标定角直接映射到 ``q_full``。"""

from __future__ import annotations

import numpy as np

from ..config import CONTROL_DT
from .state import ARM_AXES, CalculatorState


class JointMover:
    """与笛卡尔趋近分离，避免关节模式双层限速。"""

    def step(self, state: CalculatorState, dt: float = CONTROL_DT) -> None:
        del dt  # 关节模式无显式 dt 依赖，保留签名与旧 ``mover`` 一致。
        q_tgt = state.q_full.copy()
        q_tgt[:ARM_AXES] = state.q_calib_rad[:ARM_AXES] + np.deg2rad(state.joint_rel_deg_4)
        state.q_full[:ARM_AXES] = q_tgt[:ARM_AXES]


mover = JointMover
