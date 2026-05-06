"""笛卡尔（link4）几何解耦 IK 单步积分。"""

from __future__ import annotations

import numpy as np

from ..config import (
    CONTROL_DT,
    IK_DAMPING,
    IK_MAX_ITER,
    IK_POS_TOL,
    MAX_TARGET_RATE_RAD_S,
    POSE_VEL_MAX_M_S,
    Q4_SAFE_MAX,
    Q4_SAFE_MIN,
    SV_CRIT,
    SV_WARN,
)
from ..kinematics.urdf_kinematics import (
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
    NUM_JOINTS,
    Q4_OFFSET_RAD,
    Q4_Q23_COEFF,
    URDFKinematics,
)
from .state import CalculatorState


class IKCalculator:
    """在 ``MotionMode.POSE`` 下更新 ``state.q_full`` 中与位置相关的关节。"""

    def __init__(self, kin: URDFKinematics) -> None:
        self._kin = kin

    def step(self, state: CalculatorState, dt: float = CONTROL_DT) -> None:
        pose = state.pose_xyz.copy()
        if state.prev_pose_xyz is not None:
            max_step = POSE_VEL_MAX_M_S * dt
            d = pose - state.prev_pose_xyz
            n = float(np.linalg.norm(d))
            if n > max_step and n >= 1e-12:
                pose = state.prev_pose_xyz + d * (max_step / n)
        state.prev_pose_xyz = pose.copy()

        q_curr_3 = state.q_full[:3]
        j3 = self._kin.jacobian_link4_geometric_decouple(q_curr_3, state.q5_fixed_rad)
        sv = np.linalg.svd(j3, compute_uv=False)
        sv_min = float(sv.min())
        if sv_min < SV_CRIT:
            state.sing_hold = True
            return

        state.sing_hold = False
        damp = (
            min(IK_DAMPING * (SV_WARN / (sv_min + 1e-8)), IK_DAMPING * 20.0)
            if sv_min < SV_WARN
            else IK_DAMPING
        )
        q_sol, ok = self._kin.inverse_kinematics_link4_geometric_decouple(
            pose,
            q3_init=q_curr_3,
            q5_fixed=state.q5_fixed_rad,
            max_iter=IK_MAX_ITER,
            pos_tol=IK_POS_TOL,
            damping=damp,
        )
        if not ok:
            return
        q4c = Q4_OFFSET_RAD + Q4_Q23_COEFF * (q_sol[1] + q_sol[2])
        if not (Q4_SAFE_MIN <= q4c <= Q4_SAFE_MAX):
            return
        delta = q_sol[:NUM_JOINTS] - state.q_full[:NUM_JOINTS]
        dn = float(np.linalg.norm(delta))
        max_d = MAX_TARGET_RATE_RAD_S * dt
        if dn > max_d:
            delta *= max_d / dn
        state.q_full[:NUM_JOINTS] += delta
        state.q_full[3] = np.clip(
            Q4_OFFSET_RAD + Q4_Q23_COEFF * (state.q_full[1] + state.q_full[2]),
            float(JOINT_LIMITS_LOWER[3]),
            float(JOINT_LIMITS_UPPER[3]),
        )


# 历史类名（脚本/旧代码）
IK_calculator = IKCalculator
