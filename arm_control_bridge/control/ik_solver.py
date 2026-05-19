"""笛卡尔（link4）几何解耦 IK 单步积分。"""

from __future__ import annotations

import numpy as np

from ..config import CONFIG, IK_CONFIG
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

    def set_pose_reach_target(self, state: CalculatorState) -> None:
        """对最终 ``pose_xyz`` 做一次完整 IK，写入 ``q_pose_target_rad``（仅到位判定，不改 ``q_full``）。"""
        pose = state.pose_xyz.copy()
        q_sol, ok = self._kin.inverse_kinematics_link4_geometric_decouple(
            pose,
            q3_init=state.q_full[:3],
            q5_fixed=state.q5_fixed_rad,
            max_iter=IK_CONFIG.ik_max_iter,
            pos_tol=IK_CONFIG.ik_pos_tol,
            damping=IK_CONFIG.ik_damping,
        )
        if not ok:
            state.q_pose_target_rad = None
            return
        q4c = Q4_OFFSET_RAD + Q4_Q23_COEFF * (q_sol[1] + q_sol[2])
        if not (IK_CONFIG.q4_safe_min <= q4c <= IK_CONFIG.q4_safe_max):
            state.q_pose_target_rad = None
            return
        if float(np.hypot(pose[0], pose[1])) > 1e-6:
            q_sol[0] = -np.arctan2(pose[1], pose[0])
        q_tgt = q_sol[:4].copy()
        q_tgt[3] = float(np.clip(q4c, IK_CONFIG.q4_safe_min, IK_CONFIG.q4_safe_max))
        state.q_pose_target_rad = q_tgt

    def step(self, state: CalculatorState, dt: float = CONFIG.control_dt) -> None:
        pose = state.pose_xyz.copy()
        if state.prev_pose_xyz is not None:
            max_step = CONFIG.pose_vel_max_m_s * dt
            d = pose - state.prev_pose_xyz
            n = float(np.linalg.norm(d))
            if n > max_step and n >= 1e-12:
                pose = state.prev_pose_xyz + d * (max_step / n)
        state.prev_pose_xyz = pose.copy()

        q_curr_3 = state.q_full[:3]
        j3 = self._kin.jacobian_link4_geometric_decouple(q_curr_3, state.q5_fixed_rad)
        sv = np.linalg.svd(j3, compute_uv=False)
        sv_min = float(sv.min())
        if sv_min < IK_CONFIG.sv_crit:
            state.sing_hold = True
            return

        state.sing_hold = False
        damp = (
            min(
                IK_CONFIG.ik_damping * (IK_CONFIG.sv_warn / (sv_min + 1e-8)),
                IK_CONFIG.ik_damping * 20.0,
            )
            if sv_min < IK_CONFIG.sv_warn
            else IK_CONFIG.ik_damping
        )
        q_sol, ok = self._kin.inverse_kinematics_link4_geometric_decouple(
            pose,
            q3_init=q_curr_3,
            q5_fixed=state.q5_fixed_rad,
            max_iter=IK_CONFIG.ik_max_iter,
            pos_tol=IK_CONFIG.ik_pos_tol,
            damping=damp,
        )
        if not ok:
            return
        q4c = Q4_OFFSET_RAD + Q4_Q23_COEFF * (q_sol[1] + q_sol[2])
        if not (IK_CONFIG.q4_safe_min <= q4c <= IK_CONFIG.q4_safe_max):
            return
        delta = q_sol[:NUM_JOINTS] - state.q_full[:NUM_JOINTS]
        delta[0] = 0.0  # joint1 单独解析赋值，不参与限速
        dn = float(np.linalg.norm(delta))
        max_d = CONFIG.max_target_rate_rad_s * dt
        if dn > max_d:
            delta *= max_d / dn
        state.q_full[:NUM_JOINTS] += delta
        # joint1 解析约束：绕 -z 轴，q1 = -atan2(y, x)；xy 均为零时保持当前值
        xy_norm = float(np.hypot(pose[0], pose[1]))
        if xy_norm > 1e-6:
            state.q_full[0] = -np.arctan2(pose[1], pose[0])
        state.q_full[3] = np.clip(
            Q4_OFFSET_RAD + Q4_Q23_COEFF * (state.q_full[1] + state.q_full[2]),
            float(JOINT_LIMITS_LOWER[3]),
            float(JOINT_LIMITS_UPPER[3]),
        )


# 历史类名（脚本/旧代码）
IK_calculator = IKCalculator
