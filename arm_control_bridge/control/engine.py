"""统一 ``step``：命令应用 + IK/关节模式 + 下发帧构造。"""

from __future__ import annotations

import time
from typing import Optional, Tuple

import numpy as np

from ..config import CONFIG, IK_CONFIG, frontend_pose_to_internal_m
from ..kinematics.urdf_kinematics import (
    Q4_OFFSET_RAD,
    Q4_Q23_COEFF,
    URDFKinematics,
)
from ..io.listener import MotionCommand4Axis
from .joint_mover import JointMover
from .state import ARM_AXES, CalculatorState, JointFrame, MotionMode


class CalculatorEngine:
    """``step(command, state, dt) -> JointFrame`` 的单一入口。"""

    def __init__(self, kin: URDFKinematics) -> None:
        self._kin = kin
        self._mover = JointMover()

    def _pose_to_joints(self, xyz: np.ndarray, state: CalculatorState) -> bool:
        """IK 解算 xyz → joint_rel_deg_4，成功返回 True，失败保持原状态返回 False。"""
        q_sol, ok = self._kin.inverse_kinematics_link4_geometric_decouple(
            xyz,
            q3_init=state.q_full[:3],
            q5_fixed=state.q5_fixed_rad,
            max_iter=IK_CONFIG.ik_max_iter,
            pos_tol=IK_CONFIG.ik_pos_tol,
            damping=IK_CONFIG.ik_damping,
        )
        if not ok:
            return False
        q4c = float(Q4_OFFSET_RAD + Q4_Q23_COEFF * (q_sol[1] + q_sol[2]))
        if not (IK_CONFIG.q4_safe_min <= q4c <= IK_CONFIG.q4_safe_max):
            return False
        if float(np.hypot(xyz[0], xyz[1])) > 1e-6:
            q_sol[0] = -np.arctan2(xyz[1], xyz[0])
        q_tgt = q_sol[:ARM_AXES].copy()
        q_tgt[3] = float(np.clip(q4c, IK_CONFIG.q4_safe_min, IK_CONFIG.q4_safe_max))
        state.joint_rel_deg_4 = np.rad2deg(q_tgt) - state.q_calib_deg[:ARM_AXES]
        state.q_pose_target_rad = q_tgt
        return True

    def apply_command(self, cmd: MotionCommand4Axis, state: CalculatorState) -> None:
        p = cmd.payload
        if cmd.kind == "pose":
            xi, yi, zi = frontend_pose_to_internal_m(float(p["x"]), float(p["y"]), float(p["z"]))
            xyz = np.array([xi, yi, zi], dtype=float)
            self._pose_to_joints(xyz, state)
            state.pose_xyz = xyz
            state.mode = MotionMode.JOINTS
        elif cmd.kind == "pose_delta":
            xyz = state.pose_xyz.copy()
            xyz[0] += float(p["dx"])
            xyz[1] += float(p["dy"])
            xyz[2] += float(p["dz"])
            if self._pose_to_joints(xyz, state):
                state.pose_xyz = xyz
            state.mode = MotionMode.JOINTS
        elif cmd.kind == "joints":
            state.mode = MotionMode.JOINTS
            arr = p["axes_rel_deg"]
            state.joint_rel_deg_4 = np.array([float(arr[i]) for i in range(ARM_AXES)], dtype=float)
        elif cmd.kind == "joints_delta":
            state.mode = MotionMode.JOINTS
            arr = p["deltas_rel_deg"]
            for i in range(min(len(arr), ARM_AXES)):
                state.joint_rel_deg_4[i] += float(arr[i])

    def step(
        self,
        command: Optional[MotionCommand4Axis],
        state: CalculatorState,
        dt: float = CONFIG.control_dt,
    ) -> JointFrame:
        if not state.initialized:
            state.reset_command()
        if command is not None:
            self.apply_command(command, state)

        self._mover.step(state, dt=dt)

        wrist_rad = state.wrist_joint_rad()
        state.q_full[4] = wrist_rad
        state.q_cmd[4] = wrist_rad

        # Bridge 侧不做速度限制，直接跟踪目标；速度规划全部交由 Pi 侧 ramp 处理。
        state.q_cmd[:ARM_AXES] = state.q_full[:ARM_AXES].copy()

        p_rel_deg = np.rad2deg(state.q_cmd[:ARM_AXES]) - state.q_calib_deg[:ARM_AXES]
        omega_arm = np.zeros(ARM_AXES, dtype=float)
        j5_rel = float(state.wrist_rel_deg)
        return JointFrame(
            arm_rel_deg=p_rel_deg.copy(),
            arm_omega_rad_s=omega_arm.copy(),
            servo_deg=state.servo_deg.copy(),
            wrist_rel_deg=float(state.wrist_rel_deg),
            wrist_omega_rad_s=0.0,
            grip_state=float(state.grip_state),
            stepper_deg_cmd=float(state.stepper_deg_cmd),
            conveyor_run_cmd=float(state.conveyor_run_cmd),
            mode=state.mode.value,
            timestamp=time.monotonic(),
            joint5_rel_deg=j5_rel,
        )

    def is_reached(
        self,
        state: CalculatorState,
        *,
        fb_arm_rad: Optional[np.ndarray] = None,
        joints_tol_deg: float = CONFIG.reached_joints_tol_deg,
    ) -> Tuple[bool, float]:
        """到位判定：``(reached, error)``；error 均为关节角度范数（度）。"""
        if fb_arm_rad is not None:
            q_actual = np.asarray(fb_arm_rad, dtype=float).ravel()[:ARM_AXES]
        else:
            q_actual = state.q_cmd[:ARM_AXES].copy()
        q_target = state.q_calib_rad[:ARM_AXES] + np.deg2rad(state.joint_rel_deg_4)
        err_deg = np.rad2deg(np.abs(q_actual - q_target))
        error = float(np.linalg.norm(err_deg))
        reached = bool(np.all(err_deg < joints_tol_deg))
        return reached, error
