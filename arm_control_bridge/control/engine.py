"""统一 ``step``：命令应用 + IK/关节模式 + 下发帧构造。"""

from __future__ import annotations

import time
from typing import Optional, Tuple

import numpy as np

from ..config import CONFIG, IK_CONFIG, frontend_pose_to_internal_m
from ..kinematics.urdf_kinematics import (
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
    NUM_JOINTS,
    Q4_OFFSET_RAD,
    Q4_Q23_COEFF,
    URDFKinematics,
)
from ..io.listener import MotionCommand4Axis
from .ik_solver import IKCalculator
from .joint_mover import JointMover
from .state import ARM_AXES, CalculatorState, JointFrame, MotionMode


class CalculatorEngine:
    """``step(command, state, dt) -> JointFrame`` 的单一入口。"""

    def __init__(self, kin: URDFKinematics) -> None:
        self._kin = kin
        self._ik = IKCalculator(kin)
        self._mover = JointMover()

    def apply_command(self, cmd: MotionCommand4Axis, state: CalculatorState) -> None:
        p = cmd.payload
        if cmd.kind == "pose":
            prev_mode = state.mode
            state.mode = MotionMode.POSE
            xi, yi, zi = frontend_pose_to_internal_m(float(p["x"]), float(p["y"]), float(p["z"]))
            state.pose_xyz = np.array([xi, yi, zi], dtype=float)
            # 从当前 FK 起步限速，避免 POSE 模式下重复绝对 pose 时跳过渐变导致关节突变。
            state.prev_pose_xyz = self._kin.forward_kinematics_position_link4(state.q_full).copy()
            if prev_mode != MotionMode.POSE:
                state.q4_blend_active = True
                state.q4_blend_start_rad = float(state.q_cmd[3])
                state.q4_blend_t = 0.0
        elif cmd.kind == "pose_delta":
            state.mode = MotionMode.POSE
            state.pose_xyz[0] += float(p["dx"])
            state.pose_xyz[1] += float(p["dy"])
            state.pose_xyz[2] += float(p["dz"])
            state.prev_pose_xyz = None
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

        if state.mode == MotionMode.POSE:
            self._ik.step(state, dt=dt)
        else:
            self._mover.step(state, dt=dt)

        q5_pose_tgt: float | None = None
        if state.mode == MotionMode.POSE:
            q5_pose_tgt = float(
                np.clip(
                    state.q5_fixed_rad + np.deg2rad(IK_CONFIG.pose_q5_extra_deg),
                    float(JOINT_LIMITS_LOWER[4]),
                    float(JOINT_LIMITS_UPPER[4]),
                )
            )
            state.q_full[4] = q5_pose_tgt
        else:
            state.q_full[4] = float(np.clip(state.q5_fixed_rad, JOINT_LIMITS_LOWER[4], JOINT_LIMITS_UPPER[4]))
            state.q_cmd[4] = state.q_full[4]

        q4_pose_target: float | None = None
        if state.mode == MotionMode.POSE:
            q4_geo_target = float(
                np.clip(
                    Q4_OFFSET_RAD + Q4_Q23_COEFF * (state.q_full[1] + state.q_full[2]),
                    IK_CONFIG.q4_safe_min,
                    IK_CONFIG.q4_safe_max,
                )
            )
            if state.q4_blend_active:
                blend_time = max(float(IK_CONFIG.q4_blend_time_s), 1e-6)
                state.q4_blend_t = min(1.0, state.q4_blend_t + dt / blend_time)
                s = state.q4_blend_t * state.q4_blend_t * (3.0 - 2.0 * state.q4_blend_t)
                q4_pose_target = float((1.0 - s) * state.q4_blend_start_rad + s * q4_geo_target)
                if state.q4_blend_t >= 1.0:
                    state.q4_blend_active = False
            else:
                q4_pose_target = q4_geo_target
            state.q_full[3] = q4_pose_target

        q4_cmd_prev = float(state.q_cmd[3])
        q_err = state.q_full[:ARM_AXES] - state.q_cmd[:ARM_AXES]
        if state.mode == MotionMode.POSE:
            q_err[3] = 0.0
        dq_des = q_err * CONFIG.approach_gain

        dq_n = float(np.linalg.norm(dq_des))
        if dq_n > CONFIG.max_joint_vel_rad_s:
            dq_des *= CONFIG.max_joint_vel_rad_s / dq_n

        state.q_cmd[:ARM_AXES] += dq_des * dt
        if state.mode == MotionMode.POSE:
            if q4_pose_target is not None:
                state.q_cmd[3] = q4_pose_target
            if q5_pose_tgt is not None:
                state.q_cmd[4] = q5_pose_tgt

        omega_arm = np.zeros(ARM_AXES, dtype=float)
        omega_arm[:ARM_AXES] = dq_des
        if state.mode == MotionMode.POSE:
            omega_arm[3] = (float(state.q_cmd[3]) - q4_cmd_prev) / max(dt, 1e-6)

        p_rel_deg = np.rad2deg(state.q_cmd[:ARM_AXES]) - state.q_calib_deg[:ARM_AXES]
        j5_rel = float(np.rad2deg(state.q_cmd[4]) - state.q_calib_deg[4])
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
        pose_tol_m: float = CONFIG.reached_pose_tol_m,
    ) -> Tuple[bool, float]:
        """到位判定：``(reached, error)``；error 在关节模式为度范数，在 pose 模式为米。"""
        if state.mode == MotionMode.JOINTS:
            if fb_arm_rad is not None:
                q_actual = np.asarray(fb_arm_rad, dtype=float).ravel()[:ARM_AXES]
            else:
                q_actual = state.q_cmd[:ARM_AXES].copy()
            q_target = state.q_calib_rad[:ARM_AXES] + np.deg2rad(state.joint_rel_deg_4)
            err_rad = q_actual - q_target
            err_deg = np.rad2deg(np.abs(err_rad))
            error = float(np.linalg.norm(err_deg))
            reached = bool(np.all(err_deg < joints_tol_deg))
            return reached, error
        if fb_arm_rad is not None:
            q_fb = np.asarray(fb_arm_rad, dtype=float).ravel()
            q_full = np.zeros(NUM_JOINTS, dtype=float)
            n = min(len(q_fb), NUM_JOINTS)
            q_full[:n] = q_fb[:n]
            q_full[4] = float(state.q_cmd[4])
        else:
            q_full = state.q_cmd.copy()
        p_actual = self._kin.forward_kinematics_position_link4(q_full)
        error = float(np.linalg.norm(p_actual - state.pose_xyz))
        reached = error < pose_tol_m
        return reached, error
