"""控制状态与单帧关节输出。"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

from ..config import DEFAULT_INITIAL_JOINT_REL_DEG_4
from ..kinematics.urdf_kinematics import JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER, NUM_JOINTS

ARM_AXES = 4


class MotionMode(str, Enum):
    """主臂工作模式。"""

    POSE = "pose"
    JOINTS = "joints"


@dataclass
class JointFrame:
    """下发 UDP 与仿真 viewer 的一帧主臂+爪通道状态。"""

    arm_rel_deg: np.ndarray
    arm_omega_rad_s: np.ndarray
    servo_deg: np.ndarray
    wrist_rel_deg: float
    wrist_omega_rad_s: float
    grip_state: float
    stepper_deg_cmd: float
    conveyor_run_cmd: float
    mode: str
    timestamp: float
    joint5_rel_deg: float = 0.0


@dataclass
class CalculatorState:
    """IK / 关节模式共享的可变状态。"""

    mode: MotionMode = MotionMode.JOINTS
    q_calib_deg: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=float))
    q_calib_rad: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=float))
    q5_fixed_rad: float = 0.0
    pose_xyz: np.ndarray = field(default_factory=lambda: np.array([0.35, 0.2, 0.25], dtype=float))
    joint_rel_deg_4: np.ndarray = field(default_factory=lambda: np.zeros(ARM_AXES, dtype=float))
    servo_deg: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    wrist_rel_deg: float = 0.0
    grip_state: float = 1.0  # 0=关闭 1=张开；默认张开
    stepper_deg_cmd: float = 0.0
    conveyor_run_cmd: float = 0.0
    arm_speed_rad_s: np.ndarray = field(default_factory=lambda: np.array([0.8, 0.6, 0.8, 0.8], dtype=float))
    q_full: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=float))
    q_cmd: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=float))
    initialized: bool = False
    # pose 命令经 IK 解算后的目标关节角（rad），供 is_reached 判定使用
    q_pose_target_rad: Optional[np.ndarray] = None
    # pose_seq 模式：待发送的关节角序列（每帧 arm_rel_deg_4，度，相对标定）
    pose_seq_frames: list = field(default_factory=list)

    def wrist_joint_rad(self) -> float:
        """手腕舵机角（弧度，含标定）：与 ``wrist_rel_deg`` / UDP ``p_rel_deg[4]`` 一致。"""
        return float(
            np.clip(
                self.q_calib_rad[4] + np.deg2rad(self.wrist_rel_deg),
                JOINT_LIMITS_LOWER[4],
                JOINT_LIMITS_UPPER[4],
            )
        )

    def set_wrist_rel_deg(self, deg: float, *, sync_joint5_cmd: bool = True) -> None:
        """更新手腕相对角；仿真与 resync 需与 ``q_cmd[4]`` 一致时设 ``sync_joint5_cmd``。"""
        self.wrist_rel_deg = float(deg)
        self.servo_deg = np.array([self.wrist_rel_deg, self.grip_state], dtype=float)
        if sync_joint5_cmd:
            rad = self.wrist_joint_rad()
            self.q_full[4] = rad
            self.q_cmd[4] = rad

    def reset_command(self) -> None:
        """标定零点不变；前四轴相对标定角由 ``DEFAULT_INITIAL_JOINT_REL_DEG_4`` 初始化。"""
        self.joint_rel_deg_4 = np.asarray(DEFAULT_INITIAL_JOINT_REL_DEG_4, dtype=float).reshape(ARM_AXES).copy()
        self.q_full[:ARM_AXES] = self.q_calib_rad[:ARM_AXES] + np.deg2rad(self.joint_rel_deg_4)
        self.q_full[4] = float(self.q5_fixed_rad)
        self.q_cmd[:NUM_JOINTS] = self.q_full[:NUM_JOINTS].copy()
        self.mode = MotionMode.JOINTS
        self.pose_seq_frames = []
        self.initialized = True

    def sync_from_articulation_rad(self, q_joint_rad: np.ndarray, n_dof: int) -> None:
        """用读到的关节弧度覆盖内部状态，使控制与物理一致。"""
        q = np.asarray(q_joint_rad, dtype=float).ravel()
        n = min(int(n_dof), NUM_JOINTS, len(q))
        if n <= 0:
            return
        self.q_full[:n] = q[:n]
        self.q_cmd[:n] = q[:n]
        self.joint_rel_deg_4 = (np.rad2deg(self.q_full[:ARM_AXES]) - self.q_calib_deg[:ARM_AXES]).copy()
        if n >= 5:
            self.wrist_rel_deg = float(np.rad2deg(q[4]) - self.q_calib_deg[4])
            self.q5_fixed_rad = float(q[4])
            self.servo_deg = np.array([self.wrist_rel_deg, self.grip_state], dtype=float)
        self.mode = MotionMode.JOINTS
        self.initialized = True
