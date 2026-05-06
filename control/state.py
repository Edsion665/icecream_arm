"""控制状态与单帧关节输出。"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

from ..config import DEFAULT_INITIAL_JOINT_REL_DEG_4
from ..kinematics.urdf_kinematics import NUM_JOINTS

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
    prev_pose_xyz: Optional[np.ndarray] = None
    joint_rel_deg_4: np.ndarray = field(default_factory=lambda: np.zeros(ARM_AXES, dtype=float))
    servo_deg: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    wrist_rel_deg: float = 0.0
    grip_state: float = 0.0
    q_full: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=float))
    q_cmd: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=float))
    initialized: bool = False
    sing_hold: bool = False
    q4_blend_active: bool = False
    q4_blend_start_rad: float = 0.0
    q4_blend_t: float = 1.0

    def reset_command(self) -> None:
        """标定零点不变；前四轴相对标定角由 ``DEFAULT_INITIAL_JOINT_REL_DEG_4`` 初始化。"""
        self.joint_rel_deg_4 = np.asarray(DEFAULT_INITIAL_JOINT_REL_DEG_4, dtype=float).reshape(ARM_AXES).copy()
        self.q_full[:ARM_AXES] = self.q_calib_rad[:ARM_AXES] + np.deg2rad(self.joint_rel_deg_4)
        self.q_full[4] = float(self.q5_fixed_rad)
        self.q_cmd[:NUM_JOINTS] = self.q_full[:NUM_JOINTS].copy()
        self.mode = MotionMode.JOINTS
        self.initialized = True
        self.prev_pose_xyz = None
        self.q4_blend_active = False
        self.q4_blend_start_rad = float(self.q_cmd[3])
        self.q4_blend_t = 1.0

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
            self.q5_fixed_rad = float(q[4])
        self.mode = MotionMode.JOINTS
        self.initialized = True
        self.prev_pose_xyz = None
        self.q4_blend_active = False
        self.q4_blend_start_rad = float(self.q_cmd[3])
        self.q4_blend_t = 1.0
