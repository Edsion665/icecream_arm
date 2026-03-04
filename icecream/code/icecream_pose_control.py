#!/usr/bin/env python3
"""
位姿控制：给定目标空间位姿（基座系），按固定频率计算使末端到达并保持该位姿。
- 位置控制：每步或按频率求解 IK，将目标关节角下发。
- 速度控制：用雅可比伪逆将位姿误差转为关节速度，积分得到目标关节角再下发。
提供调试接口：set_target_pose、get_current_pose、get_pose_error、get_target_pose。
"""
from __future__ import annotations

from typing import Literal, Optional, Tuple

import numpy as np

from icecream_kinematics import (
    URDFKinematics,
    get_kinematics,
    _matrix_to_pose,
    _pose_to_matrix,
    _quat_to_matrix,
    _matrix_to_quat,
    NUM_JOINTS,
)

# 关节限位（与 driver 一致）
JOINT_LIMITS_LOWER = np.array([-3.14] * NUM_JOINTS)
JOINT_LIMITS_UPPER = np.array([3.14] * NUM_JOINTS)


class PoseController:
    """
    末端位姿控制器（基座系下）。
    支持 position 控制（IK）与 velocity 控制（雅可比伪逆 + 积分）。
    所有位姿均为基座系 base（link0）。
    """

    def __init__(
        self,
        mode: Literal["position", "velocity"] = "position",
        control_frequency_hz: float = 60.0,
        position_gain: float = 2.0,
        orientation_gain: float = 1.0,
        velocity_gain: float = 1.0,
        ik_position_only: bool = False,
        urdf_path: Optional[str] = None,
    ):
        """
        Args:
            mode: "position" 使用 IK 得到目标关节角；"velocity" 使用雅可比伪逆得到关节速度再积分。
            control_frequency_hz: 控制律更新频率（Hz），用于 velocity 模式的积分步长或 IK 调用间隔。
            position_gain: 位置误差增益（velocity 模式下的线性速度增益）。
            orientation_gain: 姿态误差增益（velocity 模式下的角速度增益）。
            velocity_gain: velocity 模式下整体缩放。
            ik_position_only: 为 True 时 IK 只满足位置，不约束姿态。
            urdf_path: URDF 路径，None 则用默认工程路径。
        """
        self._mode = mode
        self._control_dt = 1.0 / control_frequency_hz
        self._position_gain = position_gain
        self._orientation_gain = orientation_gain
        self._velocity_gain = velocity_gain
        self._ik_position_only = ik_position_only
        self._kin = get_kinematics(urdf_path)

        # 目标位姿（基座系）：position (3,), orientation (3,3)
        self._target_position = np.zeros(3, dtype=float)
        self._target_orientation = np.eye(3, dtype=float)

        self._target_set = False
        self._position_only_target = False  # 仅位置目标时不约束姿态

    def set_target_pose(
        self,
        position: np.ndarray,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        设置目标位姿（基座系）。
        position: (3,) 米。
        orientation: 3x3 旋转矩阵，或 (4,) 四元数 xyzw；None 表示只控位置（不约束姿态）。
        """
        self._target_position = np.asarray(position, dtype=float).ravel()[:3]
        if orientation is not None:
            o = np.asarray(orientation, dtype=float)
            if o.size == 4:
                self._target_orientation = _quat_to_matrix(o)
            else:
                self._target_orientation = o.reshape(3, 3).copy()
        else:
            self._target_orientation = self._target_orientation  # 保持上次
        self._target_set = True
        self._position_only_target = False

    def set_target_position(self, position: np.ndarray) -> None:
        """仅设置目标位置（基座系），姿态不约束。"""
        self._target_position = np.asarray(position, dtype=float).ravel()[:3]
        self._target_set = True
        self._position_only_target = True

    def get_target_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """当前目标位姿：position (3,), rotation (3,3)。"""
        return self._target_position.copy(), self._target_orientation.copy()

    def get_current_pose(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """由当前关节角 q 通过 FK 得到末端位姿（基座系）：position (3,), rotation (3,3)。"""
        T = self._kin.forward_kinematics(q)
        return _matrix_to_pose(T)

    def get_pose_error(
        self,
        q: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, float, float]:
        """
        当前位姿与目标的误差（基座系）。
        返回: (position_error (3,), orientation_error_axis_angle (3,), pos_error_norm, ori_error_rad).
        """
        p_cur, R_cur = self.get_current_pose(q)
        err_pos = self._target_position - p_cur
        if self._position_only_target:
            return err_pos, np.zeros(3), float(np.linalg.norm(err_pos)), 0.0
        R_des = self._target_orientation
        R_err = R_des @ R_cur.T
        angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
        if angle > 1e-8:
            axis = np.array([
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1],
            ], dtype=float)
            err_ori = (axis / (2 * np.sin(angle))) * angle
        else:
            err_ori = np.zeros(3)
        return err_pos, err_ori, float(np.linalg.norm(err_pos)), float(angle)

    def update(
        self,
        dt: float,
        current_joint_positions: np.ndarray,
    ) -> np.ndarray:
        """
        根据当前关节角和目标位姿，计算下一步的目标关节角。
        dt: 本步仿真时间（秒）。
        current_joint_positions: (5,) 当前关节角弧度。
        返回: (5,) 目标关节角弧度，可直接交给 driver.set_joint_positions(...) 并 apply()。
        """
        q = np.asarray(current_joint_positions, dtype=float).ravel()[:NUM_JOINTS]
        if not self._target_set:
            return q.copy()

        if self._mode == "position":
            use_orientation = not (self._ik_position_only or self._position_only_target)
            q_target, _ = self._kin.inverse_kinematics(
                self._target_position,
                target_orientation=None if not use_orientation else self._target_orientation,
                q_init=q,
                position_only=self._ik_position_only or self._position_only_target,
                joint_limits_lower=JOINT_LIMITS_LOWER,
                joint_limits_upper=JOINT_LIMITS_UPPER,
            )
            return q_target

        else:  # velocity
            err_pos, err_ori, _, _ = self.get_pose_error(q)
            v_linear = self._velocity_gain * self._position_gain * err_pos
            v_angular = self._velocity_gain * self._orientation_gain * err_ori if not self._position_only_target else np.zeros(3)
            v_ee = np.concatenate([v_linear, v_angular])  # (6,)
            J = self._kin.jacobian(q)
            damping = 1e-2
            Jt = J.T
            JJt = J @ Jt + (damping ** 2) * np.eye(6)
            dq = Jt @ np.linalg.solve(JJt, v_ee)
            step = min(dt, self._control_dt * 2)
            q_target = q + dq * step
            return np.clip(q_target, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)


# ---------------------------------------------------------------------------
# 调试接口：命令行可调用的辅助函数
# ---------------------------------------------------------------------------

def pose_from_position_rpy(position: np.ndarray, roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[np.ndarray, np.ndarray]:
    """由位置 (3,) 和 RPY 度构造目标位姿：position (3,), rotation (3,3)。"""
    from icecream_kinematics import _rpy_to_matrix
    rpy = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    R = _rpy_to_matrix(rpy)
    return np.asarray(position, dtype=float).ravel()[:3], R


def pose_from_position_rpy_rad(position: np.ndarray, roll: float, pitch: float, yaw: float) -> Tuple[np.ndarray, np.ndarray]:
    """由位置 (3,) 和 RPY 弧度构造目标位姿。"""
    from icecream_kinematics import _rpy_to_matrix
    R = _rpy_to_matrix(np.array([roll, pitch, yaw]))
    return np.asarray(position, dtype=float).ravel()[:3], R
