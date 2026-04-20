"""仿真显示层：receiver + show。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .calculator import JointFrame


@dataclass
class receiver:
    _latest: Optional[JointFrame] = None

    def accept(self, frame: JointFrame) -> None:
        self._latest = frame

    def latest(self) -> Optional[JointFrame]:
        return self._latest


class show:
    def __init__(self, arm, arm_dof: int, q_calib_deg=None, controlled_dof: int = 5):
        self._arm = arm
        self._arm_dof = arm_dof
        self._controlled_dof = max(1, min(controlled_dof, arm_dof))
        self._controller = arm.get_articulation_controller()
        self._q_cmd = np.zeros(arm_dof, dtype=float)
        if q_calib_deg is None:
            self._q_calib_deg = np.zeros(arm_dof, dtype=float)
        else:
            arr = np.array(q_calib_deg, dtype=float).ravel()
            self._q_calib_deg = np.zeros(arm_dof, dtype=float)
            self._q_calib_deg[: min(len(arr), arm_dof)] = arr[: min(len(arr), arm_dof)]

    def initialize(self, q_init: np.ndarray) -> None:
        self._q_cmd[:] = q_init[: self._arm_dof]
        try:
            self._arm.set_joint_positions(q_init)
        except Exception:
            pass

    def apply(self, frame: JointFrame, q5_fixed_rad: float = 0.0) -> np.ndarray:
        if frame is None:
            return self._q_cmd.copy()
        # 仅覆盖受控关节，其余关节保持仿真当前值，避免被强行写零导致发散。
        q_actual = self._arm.get_joint_positions()
        q = np.array(q_actual, dtype=float) if q_actual is not None else np.zeros(self._arm_dof, dtype=float)
        omega = np.zeros(self._arm_dof, dtype=float)
        n = min(4, self._controlled_dof)
        q[:n] = np.deg2rad(frame.arm_rel_deg[:n] + self._q_calib_deg[:n])
        omega[:n] = frame.arm_omega_rad_s[:n]
        if self._controlled_dof >= 5:
            q[4] = q5_fixed_rad
        self._q_cmd = q.copy()
        try:
            from isaacsim.core.utils.types import ArticulationAction
        except ModuleNotFoundError:
            from omni.isaac.core.utils.types import ArticulationAction
        self._controller.apply_action(
            ArticulationAction(joint_positions=q.tolist(), joint_velocities=omega.tolist())
        )
        return q

