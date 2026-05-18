"""仿真显示层：缓存最近一帧并应用到 Isaac 关节控制器。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..calculator import JointFrame
from ..exceptions import ArticulationCommandError


@dataclass
class FrameReceiver:
    """线程内缓存 ``JointFrame``（单生产者单消费者）。"""

    _latest: Optional[JointFrame] = None

    def accept(self, frame: JointFrame) -> None:
        self._latest = frame

    def latest(self) -> Optional[JointFrame]:
        return self._latest


class ArticulationViewer:
    """将 ``JointFrame`` 转为 ``ArticulationAction`` 并下发。"""

    def __init__(
        self,
        arm: object,
        arm_dof: int,
        q_calib_deg: Optional[np.ndarray] = None,
        controlled_dof: int = 5,
    ) -> None:
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
        """将仿真关节同步到 ``q_init``；失败抛出 ``ArticulationCommandError``。"""
        self._q_cmd[:] = q_init[: self._arm_dof]
        try:
            self._arm.set_joint_positions(q_init)
        except (TypeError, ValueError, RuntimeError, OSError) as exc:
            raise ArticulationCommandError(
                f"set_joint_positions 失败: {type(exc).__name__}: {exc}"
            ) from exc

    def apply(self, frame: Optional[JointFrame]) -> np.ndarray:
        """根据 ``frame`` 写关节目标；``frame`` 为 ``None`` 时返回上一帧缓存。"""
        if frame is None:
            return self._q_cmd.copy()
        q_actual = self._arm.get_joint_positions()
        q = np.array(q_actual, dtype=float) if q_actual is not None else np.zeros(self._arm_dof, dtype=float)
        omega = np.zeros(self._arm_dof, dtype=float)
        n = min(4, self._controlled_dof)
        q[:n] = np.deg2rad(frame.arm_rel_deg[:n] + self._q_calib_deg[:n])
        omega[:n] = frame.arm_omega_rad_s[:n]
        if self._controlled_dof >= 5:
            wrist_deg = float(getattr(frame, "wrist_rel_deg", 0.0))
            if abs(wrist_deg) < 1e-9:
                wrist_deg = float(getattr(frame, "joint5_rel_deg", 0.0))
            q[4] = np.deg2rad(wrist_deg + float(self._q_calib_deg[4]))
        self._q_cmd = q.copy()
        try:
            from isaacsim.core.utils.types import ArticulationAction
        except ModuleNotFoundError:
            from omni.isaac.core.utils.types import ArticulationAction
        try:
            self._controller.apply_action(
                ArticulationAction(joint_positions=q.tolist(), joint_velocities=omega.tolist())
            )
        except (TypeError, ValueError, RuntimeError, OSError) as exc:
            raise ArticulationCommandError(
                f"apply_action 失败: {type(exc).__name__}: {exc}"
            ) from exc
        return q


receiver = FrameReceiver
show = ArticulationViewer
