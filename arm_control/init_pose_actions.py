"""在线姿态动作：归位与工作位（在主 tau_ff 回路内执行，不占额外通道）。"""

from __future__ import annotations

import logging
import math
import os
from typing import Tuple

from .config import (
    TAU_FF,
    set_mit_motor_cmd_params,
)

logger = logging.getLogger(__name__)

HOME_REL_DEG: Tuple[float, float, float, float] = (3.45333, -87.340324, -177.172368, 30.424878)
WORK_REL_DEG: Tuple[float, float, float, float] = (-1.704907, -44.216584, -130.8138, -1.529966)


def boot_pose_mode_from_env() -> str:
    """启动姿态：none|home|work（home=归位；work=工作位并保持）。"""
    v = os.environ.get("ARM_CONTROL_BOOT_POSE", "none").strip().lower()
    return v if v in ("none", "home", "work") else "none"


def _v_max() -> float:
    try:
        return max(1e-6, float(os.environ.get("ARM_CONTROL_BOOT_POSE_VMAX", "0.4")))
    except ValueError:
        return 0.4


def _tol_rad() -> float:
    try:
        return max(1e-4, float(os.environ.get("ARM_CONTROL_BOOT_POSE_TOL_RAD", "0.008")))
    except ValueError:
        return 0.008


class InitPoseController:
    """在 tau_ff 主循环内逐步推进目标姿态，输出 p/v 覆盖值。"""

    def __init__(self) -> None:
        self._target_rel_deg: Tuple[float, float, float, float] | None = None
        self._target_p: list[float] | None = None
        self._cmd_p: list[float] | None = None
        self._tag: str = ""
        self._v_max = _v_max()
        self._tol = _tol_rad()

    def request_home(self) -> None:
        self._target_rel_deg = HOME_REL_DEG
        self._target_p = None
        self._cmd_p = None
        self._tag = "归位"
        logger.info("已请求归位动作")

    def request_work(self) -> None:
        self._target_rel_deg = WORK_REL_DEG
        self._target_p = None
        self._cmd_p = None
        self._tag = "工作位"
        logger.info("已请求工作位动作")

    def has_active_request(self) -> bool:
        return self._target_rel_deg is not None

    def step(
        self, arm_rad: Tuple[float, float, float, float], interval: float
    ) -> tuple[list[float], list[float]] | None:
        if self._target_rel_deg is None:
            return None
        cal = TAU_FF.calibration_rad
        if len(cal) != 4:
            logger.error("姿态动作：calibration_rad 长度须为 4，忽略本次请求")
            self._target_rel_deg = None
            return None
        if self._target_p is None:
            self._target_p = [
                float(cal[i]) + math.radians(float(self._target_rel_deg[i])) for i in range(4)
            ]
            self._cmd_p = [float(arm_rad[i]) for i in range(4)]
            logger.info(
                "%s开始：目标相对标定角(°)=%s  v_max=%.3g rad/s  tol=%.4g rad",
                self._tag,
                self._target_rel_deg,
                self._v_max,
                self._tol,
            )

        assert self._target_p is not None
        assert self._cmd_p is not None
        out_p: list[float] = []
        out_v: list[float] = []
        step_max = self._v_max * max(1e-6, interval)
        for i in range(4):
            err = self._target_p[i] - self._cmd_p[i]
            step = max(-step_max, min(step_max, err))
            self._cmd_p[i] += step
            out_p.append(self._cmd_p[i])
            out_v.append(step / max(1e-6, interval))

        max_err = max(abs(self._target_p[i] - self._cmd_p[i]) for i in range(4))
        if max_err <= self._tol:
            hold = [
                {"p": self._target_p[0], "v": 0.0},
                {"p": self._target_p[1], "v": 0.0},
                {"p": self._target_p[2], "v": 0.0},
                {"p": self._target_p[3], "v": 0.0},
            ]
            set_mit_motor_cmd_params(hold)
            out_p = [float(v["p"]) for v in hold]
            out_v = [0.0, 0.0, 0.0, 0.0]
            logger.info("%s到位：已切换为保持位姿（p=目标，v=0）", self._tag)
            self._target_rel_deg = None
            self._target_p = None
            self._cmd_p = None
            self._tag = ""
        return out_p, out_v

