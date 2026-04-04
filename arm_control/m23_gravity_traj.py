"""M2/M3 离散步进与重力前馈 MIT 下行协同（环境变量开关）。

与 ``motor23_drive_discrete.py`` 相同步进规则：各轴 |Δ_i|/N ≤ v_max·dt（dt=1/send_hz），
N = max_i ceil(|Δ_i|/(v_max·dt))，两轴同步到第 N 帧到位；末步 v=0；之后保持终点。

开启时：下行 p/v 中 M2、M3 走轨迹；M1、M4 的 p、v 用当前反馈（保持不动）。
力矩 t 仍由 ``compute_tau_ff_nm`` 按当前上行/FB 弧度算重力项。
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field


def _env_flag(key: str) -> bool:
    return os.environ.get(key, "").strip().lower() in ("1", "true", "yes", "on")


def _env_float(key: str, default: float) -> float:
    try:
        return float(os.environ.get(key, str(default)).strip())
    except ValueError:
        return default


@dataclass
class M23GravityTraj:
    """第二节(1)、第三节(2) 各转 +deg_m2°、+deg_m3°（弧度在 __post_init__ 中计算）。"""

    enabled: bool
    deg_m2: float = 140.0
    deg_m3: float = 130.0
    v_max: float = 0.3
    phase: str = "wait"  # wait -> run -> hold
    p1_0: float | None = None
    p2_0: float | None = None
    n_steps: int = 0
    k: int = 0
    delta_m2_rad: float = field(init=False)
    delta_m3_rad: float = field(init=False)

    def __post_init__(self) -> None:
        self.delta_m2_rad = math.radians(self.deg_m2)
        self.delta_m3_rad = math.radians(self.deg_m3)

    @classmethod
    def from_env(cls) -> M23GravityTraj:
        return cls(
            enabled=_env_flag("ARM_CONTROL_M23_GRAVITY_TRAJ"),
            deg_m2=_env_float("ARM_CONTROL_M23_DEG_M2", 140.0),
            deg_m3=_env_float("ARM_CONTROL_M23_DEG_M3", 130.0),
            v_max=_env_float("ARM_CONTROL_M23_V_MAX", 0.3),
        )

    def tick(self, dt: float, arm_rad: tuple[float, float, float, float]) -> bool:
        """
        推进一步。返回 True 表示本周期应用 M2/M3 轨迹覆盖（run 或 hold）；
        False 表示未启用。
        """
        if not self.enabled:
            return False
        if self.phase == "wait":
            self.p1_0 = float(arm_rad[1])
            self.p2_0 = float(arm_rad[2])
            max_step = self.v_max * dt
            n2 = max(1, int(math.ceil(abs(self.delta_m2_rad) / max_step)))
            n3 = max(1, int(math.ceil(abs(self.delta_m3_rad) / max_step)))
            self.n_steps = max(n2, n3)
            self.k = 0
            self.phase = "run"
        if self.phase == "run":
            assert self.p1_0 is not None and self.p2_0 is not None
            self.k += 1
            if self.k >= self.n_steps:
                self.phase = "hold"
            return True
        if self.phase == "hold":
            return True
        return False

    def p_v_for_motor_1_2(self, dt: float) -> tuple[float, float, float, float]:
        """索引 1、2 的 p 与 v（仅当 tick 已返回 True 时调用）。"""
        assert self.p1_0 is not None and self.p2_0 is not None
        if self.phase == "hold":
            return (
                self.p1_0 + self.delta_m2_rad,
                self.p2_0 + self.delta_m3_rad,
                0.0,
                0.0,
            )
        # run
        frac = min(1.0, self.k / self.n_steps)
        p1 = self.p1_0 + self.delta_m2_rad * frac
        p2 = self.p2_0 + self.delta_m3_rad * frac
        step_m2 = self.delta_m2_rad / self.n_steps
        step_m3 = self.delta_m3_rad / self.n_steps
        if self.k >= self.n_steps:
            return p1, p2, 0.0, 0.0
        return p1, p2, step_m2 / dt, step_m3 / dt
