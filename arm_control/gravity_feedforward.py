"""v8 URDF + Pinocchio 重力前馈：将四轴弧度差转为关节 deg×100 后计算 τ_g。"""

from __future__ import annotations

import logging
import math
import sys
from pathlib import Path
from typing import Sequence, Tuple

LOGGER = logging.getLogger(__name__)

_ROBOTARM = Path(__file__).resolve().parent.parent / "robotarm"
if str(_ROBOTARM) not in sys.path:
    sys.path.insert(0, str(_ROBOTARM))


def rad_to_deg_x100(rad: float) -> float:
    """弧度 → 与 gravity_torque / 固件一致的「度×100」。"""
    return rad * 180.0 / math.pi * 100.0


def motor_rad_deltas_to_joint_deg_x100(
    d0: float,
    d1: float,
    d2: float,
    d3: float,
) -> list[float]:
    """四电机弧度差 → 五关节 URDF 输入（度×100），与 run_gravity_on_captures 轴向一致。

    motor0→joint1，motor1→joint2（轴反向故取负），motor2→joint3，motor3→joint4，joint5 固定 0。
    """
    m0 = rad_to_deg_x100(d0)
    m1 = rad_to_deg_x100(d1)
    m2 = rad_to_deg_x100(d2)
    m3 = rad_to_deg_x100(d3)
    return [m0, -m1, m2, m3, 0.0]


def pinocchio_tau_to_four_motor_nm(tau: Sequence[float]) -> Tuple[float, float, float, float]:
    """Pinocchio 广义重力 τ（nv 维）→ 四路电机前馈力矩 (Nm)，与 run_gravity_on_captures 符号一致。"""
    # joint1..4 对应 tau[0]..tau[3]；joint2 与电机轴反向故电机1 前馈取 -tau[1]
    return (
        float(tau[0]),
        -float(tau[1]),
        float(tau[2]),
        float(tau[3]),
    )


def compute_tau_joint_nm(
    delta_rad: Sequence[float],
    finger_l_m: float = 0.0,
    finger_r_m: float = 0.0,
) -> Tuple[float, float, float, float]:
    """四轴相对标定零位弧度差 → URDF joint1..4 重力矩 (Nm)，与广义重力向量前 4 维一致。"""
    if len(delta_rad) != 4:
        raise ValueError("delta_rad 须为 4 个弧度值")
    d0, d1, d2, d3 = (float(x) for x in delta_rad)
    joint_deg_x100 = motor_rad_deltas_to_joint_deg_x100(d0, d1, d2, d3)

    from gravity_comp.gravity_torque import get_gravity_torques_arm_deg_x100

    tau = get_gravity_torques_arm_deg_x100(joint_deg_x100, finger_l_m, finger_r_m)
    n = len(tau)
    if n < 4:
        raise ValueError(f"广义重力维数不足 4：got {n}")
    return tuple(float(tau[i]) for i in range(4))


def compute_tau_ff_nm(
    delta_rad: Sequence[float],
    finger_l_m: float = 0.0,
    finger_r_m: float = 0.0,
) -> Tuple[float, float, float, float]:
    """输入四轴相对标定零位的弧度差，返回四路重力前馈力矩 (Nm)。"""
    tau_joint = compute_tau_joint_nm(delta_rad, finger_l_m, finger_r_m)
    return pinocchio_tau_to_four_motor_nm(tau_joint)
