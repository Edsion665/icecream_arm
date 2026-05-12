"""Centralized joint-axis/sign mapping helpers."""

from __future__ import annotations

import math
from typing import Sequence

from ..config import MOTOR_AXIS_SIGN


def udp_rel_to_motor_pv(
    calibration_rad: Sequence[float],
    p_rel_deg: Sequence[float],
    omega_rad_s: Sequence[float],
) -> tuple[list[float], list[float]]:
    p_cmd: list[float] = []
    v_cmd: list[float] = []
    for i, sign in enumerate(MOTOR_AXIS_SIGN):
        p_cmd.append(float(calibration_rad[i]) + sign * math.radians(float(p_rel_deg[i])))
        v_cmd.append(sign * float(omega_rad_s[i]))
    return p_cmd, v_cmd


def motor_delta_rad_to_joint_deg_x100(delta_rad: Sequence[float]) -> list[float]:
    if len(delta_rad) != 4:
        raise ValueError("delta_rad must contain 4 values")
    out: list[float] = []
    for i, sign in enumerate(MOTOR_AXIS_SIGN):
        deg_x100 = float(delta_rad[i]) * 180.0 / math.pi * 100.0
        out.append(sign * deg_x100)
    out.append(0.0)
    return out


def joint_tau_to_motor_tau(joint_tau: Sequence[float], gain: float) -> tuple[float, float, float, float]:
    if len(joint_tau) < 4:
        raise ValueError("joint_tau must contain at least 4 values")
    return (
        float(joint_tau[0]) * MOTOR_AXIS_SIGN[0] * gain,
        float(joint_tau[1]) * MOTOR_AXIS_SIGN[1] * gain,
        float(joint_tau[2]) * MOTOR_AXIS_SIGN[2] * gain,
        float(joint_tau[3]) * MOTOR_AXIS_SIGN[3] * gain,
    )
