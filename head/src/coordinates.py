"""感知坐标 → bridge 笛卡尔目标：按 role 对 z 做固定抬升（米）。"""

from __future__ import annotations

import math

from src.config import Settings
from src.models import Position, Role


def role_z_offset_m(settings: Settings, role: Role) -> float:
    if role == "target":
        return float(settings.target_z_offset_m)
    if role == "object":
        return float(settings.object_z_offset_m)
    return 0.0


def apply_role_z_offset(pos: Position, role: Role, settings: Settings) -> Position:
    dz = role_z_offset_m(settings, role)
    if abs(dz) < 1e-12:
        return pos
    return Position(pos.x, pos.y, pos.z + dz)


def j1_deg_from_xy(x: float, y: float) -> float:
    """与 bridge IK 一致：joint1 相对角 q1 = -atan2(y, x)（度）。"""
    if math.hypot(x, y) <= 1e-6:
        return 0.0
    return -math.degrees(math.atan2(float(y), float(x)))


def wrist_j1_bias_deg(x: float, y: float, j1_obs_deg: float) -> float:
    """目标点相对观测位的 J1 差：J1(x,y) − J1_obs（度）。"""
    return j1_deg_from_xy(x, y) - float(j1_obs_deg)


def wrist_cmd_deg(camera_wrist_yaw_deg: float, pos: Position, j1_obs_deg: float) -> float:
    """下发 bridge claw：wrist_cam − j1_bias。"""
    bias = wrist_j1_bias_deg(pos.x, pos.y, j1_obs_deg)
    return float(camera_wrist_yaw_deg) - bias


def wrist_deg_for_object(
    camera_wrist_yaw_deg: float, pos: Position, settings: Settings
) -> float:
    """抓取：wrist_cam − (J1(物体 xy) − obs2 J1)。"""
    j1_obs = float(settings.observe2_axes_rel_deg[0])
    return wrist_cmd_deg(camera_wrist_yaw_deg, pos, j1_obs)


def wrist_deg_for_target(
    camera_wrist_yaw_deg: float, pos: Position, settings: Settings
) -> float:
    """放置：wrist_cam − (J1(目标 xy) − obs1 J1)。"""
    j1_obs = float(settings.observe1_axes_rel_deg[0])
    return wrist_cmd_deg(camera_wrist_yaw_deg, pos, j1_obs)
