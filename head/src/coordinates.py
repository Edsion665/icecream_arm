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


def strip_role_z_offset(pos: Position, role: Role, settings: Settings) -> Position:
    """去掉 ingest 时加的 role z 抬升，得到相机估计的真实目标位置。"""
    dz = role_z_offset_m(settings, role)
    if abs(dz) < 1e-12:
        return pos
    return Position(pos.x, pos.y, pos.z - dz)


def work_pose_from_camera(pos: Position, role: Role, settings: Settings) -> Position:
    """相机真实坐标 → bridge 工作高度（真实 z + role_z_offset_m）。

    用于 step3 抑制 z 抬升期间的 ingest：slot 内为 z_req，须显式加偏置后再入队。
    """
    return apply_role_z_offset(pos, role, settings)


def work_pose_for_role(pos: Position, role: Role, settings: Settings) -> Position:
    """抓取/放置到位：真实 z + role_z_offset_m。

    约定队列 / 正常 ingest 的 slot 已含 role z 偏置；先 strip 再 apply 得到同一工作高度。
    """
    return apply_role_z_offset(strip_role_z_offset(pos, role, settings), role, settings)


def object_pick_hover_and_work(
    pos: Position, settings: Settings
) -> tuple[Position, Position]:
    """object step5：预接近 z、夹取 z（固定高度，xy 来自感知）。"""
    x, y = float(pos.x), float(pos.y)
    return (
        Position(x, y, float(settings.object_pick_hover_z_m)),
        Position(x, y, float(settings.object_pick_work_z_m)),
    )


def target_place_hover_and_work(
    pos: Position, settings: Settings
) -> tuple[Position, Position]:
    """target step7：预接近 z、放置 z（固定高度，xy 来自队列/感知）。"""
    x, y = float(pos.x), float(pos.y)
    return (
        Position(x, y, float(settings.target_place_hover_z_m)),
        Position(x, y, float(settings.target_place_work_z_m)),
    )


def target_refine_hover_pose(pos: Position, settings: Settings) -> Position:
    """target step3：正上方精观测位 z（固定 0.5 m 等，xy 来自粗观测）。"""
    return Position(
        float(pos.x),
        float(pos.y),
        float(settings.target_refine_hover_z_m),
    )


def target_queue_pose_after_refine(cam: Position, settings: Settings) -> Position:
    """step3 写回队列：xy 用精观测，z 用固定放置工作高度。"""
    return Position(
        float(cam.x),
        float(cam.y),
        float(settings.target_place_work_z_m),
    )


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
