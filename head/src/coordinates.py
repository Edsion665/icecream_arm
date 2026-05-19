"""感知坐标 → bridge 笛卡尔目标：按 role 对 z 做固定抬升（米）。"""

from __future__ import annotations

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
