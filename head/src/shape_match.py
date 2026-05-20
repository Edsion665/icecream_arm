"""class_id 形状前缀：Triangle / Circle / Trapezium 等与槽位配对。"""

from __future__ import annotations

from typing import Any, List, Optional

from src.models import Position, TrackSlot


def shape_prefix(class_id: Any) -> str:
    """``Triangle_Pedestal_Red`` → ``Triangle``；无下划线时整段为前缀。"""
    s = str(class_id).strip()
    if not s:
        return ""
    return s.split("_", 1)[0]


def same_shape_prefix(a: Any, b: Any) -> bool:
    pa, pb = shape_prefix(a), shape_prefix(b)
    return bool(pa) and pa == pb


def slot_distance(slot: TrackSlot, ref: Position) -> float:
    return slot.position.dist(ref)


def select_nearest_slot(slots: List[TrackSlot], ref: Position) -> Optional[TrackSlot]:
    if not slots:
        return None
    return min(slots, key=lambda s: slot_distance(s, ref))


def select_slot_nearest_to(slots: List[TrackSlot], ref: Position) -> Optional[TrackSlot]:
    """在 slots 中选 position 距 ref 最近者（用于同前缀放置位消歧）。"""
    if not slots:
        return None
    return min(slots, key=lambda s: s.position.dist(ref))


def filter_slots_by_prefix(slots: List[TrackSlot], prefix: str) -> List[TrackSlot]:
    if not prefix:
        return list(slots)
    return [s for s in slots if shape_prefix(s.class_id) == prefix]
