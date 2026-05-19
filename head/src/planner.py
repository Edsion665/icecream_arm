from __future__ import annotations

from typing import Deque, List, Optional

from src.config import Settings
from src.models import PlanResult, Position, SceneSnapshot, TrackSlot
from src.shape_match import (
    filter_slots_by_prefix,
    select_nearest_slot,
    shape_prefix,
    slot_distance,
)


def _plan_reference(s: Settings) -> Position:
    """最近 target/object 的距离参考点（robot_base 原点）。"""
    o = s.plan_reference_xyz
    return Position(float(o[0]), float(o[1]), float(o[2]))


def plan_pick(
    settings: Settings,
    snap: SceneSnapshot,
    target_queue: Deque[Position],
) -> PlanResult:
    """选最近 target，再在同形状前缀的 object 里选抓取目标。"""

    ref = _plan_reference(settings)
    targets: List[TrackSlot] = list(snap.slots_by_role.get("target") or [])
    if not targets and not target_queue:
        return PlanResult(ok=False, reason="no_target")

    if targets:
        nearest_tgt = select_nearest_slot(targets, ref)
    else:
        nearest_tgt = None

    if nearest_tgt is None and target_queue:
        p = target_queue[0]
        nearest_tgt = TrackSlot(
            role="target",
            class_id="queue",
            label="queued_target",
            position=p,
            wrist_yaw_deg=0.0,
            track_id=None,
            confidence=None,
            last_frame_id=None,
            last_ts=None,
            frame="robot_base",
        )

    if nearest_tgt is None:
        return PlanResult(ok=False, reason="no_target")

    prefix = shape_prefix(nearest_tgt.class_id)
    if not prefix:
        return PlanResult(ok=False, reason="bad_target_class_id")

    objs_all: List[TrackSlot] = list(snap.slots_by_role.get("object") or [])
    objs = filter_slots_by_prefix(objs_all, prefix)
    if not objs:
        return PlanResult(
            ok=False,
            reason=f"no_object_for_prefix:{prefix}",
        )

    def score(s: TrackSlot) -> tuple[float, float]:
        conf = s.confidence if s.confidence is not None else 0.0
        return (conf, -slot_distance(s, ref))

    objs.sort(key=score, reverse=True)
    best_obj = objs[0]
    if settings.min_object_confidence > 0 and (best_obj.confidence or 0.0) < settings.min_object_confidence:
        return PlanResult(ok=False, reason="low_confidence")

    return PlanResult(ok=True, reason="", object_slot=best_obj, target_slot=nearest_tgt)


def peek_target_pose(
    snap: SceneSnapshot,
    target_queue: Deque[Position],
    *,
    shape_prefix_lock: str | None = None,
) -> Position | None:
    t = peek_target_track(snap, target_queue, shape_prefix_lock=shape_prefix_lock)
    return None if t is None else t.position


def peek_target_track(
    snap: SceneSnapshot,
    target_queue: Deque[Position],
    *,
    shape_prefix_lock: str | None = None,
) -> TrackSlot | None:
    """放置阶段：优先同前缀槽位里最近的 target；无匹配再回退 FIFO 队列。"""

    ref = Position(0.0, 0.0, 0.0)
    targets: List[TrackSlot] = list(snap.slots_by_role.get("target") or [])

    if shape_prefix_lock:
        matched = filter_slots_by_prefix(targets, shape_prefix_lock)
        if matched:
            return select_nearest_slot(matched, ref)

    if target_queue:
        p = target_queue[0]
        return TrackSlot(
            role="target",
            class_id="queue",
            label="queued_target",
            position=p,
            wrist_yaw_deg=0.0,
            track_id=None,
            confidence=None,
            last_frame_id=None,
            last_ts=None,
            frame="robot_base",
        )

    if not targets:
        return None
    return select_nearest_slot(targets, ref)
