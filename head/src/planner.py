from __future__ import annotations

from typing import Deque, List, Optional

from src.config import Settings
from src.models import PlanResult, Position, QueuedTarget, SceneSnapshot, TrackSlot
from src.shape_match import (
    filter_slots_by_prefix,
    select_nearest_slot,
    select_slot_nearest_to,
    shape_prefix,
    slot_distance,
)


def _plan_reference(s: Settings) -> Position:
    """队列排序与 object 评分的距离参考点（robot_base, m）。"""
    o = s.plan_reference_xyz
    return Position(float(o[0]), float(o[1]), float(o[2]))


def build_target_queue(targets: List[TrackSlot], ref: Position) -> List[QueuedTarget]:
    """obs1 建队：距 ref 越近优先级越高（队首最先服务）。队列仅表示优先级。"""
    sorted_slots = sorted(targets, key=lambda s: slot_distance(s, ref))
    out: List[QueuedTarget] = []
    for s in sorted_slots:
        prefix = shape_prefix(s.class_id)
        if not prefix:
            continue
        out.append(
            QueuedTarget(
                class_id=s.class_id,
                shape_prefix=prefix,
                position=s.position,
                wrist_yaw_deg=s.wrist_yaw_deg,
                label=s.label,
            )
        )
    return out


def queued_to_track_slot(q: QueuedTarget) -> TrackSlot:
    return TrackSlot(
        role="target",
        class_id=q.class_id,
        label=q.label,
        position=q.position,
        wrist_yaw_deg=q.wrist_yaw_deg,
        track_id=None,
        confidence=None,
        last_frame_id=None,
        last_ts=None,
        frame="robot_base",
    )


def _best_object_for_prefix(
    objs: List[TrackSlot], ref: Position, min_confidence: float
) -> TrackSlot | None:
    if not objs:
        return None

    def score(s: TrackSlot) -> tuple[float, float]:
        conf = s.confidence if s.confidence is not None else 0.0
        return (conf, -slot_distance(s, ref))

    objs = sorted(objs, key=score, reverse=True)
    best = objs[0]
    if min_confidence > 0 and (best.confidence or 0.0) < min_confidence:
        return None
    return best


def plan_pick(
    settings: Settings,
    snap: SceneSnapshot,
    target_queue: Deque[QueuedTarget],
) -> PlanResult:
    """按 obs1 队列优先级选本轮形状；传送带 object 按同前缀匹配。

    - 队列只决定优先级（近者优先）。
    - 放置目标位始终来自 obs1 队列项，不要求 obs2 再看到 target。
    - 传送带未见队首形状时，依次尝试队列中下一项；只要在 obs1 队列里
      且传送带上有同前缀 object，即可抓取。
    """

    ref = _plan_reference(settings)
    objs_all: List[TrackSlot] = list(snap.slots_by_role.get("object") or [])

    queue: List[QueuedTarget] = list(target_queue)
    if not queue:
        targets: List[TrackSlot] = list(snap.slots_by_role.get("target") or [])
        if targets:
            queue = build_target_queue(targets, ref)

    if not queue:
        return PlanResult(ok=False, reason="no_target")

    min_conf = settings.min_object_confidence

    for q_item in queue:
        if q_item.placed:
            continue
        prefix = q_item.shape_prefix
        if not prefix:
            continue
        objs = filter_slots_by_prefix(objs_all, prefix)
        best_obj = _best_object_for_prefix(objs, ref, min_conf)
        if best_obj is None:
            continue
        planned_tgt = queued_to_track_slot(q_item)
        return PlanResult(
            ok=True,
            reason="",
            object_slot=best_obj,
            target_slot=planned_tgt,
            queued_target=q_item,
        )

    prefixes = [q.shape_prefix for q in queue]
    return PlanResult(
        ok=False,
        reason=f"no_object_on_belt_for_obs1_queue:{prefixes}",
    )


def peek_target_pose(
    snap: SceneSnapshot,
    target_queue: Deque[QueuedTarget],
    *,
    shape_prefix_lock: str | None = None,
    preferred: TrackSlot | None = None,
) -> Position | None:
    t = peek_target_track(
        snap,
        target_queue,
        shape_prefix_lock=shape_prefix_lock,
        preferred=preferred,
    )
    return None if t is None else t.position


def peek_target_track(
    snap: SceneSnapshot,
    target_queue: Deque[QueuedTarget],
    *,
    shape_prefix_lock: str | None = None,
    preferred: TrackSlot | None = None,
) -> TrackSlot | None:
    """放置阶段：仅选与手中物体同前缀的 target；无匹配返回 None（禁止全局最近回退）。"""

    if not shape_prefix_lock:
        return None

    targets: List[TrackSlot] = list(snap.slots_by_role.get("target") or [])
    matched = filter_slots_by_prefix(targets, shape_prefix_lock)
    if matched:
        if preferred is not None:
            picked = select_slot_nearest_to(matched, preferred.position)
            if picked is not None:
                return picked
        ref = Position(0.0, 0.0, 0.0)
        return select_nearest_slot(matched, ref)

    for q in target_queue:
        if q.shape_prefix == shape_prefix_lock:
            return queued_to_track_slot(q)

    return None
