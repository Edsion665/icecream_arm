from __future__ import annotations

from typing import Deque, List

from src.config import Settings
from src.models import PlanResult, Position, SceneSnapshot, TrackSlot


def plan_pick(
    settings: Settings,
    snap: SceneSnapshot,
    target_queue: Deque[Position],
) -> PlanResult:
    """Pick best object + target slots (or queue-backed target). Both required for v3 PLAN."""

    objs: List[TrackSlot] = list(snap.slots_by_role.get("object") or [])
    if not objs:
        return PlanResult(ok=False, reason="no_object")

    def score(s: TrackSlot) -> float:
        return s.confidence if s.confidence is not None else 0.0

    objs.sort(key=score, reverse=True)
    best_obj = objs[0]
    if settings.min_object_confidence > 0 and (best_obj.confidence or 0.0) < settings.min_object_confidence:
        return PlanResult(ok=False, reason="low_confidence")

    tgt = peek_target_track(snap, target_queue)
    if tgt is None:
        return PlanResult(ok=False, reason="no_target")

    return PlanResult(ok=True, reason="", object_slot=best_obj, target_slot=tgt)


def peek_target_pose(snap: SceneSnapshot, target_queue: Deque[Position]) -> Position | None:
    t = peek_target_track(snap, target_queue)
    return None if t is None else t.position


def peek_target_track(snap: SceneSnapshot, target_queue: Deque[Position]) -> TrackSlot | None:
    """Prefer FIFO queue (position-only); else best `role==target` slot from tracker."""

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
    targets: List[TrackSlot] = list(snap.slots_by_role.get("target") or [])
    if not targets:
        return None
    targets.sort(key=lambda s: s.confidence or 0.0, reverse=True)
    return targets[0]
