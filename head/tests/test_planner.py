"""Planner: target queue priority, plan_pick, peek_target_track."""

from __future__ import annotations

from collections import deque

import pytest

from src.config import Settings
from src.models import Position, QueuedTarget, SceneSnapshot, TrackSlot
from src.planner import (
    build_target_queue,
    peek_target_track,
    plan_pick,
    queued_to_track_slot,
)


def _slot(
    role: str,
    class_id: str,
    x: float,
    y: float = 0.0,
    z: float = 0.1,
    conf: float | None = 1.0,
) -> TrackSlot:
    return TrackSlot(
        role=role,  # type: ignore[arg-type]
        class_id=class_id,
        label=class_id,
        position=Position(x, y, z),
        wrist_yaw_deg=0.0,
        confidence=conf,
    )


def _snap(targets: list[TrackSlot], objects: list[TrackSlot]) -> SceneSnapshot:
    return SceneSnapshot(
        slots_by_role={"target": targets, "object": objects, "lid": []},
    )


def test_build_target_queue_nearest_first() -> None:
    ref = Position(0.0, 0.0, 0.0)
    targets = [
        _slot("target", "Square_Pedestal_A", 0.9, 0.0, 0.0),
        _slot("target", "Trapezium_Pedestal_B", 0.5, 0.0, 0.0),
        _slot("target", "Square_Pedestal_C", 0.1, 0.0, 0.0),
    ]
    q = build_target_queue(targets, ref)
    assert [x.shape_prefix for x in q] == ["Square", "Trapezium", "Square"]
    assert q[0].position.x == pytest.approx(0.1)
    assert q[1].position.x == pytest.approx(0.5)
    assert q[2].position.x == pytest.approx(0.9)


def test_plan_pick_uses_queue_head_prefix_when_on_belt() -> None:
    settings = Settings()
    queue = deque(
        [
            QueuedTarget(
                class_id="Trapezium_Pedestal_B",
                shape_prefix="Trapezium",
                position=Position(0.1, 0.0, 0.0),
                wrist_yaw_deg=0.0,
                label="t",
            )
        ]
    )
    snap = _snap(
        [],
        [
            _slot("object", "Square", 0.2),
            _slot("object", "Trapezium", 0.3),
        ],
    )
    pr = plan_pick(settings, snap, queue)
    assert pr.ok
    assert pr.object_slot is not None
    assert pr.object_slot.class_id == "Trapezium"
    assert pr.target_slot is not None
    assert pr.target_slot.class_id == "Trapezium_Pedestal_B"


def test_plan_pick_skips_head_when_not_on_belt_tries_next() -> None:
    settings = Settings()
    queue = deque(
        [
            QueuedTarget(
                class_id="Trapezium_Pedestal_B",
                shape_prefix="Trapezium",
                position=Position(0.1, 0.0, 0.0),
                wrist_yaw_deg=0.0,
                label="t",
            ),
            QueuedTarget(
                class_id="Square_Pedestal_A",
                shape_prefix="Square",
                position=Position(0.5, 0.0, 0.0),
                wrist_yaw_deg=0.0,
                label="s",
            ),
        ]
    )
    snap = _snap([], [_slot("object", "Square", 0.2)])
    pr = plan_pick(settings, snap, queue)
    assert pr.ok
    assert pr.object_slot is not None
    assert pr.object_slot.class_id == "Square"
    assert pr.queued_target is not None
    assert pr.queued_target.shape_prefix == "Square"


def test_plan_pick_target_from_obs1_queue_not_obs2_targets() -> None:
    settings = Settings()
    queue = deque(
        [
            QueuedTarget(
                class_id="Square_Pedestal_A",
                shape_prefix="Square",
                position=Position(0.2, 0.0, 0.0),
                wrist_yaw_deg=10.0,
                label="ped",
            )
        ]
    )
    snap = _snap(
        [_slot("target", "Wrong_Pedestal", 0.9)],
        [_slot("object", "Square", 0.3)],
    )
    pr = plan_pick(settings, snap, queue)
    assert pr.ok
    assert pr.target_slot is not None
    assert pr.target_slot.class_id == "Square_Pedestal_A"
    assert pr.target_slot.wrist_yaw_deg == pytest.approx(10.0)


def test_peek_target_track_no_fallback_to_wrong_prefix() -> None:
    snap = _snap(
        [_slot("target", "Trapezium_Pedestal_Blue", 0.1)],
        [],
    )
    t = peek_target_track(snap, deque(), shape_prefix_lock="Square")
    assert t is None


def test_peek_target_track_preferred_disambiguation() -> None:
    preferred = _slot("target", "Square_Pedestal_A", 0.2, 0.0, 0.0)
    snap = _snap(
        [
            _slot("target", "Square_Pedestal_A", 0.2, 0.0, 0.0),
            _slot("target", "Square_Pedestal_C", 0.8, 0.0, 0.0),
        ],
        [],
    )
    t = peek_target_track(
        snap, deque(), shape_prefix_lock="Square", preferred=preferred
    )
    assert t is not None
    assert t.class_id == "Square_Pedestal_A"


def test_queued_to_track_slot_uses_refined_queue_position() -> None:
    q = QueuedTarget(
        class_id="Square_Pedestal_A",
        shape_prefix="Square",
        position=Position(0.99, 0.88, 0.77),
        wrist_yaw_deg=15.0,
        label="ped",
        refined=True,
    )
    t = queued_to_track_slot(q)
    assert t.position.x == pytest.approx(0.99)
    assert t.wrist_yaw_deg == pytest.approx(15.0)


def test_plan_pick_skips_placed_uses_next() -> None:
    settings = Settings()
    queue = deque(
        [
            QueuedTarget(
                class_id="Circle_Pedestal_Blue",
                shape_prefix="Circle",
                position=Position(0.0, 0.0, 0.0),
                wrist_yaw_deg=0.0,
                label="c",
            ),
            QueuedTarget(
                class_id="Square_Pedestal_Blue",
                shape_prefix="Square",
                position=Position(0.1, 0.0, 0.0),
                wrist_yaw_deg=0.0,
                label="s",
            ),
        ]
    )
    queue[0].placed = True
    snap = SceneSnapshot(
        slots_by_role={
            "object": [
                _slot("object", "Square", "Square", 0.0, 0.0, 0.0),
            ],
        }
    )
    pr = plan_pick(settings, snap, queue)
    assert pr.ok
    assert pr.queued_target is not None
    assert pr.queued_target.shape_prefix == "Square"


def test_plan_pick_object_matches_prefix_via_label_when_class_id_numeric() -> None:
    settings = Settings()
    queue = deque(
        [
            QueuedTarget(
                class_id="Square_Pedestal_A",
                shape_prefix="Square",
                position=Position(0.1, 0.0, 0.0),
                wrist_yaw_deg=0.0,
                label="s",
                placed=True,
            ),
            QueuedTarget(
                class_id="Circle_Pedestal_Blue",
                shape_prefix="Circle",
                position=Position(0.2, 0.0, 0.0),
                wrist_yaw_deg=0.0,
                label="c",
            ),
        ]
    )
    snap = _snap(
        [],
        [
            TrackSlot(
                role="object",
                class_id=2,
                label="Circle_wafer",
                position=Position(0.3, 0.0, 0.1),
                wrist_yaw_deg=0.0,
                confidence=0.9,
            ),
        ],
    )
    pr = plan_pick(settings, snap, queue)
    assert pr.ok
    assert pr.object_slot is not None
    assert pr.queued_target is not None
    assert pr.queued_target.shape_prefix == "Circle"


def test_plan_pick_after_queue_head_popped_uses_next() -> None:
    settings = Settings()
    queue = deque(
        [
            QueuedTarget(
                class_id="Trapezium_Pedestal_B",
                shape_prefix="Trapezium",
                position=Position(0.1, 0.0, 0.0),
                wrist_yaw_deg=0.0,
                label="t",
            ),
            QueuedTarget(
                class_id="Square_Pedestal_A",
                shape_prefix="Square",
                position=Position(0.5, 0.0, 0.0),
                wrist_yaw_deg=0.0,
                label="s",
            ),
        ]
    )
    queue.popleft()
    snap = _snap([], [_slot("object", "Square", 0.2)])
    pr = plan_pick(settings, snap, queue)
    assert pr.ok
    assert pr.queued_target is not None
    assert pr.queued_target.shape_prefix == "Square"
