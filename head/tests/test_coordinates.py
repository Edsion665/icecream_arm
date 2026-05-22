"""固定运动高度与队列写回。"""

from __future__ import annotations

import pytest

from src.config import Settings
from src.coordinates import (
    object_pick_hover_and_work,
    target_place_hover_and_work,
    target_queue_pose_after_refine,
    target_refine_hover_pose,
)
from src.models import Position


def test_object_fixed_pick_heights() -> None:
    s = Settings()
    cam = Position(0.1, -0.4, 0.99)
    hover, work = object_pick_hover_and_work(cam, s)
    assert hover.z == pytest.approx(0.35)
    assert work.z == pytest.approx(0.25)
    assert hover.x == pytest.approx(0.1)
    assert hover.y == pytest.approx(-0.4)


def test_target_fixed_place_heights() -> None:
    s = Settings()
    slot = Position(0.04, 0.44, 9.99)
    hover, work = target_place_hover_and_work(slot, s)
    assert hover.z == pytest.approx(0.25)
    assert work.z == pytest.approx(0.15)


def test_target_refine_hover_fixed_z() -> None:
    s = Settings()
    pos = Position(0.0, 0.3, -1.0)
    h = target_refine_hover_pose(pos, s)
    assert h.z == pytest.approx(0.5)
    assert h.x == pytest.approx(0.0)


def test_step3_queue_writeback_ignores_camera_z() -> None:
    s = Settings()
    cam = Position(-0.12, 0.43, -0.21)
    q = target_queue_pose_after_refine(cam, s)
    assert q.z == pytest.approx(0.15)
    assert q.x == pytest.approx(-0.12)
