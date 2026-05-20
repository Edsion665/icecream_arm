"""腕角补偿：wrist_cmd = wrist_cam − j1_bias。"""

from __future__ import annotations

import math

from src.coordinates import (
    j1_deg_from_xy,
    wrist_cmd_deg,
    wrist_deg_for_object,
    wrist_deg_for_target,
    wrist_j1_bias_deg,
)
from src.config import Settings
from src.models import Position


def test_j1_from_xy_matches_bridge_ik_sign() -> None:
    j1 = j1_deg_from_xy(0.363, -0.188)
    expected = -math.degrees(math.atan2(-0.188, 0.363))
    assert j1 == expected
    assert 25.0 < j1 < 30.0


def test_place_wrist_cmd_is_cam_minus_bias() -> None:
    s = Settings(
        observe1_axes_rel_deg=[0.0, 20.0, -80.0, -100.0],
        observe2_axes_rel_deg=[-90.0, 10.0, -60.0, -100.0],
    )
    pos = Position(0.363, -0.188, 0.471)
    wrist_cam = -37.6
    bias = wrist_j1_bias_deg(pos.x, pos.y, 0.0)
    cmd = wrist_deg_for_target(wrist_cam, pos, s)
    assert cmd == wrist_cam - bias
    assert cmd == wrist_cmd_deg(wrist_cam, pos, 0.0)


def test_pick_wrist_cmd_subtracts_obs2_j1_bias() -> None:
    s = Settings(
        observe1_axes_rel_deg=[0.0, 20.0, -80.0, -100.0],
        observe2_axes_rel_deg=[-90.0, 10.0, -60.0, -100.0],
    )
    pos = Position(0.4, 0.0, 0.3)
    wrist_cam = 10.0
    bias = wrist_j1_bias_deg(pos.x, pos.y, -90.0)
    cmd = wrist_deg_for_object(wrist_cam, pos, s)
    assert cmd == wrist_cam - bias
