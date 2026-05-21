from __future__ import annotations

from src.conveyor_obs import object_in_pick_zone, object_xy_dist_m
from src.models import DetectionFrame, ObjectDet, Position


def _obj(x: float, y: float) -> DetectionFrame:
    return DetectionFrame(
        frame="robot_base",
        objects=[
            ObjectDet(
                role="object",
                class_id="cube",
                label="cube",
                position=Position(x, y, 0.1),
                wrist_yaw_deg=0.0,
            )
        ],
    )


def test_object_xy_dist() -> None:
    assert abs(object_xy_dist_m(Position(3.0, 4.0, 0.0)) - 5.0) < 1e-6


def test_object_in_pick_zone() -> None:
    assert object_in_pick_zone(_obj(0.3, 0.3), 0.6) is True
    assert object_in_pick_zone(_obj(0.5, 0.5), 0.6) is False
    assert object_in_pick_zone(None, 0.6) is False
    assert object_in_pick_zone(DetectionFrame(frame="robot_base", objects=[]), 0.6) is False
