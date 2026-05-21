from __future__ import annotations

import math
from typing import List

from src.models import DetectionFrame, ObjectDet, Position


def object_xy_dist_m(pos: Position) -> float:
    """物体中心相对基点 (0,0) 的水平距离（米）。"""
    return math.hypot(pos.x, pos.y)


def objects_from_frame(frame: DetectionFrame | None) -> List[ObjectDet]:
    if frame is None:
        return []
    out: List[ObjectDet] = []
    for o in frame.objects:
        if o.role != "object":
            continue
        if all(math.isfinite(v) for v in (o.position.x, o.position.y, o.position.z)):
            out.append(o)
    return out


def nearest_object_xy(objects: List[ObjectDet]) -> ObjectDet | None:
    if not objects:
        return None
    return min(objects, key=lambda o: object_xy_dist_m(o.position))


def object_in_pick_zone(frame: DetectionFrame | None, max_xy_m: float) -> bool:
    """帧内存在 object 且最近物体的水平距基点 <= max_xy_m。"""
    nearest = nearest_object_xy(objects_from_frame(frame))
    if nearest is None:
        return False
    return object_xy_dist_m(nearest.position) <= float(max_xy_m)
