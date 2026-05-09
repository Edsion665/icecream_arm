from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Literal, Optional

Role = Literal["object", "target", "lid"]
ObsTag = Literal["obs1_first", "obs2", "obs1_return"]


@dataclass
class Position:
    x: float
    y: float
    z: float

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Position":
        return Position(float(d["x"]), float(d["y"]), float(d["z"]))

    def dist(self, other: "Position") -> float:
        return (
            (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2
        ) ** 0.5


@dataclass
class ObjectDet:
    role: Role
    class_id: Any
    label: str
    position: Position
    wrist_yaw_deg: float
    track_id: Any | None = None
    confidence: float | None = None
    bbox_2d: Dict[str, float] | None = None


@dataclass
class DetectionFrame:
    frame: str
    objects: List[ObjectDet]
    frame_id: int | None = None
    ts: Any | None = None


@dataclass
class TrackSlot:
    role: Role
    class_id: Any
    label: str
    position: Position
    wrist_yaw_deg: float
    track_id: Any | None = None
    confidence: float | None = None
    last_frame_id: int | None = None
    last_ts: Any | None = None
    frame: str = "robot_base"


@dataclass
class SceneSnapshot:
    """Immutable-ish snapshot for planner."""

    slots_by_role: Dict[str, List[TrackSlot]] = field(default_factory=dict)
    last_frame: DetectionFrame | None = None


@dataclass
class PlanResult:
    ok: bool
    reason: str = ""
    object_slot: TrackSlot | None = None
    target_slot: TrackSlot | None = None
