"""Tracker stable observation before slot apply."""

from __future__ import annotations

import threading
import time

from src.config import Settings
from src.models import DetectionFrame, ObjectDet, Position
from src.tracker import Tracker


def _frame(
    role: str,
    class_id: str,
    x: float,
    y: float,
    z: float,
    wy: float,
    *,
    frame_id: int,
) -> DetectionFrame:
    return DetectionFrame(
        frame="robot_base",
        objects=[
            ObjectDet(
                role=role,  # type: ignore[arg-type]
                class_id=class_id,
                label=class_id,
                position=Position(x, y, z),
                wrist_yaw_deg=wy,
            )
        ],
        frame_id=frame_id,
    )


def _feed_frames(tr: Tracker, n: int, pos: tuple[float, float, float, float], start_id: int = 0) -> None:
    x, y, z, wy = pos
    for i in range(n):
        tr.submit_frame(
            _frame("target", "Square_Pedestal_A", x, y, z, wy, frame_id=start_id + i)
        )
        time.sleep(0.08)


def test_observe_requires_three_stable_frames() -> None:
    s = Settings(observe_stable_frames=3, observe_stable_pos_eps_m=0.02, observe_stable_yaw_eps_deg=5.0)
    tr = Tracker(s)
    tr.start()
    ok = [False]
    try:

        def observe() -> None:
            ok[0] = tr.observe_role_stable("target", timeout=5.0)

        t_obs = threading.Thread(target=observe, daemon=True)
        t_obs.start()
        time.sleep(0.05)
        _feed_frames(tr, 3, (0.1, 0.0, 0.1, 1.0))
        t_obs.join(timeout=6.0)
        assert ok[0]
        snap = tr.get_snapshot()
        assert len(snap.slots_by_role.get("target") or []) == 1
    finally:
        tr.stop()


def test_observe_succeeds_when_one_of_two_targets_stable() -> None:
    """画面内两个 target，只需其中一个连续 3 帧稳定即可结束观测。"""
    s = Settings(observe_stable_frames=3, observe_stable_pos_eps_m=0.02, observe_stable_yaw_eps_deg=5.0)
    tr = Tracker(s)
    tr.start()
    ok = [False]
    try:

        def observe() -> None:
            ok[0] = tr.observe_role_stable("target", timeout=6.0)

        def feed() -> None:
            time.sleep(0.05)
            for i in range(3):
                tr.submit_frame(
                    DetectionFrame(
                        frame="robot_base",
                        objects=[
                            ObjectDet(
                                role="target",
                                class_id="Square_A",
                                label="A",
                                position=Position(0.1, 0.0, 0.1),
                                wrist_yaw_deg=1.0,
                            ),
                            ObjectDet(
                                role="target",
                                class_id="Circle_B",
                                label="B",
                                position=Position(0.5 + i * 0.2, 0.0, 0.1),
                                wrist_yaw_deg=0.0,
                            ),
                        ],
                        frame_id=i,
                    )
                )
                time.sleep(0.08)

        threading.Thread(target=observe, daemon=True).start()
        threading.Thread(target=feed, daemon=True).start()
        time.sleep(3.0)
        assert ok[0]
        snap = tr.get_snapshot()
        slots = snap.slots_by_role.get("target") or []
        assert len(slots) == 1
        assert slots[0].class_id == "Square_A"
        assert slots[0].position.x == 0.1
    finally:
        tr.stop()


def test_observe_resets_streak_on_large_jump() -> None:
    s = Settings(observe_stable_frames=3)
    tr = Tracker(s)
    tr.start()
    ok = [False]
    try:

        def observe() -> None:
            ok[0] = tr.observe_role_stable("target", timeout=8.0)

        def feed() -> None:
            time.sleep(0.05)
            _feed_frames(tr, 2, (0.1, 0.0, 0.1, 0.0), start_id=0)
            tr.submit_frame(_frame("target", "Square_Pedestal_A", 0.5, 0.0, 0.1, 0.0, frame_id=10))
            time.sleep(0.08)
            _feed_frames(tr, 3, (0.5, 0.0, 0.1, 0.0), start_id=11)

        threading.Thread(target=observe, daemon=True).start()
        threading.Thread(target=feed, daemon=True).start()
        time.sleep(4.0)
        assert ok[0]
        snap = tr.get_snapshot()
        assert len(snap.slots_by_role.get("target") or []) == 1
        assert snap.slots_by_role["target"][0].position.x == 0.5
    finally:
        tr.stop()
