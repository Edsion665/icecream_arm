from __future__ import annotations

import time

from src.config import Settings
from src.models import DetectionFrame, ObjectDet, Position
from src.tracker import Tracker


def _settings() -> Settings:
    return Settings(
        bridge_base_url="http://127.0.0.1:9",
        ingest_host="127.0.0.1",
        ingest_port=0,
        merge_pos_eps_m=0.01,
        merge_yaw_eps_deg=5.0,
        observe1_axes_rel_deg=[0, 0, 0, 0],
        observe2_axes_rel_deg=[0, 0, 0, 0],
        refresh_wait_s=1.0,
    )


def test_tracker_merge_small_move_updates_slot() -> None:
    s = _settings()
    tr = Tracker(s)
    tr.start()
    try:
        f1 = DetectionFrame(
            frame="robot_base",
            objects=[
                ObjectDet(
                    role="object",
                    class_id=1,
                    label="c",
                    position=Position(0.4, 0.0, 0.1),
                    wrist_yaw_deg=1.0,
                    track_id="a",
                )
            ],
            frame_id=1,
        )
        tr.submit_frame(f1)
        tr.request_refresh("obs1_first", timeout=1.0)
        snap = tr.get_snapshot()
        assert len(snap.slots_by_role["object"]) == 1
        f2 = DetectionFrame(
            frame="robot_base",
            objects=[
                ObjectDet(
                    role="object",
                    class_id=1,
                    label="c",
                    position=Position(0.401, 0.0, 0.1),
                    wrist_yaw_deg=1.5,
                    track_id="a",
                )
            ],
            frame_id=2,
        )
        tr.submit_frame(f2)
        tr.request_refresh("obs2", timeout=1.0)
        snap2 = tr.get_snapshot()
        assert len(snap2.slots_by_role["object"]) == 1
        assert abs(snap2.slots_by_role["object"][0].position.x - 0.401) < 1e-6
    finally:
        tr.stop()


def test_parse_detection_minimal() -> None:
    from src.listener import parse_detection_payload

    ok, err, det = parse_detection_payload(
        {
            "frame": "robot_base",
            "objects": [
                {
                    "role": "target",
                    "class_id": 1,
                    "label": "t",
                    "position": {"x": 0.5, "y": 0, "z": 0.1},
                    "wrist_yaw_deg": 0.0,
                }
            ],
        },
        implied_detection=True,
    )
    assert ok and err is None and det is not None
    assert det.objects[0].role == "target"


def test_clear_role_then_apply_from_last_frame() -> None:
    s = _settings()
    tr = Tracker(s)
    tr.start()
    try:
        f = DetectionFrame(
            frame="robot_base",
            objects=[
                ObjectDet(
                    role="target",
                    class_id=9,
                    label="slot",
                    position=Position(0.5, 0.0, 0.1),
                    wrist_yaw_deg=2.0,
                    track_id="t1",
                ),
                ObjectDet(
                    role="object",
                    class_id=1,
                    label="cone",
                    position=Position(0.4, 0.0, 0.1),
                    wrist_yaw_deg=3.0,
                    track_id="o1",
                ),
            ],
            frame_id=10,
        )
        tr.submit_frame(f)
        tr.request_refresh("obs1_first", timeout=1.0)
        assert tr.clear_role("target", timeout=1.0)
        snap_mid = tr.get_snapshot()
        assert snap_mid.slots_by_role["target"] == []
        assert tr.apply_roles_from_last_frame(frozenset({"target"}), timeout=1.0)
        snap = tr.get_snapshot()
        assert len(snap.slots_by_role["target"]) == 1
        assert abs(snap.slots_by_role["target"][0].position.x - 0.5) < 1e-6
        assert len(snap.slots_by_role["object"]) == 1
    finally:
        tr.stop()


def test_wait_for_roles_unblocks_after_ingest() -> None:
    s = _settings()
    tr = Tracker(s)
    tr.start()
    try:

        def _late_send() -> None:
            time.sleep(0.05)
            tr.submit_frame(
                DetectionFrame(
                    frame="robot_base",
                    objects=[
                        ObjectDet(
                            role="object",
                            class_id=1,
                            label="c",
                            position=Position(0.1, 0.2, 0.3),
                            wrist_yaw_deg=0.0,
                        )
                    ],
                    frame_id=1,
                )
            )

        import threading

        threading.Thread(target=_late_send, daemon=True).start()
        assert tr.wait_for_roles(frozenset({"object"}), timeout=2.0)
    finally:
        tr.stop()
