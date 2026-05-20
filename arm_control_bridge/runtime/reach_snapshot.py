"""供 GET /api/reached 读取的到位快照（与主循环同频更新）。"""

from __future__ import annotations

import collections
import threading
from typing import Any

from ..config import CONFIG


class ReachSnapshot:
    """线程安全；连续 ``reached_stable_frames`` 帧到位后才置 ``reached: true``。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._buf: collections.deque[bool] = collections.deque(maxlen=CONFIG.reached_stable_frames)
        self._body: dict[str, Any] = {"ok": True, "reached": False, "mode": "joints"}
        self._last_cmd_kind: str = ""

    def reset_stable_buffer(self) -> None:
        with self._lock:
            self._buf.clear()

    def set_last_cmd(self, kind: str) -> None:
        with self._lock:
            self._last_cmd_kind = str(kind)

    def last_cmd_is_pose(self) -> bool:
        with self._lock:
            return self._last_cmd_kind in ("pose", "pose_delta")

    def update(
        self,
        *,
        mode: str,
        reached_now: bool,
        error_joints_deg: float,
        actual_pose: dict[str, float] | None = None,
        reach_meta: dict[str, Any] | None = None,
    ) -> None:
        with self._lock:
            self._buf.append(reached_now)
            stable = (
                len(self._buf) == CONFIG.reached_stable_frames and all(self._buf)
            )
            body: dict[str, Any] = {
                "ok": True,
                "reached": stable,
                "mode": mode,
                "error_joints_deg": round(float(error_joints_deg), 3),
            }
            if self._last_cmd_kind:
                body["cmd_kind"] = self._last_cmd_kind
            if actual_pose is not None:
                body["actual_pose"] = actual_pose
            if reach_meta:
                body.update(reach_meta)
            self._body = body

    def get(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._body)
