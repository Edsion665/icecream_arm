from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Any, Dict, List
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from src.config import Settings

log = logging.getLogger(__name__)


@dataclass
class BridgeReply:
    ok: bool
    status_code: int
    body: Dict[str, Any]
    error: str | None = None


class BridgeClient:
    """HTTP client for head2bridge (joints / pose / claw)."""

    def __init__(self, settings: Settings) -> None:
        self._base = settings.bridge_base_url.rstrip("/")
        # 逻辑层 True=合拢 False=张开；下发 bridge 时 grip_state：0=合拢，1=张开
        self._last_grip_closed: bool = not settings.initial_grip_open

    @property
    def last_grip_closed(self) -> bool:
        return self._last_grip_closed

    def _post(self, path: str, payload: Dict[str, Any]) -> BridgeReply:
        url = f"{self._base}{path}"
        data = json.dumps(payload).encode("utf-8")
        req = Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urlopen(req, timeout=30.0) as resp:  # nosec B310
                raw = resp.read().decode("utf-8")
                code = resp.getcode() or 200
                try:
                    body = json.loads(raw) if raw else {}
                except json.JSONDecodeError:
                    return BridgeReply(False, code, {}, error="invalid json in bridge response")
                if not isinstance(body, dict):
                    return BridgeReply(False, code, {}, error="bridge response not object")
                ok = bool(body.get("ok", False))
                return BridgeReply(ok, code, body, error=None if ok else str(body.get("error", "unknown")))
        except HTTPError as e:
            try:
                raw = e.read().decode("utf-8")
                body = json.loads(raw) if raw else {}
            except Exception:  # noqa: BLE001
                body = {}
            return BridgeReply(False, e.code, body if isinstance(body, dict) else {}, error=str(e))
        except URLError as e:
            return BridgeReply(False, 0, {}, error=str(e.reason))

    def send_joints(self, axes_rel_deg: List[float], *, context: str = "") -> BridgeReply:
        if len(axes_rel_deg) != 4:
            return BridgeReply(False, 0, {}, error="axes_rel_deg must have length 4")
        log.info(
            "bridge POST /api/joints %s axes_rel_deg=%s",
            f"({context})" if context else "",
            list(axes_rel_deg),
        )
        return self._post(
            "/api/joints",
            {"cmd": "joints", "axes_rel_deg": list(axes_rel_deg)},
        )

    def joints_in_position(self, reply: BridgeReply) -> bool:
        return reply.ok and reply.status_code == 200

    def send_pose(self, x: float, y: float, z: float, *, context: str = "") -> BridgeReply:
        log.info(
            "bridge POST /api/pose %s x=%.4f y=%.4f z=%.4f (pose 不含 wrist；腕角仅由 claw 单独下发)",
            f"({context})" if context else "",
            x,
            y,
            z,
        )
        return self._post("/api/pose", {"cmd": "pose", "x": x, "y": y, "z": z})

    def pose_in_position(self, reply: BridgeReply) -> bool:
        if not reply.ok or reply.status_code != 200:
            return False
        b = reply.body
        if not bool(b.get("reached", False)):
            return False
        ap = b.get("actual_pose")
        if not isinstance(ap, dict):
            return False
        return all(k in ap for k in ("x", "y", "z"))

    def send_claw(self, wrist_deg: float, closed: bool, *, context: str = "") -> BridgeReply:
        # head2bridge：grip_state 0=合拢，1=张开
        grip_state = 0 if closed else 1
        payload = {
            "cmd": "claw",
            "wrist_deg": float(wrist_deg),
            "grip_state": grip_state,
        }
        log.info(
            "bridge POST /api/claw %s wrist_deg=%.3f grip_state=%s (0=合拢1=张开) closed(logical)=%s last_grip_closed(before)=%s",
            f"({context})" if context else "",
            float(wrist_deg),
            grip_state,
            closed,
            self._last_grip_closed,
        )
        rep = self._post("/api/claw", payload)
        if rep.ok:
            self._last_grip_closed = closed
            log.info(
                "bridge claw ok status=%s body_ok=%s",
                rep.status_code,
                rep.body.get("ok"),
            )
        else:
            log.warning("bridge claw failed status=%s error=%s body=%s", rep.status_code, rep.error, rep.body)
        return rep

    def require_claw(self, rep: BridgeReply, label: str) -> None:
        if not rep.ok:
            raise RuntimeError(f"{label}: claw failed: {rep.error or rep.body}")

    def toggle_grip(self, wrist_deg: float) -> BridgeReply:
        return self.send_claw(wrist_deg, not self._last_grip_closed)
