from __future__ import annotations

import json
import logging
import time
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
        self._settings = settings
        self._base = settings.bridge_base_url.rstrip("/")
        self._reached_poll_s = float(settings.bridge_reached_poll_s)
        self._require_bridge_feedback = bool(settings.require_bridge_feedback)
        # 逻辑层 closed=True/False；下发 grip_state：0=关闭，1=张开
        self._last_grip_closed: bool = not settings.initial_grip_open

    @property
    def last_grip_closed(self) -> bool:
        return self._last_grip_closed

    def _post(self, path: str, payload: Dict[str, Any]) -> BridgeReply:
        url = f"{self._base}{path}"
        data = json.dumps(payload).encode("utf-8")
        req = Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
        try:
            post_timeout = max(30.0, float(self._settings.state_timeout_s) + 5.0)
            with urlopen(req, timeout=post_timeout) as resp:  # nosec B310
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

    def _get(self, path: str) -> BridgeReply:
        url = f"{self._base}{path}"
        req = Request(url, method="GET")
        try:
            with urlopen(req, timeout=10.0) as resp:  # nosec B310
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

    def poll_reached(self) -> BridgeReply:
        return self._get("/api/reached")

    @staticmethod
    def log_reached_timeout(context: str, rep: BridgeReply, *, timeout_s: float) -> None:
        """超时：打印四轴反馈 vs 设定角（来自 bridge /api/reached 的 pi2camera 字段）。"""
        b = rep.body if isinstance(rep.body, dict) else {}
        actual = b.get("actual_rel_deg")
        target = b.get("target_rel_deg")
        per_axis = b.get("per_axis_err_deg")
        lines = [
            f"到位超时 ({context}) timeout={timeout_s}s | "
            f"reached={b.get('reached')} feedback_available={b.get('feedback_available')} "
            f"reach_reason={b.get('reach_reason')!r} udp_listening={b.get('udp_listening')} "
            f"target_source={b.get('target_source')!r} packet_age_ms={b.get('packet_age_ms')} "
            f"error_joints_deg={b.get('error_joints_deg')} reached_tol_deg={b.get('reached_tol_deg')} "
            f"mode={b.get('mode')!r} cmd_kind={b.get('cmd_kind')!r}",
        ]
        if b.get("motor_rad") is not None:
            lines.append(f"  motor_rad(rad): {b.get('motor_rad')}")
        if b.get("commanded_rel_deg") is not None:
            lines.append(f"  bridge 指令关节(deg): {b.get('commanded_rel_deg')}")
        if actual is not None and target is not None:
            lines.append(f"  四轴 反馈(actual_rel_deg): {actual}")
            lines.append(f"  四轴 设定(target_rel_deg): {target}")
            if per_axis is not None:
                lines.append(f"  四轴 误差|反馈-设定|(deg): {per_axis}")
        else:
            lines.append(
                "  （无 actual_rel_deg：未收到 pi2camera UDP camera_state.motor_rad，"
                "或 require_bridge_feedback 阻止判到位；确认 bridge 监听 9982）"
            )
        if b.get("actual_pose"):
            lines.append(f"  actual_pose: {b.get('actual_pose')}")
        log.error("\n".join(lines))

    def _log_reached_wait(self, label: str, t0: float, rep: BridgeReply) -> None:
        dt = time.monotonic() - t0
        if dt < 0.8:
            return
        b = rep.body if isinstance(rep.body, dict) else {}
        err = b.get("error_joints_deg")
        per = b.get("per_axis_err_deg")
        reason = b.get("reach_reason")
        log.info(
            "bridge 到位等待 %.2fs (%s) reached=%s err_deg=%s per_axis=%s reason=%s",
            dt,
            label,
            b.get("reached"),
            err,
            per,
            reason,
        )

    def _require_blocking_reached(
        self, rep: BridgeReply, label: str, *, expect_pose: bool = False
    ) -> BridgeReply:
        """方案 A：POST 阻塞至 bridge UDP 判到位；失败/被抢占/超时在此抛出。"""
        b = rep.body if isinstance(rep.body, dict) else {}
        err = b.get("error") or rep.error
        if err == "superseded":
            raise RuntimeError(f"{label}: superseded by newer command")
        if not rep.ok or rep.status_code != 200:
            if err == "timeout":
                self.log_reached_timeout(label, rep, timeout_s=float(self._settings.state_timeout_s))
            raise RuntimeError(f"{label}: bridge failed: {err or b}")
        if self._require_bridge_feedback and not bool(b.get("feedback_available", False)):
            raise RuntimeError(
                f"{label}: no pi feedback (reach_reason={b.get('reach_reason')!r})"
            )
        if not bool(b.get("reached", False)):
            raise RuntimeError(f"{label}: not reached: {b}")
        if expect_pose:
            ap = b.get("actual_pose")
            if not isinstance(ap, dict) or not all(k in ap for k in ("x", "y", "z")):
                raise RuntimeError(f"{label}: missing actual_pose in bridge response")
        return rep

    def send_joints(self, axes_rel_deg: List[float], *, context: str = "") -> BridgeReply:
        if len(axes_rel_deg) != 4:
            return BridgeReply(False, 0, {}, error="axes_rel_deg must have length 4")
        label = context or "joints"
        log.debug("bridge POST /api/joints (%s) axes_rel_deg=%s", label, list(axes_rel_deg))
        rep = self._post(
            "/api/joints",
            {"cmd": "joints", "axes_rel_deg": list(axes_rel_deg)},
        )
        return self._require_blocking_reached(rep, label)

    def _reached_with_feedback(self, reply: BridgeReply) -> bool:
        if not reply.ok or reply.status_code != 200:
            return False
        b = reply.body
        if self._require_bridge_feedback and not bool(b.get("feedback_available", False)):
            return False
        return bool(b.get("reached", False))

    def joints_in_position(self, reply: BridgeReply) -> bool:
        if not reply.ok or reply.status_code != 200:
            return False
        b = reply.body
        mode = b.get("mode")
        if mode is not None and mode != "joints":
            return False
        return self._reached_with_feedback(reply)

    def send_pose(self, x: float, y: float, z: float, *, context: str = "") -> BridgeReply:
        label = context or "pose"
        log.debug("bridge POST /api/pose (%s) x=%.4f y=%.4f z=%.4f", label, x, y, z)
        rep = self._post("/api/pose", {"cmd": "pose", "x": x, "y": y, "z": z})
        return self._require_blocking_reached(rep, label, expect_pose=True)

    def send_pose_delta(
        self, dx: float, dy: float, dz: float, *, context: str = ""
    ) -> BridgeReply:
        label = context or "pose_delta"
        log.debug(
            "bridge POST /api/pose_delta (%s) dx=%.4f dy=%.4f dz=%.4f",
            label,
            float(dx),
            float(dy),
            float(dz),
        )
        rep = self._post(
            "/api/pose_delta",
            {"cmd": "pose_delta", "dx": float(dx), "dy": float(dy), "dz": float(dz)},
        )
        return self._require_blocking_reached(rep, label, expect_pose=True)

    def pose_in_position(self, reply: BridgeReply) -> bool:
        if not reply.ok or reply.status_code != 200:
            return False
        b = reply.body
        # bridge 将 pose 转为 joints 下发；以 pi2camera 反馈到位 + actual_pose 为准
        if not self._reached_with_feedback(reply):
            return False
        ap = b.get("actual_pose")
        if not isinstance(ap, dict):
            return False
        return all(k in ap for k in ("x", "y", "z"))

    def send_claw(self, wrist_deg: float, closed: bool, *, context: str = "") -> BridgeReply:
        # 现场约定：grip_state 0=关闭，1=张开
        grip_state = 0 if closed else 1
        payload = {
            "cmd": "claw",
            "wrist_deg": float(wrist_deg),
            "grip_state": grip_state,
        }
        log.debug(
            "bridge POST /api/claw %s wrist_deg=%.3f grip_state=%s",
            f"({context})" if context else "",
            float(wrist_deg),
            grip_state,
        )
        label = context or "claw"
        rep = self._post("/api/claw", payload)
        try:
            rep = self._require_blocking_reached(rep, label)
        except RuntimeError:
            log.warning("bridge claw failed status=%s body=%s", rep.status_code, rep.body)
            raise
        self._last_grip_closed = closed
        return rep

    def require_claw(self, rep: BridgeReply, label: str) -> None:
        """``send_claw`` 已阻塞判到位；保留兼容旧调用。"""
        self._require_blocking_reached(rep, label)

    def toggle_grip(self, wrist_deg: float) -> BridgeReply:
        return self.send_claw(wrist_deg, not self._last_grip_closed)

    def send_conveyor(self, run: int, *, context: str = "") -> BridgeReply:
        """传送带启停：run 0=停，1=转（锁存至 bridge UDP p_rel_deg[7]）。"""
        run_i = 1 if int(run) != 0 else 0
        log.debug(
            "bridge POST /api/conveyor %s run=%s",
            f"({context})" if context else "",
            run_i,
        )
        return self._post("/api/conveyor", {"cmd": "conveyor", "run": run_i})

    def send_idle_zeros(
        self,
        axes_rel_deg: List[float],
        *,
        wrist_deg: float = 0.0,
        grip_closed: bool = False,
        context: str = "idle_hold",
    ) -> None:
        """启动待命：向 bridge 下发关节与 claw（``wait_reached: false``，不阻塞、不要求 UDP 反馈）。"""
        if len(axes_rel_deg) != 4:
            log.warning("send_idle_zeros: axes 长度须为 4，跳过")
            return
        jr = self._post(
            "/api/joints",
            {
                "cmd": "joints",
                "axes_rel_deg": list(axes_rel_deg),
                "wait_reached": False,
            },
        )
        if not jr.ok:
            log.warning("idle joints failed: %s", jr.error or jr.body)
        grip_state = 0 if grip_closed else 1
        cr = self._post(
            "/api/claw",
            {
                "cmd": "claw",
                "wrist_deg": float(wrist_deg),
                "grip_state": grip_state,
                "wait_reached": False,
            },
        )
        if cr.ok:
            self._last_grip_closed = grip_closed
        else:
            log.warning("idle claw failed: %s", cr.error or cr.body)
