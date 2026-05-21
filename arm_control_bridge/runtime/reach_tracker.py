"""命令到位追踪与异步 TCP/HTTP 回传。"""

from __future__ import annotations

import collections
import json
import queue
import threading
import time
from typing import Any, Callable, Optional, Tuple

from ..config import CONFIG
from ..io.listener import ReplySlot

ReplyQueueItem = Tuple[dict[str, Any], Optional[Any], Optional[ReplySlot]]
OnReplyTcpError = Callable[[Any, BaseException], None]

_SUPERSEDED = {"ok": False, "reached": False, "error": "superseded"}


class ReachTracker:
    """到位状态机：HTTP/TCP 阻塞到 UDP 判到位；新命令抢占旧等待。

    ``register_pending`` 在 listener 线程调用；``accept``/``feed`` 在控制循环调用。
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.waiting = False
        self.cmd_kind = ""
        self._buf: collections.deque[bool] = collections.deque(maxlen=CONFIG.reached_stable_frames)
        self._deadline = 0.0
        self._tcp_conn: Optional[Any] = None
        self._http_slot: Optional[ReplySlot] = None
        self.reply_q: queue.Queue[Optional[ReplyQueueItem]] = queue.Queue()
        self._pending_tcp: Optional[Any] = None
        self._pending_slot: Optional[ReplySlot] = None

    def register_pending(self, *, tcp_conn: Any = None, http_slot: Optional[ReplySlot] = None) -> None:
        """登记回传槽；若上一条仍在等待或尚未 ``accept``，先以 ``superseded`` 唤醒。"""
        with self._lock:
            if self.waiting:
                self._finish_locked(_SUPERSEDED)
            elif self._pending_slot is not None or self._pending_tcp is not None:
                self._wake_pending_locked(_SUPERSEDED)
            self._pending_tcp = tcp_conn
            self._pending_slot = http_slot

    def accept(self, kind: str) -> None:
        """控制循环在应用新命令后调用：绑定 pending 槽并开始到位计时。"""
        with self._lock:
            self.waiting = True
            self.cmd_kind = kind
            self._buf.clear()
            self._deadline = time.monotonic() + CONFIG.reached_timeout_s
            self._tcp_conn = self._pending_tcp
            self._http_slot = self._pending_slot
            self._pending_tcp = None
            self._pending_slot = None

    def feed(self, reached: bool, result: dict[str, Any]) -> None:
        """控制循环每帧：关节/夹爪均用 UDP 反馈 + 连续 N 帧防抖。"""
        with self._lock:
            if not self.waiting:
                return
            if time.monotonic() > self._deadline:
                self._finish_locked({"ok": False, "reached": False, "error": "timeout"})
                return
            self._buf.append(reached)
            if len(self._buf) == CONFIG.reached_stable_frames and all(self._buf):
                out = dict(result)
                out["ok"] = True
                out["reached"] = True
                self._finish_locked(out)

    def _wake_pending_locked(self, result: dict[str, Any]) -> None:
        """尚未 ``accept`` 的 pending 槽（下一条命令已覆盖登记）。"""
        slot = self._pending_slot
        tcp = self._pending_tcp
        self._pending_slot = None
        self._pending_tcp = None
        if slot is not None:
            slot.result = dict(result)
            slot.event.set()
        if tcp is not None:
            self.reply_q.put_nowait((dict(result), tcp, None))

    def _finish_locked(self, result: dict[str, Any]) -> None:
        self.waiting = False
        tcp = self._tcp_conn
        slot = self._http_slot
        self._tcp_conn = None
        self._http_slot = None
        self.reply_q.put_nowait((dict(result), tcp, slot))

    def _finish(self, result: dict[str, Any]) -> None:
        with self._lock:
            self._finish_locked(result)


def reply_worker_loop(
    reply_q: queue.Queue[Optional[ReplyQueueItem]],
    *,
    on_tcp_send_error: Optional[OnReplyTcpError] = None,
) -> None:
    """阻塞处理回传队列直到收到 ``None`` 哨兵。"""
    while True:
        item = reply_q.get()
        if item is None:
            break
        result, tcp_conn, http_slot = item
        if tcp_conn is not None:
            try:
                tcp_conn.sendall((json.dumps(result) + "\n").encode())
            except OSError as exc:
                if on_tcp_send_error is not None:
                    on_tcp_send_error(tcp_conn, exc)
        if http_slot is not None:
            http_slot.result = result
            http_slot.event.set()


def start_reply_worker(
    tracker: ReachTracker,
    *,
    on_tcp_send_error: Optional[OnReplyTcpError] = None,
) -> threading.Thread:
    t = threading.Thread(
        target=reply_worker_loop,
        args=(tracker.reply_q,),
        kwargs={"on_tcp_send_error": on_tcp_send_error},
        daemon=True,
    )
    t.start()
    return t


def stop_reply_worker(tracker: ReachTracker) -> None:
    tracker.reply_q.put(None)
