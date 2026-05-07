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


class ReachTracker:
    """无锁到位状态机：listener 登记 pending，控制循环 accept/feed。

    设计背景：TCP/HTTP 与主控制循环线程不同，用原子字段传递 pending 连接，
    避免在热路径加锁。
    """

    def __init__(self) -> None:
        self.waiting = False
        self.cmd_kind = ""
        self._buf: collections.deque[bool] = collections.deque(maxlen=CONFIG.reached_stable_frames)
        self._deadline = 0.0
        self._tcp_conn: Optional[Any] = None
        self._http_slot: Optional[ReplySlot] = None
        self._claw_timer: Optional[threading.Timer] = None
        self.reply_q: queue.Queue[Optional[ReplyQueueItem]] = queue.Queue()
        self._pending_tcp: Optional[Any] = None
        self._pending_slot: Optional[ReplySlot] = None

    def register_pending(self, *, tcp_conn: Any = None, http_slot: Optional[ReplySlot] = None) -> None:
        """由 listener 线程调用，仅写入 pending（控制循环随后 ``accept`` 消费）。"""
        self._pending_tcp = tcp_conn
        self._pending_slot = http_slot

    def accept(self, kind: str) -> None:
        """控制循环在解析到新运动/爪命令时调用。"""
        if self._claw_timer is not None:
            self._claw_timer.cancel()
            self._claw_timer = None
        self.waiting = True
        self.cmd_kind = kind
        self._buf.clear()
        self._deadline = time.monotonic() + CONFIG.reached_timeout_s
        self._tcp_conn = self._pending_tcp
        self._http_slot = self._pending_slot
        self._pending_tcp = None
        self._pending_slot = None
        if kind == "claw":
            t = threading.Timer(CONFIG.reached_claw_delay_s, self._claw_done)
            t.daemon = True
            t.start()
            self._claw_timer = t

    def feed(self, reached: bool, result: dict[str, Any]) -> None:
        """控制循环每帧调用；claw 模式由定时器结束，不走 feed。"""
        if not self.waiting or self.cmd_kind == "claw":
            return
        if time.monotonic() > self._deadline:
            self._finish({"ok": False, "reached": False, "error": "timeout"})
            return
        self._buf.append(reached)
        if len(self._buf) == CONFIG.reached_stable_frames and all(self._buf):
            self._finish(result)

    def _claw_done(self) -> None:
        self._finish({"ok": True, "reached": True})

    def _finish(self, result: dict[str, Any]) -> None:
        self.waiting = False
        self._claw_timer = None
        self.reply_q.put_nowait((result, self._tcp_conn, self._http_slot))
        self._tcp_conn = None
        self._http_slot = None


def reply_worker_loop(
    reply_q: queue.Queue[Optional[ReplyQueueItem]],
    *,
    on_tcp_send_error: Optional[OnReplyTcpError] = None,
) -> None:
    """阻塞处理回传队列直到收到 ``None`` 哨兵。

    Args:
        reply_q: ``(result_dict, tcp_conn, http_slot)``；``None`` 结束循环。
        on_tcp_send_error: TCP ``sendall`` 失败时回调 ``(conn, exc)``。
    """
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
    """启动守护线程执行 ``reply_worker_loop``。"""
    t = threading.Thread(
        target=reply_worker_loop,
        args=(tracker.reply_q,),
        kwargs={"on_tcp_send_error": on_tcp_send_error},
        daemon=True,
    )
    t.start()
    return t


def stop_reply_worker(tracker: ReachTracker) -> None:
    """向回传线程发送停止哨兵。"""
    tracker.reply_q.put(None)
