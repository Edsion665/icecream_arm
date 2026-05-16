"""树莓派 -> head：TCP JSON 行（pi2head），用于下发 start 等控制信号。"""

from __future__ import annotations

import json
import logging
import socket
import threading
from typing import Any, Callable, Dict, Tuple

log = logging.getLogger(__name__)

LogFn = Callable[[str], None]


def parse_pi2head_payload(body: Dict[str, Any]) -> Tuple[bool, str | None, str | None]:
    """解析一行 JSON。返回 (ok, error, command_name)。"""
    cmd = body.get("cmd") or body.get("type")
    if cmd is None:
        return False, "missing_field: 缺少字段 cmd 或 type", None
    name = str(cmd).strip().lower()
    if name == "start":
        return True, None, "start"
    if name == "ping":
        return True, None, "ping"
    return False, f"unknown_cmd: {cmd}", None


class Pi2HeadServer:
    """监听树莓派 TCP 连接；收到 ``start`` 后置位，供 run 主循环放行。"""

    def __init__(self, host: str, port: int, *, on_log: LogFn | None = None) -> None:
        self._host = host
        self._port = int(port)
        self._on_log = on_log or (lambda msg: log.info("%s", msg))
        self._started = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def is_started(self) -> bool:
        return self._started.is_set()

    def wait_started(self, timeout: float | None = None) -> bool:
        return self._started.wait(timeout=timeout)

    def start_background(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._serve, name="pi2head-tcp", daemon=True)
        self._thread.start()
        self._on_log(f"pi2head TCP 监听 {self._host}:{self._port}（等待 cmd=start）")

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _reply(self, conn: socket.socket, payload: Dict[str, Any]) -> None:
        line = json.dumps(payload, ensure_ascii=False).encode("utf-8") + b"\n"
        conn.sendall(line)

    def _serve(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((self._host, self._port))
        except OSError as exc:
            self._on_log(f"pi2head bind 失败 {self._host}:{self._port}: {exc}")
            return
        srv.listen(5)
        srv.settimeout(0.5)
        while not self._stop.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with conn:
                self._on_log(f"pi2head 连接来自 {addr[0]}:{addr[1]}")
                buf = b""
                while not self._stop.is_set():
                    try:
                        chunk = conn.recv(4096)
                    except OSError:
                        break
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            body = json.loads(line.decode("utf-8"))
                        except (UnicodeDecodeError, json.JSONDecodeError):
                            self._reply(conn, {"ok": False, "error": "invalid json"})
                            continue
                        if not isinstance(body, dict):
                            self._reply(conn, {"ok": False, "error": "invalid json"})
                            continue
                        ok, err, cmd_name = parse_pi2head_payload(body)
                        if not ok or cmd_name is None:
                            self._reply(conn, {"ok": False, "error": err or "bad request"})
                            continue
                        if cmd_name == "ping":
                            self._reply(conn, {"ok": True, "pong": True})
                            continue
                        if cmd_name == "start":
                            with self._lock:
                                already = self._started.is_set()
                                self._started.set()
                            self._reply(conn, {"ok": True, "started": True, "already_started": already})
                            self._on_log(
                                "pi2head 收到 start"
                                + ("（重复，已忽略）" if already else " → 放行 FSM")
                            )
        try:
            srv.close()
        except OSError:
            pass
