"""
统一监听与命令归一化。

- ``FrontendListener`` / ``frontend_listener``: HTTP 入口
- ``NetworkListener`` / ``network_listener``: TCP JSON 行协议
- ``ClawListener`` / ``claw_listener``: 仅爪通道队列
"""

from __future__ import annotations

import json
import os
import queue
import socket
import threading
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Callable, Optional

LogFn = Optional[Callable[[str], None]]
PendingFn = Optional[Callable[..., None]]


class ReplySlot:
    """HTTP 与异步回传线程之间的结果槽。"""

    def __init__(self) -> None:
        self.event = threading.Event()
        self.result: dict[str, Any] = {"ok": True}


@dataclass
class MotionCommand4Axis:
    """主臂四轴 + 笛卡尔类命令。"""

    kind: str
    payload: dict[str, Any]


@dataclass
class ClawCommand:
    """腕/爪独立通道。"""

    kind: str
    payload: dict[str, Any]


NormalizedCommand = MotionCommand4Axis | ClawCommand


class CommandNormalizer:
    """将 JSON/dict 规范为 ``MotionCommand4Axis`` 或 ``ClawCommand``。"""

    @staticmethod
    def _command_from_obj(obj: dict[str, Any]) -> str:
        cmd = obj.get("cmd") or obj.get("type")
        if not cmd:
            raise ValueError("missing_field: 缺少字段 cmd 或 type")
        return str(cmd).lower()

    @staticmethod
    def _normalize_grip_state(v: Any) -> float:
        if isinstance(v, str):
            s = v.strip().lower()
            if s in ("open", "0", "false"):
                return 0.0
            if s in ("close", "closed", "1", "true"):
                return 1.0
            raise ValueError("invalid_value: grip_state 必须为 0/1")
        try:
            x = float(v)
        except (TypeError, ValueError) as ex:
            raise ValueError("invalid_value: grip_state 必须为 0/1") from ex
        return 1.0 if x >= 0.5 else 0.0

    @classmethod
    def normalize_obj(cls, obj: dict[str, Any]) -> NormalizedCommand:
        """解析完整命令对象。

        Args:
            obj: 至少含 ``cmd`` 或 ``type`` 字段。

        Returns:
            归一化后的命令实例。

        Raises:
            ValueError: 字段缺失、长度非法或未知命令字。
        """
        k = cls._command_from_obj(obj)
        if k in ("pose", "set_pose", "xyz"):
            for key in ("x", "y", "z"):
                if key not in obj:
                    raise ValueError(f"missing_field: pose 缺少 {key}")
            return MotionCommand4Axis(kind="pose", payload=obj)
        if k in ("pose_delta", "delta_pose", "nudge"):
            for key in ("dx", "dy", "dz"):
                if key not in obj:
                    raise ValueError("missing_field: pose_delta 需要 dx,dy,dz")
            return MotionCommand4Axis(kind="pose_delta", payload=obj)
        if k in ("joints", "joint", "axes", "set_joints"):
            if "axes_rel_deg" not in obj:
                raise ValueError("missing_field: joints 需要 axes_rel_deg")
            arr = obj["axes_rel_deg"]
            if not isinstance(arr, (list, tuple)) or len(arr) != 4:
                raise ValueError("invalid_length: axes_rel_deg 必须长度 4")
            obj2 = dict(obj)
            obj2["axes_rel_deg"] = [float(arr[i]) for i in range(4)]
            return MotionCommand4Axis(kind="joints", payload=obj2)
        if k in ("joints_delta", "delta_joints", "axes_delta"):
            if "deltas_rel_deg" not in obj:
                raise ValueError("missing_field: joints_delta 需要 deltas_rel_deg")
            arr = obj["deltas_rel_deg"]
            if not isinstance(arr, (list, tuple)) or len(arr) != 4:
                raise ValueError("invalid_length: deltas_rel_deg 必须长度 4")
            obj2 = dict(obj)
            obj2["deltas_rel_deg"] = [float(arr[i]) for i in range(4)]
            return MotionCommand4Axis(kind="joints_delta", payload=obj2)
        if k in ("claw", "wrist", "gripper"):
            wrist = obj.get("wrist_deg")
            grip_state = obj.get("grip_state")
            open_close = obj.get("open_close")
            if wrist is None or (grip_state is None and open_close is None):
                raise ValueError("missing_field: claw 需要 wrist_deg + (grip_state/open_close)")
            payload = dict(obj)
            payload["wrist_deg"] = float(wrist)
            grip_src = grip_state if grip_state is not None else open_close
            payload["grip_state"] = cls._normalize_grip_state(grip_src)
            return ClawCommand(kind="claw", payload=payload)
        if k in ("stop", "estop", "halt"):
            return MotionCommand4Axis(kind="stop", payload=obj)
        if k in ("ping",):
            return MotionCommand4Axis(kind="ping", payload=obj)
        raise ValueError(f"unknown_cmd: {k}")

    @classmethod
    def normalize_line(cls, line: str) -> Optional[NormalizedCommand]:
        """解析单行 JSON；空行返回 ``None``。"""
        line = line.strip()
        if not line:
            return None
        try:
            obj = json.loads(line)
        except json.JSONDecodeError as e:
            raise ValueError(f"invalid_json: {e}") from e
        return cls.normalize_obj(obj)


class BaseListener:
    """命令入队基类。"""

    def __init__(
        self,
        command_queue: "queue.Queue[Optional[NormalizedCommand]]",
        on_log: LogFn = None,
    ) -> None:
        self._queue = command_queue
        self._on_log = on_log or (lambda s: None)

    def emit(self, cmd: NormalizedCommand) -> None:
        self._queue.put(cmd)

    def emit_from_obj(self, obj: dict[str, Any]) -> None:
        cmd = CommandNormalizer.normalize_obj(obj)
        self.emit(cmd)

    def emit_from_line(self, line: str) -> None:
        cmd = CommandNormalizer.normalize_line(line)
        if cmd is not None:
            self.emit(cmd)


class FrontendListener(BaseListener):
    """HTTP 测试页等入口使用的监听器。"""


class ClawListener(BaseListener):
    """仅将 ``claw`` 类命令写入独立队列。"""

    def emit(self, cmd: NormalizedCommand) -> None:
        if isinstance(cmd, ClawCommand):
            self._queue.put(cmd)
            return
        if cmd.kind == "claw":
            self._queue.put(ClawCommand(kind="claw", payload=cmd.payload))


class NetworkListener(BaseListener):
    """TCP ``accept`` + 每连接按行 JSON 解析。"""

    def __init__(
        self,
        host: str,
        port: int,
        command_queue: "queue.Queue[Optional[NormalizedCommand]]",
        on_log: LogFn = None,
        on_pending: PendingFn = None,
    ) -> None:
        super().__init__(command_queue, on_log=on_log)
        self._host = host
        self._port = port
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._on_pending = on_pending

    def start_background(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._host, self._port))
        self._sock.listen(8)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._on_log(f"[TCP] 监听 {self._host}:{self._port}（JSON 行协议）")

    def _run(self) -> None:
        assert self._sock is not None
        self._sock.settimeout(1.0)
        while not self._stop.is_set():
            try:
                conn, addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            t = threading.Thread(target=self._handle_client, args=(conn, addr), daemon=True)
            t.start()

    def _handle_client(self, conn: socket.socket, addr: tuple[Any, ...]) -> None:
        self._on_log(f"[TCP] 连接 {addr}")
        buf = b""
        try:
            conn.settimeout(300.0)
            while not self._stop.is_set():
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        cmd = CommandNormalizer.normalize_line(line.decode("utf-8"))
                        if cmd is not None:
                            if self._on_pending is not None:
                                self._on_pending(tcp_conn=conn, http_slot=None)
                            self.emit(cmd)
                    except ValueError as e:
                        self._on_log(f"[TCP] 忽略行: {e}")
        finally:
            try:
                conn.close()
            except OSError:
                pass
            self._on_log(f"[TCP] 断开 {addr}")

    def close(self) -> None:
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None


@dataclass
class Command:
    """遗留扁平结构（见 ``parse_line``）。"""

    kind: str
    payload: dict[str, Any]


def parse_line(line: str) -> Optional[Command]:
    """遗留 API：将一行 JSON 转为 ``Command``。"""
    cmd = CommandNormalizer.normalize_line(line)
    if cmd is None:
        return None
    return Command(kind=cmd.kind, payload=cmd.payload)


def handle_command_obj(cmd: Command) -> Command:
    """遗留 API：二次归一化（排除 claw）。"""
    normalized = CommandNormalizer.normalize_obj(dict(cmd.payload, cmd=cmd.kind))
    if not isinstance(normalized, MotionCommand4Axis):
        raise ValueError("unknown_cmd: claw 指令应走 claw_listener")
    return Command(kind=normalized.kind, payload=normalized.payload)


class TCPCommandServer(NetworkListener):
    """与 ``NetworkListener`` 等价的历史类型名。"""

    def __init__(
        self,
        host: str,
        port: int,
        command_queue: "queue.Queue[Optional[Command]]",
        on_log: LogFn = None,
    ) -> None:
        super().__init__(host=host, port=port, command_queue=command_queue, on_log=on_log)


def start_http_server(
    cmd_q: "queue.Queue[Any]",
    *,
    host: str,
    port: int,
    web_dir: str,
    on_log: Callable[[str], None],
    on_pending: PendingFn = None,
) -> threading.Thread:
    """在后台线程启动 ``HTTPServer``，路由见内部 ``Handler``。"""
    from ..config import REACHED_TIMEOUT_S

    frontend = FrontendListener(cmd_q, on_log=on_log)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: object) -> None:
            del fmt, args

        def _send_json(self, code: int, obj: dict[str, Any]) -> None:
            b = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_OPTIONS(self) -> None:
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self) -> None:
            if self.path.split("?")[0] in ("/", "/index.html"):
                path = os.path.join(web_dir, "index.html")
                if not os.path.isfile(path):
                    self._send_json(404, {"ok": False, "error": "no index.html"})
                    return
                with open(path, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self) -> None:
            path = self.path.split("?")[0]
            routes = {
                "/api/pose": "pose",
                "/api/pose_delta": "pose_delta",
                "/api/joints": "joints",
                "/api/joints_delta": "joints_delta",
                "/api/claw": "claw",
            }
            if path not in routes:
                self._send_json(404, {"ok": False})
                return
            n = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(n) if n else b"{}"
            try:
                obj = json.loads(body.decode("utf-8"))
            except json.JSONDecodeError:
                self._send_json(400, {"ok": False, "error": "invalid json"})
                return
            kind = routes[path]
            slot: Optional[ReplySlot] = None
            if on_pending is not None:
                slot = ReplySlot()
                on_pending(tcp_conn=None, http_slot=slot)
            try:
                frontend.emit_from_obj({"cmd": kind, **obj})
            except ValueError as e:
                self._send_json(400, {"ok": False, "error": str(e)})
                return
            on_log(f"[HTTP] recv {kind}: {obj}")
            if slot is not None:
                slot.event.wait(timeout=REACHED_TIMEOUT_S + 0.5)
                result = slot.result
                code = 200 if result.get("ok") else 408
                self._send_json(code, result)
            else:
                self._send_json(200, {"ok": True})

    def _run_http() -> None:
        httpd = HTTPServer((host, port), Handler)
        on_log(f"[HTTP] 服务 http://{host}:{port}/")
        httpd.serve_forever()

    t = threading.Thread(target=_run_http, daemon=True)
    t.start()
    return t


frontend_listener = FrontendListener
claw_listener = ClawListener
network_listener = NetworkListener
