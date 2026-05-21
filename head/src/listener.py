from __future__ import annotations

import json
import socket
import socketserver
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Tuple

from src.config import Settings
from src.coordinates import apply_role_z_offset
from src.models import DetectionFrame, ObjectDet, Position, Role
from src.tracker import Tracker

_VALID_FRAMES = frozenset({"robot_base", "camera_optical"})
_VALID_ROLES = frozenset({"object", "target", "lid"})


def parse_detection_payload(
    body: Dict[str, Any],
    implied_detection: bool,
    settings: Settings | None = None,
    tracker: Tracker | None = None,
) -> Tuple[bool, str | None, DetectionFrame | None]:
    cmd = body.get("cmd") or body.get("type")
    if implied_detection:
        if cmd is not None and str(cmd).lower() not in ("detection", "objects", "perception"):
            return False, f"unknown_cmd: {cmd}", None
    else:
        if cmd is None:
            return False, "missing_field: 缺少字段 cmd 或 type", None
        if str(cmd).lower() not in ("detection", "objects", "perception"):
            return False, f"unknown_cmd: {cmd}", None

    frame = body.get("frame")
    if not isinstance(frame, str) or frame not in _VALID_FRAMES:
        return False, "invalid_value: frame 不支持", None

    objs = body.get("objects")
    if not isinstance(objs, list):
        return False, "missing_field: detection 需要 objects 数组", None

    out: list[ObjectDet] = []
    for i, o in enumerate(objs):
        if not isinstance(o, dict):
            return False, f"invalid objects[{i}]", None
        role = o.get("role")
        if role not in _VALID_ROLES:
            return False, "invalid_value: role 必须为 object|target|lid", None
        r: Role = role  # type: ignore[assignment]
        if "class_id" not in o or "label" not in o:
            return False, "missing_field: objects[i] 缺少必选字段", None
        pos = o.get("position")
        if not isinstance(pos, dict) or not all(k in pos for k in ("x", "y", "z")):
            return False, "missing_field: position 需要 x, y, z", None
        if "wrist_yaw_deg" not in o:
            return False, "missing_field: objects[i] 缺少必选字段", None
        try:
            p = Position.from_dict(pos)
            if settings is not None and (
                tracker is None or not tracker.is_role_z_offset_suppressed(r)
            ):
                p = apply_role_z_offset(p, r, settings)
            wy = float(o["wrist_yaw_deg"])
        except (TypeError, ValueError):
            return False, f"invalid_value: objects[{i}] numeric fields", None
        bbox = o.get("bbox_2d")
        if bbox is not None and not isinstance(bbox, dict):
            return False, f"invalid bbox_2d at {i}", None
        out.append(
            ObjectDet(
                role=r,
                class_id=o["class_id"],
                label=str(o["label"]),
                position=p,
                wrist_yaw_deg=wy,
                track_id=o.get("track_id"),
                confidence=float(o["confidence"]) if o.get("confidence") is not None else None,
                bbox_2d=bbox if isinstance(bbox, dict) else None,
            )
        )

    frame_id = body.get("frame_id")
    if frame_id is not None:
        try:
            frame_id = int(frame_id)
        except (TypeError, ValueError):
            frame_id = None

    det = DetectionFrame(
        frame=frame,
        objects=out,
        frame_id=frame_id,
        ts=body.get("ts"),
    )
    return True, None, det


def _json_response(handler: BaseHTTPRequestHandler, code: int, payload: Dict[str, Any]) -> None:
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(code)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(data)))
    handler.end_headers()
    handler.wfile.write(data)


def make_detection_handler_class(tracker: Tracker, settings: Settings | None = None):
    class DetectionHandler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
            return

        def do_POST(self) -> None:  # noqa: N802
            path = self.path.split("?", 1)[0].rstrip("/") or "/"
            implied = path in ("/api/detection", "/api/objects")
            if not implied:
                _json_response(self, 404, {"ok": False, "error": f"not found: {path}"})
                return
            length = int(self.headers.get("Content-Length", "0") or 0)
            raw = self.rfile.read(length) if length > 0 else b"{}"
            try:
                body = json.loads(raw.decode("utf-8"))
            except json.JSONDecodeError:
                _json_response(self, 400, {"ok": False, "error": "invalid json"})
                return
            if not isinstance(body, dict):
                _json_response(self, 400, {"ok": False, "error": "invalid json"})
                return

            ok, err, det = parse_detection_payload(
                body, implied_detection=implied, settings=settings, tracker=tracker
            )
            if not ok or det is None:
                _json_response(self, 400, {"ok": False, "error": err or "bad request"})
                return
            tracker.submit_frame(det)
            _json_response(self, 200, {"ok": True})

    return DetectionHandler


class IngestionServer:
    def __init__(self, host: str, port: int, tracker: Tracker, settings: Settings | None = None) -> None:
        self._host = host
        self._port = port
        self._tracker = tracker
        self._settings = settings
        self._http: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._tcp_thread: threading.Thread | None = None
        self._tcp_stop = threading.Event()

    def start_background(self, tcp_port: int | None = None) -> None:
        handler_cls = make_detection_handler_class(self._tracker, self._settings)
        self._http = ThreadingHTTPServer((self._host, self._port), handler_cls)
        self._thread = threading.Thread(target=self._http.serve_forever, name="ingest-http", daemon=True)
        self._thread.start()
        if tcp_port is not None:
            self._tcp_stop.clear()
            self._tcp_thread = threading.Thread(
                target=self._serve_tcp_lines,
                args=(self._host, tcp_port),
                name="ingest-tcp",
                daemon=True,
            )
            self._tcp_thread.start()

    def _serve_tcp_lines(self, host: str, port: int) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(5)
        srv.settimeout(0.5)
        while not self._tcp_stop.is_set():
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with conn:
                buf = b""
                while not self._tcp_stop.is_set():
                    chunk = conn.recv(4096)
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
                            continue
                        if not isinstance(body, dict):
                            continue
                        ok, _, det = parse_detection_payload(
                            body,
                            implied_detection=True,
                            settings=self._settings,
                            tracker=self._tracker,
                        )
                        if ok and det is not None:
                            self._tracker.submit_frame(det)
        try:
            srv.close()
        except OSError:
            pass

    def stop(self) -> None:
        self._tcp_stop.set()
        if self._http:
            self._http.shutdown()
            self._http.server_close()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._tcp_thread:
            self._tcp_thread.join(timeout=2.0)
