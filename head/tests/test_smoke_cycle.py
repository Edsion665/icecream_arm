from __future__ import annotations

import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import yaml

from src.config import load_settings
from src.listener import IngestionServer
from src.run import HeadFSM
from src.speaker import BridgeClient
from src.tracker import Tracker


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    _, p = s.getsockname()
    s.close()
    return int(p)


class _Bridge(BaseHTTPRequestHandler):
    def log_message(self, *args: object) -> None:
        return

    def do_POST(self) -> None:  # noqa: N802
        n = int(self.headers.get("Content-Length", "0") or 0)
        _ = self.rfile.read(n) if n else b"{}"
        path = self.path.split("?", 1)[0]
        if path == "/api/joints":
            body = {"ok": True}
        elif path == "/api/pose":
            body = {"ok": True, "reached": True, "actual_pose": {"x": 0.1, "y": 0.2, "z": 0.3}}
        elif path == "/api/claw":
            body = {"ok": True}
        else:
            self.send_error(404)
            return
        raw = json.dumps(body).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)


def test_one_cycle_smoke(tmp_path: Path) -> None:
    import urllib.request
    from collections import deque

    bridge = HTTPServer(("127.0.0.1", 0), _Bridge)
    bthread = threading.Thread(target=bridge.serve_forever, daemon=True)
    bthread.start()
    _, bport = bridge.server_address  # type: ignore[misc]

    ingest_port = _free_port()
    cfg = {
        "bridge_base_url": f"http://127.0.0.1:{bport}",
        "ingest_host": "127.0.0.1",
        "ingest_port": ingest_port,
        "observe1_axes_rel_deg": [0, 0, 0, 0],
        "observe2_axes_rel_deg": [1, 0, 0, 0],
        "refresh_wait_s": 2.0,
        "ingest_tcp_port": None,
        # 单测只 POST 一次 detection；若 true 会在 obs 后清空 _last_frame 导致 wait 死等第二帧
        "require_fresh_detection_after_obs": False,
    }
    cfg_path = tmp_path / "cfg.yaml"
    cfg_path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
    settings = load_settings(cfg_path)

    tracker = Tracker(settings)
    tracker.start()
    ingest_srv = IngestionServer(settings.ingest_host, settings.ingest_port, tracker)
    ingest_srv.start_background(tcp_port=None)

    det = {
        "frame": "robot_base",
        "objects": [
            {
                "role": "object",
                "class_id": 1,
                "label": "cone",
                "position": {"x": 0.4, "y": 0.0, "z": 0.1},
                "wrist_yaw_deg": 5.0,
            },
            {
                "role": "target",
                "class_id": 2,
                "label": "slot",
                "position": {"x": 0.5, "y": 0.0, "z": 0.12},
                "wrist_yaw_deg": 0.0,
            },
        ],
    }
    data = json.dumps(det).encode("utf-8")
    req = urllib.request.Request(
        f"http://127.0.0.1:{ingest_port}/api/detection",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=2.0) as resp:
        assert json.loads(resp.read().decode())["ok"] is True

    spk = BridgeClient(settings)
    fsm = HeadFSM(settings, tracker, spk, deque())
    fsm.run_one_cycle()

    ingest_srv.stop()
    tracker.stop()
    bridge.shutdown()
    bridge.server_close()
    bthread.join(timeout=1.0)
