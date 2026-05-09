#!/usr/bin/env python3
"""Minimal HTTP bridge for local curl / head FSM tests (not dynamics)."""

from __future__ import annotations

import argparse
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Dict


class H(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args: Any) -> None:  # noqa: A003
        print(fmt % args)

    def _read_json(self) -> Dict[str, Any]:
        n = int(self.headers.get("Content-Length", "0") or 0)
        raw = self.rfile.read(n) if n else b"{}"
        try:
            o = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            return {}
        return o if isinstance(o, dict) else {}

    def _reply(self, code: int, body: Dict[str, Any]) -> None:
        data = json.dumps(body).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        _ = self._read_json()
        if path == "/api/joints":
            self._reply(200, {"ok": True})
        elif path == "/api/pose":
            self._reply(
                200,
                {
                    "ok": True,
                    "reached": True,
                    "actual_pose": {"x": 0.35, "y": 0.2, "z": 0.25},
                    "error_pose_m": 0.001,
                },
            )
        elif path == "/api/claw":
            self._reply(200, {"ok": True})
        else:
            self._reply(404, {"ok": False, "error": "not found"})


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8775)
    args = ap.parse_args()
    srv = HTTPServer((args.host, args.port), H)
    print(f"mock bridge http://{args.host}:{args.port}/")
    srv.serve_forever()


if __name__ == "__main__":
    main()
