"""TCP JSON-line client for pi2head (Pi -> head)."""

from __future__ import annotations

import json
import logging
import socket
from typing import Any

LOGGER = logging.getLogger(__name__)


def _read_json_line(sock: socket.socket, timeout_sec: float) -> dict[str, Any]:
    sock.settimeout(timeout_sec)
    line = b""
    while b"\n" not in line:
        chunk = sock.recv(4096)
        if not chunk:
            break
        line += chunk
    text = line.decode("utf-8").strip()
    if not text:
        raise ValueError("empty response from head")
    return json.loads(text)


def send_pi2head_start(
    head_host: str,
    port: int = 8778,
    *,
    connect_timeout_sec: float = 5.0,
    read_timeout_sec: float = 5.0,
) -> dict[str, Any]:
    """Send ``{"cmd":"start"}`` per ``docs/pi2head.md`` §4.2."""
    msg = json.dumps({"cmd": "start"}) + "\n"
    LOGGER.info("pi2head start -> %s:%d", head_host, port)
    with socket.create_connection((head_host, port), timeout=connect_timeout_sec) as sock:
        sock.sendall(msg.encode("utf-8"))
        rep = _read_json_line(sock, read_timeout_sec)
    if not rep.get("ok"):
        raise RuntimeError(f"pi2head start rejected: {rep!r}")
    LOGGER.info(
        "pi2head start ok started=%s already_started=%s",
        rep.get("started"),
        rep.get("already_started"),
    )
    return rep
