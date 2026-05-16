from __future__ import annotations

import json
import socket
import threading
import time

from src.config import Settings
from src.pi2head_listener import Pi2HeadServer, parse_pi2head_payload


def test_parse_start_and_ping() -> None:
    ok, err, cmd = parse_pi2head_payload({"cmd": "start"})
    assert ok and err is None and cmd == "start"
    ok, err, cmd = parse_pi2head_payload({"type": "PING"})
    assert ok and cmd == "ping"
    ok, err, _ = parse_pi2head_payload({"cmd": "nope"})
    assert not ok and err and "unknown_cmd" in err


def test_pi2head_tcp_start() -> None:
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    srv = Pi2HeadServer("127.0.0.1", port)
    srv.start_background()
    time.sleep(0.15)
    assert not srv.is_started()
    with socket.create_connection(("127.0.0.1", port), timeout=2.0) as c:
        c.sendall(b'{"cmd":"start"}\n')
        line = b""
        while b"\n" not in line:
            line += c.recv(4096)
        body = json.loads(line.decode().strip())
        assert body.get("ok") is True
    assert srv.wait_started(timeout=1.0)
    srv.stop()
