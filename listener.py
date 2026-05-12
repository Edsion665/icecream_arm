"""UDP listener for PC joint stream."""

from __future__ import annotations

import logging
import socket
import threading
from typing import Optional

from .config import UdpConfig
from .infra.udp.packet import UDP_PACKET_SIZE, unpack_udp_packet
from .state_store import StateStore

LOGGER = logging.getLogger(__name__)

class UdpListener:
    """Receive UDP packets and refresh state_store register."""

    def __init__(self, cfg: UdpConfig, state_store: StateStore) -> None:
        self._cfg = cfg
        self._store = state_store
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self) -> None:
        if not self._cfg.enabled:
            LOGGER.info("UDP listener disabled by config")
            return
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="UdpListener")
        self._thread.start()
        LOGGER.info("UDP listener started on %s:%d", self._cfg.host, self._cfg.port)

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((self._cfg.host, self._cfg.port))
        except OSError as exc:
            LOGGER.error("UDP bind failed on %s:%d: %s", self._cfg.host, self._cfg.port, exc)
            return
        sock.settimeout(0.5)

        while not self._stop.is_set():
            try:
                data, _ = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    break
                continue

            if len(data) != UDP_PACKET_SIZE:
                continue

            try:
                pkt = unpack_udp_packet(data)
            except Exception:
                continue

            self._store.update_udp(
                seq=int(pkt["seq"]),
                p_rel_deg=tuple(pkt["p_rel_deg"]),  # type: ignore[arg-type]
                omega_rad_s=tuple(pkt["omega_rad_s"]),  # type: ignore[arg-type]
            )

        sock.close()

