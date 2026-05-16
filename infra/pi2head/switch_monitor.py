"""Hardware switch loopback on ttyAMA4 -> pi2head start."""

from __future__ import annotations

import json
import logging
import threading
import time
from typing import TYPE_CHECKING, Optional

import serial  # type: ignore[import]

from .client import send_pi2head_start

if TYPE_CHECKING:
    from ...config import SwitchGateConfig

LOGGER = logging.getLogger(__name__)

_PROBE = b"ICECREAM_SWITCH?\r\n"
_SERIAL_READ_TIMEOUT = 0.1
_RECONNECT_SEC = 2.0


class SwitchGateMonitor(threading.Thread):
    """开关闭合（TX-RX 环路）时向 head 发送 ``start``。"""

    def __init__(self, cfg: "SwitchGateConfig") -> None:
        super().__init__(daemon=True, name="SwitchGateMonitor")
        self._cfg = cfg
        self._stop_event = threading.Event()
        self._was_closed = False

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        LOGGER.info(
            "switch gate %s @ %d -> pi2head %s:%d (poll %.1fs)",
            self._cfg.port,
            self._cfg.baudrate,
            self._cfg.head_host,
            self._cfg.head_port,
            self._cfg.poll_interval_sec,
        )
        while not self._stop_event.is_set():
            try:
                with serial.Serial(
                    port=self._cfg.port,
                    baudrate=self._cfg.baudrate,
                    timeout=_SERIAL_READ_TIMEOUT,
                ) as ser:
                    self._poll_loop(ser)
            except serial.SerialException as exc:  # type: ignore[attr-defined]
                LOGGER.warning("switch serial: %s", exc)
            if self._stop_event.wait(_RECONNECT_SEC):
                break

    def _poll_loop(self, ser: serial.Serial) -> None:
        while not self._stop_event.is_set():
            closed = self._loopback_closed(ser)
            if closed and not self._was_closed:
                try:
                    send_pi2head_start(self._cfg.head_host, self._cfg.head_port)
                except OSError as exc:
                    LOGGER.error("pi2head start (network): %s", exc)
                except (ValueError, RuntimeError, json.JSONDecodeError) as exc:
                    LOGGER.error("pi2head start: %s", exc)
            self._was_closed = closed
            if self._stop_event.wait(self._cfg.poll_interval_sec):
                break

    @staticmethod
    def _loopback_closed(ser: serial.Serial) -> bool:
        ser.reset_input_buffer()
        ser.write(_PROBE)
        ser.flush()
        time.sleep(0.02)
        return _PROBE in ser.read(len(_PROBE) + 32)
