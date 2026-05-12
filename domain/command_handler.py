"""Handle WS command payloads against state port."""

from __future__ import annotations

import json
import logging
import math

from .ports import StatePort

LOGGER = logging.getLogger(__name__)


class StateCommandHandler:
    def __init__(self, store: StatePort) -> None:
        self._store = store

    def handle_raw(self, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return
        if msg.get("type") != "command":
            return

        cmd = str(msg.get("cmd", ""))
        data = msg.get("data", {})
        if cmd == "set_joint":
            self._store.set_servo_command({"cmd": cmd, "data": data})
            return

        if cmd in ("set_tau_calibration", "set_calibration"):
            self._handle_set_calibration(data)
            return

        if cmd == "adjust_calibration_deg":
            self._handle_adjust_calibration(data)

    def _handle_set_calibration(self, data: object) -> None:
        try:
            assert isinstance(data, dict)
            self._store.set_calibration_rad(
                float(data["r0"]),
                float(data["r1"]),
                float(data["r2"]),
                float(data["r3"]),
            )
            LOGGER.info("calibration set (rad): %s", list(self._store.get_calibration_rad()))
        except (AssertionError, KeyError, TypeError, ValueError) as exc:
            LOGGER.warning("set_calibration ignored: %s", exc)

    def _handle_adjust_calibration(self, data: object) -> None:
        try:
            assert isinstance(data, dict)
            self._store.adjust_calibration_deg(
                float(data["d0"]),
                float(data["d1"]),
                float(data["d2"]),
                float(data["d3"]),
            )
            LOGGER.info(
                "calibration after adjust (deg): %s",
                [round(math.degrees(x), 6) for x in self._store.get_calibration_rad()],
            )
        except (AssertionError, KeyError, TypeError, ValueError) as exc:
            LOGGER.warning("adjust_calibration_deg ignored: %s", exc)
