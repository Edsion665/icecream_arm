"""Serial uplink/downlink manager for STM32 communication."""

from __future__ import annotations

import logging
import re
import threading
import time
from queue import Empty, Queue
from typing import Optional

import serial  # type: ignore[import]

from .config import CONFIG, SerialConfig
from .infra.serial.codec import (
    MIT_CMD_FRAME_LEN,
    UPLINK_FRAME_LEN,
    decode_mit_cmd_35,
    decode_mit_uplink,
    encode_mit_cmd_35,
    encode_mit_cmd_39,
)
from .state_store import StateStore

LOGGER = logging.getLogger(__name__)
FB_PATTERN = re.compile(r"^FB\s+([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)\s+([+-]?\d+(?:\.\d+)?)")

class SerialManager(threading.Thread):
    """Serial thread with reconnect and register updates."""

    def __init__(self, cfg: SerialConfig, state_store: StateStore) -> None:
        super().__init__(daemon=True, name="SerialManagerV2")
        self._cfg = cfg
        self._store = state_store
        self._ser: Optional[serial.Serial] = None
        self._stop_event = threading.Event()
        self._tx_queue: "Queue[bytes]" = Queue()
        self._rx_buf = bytearray()
        self._debug_lock = threading.Lock()
        self._last_tx_frame: bytes | None = None
        self._last_rx_frame: bytes | None = None
        self._last_rx_kind: str = "none"
        self._last_fb_log_mono: float = 0.0

    def _serial_feedback_fresh_for_log(self) -> bool:
        fb = self._store.snapshot_feedback()
        if fb.serial_feedback_mono <= 0.0:
            return False
        return time.monotonic() - fb.serial_feedback_mono <= CONFIG.control.serial_feedback_stale_sec

    def stop(self) -> None:
        self._stop_event.set()

    def send_raw(self, data: bytes) -> None:
        with self._debug_lock:
            self._last_tx_frame = bytes(data)
        if self._is_mit_cmd(data):
            self._drop_pending_mit_frames()
        self._tx_queue.put(data)

    def send_mit_cmd(self, motors: list[dict[str, float]]) -> None:
        self.send_raw(encode_mit_cmd_35(motors))

    def send_mit_cmd_with_servo(self, motors: list[dict[str, float]], wrist_us: int, gripper_us: int) -> None:
        self.send_raw(encode_mit_cmd_39(motors, wrist_us, gripper_us))

    @staticmethod
    def _is_mit_cmd(data: bytes) -> bool:
        return len(data) in (MIT_CMD_FRAME_LEN, 39) and data[0] == 0xAA and data[1] == 0x55

    def _drop_pending_mit_frames(self) -> None:
        pending: list[bytes] = []
        while True:
            try:
                pending.append(self._tx_queue.get_nowait())
            except Empty:
                break
        for item in pending:
            if self._is_mit_cmd(item):
                self._tx_queue.task_done()
                continue
            self._tx_queue.put(item)

    def snapshot_debug_frames(self) -> dict[str, str | None]:
        tx_decoded = None
        tx_xor_ok = None
        rx_decoded = None
        with self._debug_lock:
            if self._last_tx_frame is not None:
                try:
                    motors, xor_ok = decode_mit_cmd_35(self._last_tx_frame)
                    tx_decoded = str(motors)
                    tx_xor_ok = str(xor_ok)
                except ValueError:
                    tx_decoded = None
                    tx_xor_ok = None
            if self._last_rx_frame is not None and self._last_rx_kind == "mit39":
                try:
                    motors_d, servo_d = decode_mit_uplink(self._last_rx_frame)
                    rx_decoded = str({"motors": motors_d, "servo": servo_d})
                except ValueError:
                    rx_decoded = None
            return {
                "tx_hex": self._last_tx_frame.hex().upper() if self._last_tx_frame else None,
                "tx_decoded": tx_decoded,
                "tx_xor_ok": tx_xor_ok,
                "rx_kind": self._last_rx_kind,
                "rx_hex": self._last_rx_frame.hex().upper() if self._last_rx_frame else None,
                "rx_decoded": rx_decoded,
            }

    def run(self) -> None:
        while not self._stop_event.is_set():
            if self._ser is None or not self._ser.is_open:
                self._ser = self._open()
                if self._ser is None:
                    time.sleep(self._cfg.reconnect_interval_sec)
                    continue
            try:
                self._io_step()
            except Exception as exc:  # noqa: BLE001
                LOGGER.error("serial io error: %s", exc)
                self._close()
                time.sleep(self._cfg.reconnect_interval_sec)
        self._close()

    def _open(self) -> Optional[serial.Serial]:
        try:
            LOGGER.info("opening serial %s @ %d", self._cfg.port, self._cfg.baudrate)
            return serial.Serial(
                port=self._cfg.port,
                baudrate=self._cfg.baudrate,
                bytesize=8,
                stopbits=1,
                parity="N",
                timeout=0.01,
            )
        except serial.SerialException as exc:  # type: ignore[attr-defined]
            LOGGER.error("open serial failed: %s", exc)
            return None

    def _close(self) -> None:
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
        self._ser = None
        self._rx_buf.clear()

    def _io_step(self) -> None:
        assert self._ser is not None
        data = self._ser.read(256)
        if data:
            self._feed_rx(data)
        try:
            frame = self._tx_queue.get_nowait()
        except Empty:
            return
        self._ser.write(frame)
        self._tx_queue.task_done()

    def _feed_rx(self, data: bytes) -> None:
        self._rx_buf.extend(data)
        while True:
            if not self._rx_buf:
                return
            p = self._rx_buf.find(b"\xaa\x55")
            if p == 0 and len(self._rx_buf) >= UPLINK_FRAME_LEN:
                raw = bytes(self._rx_buf[:UPLINK_FRAME_LEN])
                del self._rx_buf[:UPLINK_FRAME_LEN]
                self._handle_mit(raw)
                continue
            if p > 0:
                nl = self._rx_buf.find(b"\n", 0, p)
                if nl >= 0:
                    line = bytes(self._rx_buf[:nl]).decode(errors="ignore").strip()
                    del self._rx_buf[: nl + 1]
                    self._handle_line(line)
                    continue
                del self._rx_buf[:p]
                continue
            nl = self._rx_buf.find(b"\n")
            if nl < 0:
                return
            line = bytes(self._rx_buf[:nl]).decode(errors="ignore").strip()
            del self._rx_buf[: nl + 1]
            self._handle_line(line)

    def _handle_mit(self, raw: bytes) -> None:
        with self._debug_lock:
            self._last_rx_kind = "mit39"
            self._last_rx_frame = bytes(raw)
        try:
            motors, servo = decode_mit_uplink(raw)
        except ValueError:
            return
        for i, m in enumerate(motors):
            self._store.update_mit_feedback(i, m)
        self._store.touch_serial_feedback_recv()
        now = time.monotonic()
        if now - self._last_fb_log_mono >= 1.0:
            self._last_fb_log_mono = now
            if self._serial_feedback_fresh_for_log():
                LOGGER.info(
                    "[fb] p=[%.4f %.4f %.4f %.4f] t=[%.4f %.4f %.4f %.4f] wrist=%dus gripper=%dus",
                    motors[0]["p"], motors[1]["p"], motors[2]["p"], motors[3]["p"],
                    motors[0]["t"], motors[1]["t"], motors[2]["t"], motors[3]["t"],
                    servo["wrist_us"], servo["gripper_us"],
                )

    def _handle_line(self, line: str) -> None:
        if not line:
            return
        with self._debug_lock:
            self._last_rx_kind = "text"
            self._last_rx_frame = line.encode("utf-8", errors="ignore")
        m = FB_PATTERN.match(line)
        if m:
            self._store.set_fb_arm_rad(tuple(float(m.group(i)) for i in range(1, 5)))

