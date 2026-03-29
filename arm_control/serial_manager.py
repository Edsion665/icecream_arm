"""串口收发管理：在独立线程中运行，负责与 STM32 通信。"""

from __future__ import annotations

import logging
import re
import threading
import time
from queue import Queue, Empty
from typing import Callable, Optional

import serial  # type: ignore[import]

from .config import CONFIG
from .protocol import FrameParser, ParsedFrame, pack_frame
from .state_store import StateStore

LOGGER = logging.getLogger(__name__)
FB_JOINT_PATTERN = re.compile(r"j(\d+):([+-]?\d+(?:\.\d+)?)")


OnFrameCallback = Callable[[ParsedFrame], None]


class SerialManager(threading.Thread):
    """串口管理线程。

    - 负责自动重连
    - 负责读串口并使用 FrameParser 解析
    - 写入通过线程安全队列实现
    """

    def __init__(
        self,
        state_store: StateStore,
        on_frame: Optional[OnFrameCallback] = None,
        daemon: bool = True,
    ) -> None:
        super().__init__(daemon=daemon, name="SerialManager")
        self._cfg = CONFIG.serial
        self._state_store = state_store
        self._on_frame = on_frame
        self._tx_queue: "Queue[bytes]" = Queue()
        self._stop_event = threading.Event()
        self._parser = FrameParser()
        self._ser: Optional[serial.Serial] = None
        self._rx_line_buffer = bytearray()
        self._ready_event = threading.Event()
        self._res_event = threading.Event()
        self._reconnect_event = threading.Event()
        # 记录握手失败次数序号，用于判断“重连期间是否握手失败过多”
        self._handshake_fail_seq = 0

    def stop(self) -> None:
        self._stop_event.set()

    def send_command(self, cmd: int, payload: bytes) -> None:
        frame = pack_frame(cmd, payload)
        LOGGER.info("发送二进制帧：cmd=0x%02X, len=%d", cmd, len(payload))
        self._tx_queue.put(frame)

    def send_raw(self, data: bytes) -> None:
        """直接发送原始字节（用于文本协议等场景）。"""
        LOGGER.info("发送原始串口数据：%r", data)
        self._tx_queue.put(data)

    def send_raw_and_wait_for_res(self, data: bytes) -> bool:
        """发送 raw 指令，并等待收到下一条 RES 回传文本。

        超时视为 STM32 断链：触发重新握手/重连，并重发一次（或多次）。
        """
        attempts = max(1, self._cfg.res_retry_count + 1)
        timeout = float(self._cfg.res_wait_timeout_sec)
        expect_prefix = self._cfg.res_expect_prefix

        for attempt_idx in range(attempts):
            self._res_event.clear()
            self.send_raw(data)
            LOGGER.info("等待回传：prefix=%r (attempt %d/%d)", expect_prefix, attempt_idx + 1, attempts)

            if self._res_event.wait(timeout=timeout):
                return True

            LOGGER.warning("未收到 %r 回传（%.2fs），触发重连并重发", expect_prefix, timeout)
            # 记录此次“等待 RES 失败后”重连过程中握手失败次数
            handshake_fail_seq_start = self._handshake_fail_seq
            self._request_reconnect()

            ready_timeout = float(self._cfg.reconnect_wait_timeout_sec)
            if not self._ready_event.wait(timeout=ready_timeout):
                LOGGER.error("重连等待超时（%.2fs），放弃本次命令", ready_timeout)
                return False

            handshake_fail_seq_delta = self._handshake_fail_seq - handshake_fail_seq_start
            if handshake_fail_seq_delta >= int(self._cfg.res_handshake_fail_stop_threshold):
                LOGGER.error(
                    "重连期间握手连续失败次数达到阈值（delta=%d >= %d），"
                    "即使握手成功也不再发送 DATA",
                    handshake_fail_seq_delta,
                    int(self._cfg.res_handshake_fail_stop_threshold),
                )
                return False

        return False

    # ---- 内部方法 ---------------------------------------------------------

    def _open_serial(self) -> Optional[serial.Serial]:
        try:
            LOGGER.info("尝试打开串口 %s", self._cfg.port)
            ser = serial.Serial(
                port=self._cfg.port,
                baudrate=self._cfg.baudrate,
                bytesize=self._cfg.bytesize,
                stopbits=self._cfg.stopbits,
                parity=self._cfg.parity,
                timeout=0.05,
            )
            LOGGER.info("串口已打开，准备握手检查")

            if self._cfg.handshake_enabled:
                if not self._do_handshake(ser):
                    self._handshake_fail_seq += 1
                    LOGGER.error("握手失败，关闭串口")
                    try:
                        ser.close()
                    except Exception as exc:  # noqa: BLE001
                        LOGGER.error("关闭串口失败（握手失败后）：%s", exc)
                    return None

            LOGGER.info("串口握手完成，连接就绪")
            self._ready_event.set()
            return ser
        except serial.SerialException as exc:  # type: ignore[attr-defined]
            LOGGER.error("打开串口失败：%s", exc)
            return None

    def _close_serial(self) -> None:
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
                LOGGER.info("串口已关闭")
            except Exception as exc:  # noqa: BLE001
                LOGGER.error("关闭串口失败：%s", exc)
        self._ser = None
        self._ready_event.clear()

    def _request_reconnect(self) -> None:
        """请求串口线程关闭并重新建立连接（内部会触发握手）。"""
        # 立即清除 ready，避免发送线程误判“握手成功”
        self._ready_event.clear()
        self._res_event.clear()
        self._reconnect_event.set()

    # ---- 握手流程 ---------------------------------------------------------

    def _do_handshake(self, ser: serial.Serial) -> bool:
        """与 STM32 进行一次握手：发送 start，等待 ok。"""
        req = self._cfg.handshake_request.encode("utf-8")
        expect = self._cfg.handshake_expect.lower()
        deadline = time.time() + self._cfg.handshake_timeout_sec

        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("重置串口缓冲失败（握手前）：%s", exc)

        LOGGER.info("开始握手：发送 %r，等待回复包含 %r", req, expect)
        try:
            ser.write(req)
            ser.flush()
        except Exception as exc:  # noqa: BLE001
            LOGGER.error("握手发送失败：%s", exc)
            return False

        buf = b""
        while time.time() < deadline:
            try:
                chunk = ser.read(64)
            except Exception as exc:  # noqa: BLE001
                LOGGER.error("握手读取失败：%s", exc)
                return False

            if chunk:
                buf += chunk
                # 按行拆分，逐行判断是否包含期望关键字
                while b"\n" in buf:
                    line, _, rest = buf.partition(b"\n")
                    buf = rest
                    text = line.decode(errors="ignore").strip().lower()
                    LOGGER.info("握手收到行：%r", text)
                    if expect in text:
                        LOGGER.info("握手成功，收到期望回复")
                        return True
            else:
                time.sleep(0.01)

        LOGGER.error(
            "握手超时（%.2fs 内未收到包含 %r 的回复）",
            self._cfg.handshake_timeout_sec,
            expect,
        )
        return False

    # ---- 主线程循环 ------------------------------------------------------

    def _try_handle_data_line(self, text: str) -> None:
        """
        解析文本协议：DATA:a0,a1,a2,a3,a4,a5*XX

        若校验通过，则把 a0..a5 作为舵机角度写入 state_store（单位：度×100 -> 度）。
        """
        if "*" not in text:
            return

        star_idx = text.rfind("*")
        body = text[:star_idx]
        cs_hex = text[star_idx + 1 :].strip()
        if not body.upper().startswith("DATA:"):
            return

        try:
            cs_recv = int(cs_hex, 16)
        except ValueError:
            return

        calc = 0
        for b in body.encode("ascii", errors="ignore"):
            calc ^= b

        if calc != cs_recv:
            self._state_store.inc_crc_error()
            LOGGER.warning("[RX DATA] checksum fail body=%r recv=%r calc=%r", body, cs_hex, f"{calc:02X}")
            return

        # body: "DATA:a0,a1,a2,a3,a4,a5"
        _, nums_str = body.split(":", 1)
        parts = [p.strip() for p in nums_str.split(",")]
        if len(parts) != 6:
            return

        try:
            vals = [int(p) for p in parts]
        except ValueError:
            return

        for idx, raw_val in enumerate(vals):
            # 协议定义的单位为 度×100，这里换算为度
            self._state_store.update_servo(idx, angle=float(raw_val) / 100.0)

    def _try_handle_res_line(self, text: str) -> None:
        """解析 STM32 回传 RES 行，并更新状态。

        期望格式示例：
        RES world_rel_rad: 0.0175 0.0349 0.0524 0.0698 target_abs: 3.1515 1.4739 2.5354 1.5478
        """
        if not text.upper().startswith(self._cfg.res_expect_prefix.upper()):
            return

        self._res_event.set()

        if "world_rel_rad" not in text or "target_abs" not in text:
            return

        try:
            _, rest = text.split("world_rel_rad:", 1)
            rel_part, abs_part = rest.split("target_abs:", 1)
            rel_vals = [float(x) for x in rel_part.strip().split()]
            abs_vals = [float(x) for x in abs_part.strip().split()]
        except Exception:
            # 只要能触发握手/重连判断即可，不强依赖解析成功
            return

        # 这里按最常见的 4 个通道回传做映射：motor_id 0..3
        for i in range(min(4, len(abs_vals))):
            self._state_store.update_motor(i, position=abs_vals[i])
        for i in range(min(4, len(rel_vals))):
            self._state_store.update_motor(i, velocity=rel_vals[i])

    def _try_handle_fb_line(self, text: str) -> None:
        """解析 STM32 回传 FB 行中的 j1..jN，并写入 joints 状态。"""
        if not text.startswith("FB "):
            return

        # 新协议：FB 后 4 个浮点弧度（可含尾部 j1:...）
        # 示例: FB 3.16262 -0.20504 -0.77687 0.02613
        # 旧协议：FB 后 4 个整数（度×100 等）仍兼容：用 float 解析即可
        try:
            head_part = text.split(" j1:", 1)[0]
            parts = head_part.split()
            if len(parts) >= 5:
                raw4 = [float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])]
                self._state_store.set_fb_arm_rad((raw4[0], raw4[1], raw4[2], raw4[3]))
                for i, value in enumerate(raw4):
                    self._state_store.update_motor(i, position=float(value))
        except (ValueError, TypeError):
            pass

        matches = FB_JOINT_PATTERN.findall(text)
        if not matches:
            return

        for joint_idx_str, angle_str in matches:
            try:
                joint_id = int(joint_idx_str)
                angle = float(angle_str)
            except ValueError:
                continue
            self._state_store.update_joint(joint_id, angle=angle)

    def _log_text_lines(self, data: bytes) -> None:
        """按行提取串口文本并打印，便于联调 STM32 回传。"""
        self._rx_line_buffer.extend(data)
        while b"\n" in self._rx_line_buffer:
            line_raw, _, rest = self._rx_line_buffer.partition(b"\n")
            self._rx_line_buffer = bytearray(rest)
            text = line_raw.decode(errors="ignore").strip()
            if text:
                LOGGER.info("[RX text] %s", text)
                self._try_handle_data_line(text)
                self._try_handle_res_line(text)
                self._try_handle_fb_line(text)

    def run(self) -> None:  # noqa: D401 - 线程入口
        """线程主循环。"""
        while not self._stop_event.is_set():
            if self._reconnect_event.is_set():
                self._reconnect_event.clear()
                self._close_serial()
                time.sleep(self._cfg.reconnect_interval_sec)
                continue

            if self._ser is None or not self._ser.is_open:
                self._ser = self._open_serial()
                if self._ser is None:
                    time.sleep(self._cfg.reconnect_interval_sec)
                    continue

            try:
                self._io_step()
            except serial.SerialException as exc:  # type: ignore[attr-defined]
                LOGGER.error("串口异常，准备重连：%s", exc)
                self._close_serial()
                time.sleep(self._cfg.reconnect_interval_sec)
            except Exception as exc:  # noqa: BLE001
                LOGGER.error("串口线程未知异常：%s", exc)
                self._close_serial()
                time.sleep(self._cfg.reconnect_interval_sec)

        self._close_serial()

    def _io_step(self) -> None:
        assert self._ser is not None

        # 读取数据
        data = self._ser.read(256)
        if data:
            # STM32 当前主要用文本协议，先按文本行打印回传
            self._log_text_lines(data)

            # 兼容保留：仍尝试按旧二进制协议解帧
            frames = self._parser.feed(data)
            for frame in frames:
                if self._on_frame:
                    self._on_frame(frame)

        # 发送队列中的数据
        try:
            frame = self._tx_queue.get_nowait()
        except Empty:
            return

        self._ser.write(frame)
        self._tx_queue.task_done()

