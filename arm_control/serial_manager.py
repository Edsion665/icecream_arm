"""串口收发管理：在独立线程中运行，负责与 STM32 通信。"""

from __future__ import annotations

import logging
import re
import threading
import time
from queue import Queue, Empty
from typing import Any, Callable, Optional

import serial  # type: ignore[import]

from .config import CONFIG, MIT_UPLINK_MODE
from .mit_stm32_codec import (
    MIT_CMD_FRAME_LEN,
    UPLINK_FRAME_LEN,
    UPLINK_HEADER,
    build_mit_cmd_downlink_log_lines,
    decode_uplink,
    try_parse_uplink_hex_from_line,
)
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
        self._rx_stream_buf = bytearray()
        self._mit_mode = MIT_UPLINK_MODE
        self._ready_event = threading.Event()
        self._res_event = threading.Event()
        self._reconnect_event = threading.Event()
        # 记录握手失败次数序号，用于判断“重连期间是否握手失败过多”
        self._handshake_fail_seq = 0
        if self._mit_mode != "none":
            LOGGER.info(
                "MIT 上行反馈解码已启用：mode=%s（见 RPI_STM32_PROTOCOL.md；与 TAU/FB 独立）",
                self._mit_mode,
            )

    def stop(self) -> None:
        self._stop_event.set()

    def send_command(self, cmd: int, payload: bytes) -> None:
        frame = pack_frame(cmd, payload)
        LOGGER.info("发送二进制帧：cmd=0x%02X, len=%d", cmd, len(payload))
        self._tx_queue.put(frame)

    def send_raw(self, data: bytes) -> None:
        """直接发送原始字节（用于文本协议等场景）。"""
        self._log_tx_raw(data)
        self._tx_queue.put(data)

    def _log_tx_raw(self, data: bytes) -> None:
        """日志：MIT 35B 下行显示连续 HEX + 分组 HEX + p/v/kp/kd/t；文本协议显示可读字符串；其余显示 HEX。"""
        n = len(data)
        if (
            n == MIT_CMD_FRAME_LEN
            and data[0] == UPLINK_HEADER[0]
            and data[1] == UPLINK_HEADER[1]
        ):
            LOGGER.info(
                "发送 MIT 下行（35B，RPI_MIT_CMD_BINARY_ENCODE）\n%s",
                "\n".join(build_mit_cmd_downlink_log_lines(data)),
            )
            return
        try:
            text = data.decode("ascii")
        except UnicodeDecodeError:
            LOGGER.info(
                "发送原始串口数据：%d 字节，HEX=%s",
                n,
                data.hex().upper(),
            )
            return
        if all(32 <= ord(c) < 127 or c in "\r\n\t" for c in text):
            LOGGER.info("发送原始串口数据（ASCII 文本）: %r", text)
            return
        LOGGER.info(
            "发送原始串口数据：%d 字节，HEX=%s",
            n,
            data.hex().upper(),
        )

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
            LOGGER.info("串口已打开（不等待 start/ok 握手，直接就绪）")

            # 已注释：先发 ``start``、收到 ``ok`` 后才视为连接就绪（见 ``SerialConfig.handshake_*`` / ``_do_handshake``）
            # if self._cfg.handshake_enabled:
            #     if not self._do_handshake(ser):
            #         self._handshake_fail_seq += 1
            #         LOGGER.error("握手失败，关闭串口")
            #         try:
            #             ser.close()
            #         except Exception as exc:  # noqa: BLE001
            #             LOGGER.error("关闭串口失败（握手失败后）：%s", exc)
            #         return None

            LOGGER.info("串口连接就绪，可收发")
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
        self._rx_stream_buf.clear()
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

    def _log_mit_uplink_decode(self, raw: bytes, decoded: list[dict[str, Any]]) -> None:
        """打印原始 hex 码（连续大写 + 空格分组）与协议解码后的四电机信息。"""
        hex_compact = raw.hex().upper()
        lines: list[str] = [
            f"  原始上行(hex码): {hex_compact}",
            f"  原始上行(HEX): " + " ".join(f"{b:02X}" for b in raw),
        ]
        for i, m in enumerate(decoded):
            lines.append(
                f"  M{i + 1}: err={m['err']}  p={m['p']:.6f} rad  "
                f"v={m['v']:.6f} rad/s  t={m['t']:.6f} Nm  "
                f"mos_temp={m['mos_temp']}  rotor_temp={m['rotor_temp']}"
            )
        LOGGER.info("[MIT] 上行解码（34 字节帧，RPI_STM32_PROTOCOL）\n%s", "\n".join(lines))

    def _apply_mit_uplink(self, raw: bytes) -> None:
        """解析 34 字节 MIT 上行（0xAA 0x55 + 四电机×8B CAN 反馈），写入 motors（不改 FB/TAU 弧度源）。"""
        try:
            decoded = decode_uplink(raw)
        except ValueError as exc:
            LOGGER.warning("[MIT] 上行解码跳过：%s", exc)
            return
        for i, m in enumerate(decoded):
            self._state_store.update_motor(
                i,
                position=m["p"],
                velocity=m["v"],
                torque=m["t"],
                err=m["err"],
                mos_temp=m["mos_temp"],
                rotor_temp=m["rotor_temp"],
            )
        self._state_store.set_mit_arm_rad(
            (
                decoded[0]["p"],
                decoded[1]["p"],
                decoded[2]["p"],
                decoded[3]["p"],
            )
        )
        self._log_mit_uplink_decode(raw, decoded)

    def _dispatch_line(self, text: str) -> None:
        """一行文本：可能是 34 字节上行 hex 文本、DATA/RES/FB 等。"""
        if self._mit_mode != "none":
            raw = try_parse_uplink_hex_from_line(text)
            if raw is not None:
                self._apply_mit_uplink(raw)
                return

        if text:
            LOGGER.info("[RX text] %s", text)
        self._try_handle_data_line(text)
        self._try_handle_res_line(text)
        self._try_handle_fb_line(text)

    def _feed_rx_stream(self, data: bytes) -> None:
        """字节流：先按 0xAA 0x55 截取 34 字节 MIT 上行，其余再按 \\n 切文本（避免二进制与 T12 混成一行）。

        与 readline 不同：二进制帧内无 \\r\\n，必须在缓冲区中先定位帧头再读满 UPLINK_FRAME_LEN。
        """
        self._rx_stream_buf.extend(data)
        buf = self._rx_stream_buf
        while True:
            if not buf:
                break
            if MIT_UPLINK_MODE == "none":
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                text = bytes(buf[:nl]).decode(errors="ignore").strip("\r\n")
                del buf[: nl + 1]
                if text:
                    self._dispatch_line(text)
                continue

            p_aa = buf.find(b"\xaa\x55")
            if p_aa < 0:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                text = bytes(buf[:nl]).decode(errors="ignore").strip("\r\n")
                del buf[: nl + 1]
                if text:
                    self._dispatch_line(text)
                continue

            if p_aa > 0:
                nl = buf.find(b"\n", 0, p_aa)
                if nl >= 0:
                    text = bytes(buf[:nl]).decode(errors="ignore").strip("\r\n")
                    del buf[: nl + 1]
                    if text:
                        self._dispatch_line(text)
                    continue
                if len(buf) < p_aa + UPLINK_FRAME_LEN:
                    break
                LOGGER.debug(
                    "[MIT] 帧头前有 %d 字节且无换行（视为噪声/半包），丢弃至 0xAA 0x55",
                    p_aa,
                )
                del buf[:p_aa]
                continue

            if len(buf) < UPLINK_FRAME_LEN:
                break
            frame = bytes(buf[:UPLINK_FRAME_LEN])
            del buf[:UPLINK_FRAME_LEN]
            self._apply_mit_uplink(frame)
            continue

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
            # 文本行（FB/RES/DATA/TAU 等）+ 可选 MIT 上行（hex68 或 0xAA0x55 同步 34 字节二进制）
            self._feed_rx_stream(data)

            # 旧二进制帧与 MIT 上行均以 0xAA 0x55 开头；MIT 启用时勿并行解帧，否则会误当 len/cmd 并 CRC 报错
            if MIT_UPLINK_MODE == "none":
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

