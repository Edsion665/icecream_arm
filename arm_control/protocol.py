"""串口帧协议：封帧、解帧、CRC16 与有限状态机解析。"""

from __future__ import annotations

import enum
import logging
import struct
from dataclasses import dataclass
from typing import List, Tuple

LOGGER = logging.getLogger(__name__)

FRAME_HEAD1 = 0xAA
FRAME_HEAD2 = 0x55
MAX_PAYLOAD_LEN = 250


def crc16(data: bytes, poly: int = 0xA001, init_value: int = 0xFFFF) -> int:
    """计算 CRC16 (Modbus 风格)，结果为 0~0xFFFF。

    校验范围：len + cmd + payload。
    """
    crc = init_value
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
    return crc & 0xFFFF


def pack_frame(cmd: int, payload: bytes) -> bytes:
    """根据协议封装一帧二进制数据。"""
    if not 0 <= cmd <= 0xFF:
        raise ValueError("cmd 必须在 0~255 之间")
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError(f"payload 过长，最大 {MAX_PAYLOAD_LEN} 字节")

    length = len(payload)
    header = struct.pack("<BBB", FRAME_HEAD1, FRAME_HEAD2, length)
    body = struct.pack("<B", cmd) + payload
    crc_val = crc16(bytes([length, cmd]) + payload)
    crc_bytes = struct.pack("<H", crc_val)
    return header + body + crc_bytes


@dataclass
class ParsedFrame:
    cmd: int
    payload: bytes


class ParseState(enum.Enum):
    WAIT_HEAD1 = enum.auto()
    WAIT_HEAD2 = enum.auto()
    WAIT_LEN = enum.auto()
    WAIT_CMD = enum.auto()
    WAIT_DATA = enum.auto()
    WAIT_CRC1 = enum.auto()
    WAIT_CRC2 = enum.auto()


class FrameParser:
    """有限状态机逐字节解析串口帧。"""

    def __init__(self) -> None:
        self.state = ParseState.WAIT_HEAD1
        self.length = 0
        self.cmd = 0
        self._payload_bytes: bytearray = bytearray()
        self._crc_low = 0

    def reset(self) -> None:
        self.state = ParseState.WAIT_HEAD1
        self.length = 0
        self.cmd = 0
        self._payload_bytes.clear()
        self._crc_low = 0

    def feed(self, data: bytes) -> List[ParsedFrame]:
        """喂入若干字节，返回解析出的完整帧列表。"""
        frames: List[ParsedFrame] = []

        for b in data:
            if self.state == ParseState.WAIT_HEAD1:
                if b == FRAME_HEAD1:
                    self.state = ParseState.WAIT_HEAD2
                # 否则继续等待

            elif self.state == ParseState.WAIT_HEAD2:
                if b == FRAME_HEAD2:
                    self.state = ParseState.WAIT_LEN
                else:
                    # 回退到重新寻找帧头
                    self.state = ParseState.WAIT_HEAD1

            elif self.state == ParseState.WAIT_LEN:
                self.length = b
                if self.length > MAX_PAYLOAD_LEN:
                    LOGGER.warning("收到非法长度：%d，丢弃帧", self.length)
                    self.reset()
                else:
                    self.state = ParseState.WAIT_CMD

            elif self.state == ParseState.WAIT_CMD:
                self.cmd = b
                self._payload_bytes.clear()
                if self.length == 0:
                    self.state = ParseState.WAIT_CRC1
                else:
                    self.state = ParseState.WAIT_DATA

            elif self.state == ParseState.WAIT_DATA:
                self._payload_bytes.append(b)
                if len(self._payload_bytes) >= self.length:
                    self.state = ParseState.WAIT_CRC1

            elif self.state == ParseState.WAIT_CRC1:
                self._crc_low = b
                self.state = ParseState.WAIT_CRC2

            elif self.state == ParseState.WAIT_CRC2:
                crc_high = b
                received_crc = self._crc_low | (crc_high << 8)
                calc_crc = crc16(
                    bytes([self.length, self.cmd]) + bytes(self._payload_bytes)
                )
                if received_crc == calc_crc:
                    frames.append(ParsedFrame(cmd=self.cmd, payload=bytes(self._payload_bytes)))
                else:
                    LOGGER.warning(
                        "CRC 校验失败：received=0x%04X, calc=0x%04X",
                        received_crc,
                        calc_crc,
                    )
                self.reset()

        return frames

