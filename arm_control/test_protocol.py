"""串口协议单元测试：封帧、解帧、CRC16。"""

from __future__ import annotations

from .protocol import FrameParser, ParsedFrame, crc16, pack_frame


def test_crc16_basic() -> None:
    data = b"\x01\x10\x01\x02\x03"
    value = crc16(data)
    # 只检查结果稳定性
    assert isinstance(value, int)
    assert 0 <= value <= 0xFFFF


def test_pack_and_parse_roundtrip() -> None:
    cmd = 0x10
    payload = b"\x01\x02\x03\x04"

    frame = pack_frame(cmd, payload)
    parser = FrameParser()
    frames = parser.feed(frame)

    assert len(frames) == 1
    parsed: ParsedFrame = frames[0]
    assert parsed.cmd == cmd
    assert parsed.payload == payload


def test_parser_resync_on_garbage() -> None:
    parser = FrameParser()
    garbage = b"\x00\xFF\xAA\x00\x55"  # 中间插入假头部
    frames = parser.feed(garbage)
    assert frames == []

