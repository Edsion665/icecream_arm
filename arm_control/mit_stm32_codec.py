"""STM32 ↔ 树莓派 MIT 二进制帧编解码（与 RPI_STM32_PROTOCOL.md 一致）。

上行：34 字节 = 帧头 0xAA 0x55 + 四电机 × 8 字节 CAN 原始反馈。
下行：32 字节 = 四电机 × 8 字节控制（与 CAN MIT 一致）。
"""

from __future__ import annotations

from typing import Any

# 各电机 P/V/T 物理范围（与协议表一致；解码必须按电机索引选用）
MOTOR_PARAMS: list[dict[str, tuple[float, float]]] = [
    {"p": (-12.5, 12.5), "v": (-30.0, 30.0), "t": (-10.0, 10.0)},  # M1
    {"p": (-12.5, 12.5), "v": (-10.0, 10.0), "t": (-28.0, 28.0)},  # M2
    {"p": (-12.5, 12.5), "v": (-10.0, 10.0), "t": (-28.0, 28.0)},  # M3
    {"p": (-12.5, 12.5), "v": (-30.0, 30.0), "t": (-10.0, 10.0)},  # M4
]

KP_RANGE: tuple[float, float] = (0.0, 500.0)
KD_RANGE: tuple[float, float] = (0.0, 5.0)

UPLINK_HEADER: tuple[int, int] = (0xAA, 0x55)
# 2 字节帧头 + 4×8 字节原始 CAN 反馈
UPLINK_FRAME_LEN: int = 34

# 树莓派 → STM32 MIT 命令帧（RPI_MIT_CMD_BINARY_ENCODE.md）：35 字节 = 帧头 + 4×8 + XOR
MIT_CMD_P_MIN, MIT_CMD_P_MAX = -12.5, 12.5
MIT_CMD_V_MIN, MIT_CMD_V_MAX = -45.0, 45.0
MIT_CMD_T_MIN, MIT_CMD_T_MAX = -18.0, 18.0
MIT_CMD_FRAME_LEN: int = 35


def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    return float(x) / float((1 << bits) - 1) * (x_max - x_min) + x_min


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * float((1 << bits) - 1))


def decode_uplink(data: bytes) -> list[dict[str, Any]]:
    """解码上行帧（34 字节）→ 四电机 err / p / v / t / mos_temp / rotor_temp（与协议示例一致）。"""
    if len(data) != UPLINK_FRAME_LEN:
        raise ValueError(
            f"上行帧须为 {UPLINK_FRAME_LEN} 字节，实际 {len(data)}"
        )
    if data[0] != UPLINK_HEADER[0] or data[1] != UPLINK_HEADER[1]:
        raise ValueError(
            f"上行帧头须为 0x{UPLINK_HEADER[0]:02X} 0x{UPLINK_HEADER[1]:02X}"
        )
    result: list[dict[str, Any]] = []
    for i in range(4):
        b = data[2 + i * 8 : 2 + i * 8 + 8]
        err = b[0] >> 4
        p = (b[1] << 8) | b[2]
        v = (b[3] << 4) | (b[4] >> 4)
        t = ((b[4] & 0x0F) << 8) | b[5]
        pm = MOTOR_PARAMS[i]
        pp, vp, tp = pm["p"], pm["v"], pm["t"]
        result.append(
            {
                "err": err,
                "p": uint_to_float(p, pp[0], pp[1], 16),
                "v": uint_to_float(v, vp[0], vp[1], 12),
                "t": uint_to_float(t, tp[0], tp[1], 12),
                "mos_temp": int(b[6]),
                "rotor_temp": int(b[7]),
            }
        )
    return result


def encode_downlink(cmds: list[dict[str, Any]]) -> bytes:
    """编码下行帧 → 32 字节；cmds 每项含 p、v、t、kp、kd（物理量）。"""
    if len(cmds) != 4:
        raise ValueError("须恰好 4 个电机指令")
    out = bytearray(32)
    for i, cmd in enumerate(cmds):
        pm = MOTOR_PARAMS[i]
        pp, vp, tp = pm["p"], pm["v"], pm["t"]
        p = float_to_uint(float(cmd["p"]), pp[0], pp[1], 16)
        v = float_to_uint(float(cmd["v"]), vp[0], vp[1], 12)
        kp = float_to_uint(float(cmd["kp"]), KP_RANGE[0], KP_RANGE[1], 12)
        kd = float_to_uint(float(cmd["kd"]), KD_RANGE[0], KD_RANGE[1], 12)
        t = float_to_uint(float(cmd["t"]), tp[0], tp[1], 12)
        base = i * 8
        out[base + 0] = (p >> 8) & 0xFF
        out[base + 1] = p & 0xFF
        out[base + 2] = (v >> 4) & 0xFF
        out[base + 3] = ((v & 0x0F) << 4) | ((kp >> 8) & 0x0F)
        out[base + 4] = kp & 0xFF
        out[base + 5] = (kd >> 4) & 0xFF
        out[base + 6] = ((kd & 0x0F) << 4) | ((t >> 8) & 0x0F)
        out[base + 7] = t & 0xFF
    return bytes(out)


def _encode_motor_cmd_mit_binary_doc(
    p: float, v: float, kp: float, kd: float, t: float
) -> bytes:
    """单电机 8 字节，物理量范围与 RPI_MIT_CMD_BINARY_ENCODE.md 一致。"""
    p_i = float_to_uint(p, MIT_CMD_P_MIN, MIT_CMD_P_MAX, 16)
    v_i = float_to_uint(v, MIT_CMD_V_MIN, MIT_CMD_V_MAX, 12)
    kp_i = float_to_uint(kp, KP_RANGE[0], KP_RANGE[1], 12)
    kd_i = float_to_uint(kd, KD_RANGE[0], KD_RANGE[1], 12)
    t_i = float_to_uint(t, MIT_CMD_T_MIN, MIT_CMD_T_MAX, 12)
    b = bytearray(8)
    b[0] = (p_i >> 8) & 0xFF
    b[1] = p_i & 0xFF
    b[2] = (v_i >> 4) & 0xFF
    b[3] = ((v_i & 0xF) << 4) | ((kp_i >> 8) & 0xF)
    b[4] = kp_i & 0xFF
    b[5] = (kd_i >> 4) & 0xFF
    b[6] = ((kd_i & 0xF) << 4) | ((t_i >> 8) & 0xF)
    b[7] = t_i & 0xFF
    return bytes(b)


def decode_motor_cmd_mit_binary_doc(b: bytes) -> dict[str, float]:
    """单电机 8 字节 → p/v/kp/kd/t（与 `_encode_motor_cmd_mit_binary_doc` 互逆，RPI_MIT_CMD_BINARY_ENCODE.md）。"""
    if len(b) != 8:
        raise ValueError("单电机载荷须为 8 字节")
    p_i = (b[0] << 8) | b[1]
    v_i = (((b[2] << 4) | (b[3] >> 4)) & 0xFFF)
    kp_i = ((b[3] & 0x0F) << 8) | b[4]
    kd_i = (((b[5] << 4) | (b[6] >> 4)) & 0xFFF)
    t_i = ((b[6] & 0x0F) << 8) | b[7]
    return {
        "p": uint_to_float(p_i, MIT_CMD_P_MIN, MIT_CMD_P_MAX, 16),
        "v": uint_to_float(v_i, MIT_CMD_V_MIN, MIT_CMD_V_MAX, 12),
        "kp": uint_to_float(kp_i, KP_RANGE[0], KP_RANGE[1], 12),
        "kd": uint_to_float(kd_i, KD_RANGE[0], KD_RANGE[1], 12),
        "t": uint_to_float(t_i, MIT_CMD_T_MIN, MIT_CMD_T_MAX, 12),
    }


def decode_mit_cmd_frame_35(data: bytes) -> tuple[list[dict[str, float]], bool]:
    """解码 35 字节 MIT 命令帧 → 四电机物理量，以及 XOR 校验是否与末字节一致。"""
    if len(data) != MIT_CMD_FRAME_LEN:
        raise ValueError(
            f"MIT 命令帧须为 {MIT_CMD_FRAME_LEN} 字节，实际 {len(data)}"
        )
    if data[0] != UPLINK_HEADER[0] or data[1] != UPLINK_HEADER[1]:
        raise ValueError(
            f"帧头须为 0x{UPLINK_HEADER[0]:02X} 0x{UPLINK_HEADER[1]:02X}"
        )
    xor_acc = 0
    for i in range(34):
        xor_acc ^= data[i]
    chk_ok = (xor_acc & 0xFF) == data[34]
    motors: list[dict[str, float]] = []
    for i in range(4):
        chunk = data[2 + i * 8 : 2 + i * 8 + 8]
        motors.append(decode_motor_cmd_mit_binary_doc(chunk))
    return motors, chk_ok


def build_mit_cmd_downlink_log_lines(raw: bytes) -> list[str]:
    """与 `serial_manager._log_mit_uplink_decode` 风格一致：连续 hex、空格 HEX、解码四电机 p/v/kp/kd/t。"""
    hex_compact = raw.hex().upper()
    lines: list[str] = [
        f"  原始下行(hex码): {hex_compact}",
        f"  原始下行(HEX): " + " ".join(f"{b:02X}" for b in raw),
    ]
    try:
        motors, xor_ok = decode_mit_cmd_frame_35(raw)
        lines.append(f"  xor_ok={xor_ok}")
        for i, m in enumerate(motors):
            lines.append(
                f"  M{i + 1}: p={m['p']:.6f} rad  v={m['v']:.6f} rad/s  "
                f"kp={m['kp']:.6f}  kd={m['kd']:.6f}  t={m['t']:.6f} Nm"
            )
    except (ValueError, RuntimeError) as exc:
        lines.append(f"  下行解码失败: {exc}")
    return lines


def encode_mit_cmd_frame_35(motors: list[dict[str, Any]]) -> bytes:
    """编码 35 字节 MIT 命令帧：帧头 0xAA 0x55 + 四电机×8B + XOR（见 RPI_MIT_CMD_BINARY_ENCODE.md）。

    每项须含 ``p, v, kp, kd, t``（物理量）；``t`` 通常由 Pinocchio 重力前馈填入。
    """
    if len(motors) != 4:
        raise ValueError("须恰好 4 个电机指令")
    frame = bytearray([0xAA, 0x55])
    for m in motors:
        frame += _encode_motor_cmd_mit_binary_doc(
            float(m["p"]),
            float(m["v"]),
            float(m["kp"]),
            float(m["kd"]),
            float(m["t"]),
        )
    xor_acc = 0
    for byte in frame:
        xor_acc ^= byte
    frame.append(xor_acc & 0xFF)
    if len(frame) != MIT_CMD_FRAME_LEN:
        raise RuntimeError(f"MIT 命令帧长度异常: {len(frame)}")
    return bytes(frame)


def try_parse_uplink_hex_from_line(text: str) -> bytes | None:
    """从一行文本恢复 34 字节上行帧：抽取十六进制，定位 ``AA55`` 后取连续 68 个十六进制位。

    若仅有 64 个十六进制位（32 字节载荷、无帧头），则自动补上 ``0xAA 0x55``。
    """
    s = "".join(c for c in text if c in "0123456789abcdefABCDEF")
    if len(s) < 64:
        return None
    su = s.upper()
    pos = su.find("AA55")
    if pos >= 0 and len(su) >= pos + 68:
        chunk = su[pos : pos + 68]
        try:
            raw = bytes.fromhex(chunk)
            if (
                len(raw) == UPLINK_FRAME_LEN
                and raw[0] == UPLINK_HEADER[0]
                and raw[1] == UPLINK_HEADER[1]
            ):
                return raw
        except ValueError:
            pass
    if len(su) >= 68:
        try:
            raw = bytes.fromhex(su[:68])
            if raw[0] == UPLINK_HEADER[0] and raw[1] == UPLINK_HEADER[1]:
                return raw
        except ValueError:
            pass
    if len(su) == 64:
        try:
            payload = bytes.fromhex(su)
            if len(payload) == 32:
                return bytes(UPLINK_HEADER) + payload
        except ValueError:
            pass
    return None


def try_pop_uplink_frame(buf: bytearray) -> bytes | None:
    """按 0xAA 0x55 同步，凑满 34 字节上行帧则取出并从 buf 删除。"""
    while True:
        if len(buf) < 2:
            return None
        if buf[0] != UPLINK_HEADER[0]:
            del buf[0]
            continue
        if buf[1] != UPLINK_HEADER[1]:
            del buf[0]
            continue
        if len(buf) < UPLINK_FRAME_LEN:
            return None
        frame = bytes(buf[:UPLINK_FRAME_LEN])
        del buf[:UPLINK_FRAME_LEN]
        return frame
