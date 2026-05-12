"""MIT frame codec utilities (pure functions)."""

from __future__ import annotations

MOTOR_PARAMS = [
    {"p": (-12.5, 12.5), "v": (-30.0, 30.0), "t": (-10.0, 10.0)},
    {"p": (-12.5, 12.5), "v": (-10.0, 10.0), "t": (-28.0, 28.0)},
    {"p": (-12.5, 12.5), "v": (-10.0, 10.0), "t": (-28.0, 28.0)},
    {"p": (-12.5, 12.5), "v": (-30.0, 30.0), "t": (-10.0, 10.0)},
]
KP_RANGE = (0.0, 500.0)
KD_RANGE = (0.0, 5.0)
MIT_P_RANGE = (-12.5, 12.5)
MIT_V_RANGE = (-45.0, 45.0)
MIT_T_RANGE = (-18.0, 18.0)

UPLINK_HEADER = (0xAA, 0x55)
UPLINK_FRAME_LEN = 39
MIT_CMD_FRAME_LEN = 35


def uint_to_float(x: int, lo: float, hi: float, bits: int) -> float:
    return float(x) / float((1 << bits) - 1) * (hi - lo) + lo


def float_to_uint(x: float, lo: float, hi: float, bits: int) -> int:
    x = max(lo, min(hi, x))
    return int((x - lo) / (hi - lo) * float((1 << bits) - 1))


def decode_mit_uplink(raw: bytes) -> tuple[list[dict[str, float | int]], dict[str, int]]:
    if len(raw) != UPLINK_FRAME_LEN or raw[0] != 0xAA or raw[1] != 0x55:
        raise ValueError("invalid MIT uplink frame")
    xor_calc = 0
    for b in raw[:38]:
        xor_calc ^= b
    if xor_calc != raw[38]:
        raise ValueError("MIT uplink XOR mismatch")
    out: list[dict[str, float | int]] = []
    for i in range(4):
        b = raw[2 + i * 8 : 2 + i * 8 + 8]
        err = b[0] >> 4
        p_u = (b[1] << 8) | b[2]
        v_u = (b[3] << 4) | (b[4] >> 4)
        t_u = ((b[4] & 0x0F) << 8) | b[5]
        rng = MOTOR_PARAMS[i]
        out.append(
            {
                "err": int(err),
                "p": uint_to_float(p_u, rng["p"][0], rng["p"][1], 16),
                "v": uint_to_float(v_u, rng["v"][0], rng["v"][1], 12),
                "t": uint_to_float(t_u, rng["t"][0], rng["t"][1], 12),
                "mos_temp": int(b[6]),
                "rotor_temp": int(b[7]),
            }
        )
    servo = {
        "wrist_us": (raw[34] << 8) | raw[35],
        "gripper_us": (raw[36] << 8) | raw[37],
    }
    return out, servo


def encode_mit_cmd_35(motors: list[dict[str, float]]) -> bytes:
    if len(motors) != 4:
        raise ValueError("expected 4 motor commands")
    frame = bytearray([0xAA, 0x55])
    for m in motors:
        p_u = float_to_uint(float(m["p"]), MIT_P_RANGE[0], MIT_P_RANGE[1], 16)
        v_u = float_to_uint(float(m["v"]), MIT_V_RANGE[0], MIT_V_RANGE[1], 12)
        kp_u = float_to_uint(float(m["kp"]), KP_RANGE[0], KP_RANGE[1], 12)
        kd_u = float_to_uint(float(m["kd"]), KD_RANGE[0], KD_RANGE[1], 12)
        t_u = float_to_uint(float(m["t"]), MIT_T_RANGE[0], MIT_T_RANGE[1], 12)
        frame.extend(
            [
                (p_u >> 8) & 0xFF,
                p_u & 0xFF,
                (v_u >> 4) & 0xFF,
                ((v_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),
                kp_u & 0xFF,
                (kd_u >> 4) & 0xFF,
                ((kd_u & 0x0F) << 4) | ((t_u >> 8) & 0x0F),
                t_u & 0xFF,
            ]
        )
    xor_acc = 0
    for b in frame:
        xor_acc ^= b
    frame.append(xor_acc & 0xFF)
    return bytes(frame)


def decode_mit_cmd_35(raw: bytes) -> tuple[list[dict[str, float]], bool]:
    if len(raw) != MIT_CMD_FRAME_LEN or raw[0] != 0xAA or raw[1] != 0x55:
        raise ValueError("invalid MIT cmd frame")
    xor_acc = 0
    for b in raw[:34]:
        xor_acc ^= b
    xor_ok = (xor_acc & 0xFF) == raw[34]

    motors: list[dict[str, float]] = []
    for i in range(4):
        b = raw[2 + i * 8 : 2 + i * 8 + 8]
        p_u = (b[0] << 8) | b[1]
        v_u = (b[2] << 4) | (b[3] >> 4)
        kp_u = ((b[3] & 0x0F) << 8) | b[4]
        kd_u = (b[5] << 4) | (b[6] >> 4)
        t_u = ((b[6] & 0x0F) << 8) | b[7]
        motors.append(
            {
                "p": uint_to_float(p_u, MIT_P_RANGE[0], MIT_P_RANGE[1], 16),
                "v": uint_to_float(v_u, MIT_V_RANGE[0], MIT_V_RANGE[1], 12),
                "kp": uint_to_float(kp_u, KP_RANGE[0], KP_RANGE[1], 12),
                "kd": uint_to_float(kd_u, KD_RANGE[0], KD_RANGE[1], 12),
                "t": uint_to_float(t_u, MIT_T_RANGE[0], MIT_T_RANGE[1], 12),
            }
        )
    return motors, xor_ok


SERVO_MIN_US = 500
SERVO_MAX_US = 2500
# MIT39 下行舵机脉宽中性位（与 ``encode_mit_cmd_39`` 钳位区间一致）；init/boot 等只读此层定义。
SERVO_CENTER_US = (SERVO_MIN_US + SERVO_MAX_US) // 2


def encode_mit_cmd_39(motors: list[dict[str, float]], wrist_us: int, gripper_us: int) -> bytes:
    if len(motors) != 4:
        raise ValueError('expected 4 motor commands')
    wrist_us = max(SERVO_MIN_US, min(SERVO_MAX_US, int(wrist_us)))
    gripper_us = max(SERVO_MIN_US, min(SERVO_MAX_US, int(gripper_us)))
    frame = bytearray([0xAA, 0x55])
    for m in motors:
        p_u = float_to_uint(float(m['p']), MIT_P_RANGE[0], MIT_P_RANGE[1], 16)
        v_u = float_to_uint(float(m['v']), MIT_V_RANGE[0], MIT_V_RANGE[1], 12)
        kp_u = float_to_uint(float(m['kp']), KP_RANGE[0], KP_RANGE[1], 12)
        kd_u = float_to_uint(float(m['kd']), KD_RANGE[0], KD_RANGE[1], 12)
        t_u = float_to_uint(float(m['t']), MIT_T_RANGE[0], MIT_T_RANGE[1], 12)
        frame.extend([
            (p_u >> 8) & 0xFF, p_u & 0xFF,
            (v_u >> 4) & 0xFF,
            ((v_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),
            kp_u & 0xFF,
            (kd_u >> 4) & 0xFF,
            ((kd_u & 0x0F) << 4) | ((t_u >> 8) & 0x0F),
            t_u & 0xFF,
        ])
    frame.append((wrist_us >> 8) & 0xFF)
    frame.append(wrist_us & 0xFF)
    frame.append((gripper_us >> 8) & 0xFF)
    frame.append(gripper_us & 0xFF)
    xor = 0
    for b in frame:
        xor ^= b
    frame.append(xor & 0xFF)
    assert len(frame) == 39
    return bytes(frame)
