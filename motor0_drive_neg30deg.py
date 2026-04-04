#!/usr/bin/env python3
"""
第一节电机（索引 0 / M1）沿负向转动 30° 的驱动脚本。

- 按与 arm_control 相同的解码流读取首帧 34B 上行（0xAA 0x55 + decode_uplink），取 M1 当前 p(rad)。
- 目标总转角 Δ = −30°；将 |Δ| 离散成 N 个等角度步，使每步不超过 v_max·dt（默认 v_max=0.3 rad/s）。
- 每一发送周期发送一个目标角 p_k = p0 + (k/N)·Δ（k=1…N），递增直至终点；最后一帧 v=0，再保持。
- 中间帧前馈速度 v = (Δ/N)/dt（与离散阶梯一致）；末帧 v=0。
- kp/kd：MIT_CMD_FIXED_KP_NORMAL、MIT_CMD_FIXED_KD（与 config 一致）。
- 下发：encode_mit_cmd_frame_35 → 35 字节二进制。

用法（在仓库根目录、已激活 venv）:
  python3 motor0_drive_neg30deg.py
  python3 motor0_drive_neg30deg.py --port /dev/ttyAMA2 --hz 100
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import serial  # type: ignore[import]

from arm_control.config import MIT_CMD_FIXED_KP_NORMAL, MIT_CMD_FIXED_KD
from arm_control.mit_stm32_codec import (
    MIT_CMD_FRAME_LEN,
    decode_uplink,
    encode_mit_cmd_frame_35,
    try_pop_uplink_frame,
)


def _read_first_uplink(ser: serial.Serial, timeout_sec: float = 5.0) -> tuple[list[dict], bytes]:
    """从串口字节流中同步出第一帧 34B 上行并解码。"""
    buf = bytearray()
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
        frame = try_pop_uplink_frame(buf)
        if frame is not None:
            motors = decode_uplink(frame)
            return motors, frame
        if not chunk:
            time.sleep(0.002)
    raise TimeoutError(f"{timeout_sec}s 内未收到完整 34B 上行帧（0xAA 0x55…）")


def _build_cmds_hold(
    p_four: list[float],
    kp4: tuple[float, float, float, float],
    kd4: tuple[float, float, float, float],
) -> list[dict[str, float]]:
    out: list[dict[str, float]] = []
    for i in range(4):
        out.append(
            {
                "p": float(p_four[i]),
                "v": 0.0,
                "kp": float(kp4[i]),
                "kd": float(kd4[i]),
                "t": 0.0,
            }
        )
    return out


def _discrete_step_count(delta_rad: float, v_max: float, dt: float) -> int:
    """
    将总转角 |Δ| 分成 N 份等角度步，使每步转角 |Δ|/N ≤ v_max·dt（每帧角位移上限）。
    返回 N ≥ 1（若 Δ≈0 则 0）。
    """
    d = abs(delta_rad)
    if d < 1e-12:
        return 0
    if v_max <= 0 or dt <= 0:
        raise ValueError("v_max 与 dt 须为正")
    max_step = v_max * dt
    return max(1, int(math.ceil(d / max_step)))


def main() -> None:
    parser = argparse.ArgumentParser(description="M1 负向 30°，离散目标角步进 + 到位保持")
    parser.add_argument("--port", default="/dev/ttyAMA2", help="串口设备")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--hz", type=float, default=100.0, help="指令发送频率（Hz）")
    parser.add_argument(
        "--v-max",
        type=float,
        default=0.3,
        help="相邻帧之间「目标角变化率」上限（rad/s），默认 0.3；步长 = min(|Δ|/N, v_max·dt)",
    )
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=3.0,
        help="到位后保持发送的时长（秒）",
    )
    parser.add_argument(
        "--print-tx-hex",
        action="store_true",
        help="每周期打印一行 35B 下行连续 HEX（与串口实际发送字节一致）",
    )
    args = parser.parse_args()

    kp4 = tuple(float(x) for x in MIT_CMD_FIXED_KP_NORMAL)
    kd4 = tuple(float(x) for x in MIT_CMD_FIXED_KD)

    delta_rad = -math.radians(30.0)

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.05,
    )
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except OSError:
        pass

    print("等待第一帧上行（34B，decode_uplink）…", flush=True)
    motors, raw_first = _read_first_uplink(ser)
    p_init = [motors[i]["p"] for i in range(4)]
    print(
        f"首帧解码 OK，帧长 {len(raw_first)}，帧头 {raw_first[:2].hex().upper()}",
        flush=True,
    )
    print(
        f"M1 当前 p0 = {p_init[0]:.6f} rad | M2..M4 p = "
        f"{p_init[1]:.6f}, {p_init[2]:.6f}, {p_init[3]:.6f}",
        flush=True,
    )
    print(f"首帧原始 HEX（68 字符）: {raw_first.hex().upper()}", flush=True)

    p_target = p_init[0] + delta_rad
    dt = 1.0 / max(1.0, float(args.hz))
    n_steps = _discrete_step_count(delta_rad, args.v_max, dt)
    step_rad = delta_rad / n_steps if n_steps else 0.0
    step_deg = math.degrees(math.copysign(abs(step_rad), step_rad) if step_rad else 0.0)
    print(
        f"目标 M1 p* = {p_target:.6f} rad（Δ = {math.degrees(delta_rad):.1f}°），"
        f"离散 {n_steps} 步，每步 ≈ {math.degrees(abs(step_rad)):.6f}° "
        f"（|v|·dt 上限 = {args.v_max * dt:.6f} rad），发送周期 dt = {dt*1000:.3f} ms",
        flush=True,
    )

    # --- 运动段：k=1..N，每帧一个目标角 p_k = p0 + (k/N)*Δ；末帧 v=0 ---
    for k in range(1, n_steps + 1):
        p_k = p_init[0] + delta_rad * (k / n_steps)
        if k >= n_steps:
            v_k = 0.0
        else:
            v_k = step_rad / dt
        cmds = _build_cmds_hold(
            [p_k, p_init[1], p_init[2], p_init[3]],
            kp4,
            kd4,
        )
        cmds[0]["v"] = float(v_k)
        frame = encode_mit_cmd_frame_35(cmds)
        assert len(frame) == MIT_CMD_FRAME_LEN
        ser.write(frame)
        if args.print_tx_hex:
            print(frame.hex().upper(), flush=True)
        time.sleep(dt)

    # --- 保持段：p = 目标，v = 0 ---
    p_hold = [p_target, p_init[1], p_init[2], p_init[3]]
    hold_end = time.monotonic() + args.hold_sec
    print(f"到位保持 {args.hold_sec} s（M1 p={p_hold[0]:.6f} rad）…", flush=True)
    while time.monotonic() < hold_end:
        cmds = _build_cmds_hold(p_hold, kp4, kd4)
        frame = encode_mit_cmd_frame_35(cmds)
        ser.write(frame)
        if args.print_tx_hex:
            print(frame.hex().upper(), flush=True)
        time.sleep(dt)

    print("完成。", flush=True)
    ser.close()


if __name__ == "__main__":
    main()
