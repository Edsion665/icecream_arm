#!/usr/bin/env python3
"""
第二节电机（索引 1 / M2）正转 140° + 第三节电机（索引 2 / M3）正转 130°。

- 读首帧 34B 上行（decode_uplink），记录四轴初值。
- 仅 M2、M3 运动（+140° / +130°）；M1、M4 保持首帧 p。
- 与 motor0_drive_neg30deg 相同离散步进：|Δ|/N ≤ v_max·dt（默认 v_max=0.3 rad/s）。
- 每帧发送目标角 p_k = p0 + (k/N)·Δ（各运动轴独立 Δ）；末帧运动轴 v=0；无重力补偿（t 全 0）。

用法:
  python3 motor23_drive_discrete.py
  python3 motor23_drive_discrete.py --port /dev/ttyAMA2 --hz 100
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


def _discrete_step_count(delta_mag_rad: float, v_max: float, dt: float) -> int:
    """按最大转角幅值取 N，使每步 ≤ v_max·dt。"""
    d = abs(delta_mag_rad)
    if d < 1e-12:
        return 0
    if v_max <= 0 or dt <= 0:
        raise ValueError("v_max 与 dt 须为正")
    max_step = v_max * dt
    return max(1, int(math.ceil(d / max_step)))


def main() -> None:
    parser = argparse.ArgumentParser(description="M2 +140° + M3 +130°，离散步进，无重力前馈")
    parser.add_argument("--port", default="/dev/ttyAMA2", help="串口设备")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--hz", type=float, default=100.0, help="指令发送频率（Hz）")
    parser.add_argument(
        "--v-max",
        type=float,
        default=0.3,
        help="各运动轴相邻帧目标角变化率上限（rad/s），默认 0.3",
    )
    parser.add_argument("--hold-sec", type=float, default=3.0, help="到位后保持（秒）")
    parser.add_argument(
        "--print-tx-hex",
        action="store_true",
        help="每周期打印 35B 下行 HEX",
    )
    args = parser.parse_args()

    kp4 = tuple(float(x) for x in MIT_CMD_FIXED_KP_NORMAL)
    kd4 = tuple(float(x) for x in MIT_CMD_FIXED_KD)

    # 第二节 +140°、第三节 +130°（索引 1、2）
    delta = [0.0, math.radians(140.0), math.radians(130.0), 0.0]

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
    print(f"首帧 OK，帧头 {raw_first[:2].hex().upper()}，四轴 p(rad): {p_init}", flush=True)

    dt = 1.0 / max(1.0, float(args.hz))
    n_steps = max(
        _discrete_step_count(abs(delta[1]), args.v_max, dt),
        _discrete_step_count(abs(delta[2]), args.v_max, dt),
    )
    if n_steps == 0:
        print("无位移，退出。", flush=True)
        ser.close()
        return

    step1 = delta[1] / n_steps
    step2 = delta[2] / n_steps
    p_final = [
        p_init[0],
        p_init[1] + delta[1],
        p_init[2] + delta[2],
        p_init[3],
    ]
    print(
        f"M2 Δ=+140° → p*={p_final[1]:.6f} rad | M3 Δ=+130° → p*={p_final[2]:.6f} rad | "
        f"N={n_steps} 步，dt={dt*1000:.3f} ms，v_max={args.v_max} rad/s",
        flush=True,
    )

    for k in range(1, n_steps + 1):
        p_cmd = [
            p_init[0],
            p_init[1] + delta[1] * (k / n_steps),
            p_init[2] + delta[2] * (k / n_steps),
            p_init[3],
        ]
        if k >= n_steps:
            v_cmd = [0.0, 0.0, 0.0, 0.0]
        else:
            v_cmd = [0.0, step1 / dt, step2 / dt, 0.0]

        cmds = _build_cmds_hold(p_cmd, kp4, kd4)
        for i in range(4):
            cmds[i]["v"] = float(v_cmd[i])
        frame = encode_mit_cmd_frame_35(cmds)
        assert len(frame) == MIT_CMD_FRAME_LEN
        ser.write(frame)
        if args.print_tx_hex:
            print(frame.hex().upper(), flush=True)
        time.sleep(dt)

    hold_end = time.monotonic() + args.hold_sec
    print(f"到位保持 {args.hold_sec} s…", flush=True)
    while time.monotonic() < hold_end:
        cmds = _build_cmds_hold(p_final, kp4, kd4)
        frame = encode_mit_cmd_frame_35(cmds)
        ser.write(frame)
        if args.print_tx_hex:
            print(frame.hex().upper(), flush=True)
        time.sleep(dt)

    print("完成。", flush=True)
    ser.close()


if __name__ == "__main__":
    main()
