#!/usr/bin/env python3
"""通过 Pi→STM32 MIT v3（42 字节）下行帧验证步进字段 ``stepper_deg``。

协议见 ``docs/pi2stm.md``：下行 ``stepper_deg`` 为 **增量角**（int16，°），``0`` 表示本帧不新增步进；
本脚本默认依次发送字段值 45、90、180、0（四帧），便于观察固件是否按增量响应。

**使用前请停止 ``runtime.py`` / ``start.sh``**，独占串口，避免两路同时写 UART。

运行示例::

    cd /path/to/icecream
    python3 scripts/test_stepper_mit42.py --port /dev/ttyAMA2

或::

    ARM_CONTROL_SERIAL_PORT=/dev/ttyUSB0 python3 scripts/test_stepper_mit42.py --dwell 3
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

try:
    import serial  # type: ignore[import-untyped]
except ImportError as exc:  # pragma: no cover
    raise SystemExit("需要 pyserial：pip install pyserial / uv sync") from exc

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from infra.serial.codec import SERVO_CENTER_US, encode_mit_cmd_42

# 四路电机占位：kp/kd 为 0，避免无意发力（仍遵守 MIT 打包范围）
HOLD_MOTOR = {"p": 0.0, "v": 0.0, "kp": 0.0, "kd": 0.0, "t": 0.0}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="MIT v3 下行：按序列发送 stepper_deg（42 字节帧 + HEX 打印）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--port",
        default=None,
        help="串口设备（默认环境变量 ARM_CONTROL_SERIAL_PORT，否则 /dev/ttyAMA2）",
    )
    p.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    p.add_argument(
        "--dwell",
        type=float,
        default=2.5,
        metavar="SEC",
        help="每帧发送后的等待时间（秒），给步进非阻塞执行留出时间，默认 2.5",
    )
    p.add_argument(
        "--sequence",
        default="45,90,180,0",
        help="逗号分隔的 stepper_deg 序列（每帧一个 int16），默认 45,90,180,0",
    )
    p.add_argument(
        "--wrist-us",
        type=int,
        default=None,
        help="腕舵机 us（默认中性 %d）" % SERVO_CENTER_US,
    )
    p.add_argument(
        "--gripper-us",
        type=int,
        default=None,
        help="夹爪 us（默认中性 %d）" % SERVO_CENTER_US,
    )
    p.add_argument(
        "--conveyor",
        type=int,
        choices=(0, 1),
        default=0,
        help="conveyor_run：0 停，1 转；默认 0",
    )
    p.add_argument(
        "--loops",
        type=int,
        default=1,
        help="整个序列重复次数，默认 1",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    port = args.port or os.environ.get("ARM_CONTROL_SERIAL_PORT", "/dev/ttyAMA2")
    wrist = int(args.wrist_us if args.wrist_us is not None else SERVO_CENTER_US)
    gripper = int(args.gripper_us if args.gripper_us is not None else SERVO_CENTER_US)
    try:
        seq = [int(x.strip()) for x in args.sequence.split(",") if x.strip()]
    except ValueError as exc:
        raise SystemExit(f"--sequence 解析失败: {args.sequence!r}") from exc
    if not seq:
        raise SystemExit("序列为空")

    motors = [dict(HOLD_MOTOR) for _ in range(4)]

    print(f"端口 {port} @ {args.baud}，序列 stepper_deg={seq}（共 {len(seq)} 帧 × {args.loops} 轮）")
    print("文档语义：每帧为增量角；最后一项为 0 表示该帧不追加步进指令。\n")

    with serial.Serial(
        port=port,
        baudrate=args.baud,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.2,
    ) as ser:
        for loop in range(args.loops):
            if args.loops > 1:
                print(f"--- 第 {loop + 1}/{args.loops} 轮 ---")
            for i, deg in enumerate(seq):
                frame = encode_mit_cmd_42(
                    motors,
                    wrist,
                    gripper,
                    stepper_deg=deg,
                    conveyor_run=args.conveyor,
                )
                hx = frame.hex().upper()
                # 按字节空格分隔，便于对照文档下标
                hx_spaced = " ".join(hx[j : j + 2] for j in range(0, len(hx), 2))
                print(f"[{i + 1}/{len(seq)}] stepper_deg={deg:+4d}  len={len(frame)}  HEX:\n    {hx_spaced}")
                ser.write(frame)
                ser.flush()
                time.sleep(max(0.0, float(args.dwell)))
    print("完成。")


if __name__ == "__main__":
    main()
