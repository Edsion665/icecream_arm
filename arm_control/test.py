#!/usr/bin/env python3
"""
STM32 串口连通性测试脚本

逻辑：
1) 通过串口发送 "start"
2) 在指定超时时间内等待返回
3) 收到 "ok" 则判定连接正常
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("缺少依赖 pyserial，请先执行: pip install pyserial")
    sys.exit(2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Raspberry Pi -> STM32 串口握手 + 关节发送测试")
    parser.add_argument("--port", default="/dev/ttyAMA2", help="串口设备，例如 /dev/ttyAMA0 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600, help="波特率")
    parser.add_argument("--timeout", type=float, default=5.0, help="握手超时秒数")
    # 关节角，单位：度×100（与 SerialProtocol.md 一致）
    parser.add_argument("--a0", type=int, default=100, help="关节0 角度（度×100）")
    parser.add_argument("--a1", type=int, default=200, help="关节1 角度（度×100）")
    parser.add_argument("--a2", type=int, default=300, help="关节2 角度（度×100）")
    parser.add_argument("--a3", type=int, default=400, help="关节3 角度（度×100）")
    parser.add_argument("--a4", type=int, default=0, help="关节4 角度（度×100）")
    parser.add_argument("--a5", type=int, default=0, help="关节5 角度（度×100）")
    return parser.parse_args()


def calc_cs(payload: str) -> str:
    """对 payload 的 ASCII 字节做 XOR，输出 2 位十六进制校验码。"""
    cs = 0
    for b in payload.encode("ascii"):
        cs ^= b
    return f"{cs:02X}"


def main() -> int:
    args = parse_args()

    # 1) 握手：start\r\n -> 等待 ok
    payload = b"start\r\n"
    deadline = time.time() + args.timeout

    try:
        with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
            # 清空缓存，避免历史数据干扰
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            ser.write(payload)
            ser.flush()
            print(f"[TX handshake] {payload!r}")

            while time.time() < deadline:
                raw = ser.readline()
                if not raw:
                    continue

                text = raw.decode(errors="ignore").strip().lower()
                print(f"[RX handshake] {text}")

                if text == "ok":
                    print("握手成功：收到 ok")
                    break
            else:
                print(f"连接异常：{args.timeout:.1f}s 内未收到 ok")
                return 1

            # 2) 按 SerialProtocol.md 发送一帧 6 关节指令
            a0, a1, a2, a3, a4, a5 = (
                args.a0,
                args.a1,
                args.a2,
                args.a3,
                args.a4,
                args.a5,
            )
            payload_str = f"DATA:{a0},{a1},{a2},{a3},{a4},{a5}"
            cs_str = calc_cs(payload_str)
            frame = f"{payload_str}*{cs_str}\r\n".encode("ascii")

            print(f"[TX frame] {frame!r}")
            ser.write(frame)
            ser.flush()

            # 可选：再读一小段返回，便于调试
            time.sleep(1)
            rest = ser.read(256)
            if rest:
                print(f"[RX after frame] {rest!r}")

            return 0
    except serial.SerialException as exc:
        print(f"串口打开/通信失败: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
