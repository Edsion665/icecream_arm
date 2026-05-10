#!/usr/bin/env python3
"""从臂关节角 WebSocket 读取脚本。

连接从臂树莓派 WS 8765，订阅 ``type=state`` 帧，打印四轴关节角（度）。
与主臂 ``PiFeedbackClient`` 使用相同协议，替代 UDP 方案。

运行（在仓库根 ``isaacsim`` 下）::

    python3 -m arm_control_bridge.teleop.read_slave_ws

常用参数::

    python3 -m arm_control_bridge.teleop.read_slave_ws \\
        --slave-ip 192.168.31.195 --ws-port 8765 --rate-hz 10
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from arm_control_bridge.teleop.communication.slave_arm_ws import SlaveArmWsClient


def main() -> None:
    p = argparse.ArgumentParser(description="从臂 WebSocket 读角（URDF 对齐用）")
    p.add_argument("--slave-ip", default="192.168.31.195", help="从臂树莓派 IP")
    p.add_argument("--ws-port", type=int, default=8765, help="从臂 WebSocket 端口")
    p.add_argument("--rate-hz", type=float, default=10.0, help="控制台打印频率上限")
    args = p.parse_args()

    client = SlaveArmWsClient(rpi_ip=args.slave_ip, port=args.ws_port)
    print(f"[read_slave_ws] 连接 ws://{args.slave_ip}:{args.ws_port} ，等待数据…")

    period = 1.0 / max(float(args.rate_hz), 0.1)
    next_print = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            if now >= next_print:
                next_print = now + period
                rel_rad = client.get_rel_arm_rad()
                if rel_rad is None:
                    raw_rad = client.get_fb_arm_rad()
                    if raw_rad is None:
                        print("[read_slave_ws] 等待从臂回传…")
                    else:
                        print("[read_slave_ws] 等待 calibration_rad…")
                else:
                    print(f"rel_rad(减零后)={np.array2string(rel_rad, precision=4)}")
            time.sleep(min(period * 0.1, 0.02))
    except KeyboardInterrupt:
        print("\n[read_slave_ws] 退出")
    finally:
        client.close()


if __name__ == "__main__":
    main()
