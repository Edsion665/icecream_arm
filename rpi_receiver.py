#!/usr/bin/env python3
"""
树莓派端 UDP 接收脚本
======================
配套 sim_code/rpi_bridge.py + sim_code/cartesian_ik_verify.py。
接收 PC 仿真以控制频率（--ik-rate，默认 25Hz）发送的关节数据：
  p(°)  : 相对标定零点的关节角（度）[j1..j5]
  ω(rad/s): 关节角速度（rad/s）   [j1..j5]

运行（树莓派）：
    python3 rpi_receiver.py                  # 默认端口 9870，每秒摘要
    python3 rpi_receiver.py --verbose        # 每帧打印

PC 端启动仿真（加 --rpi-ip 和 --ik-rate 参数）：
    ~/isaac-sim/python.sh sim_code/cartesian_ik_verify.py \\
        0.35 0.2 0.25 --sim --ik-rate 25 --rpi-ip 192.168.x.x
"""

import argparse
import socket
import time

from arm_control.rpi_udp_packet import PACKET_SIZE, unpack_packet

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]


def receive_loop(port: int, verbose: bool):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', port))
    sock.settimeout(2.0)

    print(f"[RPI] 监听 UDP 0.0.0.0:{port}  包大小={PACKET_SIZE}B")
    print("[RPI] 等待 PC 端 cartesian_ik_verify 数据...\n")

    last_seq    = None
    total_drop  = 0
    t_last_summary = time.monotonic()

    while True:
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            print("[RPI] 超时：未收到数据，仍在等待...")
            continue

        if len(data) != PACKET_SIZE:
            print(f"[RPI] 包长度异常 {len(data)} != {PACKET_SIZE}，丢弃")
            continue

        pkt = unpack_packet(data)
        seq   = pkt['seq']
        p_deg = pkt['p_rel_deg']      # 度
        omega = pkt['omega_rad_s']    # rad/s

        # ── 丢包检测 ──────────────────────────────────────────────────────
        if last_seq is not None and seq != last_seq + 1:
            dropped = seq - last_seq - 1
            total_drop += dropped
            print(f"[RPI] ⚠ 丢包 {dropped} 帧 (seq {last_seq}→{seq})  累计={total_drop}")
        last_seq = seq

        lat_ms = (time.monotonic() - pkt['ts']) * 1000

        # 实际控制：在树莓派上运行 ``ARM_CONTROL_RPI_UDP=1 python -m arm_control.main``，
        # main._tau_ff_loop 与 UDP 同周期取最新 p/v 并编入 MIT 35B 下发（见 rpi_udp_joint_source）。

        if verbose:
            p_str = '[' + ', '.join(f'{v:+7.2f}' for v in p_deg) + ']'
            o_str = '[' + ', '.join(f'{v:+7.4f}' for v in omega) + ']'
            print(f"[{seq:6d}] p(°):{p_str}  ω(rad/s):{o_str}  lat={lat_ms:.1f}ms")
        else:
            t_now = time.monotonic()
            if t_now - t_last_summary >= 1.0:
                p_str = '[' + ', '.join(f'{v:+7.2f}' for v in p_deg) + ']'
                o_str = '[' + ', '.join(f'{v:+7.4f}' for v in omega) + ']'
                print(
                    f"[{seq:6d}] p(°):{p_str}  ω(rad/s):{o_str}  "
                    f"lat={lat_ms:.1f}ms  丢包={total_drop}"
                )
                t_last_summary = t_now


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='RPi UDP 接收器（配套 cartesian_ik_verify）')
    ap.add_argument('--port',    type=int,  default=9870, help='监听端口（默认 9870）')
    ap.add_argument('--verbose', action='store_true',     help='每帧打印，默认每秒摘要')
    args = ap.parse_args()

    try:
        receive_loop(args.port, args.verbose)
    except KeyboardInterrupt:
        print('\n[RPI] 已停止。')
