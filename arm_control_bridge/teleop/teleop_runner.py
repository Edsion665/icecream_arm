#!/usr/bin/env python3
"""遥操作主循环：从臂 WS 读角 → 主臂 UDP 发送。

从从臂树莓派 WebSocket 订阅关节角，转换后以 joints 模式驱动主臂。
频率与 CONFIG.control_hz（默认 25 Hz）对齐。

运行（在仓库根 isaacsim 下）::

    python3 -m arm_control_bridge.teleop.teleop_runner \\
        --slave-ip 192.168.31.195 --master-ip 192.168.31.211
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

import numpy as np

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


def main() -> None:
    from arm_control_bridge.config import CONFIG, load_calibration_deg
    from arm_control_bridge.io import RPiUDPStreamer, RpiProtocolAdapter, motor
    from arm_control_bridge.calculator import (
        CalculatorEngine,
        CalculatorState,
        URDFKinematics,
    )
    from arm_control_bridge.io.listener import MotionCommand4Axis
    from arm_control_bridge.teleop.communication.slave_arm_ws import SlaveArmWsClient

    p = argparse.ArgumentParser(description="遥操作：从臂 WS → 主臂 UDP")
    p.add_argument("--slave-ip", default="192.168.31.195", help="从臂树莓派 IP")
    p.add_argument("--slave-ws-port", type=int, default=8765, help="从臂 WebSocket 端口")
    p.add_argument("--master-ip", required=True, help="主臂树莓派 IP")
    p.add_argument("--master-port", type=int, default=CONFIG.default_udp_port, help="主臂 UDP 端口")
    p.add_argument("--urdf", default=None, help="URDF 路径（可选，用于 IK 初始化）")
    args = p.parse_args()

    # --- 主臂发送链路 ---
    streamer = RPiUDPStreamer(args.master_ip, args.master_port)
    adapter = RpiProtocolAdapter(streamer)
    arm_motor = motor(adapter)

    # --- 控制状态初始化（与 run_control.py 一致）---
    _pkg_dir = os.path.dirname(os.path.abspath(__file__ + "/.."))
    calib_file = os.path.join(_ROOT, CONFIG.calibration_md_relpath)
    q_calib_deg = np.array(load_calibration_deg(calib_file), dtype=float)
    q_calib_rad = np.deg2rad(q_calib_deg)

    urdf_path = args.urdf
    if urdf_path is None:
        from arm_control_bridge.config import SIM_CONFIG
        _config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "configuration")
        urdf_path = os.path.abspath(os.path.join(_config_dir, SIM_CONFIG.arm_urdf_relpath))

    kin = URDFKinematics(urdf_path=urdf_path)
    state = CalculatorState(
        q_calib_deg=q_calib_deg,
        q_calib_rad=q_calib_rad,
        q5_fixed_rad=np.deg2rad(CONFIG.q5_fixed_deg),
        pose_xyz=np.zeros(3, dtype=float),
    )
    state.reset_command()
    engine = CalculatorEngine(kin)

    # --- 从臂 WS 客户端 ---
    slave = SlaveArmWsClient(rpi_ip=args.slave_ip, port=args.slave_ws_port)
    print(
        f"[teleop] 从臂 ws://{args.slave_ip}:{args.slave_ws_port} → "
        f"主臂 udp://{args.master_ip}:{args.master_port} @ {CONFIG.control_hz} Hz"
    )
    print("[teleop] 等待从臂首帧（需 fb_arm_rad + calibration_rad）…")

    # 从臂→主臂各轴转向符号（轴1、2方向相反，轴3、4相同）
    SLAVE_TO_MASTER_SIGN = np.array([-1.0, -1.0, -1.0, 1.0], dtype=float)

    dt = CONFIG.control_dt
    next_t = time.monotonic()

    try:
        while True:
            rel_rad = slave.get_rel_arm_rad()  # fb_arm_rad - calibration_rad，URDF 空间弧度

            if rel_rad is not None:
                # 转成度数，构造 joints 命令（与 engine.apply_command 的 joints 分支对齐）
                rel_deg = np.rad2deg(rel_rad * SLAVE_TO_MASTER_SIGN).tolist()
                cmd = MotionCommand4Axis(
                    kind="joints",
                    payload={"axes_rel_deg": rel_deg},
                )
                engine.apply_command(cmd, state)

            frame = engine.step(None, state, dt=dt)
            arm_motor.send(frame)

            next_t += dt
            sleep_t = next_t - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.monotonic()

    except KeyboardInterrupt:
        print("\n[teleop] 退出")
    finally:
        slave.close()
        adapter.close()


if __name__ == "__main__":
    main()
