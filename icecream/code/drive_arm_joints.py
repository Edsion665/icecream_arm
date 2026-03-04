#!/usr/bin/env python3
"""
第二套：机械臂驱动，直接用关节角度控制机械臂运动。
加载由 urdf_to_usd.py 导出的 USD，使用 Articulation 的关节位置控制。
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import os
import sys

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction

# 路径配置
CODE_DIR = os.path.dirname(os.path.abspath(__file__))
ARM_USD = os.path.join(CODE_DIR, "ice_cream_arm.usd")
ARM_PRIM_PATH = "/World/IceCreamArm"

# 机械臂 5 个关节（来自 URDF: joint1 ~ joint5），弧度
NUM_JOINTS = 5


def main():
    parser = argparse.ArgumentParser(description="关节角度控制 ice cream 机械臂")
    parser.add_argument("--usd", type=str, default=ARM_USD, help="机械臂 USD 文件路径")
    parser.add_argument("--test", action="store_true", help="运行短测试后退出")
    args, _ = parser.parse_known_args()

    usd_path = os.path.abspath(args.usd)
    if not os.path.isfile(usd_path):
        print(f"错误：未找到 USD 文件 {usd_path}")
        print("请先运行 urdf_to_usd.py 生成 ice_cream_arm.usd")
        simulation_app.close()
        sys.exit(1)

    physics_dt = 1.0 / 60.0
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=physics_dt)
    world.scene.add_default_ground_plane()

    # 将机械臂 USD 引用到场景
    add_reference_to_stage(usd_path=usd_path, prim_path=ARM_PRIM_PATH)
    arm = world.scene.add(SingleArticulation(ARM_PRIM_PATH, name="ice_cream_arm"))

    world.reset()

    # 关节控制器（位置控制）
    controller = arm.get_articulation_controller()

    # 可选：设置关节刚度/阻尼，使位置跟踪更稳定（若 USD 中未配置）
    try:
        controller.set_gains(
            kps=np.full(NUM_JOINTS, 1e4, dtype=float),
            kds=np.full(NUM_JOINTS, 1e3, dtype=float),
        )
    except Exception:
        pass

    # 示例：若干目标关节角（弧度），按顺序执行
    waypoints_rad = [
        np.zeros(NUM_JOINTS),
        np.array([0.5, 0.2, -0.3, 0.1, 0.0]),
        np.array([-0.3, 0.4, 0.2, -0.2, 0.1]),
        np.zeros(NUM_JOINTS),
    ]

    step_count = 0
    waypoint_index = 0
    steps_per_waypoint = 120  # 约 2 秒一个位姿
    max_steps = (steps_per_waypoint * len(waypoints_rad)) if args.test else None
    total_steps = 0

    while simulation_app.is_running():
        world.step(render=True)

        if not world.is_playing():
            continue

        # 每隔一段时间切换下一个目标关节角
        step_count += 1
        if step_count >= steps_per_waypoint:
            step_count = 0
            waypoint_index = (waypoint_index + 1) % len(waypoints_rad)

        target_positions = waypoints_rad[waypoint_index]
        controller.apply_action(ArticulationAction(joint_positions=target_positions.tolist()))

        total_steps += 1
        if args.test and max_steps and total_steps >= max_steps:
            break

    simulation_app.close()


if __name__ == "__main__":
    main()
