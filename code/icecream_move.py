#!/usr/bin/env python3
"""
使用键盘控制 Ice Cream 机械臂各关节角度：
  W/S：切换当前选中的关节（W 下一个，S 上一个）
  A/D：减小 / 增大当前关节角度
调用 icecream_driver 接口。
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import os
import sys

import carb
import numpy as np
import omni.appwindow

from isaacsim.core.api import World

# 保证可导入同目录下的 icecream_driver
_CODE_DIR = os.path.dirname(os.path.abspath(__file__))
if _CODE_DIR not in sys.path:
    sys.path.insert(0, _CODE_DIR)
from icecream_driver import IceCreamArmDriver, JOINT_NAMES, NUM_JOINTS


def main():
    parser = argparse.ArgumentParser(description="键盘控制 Ice Cream 机械臂关节：W/S 选关节，A/D 增减角度")
    parser.add_argument("--usd", type=str, default=None, help="机械臂 USD 路径")
    parser.add_argument("--step_deg", type=float, default=5.0, help="每次 A/D 调节角度（度），默认 5")
    args, _ = parser.parse_known_args()

    step_deg = args.step_deg
    step_rad = np.deg2rad(step_deg)
    physics_dt = 1.0 / 60.0
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=physics_dt)
    world.scene.add_ground_plane()

    driver = IceCreamArmDriver(world, usd_path=args.usd)
    world.reset()

    # 当前选中的关节索引 0 ~ 4
    selected_joint = 0

    def on_key(event, *_args, **_kwargs):
        nonlocal selected_joint
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return True
        name = event.input.name

        if name == "W":
            selected_joint = (selected_joint + 1) % NUM_JOINTS
            carb.log_info(f"[icecream_move] selected joint: {selected_joint} ({JOINT_NAMES[selected_joint]})")
        elif name == "S":
            selected_joint = (selected_joint - 1) % NUM_JOINTS
            carb.log_info(f"[icecream_move] selected joint: {selected_joint} ({JOINT_NAMES[selected_joint]})")
        elif name == "A":
            targets = driver.get_target_positions()
            new_val = targets[selected_joint] - step_rad
            driver.set_joint_position(selected_joint, new_val)
            carb.log_info(
                f"[icecream_move] {JOINT_NAMES[selected_joint]} -= {step_deg}° -> {np.rad2deg(new_val):.1f}°"
            )
        elif name == "D":
            targets = driver.get_target_positions()
            new_val = targets[selected_joint] + step_rad
            driver.set_joint_position(selected_joint, new_val)
            carb.log_info(
                f"[icecream_move] {JOINT_NAMES[selected_joint]} += {step_deg}° -> {np.rad2deg(new_val):.1f}°"
            )
        return True

    _input = carb.input.acquire_input_interface()
    _appwindow = omni.appwindow.get_default_app_window()
    _keyboard = _appwindow.get_keyboard()
    _input.subscribe_to_keyboard_events(_keyboard, on_key)

    carb.log_info(
        "[icecream_move] keyboard: W/S select joint, A/D decrease/increase current joint angle (step %.1f°)"
        % step_deg
    )
    carb.log_info("[icecream_move] selected joint: %s" % JOINT_NAMES[selected_joint])

    reset_needed = False
    while simulation_app.is_running():
        world.step(render=True)
        if world.is_stopped():
            reset_needed = True
        if world.is_playing():
            if reset_needed:
                world.reset()
                reset_needed = False
            driver.apply()

    simulation_app.close()


if __name__ == "__main__":
    main()
