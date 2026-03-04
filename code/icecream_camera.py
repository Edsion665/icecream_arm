#!/usr/bin/env python3
"""
机械臂 + link5 深度相机 + 键盘控制（W/S 选关节，A/D 调角度）。
深度相机驱动已并入 icecream_driver，此处从 driver 导入并运行 main。
"""
from __future__ import annotations

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import os
import sys

import carb
import numpy as np
import omni.appwindow
from pxr import Gf, Sdf, UsdLux

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.stage import get_current_stage

_code_dir = os.path.dirname(os.path.abspath(__file__))
if _code_dir not in sys.path:
    sys.path.insert(0, _code_dir)
from icecream_driver import (
    DepthCameraObjectMeasurer,
    IceCreamArmDriver,
    JOINT_NAMES,
    Link5DepthCamera,
    NUM_JOINTS,
)


def _main():
    parser = argparse.ArgumentParser(description="Ice Cream arm + link5 depth camera, W/S select joint, A/D adjust angle")
    parser.add_argument("--usd", type=str, default=None, help="Ice Cream arm USD path")
    parser.add_argument("--step_deg", type=float, default=5.0, help="A/D adjust angle step (degree)")
    parser.add_argument("--link5-path", type=str, default="/World/IceCreamArm/link5", help="link5 prim path")
    parser.add_argument("--resolution", type=int, nargs=2, default=[640, 480], help="camera resolution width height")
    parser.add_argument("--frequency", type=float, default=20.0, help="camera frequency Hz")
    args, _ = parser.parse_known_args()

    step_deg = args.step_deg
    step_rad = np.deg2rad(step_deg)
    physics_dt = 1.0 / 60.0
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=physics_dt)
    # 网格状地面（会访问 assets，可能产生一次 blocking 提示）
    world.scene.add_default_ground_plane()
    # 彩色方向光（暖白色，可改 Gf.Vec3f(r,g,b)）
    stage = get_current_stage()
    if stage is not None:
        light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/ColoredLight"))
        light.CreateIntensityAttr(800.0)
        light.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.85))

    driver = IceCreamArmDriver(world, usd_path=args.usd)
    world.reset()

    # 在世界中放置一个测试立方体，方便演示“测量物体中心点世界坐标”
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/TargetCube",
            name="target_cube",
            position=np.array([0.6, 0.0, 0.1]),
            scale=np.array([0.1, 0.1, 0.1]),
            size=1.0,
            color=np.array([0, 128, 255]),
        )
    )

    # link5 深度相机（与运动驱动分开，单独实例化）
    camera = Link5DepthCamera(
        link5_prim_path=args.link5_path,
        name="Link5DepthCamera",
        resolution=tuple(args.resolution),
        frequency=args.frequency,
        offset_above_link5=(0.0, 0.05, -0.05),
        roll_deg=-90.0,
        pitch_deg=90.0,
        yaw_deg=0.0,
    )
    camera.initialize()
    camera.add_distance_to_camera_to_frame()

    # 封装一个测量器，按下 C 键时，用深度相机测量“目标立方体几何中心”的世界坐标
    measurer = DepthCameraObjectMeasurer(camera)

    selected_joint = 0

    def on_key(event, *_a, **_k):
        nonlocal selected_joint
        if event.type != carb.input.KeyboardEventType.KEY_PRESS:
            return True
        name = event.input.name
        if name == "W":
            selected_joint = (selected_joint + 1) % NUM_JOINTS
            carb.log_info(f"[icecream_camera] selected joint: {selected_joint} ({JOINT_NAMES[selected_joint]})")
        elif name == "S":
            selected_joint = (selected_joint - 1) % NUM_JOINTS
            carb.log_info(f"[icecream_camera] selected joint: {selected_joint} ({JOINT_NAMES[selected_joint]})")
        elif name == "A":
            targets = driver.get_target_positions()
            new_val = targets[selected_joint] - step_rad
            driver.set_joint_position(selected_joint, new_val)
            carb.log_info(f"[icecream_camera] {JOINT_NAMES[selected_joint]} -= {step_deg}° -> {np.rad2deg(new_val):.1f}°")
        elif name == "D":
            targets = driver.get_target_positions()
            new_val = targets[selected_joint] + step_rad
            driver.set_joint_position(selected_joint, new_val)
            carb.log_info(f"[icecream_camera] {JOINT_NAMES[selected_joint]} += {step_deg}° -> {np.rad2deg(new_val):.1f}°")
        elif name == "C":
            # 物体中心点（这里示例为立方体几何中心）的世界坐标
            cube_pos_world, _ = cube.get_world_pose()
            cube_center_world = np.array(cube_pos_world, dtype=float)
            carb.log_info(f"[icecream_camera] gt cube center (world): {cube_center_world}")

            # 示例：先 world_to_pixel，再从深度图取深度，最后 pixel_depth_to_world
            frame = camera.get_current_frame()
            depth = frame.get("distance_to_camera", None)
            if depth is None:
                carb.log_warn("[icecream_camera] current frame has no distance_to_camera channel, cannot measure")
                return True

            depth_arr = np.array(depth).squeeze()

            # 1) 世界坐标 -> 像素坐标
            pixel_info = measurer.world_to_pixel(cube_center_world, image_shape=depth_arr.shape)
            u_i, v_i = pixel_info["pixel"]
            if not pixel_info["in_bounds"]:
                carb.log_warn(
                    "[icecream_camera] projected pixel out of range: "
                    f"(u, v)=({u_i}, {v_i}), depth shape={depth_arr.shape}"
                )
                return True

            # 2) 从深度图中读取该像素的深度值
            d_img = float(depth_arr[v_i, u_i])
            carb.log_info(
                f"[icecream_camera] cube center pixel (u, v)=({u_i}, {v_i}), depth={d_img:.4f} m"
            )

            # 3) 像素坐标 + 深度 -> 世界坐标
            p_world_est = measurer.pixel_depth_to_world(u_i, v_i, d_img)
            carb.log_info(
                "[icecream_camera] cube center (world) measurement:\n"
                f"  gt_world : {cube_center_world}\n"
                f"  est_world: {p_world_est}\n"
            )
        return True

    _input = carb.input.acquire_input_interface()
    _appwindow = omni.appwindow.get_default_app_window()
    _keyboard = _appwindow.get_keyboard()
    _input.subscribe_to_keyboard_events(_keyboard, on_key)
    carb.log_info("[icecream_camera] keyboard: W/S select joint, A/D increase/decrease angle (step %.1f°)" % step_deg)
    carb.log_info("[icecream_camera] link5 depth camera mounted, selected joint: %s" % JOINT_NAMES[selected_joint])

    reset_needed = False
    while simulation_app.is_running():
        world.step(render=True)
        if world.is_stopped():
            reset_needed = True
        if world.is_playing():
            if reset_needed:
                world.reset()
                camera.initialize()
                camera.add_distance_to_camera_to_frame()
                reset_needed = False
            driver.apply()

    simulation_app.close()


if __name__ == "__main__":
    _main()
