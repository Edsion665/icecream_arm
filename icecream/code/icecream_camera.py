#!/usr/bin/env python3
"""
在 Ice Cream 机械臂 link5 上方安装深度相机。
- 相机类：Link5DepthCamera，刚性安装在 link5 上，可输出深度（distance_to_camera）。
- main：运行仿真 + 键盘控制关节（W/S 选关节，A/D 调角度），与 icecream_move 一致。
"""
from __future__ import annotations

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import os
import sys
from typing import Tuple

import carb
import numpy as np
import omni.appwindow
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera


class Link5DepthCamera:
    """
    安装在 link5 上方的深度相机（刚性随 link5 运动）。
    - 默认在 link5 局部坐标系下沿 +Z 偏移 10 cm（“上方”）。
    - 朝向由 link5 局部系下的欧拉角 (roll, pitch, yaw) 控制，单位：度。
      顺序：绕 X → Y → Z 固定轴（extrinsic），对应滚转(roll)、俯仰(pitch)、偏航(yaw)。
      例如：pitch=90 低头，yaw=45 向左转。
    """

    def __init__(
        self,
        link5_prim_path: str,
        name: str = "Link5DepthCamera",
        resolution: Tuple[int, int] = (640, 480),
        frequency: float = 20.0,
        offset_above_link5: Tuple[float, float, float] = (0.0, 0.0, 0.1),
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        yaw_deg: float = 0.0,
        euler_deg: Tuple[float, float, float] | None = None,
    ):
        """
        Args:
            link5_prim_path: link5 的 USD 路径，例如 "/World/IceCreamArm/link5"。
                若机械臂在子节点下（如 "/World/IceCreamArm/ice_cream_1_SLDASM/link5"），请传入完整路径。
            name: 相机 prim 名称（作为 link5 的子节点）。
            resolution: (width, height)。
            frequency: 采样频率 (Hz)。
            offset_above_link5: link5 局部系下的平移 (x, y, z)，默认 (0,0,0.1) 表示上方 10 cm。
            roll_deg: 绕 link5 局部 X 轴的滚转角（度）。
            pitch_deg: 绕 link5 局部 Y 轴的俯仰角（度）。
            yaw_deg: 绕 link5 局部 Z 轴的偏航角（度）。
            euler_deg: 若给定 (roll, pitch, yaw) 三元组，则覆盖上面三个单独参数。
        """
        self._link5_prim_path = link5_prim_path.rstrip("/")
        self._camera_prim_path = f"{self._link5_prim_path}/{name}"

        translation_local = np.array(offset_above_link5, dtype=float)
        if euler_deg is not None:
            euler = np.array(euler_deg, dtype=float)
        else:
            euler = np.array([roll_deg, pitch_deg, yaw_deg], dtype=float)
        q_local = rot_utils.euler_angles_to_quats(euler, degrees=True)

        self._camera = Camera(
            prim_path=self._camera_prim_path,
            translation=translation_local,
            orientation=q_local.astype(float),
            frequency=frequency,
            resolution=resolution,
        )

    @property
    def prim_path(self) -> str:
        return self._camera_prim_path

    @property
    def camera(self) -> Camera:
        return self._camera

    def initialize(self) -> None:
        self._camera.initialize()
        self._camera.set_clipping_range(0.01, 5.0)
        self._camera.set_focal_length(1.0)

    def add_distance_to_camera_to_frame(self) -> None:
        self._camera.add_distance_to_camera_to_frame()

    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """(position, orientation_quat_wxyz) in world frame."""
        return self._camera.get_world_pose()

    def get_current_frame(self) -> dict:
        return self._camera.get_current_frame()


# ---------------------------------------------------------------------------
# 主程序：机械臂 + link5 深度相机 + 键盘控制
# ---------------------------------------------------------------------------

def _main():
    _code_dir = os.path.dirname(os.path.abspath(__file__))
    if _code_dir not in sys.path:
        sys.path.insert(0, _code_dir)
    from icecream_driver import IceCreamArmDriver, JOINT_NAMES, NUM_JOINTS

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
    world.scene.add_default_ground_plane()

    driver = IceCreamArmDriver(world, usd_path=args.usd)
    world.reset()

    # link5 深度相机（装在 link5 上方，朝向由 roll/pitch/yaw 控制）
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
