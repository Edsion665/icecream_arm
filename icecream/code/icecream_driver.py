#!/usr/bin/env python3
"""
Ice Cream 机械臂控制接口。
便于按关节设置目标角度（弧度或角度），每步调用 apply() 同步到仿真。

使用示例:
    from isaacsim.core.api import World
    from icecream_driver import IceCreamArmDriver, NUM_JOINTS

    world = World(...)
    world.scene.add_default_ground_plane()
    driver = IceCreamArmDriver(world, usd_path="/path/to/ice_cream_arm.usd")
    world.reset()

    # 在仿真循环中：
    driver.set_joint_positions([0.1, 0.2, 0, 0, 0])   # 弧度
    driver.set_joint_position(0, 0.5)                  # 仅关节 0
    driver.set_joint_positions_deg([10, 20, 0, 0, 0])  # 度
    driver.home()                                       # 归零
    driver.apply()                                      # 下发目标
    world.step(render=True)
"""
from __future__ import annotations

import os
from typing import List, Optional, Union

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction

# 关节名（与 URDF 一致）
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]
NUM_JOINTS = 5

# 默认关节限位（弧度，来自 URDF）
JOINT_LIMITS_LOWER = np.array([-3.14] * NUM_JOINTS)
JOINT_LIMITS_UPPER = np.array([3.14] * NUM_JOINTS)

# 默认 PD 增益（位置控制）
DEFAULT_KP = 1e4
DEFAULT_KD = 1e3


class IceCreamArmDriver:
    """
    Ice Cream 机械臂驱动接口：设置各关节目标角度，每步 apply() 下发到控制器。
    """

    def __init__(
        self,
        world: World,
        usd_path: Optional[str] = None,
        prim_path: str = "/World/IceCreamArm",
        name: str = "ice_cream_arm",
        kp: Optional[Union[float, List[float], np.ndarray]] = None,
        kd: Optional[Union[float, List[float], np.ndarray]] = None,
    ):
        """
        Args:
            world: Isaac Sim World 实例。
            usd_path: 机械臂 USD 文件路径；若为 None，使用同目录下的 ice_cream_arm.usd。
            prim_path: 场景中的 prim 路径。
            name: 加入 scene 时的名称。
            kp, kd: 关节位置控制刚度/阻尼；为 None 时用默认值。
        """
        if usd_path is None:
            code_dir = os.path.dirname(os.path.abspath(__file__))
            usd_path = os.path.join(code_dir, "ice_cream_arm.usd")
        usd_path = os.path.abspath(usd_path)
        if not os.path.isfile(usd_path):
            raise FileNotFoundError(f"未找到机械臂 USD: {usd_path}，请先运行 urdf_to_usd.py 生成。")

        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        self._arm = world.scene.add(SingleArticulation(prim_path=prim_path, name=name))
        self._world = world
        self._prim_path = prim_path
        self._controller = self._arm.get_articulation_controller()

        # 目标关节角（弧度），与当前 DOF 顺序一致
        self._target_positions = np.zeros(NUM_JOINTS, dtype=float)

        def _gains_array(val: Optional[Union[float, List[float], np.ndarray]], default: float) -> np.ndarray:
            if val is None:
                return np.full(NUM_JOINTS, default, dtype=float)
            arr = np.atleast_1d(np.asarray(val, dtype=float))
            if arr.size == 1:
                return np.full(NUM_JOINTS, float(arr.flat[0]), dtype=float)
            if arr.size != NUM_JOINTS:
                raise ValueError(f"增益需为标量或长度为 {NUM_JOINTS} 的数组，得到长度 {arr.size}")
            return np.asarray(arr.flat[:NUM_JOINTS], dtype=float)

        kp_arr = _gains_array(kp, DEFAULT_KP)
        kd_arr = _gains_array(kd, DEFAULT_KD)
        try:
            self._controller.set_gains(kps=kp_arr, kds=kd_arr)
        except Exception:
            pass

    def set_joint_positions(self, positions: Union[List[float], np.ndarray]) -> None:
        """设置全部关节目标角度（弧度）。"""
        arr = np.asarray(positions, dtype=float)
        if arr.size != NUM_JOINTS:
            raise ValueError(f"需要 {NUM_JOINTS} 个关节角，得到 {arr.size}")
        self._target_positions = np.clip(arr.flat[:NUM_JOINTS], JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)

    def set_joint_position(self, joint_index: int, value_rad: float) -> None:
        """设置单个关节目标角度（弧度），joint_index 为 0~4。"""
        if not 0 <= joint_index < NUM_JOINTS:
            raise IndexError(f"joint_index 应在 0~{NUM_JOINTS - 1}，得到 {joint_index}")
        self._target_positions[joint_index] = np.clip(
            float(value_rad),
            JOINT_LIMITS_LOWER[joint_index],
            JOINT_LIMITS_UPPER[joint_index],
        )

    def set_joint_positions_deg(self, positions_deg: Union[List[float], np.ndarray]) -> None:
        """设置全部关节目标角度（度）。"""
        self.set_joint_positions(np.deg2rad(np.asarray(positions_deg)))

    def set_joint_position_deg(self, joint_index: int, value_deg: float) -> None:
        """设置单个关节目标角度（度）。"""
        self.set_joint_position(joint_index, np.deg2rad(value_deg))

    def get_joint_positions(self) -> np.ndarray:
        """当前关节角度（弧度），形状 (NUM_JOINTS,)。"""
        return np.array(self._arm.get_joint_positions(), dtype=float).flat[:NUM_JOINTS]

    def get_joint_positions_deg(self) -> np.ndarray:
        """当前关节角度（度）。"""
        return np.rad2deg(self.get_joint_positions())

    def get_target_positions(self) -> np.ndarray:
        """当前目标关节角（弧度）。"""
        return self._target_positions.copy()

    def home(self) -> None:
        """将目标设为全零（弧度）。"""
        self._target_positions.fill(0.0)

    def apply(self) -> None:
        """将当前目标关节角下发到控制器，需在每步 world.step() 前调用。"""
        self._controller.apply_action(ArticulationAction(joint_positions=self._target_positions.tolist()))

    @property
    def articulation(self) -> SingleArticulation:
        """底层 Articulation，用于高级用法。"""
        return self._arm

    @property
    def joint_names(self) -> List[str]:
        return list(JOINT_NAMES)


# ---------------------------------------------------------------------------
# 命令行简单演示
# ---------------------------------------------------------------------------

def _main():
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": False})

    import argparse
    parser = argparse.ArgumentParser(description="Ice Cream 机械臂驱动接口演示")
    parser.add_argument("--usd", type=str, default=None, help="机械臂 USD 路径")
    parser.add_argument("--test", action="store_true", help="跑一轮演示后退出")
    args, _ = parser.parse_known_args()

    physics_dt = 1.0 / 60.0
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=physics_dt)
    world.scene.add_default_ground_plane()

    driver = IceCreamArmDriver(world, usd_path=args.usd)
    world.reset()

    waypoints_rad = [
        np.zeros(NUM_JOINTS),
        np.array([0.5, 0.2, -0.3, 0.1, 0.0]),
        np.array([-0.3, 0.4, 0.2, -0.2, 0.1]),
        np.zeros(NUM_JOINTS),
    ]
    step = 0
    idx = 0
    steps_per = 120
    max_steps = (steps_per * len(waypoints_rad)) if args.test else None
    total = 0

    while simulation_app.is_running():
        world.step(render=True)
        if not world.is_playing():
            continue
        step += 1
        if step >= steps_per:
            step = 0
            idx = (idx + 1) % len(waypoints_rad)
        driver.set_joint_positions(waypoints_rad[idx])
        driver.apply()
        total += 1
        if max_steps and total >= max_steps:
            break

    simulation_app.close()


if __name__ == "__main__":
    _main()
