#!/usr/bin/env python3

"""
Ice Cream 机械臂与 link5 深度相机驱动
"""

from __future__ import annotations

import os
from typing import List, Optional, Tuple, Union

import carb
import numpy as np
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera


JOINT_NAMES = ["joint0", "joint1", "joint2", "joint3", "joint4"]
NUM_JOINTS = 5

# 默认关节限位（弧度，来自 URDF）
JOINT_LIMITS_LOWER = np.array([-3.14] * NUM_JOINTS)
JOINT_LIMITS_UPPER = np.array([3.14] * NUM_JOINTS)

# 默认关节速度限位（弧度/秒，来自 URDF velocity）
JOINT_VELOCITY_LIMIT = 0.3

# 默认 PD 增益（位置控制）
DEFAULT_KP = 1e8
DEFAULT_KD = 1e3


class Link5DepthCamera:
    """
    安装在 link5 上方的深度相机（刚性随 link5 运动）。
    - 默认在 link5 局部坐标系下沿 +Z 偏移 10 cm（“上方”）。
    - 朝向由 link5 局部系下的欧拉角 (roll, pitch, yaw) 控制，单位：度。
      顺序：绕 X → Y → Z 固定轴（extrinsic），对应滚转(roll)、俯仰(pitch)、偏航(yaw)。
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
        euler_deg: Optional[Tuple[float, float, float]] = None,
    ):
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


class DepthCameraObjectMeasurer:
    """
    利用 Link5DepthCamera 对应的底层 Camera 对象，提供：
      1) 世界坐标 → 图像像素坐标（world_to_pixel）
      2) 图像像素坐标 + 深度 → 世界坐标（pixel_depth_to_world）
    """

    def __init__(self, link5_camera: Link5DepthCamera):
        # Link5DepthCamera 封装，内部持有真正的 Camera 传感器
        self._link5_camera = link5_camera
        self._cam = link5_camera.camera

    def world_to_pixel(self, world_point: np.ndarray, image_shape=None):
        """
        世界坐标 → 深度图像素坐标（不依赖具体深度值）。

        Args:
            world_point: 形状 (3,) 的世界坐标 (x, y, z)。
            image_shape: 可选，(H, W)；若提供则同时返回 in_bounds 标志。

        Returns:
            dict:
                {
                    "uv": (u, v),          # 浮点像素坐标
                    "pixel": (u_i, v_i),   # 四舍五入后的整数像素坐标
                    "in_bounds": bool,     # 若给了 image_shape，则表示像素是否在图像范围内
                }
        """
        wp = np.asarray(world_point, dtype=float).reshape(1, 3)
        img_coords = self._cam.get_image_coords_from_world_points(wp)  # (1, 2)
        u, v = img_coords[0]
        u_i = int(round(u))
        v_i = int(round(v))

        in_bounds = True
        if image_shape is not None:
            h, w = int(image_shape[0]), int(image_shape[1])
            in_bounds = (0 <= v_i < h) and (0 <= u_i < w)

        return {
            "uv": (float(u), float(v)),
            "pixel": (u_i, v_i),
            "in_bounds": in_bounds,
        }

    def pixel_depth_to_world(self, u: float, v: float, depth: float):
        """
        已知像素坐标 (u, v) 和该像素的深度值 depth（米），
        利用相机内外参还原世界坐标。

        你可以用任意方法得到 (u, v) 和 depth，只需要保证它们与当前相机/深度图一致。
        """
        uv = np.array([[float(u), float(v)]], dtype=float)
        d = np.array([float(depth)], dtype=float)
        pts_world = self._cam.get_world_points_from_image_coords(uv, d)  # (1, 3)
        return np.array(pts_world[0], dtype=float)


class IceCreamArmDriver:
    """
    Ice Cream 机械臂驱动接口：设置各关节目标角度/速度，每步 apply() 下发到控制器。
    可同时设位置与速度（速度作为前馈）；若未设速度则仅位置控制。
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
            raise FileNotFoundError(f"not found arm USD: {usd_path}, please run urdf_to_usd.py to generate.")

        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
        self._arm = world.scene.add(SingleArticulation(prim_path=prim_path, name=name))
        self._world = world
        self._prim_path = prim_path
        self._controller = self._arm.get_articulation_controller()

        # 目标关节角（弧度）、目标关节角速度（弧度/秒）；速度为 None 表示不发送速度
        self._target_positions = np.zeros(NUM_JOINTS, dtype=float)
        self._target_velocities: Optional[np.ndarray] = None

        def _gains_array(val: Optional[Union[float, List[float], np.ndarray]], default: float) -> np.ndarray:
            if val is None:
                return np.full(NUM_JOINTS, default, dtype=float)
            arr = np.atleast_1d(np.asarray(val, dtype=float))
            if arr.size == 1:
                return np.full(NUM_JOINTS, float(arr.flat[0]), dtype=float)
            if arr.size != NUM_JOINTS:
                raise ValueError(f"gains should be a scalar or an array of length {NUM_JOINTS}, got length {arr.size}")
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
            raise ValueError(f"need {NUM_JOINTS} joints angles, got {arr.size}")
        self._target_positions = np.clip(arr.flat[:NUM_JOINTS], JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)

    def set_joint_position(self, joint_index: int, value_rad: float) -> None:
        """设置单个关节目标角度（弧度），joint_index 为 0~4。"""
        if not 0 <= joint_index < NUM_JOINTS:
            raise IndexError(f"joint_index should be in 0~{NUM_JOINTS - 1}, got {joint_index}")
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

    def set_joint_velocities(self, velocities: Union[List[float], np.ndarray]) -> None:
        """设置全部关节目标角速度（弧度/秒）；会裁剪到 ±JOINT_VELOCITY_LIMIT。"""
        arr = np.asarray(velocities, dtype=float)
        if arr.size != NUM_JOINTS:
            raise ValueError(f"need {NUM_JOINTS} joints velocities, got {arr.size}")
        if self._target_velocities is None:
            self._target_velocities = np.zeros(NUM_JOINTS, dtype=float)
        self._target_velocities[:] = np.clip(arr.flat[:NUM_JOINTS], -JOINT_VELOCITY_LIMIT, JOINT_VELOCITY_LIMIT)

    def set_joint_velocity(self, joint_index: int, value_rad_per_s: float) -> None:
        """设置单个关节目标角速度（弧度/秒），joint_index 为 0~4。"""
        if not 0 <= joint_index < NUM_JOINTS:
            raise IndexError(f"joint_index should be in 0~{NUM_JOINTS - 1}, got {joint_index}")
        if self._target_velocities is None:
            self._target_velocities = np.zeros(NUM_JOINTS, dtype=float)
        self._target_velocities[joint_index] = np.clip(
            float(value_rad_per_s), -JOINT_VELOCITY_LIMIT, JOINT_VELOCITY_LIMIT
        )

    def set_joint_velocities_deg(self, velocities_deg_per_s: Union[List[float], np.ndarray]) -> None:
        """设置全部关节目标角速度（度/秒）。"""
        self.set_joint_velocities(np.deg2rad(np.asarray(velocities_deg_per_s)))

    def set_joint_velocity_deg(self, joint_index: int, value_deg_per_s: float) -> None:
        """设置单个关节目标角速度（度/秒）。"""
        self.set_joint_velocity(joint_index, np.deg2rad(value_deg_per_s))

    def get_joint_velocities(self) -> np.ndarray:
        """当前关节角速度（弧度/秒），形状 (NUM_JOINTS,)。"""
        return np.array(self._arm.get_joint_velocities(), dtype=float).flat[:NUM_JOINTS]

    def get_joint_velocities_deg(self) -> np.ndarray:
        """当前关节角速度（度/秒）。"""
        return np.rad2deg(self.get_joint_velocities())

    def get_target_velocities(self) -> Optional[np.ndarray]:
        """当前目标关节速度（弧度/秒）；未设置时为 None。"""
        return self._target_velocities.copy() if self._target_velocities is not None else None

    def clear_velocity_targets(self) -> None:
        """清除速度目标，之后 apply() 只发位置。"""
        self._target_velocities = None

    def home(self) -> None:
        """将位置目标设为全零（弧度），并清除速度目标。"""
        self._target_positions.fill(0.0)
        self._target_velocities = None

    def apply(self) -> None:
        """将当前目标关节角（及速度，若已设）下发到控制器，需在每步 world.step() 前调用。"""
        action = ArticulationAction(
            joint_positions=self._target_positions.tolist(),
            joint_velocities=self._target_velocities.tolist() if self._target_velocities is not None else None,
        )
        self._controller.apply_action(action)

    @property
    def articulation(self) -> SingleArticulation:
        """底层 Articulation，用于高级用法。"""
        return self._arm

    @property
    def joint_names(self) -> List[str]:
        return list(JOINT_NAMES)

