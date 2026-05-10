"""Isaac Sim 场景辅助：Grid 地面、Articulation 查找、Prim 调试。"""

from __future__ import annotations

import os
from typing import Any, Callable, Optional

import numpy as np

from ..config import SIM_CONFIG, SimulationConfig

LogFn = Callable[[str], None]


def find_articulation_root(stage: Any, under_path: str) -> Optional[str]:
    """在 ``under_path`` 子树中查找带 ``ArticulationRootAPI`` 的 prim 路径。"""
    try:
        from pxr import UsdPhysics
    except ImportError:
        return None
    prim = stage.GetPrimAtPath(under_path)
    if not prim.IsValid():
        return None
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return under_path
    for child in prim.GetAllChildren():
        path = child.GetPath().pathString
        if child.HasAPI(UsdPhysics.ArticulationRootAPI):
            return path
        found = find_articulation_root(stage, path)
        if found:
            return found
    return None


def log_prim_world_pose(stage: Any, prim_path: str, tag: str, log: LogFn) -> None:
    """打印 prim 世界系平移（调试用）。"""
    try:
        from pxr import Usd, UsdGeom

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            log(f"[sim][frame] {tag}: prim not found: {prim_path}")
            return
        xf = UsdGeom.Xformable(prim)
        m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        log(
            f"[sim][frame] {tag}: path={prim_path} world_t=({float(t[0]):+.4f}, {float(t[1]):+.4f}, {float(t[2]):+.4f})"
        )
    except Exception as exc:
        log(f"[sim][frame] {tag}: failed to read world pose: {type(exc).__name__}: {exc}")


def apply_arm_prim_world_z_offset(
    stage: Any,
    prim_path: str,
    dz_m: float,
    *,
    log: Optional[LogFn] = None,
) -> None:
    """在手臂根 prim 的 Xform 栈上追加沿父空间 +Z 的平移（米）。

    根 prim 的父级一般为 ``/World`` 时，效果为世界坐标系竖直向上平移整臂（link0 随动）。
    """
    dz = float(dz_m)
    if abs(dz) < 1e-12:
        return
    try:
        from pxr import Gf, UsdGeom

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            if log is not None:
                log(f"[sim][arm_z] prim not found: {prim_path}")
            return
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp(opSuffix="world_z_offset").Set(Gf.Vec3d(0.0, 0.0, dz))
        if log is not None:
            log(f"[sim][arm_z] {prim_path} append translate (0, 0, {dz:+.4f}) m")
    except Exception as exc:
        if log is not None:
            log(f"[sim][arm_z] failed: {type(exc).__name__}: {exc}")


def set_marker_xyz(
    stage: Any,
    xyz: np.ndarray,
    *,
    sim_cfg: SimulationConfig = SIM_CONFIG,
    log: Optional[LogFn] = None,
) -> None:
    """将目标 marker prim 平移到 ``xyz``（米）+ z 抬升。"""
    try:
        from pxr import Gf, UsdGeom

        prim = stage.GetPrimAtPath(sim_cfg.target_marker_path)
        if not prim.IsValid():
            if log is not None:
                log(f"[sim][marker] prim not found: {sim_cfg.target_marker_path}")
            return
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(
            Gf.Vec3d(
                float(xyz[0]),
                float(xyz[1]),
                float(xyz[2]) + sim_cfg.marker_z_lift_m,
            )
        )
    except Exception as exc:
        if log is not None:
            log(f"[sim][marker] set_marker_xyz failed: {type(exc).__name__}: {exc}")


def add_grid_ground(world: Any, package_dir: str, *, on_log: Optional[LogFn] = None) -> Any:
    """从包内 ``configuration/Isaac/Environments/Grid/default_environment.usd`` 加载默认 Grid 地面（与官方布局一致）。"""
    name = "default_ground_plane"
    prim_path = "/World/defaultGroundPlane"
    if world.scene.object_exists(name=name):
        return world.scene.get_object(name=name)
    usd_path = os.path.join(
        package_dir, "configuration", "Isaac", "Environments", "Grid", "default_environment.usd"
    )
    if not os.path.isfile(usd_path):
        raise FileNotFoundError(
            "缺少本地 Grid 地面 USD，请将 default_environment.usd 及同目录依赖放入: "
            f"{os.path.dirname(usd_path)}/"
        )
    usd_path = os.path.abspath(usd_path)
    try:
        from isaacsim.core.api.materials.physics_material import PhysicsMaterial
        from isaacsim.core.api.objects.ground_plane import GroundPlane
        from isaacsim.core.utils.prims import is_prim_path_valid
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.string import find_unique_string_name
    except ModuleNotFoundError:
        from omni.isaac.core.materials.physics_material import PhysicsMaterial
        from omni.isaac.core.objects.ground_plane import GroundPlane
        from omni.isaac.core.utils.prims import is_prim_path_valid
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.utils.string import find_unique_string_name

    if on_log is not None:
        on_log(f"[sim] Grid 地面: {usd_path}")
    add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
    physics_material_path = find_unique_string_name(
        initial_name="/World/Physics_Materials/physics_material",
        is_unique_fn=lambda x: not is_prim_path_valid(x),
    )
    physics_material = PhysicsMaterial(
        prim_path=physics_material_path,
        static_friction=0.5,
        dynamic_friction=0.5,
        restitution=0.8,
    )
    plane = GroundPlane(prim_path=prim_path, name=name, z_position=0.0, physics_material=physics_material)
    world.scene.add(plane)
    return plane
