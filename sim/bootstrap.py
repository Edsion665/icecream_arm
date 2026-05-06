"""Isaac Sim 场景辅助：USD 解析、payload 符号链接、Prim 调试。"""

from __future__ import annotations

import os
from typing import Any, Callable, Optional

import numpy as np

from ..config import DEFAULT_SIMULATION_CONFIG, SimulationConfig

LogFn = Callable[[str], None]


def resolve_sim_usd(package_dir: str, sim_usd_arg: str | None, sim_cfg: SimulationConfig = DEFAULT_SIMULATION_CONFIG) -> str:
    """解析仿真主 USD 路径：显式参数优先，否则在 ``configuration/`` 下按候选名查找。

    Args:
        package_dir: ``arm_control_bridge`` 包根目录绝对路径。
        sim_usd_arg: CLI ``--sim-usd``；为 ``None`` 时使用默认候选。
        sim_cfg: 仿真相关常量。

    Returns:
        存在的 USD 文件绝对路径。

    Raises:
        FileNotFoundError: 未找到任何候选文件。
    """
    if sim_usd_arg is not None:
        path = os.path.abspath(sim_usd_arg)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"仿真 USD 不存在: {path}")
        return path
    for name in sim_cfg.default_usd_candidates:
        p = os.path.join(package_dir, "configuration", name)
        if os.path.isfile(p):
            return p
    names = ", ".join(sim_cfg.default_usd_candidates)
    raise FileNotFoundError(
        f"未在 arm_control_bridge/configuration/ 找到 {names}，请用 --sim-usd 指定。"
    )


def ensure_v8_arm_payload_symlinks(
    usd_path: str,
    *,
    sim_cfg: SimulationConfig = DEFAULT_SIMULATION_CONFIG,
    on_warn: Optional[LogFn] = None,
) -> None:
    """为 ``ice_cream_v8_arm.usd`` 建立 ``configuration/*.usd`` 符号链，满足 Omniverse payload 解析。

    Args:
        usd_path: 已解析的 USD 绝对路径。
        sim_cfg: 含 v8 基名与子 USD 列表。
        on_warn: 创建失败时的回调（不再静默 ``pass``）。
    """
    if os.path.basename(usd_path) != sim_cfg.v8_payload_basename:
        return
    parent = os.path.dirname(os.path.abspath(usd_path))
    nested = os.path.join(parent, "configuration")
    for name in sim_cfg.v8_payload_children:
        src = os.path.join(parent, name)
        dst = os.path.join(nested, name)
        if not os.path.isfile(src) or os.path.lexists(dst):
            continue
        try:
            os.makedirs(nested, exist_ok=True)
            os.symlink(os.path.join("..", name), dst)
        except OSError as exc:
            msg = f"[sim] v8 payload symlink 失败: {dst} <- {src} ({type(exc).__name__}: {exc})"
            if on_warn is not None:
                on_warn(msg)


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


def set_marker_xyz(
    stage: Any,
    xyz: np.ndarray,
    *,
    sim_cfg: SimulationConfig = DEFAULT_SIMULATION_CONFIG,
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
