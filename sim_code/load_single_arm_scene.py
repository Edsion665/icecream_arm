#!/usr/bin/env python3
"""
仅加载机械臂 USD 并生成场景（与 icecream_3Dof-test.py 相同的加载方式）。
默认加载 ice_cream_single_arm.usd；可用 --usd 指定其他 USD。

说明：ice_cream_single_arm.usd 由 ice_cream_SINGLE.SLDASM.urdf 导出。该 URDF 中
joint1~joint5 均为 type="fixed"（固定关节），无转动自由度，故导入 USD 后关节数为 0，
无法做关节控制。若需在仿真中控制 single 模型关节，需将源 URDF 中相应关节改为
type="revolute" 并设置正确的 axis/limit，再重新运行 urdf_to_usd.py --model SINGLE。

用法（在项目根目录下，用 Isaac Sim 的 python.sh 运行）：
  ~/isaac-sim/python.sh sim_code/load_single_arm_scene.py
  ~/isaac-sim/python.sh sim_code/load_single_arm_scene.py --usd sim_code/ice_cream_arm.usd
"""
import argparse
import os
import sys

parser = argparse.ArgumentParser(description="加载机械臂 USD 并生成场景（与 3Dof-test 同方式）")
parser.add_argument("--usd", type=str, default=None,
                    help="机械臂 USD 路径（默认同目录 ice_cream_single_arm.usd）")
parser.add_argument("--headless", action="store_true", help="无界面模式")
parser.add_argument("--no-control", action="store_true", help="不施加关节控制，仅步进仿真")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": args.headless})

# Isaac Sim 4.2 等版本可能暴露 omni.isaac.core 而非 isaacsim.core，两路都试
try:
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction
except ModuleNotFoundError:
    try:
        from omni.isaac.core import World
        from omni.isaac.core.prims import SingleArticulation
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.utils.types import ArticulationAction
    except ModuleNotFoundError as e:
        print("错误：未找到 isaacsim.core 与 omni.isaac.core，请使用 Isaac Sim 的 python.sh 运行。")
        print("例如：~/isaac-sim/python.sh sim_code/load_single_arm_scene.py")
        simulation_app.close()
        sys.exit(1)

import numpy as np
import omni.usd

_CODE_DIR = os.path.dirname(os.path.abspath(os.path.realpath(__file__)))
_PROJECT_ROOT = os.path.dirname(_CODE_DIR)

# 与 icecream_3Dof-test.py 一致：地面 prim 与默认地面 USD
LOCAL_GROUND_PRIM_PATH = "/World/LocalGround"
DEFAULT_GROUND_USD_PATH = "/home/huangjianan/isaacsim_assets/Assets/Isaac/4.5/Isaac/Environments/Grid/default_environment.usd"
ARM_PRIM_PATH = "/World/IceCreamArm"
# 机械臂基坐标系在场景中的固定位置（世界系，米）
ARM_BASE_POSITION = (0.0, 0.0, 0.0)


def _find_articulation_root_prim_path(stage, under_path: str):
    """
    从 under_path 起向下查找第一个带 ArticulationRootAPI 的 prim 路径。
    URDF 导入后 ArticulationRoot 通常在子 prim（如 link0）上，用引用根创建 SingleArticulation 会得到 0 DOF。
    返回该 prim 路径；找不到则返回 None。
    """
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
        found = _find_articulation_root_prim_path(stage, path)
        if found:
            return found
    return None


def _resolve_usd(usd_arg):
    """解析机械臂 USD：--usd 指定则用该路径，否则在 sim_code 下查找 ice_cream_single_arm.usd。"""
    if usd_arg is not None:
        path = os.path.abspath(usd_arg)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"USD 文件不存在: {path}")
        return path
    candidates = [
        os.path.join(_CODE_DIR, "ice_cream_single_arm.usd"),
        os.path.join(_PROJECT_ROOT, "sim_code", "ice_cream_single_arm.usd"),
        os.path.join(_CODE_DIR, "ice_cream_arm.usd"),
        os.path.join(_PROJECT_ROOT, "sim_code", "ice_cream_arm.usd"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    raise FileNotFoundError(
        "未找到 ice_cream_single_arm.usd / ice_cream_arm.usd，请用 --usd 指定路径。"
    )


def _add_local_ground_plane(stage=None):
    """在当前 Stage 创建本地地面（与 3Dof-test 一致）。"""
    from pxr import UsdGeom, Gf
    if stage is None:
        stage = omni.usd.get_context().get_stage()
    path = LOCAL_GROUND_PRIM_PATH
    try:
        from pxr import PhysicsSchemaTools
        PhysicsSchemaTools.addGroundPlane(
            stage, path, "Z",
            15.0, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5, 0.5, 0.5)
        )
        print(f"[ground] 使用本地地面: {path} (PhysicsSchemaTools)")
        return
    except Exception as e:
        print(f"[ground] PhysicsSchemaTools 不可用: {type(e).__name__}: {e}")
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddScaleOp().Set(Gf.Vec3d(25.0, 25.0, 0.02))
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.01))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.5, 0.5, 0.55)])
    try:
        from pxr import UsdPhysics
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    except Exception:
        pass
    print(f"[ground] 使用本地地面: {path} (Cube，上表面 z=0)")


def main():
    physics_dt = 1.0 / 60.0
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=physics_dt,
        rendering_dt=physics_dt,
    )

    # 地面：与 icecream_3Dof-test.py 相同
    stage_for_scene = omni.usd.get_context().get_stage()
    if os.path.isfile(DEFAULT_GROUND_USD_PATH):
        add_reference_to_stage(usd_path=DEFAULT_GROUND_USD_PATH, prim_path=LOCAL_GROUND_PRIM_PATH)
        print(f"[ground] 使用地面 USD: {DEFAULT_GROUND_USD_PATH} -> {LOCAL_GROUND_PRIM_PATH}")
    else:
        print(f"[ground] 地面 USD 不存在: {DEFAULT_GROUND_USD_PATH}，回退到程序建地面")
        _add_local_ground_plane(stage_for_scene)

    # 加载机械臂 USD：引用根 + root_joint 时用父路径创建 SingleArticulation（与 ice_cream_single_test 一致）
    usd_path = _resolve_usd(args.usd)
    add_reference_to_stage(usd_path=usd_path, prim_path=ARM_PRIM_PATH)
    prim = stage_for_scene.GetPrimAtPath(ARM_PRIM_PATH)
    if prim.IsValid():
        from pxr import UsdGeom, Gf
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*ARM_BASE_POSITION))
    found_root = _find_articulation_root_prim_path(stage_for_scene, ARM_PRIM_PATH)
    if found_root and found_root.endswith("/root_joint"):
        articulation_path = found_root[: -len("/root_joint")].rstrip("/") or ARM_PRIM_PATH
    else:
        articulation_path = found_root if found_root else ARM_PRIM_PATH
    arm = world.scene.add(SingleArticulation(articulation_path, name="ice_cream_arm"))
    world.reset()

    q0 = arm.get_joint_positions()
    n_dof = len(q0) if q0 is not None else (getattr(arm, "num_dof", None) or 0)
    if n_dof == 0 and ("ice_cream_single_arm" in usd_path or usd_path.endswith("ice_cream_single_arm.usd")):
        print("[scene] 当前 single 模型识别为 0 DOF（源 URDF 中 joint1~5 均为 type=\"fixed\"），关节控制不可用。")
        print("        临时方案：使用 5-DOF 模型 --usd sim_code/ice_cream_arm.usd")
        print("        长期方案：将 ice_cream_SINGLE.SLDASM.urdf 中关节改为 type=\"revolute\" 后重新导出 USD。")
    print(f"[scene] 机械臂 USD: {usd_path}")
    print(f"[scene] 基座位置（世界系）: x={ARM_BASE_POSITION[0]}, y={ARM_BASE_POSITION[1]}, z={ARM_BASE_POSITION[2]}")
    print(f"[scene] Articulation 路径: {articulation_path}, 关节数: {n_dof}")

    # 关节控制器与目标（位置控制）
    controller = None
    waypoint_index = 0
    step_count = 0
    if n_dof > 0 and not args.no_control:
        controller = arm.get_articulation_controller()
        try:
            controller.set_gains(
                kps=np.full(n_dof, 1e4, dtype=float),
                kds=np.full(n_dof, 1e3, dtype=float),
            )
        except Exception:
            pass
        # 示例：若干目标关节角（弧度），循环执行。SINGLE 模型有 5 个臂关节 + 2 个夹爪关节=7 DOF，需补齐长度
        arm_dof = min(5, n_dof)
        pad = n_dof - arm_dof
        def _waypoint(arm_vals):
            return np.concatenate([np.asarray(arm_vals[:arm_dof], dtype=float), np.zeros(pad)])
        waypoints_rad = [
            np.zeros(n_dof),
            _waypoint([0.5, 0.2, -0.3, 0.1, 0.0]),
            _waypoint([-0.3, 0.4, 0.2, -0.2, 0.1]),
            np.zeros(n_dof),
        ]
        steps_per_waypoint = 120  # 约 2 秒切换一个位姿
        print(f"[control] 关节位置控制已开启，{len(waypoints_rad)} 个路径点循环（每 {steps_per_waypoint} 步切换）")
    else:
        if args.no_control:
            print("[control] --no-control：不施加关节控制")
        else:
            print("[control] 关节数为 0，跳过控制")

    while simulation_app.is_running():
        world.step(render=True)

        if not world.is_playing():
            continue

        # 关节位置控制：每步下发当前路径点目标
        if controller is not None:
            step_count += 1
            if step_count >= steps_per_waypoint:
                step_count = 0
                waypoint_index = (waypoint_index + 1) % len(waypoints_rad)
            target_positions = waypoints_rad[waypoint_index]
            controller.apply_action(ArticulationAction(joint_positions=target_positions.tolist()))

    simulation_app.close()


if __name__ == "__main__":
    main()
