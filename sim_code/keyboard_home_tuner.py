#!/usr/bin/env python3
"""
实时键盘调节 5 个关节角度，交互式寻找合适的 HOME 姿态。

特点：
- 默认加载 SINGLE 模型导出的 `ice_cream_single_arm.usd`，也可用 --usd 指定其他 USD；
- 自动适配 5 DOF 或 7 DOF（5 个臂关节 + 夹爪），只控制前 5 个关节，夹爪保持 0；
- 仿真持续运行，在 3D 场景中实时显示机械臂，你按键盘立即看到关节变化。

按键映射（所有角度单位：度，每次增量由 --step-deg 控制，默认 2°）：
  关节 1:  Q (+) / A (-)
  关节 2:  W (+) / S (-)
  关节 3:  E (+) / D (-)
  关节 4:  R (+) / F (-)
  关节 5:  T (+) / G (-)
  H:       所有关节回到 0°
  P:       在终端打印当前 5 个关节的角度（度）
  ESC:     退出脚本

用法（在项目根目录，用 Isaac Sim 的 python.sh 运行）：
  ~/isaac-sim/python.sh sim_code/keyboard_home_tuner.py
  ~/isaac-sim/python.sh sim_code/keyboard_home_tuner.py --usd sim_code/ice_cream_single_arm.usd
"""

import argparse
import os
import sys
from typing import List

import numpy as np

import carb
import omni.appwindow
from isaacsim import SimulationApp


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="实时键盘控制 5 个关节角度，用于交互式调节 HOME 姿态"
    )
    parser.add_argument(
        "--usd",
        type=str,
        default=None,
        help="机械臂 USD 路径（默认优先使用 sim_code/ice_cream_single_arm.usd，其次 ice_cream_arm.usd）",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="无界面模式（适合只关心数值，不看 3D 画面）",
    )
    parser.add_argument(
        "--step-deg",
        type=float,
        default=2.0,
        help="每次按键时关节角度增量（度，默认 2°）",
    )
    return parser.parse_args()


def _import_world_and_prims():
    """
    适配不同版本 Isaac Sim：优先 isaacsim.core.api，其次 omni.isaac.core。
    返回 (World, SingleArticulation, add_reference_to_stage, ArticulationAction, omni.usd)。
    """
    try:
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.types import ArticulationAction
        import omni.usd

        return World, SingleArticulation, add_reference_to_stage, ArticulationAction, omni.usd
    except ModuleNotFoundError:
        try:
            from omni.isaac.core import World
            from omni.isaac.core.prims import SingleArticulation
            from omni.isaac.core.utils.stage import add_reference_to_stage
            from omni.isaac.core.utils.types import ArticulationAction
            import omni.usd

            return World, SingleArticulation, add_reference_to_stage, ArticulationAction, omni.usd
        except ModuleNotFoundError as e:
            print("错误：未找到 isaacsim.core 与 omni.isaac.core，请使用 Isaac Sim 的 python.sh 运行本脚本。")
            print("例如：~/isaac-sim/python.sh sim_code/keyboard_home_tuner.py")
            raise e


def _resolve_usd(code_dir: str, project_root: str, usd_arg: str | None) -> str:
    """解析机械臂 USD：--usd 指定则用该路径，否则在常见位置查找。"""
    if usd_arg is not None:
        path = os.path.abspath(usd_arg)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"USD 文件不存在: {path}")
        return path

    candidates = [
        os.path.join(code_dir, "ice_cream_single_arm.usd"),
        os.path.join(project_root, "sim_code", "ice_cream_single_arm.usd"),
        os.path.join(code_dir, "ice_cream_arm.usd"),
        os.path.join(project_root, "sim_code", "ice_cream_arm.usd"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    raise FileNotFoundError(
        "未找到 ice_cream_single_arm.usd / ice_cream_arm.usd，请用 --usd 指定路径。"
    )


def _find_articulation_root_prim_path(stage, under_path: str) -> str | None:
    """
    从 under_path 起向下查找第一个带 ArticulationRootAPI 的 prim 路径。
    URDF 导入后 ArticulationRoot 通常在子 prim（如 link0 或 root_joint）上。
    """
    try:
        from pxr import UsdPhysics
    except Exception:
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


def _add_ground(stage) -> None:
    """简单地创建一个本地地面平面。"""
    from pxr import UsdGeom, Gf, UsdPhysics

    path = "/World/LocalGround"
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddScaleOp().Set(Gf.Vec3d(25.0, 25.0, 0.02))
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.01))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.5, 0.5, 0.55)])
    try:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    except Exception:
        pass


def _to_full_joint_vector(q_arm: np.ndarray, n_dof: int) -> np.ndarray:
    """将前 5 个臂关节角度补齐为 n_dof（多余 DOF 置 0，用于带夹爪的模型）。"""
    arm_dof = min(5, q_arm.size)
    q = np.zeros(n_dof, dtype=float)
    q[:arm_dof] = q_arm[:arm_dof]
    return q


def _print_joint_state(names: List[str], q_arm_deg: np.ndarray) -> None:
    print("\n当前关节角（度）：")
    for i in range(min(5, q_arm_deg.size)):
        name = names[i] if i < len(names) else f"joint{i+1}"
        print(f"  {i+1}: {name:>10s} = {q_arm_deg[i]:8.3f}°")
    print()


def main() -> None:
    args = _parse_args()

    simulation_app = SimulationApp({"headless": args.headless})
    World, SingleArticulation, add_reference_to_stage, ArticulationAction, omni_usd = _import_world_and_prims()

    physics_dt = 1.0 / 60.0
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=physics_dt,
        rendering_dt=physics_dt,
    )

    code_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(code_dir)
    usd_path = _resolve_usd(code_dir, project_root, args.usd)

    stage = omni_usd.get_context().get_stage()
    _add_ground(stage)

    arm_prim_path = "/World/IceCreamArm"
    add_reference_to_stage(usd_path=usd_path, prim_path=arm_prim_path)

    prim = stage.GetPrimAtPath(arm_prim_path)
    if prim.IsValid():
        from pxr import UsdGeom, Gf

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        # 基座放在世界原点
        xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

    found_root = _find_articulation_root_prim_path(stage, arm_prim_path)
    if found_root and found_root.endswith("/root_joint"):
        articulation_path = found_root[: -len("/root_joint")].rstrip("/") or arm_prim_path
    else:
        articulation_path = found_root or arm_prim_path

    arm = world.scene.add(SingleArticulation(articulation_path, name="ice_cream_arm"))
    world.reset()

    q0 = arm.get_joint_positions()
    if q0 is None:
        print("错误：无法从 articulation 读取关节角。")
        simulation_app.close()
        return

    q0 = np.array(q0, dtype=float).ravel()
    n_dof = q0.size
    arm_dof = min(5, n_dof)

    try:
        names = list(getattr(arm, "dof_names", None) or getattr(arm, "joint_names", []))
    except Exception:
        names = [f"joint{i+1}" for i in range(n_dof)]

    print(f"[info] USD: {usd_path}")
    print(f"[info] Articulation 路径: {articulation_path}")
    print(f"[info] DOF 总数: {n_dof}，臂关节数: {arm_dof}（只控制前 {arm_dof} 个）")

    # 关节限位：简单使用 [-180°, 180°] 作为安全范围，避免 URDF 初始角变化带来的绝对值依赖
    joint_lower = np.deg2rad(np.full(arm_dof, -180.0, dtype=float))
    joint_upper = np.deg2rad(np.full(arm_dof, 180.0, dtype=float))

    # 当前命令角度（弧度，仅前 5 个臂关节）
    q_arm = q0[:arm_dof].copy()

    _print_joint_state(names, np.rad2deg(q_arm))
    print("交互命令：")
    print("  show                  显示当前 5 个关节角（度）")
    print("  set <idx> <deg>       将第 idx 个关节设为绝对角度（度），idx=1..5")
    print("  add <idx> <deg>       在当前基础上增量调整（度），例如 add 2 -5")
    print("  home                  所有关节设为 0 度")
    print("  step                  仅步进一小段仿真（不改角度）")
    print("  q / quit              退出")

    try:
        while True:
            try:
                cmd = input("\n请输入命令 (show/set/add/home/step/q): ").strip()
            except EOFError:
                break
            if not cmd:
                continue

            tokens = cmd.split()
            op = tokens[0].lower()

            if op in ("q", "quit", "exit"):
                print("退出调节脚本。")
                break

            if op == "show":
                _print_joint_state(names, np.rad2deg(q_arm))
                continue

            if op == "home":
                q_arm[:] = 0.0
                _print_joint_state(names, np.rad2deg(q_arm))
            elif op in ("set", "add"):
                if len(tokens) != 3:
                    print("用法: set <idx> <deg> 或 add <idx> <deg>")
                    continue
                try:
                    idx = int(tokens[1]) - 1
                    delta_deg = float(tokens[2])
                except ValueError:
                    print("idx 必须是整数，deg 必须是浮点数。")
                    continue
                if not (0 <= idx < arm_dof):
                    print(f"关节 idx 超出范围，应在 1..{arm_dof}")
                    continue

                if op == "set":
                    q_arm[idx] = np.deg2rad(delta_deg)
                else:  # add
                    q_arm[idx] += np.deg2rad(delta_deg)

                q_arm = np.clip(q_arm, joint_lower, joint_upper)
                _print_joint_state(names, np.rad2deg(q_arm))
            elif op == "step":
                print(f"仅步进仿真 {args.steps_per_command} 步，不修改关节角。")
            else:
                print("未知命令，请使用 show/set/add/home/step/q。")
                continue

            # 将臂关节角扩展为完整 DOF，并在一段时间内下发到控制器
            q_full = _to_full_joint_vector(q_arm, n_dof)
            for _ in range(max(1, args.steps_per_command)):
                ctrl = arm.get_articulation_controller()
                ctrl.apply_action(ArticulationAction(joint_positions=q_full.tolist()))
                world.step(render=not args.headless)
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()

