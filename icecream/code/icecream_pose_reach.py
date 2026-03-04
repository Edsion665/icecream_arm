#!/usr/bin/env python3
"""
位姿到达验证脚本（Isaac Sim）：
给定目标空间位姿后，按指定频率运行位置控制或速度控制，使末端到达并保持目标位姿。
调试接口：命令行参数设置目标、控制模式、频率；运行时定期打印位姿误差。
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import os
import sys

import numpy as np

_CODE_DIR = os.path.dirname(os.path.abspath(__file__))
if _CODE_DIR not in sys.path:
    sys.path.insert(0, _CODE_DIR)

from pxr import Gf, UsdGeom

from isaacsim.core.api import World
from icecream_driver import IceCreamArmDriver, NUM_JOINTS
from icecream_pose_control import PoseController, pose_from_position_rpy


def main():
    parser = argparse.ArgumentParser(
        description="位姿控制验证：末端到达并保持目标位姿，支持位置/速度控制与调试输出"
    )
    parser.add_argument("--usd", type=str, default=None, help="机械臂 USD 路径")
    parser.add_argument(
        "--mode",
        type=str,
        choices=["position", "velocity"],
        default="position",
        help="控制模式：position=IK 角度控制，velocity=雅可比速度控制",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=60.0,
        help="控制律更新频率 Hz（默认 60）",
    )
    parser.add_argument(
        "--target",
        type=float,
        nargs="+",
        default=[0.35, 0.2, 0.15, 0, 0, 0],
        help="目标位姿（基座系）：x y z 米，可选 roll pitch yaw 度；共 3 或 6 个数",
    )
    parser.add_argument(
        "--position-only",
        action="store_true",
        help="仅位置控制，不约束末端姿态（仅 position 模式有效）",
    )
    parser.add_argument(
        "--debug-interval",
        type=int,
        default=60,
        help="每 N 个仿真步打印一次位姿误差（0=不打印）",
    )
    parser.add_argument(
        "--pos-gain",
        type=float,
        default=2.0,
        help="velocity 模式位置增益",
    )
    parser.add_argument(
        "--ori-gain",
        type=float,
        default=1.0,
        help="velocity 模式姿态增益",
    )
    args, _ = parser.parse_known_args()

    physics_dt = 1.0 / 60.0
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=physics_dt)
    world.scene.add_default_ground_plane()

    driver = IceCreamArmDriver(world, usd_path=args.usd)
    world.reset()

    # 目标位姿（基座系）
    t = args.target
    position_only = args.position_only or len(t) == 3
    if len(t) >= 6:
        target_pos, target_rot = pose_from_position_rpy([t[0], t[1], t[2]], t[3], t[4], t[5])
    else:
        target_pos = np.array([t[0], t[1], t[2]], dtype=float)
        target_rot = None

    pose_controller = PoseController(
        mode=args.mode,
        control_frequency_hz=args.frequency,
        position_gain=args.pos_gain,
        orientation_gain=args.ori_gain,
        ik_position_only=position_only,
    )
    if target_rot is not None:
        pose_controller.set_target_pose(target_pos, target_rot)
    else:
        pose_controller.set_target_position(target_pos)

    # 在目标位姿处放置一个无物理的消防块（红色方块）作为视觉标记
    # 目标在基座系下，机械臂基座在 /World/IceCreamArm，默认与世界系对齐，故直接用目标位姿
    stage = None
    try:
        from omni.usd import get_context
        stage = get_context().get_stage()
    except Exception:
        pass
    if stage is not None:
        marker_path = "/World/TargetPoseMarker"
        cube_prim = UsdGeom.Cube.Define(stage, marker_path)
        cube_prim.CreateSizeAttr(0.04)  # 4cm 边长
        xform = UsdGeom.Xformable(cube_prim)
        xform.ClearXformOpOrder()
        # 位姿 = 先旋转再平移（USD 中 xformOp 按 order 从左到右施加）
        if target_rot is not None:
            from icecream_kinematics import _matrix_to_quat
            q = _matrix_to_quat(target_rot).astype(float)  # [x,y,z,w]
            xform.AddOrientOp().Set(Gf.Quatf(q[3], Gf.Vec3f(q[0], q[1], q[2])))
        xform.AddTranslateOp().Set(Gf.Vec3d(float(target_pos[0]), float(target_pos[1]), float(target_pos[2])))
        color_attr = cube_prim.CreateDisplayColorAttr()
        color_attr.Set([Gf.Vec3f(0.9, 0.2, 0.1)])  # 消防红
        # 不添加任何物理 schema，保持纯视觉

    step_count = 0
    reset_needed = False

    while simulation_app.is_running():
        world.step(render=True)
        if world.is_stopped():
            reset_needed = True
        if not world.is_playing():
            continue
        if reset_needed:
            world.reset()
            reset_needed = False

        q = driver.get_joint_positions()
        q_target = pose_controller.update(physics_dt, q)
        driver.set_joint_positions(q_target)
        driver.apply()

        if args.debug_interval > 0:
            step_count += 1
            if step_count >= args.debug_interval:
                step_count = 0
                err_pos, err_ori, pos_norm, ori_rad = pose_controller.get_pose_error(q)
                cur_pos, _ = pose_controller.get_current_pose(q)
                print(
                    "[pose_reach] pos_err(m)=%.4f ori_err(rad)=%.4f | current_ee=[%.3f,%.3f,%.3f] target=[%.3f,%.3f,%.3f]"
                    % (
                        pos_norm,
                        ori_rad,
                        cur_pos[0],
                        cur_pos[1],
                        cur_pos[2],
                        pose_controller._target_position[0],
                        pose_controller._target_position[1],
                        pose_controller._target_position[2],
                    )
                )

    simulation_app.close()


if __name__ == "__main__":
    main()
