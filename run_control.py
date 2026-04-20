#!/usr/bin/env python3
"""
arm_control_bridge 入口：新架构启动入口（4轴主臂 + claw独立通道）。
"""

from __future__ import annotations

import argparse
import os
import queue
import sys
import time
from typing import Optional

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SIM_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
_SIM_CODE = os.path.join(_ROOT, "sim_code")
for _p in (_SIM_CODE, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

_ICECREAM_ROOT = _ROOT
# 优先 v8 URDF，否则 SINGLE；均不存在则为 None（IK 用 calculator 内硬编码关节参数）
_DEFAULT_V8_URDF = os.path.join(
    _ICECREAM_ROOT,
    "icecream_model_v8",
    "ice_cream_v8.SLDASM",
    "urdf",
    "ice_cream_v8.SLDASM.urdf",
)
_DEFAULT_SINGLE_URDF = os.path.join(
    _ICECREAM_ROOT,
    "ice_cream_SINGLE.SLDASM",
    "urdf",
    "ice_cream_SINGLE.SLDASM.urdf",
)


def _default_urdf_path() -> Optional[str]:
    if os.path.isfile(_DEFAULT_V8_URDF):
        return _DEFAULT_V8_URDF
    if os.path.isfile(_DEFAULT_SINGLE_URDF):
        return _DEFAULT_SINGLE_URDF
    return None


DEFAULT_URDF = _default_urdf_path()
_INITIAL_POSITION_MD = os.path.join(_ICECREAM_ROOT, "initial_position.md")


def run_loop(
    *,
    listen_host: str,
    listen_port: int,
    rpi_ip: Optional[str],
    rpi_port: int,
    udp_format: str,
    urdf_path: Optional[str],
    calib_file: Optional[str],
    q5_deg: float,
    web_port: int = 0,
    web_host: str = "127.0.0.1",
    log_print: bool = True,
) -> None:
    import numpy as np
    from .calculator import CalculatorEngine, CalculatorState, URDFKinematics

    from .PiController import RPiUDPStreamer, RpiProtocolAdapter, motor, servoMotor
    from .config import CONTROL_DT, CONTROL_HZ, load_calibration_deg
    from .listener import ClawCommand, MotionCommand4Axis, claw_listener, network_listener, start_http_server

    def log(msg: str) -> None:
        if log_print:
            print(msg, flush=True)

    q_calib_deg = np.array(load_calibration_deg(calib_file or ""), dtype=float)
    q_calib_rad = np.deg2rad(q_calib_deg)
    q5_fixed_rad = np.deg2rad(q5_deg)
    kin = URDFKinematics(urdf_path=urdf_path)

    state = CalculatorState(
        q_calib_deg=q_calib_deg,
        q_calib_rad=q_calib_rad,
        q5_fixed_rad=q5_fixed_rad,
        pose_xyz=np.zeros(3, dtype=float),
    )
    state.reset_command()
    state.pose_xyz = kin.forward_kinematics_position_link4(state.q_full).copy()
    engine = CalculatorEngine(kin)

    cmd_q: "queue.Queue[Optional[MotionCommand4Axis | ClawCommand]]" = queue.Queue()
    claw_q: "queue.Queue[Optional[ClawCommand]]" = queue.Queue()
    server = network_listener(listen_host, listen_port, cmd_q, on_log=log)
    claw_rx = claw_listener(claw_q, on_log=log)
    server.start_background()

    if web_port > 0:
        _web_dir = os.path.join(os.path.dirname(__file__), "web")
        start_http_server(cmd_q, host=web_host, port=web_port, web_dir=_web_dir, on_log=log)

    arm_motor: Optional[motor] = None
    claw_motor: Optional[servoMotor] = None
    adapter: Optional[RpiProtocolAdapter] = None
    if rpi_ip:
        streamer = RPiUDPStreamer(rpi_ip, rpi_port, fmt="v2" if udp_format == "v2" else "v1")
        adapter = RpiProtocolAdapter(streamer)
        arm_motor = motor(adapter)
        claw_motor = servoMotor(adapter)

    log(
        f"[runner] 控制频率 {CONTROL_HZ} Hz | URDF: {getattr(kin, '_source', '?')} | "
        f"标定(度): {np.array2string(q_calib_deg, precision=2)}"
    )

    try:
        next_t = time.monotonic()
        while True:
            while True:
                try:
                    c = cmd_q.get_nowait()
                except queue.Empty:
                    break
                if c is None:
                    continue
                if isinstance(c, ClawCommand):
                    claw_rx.emit(c)
                    continue
                if c.kind == "ping":
                    log("[runner] pong")
                    continue
                if c.kind == "stop":
                    log("[runner] estop（当前仅记录，可扩展为保持当前角）")
                    continue
                engine.apply_command(c, state)

            while True:
                try:
                    cc = claw_q.get_nowait()
                except queue.Empty:
                    break
                if cc is None:
                    continue
                payload = cc.payload
                if "servo_deg" in payload and isinstance(payload["servo_deg"], (list, tuple)):
                    sd = payload["servo_deg"]
                    if len(sd) >= 2:
                        state.servo_deg = np.array([float(sd[0]), float(sd[1])], dtype=float)
                else:
                    wrist = float(payload.get("wrist_deg", state.servo_deg[0]))
                    grip = float(payload.get("grip", state.servo_deg[1]))
                    state.servo_deg = np.array([wrist, grip], dtype=float)

            frame = engine.step(None, state, dt=CONTROL_DT)
            if arm_motor is not None:
                arm_motor.send(frame)
            if claw_motor is not None:
                claw_motor.send(state.servo_deg)

            next_t += CONTROL_DT
            sleep_t = next_t - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        log("[runner] 退出")
    finally:
        server.close()
        if adapter is not None:
            adapter.close()


def run_sim_loop(
    *,
    listen_host: str,
    listen_port: int,
    rpi_ip: Optional[str],
    rpi_port: int,
    udp_format: str,
    urdf_path: Optional[str],
    calib_file: Optional[str],
    q5_deg: float,
    sim_usd: Optional[str],
    headless: bool,
    ik_rate: float,
    web_port: int,
    web_host: str,
    damping: float,
    max_iter: int,
    tol: float,
    physics_hz: Optional[float] = None,
    no_rate_limits: bool = False,
    log_print: bool = True,
    sim_gain_scale: float = 1.0,
    sim_resync: bool = True,
) -> None:
    import numpy as np
    from isaacsim import SimulationApp

    from .PiController import RPiUDPStreamer, RpiProtocolAdapter, motor
    from .calculator import CalculatorEngine, CalculatorState, NUM_JOINTS, URDFKinematics
    from .config import (
        SIM_TRACK_SNAP_THRESHOLD_RAD,
        load_calibration_deg,
    )
    from .listener import ClawCommand, MotionCommand4Axis, network_listener, start_http_server
    from .shower import receiver, show

    simulation_app = SimulationApp({"headless": headless})

    def log(msg: str) -> None:
        if log_print:
            print(msg, flush=True)

    q_calib_deg = np.array(load_calibration_deg(calib_file or ""), dtype=float)
    q_calib_rad = np.deg2rad(q_calib_deg)
    q5_fixed_rad = np.deg2rad(q5_deg)
    kin = URDFKinematics(urdf_path=urdf_path)
    state = CalculatorState(
        q_calib_deg=q_calib_deg,
        q_calib_rad=q_calib_rad,
        q5_fixed_rad=q5_fixed_rad,
        pose_xyz=np.zeros(3, dtype=float),
    )
    state.reset_command()
    state.pose_xyz = kin.forward_kinematics_position_link4(state.q_full).copy()
    if log_print:
        log(f"[sim] 初始 pose_xyz = link4 FK @ 标定零位 (m): {state.pose_xyz}")
    engine = CalculatorEngine(kin)

    ik_follow_hz = float(ik_rate)
    # 与 sim_code/cartesian_ik_verify 一致：每控制周期一次 world.step，physics_dt = 1/ik_follow_hz
    physics_hz_eff = ik_follow_hz
    n_physics_substeps = 1
    physics_dt = 1.0 / ik_follow_hz
    if physics_hz is not None and log_print:
        log(
            f"[sim] 忽略 --physics-hz={physics_hz}，物理与 IK 锁定同频 {ik_follow_hz:.1f} Hz（单步/周期）"
        )

    try:
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
    except ModuleNotFoundError:
        from omni.isaac.core import World
        from omni.isaac.core.prims import SingleArticulation
        from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage

    cmd_q: "queue.Queue[Optional[MotionCommand4Axis | ClawCommand]]" = queue.Queue()
    tcp = network_listener(listen_host, listen_port, cmd_q, on_log=log)
    tcp.start_background()

    arm_motor: Optional[motor] = None
    adapter: Optional[RpiProtocolAdapter] = None
    if rpi_ip:
        streamer = RPiUDPStreamer(rpi_ip, rpi_port, fmt="v2" if udp_format == "v2" else "v1")
        adapter = RpiProtocolAdapter(streamer)
        arm_motor = motor(adapter)

    if web_port > 0:
        start_http_server(
            cmd_q,
            host=web_host,
            port=web_port,
            web_dir=os.path.join(os.path.dirname(__file__), "web"),
            on_log=log,
        )

    ARM_PRIM_PATH = "/World/IceCreamArm"
    TARGET_MARKER_PATH = "/World/TargetJoint4Marker"
    MARKER_Z_LIFT = 0.04

    def _find_articulation_root(stage, under_path: str):
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
            found = _find_articulation_root(stage, path)
            if found:
                return found
        return None

    def _resolve_sim_usd(sim_usd_arg):
        if sim_usd_arg is not None:
            path = os.path.abspath(sim_usd_arg)
            if not os.path.isfile(path):
                raise FileNotFoundError(f"仿真 USD 不存在: {path}")
            return path
        for name in (
            "ice_cream_v8_arm.usd",
            "ice_cream_single_arm.usd",
            "ice_cream_arm.usd",
        ):
            p = os.path.join(_SIM_TEST_DIR, name)
            if os.path.isfile(p):
                return p
            p = os.path.join(_SIM_CODE, name)
            if os.path.isfile(p):
                return p
            p = os.path.join(_ROOT, "sim_code", name)
            if os.path.isfile(p):
                return p
        raise FileNotFoundError(
            "未找到 ice_cream_v8_arm.usd / ice_cream_single_arm.usd / ice_cream_arm.usd，请用 --sim-usd 指定。"
        )

    def _set_marker_xyz(stage, xyz: np.ndarray) -> None:
        try:
            from pxr import Gf, UsdGeom
            prim = stage.GetPrimAtPath(TARGET_MARKER_PATH)
            if not prim.IsValid():
                return
            xf = UsdGeom.Xformable(prim)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(float(xyz[0]), float(xyz[1]), float(xyz[2]) + MARKER_Z_LIFT))
        except Exception:
            pass

    usd_path = _resolve_sim_usd(sim_usd)
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=physics_dt)
    world.scene.add_default_ground_plane()
    stage = get_current_stage()
    add_reference_to_stage(usd_path=usd_path, prim_path=ARM_PRIM_PATH)
    found_root = _find_articulation_root(stage, ARM_PRIM_PATH)
    articulation_path = found_root if found_root else ARM_PRIM_PATH
    arm = world.scene.add(SingleArticulation(articulation_path, name="ice_cream_arm"))
    world.reset()

    q0 = arm.get_joint_positions()
    n_dof = len(q0) if q0 is not None else 0
    if n_dof == 0:
        tcp.close()
        simulation_app.close()
        return

    # 与 calculator.reset_command() 一致（config.DEFAULT_INITIAL_JOINT_REL_DEG_4 + 标定 + q5_fixed），不用 USD 默认角
    state.reset_command()
    log(
        "[sim] 初始关节 (deg)，与 reset_command 一致: "
        + f"{np.array2string(np.rad2deg(state.q_full[: min(5, n_dof)]), precision=2)}"
    )

    controlled_dof = min(NUM_JOINTS, n_dof)
    q_full = np.zeros(n_dof, dtype=float)
    q_full[:controlled_dof] = state.q_cmd[:controlled_dof]
    viewer = show(arm, n_dof, q_calib_deg=q_calib_deg, controlled_dof=controlled_dof)
    viewer.initialize(q_full)
    try:
        _ctrl = arm.get_articulation_controller()
        _g = float(sim_gain_scale) if sim_gain_scale else 1.0
        _ctrl.set_gains(
            kps=np.full(n_dof, 1e4 * _g, dtype=float),
            kds=np.full(n_dof, 1e3 * _g, dtype=float),
        )
        if log_print:
            log(f"[sim] Articulation PD 已设 kps≈{1e4 * _g:g} kds≈{1e3 * _g:g}（cartesian_ik_verify 同量级）")
    except Exception as ex:
        if log_print:
            log(f"[sim] set_gains 跳过: {ex}")

    frame_rx = receiver()

    ik_dt = 1.0 / ik_follow_hz
    while simulation_app.is_running():
        while True:
            try:
                c = cmd_q.get_nowait()
            except queue.Empty:
                break
            if c is None:
                continue
            if isinstance(c, ClawCommand):
                log(f"[sim] recv claw: {c.payload}")
                payload = c.payload
                if "servo_deg" in payload and isinstance(payload["servo_deg"], (list, tuple)) and len(payload["servo_deg"]) >= 2:
                    state.servo_deg = np.array([float(payload["servo_deg"][0]), float(payload["servo_deg"][1])], dtype=float)
                continue
            log(f"[sim] recv {c.kind}: {c.payload}")
            engine.apply_command(c, state)
            if c.kind in ("pose", "pose_delta"):
                _set_marker_xyz(stage, state.pose_xyz)

        for _ in range(n_physics_substeps):
            world.step(render=True)
        if not world.is_playing():
            continue

        if sim_resync:
            q_actual_raw = arm.get_joint_positions()
            if q_actual_raw is not None:
                q_a = np.asarray(q_actual_raw, dtype=float).ravel()
                arm_cmp = min(4, n_dof, len(q_a))
                diff_b = float(np.linalg.norm(q_a[:arm_cmp] - state.q_cmd[:arm_cmp]))
                if diff_b > SIM_TRACK_SNAP_THRESHOLD_RAD:
                    # 滞后时把仿真对齐到指令，避免旧逻辑「把 q_cmd 拉回 q_actual」导致机械臂/UDP 被往回拽
                    q_snap = q_a.copy()
                    for i in range(min(controlled_dof, n_dof, NUM_JOINTS)):
                        q_snap[i] = float(state.q_cmd[i])
                    if n_dof >= 5 and controlled_dof >= 5:
                        q_snap[4] = float(state.q_cmd[4])
                    try:
                        arm.set_joint_positions(q_snap)
                    except Exception:
                        pass
                    if log_print:
                        log(
                            f"[sim][SYNC] ‖q_actual−q_cmd‖={diff_b:.3f} rad > {SIM_TRACK_SNAP_THRESHOLD_RAD:.2f}，已对齐仿真到指令"
                        )

        frame = engine.step(None, state, dt=ik_dt)
        frame_rx.accept(frame)
        viewer.apply(frame_rx.latest(), q5_fixed_rad=float(state.q_cmd[4]))
        if arm_motor is not None:
            arm_motor.send(frame)

    if adapter is not None:
        adapter.close()
    tcp.close()
    simulation_app.close()


def main() -> None:
    from .config import DEFAULT_TCP_PORT, DEFAULT_UDP_PORT

    p = argparse.ArgumentParser(description="arm_control_bridge 新架构：网络指令 + 可选 Isaac Sim")
    p.add_argument("--listen", default="0.0.0.0", help="TCP 监听地址")
    p.add_argument("--port", type=int, default=DEFAULT_TCP_PORT, help="TCP 端口（JSON 行指令）")
    p.add_argument("--rpi-ip", default=None, help="树莓派 IP；不填则不发送 UDP")
    p.add_argument("--rpi-port", type=int, default=DEFAULT_UDP_PORT, help="树莓派 UDP 端口")
    p.add_argument("--udp-format", choices=("v1", "v2"), default="v2", help="UDP 帧格式")
    p.add_argument(
        "--urdf",
        type=str,
        nargs="?",
        default=DEFAULT_URDF,
        const=DEFAULT_URDF,
        help="运动学 URDF；默认优先 icecream_model_v8/.../ice_cream_v8.SLDASM.urdf，否则 SINGLE；不填且无文件时用内置硬编码",
    )
    p.add_argument("--calib-file", type=str, default=None)
    p.add_argument("--q5-deg", type=float, default=0.0, help="pose IK 中 q5 固定角（度）")
    p.add_argument("--sim", action="store_true", help="启动 Isaac Sim")
    p.add_argument("--headless", action="store_true", help="仿真无窗口（仅 --sim）")
    p.add_argument(
        "--sim-usd",
        type=str,
        default=None,
        help="机械臂 USD（仅 --sim）；未指定时依次尝试本包目录 arm_control_bridge/、再 sim_code/：ice_cream_v8_arm.usd、ice_cream_single_arm.usd、ice_cream_arm.usd",
    )
    p.add_argument("--ik-rate", type=float, default=25.0, help="控制/IK 更新频率 Hz（仅 --sim）")
    p.add_argument(
        "--physics-hz",
        type=float,
        default=None,
        help="已弃用：--sim 下物理步与 --ik-rate 锁定同频（与 cartesian_ik_verify 一致），传入则仅打印忽略提示",
    )
    p.add_argument(
        "--sim-gain-scale",
        type=float,
        default=1.0,
        help="仿真关节 PD 增益缩放（仅 --sim），默认 1.0 对应 kps≈1e4、kds≈1e3",
    )
    p.add_argument(
        "--no-sim-resync",
        action="store_true",
        help="关闭仿真与指令偏差过大时自动对齐（仅 --sim）；默认把仿真关节 snap 到 q_cmd，不拉回内部状态",
    )
    p.add_argument("--no-rate-limits", action="store_true", help="关闭限速（仅 --sim）")
    p.add_argument("--web-port", type=int, default=8765, help="HTTP 测试页端口，0 关闭")
    p.add_argument("--web-host", default="127.0.0.1", help="HTTP 绑定地址")
    p.add_argument("--tol", type=float, default=1e-5, help="IK 位置容差（米）")
    p.add_argument("--max-iter", type=int, default=80, help="IK 最大迭代次数")
    p.add_argument("--damping", type=float, default=1e-2, help="IK 阻尼")
    args = p.parse_args()

    urdf = args.urdf.strip() if args.urdf and str(args.urdf).strip() else None
    if urdf and not os.path.isfile(urdf):
        print(f"错误：URDF 不存在: {urdf}")
        sys.exit(1)
    calib = args.calib_file or _INITIAL_POSITION_MD

    if args.sim:
        run_sim_loop(
            listen_host=args.listen,
            listen_port=args.port,
            rpi_ip=args.rpi_ip,
            rpi_port=args.rpi_port,
            udp_format=args.udp_format,
            urdf_path=urdf,
            calib_file=calib,
            q5_deg=args.q5_deg,
            sim_usd=args.sim_usd,
            headless=args.headless,
            ik_rate=args.ik_rate,
            web_port=args.web_port,
            web_host=args.web_host,
            damping=args.damping,
            max_iter=args.max_iter,
            tol=args.tol,
            physics_hz=args.physics_hz,
            no_rate_limits=args.no_rate_limits,
            sim_gain_scale=args.sim_gain_scale,
            sim_resync=not args.no_sim_resync,
        )
    else:
        run_loop(
            listen_host=args.listen,
            listen_port=args.port,
            rpi_ip=args.rpi_ip,
            rpi_port=args.rpi_port,
            udp_format=args.udp_format,
            urdf_path=urdf,
            calib_file=calib,
            q5_deg=args.q5_deg,
            web_port=args.web_port,
            web_host=args.web_host,
        )


if __name__ == "__main__":
    main()

