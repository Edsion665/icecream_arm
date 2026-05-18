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

from .exceptions import UDPTransportError
from .runtime import ReachTracker, log_udp_frame_preview, start_reply_worker, stop_reply_worker

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (_ROOT,):
    if _p not in sys.path:
        sys.path.insert(0, _p)

_ICECREAM_ROOT = _ROOT
# URDF 默认：见 ``SimulationConfig.arm_urdf_relpath``；SINGLE 仍在 configuration/ 根目录作备选。
_CONFIG_DIR = os.path.join(_PKG_DIR, "configuration")

_DEFAULT_SINGLE_URDF_CANDIDATES = [
    os.path.join(_CONFIG_DIR, "ice_cream_SINGLE.SLDASM.urdf"),
]


def _default_urdf_path() -> Optional[str]:
    from .config import SIM_CONFIG

    p = os.path.abspath(os.path.join(_CONFIG_DIR, SIM_CONFIG.arm_urdf_relpath))
    if os.path.isfile(p):
        return p
    for cand in _DEFAULT_SINGLE_URDF_CANDIDATES:
        if os.path.isfile(cand):
            return cand
    return None


DEFAULT_URDF = _default_urdf_path()


def run_loop(
    *,
    listen_host: str,
    listen_port: int,
    rpi_ip: Optional[str],
    rpi_port: int,
    urdf_path: Optional[str],
    log_print: bool = True,
    udp_strict: Optional[bool] = None,
) -> None:
    import numpy as np
    from arm_control_bridge.calculator import CalculatorEngine, CalculatorState, URDFKinematics

    from .config import CONFIG, RUNTIME, load_calibration_deg

    cc = CONFIG
    strict = RUNTIME.udp_strict if udp_strict is None else udp_strict
    calib_file = os.path.join(_ICECREAM_ROOT, cc.calibration_md_relpath)
    q5_deg = float(cc.q5_fixed_deg)
    web_port = int(cc.web_test_port)
    web_host = cc.web_test_host
    from .io import PiFeedbackClient, RPiUDPStreamer, RpiProtocolAdapter, motor
    from .io.listener import ClawCommand, MotionCommand4Axis, claw_listener, network_listener, start_http_server

    def log(msg: str) -> None:
        if log_print:
            print(msg, flush=True)

    tracker = ReachTracker()

    def _on_pending(*, tcp_conn=None, http_slot=None) -> None:
        tracker.register_pending(tcp_conn=tcp_conn, http_slot=http_slot)

    def _on_tcp_reply_error(_conn: object, exc: BaseException) -> None:
        log(f"[runner][reply] TCP sendall 失败: {type(exc).__name__}: {exc}")

    start_reply_worker(tracker, on_tcp_send_error=_on_tcp_reply_error)

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
    log(
        "[runner] NOSIM 启动：无关节反馈回读，内部初始目标仅由 reset_command() 给定。"
    )
    log(
        "[runner] 初始目标关节 (deg): "
        + f"{np.array2string(np.rad2deg(state.q_cmd[:4]), precision=2)}"
    )

    cmd_q: "queue.Queue[Optional[MotionCommand4Axis | ClawCommand]]" = queue.Queue()
    claw_q: "queue.Queue[Optional[ClawCommand]]" = queue.Queue()
    server = network_listener(listen_host, listen_port, cmd_q, on_log=log, on_pending=_on_pending)
    claw_rx = claw_listener(claw_q, on_log=log)
    server.start_background()

    if web_port > 0:
        _web_dir = os.path.join(os.path.dirname(__file__), "web")
        start_http_server(cmd_q, host=web_host, port=web_port, web_dir=_web_dir, on_log=log, on_pending=_on_pending)

    arm_motor: Optional[motor] = None
    adapter: Optional[RpiProtocolAdapter] = None
    pi_fb: Optional[PiFeedbackClient] = None
    if rpi_ip:
        streamer = RPiUDPStreamer(rpi_ip, rpi_port, strict_udp=strict)
        adapter = RpiProtocolAdapter(streamer)
        arm_motor = motor(adapter)
        pi_fb = PiFeedbackClient(rpi_ip)

    log(
        f"[runner] 控制频率 {CONFIG.control_hz} Hz | URDF: {getattr(kin, '_source', '?')} | "
        f"标定(度): {np.array2string(q_calib_deg, precision=2)}"
    )

    try:
        dump_next_udp_frame = False
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
                    tracker.accept("claw")
                    claw_rx.emit(c)
                    continue
                if c.kind == "ping":
                    log("[runner] pong")
                    continue
                if c.kind == "stop":
                    log("[runner] estop（当前仅记录，可扩展为保持当前角）")
                    continue
                if c.kind == "stepper":
                    state.stepper_deg_cmd = float(
                        max(-180.0, min(180.0, float(c.payload["stepper_deg"])))
                    )
                    log(f"[runner] recv stepper: stepper_deg={state.stepper_deg_cmd:.3f}")
                    dump_next_udp_frame = True
                    continue
                if c.kind == "conveyor":
                    state.conveyor_run_cmd = float(c.payload["conveyor_run"])
                    log(f"[runner] recv conveyor: run={state.conveyor_run_cmd:.0f}")
                    dump_next_udp_frame = True
                    continue
                log(f"[runner] recv {c.kind}: {c.payload}")
                tracker.accept(c.kind)
                engine.apply_command(c, state)
                dump_next_udp_frame = True

            while True:
                try:
                    cc = claw_q.get_nowait()
                except queue.Empty:
                    break
                if cc is None:
                    continue
                payload = cc.payload
                state.set_wrist_rel_deg(float(payload.get("wrist_deg", state.wrist_rel_deg)))
                state.grip_state = 1.0 if float(payload.get("grip_state", state.grip_state)) >= 0.5 else 0.0
                state.servo_deg = np.array([state.wrist_rel_deg, state.grip_state], dtype=float)
                log(f"[runner] recv claw: wrist={state.wrist_rel_deg:.3f}, grip_state={state.grip_state:.0f}")
                dump_next_udp_frame = True

            frame = engine.step(None, state, dt=CONFIG.control_dt)
            if dump_next_udp_frame:
                log_udp_frame_preview(frame, log, tag="[runner][UDP]")
                dump_next_udp_frame = False
            if arm_motor is not None:
                arm_motor.send(frame)
                if state.stepper_deg_cmd != 0.0:
                    state.stepper_deg_cmd = 0.0

            if tracker.waiting:
                fb = pi_fb.get_fb_arm_rad() if pi_fb is not None else None
                reached, err = engine.is_reached(state, fb_arm_rad=fb)
                if tracker.cmd_kind in ("pose", "pose_delta"):
                    q_for_fk = fb if fb is not None else state.q_cmd
                    p = kin.forward_kinematics_position_link4(q_for_fk).tolist()
                    result = {"ok": True, "reached": True,
                              "actual_pose": {"x": round(p[0],4), "y": round(p[1],4), "z": round(p[2],4)},
                              "error_pose_m": round(err, 4)}
                else:
                    result = {"ok": True, "reached": True, "error_joints_deg": round(err, 3)}
                tracker.feed(reached, result)

            next_t += CONFIG.control_dt
            sleep_t = next_t - time.monotonic()
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                next_t = time.monotonic()
    except UDPTransportError as exc:
        log(f"[runner] UDP 发送失败，退出: {exc}")
        raise SystemExit(1) from exc
    except KeyboardInterrupt:
        log("[runner] 退出")
    finally:
        stop_reply_worker(tracker)
        server.close()
        if adapter is not None:
            adapter.close()


def run_sim_loop(
    *,
    listen_host: str,
    listen_port: int,
    rpi_ip: Optional[str],
    rpi_port: int,
    urdf_path: Optional[str],
    log_print: bool = True,
    udp_strict: Optional[bool] = None,
) -> None:
    import numpy as np
    from isaacsim import SimulationApp

    from .calculator import CalculatorEngine, CalculatorState, NUM_JOINTS, URDFKinematics
    from .config import CONFIG, RUNTIME, SIM_CONFIG, load_calibration_deg

    cc = CONFIG
    sim_cfg = SIM_CONFIG
    strict = RUNTIME.udp_strict if udp_strict is None else udp_strict
    calib_file = os.path.join(_ICECREAM_ROOT, cc.calibration_md_relpath)
    q5_deg = float(cc.q5_fixed_deg)
    headless = bool(sim_cfg.sim_headless)
    web_port = int(sim_cfg.sim_web_port)
    web_host = sim_cfg.sim_web_host
    sim_gain_scale = float(sim_cfg.sim_gain_scale)
    sim_resync = bool(sim_cfg.sim_resync)
    from .io import RPiUDPStreamer, RpiProtocolAdapter, motor
    from .io.listener import ClawCommand, MotionCommand4Axis, network_listener, start_http_server
    from .sim import (
        add_grid_ground,
        apply_arm_prim_world_z_offset,
        find_articulation_root,
        log_prim_world_pose,
        receiver,
        set_marker_xyz,
        show,
    )

    simulation_app = SimulationApp({"headless": headless})

    def log(msg: str) -> None:
        if log_print:
            print(msg, flush=True)

    tracker = ReachTracker()

    def _on_pending(*, tcp_conn=None, http_slot=None) -> None:
        tracker.register_pending(tcp_conn=tcp_conn, http_slot=http_slot)

    def _on_tcp_reply_error(_conn: object, exc: BaseException) -> None:
        log(f"[sim][reply] TCP sendall 失败: {type(exc).__name__}: {exc}")

    start_reply_worker(tracker, on_tcp_send_error=_on_tcp_reply_error)

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

    ik_follow_hz = float(CONFIG.control_hz)
    n_physics_substeps = 1
    physics_dt = 1.0 / ik_follow_hz

    try:
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
    except ModuleNotFoundError:
        from omni.isaac.core import World
        from omni.isaac.core.prims import SingleArticulation
        from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage

    cmd_q: "queue.Queue[Optional[MotionCommand4Axis | ClawCommand]]" = queue.Queue()
    tcp = network_listener(listen_host, listen_port, cmd_q, on_log=log, on_pending=_on_pending)
    tcp.start_background()

    arm_motor: Optional[motor] = None
    adapter: Optional[RpiProtocolAdapter] = None
    if rpi_ip:
        streamer = RPiUDPStreamer(rpi_ip, rpi_port, strict_udp=strict)
        adapter = RpiProtocolAdapter(streamer)
        arm_motor = motor(adapter)

    if web_port > 0:
        start_http_server(
            cmd_q,
            host=web_host,
            port=web_port,
            web_dir=os.path.join(os.path.dirname(__file__), "web"),
            on_log=log,
            on_pending=_on_pending,
        )

    usd_path = os.path.abspath(os.path.join(_PKG_DIR, "configuration", sim_cfg.arm_usd_relpath))
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=physics_dt)
    add_grid_ground(world, _PKG_DIR, on_log=log)
    stage = get_current_stage()
    arm_prim = add_reference_to_stage(usd_path=usd_path, prim_path=sim_cfg.arm_prim_path)
    try:
        arm_prim.Load()
    except Exception as ex:
        log(f"[sim] arm_prim.Load() 跳过: {type(ex).__name__}: {ex}")
    apply_arm_prim_world_z_offset(
        stage, sim_cfg.arm_prim_path, sim_cfg.arm_world_z_offset_m, log=log
    )
    if sim_cfg.articulation_prim_path:
        articulation_path = sim_cfg.articulation_prim_path
    else:
        found_root = find_articulation_root(stage, sim_cfg.arm_prim_path)
        articulation_path = found_root if found_root else sim_cfg.arm_prim_path
    arm = world.scene.add(SingleArticulation(articulation_path, name="ice_cream_arm"))
    world.reset()
    log_prim_world_pose(stage, sim_cfg.arm_prim_path, "arm_prim", log)
    if articulation_path != sim_cfg.arm_prim_path:
        log_prim_world_pose(stage, articulation_path, "articulation_root", log)

    q0 = arm.get_joint_positions()
    n_dof = len(q0) if q0 is not None else 0
    if n_dof == 0:
        stop_reply_worker(tracker)
        tcp.close()
        simulation_app.close()
        return

    # 与 calculator.reset_command() 一致（DEFAULT_INITIAL_JOINT_REL_DEG_4 + 标定 + q5_fixed），不用 USD 默认角
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
            kps=np.full(n_dof, sim_cfg.pd_kp_base * _g, dtype=float),
            kds=np.full(n_dof, sim_cfg.pd_kd_base * _g, dtype=float),
        )
        if log_print:
            log(
                f"[sim] Articulation PD 已设 kps≈{sim_cfg.pd_kp_base * _g:g} "
                f"kds≈{sim_cfg.pd_kd_base * _g:g}（cartesian_ik_verify 同量级）"
            )
    except Exception as ex:
        if log_print:
            log(f"[sim] set_gains 跳过: {ex}")

    frame_rx = receiver()
    dump_next_udp_frame = False
    _init_frame_sent = False

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
                tracker.accept("claw")
                payload = c.payload
                state.set_wrist_rel_deg(float(payload.get("wrist_deg", state.wrist_rel_deg)))
                state.grip_state = 1.0 if float(payload.get("grip_state", state.grip_state)) >= 0.5 else 0.0
                state.servo_deg = np.array([state.wrist_rel_deg, state.grip_state], dtype=float)
                dump_next_udp_frame = True
                continue
            if c.kind == "stepper":
                state.stepper_deg_cmd = float(
                    max(-180.0, min(180.0, float(c.payload["stepper_deg"])))
                )
                log(f"[sim] recv stepper: stepper_deg={state.stepper_deg_cmd:.3f}")
                dump_next_udp_frame = True
                continue
            if c.kind == "conveyor":
                state.conveyor_run_cmd = float(c.payload["conveyor_run"])
                log(f"[sim] recv conveyor: run={state.conveyor_run_cmd:.0f}")
                dump_next_udp_frame = True
                continue
            log(f"[sim] recv {c.kind}: {c.payload}")
            tracker.accept(c.kind)
            engine.apply_command(c, state)
            dump_next_udp_frame = True
            if c.kind in ("pose", "pose_delta"):
                log(
                    "[sim][frame] pose target interpreted as link0/local = "
                    + f"({float(state.pose_xyz[0]):+.4f}, {float(state.pose_xyz[1]):+.4f}, {float(state.pose_xyz[2]):+.4f})"
                )
                set_marker_xyz(stage, state.pose_xyz, log=log)

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
                if diff_b > SIM_CONFIG.sim_track_snap_threshold_rad:
                    q_snap = q_a.copy()
                    for i in range(min(controlled_dof, n_dof, NUM_JOINTS)):
                        q_snap[i] = float(state.q_cmd[i])
                    if n_dof >= 5 and controlled_dof >= 5:
                        q_snap[4] = state.wrist_joint_rad()
                    try:
                        arm.set_joint_positions(q_snap)
                    except Exception as ex:
                        log(f"[sim][SYNC] set_joint_positions 失败: {type(ex).__name__}: {ex}")

        frame = engine.step(None, state, dt=ik_dt)
        if not _init_frame_sent:
            frame.arm_omega_rad_s = np.full(4, 0.05, dtype=float)
            _init_frame_sent = True
        if dump_next_udp_frame:
            log_udp_frame_preview(frame, log, tag="[sim][UDP]")
            dump_next_udp_frame = False
        frame_rx.accept(frame)
        viewer.apply(frame_rx.latest())
        if arm_motor is not None:
            try:
                arm_motor.send(frame)
                if state.stepper_deg_cmd != 0.0:
                    state.stepper_deg_cmd = 0.0
            except UDPTransportError as exc:
                log(f"[sim] UDP 发送失败，退出: {exc}")
                raise SystemExit(1) from exc

        if tracker.waiting:
            q_sim_raw = arm.get_joint_positions()
            fb_sim = np.asarray(q_sim_raw, dtype=float).ravel()[:4] if q_sim_raw is not None else None
            reached, err = engine.is_reached(state, fb_arm_rad=fb_sim)
            if tracker.cmd_kind in ("pose", "pose_delta"):
                q_for_fk = fb_sim if fb_sim is not None else state.q_cmd
                p = kin.forward_kinematics_position_link4(q_for_fk).tolist()
                result = {"ok": True, "reached": True,
                          "actual_pose": {"x": round(p[0],4), "y": round(p[1],4), "z": round(p[2],4)},
                          "error_pose_m": round(err, 4)}
            else:
                result = {"ok": True, "reached": True, "error_joints_deg": round(err, 3)}
            tracker.feed(reached, result)

    stop_reply_worker(tracker)
    if adapter is not None:
        adapter.close()
    tcp.close()
    simulation_app.close()


def main() -> None:
    from arm_control_bridge.config import CONFIG

    cc = CONFIG
    p = argparse.ArgumentParser(description="arm_control_bridge：网络指令 + 可选 Isaac Sim（其余见 config）")
    p.add_argument("--sim", action="store_true", help="启动 Isaac Sim")
    p.add_argument("--listen", default=cc.listen_host, help="TCP 监听（默认 ControlConfig.listen_host）")
    p.add_argument("--port", type=int, default=cc.default_tcp_port, help="TCP 端口")
    p.add_argument("--rpi-ip", default=None, help="树莓派 IP（未传则用 ControlConfig.rpi_ip）")
    p.add_argument("--rpi-port", type=int, default=cc.default_udp_port, help="树莓派 UDP 端口")
    args = p.parse_args()

    rpi_ip: Optional[str] = args.rpi_ip if args.rpi_ip is not None else cc.rpi_ip
    if rpi_ip is not None and not str(rpi_ip).strip():
        rpi_ip = None

    urdf = _default_urdf_path()
    if urdf is None or not os.path.isfile(urdf):
        print(
            "错误：未找到 URDF（检查 SimulationConfig.arm_urdf_relpath 或 configuration/ice_cream_SINGLE.SLDASM.urdf）。"
        )
        sys.exit(1)

    if args.sim:
        run_sim_loop(
            listen_host=args.listen,
            listen_port=args.port,
            rpi_ip=rpi_ip,
            rpi_port=args.rpi_port,
            urdf_path=urdf,
        )
    else:
        run_loop(
            listen_host=args.listen,
            listen_port=args.port,
            rpi_ip=rpi_ip,
            rpi_port=args.rpi_port,
            urdf_path=urdf,
        )


if __name__ == "__main__":
    main()

