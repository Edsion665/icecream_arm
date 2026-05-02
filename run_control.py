#!/usr/bin/env python3
"""
arm_control_bridge 入口：新架构启动入口（4轴主臂 + claw独立通道）。
"""

from __future__ import annotations

import argparse
import collections
import os
import queue
import sys
import threading
import time
from typing import Optional

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
for _p in (_ROOT,):
    if _p not in sys.path:
        sys.path.insert(0, _p)

_ICECREAM_ROOT = _ROOT
# URDF 默认仅从 arm_control_bridge/configuration/ 读取（V8 优先，其次 SINGLE）。
_CONFIG_DIR = os.path.join(_PKG_DIR, "configuration")

_DEFAULT_V8_URDF_CANDIDATES = [
    os.path.join(_CONFIG_DIR, "ice_cream_v8.SLDASM.urdf"),
]

_DEFAULT_SINGLE_URDF_CANDIDATES = [
    os.path.join(_CONFIG_DIR, "ice_cream_SINGLE.SLDASM.urdf"),
]


def _default_urdf_path() -> Optional[str]:
    for p in _DEFAULT_V8_URDF_CANDIDATES:
        if os.path.isfile(p):
            return p
    for p in _DEFAULT_SINGLE_URDF_CANDIDATES:
        if os.path.isfile(p):
            return p
    return None


DEFAULT_URDF = _default_urdf_path()
_INITIAL_POSITION_MD = os.path.join(_ICECREAM_ROOT, "initial_position.md")


def _ensure_v8_arm_payload_symlinks(usd_path: str) -> None:
    """保证 ice_cream_v8_arm.usd 内 @configuration/*.usd@ payload 能解析。

    该资源从 Omniverse 导出时，payload 路径为相对于 composer 目录的
    configuration/ice_cream_v8_arm_{physics,sensor}.usd，即要求存在
    configuration/configuration/*.usd。将三个分块放在同一目录时，需额外建
    configuration/ 子目录链并链回同级文件，否则 PhysX 层永不加载，场景中无关节链。
    """
    if os.path.basename(usd_path) != "ice_cream_v8_arm.usd":
        return
    parent = os.path.dirname(os.path.abspath(usd_path))
    nested = os.path.join(parent, "configuration")
    for name in (
        "ice_cream_v8_arm_physics.usd",
        "ice_cream_v8_arm_sensor.usd",
        "ice_cream_v8_arm_base.usd",
    ):
        src = os.path.join(parent, name)
        dst = os.path.join(nested, name)
        if not os.path.isfile(src) or os.path.lexists(dst):
            continue
        try:
            os.makedirs(nested, exist_ok=True)
            os.symlink(os.path.join("..", name), dst)
        except OSError:
            pass


def run_loop(
    *,
    listen_host: str,
    listen_port: int,
    rpi_ip: Optional[str],
    rpi_port: int,
    urdf_path: Optional[str],
    calib_file: Optional[str],
    q5_deg: float,
    web_port: int = 0,
    web_host: str = "127.0.0.1",
    log_print: bool = True,
) -> None:
    import numpy as np
    from arm_control_bridge.calculator import CalculatorEngine, CalculatorState, URDFKinematics

    from .PiController import RPiUDPStreamer, RpiProtocolAdapter, motor
    from .config import (
        CONTROL_DT, CONTROL_HZ, REACHED_CLAW_DELAY_S, REACHED_STABLE_FRAMES,
        REACHED_TIMEOUT_S, load_calibration_deg,
    )
    from .listener import ClawCommand, MotionCommand4Axis, ReplySlot, claw_listener, network_listener, start_http_server
    from .pi_feedback import PiFeedbackClient

    def log(msg: str) -> None:
        if log_print:
            print(msg, flush=True)

    # ------------------------------------------------------------------
    # 到位追踪器（内联，无锁，仅控制循环访问）
    # ------------------------------------------------------------------
    class _ReachTracker:
        def __init__(self) -> None:
            self.waiting = False
            self.cmd_kind = ""
            self._buf: collections.deque[bool] = collections.deque(maxlen=REACHED_STABLE_FRAMES)
            self._deadline = 0.0
            self._tcp_conn: Optional[object] = None
            self._http_slot: Optional[ReplySlot] = None
            self._claw_timer: Optional[threading.Timer] = None
            # 回传队列：控制循环 put，回传线程 get
            self.reply_q: queue.Queue = queue.Queue()

        def register_pending(self, *, tcp_conn=None, http_slot=None) -> None:
            """listener 线程调用，仅写 pending 字段（原子赋值，无锁）。"""
            self._pending_tcp = tcp_conn
            self._pending_slot = http_slot

        def accept(self, kind: str) -> None:
            """控制循环处理命令时调用。"""
            if self._claw_timer is not None:
                self._claw_timer.cancel()
                self._claw_timer = None
            self.waiting = True
            self.cmd_kind = kind
            self._buf.clear()
            self._deadline = time.monotonic() + REACHED_TIMEOUT_S
            self._tcp_conn = getattr(self, "_pending_tcp", None)
            self._http_slot = getattr(self, "_pending_slot", None)
            self._pending_tcp = None
            self._pending_slot = None
            if kind == "claw":
                t = threading.Timer(REACHED_CLAW_DELAY_S, self._claw_done)
                t.daemon = True
                t.start()
                self._claw_timer = t

        def feed(self, reached: bool, result: dict) -> None:
            """控制循环每帧调用（claw 跳过）。"""
            if not self.waiting or self.cmd_kind == "claw":
                return
            if time.monotonic() > self._deadline:
                self._finish({"ok": False, "reached": False, "error": "timeout"})
                return
            self._buf.append(reached)
            if len(self._buf) == REACHED_STABLE_FRAMES and all(self._buf):
                self._finish(result)

        def _claw_done(self) -> None:
            self._finish({"ok": True, "reached": True})

        def _finish(self, result: dict) -> None:
            self.waiting = False
            self._claw_timer = None
            self.reply_q.put_nowait((result, self._tcp_conn, self._http_slot))
            self._tcp_conn = None
            self._http_slot = None

    tracker = _ReachTracker()

    def _on_pending(*, tcp_conn=None, http_slot=None) -> None:
        tracker.register_pending(tcp_conn=tcp_conn, http_slot=http_slot)

    # 回传线程：异步发送，不阻塞控制循环
    def _reply_worker() -> None:
        import json
        while True:
            item = tracker.reply_q.get()
            if item is None:
                break
            result, tcp_conn, http_slot = item
            if tcp_conn is not None:
                try:
                    tcp_conn.sendall((json.dumps(result) + "\n").encode())
                except OSError:
                    pass
            if http_slot is not None:
                http_slot.result = result
                http_slot.event.set()

    reply_thread = threading.Thread(target=_reply_worker, daemon=True)
    reply_thread.start()

    def _log_udp_frame_preview(frame) -> None:
        p6 = np.zeros(6, dtype=float)
        w6 = np.zeros(6, dtype=float)
        p6[:4] = frame.arm_rel_deg[:4]
        w6[:4] = frame.arm_omega_rad_s[:4]
        p6[4] = float(getattr(frame, "wrist_rel_deg", 0.0))
        w6[4] = float(getattr(frame, "wrist_omega_rad_s", 0.0))
        p6[5] = float(getattr(frame, "grip_state", 0.0))
        w6[5] = 0.0
        log(
            "[runner][UDP] 即将发送一帧: "
            + f"p_rel_deg={np.array2string(p6, precision=3)} "
            + f"omega_rad_s={np.array2string(w6, precision=3)}"
        )

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
        streamer = RPiUDPStreamer(rpi_ip, rpi_port)
        adapter = RpiProtocolAdapter(streamer)
        arm_motor = motor(adapter)
        pi_fb = PiFeedbackClient(rpi_ip)

    log(
        f"[runner] 控制频率 {CONTROL_HZ} Hz | URDF: {getattr(kin, '_source', '?')} | "
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
                state.wrist_rel_deg = float(payload.get("wrist_deg", state.wrist_rel_deg))
                state.grip_state = 1.0 if float(payload.get("grip_state", state.grip_state)) >= 0.5 else 0.0
                state.servo_deg = np.array([state.wrist_rel_deg, state.grip_state], dtype=float)
                log(f"[runner] recv claw: wrist={state.wrist_rel_deg:.3f}, grip_state={state.grip_state:.0f}")
                dump_next_udp_frame = True

            frame = engine.step(None, state, dt=CONTROL_DT)
            if dump_next_udp_frame:
                _log_udp_frame_preview(frame)
                dump_next_udp_frame = False
            if arm_motor is not None:
                arm_motor.send(frame)

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
        REACHED_CLAW_DELAY_S, REACHED_STABLE_FRAMES, REACHED_TIMEOUT_S,
        SIM_TRACK_SNAP_THRESHOLD_RAD, load_calibration_deg,
    )
    from .listener import ClawCommand, MotionCommand4Axis, ReplySlot, network_listener, start_http_server
    from .shower import receiver, show

    simulation_app = SimulationApp({"headless": headless})

    def log(msg: str) -> None:
        if log_print:
            print(msg, flush=True)

    # 到位追踪器（与 run_loop 相同结构，内联）
    class _ReachTracker:
        def __init__(self) -> None:
            self.waiting = False
            self.cmd_kind = ""
            self._buf: collections.deque[bool] = collections.deque(maxlen=REACHED_STABLE_FRAMES)
            self._deadline = 0.0
            self._tcp_conn = None
            self._http_slot: Optional[ReplySlot] = None
            self._claw_timer: Optional[threading.Timer] = None
            self.reply_q: queue.Queue = queue.Queue()

        def register_pending(self, *, tcp_conn=None, http_slot=None) -> None:
            self._pending_tcp = tcp_conn
            self._pending_slot = http_slot

        def accept(self, kind: str) -> None:
            if self._claw_timer is not None:
                self._claw_timer.cancel()
                self._claw_timer = None
            self.waiting = True
            self.cmd_kind = kind
            self._buf.clear()
            self._deadline = time.monotonic() + REACHED_TIMEOUT_S
            self._tcp_conn = getattr(self, "_pending_tcp", None)
            self._http_slot = getattr(self, "_pending_slot", None)
            self._pending_tcp = None
            self._pending_slot = None
            if kind == "claw":
                t = threading.Timer(REACHED_CLAW_DELAY_S, self._claw_done)
                t.daemon = True
                t.start()
                self._claw_timer = t

        def feed(self, reached: bool, result: dict) -> None:
            if not self.waiting or self.cmd_kind == "claw":
                return
            if time.monotonic() > self._deadline:
                self._finish({"ok": False, "reached": False, "error": "timeout"})
                return
            self._buf.append(reached)
            if len(self._buf) == REACHED_STABLE_FRAMES and all(self._buf):
                self._finish(result)

        def _claw_done(self) -> None:
            self._finish({"ok": True, "reached": True})

        def _finish(self, result: dict) -> None:
            self.waiting = False
            self._claw_timer = None
            self.reply_q.put_nowait((result, self._tcp_conn, self._http_slot))
            self._tcp_conn = None
            self._http_slot = None

    tracker = _ReachTracker()

    def _on_pending(*, tcp_conn=None, http_slot=None) -> None:
        tracker.register_pending(tcp_conn=tcp_conn, http_slot=http_slot)

    def _reply_worker() -> None:
        import json
        while True:
            item = tracker.reply_q.get()
            if item is None:
                break
            result, tcp_conn, http_slot = item
            if tcp_conn is not None:
                try:
                    tcp_conn.sendall((json.dumps(result) + "\n").encode())
                except OSError:
                    pass
            if http_slot is not None:
                http_slot.result = result
                http_slot.event.set()

    reply_thread = threading.Thread(target=_reply_worker, daemon=True)
    reply_thread.start()

    def _log_udp_frame_preview(frame) -> None:
        p6 = np.zeros(6, dtype=float)
        w6 = np.zeros(6, dtype=float)
        p6[:4] = frame.arm_rel_deg[:4]
        w6[:4] = frame.arm_omega_rad_s[:4]
        p6[4] = float(getattr(frame, "wrist_rel_deg", 0.0))
        w6[4] = float(getattr(frame, "wrist_omega_rad_s", 0.0))
        p6[5] = float(getattr(frame, "grip_state", 0.0))
        w6[5] = 0.0
        log(
            "[sim][UDP] 即将发送一帧: "
            + f"p_rel_deg={np.array2string(p6, precision=3)} "
            + f"omega_rad_s={np.array2string(w6, precision=3)}"
        )

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
    # 与验证脚本一致：每控制周期一次 world.step，physics_dt = 1/ik_follow_hz
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
    tcp = network_listener(listen_host, listen_port, cmd_q, on_log=log, on_pending=_on_pending)
    tcp.start_background()

    arm_motor: Optional[motor] = None
    adapter: Optional[RpiProtocolAdapter] = None
    if rpi_ip:
        streamer = RPiUDPStreamer(rpi_ip, rpi_port)
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

    ARM_PRIM_PATH = "/World/IceCreamArm"
    TARGET_MARKER_PATH = "/World/TargetJoint4Marker"
    MARKER_Z_LIFT = 0.04

    def _log_prim_world_pose(stage, prim_path: str, tag: str) -> None:
        try:
            from pxr import Usd, UsdGeom
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                log(f"[sim][frame] {tag}: prim not found: {prim_path}")
                return
            xf = UsdGeom.Xformable(prim)
            m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            t = m.ExtractTranslation()
            # 只打印平移；目前问题先聚焦「原点不一致」
            log(
                f"[sim][frame] {tag}: path={prim_path} world_t=({float(t[0]):+.4f}, {float(t[1]):+.4f}, {float(t[2]):+.4f})"
            )
        except Exception as ex:
            log(f"[sim][frame] {tag}: failed to read world pose: {ex}")

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
            p = os.path.join(_PKG_DIR, "configuration", name)
            if os.path.isfile(p):
                return p
        raise FileNotFoundError(
            "未在 arm_control_bridge/configuration/ 找到 ice_cream_v8_arm.usd / ice_cream_single_arm.usd / ice_cream_arm.usd，请用 --sim-usd 指定。"
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
    _ensure_v8_arm_payload_symlinks(usd_path)
    world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=physics_dt)
    world.scene.add_default_ground_plane()
    stage = get_current_stage()
    arm_prim = add_reference_to_stage(usd_path=usd_path, prim_path=ARM_PRIM_PATH)
    try:
        arm_prim.Load()
    except Exception:
        pass
    found_root = _find_articulation_root(stage, ARM_PRIM_PATH)
    articulation_path = found_root if found_root else ARM_PRIM_PATH
    arm = world.scene.add(SingleArticulation(articulation_path, name="ice_cream_arm"))
    world.reset()
    _log_prim_world_pose(stage, ARM_PRIM_PATH, "arm_prim")
    if articulation_path != ARM_PRIM_PATH:
        _log_prim_world_pose(stage, articulation_path, "articulation_root")

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
    dump_next_udp_frame = False

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
                state.wrist_rel_deg = float(payload.get("wrist_deg", state.wrist_rel_deg))
                state.grip_state = 1.0 if float(payload.get("grip_state", state.grip_state)) >= 0.5 else 0.0
                state.servo_deg = np.array([state.wrist_rel_deg, state.grip_state], dtype=float)
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
        if dump_next_udp_frame:
            _log_udp_frame_preview(frame)
            dump_next_udp_frame = False
        frame_rx.accept(frame)
        viewer.apply(frame_rx.latest())
        if arm_motor is not None:
            arm_motor.send(frame)

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

    if adapter is not None:
        adapter.close()
    tcp.close()
    simulation_app.close()


def main() -> None:
    from arm_control_bridge.config import DEFAULT_TCP_PORT, DEFAULT_UDP_PORT

    p = argparse.ArgumentParser(description="arm_control_bridge 新架构：网络指令 + 可选 Isaac Sim")
    p.add_argument("--listen", default="0.0.0.0", help="TCP 监听地址")
    p.add_argument("--port", type=int, default=DEFAULT_TCP_PORT, help="TCP 端口（JSON 行指令）")
    p.add_argument("--rpi-ip", default=None, help="树莓派 IP；不填则不发送 UDP")
    p.add_argument("--rpi-port", type=int, default=DEFAULT_UDP_PORT, help="树莓派 UDP 端口")
    p.add_argument(
        "--urdf",
        type=str,
        nargs="?",
        default=DEFAULT_URDF,
        const=DEFAULT_URDF,
        help="运动学 URDF；默认仅从 arm_control_bridge/configuration/ 读取（V8 优先，其次 SINGLE）；若缺失则报错退出",
    )
    p.add_argument("--calib-file", type=str, default=None)
    p.add_argument("--q5-deg", type=float, default=0.0, help="pose IK 中 q5 固定角（度）")
    p.add_argument("--sim", action="store_true", help="启动 Isaac Sim")
    p.add_argument("--headless", action="store_true", help="仿真无窗口（仅 --sim）")
    p.add_argument(
        "--sim-usd",
        type=str,
        default=None,
        help="机械臂 USD（仅 --sim）；未指定时仅从本包目录 arm_control_bridge/configuration/ 查找：ice_cream_v8_arm.usd、ice_cream_single_arm.usd、ice_cream_arm.usd",
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
    if urdf is None:
        print(
            "错误：未在 arm_control_bridge/configuration/ 找到默认 URDF（ice_cream_v8.SLDASM.urdf 或 ice_cream_SINGLE.SLDASM.urdf），请补齐文件或显式传 --urdf。"
        )
        sys.exit(1)
    calib = args.calib_file or _INITIAL_POSITION_MD

    if args.sim:
        run_sim_loop(
            listen_host=args.listen,
            listen_port=args.port,
            rpi_ip=args.rpi_ip,
            rpi_port=args.rpi_port,
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
            urdf_path=urdf,
            calib_file=calib,
            q5_deg=args.q5_deg,
            web_port=args.web_port,
            web_host=args.web_host,
        )


if __name__ == "__main__":
    main()

