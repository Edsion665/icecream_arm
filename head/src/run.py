from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from collections import deque
from typing import Deque, FrozenSet

from src.conveyor_obs import object_in_pick_zone
from src.coordinates import (
    strip_role_z_offset,
    work_pose_for_role,
    wrist_deg_for_object,
    wrist_deg_for_target,
    wrist_j1_bias_deg,
)
from src.config import Settings, load_settings
from src.listener import IngestionServer
from src.models import Position, QueuedTarget, Role, TrackSlot
from src.pi2head_listener import Pi2HeadServer
from src.planner import build_target_queue, peek_target_track, plan_pick
from src.shape_match import shape_prefix
from src.speaker import BridgeClient
from src.tracker import Tracker

log = logging.getLogger("src.run")


class HeadFSM:
    def __init__(
        self, settings: Settings, tracker: Tracker, speaker: BridgeClient, target_queue: Deque[QueuedTarget]
    ) -> None:
        self.settings = settings
        self.tracker = tracker
        self.speaker = speaker
        self.target_queue = target_queue
        self._cycle_shape_prefix: str | None = None
        self._cycle_planned_target: TrackSlot | None = None
        self._cycle_queued_target: QueuedTarget | None = None

    def _timeout(self) -> float:
        return self.settings.state_timeout_s

    def _cam_wait(self) -> float | None:
        return self.settings.camera_wait_timeout_s

    def _emit_claw(self, wrist_deg: float, label: str, *, closed: bool | None = None) -> None:
        grip_closed = self.speaker.last_grip_closed if closed is None else closed
        rep = self.speaker.send_claw(wrist_deg, grip_closed, context=label)
        self.speaker.require_claw(rep, label)

    def _maybe_emit_claw(self, wrist_deg: float, label: str, *, closed: bool | None = None) -> None:
        if self.settings.emit_claw_on_every_transition:
            self._emit_claw(wrist_deg, label, closed=closed)

    def _joints_obs(self, axes: list[float], label: str) -> None:
        self.speaker.send_joints(axes, context=label)

    def _pose_to(self, pos: Position, label: str, *, role: Role) -> None:
        """向 bridge 发笛卡尔目标（pos 为最终笛卡尔目标，已含工作高度偏置时用 work_pose）。"""
        self.speaker.send_pose(pos.x, pos.y, pos.z, context=label)

    def _work_pose(self, slot: TrackSlot, role: Role) -> Position:
        """抓取/放置到位高度：真实目标 + role_z_offset_m。"""
        return work_pose_for_role(slot.position, role, self.settings)

    def _place_reobserve_at_hover(self, rough_tgt: TrackSlot, label: str) -> TrackSlot:
        """真实目标上方 place_reobserve_hover_m 重观测；ingest 关闭 z 偏置与 j1 腕角补偿。"""
        hover_m = float(self.settings.place_reobserve_hover_m)
        raw_pos = strip_role_z_offset(rough_tgt.position, "target", self.settings)
        hover_pos = Position(raw_pos.x, raw_pos.y, raw_pos.z + hover_m)
        work_pos = self._work_pose(rough_tgt, "target")
        wrist = float(rough_tgt.wrist_yaw_deg)
        log.info(
            "%s: 悬停重观测 真实目标上方+%.3f m | slot=%.3f,%.3f,%.3f raw=%.3f,%.3f,%.3f "
            "hover=%.3f,%.3f,%.3f 放置工作高=%.3f,%.3f,%.3f wrist_cam=%.1f",
            label,
            hover_m,
            rough_tgt.position.x,
            rough_tgt.position.y,
            rough_tgt.position.z,
            raw_pos.x,
            raw_pos.y,
            raw_pos.z,
            hover_pos.x,
            hover_pos.y,
            hover_pos.z,
            work_pos.x,
            work_pos.y,
            work_pos.z,
            wrist,
        )
        self._emit_claw(wrist, f"{label}_claw_before_hover")
        self._pose_to(hover_pos, f"{label}_goto_hover", role="target")
        self._invalidate_camera_cache(f"{label}_after_hover_before_observe")
        self.tracker.set_role_z_offset_suppressed("target", True)
        try:
            self._observe_stable("target", f"{label}_observe_at_hover")
        finally:
            self.tracker.set_role_z_offset_suppressed("target", False)
        snap = self.tracker.get_snapshot()
        refined = peek_target_track(
            snap,
            self.target_queue,
            shape_prefix_lock=self._cycle_shape_prefix,
            preferred=self._cycle_planned_target,
        )
        if refined is None:
            raise RuntimeError(
                f"{label}: 悬停重观测后无匹配 target（前缀 {self._cycle_shape_prefix!r}）"
            )
        if self._cycle_shape_prefix and shape_prefix(refined.class_id) != self._cycle_shape_prefix:
            raise RuntimeError(
                f"target 前缀不一致: want {self._cycle_shape_prefix}, got {refined.class_id}"
            )
        self._log_all_targets(f"{label} 重观测后 target")
        return refined

    def _pose_delta_to(self, dx: float, dy: float, dz: float, label: str) -> None:
        """相对 bridge 当前 pose 目标增量移动（用于抓取/放置后上抬）。"""
        self.speaker.send_pose_delta(dx, dy, dz, context=label)

    def _retreat_to_obs(
        self,
        *,
        hold_wrist_deg: float,
        grip_closed: bool,
        obs_axes: list[float],
        obs_wrist_deg: float,
        tag: str,
    ) -> None:
        """夹/松爪后：保持夹爪与腕角 → 相对当前位姿上抬 → 转腕 → 关节回观测位。"""
        lift_m = float(self.settings.retreat_lift_m)
        if lift_m > 0.0:
            self._emit_claw(hold_wrist_deg, f"{tag}_claw_before_lift", closed=grip_closed)
            log.info("%s: 上抬 dz=+%.3f m（pose_delta，相对 bridge 当前位姿）", tag, lift_m)
            self._pose_delta_to(0.0, 0.0, lift_m, f"{tag}_lift")
        if abs(hold_wrist_deg - obs_wrist_deg) > 1e-6:
            self._emit_claw(obs_wrist_deg, f"{tag}_claw_wrist_before_obs", closed=grip_closed)
        self._joints_obs(obs_axes, f"{tag}_obs_joints")
        self._emit_claw(obs_wrist_deg, f"{tag}_claw_at_obs", closed=grip_closed)
        self._maybe_emit_claw(obs_wrist_deg, f"{tag}_claw_at_obs_dup", closed=grip_closed)

    def _set_conveyor(self, run: int, label: str) -> None:
        rep = self.speaker.send_conveyor(run, context=label)
        if not rep.ok:
            raise RuntimeError(f"{label}: conveyor POST failed: {rep.error or rep.body}")

    def _obs2_last_frame(self):
        return self.tracker.get_snapshot().last_frame

    def _obs2_probe_object_ready(self, probe_s: float, max_xy_m: float) -> bool:
        """obs2 探视野：若 probe_s 内已有 object 且水平距基点 <= max_xy_m 则 True。"""
        deadline = time.monotonic() + float(probe_s)
        poll_s = max(self.settings.bridge_reached_poll_s, 0.05)
        while time.monotonic() < deadline:
            if object_in_pick_zone(self._obs2_last_frame(), max_xy_m):
                return True
            time.sleep(poll_s)
        return False

    def _conveyor_until_object_ready(self, max_xy_m: float) -> None:
        """开传送带，直到最近 object 水平距基点 <= max_xy_m，再停带。"""
        self._set_conveyor(1, "obs2_conveyor_start")
        poll_s = max(self.settings.bridge_reached_poll_s, 0.05)
        try:
            while True:
                frame = self._obs2_last_frame()
                if object_in_pick_zone(frame, max_xy_m):
                    log.info(
                        "obs2 传送带：物体已进入抓取区（水平距基点 <= %.2f m）",
                        max_xy_m,
                    )
                    return
                time.sleep(poll_s)
        finally:
            self._set_conveyor(0, "obs2_conveyor_stop")

    def _ensure_obs2_object_in_zone(self) -> None:
        s = self.settings
        probe_s = float(s.conveyor_obs2_probe_s)
        max_xy = float(s.conveyor_object_max_xy_m)
        if self._obs2_probe_object_ready(probe_s, max_xy):
            log.info(
                "obs2: %.1fs 内已有合格 object（水平距基点 <= %.2f m），无需传送带",
                probe_s,
                max_xy,
            )
            return
        log.info(
            "obs2: %.1fs 内无合格 object（无物体或水平距基点 > %.2f m），启动传送带",
            probe_s,
            max_xy,
        )
        self._conveyor_until_object_ready(max_xy)

    def _observe_stable(self, role: Role, label: str) -> None:
        """至少一个 {role} 连续 N 帧稳定后写入槽位（仅写入已达 N 帧的目标）。"""
        tw = self._cam_wait()
        n = self.settings.observe_stable_frames
        ok = self.tracker.observe_role_stable(role, tw)
        if not ok:
            if tw is None or tw <= 0.0:
                raise RuntimeError(f"{label}: 观测被中断（tracker 已停止）")
            raise RuntimeError(
                f"{label}: {tw}s 内未见任一 {role} 连续 {n} 帧稳定；"
                f"可增大 camera_wait_timeout_s 或放宽 observe_stable_*"
            )

    def _discard_slot(self, role: Role, label: str) -> None:
        """清空槽位（用完即弃，不从 last_frame 回填）。"""
        if not self.tracker.clear_role(role, timeout=self._timeout()):
            raise RuntimeError(f"{label}: clear_role({role}) timeout")

    def _invalidate_camera_cache(self, label: str) -> None:
        if not self.settings.require_fresh_detection_after_obs:
            return
        if not self.tracker.invalidate_last_frame(timeout=self._timeout()):
            raise RuntimeError(f"{label}: invalidate_last_frame timeout")

    def _plan_ref(self) -> Position:
        o = self.settings.plan_reference_xyz
        return Position(float(o[0]), float(o[1]), float(o[2]))

    def _fmt_slot_pos(self, s: TrackSlot) -> str:
        return (
            f"{s.class_id} ({s.position.x:.3f},{s.position.y:.3f},{s.position.z:.3f})"
            f" wrist_cam={s.wrist_yaw_deg:.1f}"
        )

    def _wrist_pick(self, slot: TrackSlot) -> float:
        return wrist_deg_for_object(slot.wrist_yaw_deg, slot.position, self.settings)

    def _wrist_place(self, slot: TrackSlot) -> float:
        return wrist_deg_for_target(slot.wrist_yaw_deg, slot.position, self.settings)

    def _log_all_targets(self, title: str) -> None:
        snap = self.tracker.get_snapshot()
        slots = list(snap.slots_by_role.get("target") or [])
        if not slots:
            log.info("%s: (无)", title)
            return
        log.info("%s: %s", title, " | ".join(self._fmt_slot_pos(s) for s in slots))

    def _log_all_objects(self, title: str) -> None:
        snap = self.tracker.get_snapshot()
        slots = list(snap.slots_by_role.get("object") or [])
        if not slots:
            log.info("%s: (无)", title)
            return
        log.info("%s: %s", title, " | ".join(self._fmt_slot_pos(s) for s in slots))

    def _log_pick(self, obj: TrackSlot, tgt_plan: TrackSlot) -> None:
        s = self.settings
        j1_obs = float(s.observe2_axes_rel_deg[0])
        bias = wrist_j1_bias_deg(obj.position.x, obj.position.y, j1_obs)
        log.info(
            "抓取 %s j1_bias=%.1f wrist_cmd=%.1f -> 配对目标(规划) %s",
            self._fmt_slot_pos(obj),
            bias,
            self._wrist_pick(obj),
            self._fmt_slot_pos(tgt_plan),
        )

    def _log_place(self, tgt: TrackSlot) -> None:
        s = self.settings
        j1_obs = float(s.observe1_axes_rel_deg[0])
        bias = wrist_j1_bias_deg(tgt.position.x, tgt.position.y, j1_obs)
        log.info(
            "放置 %s j1_bias=%.1f wrist_cmd=%.1f",
            self._fmt_slot_pos(tgt),
            bias,
            self._wrist_place(tgt),
        )

    def _rebuild_target_queue_from_snapshot(self) -> None:
        snap = self.tracker.get_snapshot()
        targets = list(snap.slots_by_role.get("target") or [])
        self.target_queue.clear()
        self.target_queue.extend(build_target_queue(targets, self._plan_ref()))

    def _pop_queue_after_place(self, tgt: TrackSlot) -> None:
        if not self.target_queue:
            return
        planned = self._cycle_queued_target
        if planned is not None:
            for i, q in enumerate(self.target_queue):
                if q.shape_prefix == planned.shape_prefix and q.class_id == planned.class_id:
                    if q.position.dist(planned.position) < 0.02:
                        del self.target_queue[i]
                        return
        head = self.target_queue[0]
        if head.shape_prefix == shape_prefix(tgt.class_id):
            self.target_queue.popleft()

    def run_one_cycle(self) -> None:
        s = self.settings
        sp = self.speaker
        tw = s.claw_after_joints_wrist_deg
        obs2_wrist = s.obs2_entry_wrist_deg

        # 0: 每轮开始丢弃上一轮残留；target/object 只允许在本轮 obs1/obs2 后的 clear+apply 中建立
        self._discard_slot("target", "cycle_start_discard_target")
        self._discard_slot("object", "cycle_start_discard_object")
        self.target_queue.clear()
        self._cycle_shape_prefix = None
        self._cycle_planned_target = None
        self._cycle_queued_target = None

        # 1: 先到 obs1 关节位，再腕零 + 夹爪张开
        self._joints_obs(list(s.observe1_axes_rel_deg), "step1_obs1_joints")
        self._emit_claw(tw, "step1_claw_wrist0_open", closed=False)
        self._maybe_emit_claw(tw, "step1_emit_dup", closed=False)
        self._invalidate_camera_cache("after_step1_obs1_before_wait_target")

        # 2: obs1 稳定观测 target（连续 N 帧）后建队
        self._observe_stable("target", "step2_observe_target")
        self._rebuild_target_queue_from_snapshot()
        self._log_all_targets("obs1 第1次 target")

        # 3: obs2 + claw at obs2 entry wrist
        self._emit_claw(obs2_wrist, "step3_claw_before_obs2")
        self._maybe_emit_claw(obs2_wrist, "step3_emit_dup")
        self._joints_obs(list(s.observe2_axes_rel_deg), "step3_obs2_joints")
        self._invalidate_camera_cache("after_step3_obs2_before_wait_object")

        # 4: obs2 探视野 / 必要时传送带送料，再稳定观测 object
        self._ensure_obs2_object_in_zone()
        self._observe_stable("object", "step4_observe_object")
        self._log_all_objects("obs2 全部 object")

        # 5: PLAN — obs1 队列优先级（近→远）；传送带按同前缀 object 抓取
        snap = self.tracker.get_snapshot()
        pr = plan_pick(s, snap, self.target_queue)
        if not pr.ok or pr.object_slot is None or pr.target_slot is None:
            raise RuntimeError(f"PLAN failed: {pr.reason}")
        obj: TrackSlot = pr.object_slot
        tgt_plan = pr.target_slot
        if tgt_plan is None:
            raise RuntimeError("PLAN internal: target_slot missing")
        self._cycle_queued_target = pr.queued_target
        self._cycle_planned_target = tgt_plan
        self._cycle_shape_prefix = shape_prefix(tgt_plan.class_id)
        self._log_pick(obj, tgt_plan)

        pick_wrist = self._wrist_pick(obj)
        if s.emit_claw_on_every_transition:
            self._emit_claw(pick_wrist, "step5_claw_after_plan")

        # 6: 真实物体 + object_z_offset_m（腕角在移动前已对齐）
        self._emit_claw(pick_wrist, "step6_claw_before_object_pose")
        self._pose_to(self._work_pose(obj, "object"), "step6_object_pose", role="object")

        # 7: pick claw + object wrist
        sp.send_claw(pick_wrist, s.claw_closed_for_pick, context="step7_claw_pick")
        if s.claw_settle_after_pick_s > 0:
            time.sleep(s.claw_settle_after_pick_s)

        # 规划用 object 已消费，丢弃槽位；下次 object 必须再在 obs2 后等待帧并 clear+apply
        self._discard_slot("object", "step7_discard_object_after_pick")

        # 8: 夹紧后上抬 → 转腕 → 回 obs1（夹爪保持闭合）
        self._retreat_to_obs(
            hold_wrist_deg=pick_wrist,
            grip_closed=s.claw_closed_for_pick,
            obs_axes=list(s.observe1_axes_rel_deg),
            obs_wrist_deg=tw,
            tag="step8_after_pick",
        )
        self._invalidate_camera_cache("after_step8_obs1_before_wait_target_place")

        # 9: obs1 第二次稳定观测 target（放置用，不得沿用抓取前槽位）
        self._observe_stable("target", "step9_observe_target")
        self._log_all_targets("obs1 第2次 target")

        # 10: obs1 粗选 target → 移到目标上方 40cm 重观测 → 垂直接近放置位
        snap2 = self.tracker.get_snapshot()
        rough_tgt = peek_target_track(
            snap2,
            self.target_queue,
            shape_prefix_lock=self._cycle_shape_prefix,
            preferred=self._cycle_planned_target,
        )
        if rough_tgt is None:
            raise RuntimeError(
                f"手中前缀 {self._cycle_shape_prefix!r}，obs1 无同形状 target，无法放置"
            )
        if self._cycle_shape_prefix and shape_prefix(rough_tgt.class_id) != self._cycle_shape_prefix:
            raise RuntimeError(
                f"target 前缀不一致: want {self._cycle_shape_prefix}, got {rough_tgt.class_id}"
            )
        self._log_place(rough_tgt)

        tgt = self._place_reobserve_at_hover(rough_tgt, "step10")
        self._log_place(tgt)

        place_wrist = self._wrist_place(tgt)
        self._emit_claw(place_wrist, "step10_claw_before_place_descend")
        self._pose_to(self._work_pose(tgt, "target"), "step10_target_pose", role="target")

        self._pop_queue_after_place(tgt)

        # 11: place claw + target wrist
        sp.send_claw(
            place_wrist, closed=not s.claw_open_for_place, context="step11_claw_place"
        )
        if s.claw_settle_after_place_s > 0:
            time.sleep(s.claw_settle_after_place_s)

        # 11b: 松爪后上抬 → 转腕 → 回 obs1（夹爪保持张开）
        self._retreat_to_obs(
            hold_wrist_deg=place_wrist,
            grip_closed=not s.claw_open_for_place,
            obs_axes=list(s.observe1_axes_rel_deg),
            obs_wrist_deg=tw,
            tag="step11b_after_place",
        )

        # 本轮 target/object 已用完，丢弃；下一轮必须再在 obs1/obs2 后重新获得
        self._discard_slot("target", "step11_discard_target_after_place")
        self._discard_slot("object", "step11_discard_object_after_place")

        # 12: loop — caller runs run_forever

    def run_forever(self) -> None:
        single = os.environ.get("HEAD_SINGLE_CYCLE", "").lower() in ("1", "true", "yes")
        while True:
            try:
                self.run_one_cycle()
            except Exception as e:  # noqa: BLE001
                log.exception("cycle error: %s", e)
                time.sleep(1.0)
            if single:
                break
            time.sleep(0.05)


def _hold_idle_until_pi_start(settings: Settings, speaker: BridgeClient, pi_server: Pi2HeadServer) -> None:
    """收到 pi2head ``start`` 前，周期向 bridge 下发全零关节与 claw。"""
    period = 1.0 / float(settings.idle_bridge_hz)
    axes = list(settings.idle_axes_rel_deg or [0.0, 0.0, 0.0, 0.0])
    log.info(
        "待命：周期 %.2f Hz 向 bridge 下发 idle 全零 joints=%s claw(wrist=%.1f, grip=%s)，"
        "直至 pi2head TCP 收到 start …",
        settings.idle_bridge_hz,
        axes,
        settings.idle_claw_wrist_deg,
        settings.idle_grip_closed,
    )
    while not pi_server.is_started():
        speaker.send_idle_zeros(
            axes,
            wrist_deg=settings.idle_claw_wrist_deg,
            grip_closed=settings.idle_grip_closed,
            context="idle_hold",
        )
        if pi_server.wait_started(timeout=period):
            break
    log.info("pi2head start 已收到，进入 v3 FSM 主循环")


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    ap = argparse.ArgumentParser(description="icecream head FSM")
    ap.add_argument("--config", default="config.yaml", help="Path to YAML config")
    args = ap.parse_args()

    try:
        settings = load_settings(args.config)
    except FileNotFoundError:
        log.error("config not found: %s (copy config.example.yaml)", args.config)
        sys.exit(2)
    except Exception as e:  # noqa: BLE001
        log.error("bad config: %s", e)
        sys.exit(2)

    tracker = Tracker(settings)
    tracker.start()
    ingest = IngestionServer(settings.ingest_host, settings.ingest_port, tracker, settings)
    ingest.start_background(tcp_port=settings.ingest_tcp_port)

    speaker = BridgeClient(settings)
    target_queue: Deque[QueuedTarget] = deque()

    log.info("ingestion http://%s:%s/ (tcp=%s)", settings.ingest_host, settings.ingest_port, settings.ingest_tcp_port)
    log.info("bridge %s", settings.bridge_base_url)

    pi_server = Pi2HeadServer(settings.pi2head_host, settings.pi2head_tcp_port)
    pi_server.start_background()

    fsm = HeadFSM(settings, tracker, speaker, target_queue)
    try:
        _hold_idle_until_pi_start(settings, speaker, pi_server)
        fsm.run_forever()
    except KeyboardInterrupt:
        log.info("interrupt")
    finally:
        pi_server.stop()
        ingest.stop()
        tracker.stop()


if __name__ == "__main__":
    main()
