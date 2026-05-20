from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from collections import deque
from typing import Deque, FrozenSet

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
        ack = self.speaker.send_joints(axes, context=label)
        if not ack.ok:
            raise RuntimeError(f"{label}: joints POST failed: {ack.error or ack.body}")
        rep = self.speaker.wait_joints_reached(self._timeout(), context=label)
        if not self.speaker.joints_in_position(rep):
            raise RuntimeError(f"{label}: joints not reached: {rep.error or rep.body}")

    def _pose_to(self, pos: Position, label: str, *, role: Role) -> None:
        """向 bridge 发笛卡尔目标（z 已在相机 ingest 时按 role 抬升）。"""
        ack = self.speaker.send_pose(pos.x, pos.y, pos.z, context=label)
        if not ack.ok:
            raise RuntimeError(f"{label}: pose POST failed: {ack.error or ack.body}")
        rep = self.speaker.wait_pose_reached(self._timeout(), context=label)
        if not self.speaker.pose_in_position(rep):
            raise RuntimeError(f"{label}: pose not reached: {rep.error or rep.body}")

    def _observe_stable(self, role: Role, label: str) -> None:
        """等待连续 N 帧位置/腕角稳定后再写入槽位。"""
        tw = self._cam_wait()
        n = self.settings.observe_stable_frames
        ok = self.tracker.observe_role_stable(role, tw)
        if not ok:
            if tw is None or tw <= 0.0:
                raise RuntimeError(f"{label}: 观测被中断（tracker 已停止）")
            raise RuntimeError(
                f"{label}: {tw}s 内未见 {role} 连续 {n} 帧稳定；"
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
            f" wrist={s.wrist_yaw_deg:.1f}"
        )

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
        log.info(
            "抓取 %s -> 配对目标(规划) %s",
            self._fmt_slot_pos(obj),
            self._fmt_slot_pos(tgt_plan),
        )

    def _log_place(self, tgt: TrackSlot) -> None:
        log.info("放置 %s", self._fmt_slot_pos(tgt))

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

        # 4: obs2 稳定观测 object
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

        if s.emit_claw_on_every_transition:
            self._emit_claw(obj.wrist_yaw_deg, "step5_claw_after_plan")

        # 6: pose to object (wrist aligned in claw before move)
        self._emit_claw(obj.wrist_yaw_deg, "step6_claw_before_object_pose")
        self._pose_to(obj.position, "step6_object_pose", role="object")

        # 7: pick claw + object wrist
        cr = sp.send_claw(obj.wrist_yaw_deg, s.claw_closed_for_pick, context="step7_claw_pick")
        sp.require_claw(cr, "step7_claw_pick")
        if s.claw_settle_after_pick_s > 0:
            time.sleep(s.claw_settle_after_pick_s)

        # 规划用 object 已消费，丢弃槽位；下次 object 必须再在 obs2 后等待帧并 clear+apply
        self._discard_slot("object", "step7_discard_object_after_pick")

        # 8: 物体抓取后先关节回 obs1，到位后再腕零（回程中保持步7腕角直至观测位）
        self._joints_obs(list(s.observe1_axes_rel_deg), "step8_obs1_return_joints")
        self._emit_claw(tw, "step8_claw_wrist0")
        self._maybe_emit_claw(tw, "step8_emit_dup")
        self._invalidate_camera_cache("after_step8_obs1_before_wait_target_place")

        # 9: obs1 第二次稳定观测 target（放置用，不得沿用抓取前槽位）
        self._observe_stable("target", "step9_observe_target")
        self._log_all_targets("obs1 第2次 target")

        # 10: 仅选与手中物体同前缀的 target；多个时按 step5 规划位消歧
        snap2 = self.tracker.get_snapshot()
        tgt = peek_target_track(
            snap2,
            self.target_queue,
            shape_prefix_lock=self._cycle_shape_prefix,
            preferred=self._cycle_planned_target,
        )
        if tgt is None:
            raise RuntimeError(
                f"手中前缀 {self._cycle_shape_prefix!r}，obs1 无同形状 target，无法放置"
            )
        if self._cycle_shape_prefix and shape_prefix(tgt.class_id) != self._cycle_shape_prefix:
            raise RuntimeError(
                f"target 前缀不一致: want {self._cycle_shape_prefix}, got {tgt.class_id}"
            )
        self._log_place(tgt)

        self._emit_claw(tgt.wrist_yaw_deg, "step10_claw_before_target_pose")
        self._pose_to(tgt.position, "step10_target_pose", role="target")

        self._pop_queue_after_place(tgt)

        # 11: place claw + target wrist
        cr2 = sp.send_claw(
            tgt.wrist_yaw_deg, closed=not s.claw_open_for_place, context="step11_claw_place"
        )
        sp.require_claw(cr2, "step11_claw_place")
        if s.claw_settle_after_place_s > 0:
            time.sleep(s.claw_settle_after_place_s)

        # 目标位放置后先回 obs1，再腕零（回程保持步11张开+目标腕角直至观测位）
        self._joints_obs(list(s.observe1_axes_rel_deg), "step11b_obs1_after_place")
        self._emit_claw(tw, "step11b_claw_wrist0_after_return")
        self._maybe_emit_claw(tw, "step11b_emit_dup")

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
