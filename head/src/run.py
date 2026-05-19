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
from src.models import Position, Role, TrackSlot
from src.pi2head_listener import Pi2HeadServer
from src.planner import peek_target_track, plan_pick
from src.speaker import BridgeClient
from src.tracker import Tracker

log = logging.getLogger("src.run")


class HeadFSM:
    def __init__(self, settings: Settings, tracker: Tracker, speaker: BridgeClient, target_queue: Deque[Position]) -> None:
        self.settings = settings
        self.tracker = tracker
        self.speaker = speaker
        self.target_queue = target_queue

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
        rep = self.speaker.wait_joints_reached(self._timeout())
        if not self.speaker.joints_in_position(rep):
            raise RuntimeError(f"{label}: joints not reached: {rep.error or rep.body}")

    def _pose_to(self, pos: Position, label: str, *, role: Role) -> None:
        """向 bridge 发笛卡尔目标（z 已在相机 ingest 时按 role 抬升）。"""
        log.info("%s: bridge pose role=%s x=%.4f y=%.4f z=%.4f", label, role, pos.x, pos.y, pos.z)
        ack = self.speaker.send_pose(pos.x, pos.y, pos.z, context=label)
        if not ack.ok:
            raise RuntimeError(f"{label}: pose POST failed: {ack.error or ack.body}")
        rep = self.speaker.wait_pose_reached(self._timeout())
        if not self.speaker.pose_in_position(rep):
            raise RuntimeError(f"{label}: pose not reached: {rep.error or rep.body}")

    def _wait_roles(self, roles: FrozenSet[Role], label: str) -> None:
        tw = self._cam_wait()
        ok = self.tracker.wait_for_roles(roles, tw)
        if not ok:
            if tw is None or tw <= 0.0:
                raise RuntimeError(f"{label}: 等待相机帧时被中断（tracker 已停止）")
            raise RuntimeError(
                f"{label}: {tw}s 内未在帧内看到 {set(roles)}；"
                f"外层会整轮重跑 FSM，机械臂会在 obs1/obs2 间反复运动。"
                f"联调请设 camera_wait_timeout_s: null（一直等）或增大秒数。"
            )

    def _clear_apply(self, role: Role, label: str) -> None:
        if not self.tracker.clear_role(role, timeout=self._timeout()):
            raise RuntimeError(f"{label}: tracker clear_role({role}) timeout")
        rset: FrozenSet[Role] = frozenset({role})
        if not self.tracker.apply_roles_from_last_frame(rset, timeout=self._timeout()):
            raise RuntimeError(f"{label}: tracker apply_roles timeout")

    def _discard_slot(self, role: Role, label: str) -> None:
        """清空槽位（用完即弃，不从 last_frame 回填）。"""
        if not self.tracker.clear_role(role, timeout=self._timeout()):
            raise RuntimeError(f"{label}: clear_role({role}) timeout")

    def _invalidate_camera_cache(self, label: str) -> None:
        if not self.settings.require_fresh_detection_after_obs:
            return
        log.info("invalidate_last_frame (%s): 下一 wait 须等新 ingest，避免沿用旧 _last_frame", label)
        if not self.tracker.invalidate_last_frame(timeout=self._timeout()):
            raise RuntimeError(f"{label}: invalidate_last_frame timeout")

    def _log_queue(self, where: str) -> None:
        n = len(self.target_queue)
        if n == 0:
            log.info("%s | target_queue: empty", where)
            return
        heads = []
        for i, p in enumerate(list(self.target_queue)[:3]):
            heads.append(f"#{i}({p.x:.3f},{p.y:.3f},{p.z:.3f})")
        more = f" (+{n - 3} more)" if n > 3 else ""
        log.info("%s | target_queue len=%d heads: %s%s", where, n, " ".join(heads), more)

    def run_one_cycle(self) -> None:
        s = self.settings
        sp = self.speaker
        tw = s.claw_after_joints_wrist_deg
        obs2_wrist = s.obs2_entry_wrist_deg

        log.info(
            "=== cycle start === observe1=%s observe2=%s",
            list(s.observe1_axes_rel_deg),
            list(s.observe2_axes_rel_deg),
        )
        self._log_queue("cycle_start (before discard)")

        # 0: 每轮开始丢弃上一轮残留；target/object 只允许在本轮 obs1/obs2 后的 clear+apply 中建立
        self._discard_slot("target", "cycle_start_discard_target")
        self._discard_slot("object", "cycle_start_discard_object")
        self._log_queue("cycle_start (after discard)")

        # 1: 先到 obs1 关节位，再腕零 + 夹爪张开
        log.info("step1: joints obs1 first, then claw wrist=%.3f + grip open", tw)
        self._joints_obs(list(s.observe1_axes_rel_deg), "step1_obs1_joints")
        self._emit_claw(tw, "step1_claw_wrist0_open", closed=False)
        self._maybe_emit_claw(tw, "step1_emit_dup", closed=False)
        self._invalidate_camera_cache("after_step1_obs1_before_wait_target")

        # 2: wait camera target; clear + apply target from last frame
        log.info("step2: wait target in frame @ obs1, then clear+apply target slots")
        self._wait_roles(frozenset({"target"}), "step2_wait_target_frame")
        self._clear_apply("target", "step2_target_slots")

        # 3: obs2 + claw at obs2 entry wrist
        log.info("step3: move to obs2 (observe2_axes_rel_deg); 正常顺序是 obs1→步2→步3 才到 obs2")
        self._emit_claw(obs2_wrist, "step3_claw_before_obs2")
        self._maybe_emit_claw(obs2_wrist, "step3_emit_dup")
        self._joints_obs(list(s.observe2_axes_rel_deg), "step3_obs2_joints")
        self._invalidate_camera_cache("after_step3_obs2_before_wait_object")

        # 4: wait object in frame; clear + apply object
        self._wait_roles(frozenset({"object"}), "step4_wait_object_frame")
        self._clear_apply("object", "step4_object_slots")

        # 5: PLAN (snapshot after slots filled)
        self._log_queue("step5_plan (queue优先于槽位里的 target)")
        snap = self.tracker.get_snapshot()
        pr = plan_pick(s, snap, self.target_queue)
        if not pr.ok or pr.object_slot is None or pr.target_slot is None:
            raise RuntimeError(f"PLAN failed: {pr.reason}")
        obj: TrackSlot = pr.object_slot
        tgt_plan = pr.target_slot
        if tgt_plan is None:
            raise RuntimeError("PLAN internal: target_slot missing")
        log.info(
            "step5 plan ok | object pos=(%.3f,%.3f,%.3f) wrist=%.2f | target class_id=%s label=%s pos=(%.3f,%.3f,%.3f) wrist=%.2f",
            obj.position.x,
            obj.position.y,
            obj.position.z,
            obj.wrist_yaw_deg,
            tgt_plan.class_id,
            tgt_plan.label,
            tgt_plan.position.x,
            tgt_plan.position.y,
            tgt_plan.position.z,
            tgt_plan.wrist_yaw_deg,
        )

        if s.emit_claw_on_every_transition:
            self._emit_claw(obj.wrist_yaw_deg, "step5_claw_after_plan")

        # 6: pose to object (wrist aligned in claw before move)
        log.info(
            "step6: last claw before object pose wrist=%.2f; head does NOT resend claw during /api/pose "
            "(wrist going to 0 during move → bridge likely not coupling wrist with Cartesian)",
            obj.wrist_yaw_deg,
        )
        self._emit_claw(obj.wrist_yaw_deg, "step6_claw_before_object_pose")
        self._pose_to(obj.position, "step6_object_pose", role="object")

        # 7: pick claw + object wrist
        cr = sp.send_claw(obj.wrist_yaw_deg, s.claw_closed_for_pick, context="step7_claw_pick")
        sp.require_claw(cr, "step7_claw_pick")
        if s.claw_settle_after_pick_s > 0:
            log.info("step7: wait %.2fs after pick (grip close settle)", s.claw_settle_after_pick_s)
            time.sleep(s.claw_settle_after_pick_s)

        # 规划用 object 已消费，丢弃槽位；下次 object 必须再在 obs2 后等待帧并 clear+apply
        self._discard_slot("object", "step7_discard_object_after_pick")

        # 8: 物体抓取后先关节回 obs1，到位后再腕零（回程中保持步7腕角直至观测位）
        log.info("step8: obs1 joints first after pick, then wrist=%.3f (grip stays closed)", tw)
        self._joints_obs(list(s.observe1_axes_rel_deg), "step8_obs1_return_joints")
        self._emit_claw(tw, "step8_claw_wrist0")
        self._maybe_emit_claw(tw, "step8_emit_dup")
        self._invalidate_camera_cache("after_step8_obs1_before_wait_target_place")

        # 9: 在 obs1 下等待 target 帧，再 clear+apply（与步 2 相同；放置用 target 不得沿用抓取前的槽）
        log.info("step9: wait target @ obs1 return, clear+apply slots (步10 peek 时若 target_queue 非空仍优先队列)")
        self._wait_roles(frozenset({"target"}), "step9_wait_target_frame")
        self._clear_apply("target", "step9_target_slots")
        self._log_queue("step9_after_apply (queue 仍在，peek 时队列优先于槽)")

        # 10: pose to target (peek target / queue; do not require object still visible)
        snap2 = self.tracker.get_snapshot()
        tgt = peek_target_track(snap2, self.target_queue)
        if tgt is None:
            raise RuntimeError("no target pose after step9 (queue empty and no target slot)")
        from_queue = getattr(tgt, "class_id", None) == "queue"
        log.info(
            "step10 peek_target: from_queue=%s class_id=%s label=%s pos=(%.3f,%.3f,%.3f) wrist=%.2f",
            from_queue,
            tgt.class_id,
            tgt.label,
            tgt.position.x,
            tgt.position.y,
            tgt.position.z,
            tgt.wrist_yaw_deg,
        )

        log.info(
            "step10: last claw before target pose wrist=%.2f; same as step6 — no claw during pose TCP wait",
            tgt.wrist_yaw_deg,
        )
        self._emit_claw(tgt.wrist_yaw_deg, "step10_claw_before_target_pose")
        self._pose_to(tgt.position, "step10_target_pose", role="target")

        if self.target_queue and getattr(tgt, "class_id", None) == "queue":
            self.target_queue.popleft()
            log.info("step10: popped one position from target_queue after target pose")
            self._log_queue("step10_after_queue_pop")

        # 11: place claw + target wrist
        cr2 = sp.send_claw(tgt.wrist_yaw_deg, s.claw_open_for_place, context="step11_claw_place")
        sp.require_claw(cr2, "step11_claw_place")
        if s.claw_settle_after_place_s > 0:
            log.info("step11: wait %.2fs after place (grip open settle)", s.claw_settle_after_place_s)
            time.sleep(s.claw_settle_after_place_s)

        # 目标位放置后先回 obs1，再腕零（回程保持步11张开+目标腕角直至观测位）
        log.info("step11b: obs1 joints after place, then wrist=%.3f (grip stays open)", tw)
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
                log.info("cycle complete (下一轮将 cycle_start discard → step1 obs1 → …)")
                self._log_queue("cycle_complete")
            except Exception as e:  # noqa: BLE001
                log.exception("cycle error: %s", e)
                self._log_queue("cycle_error_before_retry (整轮将重跑；若刚失败在步10 附近，易误判为「直接去 obs2」)")
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
        settings.idle_grip_state,
    )
    while not pi_server.is_started():
        speaker.send_idle_zeros(
            axes,
            wrist_deg=settings.idle_claw_wrist_deg,
            grip_state=settings.idle_grip_state,
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
    target_queue: Deque[Position] = deque()

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
