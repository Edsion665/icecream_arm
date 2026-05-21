from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from collections import deque
from typing import Deque

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
from src.planner import (
    build_target_queue,
    peek_target_track,
    plan_pick,
    queued_to_track_slot,
)
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
        self._last_place_wrist_deg: float = 0.0

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

    def _apply_arm_speed(self, axes_rad_s: list[float], label: str) -> None:
        """下发四轴 ramp 速度上限（rad/s），见 bridge ``POST /api/speed``。"""
        self.speaker.send_speed(axes_rad_s, context=label)

    def _apply_speed_pi_default(self, label: str) -> None:
        """四轴全零 → Pi 沿用 config.max_cmd_speed_rad_s（非 bridge 内部默认）。"""
        self._apply_arm_speed([0.0, 0.0, 0.0, 0.0], label)

    def _apply_speed_place(self, label: str) -> None:
        self._apply_arm_speed(list(self.settings.arm_speed_place_rad_s), label)

    def _apply_speed_approach(self, label: str) -> None:
        self._apply_arm_speed(list(self.settings.arm_speed_approach_rad_s), label)

    def _work_hover_and_pose(
        self, slot: TrackSlot, role: Role
    ) -> tuple[Position, Position]:
        """工作位与上方预接近点（+approach_hover_m）。"""
        work = self._work_pose(slot, role)
        h = float(self.settings.approach_hover_m)
        hover = Position(work.x, work.y, work.z + h)
        return hover, work

    def _pose_seq_to(self, pos: Position, label: str) -> None:
        """笛卡尔目标：bridge ``pose_seq``（25Hz 关节序列，末端竖直）。"""
        self.speaker.send_pose_seq(pos.x, pos.y, pos.z, context=label)

    def _approach_then_descend(
        self,
        slot: TrackSlot,
        role: Role,
        wrist_deg: float,
        *,
        tag: str,
        grip_closed: bool,
        descend_speed: str = "approach",
    ) -> None:
        """先到工作位上方 approach_hover_m，再 pose_seq 下降到工作位；bridge POST 阻塞到位后返回。

        悬停段用 Pi 默认速度（不限速）；仅下降段可设 ``descend_speed`` 为 ``approach`` / ``place``。
        ``grip_closed`` 仅控制预接近/下降过程中是否保持夹爪状态（抓取应 False=张开；
        放置应 True=闭合）。夹紧/松爪在调用方于 ``approach_settle_after_descend_s`` 之后 ``send_claw``。
        """
        s = self.settings
        hover, work = self._work_hover_and_pose(slot, role)
        h = float(self.settings.approach_hover_m)
        log.info(
            "%s: 预接近 上方+%.3fm → (%.3f,%.3f,%.3f) 再 pose_seq 下降 (%.3f,%.3f,%.3f) | 夹爪=%s",
            tag,
            h,
            hover.x,
            hover.y,
            hover.z,
            work.x,
            work.y,
            work.z,
            "闭合" if grip_closed else "张开",
        )
        self._emit_claw(wrist_deg, f"{tag}_align_wrist", closed=grip_closed)
        # 悬停：不 POST /api/speed（保持 Pi 默认）；bridge pose_seq 对全零速度用 pose_seq_plan_speed_rad_s 算帧数
        self._pose_seq_to(hover, f"{tag}_goto_hover")
        if descend_speed == "place":
            self._apply_speed_place(f"{tag}_descend_speed")
        elif descend_speed == "approach":
            self._apply_speed_approach(f"{tag}_descend_speed")
        self._pose_seq_to(work, f"{tag}_descend_to_work")
        settle = float(s.approach_settle_after_descend_s)
        if settle > 0.0:
            log.info("%s: 到位后额外等待 %.2fs", tag, settle)
            time.sleep(settle)
        self._apply_speed_pi_default(f"{tag}_after_descend")

    def _pose_to(self, pos: Position, label: str, *, role: Role) -> None:
        """向 bridge 发笛卡尔目标；阻塞至 bridge 判到位（见 ``send_pose``）。"""
        self.speaker.send_pose(pos.x, pos.y, pos.z, context=label)

    def _place_descend_then_release(
        self,
        tgt: TrackSlot,
        place_wrist: float,
        *,
        tag: str = "step7",
        descend_speed: str = "place",
    ) -> None:
        """放置：上方预接近 → pose_seq 下降到位 → 再松爪。"""
        s = self.settings
        self._approach_then_descend(
            tgt,
            "target",
            place_wrist,
            tag=tag,
            grip_closed=True,
            descend_speed=descend_speed,
        )
        log.info("%s: 已确认下降到位，松爪", tag)
        self.speaker.send_claw(
            place_wrist,
            closed=not s.claw_open_for_place,
            context=f"{tag}_release_at_place",
        )
        if s.claw_settle_after_place_s > 0:
            time.sleep(s.claw_settle_after_place_s)

    def _work_pose(self, slot: TrackSlot, role: Role) -> Position:
        """抓取/放置到位高度：真实目标 + role_z_offset_m。"""
        return work_pose_for_role(slot.position, role, self.settings)

    @staticmethod
    def _is_hover_refine_ik_failure(exc: BaseException) -> bool:
        msg = str(exc).lower()
        return "ik failed" in msg or "pose ik failed" in msg

    def _drop_queued_target(self, q: QueuedTarget, *, reason: str) -> None:
        """从目标队列移除一项（step3 精观测失败时跳过该 pedestal）。"""
        removed = False
        try:
            self.target_queue.remove(q)
            removed = True
        except ValueError:
            for item in list(self.target_queue):
                if item.class_id == q.class_id and item.shape_prefix == q.shape_prefix:
                    self.target_queue.remove(item)
                    removed = True
                    break
        if removed:
            log.warning(
                "队列移除 %s (前缀 %s): %s；剩余 %d 项",
                q.class_id,
                q.shape_prefix,
                reason,
                len(self.target_queue),
            )
        else:
            log.warning("队列移除失败（未找到 %s）: %s", q.class_id, reason)

    def _try_observe_target_at_hover(
        self,
        rough_tgt: TrackSlot,
        label: str,
        *,
        shape_prefix_lock: str,
        preferred: TrackSlot,
    ) -> TrackSlot | None:
        """正上方精观测；IK 失败或超时无 target 时返回 None（调用方删队列项）。"""
        hover_m = float(self.settings.place_reobserve_hover_m)
        obs_timeout = float(self.settings.hover_refine_observe_timeout_s)
        raw_pos = strip_role_z_offset(rough_tgt.position, "target", self.settings)
        hover_pos = Position(raw_pos.x, raw_pos.y, raw_pos.z + hover_m)
        work_pos = self._work_pose(rough_tgt, "target")
        wrist = float(rough_tgt.wrist_yaw_deg)
        log.info(
            "%s: 正上方观测 真实目标上方+%.3f m | slot=%.3f,%.3f,%.3f raw=%.3f,%.3f,%.3f "
            "hover=%.3f,%.3f,%.3f 放置工作高=%.3f,%.3f,%.3f wrist_cam=%.1f timeout=%.1fs",
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
            obs_timeout,
        )
        self._emit_claw(wrist, f"{label}_claw_before_hover")
        try:
            self._pose_to(hover_pos, f"{label}_goto_hover", role="target")
        except RuntimeError as exc:
            if self._is_hover_refine_ik_failure(exc):
                log.warning("%s: 悬停位 IK 失败，跳过队列项 %s", label, rough_tgt.class_id)
                return None
            raise
        self._invalidate_camera_cache(f"{label}_after_hover_before_observe")
        self.tracker.set_role_z_offset_suppressed("target", True)
        try:
            if not self._observe_stable(
                "target",
                f"{label}_observe_at_hover",
                timeout_s=obs_timeout,
                raise_on_fail=False,
            ):
                log.warning(
                    "%s: 正上方观测 %.1fs 内无稳定 target，跳过队列项 %s",
                    label,
                    obs_timeout,
                    rough_tgt.class_id,
                )
                return None
        finally:
            self.tracker.set_role_z_offset_suppressed("target", False)
        snap = self.tracker.get_snapshot()
        refined = peek_target_track(
            snap,
            self.target_queue,
            shape_prefix_lock=shape_prefix_lock,
            preferred=preferred,
        )
        if refined is None:
            log.warning(
                "%s: 观测后无匹配 target（前缀 %r），跳过队列项 %s",
                label,
                shape_prefix_lock,
                rough_tgt.class_id,
            )
            return None
        if shape_prefix(refined.class_id) != shape_prefix_lock:
            log.warning(
                "%s: target 前缀不一致 want=%s got=%s，跳过队列项",
                label,
                shape_prefix_lock,
                refined.class_id,
            )
            return None
        self._log_all_targets(f"{label} 精观测后 target")
        return refined

    def _refine_queue_hover(self, n: int) -> None:
        """step3：对队列前 n 项依次正上方精观测；失败则从队列删除。"""
        items = list(self.target_queue)[: max(0, int(n))]
        if not items:
            log.warning("step3: 队列为空，跳过精观测")
            return
        for i, q in enumerate(items):
            if q not in self.target_queue:
                continue
            rough = queued_to_track_slot(q)
            refined = self._try_observe_target_at_hover(
                rough,
                f"step3_hover_{i}",
                shape_prefix_lock=q.shape_prefix,
                preferred=rough,
            )
            if refined is None:
                self._drop_queued_target(
                    q,
                    reason="hover_observe_timeout_or_ik_failed",
                )
                continue
            q.position = refined.position
            q.wrist_yaw_deg = refined.wrist_yaw_deg
            q.refined = True
            log.info(
                "step3[%d] 已写回队列 %s refined pos=(%.3f,%.3f,%.3f)",
                i,
                q.class_id,
                q.position.x,
                q.position.y,
                q.position.z,
            )

    def _pose_delta_to(self, dx: float, dy: float, dz: float, label: str) -> None:
        """相对 bridge 当前 pose 目标增量移动（用于抓取/放置后上抬）。"""
        self.speaker.send_pose_delta(dx, dy, dz, context=label)

    def _retreat_lift_and_wrist(
        self,
        *,
        hold_wrist_deg: float,
        grip_closed: bool,
        next_wrist_deg: float,
        tag: str,
    ) -> None:
        """夹/松爪后：保持夹爪 → 上抬 → 转腕（不回观测关节位）。"""
        lift_m = float(self.settings.retreat_lift_m)
        if lift_m > 0.0:
            self._emit_claw(hold_wrist_deg, f"{tag}_claw_before_lift", closed=grip_closed)
            log.info("%s: 上抬 dz=+%.3f m（pose_delta）", tag, lift_m)
            self._pose_delta_to(0.0, 0.0, lift_m, f"{tag}_lift")
        if abs(hold_wrist_deg - next_wrist_deg) > 1e-6:
            self._emit_claw(next_wrist_deg, f"{tag}_claw_wrist", closed=grip_closed)

    def _retreat_to_obs1_after_pick(
        self,
        *,
        pick_wrist_deg: float,
        place_wrist_deg: float,
        obs_wrist_deg: float,
        tag: str,
    ) -> None:
        """step6：上抬 → 放置腕角 → joints 回 obs1（夹爪保持闭合）；在 Pi 默认速度下完成。"""
        self._retreat_lift_and_wrist(
            hold_wrist_deg=pick_wrist_deg,
            grip_closed=self.settings.claw_closed_for_pick,
            next_wrist_deg=place_wrist_deg,
            tag=tag,
        )
        self._joints_obs(list(self.settings.observe1_axes_rel_deg), f"{tag}_obs1_joints")
        self._emit_claw(
            obs_wrist_deg,
            f"{tag}_claw_at_obs1",
            closed=self.settings.claw_closed_for_pick,
        )

    def _go_obs2_joints(self, *, from_wrist_deg: float, grip_closed: bool, tag: str) -> None:
        """上抬（可选）→ obs2 入口腕角 → joints obs2。"""
        obs2_wrist = float(self.settings.obs2_entry_wrist_deg)
        self._retreat_lift_and_wrist(
            hold_wrist_deg=from_wrist_deg,
            grip_closed=grip_closed,
            next_wrist_deg=obs2_wrist,
            tag=tag,
        )
        self._joints_obs(list(self.settings.observe2_axes_rel_deg), f"{tag}_obs2_joints")
        self._emit_claw(obs2_wrist, f"{tag}_claw_at_obs2", closed=grip_closed)

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

    def _observe_stable(
        self,
        role: Role,
        label: str,
        *,
        timeout_s: float | None = None,
        raise_on_fail: bool = True,
    ) -> bool:
        """至少一个 {role} 连续 N 帧稳定后写入槽位。失败时返回 False 或抛错（由 ``raise_on_fail`` 决定）。"""
        tw = float(timeout_s) if timeout_s is not None else self._cam_wait()
        n = self.settings.observe_stable_frames
        ok = self.tracker.observe_role_stable(role, tw)
        if ok:
            return True
        if not raise_on_fail:
            return False
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

    def _pending_queue_count(self) -> int:
        return sum(1 for q in self.target_queue if not q.placed)

    def _mark_queue_item_placed(self, tgt: TrackSlot) -> None:
        """放置完成：仅标记队项，不删队；stepper 后再 ``target_queue.clear()``。"""
        if not self.target_queue:
            return
        planned = self._cycle_queued_target
        if planned is not None:
            for q in self.target_queue:
                if q.shape_prefix == planned.shape_prefix and q.class_id == planned.class_id:
                    q.placed = True
                    log.info(
                        "队列标记已放置: %s（剩余待做 %d / 共 %d）",
                        q.class_id,
                        self._pending_queue_count(),
                        len(self.target_queue),
                    )
                    return
        prefix = shape_prefix(tgt.class_id)
        for q in self.target_queue:
            if q.shape_prefix == prefix:
                q.placed = True
                log.info(
                    "队列标记已放置(按前缀): %s（剩余待做 %d / 共 %d）",
                    q.class_id,
                    self._pending_queue_count(),
                    len(self.target_queue),
                )
                return
        log.warning("放置完成但未匹配队列项 prefix=%s", prefix)

    def _clear_target_queue_after_stepper(self) -> None:
        n = len(self.target_queue)
        if n:
            log.info("stepper 后清空目标队列（%d 项）", n)
        self.target_queue.clear()

    def _cycle_reset(self) -> None:
        self._discard_slot("target", "cycle_start_discard_target")
        self._discard_slot("object", "cycle_start_discard_object")
        self.target_queue.clear()
        self._cycle_shape_prefix = None
        self._cycle_planned_target = None
        self._cycle_queued_target = None
        self._last_place_wrist_deg = 0.0

    def _setup_obs1_coarse_queue(self) -> None:
        """step1–2：obs1 关节位 + 粗观测建队。"""
        s = self.settings
        tw = s.claw_after_joints_wrist_deg
        self._joints_obs(list(s.observe1_axes_rel_deg), "step1_obs1_joints")
        self._emit_claw(tw, "step1_claw_wrist0_open", closed=False)
        self._maybe_emit_claw(tw, "step1_emit_dup", closed=False)
        self._invalidate_camera_cache("after_step1_obs1_before_wait_target")
        self._observe_stable("target", "step2_observe_target")
        self._rebuild_target_queue_from_snapshot()
        self._log_all_targets("step2 粗观测 target")
        if not self.target_queue:
            raise RuntimeError("step2: obs1 粗观测后 target 队列为空")

    def _setup_obs2_for_plan(self) -> None:
        """step4：obs2 关节位 + 传送带送料 + 稳定观测 object。"""
        s = self.settings
        obs2_wrist = s.obs2_entry_wrist_deg
        self._emit_claw(obs2_wrist, "step4_claw_before_obs2")
        self._maybe_emit_claw(obs2_wrist, "step4_emit_dup")
        self._joints_obs(list(s.observe2_axes_rel_deg), "step4_obs2_joints")
        self._invalidate_camera_cache("after_step4_obs2_before_wait_object")
        self._ensure_obs2_object_in_zone()
        self._observe_stable("object", "step4_observe_object")
        self._log_all_objects("step4 obs2 object")

    def _pick_place_one(self) -> bool:
        """step5–7：plan → 抓取 → 回 obs1 → 放置。成功返回 True。"""
        s = self.settings
        sp = self.speaker
        tw = s.claw_after_joints_wrist_deg

        if self._pending_queue_count() == 0:
            return False

        snap = self.tracker.get_snapshot()
        pr = plan_pick(s, snap, self.target_queue)
        if not pr.ok or pr.object_slot is None or pr.target_slot is None:
            log.warning("step5 plan_pick 失败: %s", pr.reason)
            return False

        obj = pr.object_slot
        tgt = pr.target_slot
        self._cycle_queued_target = pr.queued_target
        self._cycle_planned_target = tgt
        self._cycle_shape_prefix = shape_prefix(tgt.class_id)
        self._log_pick(obj, tgt)

        pick_wrist = self._wrist_pick(obj)
        place_wrist = self._wrist_place(tgt)
        self._log_place(tgt)

        if s.emit_claw_on_every_transition:
            self._emit_claw(pick_wrist, "step5_claw_after_plan")

        self._approach_then_descend(
            obj,
            "object",
            pick_wrist,
            tag="step5",
            grip_closed=False,
            descend_speed="approach",
        )
        log.info("step5: 已确认下降到位，开始夹紧")

        sp.send_claw(pick_wrist, s.claw_closed_for_pick, context="step5_claw_pick")
        if s.claw_settle_after_pick_s > 0:
            time.sleep(s.claw_settle_after_pick_s)

        self._discard_slot("object", "step5_discard_object_after_pick")

        self._retreat_to_obs1_after_pick(
            pick_wrist_deg=pick_wrist,
            place_wrist_deg=place_wrist,
            obs_wrist_deg=tw,
            tag="step6_after_pick",
        )
        self._place_descend_then_release(
            tgt, place_wrist, tag="step7", descend_speed="place"
        )

        self._last_place_wrist_deg = place_wrist
        self._mark_queue_item_placed(tgt)
        log.info(
            "step7 放置完成，队列待做 %d / 共 %d 项",
            self._pending_queue_count(),
            len(self.target_queue),
        )
        return True

    def _obs2_check_more_work(self) -> bool:
        """step8：回 obs2；队列非空且能 plan 则继续 5–8，否则结束内层循环。"""
        pending = self._pending_queue_count()
        if pending == 0:
            log.info("step8: 队列无待做项，结束内层循环")
            return False

        s = self.settings
        grip_open = not s.claw_open_for_place
        from_wrist = float(self._last_place_wrist_deg)

        self._go_obs2_joints(
            from_wrist_deg=from_wrist,
            grip_closed=grip_open,
            tag="step8_after_place",
        )
        self._invalidate_camera_cache("step8_after_obs2_before_replan")

        self._ensure_obs2_object_in_zone()
        self._observe_stable("object", "step8_observe_object_replan")
        self._log_all_objects("step8 obs2 object（plan 前）")

        snap = self.tracker.get_snapshot()
        pr = plan_pick(s, snap, self.target_queue)
        if pr.ok and pr.object_slot is not None and pr.target_slot is not None:
            log.info(
                "step8: 待做 %d / 共 %d，可继续下一轮抓取（%s）",
                pending,
                len(self.target_queue),
                pr.queued_target.shape_prefix if pr.queued_target else "?",
            )
            return True

        log.info(
            "step8: 待做 %d / 共 %d 但 plan_pick 失败（%s），结束内层循环",
            pending,
            len(self.target_queue),
            pr.reason,
        )
        return False

    def _turntable_step(self) -> None:
        """队列本轮放完后：stepper 增量转动转盘。"""
        s = self.settings
        deg = float(s.turntable_stepper_deg)
        self.speaker.send_stepper(deg, context="turntable_step")
        settle = float(s.stepper_settle_s)
        if settle > 0.0:
            log.info("stepper 后等待 %.1fs", settle)
            time.sleep(settle)

    def run_one_cycle(self) -> None:
        s = self.settings

        self._cycle_reset()
        self._apply_speed_pi_default("cycle_start_pi_default")

        self._setup_obs1_coarse_queue()
        n_refine = min(int(s.max_refine_targets), len(self.target_queue))
        self._refine_queue_hover(n_refine)

        self._setup_obs2_for_plan()

        pick_round = 0
        while self._pending_queue_count() > 0:
            pick_round += 1
            log.info(
                "内层循环 第 %d 轮，队列待做 %d / 共 %d 项",
                pick_round,
                self._pending_queue_count(),
                len(self.target_queue),
            )
            if not self._pick_place_one():
                raise RuntimeError("step5–7 pick_place 失败")
            if not self._obs2_check_more_work():
                break

        self._turntable_step()
        self._clear_target_queue_after_stepper()

        self._discard_slot("target", "cycle_end_discard_target")
        self._discard_slot("object", "cycle_end_discard_object")

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
