from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, List

import yaml


def _parse_optional_camera_wait(raw: Any) -> float | None:
    """YAML 缺省 / null / 非正数 → 无限等待；正数 → 有限超时（秒）。"""
    if raw is None:
        return None
    v = float(raw)
    if v <= 0.0:
        return None
    return v


@dataclass
class Settings:
    bridge_base_url: str = "http://127.0.0.1:8775"
    ingest_host: str = "0.0.0.0"
    ingest_port: int = 8776
    ingest_tcp_port: int | None = None  # optional second listener; None = disabled

    # 树莓派 pi2head：TCP JSON 行，收到 start 后 head 才进入 FSM
    pi2head_host: str = "0.0.0.0"
    pi2head_tcp_port: int = 8778
    # 待命阶段周期向 bridge 下发的关节相对角（deg，长度 4）
    idle_axes_rel_deg: List[float] | None = None
    idle_bridge_hz: float = 2.0
    idle_claw_wrist_deg: float = 0.0
    # 待命阶段夹爪：True=合拢（下发 grip_state=0）
    idle_grip_closed: bool = True

    merge_pos_eps_m: float = 0.005
    merge_yaw_eps_deg: float = 3.0
    # 观测阶段：单目标连续 N 帧位置/腕角相近即写入槽位（不要求同帧内全部目标都稳定）
    observe_stable_frames: int = 5
    observe_stable_pos_eps_m: float | None = None
    observe_stable_yaw_eps_deg: float | None = None
    # 目标队列排序与 object 评分参考点 (robot_base, m)；距此点越近优先级越高
    plan_reference_xyz: List[float] | None = None
    # 相机上报 position.z（m）抬升：target +0.4m，object +0.2m；lid 不变
    target_z_offset_m: float = 0.25
    object_z_offset_m: float = 0.2

    observe1_axes_rel_deg: List[float] | None = None
    observe2_axes_rel_deg: List[float] | None = None

    state_timeout_s: float = 60.0
    refresh_wait_s: float = 2.0
    min_object_confidence: float = 0.0

    # v3 FSM / claw / camera wait：None = 在 obs1/obs2 一直等帧内 target/object，不超时
    camera_wait_timeout_s: float | None = None
    claw_closed_for_pick: bool = True
    # True：放置时张开（下发 grip_state=1）
    claw_open_for_place: bool = True
    claw_after_joints_wrist_deg: float = 0.0
    obs2_entry_wrist_deg: float = 0.0
    emit_claw_on_every_transition: bool = False
    initial_grip_open: bool = True
    # 轮询 /api/reached 时要求 bridge 已有 pi2camera 关节反馈（feedback_available）
    require_bridge_feedback: bool = True
    # 步7 夹紧 / 步11 松开成功后，再等待若干秒再进入后续关节运动，避免夹爪未到位就抬臂
    claw_settle_after_pick_s: float = 1.0
    claw_settle_after_place_s: float = 1.0
    # step3：目标正上方精观测高度（米）；与 target_z_offset_m 无关
    place_reobserve_hover_m: float = 0.6
    # step3：对粗观测队列前 N 项做正上方精观测并写回队列
    max_refine_targets: int = 3
    # step3 正上方观测：该秒数内无稳定 target 则从队列删除该项
    hover_refine_observe_timeout_s: float = 10.0
    # 本轮队列全部放置后：转盘步进增量角（deg，bridge stepper 语义）
    turntable_stepper_deg: float = 90.0
    stepper_settle_s: float = 0.0
    # 抓取/放置后撤离：先沿 +z 平移（米），再转腕，再关节回观测位
    retreat_lift_m: float = 0.1
    # obs2：先探视野若干秒；无 object 或水平距基点过远则开传送带，直至物体进入抓取区
    conveyor_obs2_probe_s: float = 5.0
    conveyor_object_max_xy_m: float = 0.6
    # obs1/obs2 关节到位后丢弃 _last_frame，wait 须等到下一帧 ingest（避免沿用上一轮缓存）
    require_fresh_detection_after_obs: bool = True
    # 下发 joints/pose 后轮询 bridge GET /api/reached 的间隔（秒）
    bridge_reached_poll_s: float = 0.04
    # 抓取/放置：工作位上方预接近高度（米），再 pose_seq 竖直下降到位
    approach_hover_m: float = 0.1
    # bridge POST 判到位后的额外等待（秒），给机械 settling 再夹爪/松爪
    approach_settle_after_descend_s: float = 0.0
    # 预接近后的竖直下降段 ramp 上限（rad/s）；抓取 step5 与放置 step7 共用
    arm_speed_approach_rad_s: List[float] | None = None
    # step6 之后、放置下降前：四轴 ramp 上限（rad/s）；pose_seq 帧数由此估算
    arm_speed_place_rad_s: List[float] | None = None

    def __post_init__(self) -> None:
        if self.observe1_axes_rel_deg is None:
            self.observe1_axes_rel_deg = [0.0, 0.0, -45.0, -60.0]
        if self.observe2_axes_rel_deg is None:
            self.observe2_axes_rel_deg = [0.0, 5.0, -30.0, -50.0]
        if len(self.observe1_axes_rel_deg) != 4 or len(self.observe2_axes_rel_deg) != 4:
            raise ValueError("observe*_axes_rel_deg must have length 4")
        if self.plan_reference_xyz is None:
            self.plan_reference_xyz = [0.0, 0.0, 0.0]
        if len(self.plan_reference_xyz) != 3:
            raise ValueError("plan_reference_xyz must have length 3")
        if self.idle_axes_rel_deg is None:
            self.idle_axes_rel_deg = [0.0, 0.0, 0.0, 0.0]
        if len(self.idle_axes_rel_deg) != 4:
            raise ValueError("idle_axes_rel_deg must have length 4")
        if self.idle_bridge_hz <= 0:
            raise ValueError("idle_bridge_hz must be positive")
        if self.bridge_reached_poll_s <= 0:
            raise ValueError("bridge_reached_poll_s must be positive")
        if self.observe_stable_frames < 1:
            raise ValueError("observe_stable_frames must be >= 1")
        if self.place_reobserve_hover_m < 0.0:
            raise ValueError("place_reobserve_hover_m must be >= 0")
        if self.hover_refine_observe_timeout_s <= 0.0:
            raise ValueError("hover_refine_observe_timeout_s must be > 0")
        if self.approach_hover_m < 0.0:
            raise ValueError("approach_hover_m must be >= 0")
        if self.approach_settle_after_descend_s < 0.0:
            raise ValueError("approach_settle_after_descend_s must be >= 0")
        if self.retreat_lift_m < 0.0:
            raise ValueError("retreat_lift_m must be >= 0")
        if self.conveyor_obs2_probe_s <= 0.0:
            raise ValueError("conveyor_obs2_probe_s must be positive")
        if self.conveyor_object_max_xy_m <= 0.0:
            raise ValueError("conveyor_object_max_xy_m must be positive")
        if self.max_refine_targets < 1:
            raise ValueError("max_refine_targets must be >= 1")
        if self.stepper_settle_s < 0.0:
            raise ValueError("stepper_settle_s must be >= 0")
        if self.arm_speed_place_rad_s is None:
            self.arm_speed_place_rad_s = [0.25, 0.2, 0.25, 0.2]
        if self.arm_speed_approach_rad_s is None:
            self.arm_speed_approach_rad_s = list(self.arm_speed_place_rad_s)
        for name, speeds in (
            ("arm_speed_place_rad_s", self.arm_speed_place_rad_s),
            ("arm_speed_approach_rad_s", self.arm_speed_approach_rad_s),
        ):
            if len(speeds) != 4:
                raise ValueError(f"{name} must have length 4")
            if any(v <= 0.0 for v in speeds):
                raise ValueError(f"{name} values must be > 0")


def load_settings(path: str | Path) -> Settings:
    p = Path(path)
    raw: dict[str, Any] = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    return Settings(
        bridge_base_url=str(raw.get("bridge_base_url", Settings.bridge_base_url)),
        ingest_host=str(raw.get("ingest_host", Settings.ingest_host)),
        ingest_port=int(raw.get("ingest_port", Settings.ingest_port)),
        ingest_tcp_port=(int(raw["ingest_tcp_port"]) if raw.get("ingest_tcp_port") is not None else None),
        pi2head_host=str(raw.get("pi2head_host", Settings.pi2head_host)),
        pi2head_tcp_port=int(raw.get("pi2head_tcp_port", Settings.pi2head_tcp_port)),
        idle_axes_rel_deg=list(raw.get("idle_axes_rel_deg") or [0.0, 0.0, 0.0, 0.0]),
        idle_bridge_hz=float(raw.get("idle_bridge_hz", 2.0)),
        idle_claw_wrist_deg=float(raw.get("idle_claw_wrist_deg", 0.0)),
        idle_grip_closed=(
            bool(raw["idle_grip_closed"])
            if "idle_grip_closed" in raw
            else int(raw.get("idle_grip_state", 0)) == 0
        ),
        merge_pos_eps_m=float(raw.get("merge_pos_eps_m", 0.005)),
        merge_yaw_eps_deg=float(raw.get("merge_yaw_eps_deg", 3.0)),
        observe_stable_frames=int(raw.get("observe_stable_frames", 3)),
        observe_stable_pos_eps_m=(
            float(raw["observe_stable_pos_eps_m"])
            if raw.get("observe_stable_pos_eps_m") is not None
            else None
        ),
        observe_stable_yaw_eps_deg=(
            float(raw["observe_stable_yaw_eps_deg"])
            if raw.get("observe_stable_yaw_eps_deg") is not None
            else None
        ),
        plan_reference_xyz=list(raw.get("plan_reference_xyz") or [0.0, 0.0, 0.0]),
        target_z_offset_m=float(raw.get("target_z_offset_m", 0.4)),
        object_z_offset_m=float(raw.get("object_z_offset_m", 0.2)),
        observe1_axes_rel_deg=list(raw.get("observe1_axes_rel_deg") or [0.0, 0.0, -45.0, -60.0]),
        observe2_axes_rel_deg=list(raw.get("observe2_axes_rel_deg") or [0.0, 5.0, -30.0, -50.0]),
        state_timeout_s=float(raw.get("state_timeout_s", 60.0)),
        refresh_wait_s=float(raw.get("refresh_wait_s", 2.0)),
        min_object_confidence=float(raw.get("min_object_confidence", 0.0)),
        camera_wait_timeout_s=_parse_optional_camera_wait(raw.get("camera_wait_timeout_s")),
        claw_closed_for_pick=bool(raw.get("claw_closed_for_pick", True)),
        claw_open_for_place=bool(raw.get("claw_open_for_place", True)),
        claw_after_joints_wrist_deg=float(raw.get("claw_after_joints_wrist_deg", 0.0)),
        obs2_entry_wrist_deg=float(raw.get("obs2_entry_wrist_deg", 0.0)),
        emit_claw_on_every_transition=bool(raw.get("emit_claw_on_every_transition", False)),
        initial_grip_open=bool(raw.get("initial_grip_open", True)),
        claw_settle_after_pick_s=float(raw.get("claw_settle_after_pick_s", 1.0)),
        claw_settle_after_place_s=float(raw.get("claw_settle_after_place_s", 1.0)),
        place_reobserve_hover_m=float(raw.get("place_reobserve_hover_m", 0.6)),
        retreat_lift_m=float(raw.get("retreat_lift_m", 0.1)),
        conveyor_obs2_probe_s=float(raw.get("conveyor_obs2_probe_s", 5.0)),
        conveyor_object_max_xy_m=float(raw.get("conveyor_object_max_xy_m", 0.6)),
        require_fresh_detection_after_obs=bool(raw.get("require_fresh_detection_after_obs", True)),
        bridge_reached_poll_s=float(raw.get("bridge_reached_poll_s", 0.04)),
        require_bridge_feedback=bool(raw.get("require_bridge_feedback", True)),
        max_refine_targets=int(raw.get("max_refine_targets", 3)),
        hover_refine_observe_timeout_s=float(raw.get("hover_refine_observe_timeout_s", 10.0)),
        turntable_stepper_deg=float(raw.get("turntable_stepper_deg", 90.0)),
        stepper_settle_s=float(raw.get("stepper_settle_s", 0.0)),
        approach_hover_m=float(raw.get("approach_hover_m", 0.1)),
        approach_settle_after_descend_s=float(raw.get("approach_settle_after_descend_s", 0.0)),
        arm_speed_place_rad_s=(
            list(raw["arm_speed_place_rad_s"])
            if raw.get("arm_speed_place_rad_s") is not None
            else None
        ),
        arm_speed_approach_rad_s=(
            list(raw["arm_speed_approach_rad_s"])
            if raw.get("arm_speed_approach_rad_s") is not None
            else None
        ),
    )
