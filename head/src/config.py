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
    idle_grip_state: int = 0  # head2bridge：0=合拢，1=张开

    merge_pos_eps_m: float = 0.005
    merge_yaw_eps_deg: float = 3.0

    observe1_axes_rel_deg: List[float] | None = None
    observe2_axes_rel_deg: List[float] | None = None

    state_timeout_s: float = 60.0
    refresh_wait_s: float = 2.0
    min_object_confidence: float = 0.0

    # v3 FSM / claw / camera wait：None = 在 obs1/obs2 一直等帧内 target/object，不超时
    camera_wait_timeout_s: float | None = None
    claw_closed_for_pick: bool = True
    claw_open_for_place: bool = False
    claw_after_joints_wrist_deg: float = 0.0
    obs2_entry_wrist_deg: float = 0.0
    emit_claw_on_every_transition: bool = False
    # True：启动为张开 → 下发 grip_state=1；False：合拢 → grip_state=0（与 head2bridge 0=合拢1=张开一致）
    initial_grip_open: bool = True
    # 步7 夹紧 / 步11 松开成功后，再等待若干秒再进入后续关节运动，避免夹爪未到位就抬臂
    claw_settle_after_pick_s: float = 1.0
    claw_settle_after_place_s: float = 1.0
    # obs1/obs2/回 obs1 关节到位后丢弃 _last_frame，wait 必须等到下一帧 ingest（避免沿用上一轮缓存导致立刻去 obs2）
    require_fresh_detection_after_obs: bool = True

    def __post_init__(self) -> None:
        if self.observe1_axes_rel_deg is None:
            self.observe1_axes_rel_deg = [0.0, 0.0, -45.0, -60.0]
        if self.observe2_axes_rel_deg is None:
            self.observe2_axes_rel_deg = [0.0, 5.0, -30.0, -50.0]
        if len(self.observe1_axes_rel_deg) != 4 or len(self.observe2_axes_rel_deg) != 4:
            raise ValueError("observe*_axes_rel_deg must have length 4")
        if self.idle_axes_rel_deg is None:
            self.idle_axes_rel_deg = [0.0, 0.0, 0.0, 0.0]
        if len(self.idle_axes_rel_deg) != 4:
            raise ValueError("idle_axes_rel_deg must have length 4")
        if self.idle_bridge_hz <= 0:
            raise ValueError("idle_bridge_hz must be positive")
        if self.idle_grip_state not in (0, 1):
            raise ValueError("idle_grip_state must be 0 or 1")


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
        idle_grip_state=int(raw.get("idle_grip_state", 0)),
        merge_pos_eps_m=float(raw.get("merge_pos_eps_m", 0.005)),
        merge_yaw_eps_deg=float(raw.get("merge_yaw_eps_deg", 3.0)),
        observe1_axes_rel_deg=list(raw.get("observe1_axes_rel_deg") or [0.0, 0.0, -45.0, -60.0]),
        observe2_axes_rel_deg=list(raw.get("observe2_axes_rel_deg") or [0.0, 5.0, -30.0, -50.0]),
        state_timeout_s=float(raw.get("state_timeout_s", 60.0)),
        refresh_wait_s=float(raw.get("refresh_wait_s", 2.0)),
        min_object_confidence=float(raw.get("min_object_confidence", 0.0)),
        camera_wait_timeout_s=_parse_optional_camera_wait(raw.get("camera_wait_timeout_s")),
        claw_closed_for_pick=bool(raw.get("claw_closed_for_pick", True)),
        claw_open_for_place=bool(raw.get("claw_open_for_place", False)),
        claw_after_joints_wrist_deg=float(raw.get("claw_after_joints_wrist_deg", 0.0)),
        obs2_entry_wrist_deg=float(raw.get("obs2_entry_wrist_deg", 0.0)),
        emit_claw_on_every_transition=bool(raw.get("emit_claw_on_every_transition", False)),
        initial_grip_open=bool(raw.get("initial_grip_open", True)),
        claw_settle_after_pick_s=float(raw.get("claw_settle_after_pick_s", 1.0)),
        claw_settle_after_place_s=float(raw.get("claw_settle_after_place_s", 1.0)),
        require_fresh_detection_after_obs=bool(raw.get("require_fresh_detection_after_obs", True)),
    )
