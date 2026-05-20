from __future__ import annotations

import math
import queue
import threading
import time
from dataclasses import dataclass, replace
from typing import Any, Dict, FrozenSet, List, Tuple

from src.config import Settings
from src.models import DetectionFrame, ObjectDet, ObsTag, Role, SceneSnapshot, TrackSlot


def _dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _position_ok(p: Tuple[float, float, float]) -> bool:
    return all(math.isfinite(v) for v in p)


def _frame_has_roles(frame: DetectionFrame, roles: FrozenSet[str]) -> bool:
    for o in frame.objects:
        if o.role in roles:
            pos = (o.position.x, o.position.y, o.position.z)
            if _position_ok(pos):
                return True
    return False


def _dets_for_role(frame: DetectionFrame, role: Role) -> List[ObjectDet]:
    out: List[ObjectDet] = []
    for o in frame.objects:
        if o.role != role:
            continue
        pos = (o.position.x, o.position.y, o.position.z)
        if _position_ok(pos):
            out.append(o)
    return out


@dataclass
class _ObsTrack:
    det: ObjectDet
    streak: int = 1


class Tracker:
    """Dedicated thread: SceneSlots + merge (camera2head §7)."""

    def __init__(self, settings: Settings) -> None:
        self._settings = settings
        self._q: queue.Queue[Tuple[str, Any]] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._frame_ready = threading.Event()
        self._slots: Dict[str, List[TrackSlot]] = {"object": [], "target": [], "lid": []}
        self._last_frame: DetectionFrame | None = None
        self._observing_role: Role | None = None
        self._ingest_serial: int = 0
        self._obs_tracks: List[_ObsTrack] = []
        self._obs_last_serial: int = 0
        self._obs_done_ev: threading.Event | None = None
        self._obs_result: list[bool] | None = None
        self._obs_deadline: float | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="tracker", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._q.put(("shutdown", None))
        if self._thread:
            self._thread.join(timeout=2.0)

    def submit_frame(self, frame: DetectionFrame) -> None:
        self._q.put(("ingest", frame))

    def request_refresh(self, obs_tag: ObsTag, timeout: float | None = None) -> bool:
        ev = threading.Event()
        self._q.put(("refresh", (obs_tag, ev)))
        wait = timeout if timeout is not None else self._settings.refresh_wait_s
        return ev.wait(timeout=wait)

    def clear_role(self, role: Role, timeout: float | None = None) -> bool:
        ev = threading.Event()
        self._q.put(("clear", (role, ev)))
        wait = timeout if timeout is not None else self._settings.refresh_wait_s
        return ev.wait(timeout=wait)

    def apply_roles_from_last_frame(self, roles: FrozenSet[Role], timeout: float | None = None) -> bool:
        ev = threading.Event()
        self._q.put(("apply_roles", (roles, ev)))
        wait = timeout if timeout is not None else self._settings.refresh_wait_s
        return ev.wait(timeout=wait)

    def observe_role_stable(self, role: Role, timeout: float | None = None) -> bool:
        """清空 role 槽后，等待连续 N 帧位置/腕角相近，再写入槽位。"""
        ev = threading.Event()
        result: list[bool] = [False]
        self._q.put(("observe_begin", (role, timeout, ev, result)))
        if timeout is None or timeout <= 0.0:
            while not ev.wait(0.25):
                if self._stop.is_set():
                    self._q.put(("observe_cancel", None))
                    return False
        elif not ev.wait(timeout=float(timeout)):
            self._q.put(("observe_cancel", None))
            return False
        return result[0]

    def invalidate_last_frame(self, timeout: float | None = None) -> bool:
        """丢弃相机缓存帧；下一次 wait_for_roles 须等到新的 ingest（用于 obs1/obs2 后必须「再看到」检测）。"""
        ev = threading.Event()
        self._q.put(("invalidate_last_frame", ev))
        wait = timeout if timeout is not None else self._settings.refresh_wait_s
        return ev.wait(timeout=wait)

    def wait_for_roles(self, roles: FrozenSet[Role], timeout: float | None) -> bool:
        """timeout 为 None 或 <=0：一直等到帧内出现所需 role（或 tracker stop）。"""
        self._frame_ready.clear()
        roles_s = frozenset(str(r) for r in roles)
        if timeout is None or timeout <= 0.0:
            while not self._stop.is_set():
                with self._lock:
                    lf = self._last_frame
                    if lf is not None and _frame_has_roles(lf, roles_s):
                        return True
                self._frame_ready.wait(timeout=0.5)
            return False
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            with self._lock:
                lf = self._last_frame
                if lf is not None and _frame_has_roles(lf, roles_s):
                    return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._frame_ready.wait(timeout=min(0.2, remaining))
        return False

    def get_snapshot(self) -> SceneSnapshot:
        with self._lock:
            snap = SceneSnapshot(
                slots_by_role={k: [replace(s) for s in v] for k, v in self._slots.items()},
                last_frame=self._last_frame,
            )
        return snap

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                kind, payload = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            if kind == "shutdown":
                break
            if kind == "ingest":
                self._handle_ingest(payload)
            elif kind == "refresh":
                obs_tag, ev = payload
                self._handle_refresh(obs_tag)
                ev.set()
            elif kind == "clear":
                role, ev = payload
                self._handle_clear(role)
                ev.set()
            elif kind == "apply_roles":
                roles, ev = payload
                self._handle_apply_roles(roles)
                ev.set()
            elif kind == "observe_begin":
                role, obs_timeout, ev, result = payload
                self._handle_observe_begin(role, obs_timeout, ev, result)
            elif kind == "observe_cancel":
                self._handle_observe_cancel()
            elif kind == "invalidate_last_frame":
                ev = payload
                self._handle_invalidate_last_frame()
                ev.set()

    def _handle_invalidate_last_frame(self) -> None:
        with self._lock:
            self._last_frame = None
        self._frame_ready.clear()

    def _handle_ingest(self, frame: DetectionFrame) -> None:
        serial: int
        observing: Role | None
        with self._lock:
            self._ingest_serial += 1
            serial = self._ingest_serial
            self._last_frame = frame
            observing = self._observing_role
            if observing is None:
                self._merge_frame(frame)
        self._frame_ready.set()
        if observing is not None:
            self._observe_step(frame, serial)

    def _handle_refresh(self, _obs_tag: ObsTag) -> None:
        with self._lock:
            if self._last_frame is not None:
                self._merge_frame(self._last_frame)

    def _handle_clear(self, role: Role) -> None:
        with self._lock:
            self._slots.setdefault(role, [])
            self._slots[role] = []

    def _handle_apply_roles(self, roles: FrozenSet[Role]) -> None:
        with self._lock:
            if self._last_frame is None:
                return
            eps_p = self._settings.merge_pos_eps_m
            eps_y = self._settings.merge_yaw_eps_deg
            for det in self._last_frame.objects:
                if det.role in roles:
                    self._merge_one(det, self._last_frame, eps_p, eps_y)

    def _merge_frame(self, frame: DetectionFrame) -> None:
        eps_p = self._settings.merge_pos_eps_m
        eps_y = self._settings.merge_yaw_eps_deg
        for det in frame.objects:
            self._merge_one(det, frame, eps_p, eps_y)

    def _stable_eps(self) -> tuple[float, float, int]:
        s = self._settings
        eps_p = (
            float(s.observe_stable_pos_eps_m)
            if s.observe_stable_pos_eps_m is not None
            else float(s.merge_pos_eps_m)
        )
        eps_y = (
            float(s.observe_stable_yaw_eps_deg)
            if s.observe_stable_yaw_eps_deg is not None
            else float(s.merge_yaw_eps_deg)
        )
        return eps_p, eps_y, int(s.observe_stable_frames)

    def _match_obs_track(
        self, det: ObjectDet, tracks: List[_ObsTrack], eps_p: float, eps_y: float
    ) -> _ObsTrack | None:
        pos = (det.position.x, det.position.y, det.position.z)
        best: _ObsTrack | None = None
        best_d = float("inf")
        for tr in tracks:
            if tr.det.class_id != det.class_id:
                continue
            if det.track_id is not None and tr.det.track_id is not None:
                if det.track_id != tr.det.track_id:
                    continue
            old = (tr.det.position.x, tr.det.position.y, tr.det.position.z)
            dpos = _dist(pos, old)
            dyaw = abs(det.wrist_yaw_deg - tr.det.wrist_yaw_deg)
            if dpos <= eps_p and dyaw <= eps_y and dpos < best_d:
                best_d = dpos
                best = tr
        return best

    def _handle_observe_begin(
        self,
        role: Role,
        timeout: float | None,
        ev: threading.Event,
        result: list[bool],
    ) -> None:
        self._handle_clear(role)
        self._observing_role = role
        self._obs_tracks = []
        with self._lock:
            self._obs_last_serial = self._ingest_serial
        self._obs_done_ev = ev
        self._obs_result = result
        if timeout is None or timeout <= 0.0:
            self._obs_deadline = None
        else:
            self._obs_deadline = time.monotonic() + float(timeout)

    def _handle_observe_cancel(self) -> None:
        self._finish_observe(False)

    def _finish_observe(self, ok: bool) -> None:
        if self._obs_result is not None:
            self._obs_result[0] = ok
        if self._obs_done_ev is not None:
            self._obs_done_ev.set()
        self._observing_role = None
        self._obs_tracks = []
        self._obs_done_ev = None
        self._obs_result = None
        self._obs_deadline = None

    def _observe_step(self, frame: DetectionFrame, serial: int) -> None:
        role = self._observing_role
        if role is None:
            return
        if serial == self._obs_last_serial:
            return
        self._obs_last_serial = serial
        if self._obs_deadline is not None and time.monotonic() > self._obs_deadline:
            self._finish_observe(False)
            return
        eps_p, eps_y, required = self._stable_eps()
        dets = _dets_for_role(frame, role)
        if not dets:
            self._obs_tracks = []
            return
        next_tracks: List[_ObsTrack] = []
        for det in dets:
            tr = self._match_obs_track(det, self._obs_tracks, eps_p, eps_y)
            if tr is None:
                next_tracks.append(_ObsTrack(det=det, streak=1))
            else:
                next_tracks.append(_ObsTrack(det=det, streak=tr.streak + 1))
        self._obs_tracks = next_tracks
        if all(t.streak >= required for t in self._obs_tracks):
            self._apply_dets(role, dets, frame)
            self._finish_observe(True)

    def _apply_dets(self, role: Role, dets: List[ObjectDet], frame: DetectionFrame) -> None:
        eps_p = self._settings.merge_pos_eps_m
        eps_y = self._settings.merge_yaw_eps_deg
        self._observing_role = None
        with self._lock:
            self._slots.setdefault(role, [])
            self._slots[role] = []
            for det in dets:
                self._merge_one(det, frame, eps_p, eps_y)

    def _merge_one(self, det: ObjectDet, frame: DetectionFrame, eps_p: float, eps_y: float) -> None:
        role: Role = det.role
        if self._observing_role == role:
            return
        lst = self._slots.setdefault(role, [])
        pos = (det.position.x, det.position.y, det.position.z)

        idx: int | None = None
        if det.track_id is not None:
            for i, s in enumerate(lst):
                if s.track_id == det.track_id:
                    idx = i
                    break
        if idx is None:
            best_i: int | None = None
            best_d = float("inf")
            for i, s in enumerate(lst):
                if s.class_id != det.class_id:
                    continue
                d = _dist(pos, (s.position.x, s.position.y, s.position.z))
                if d < best_d:
                    best_d = d
                    best_i = i
            if best_i is not None and best_d <= eps_p:
                idx = best_i

        if idx is None:
            lst.append(
                TrackSlot(
                    role=role,
                    class_id=det.class_id,
                    label=det.label,
                    position=det.position,
                    wrist_yaw_deg=det.wrist_yaw_deg,
                    track_id=det.track_id,
                    confidence=det.confidence,
                    last_frame_id=frame.frame_id,
                    last_ts=frame.ts,
                    frame=frame.frame,
                )
            )
            return

        cur = lst[idx]
        old = (cur.position.x, cur.position.y, cur.position.z)
        dpos = _dist(pos, old)
        dyaw = abs(det.wrist_yaw_deg - cur.wrist_yaw_deg)
        if dpos <= eps_p and dyaw <= eps_y:
            lst[idx] = TrackSlot(
                role=role,
                class_id=det.class_id,
                label=det.label,
                position=det.position,
                wrist_yaw_deg=det.wrist_yaw_deg,
                track_id=det.track_id if det.track_id is not None else cur.track_id,
                confidence=det.confidence if det.confidence is not None else cur.confidence,
                last_frame_id=frame.frame_id,
                last_ts=frame.ts,
                frame=frame.frame,
            )
        else:
            lst.append(
                TrackSlot(
                    role=role,
                    class_id=det.class_id,
                    label=det.label,
                    position=det.position,
                    wrist_yaw_deg=det.wrist_yaw_deg,
                    track_id=det.track_id,
                    confidence=det.confidence,
                    last_frame_id=frame.frame_id,
                    last_ts=frame.ts,
                    frame=frame.frame,
                )
            )
