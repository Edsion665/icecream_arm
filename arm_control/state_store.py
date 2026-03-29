"""线程安全的全局状态管理。所有模块通过 StateStore 共享状态。"""

from __future__ import annotations

from dataclasses import dataclass, field
from threading import Lock
from typing import Dict, List, Optional, Tuple, TypedDict


class MotorStateDict(TypedDict, total=False):
    id: int
    position: float
    velocity: float
    torque: float


class ServoStateDict(TypedDict, total=False):
    id: int
    angle: float


class JointStateDict(TypedDict, total=False):
    id: int
    angle: float


@dataclass
class SystemState:
    motors: Dict[int, MotorStateDict] = field(default_factory=dict)
    servos: Dict[int, ServoStateDict] = field(default_factory=dict)
    joints: Dict[int, JointStateDict] = field(default_factory=dict)
    crc_error_count: int = 0


class StateStore:
    """集中管理系统状态，内部使用线程锁保证安全。"""

    def __init__(self) -> None:
        self._state = SystemState()
        self._lock = Lock()
        self._fb_arm_rad: Optional[Tuple[float, float, float, float]] = None

    def snapshot(self) -> SystemState:
        """返回状态快照（浅拷贝字典，防止外部直接修改内部引用）。"""
        with self._lock:
            return SystemState(
                motors=dict(self._state.motors),
                servos=dict(self._state.servos),
                joints=dict(self._state.joints),
                crc_error_count=self._state.crc_error_count,
            )

    def update_motor(self, motor_id: int, **fields: float) -> None:
        with self._lock:
            current = self._state.motors.get(motor_id, {"id": motor_id})
            current.update(fields)
            self._state.motors[motor_id] = current  # type: ignore[assignment]

    def update_servo(self, servo_id: int, **fields: float) -> None:
        with self._lock:
            current = self._state.servos.get(servo_id, {"id": servo_id})
            current.update(fields)
            self._state.servos[servo_id] = current  # type: ignore[assignment]

    def inc_crc_error(self) -> None:
        with self._lock:
            self._state.crc_error_count += 1

    def update_joint(self, joint_id: int, **fields: float) -> None:
        with self._lock:
            current = self._state.joints.get(joint_id, {"id": joint_id})
            current.update(fields)
            self._state.joints[joint_id] = current  # type: ignore[assignment]

    def set_fb_arm_rad(self, rad: Tuple[float, float, float, float]) -> None:
        """STM32 `FB f f f f` 行解析到的四轴弧度（线程内写入）。"""
        with self._lock:
            self._fb_arm_rad = rad

    def get_fb_arm_rad(self) -> Optional[Tuple[float, float, float, float]]:
        with self._lock:
            return self._fb_arm_rad

    def to_payload(self) -> Dict[str, List[Dict[str, float]]]:
        """将内部结构转换为可 JSON 序列化的字典，用于 WebSocket 推送。"""
        snap = self.snapshot()
        return {
            "motors": list(snap.motors.values()),
            "servos": list(snap.servos.values()),
            "joints": list(snap.joints.values()),
            "crc_error_count": snap.crc_error_count,
        }

