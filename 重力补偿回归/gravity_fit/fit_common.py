"""
重力拟合共用：读 jsonl、特征矩阵、与 motor_config.h 一致的 deg100→rad。
约定：q 为相对 HOME 的关节角，单位弧度（与 Motor_States[i].pos - WORLD_HOME_ABS[i] 一致）。
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


def deg100_to_rad(d100: float) -> float:
    return (d100 * 0.01) * (math.pi / 180.0)


def load_jsonl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Q (N,4) rad, T (N,4) N·m。"""
    q_rows: list[list[float]] = []
    t_rows: list[list[float]] = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            o = json.loads(line)
            avg = o.get("avg") or {}
            q = [
                deg100_to_rad(float(avg["motors.0.position"])),
                deg100_to_rad(float(avg["motors.1.position"])),
                deg100_to_rad(float(avg["motors.2.position"])),
                deg100_to_rad(float(avg["motors.3.position"])),
            ]
            tau = [
                float(avg["joints.1.torque"]),
                float(avg["joints.2.torque"]),
                float(avg["joints.3.torque"]),
                float(avg["joints.4.torque"]),
            ]
            q_rows.append(q)
            t_rows.append(tau)
    if not q_rows:
        raise ValueError("无有效样本")
    return np.asarray(q_rows, dtype=np.float64), np.asarray(t_rows, dtype=np.float64)


def load_jsonl_with_deg100(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Q rad, T N·m, deg100 (N,4) 原始整数度×100（浮点）。"""
    q_rows: list[list[float]] = []
    t_rows: list[list[float]] = []
    d_rows: list[list[float]] = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            o = json.loads(line)
            avg = o.get("avg") or {}
            d = [
                float(avg["motors.0.position"]),
                float(avg["motors.1.position"]),
                float(avg["motors.2.position"]),
                float(avg["motors.3.position"]),
            ]
            q = [deg100_to_rad(x) for x in d]
            tau = [
                float(avg["joints.1.torque"]),
                float(avg["joints.2.torque"]),
                float(avg["joints.3.torque"]),
                float(avg["joints.4.torque"]),
            ]
            d_rows.append(d)
            q_rows.append(q)
            t_rows.append(tau)
    if not q_rows:
        raise ValueError("无有效样本")
    return (
        np.asarray(q_rows, dtype=np.float64),
        np.asarray(t_rows, dtype=np.float64),
        np.asarray(d_rows, dtype=np.float64),
    )


def load_jsonl_meta(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str | None]]:
    """同上，另返回每行 time 字段（无则为 None）。"""
    q_rows: list[list[float]] = []
    t_rows: list[list[float]] = []
    d_rows: list[list[float]] = []
    times: list[str | None] = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            o = json.loads(line)
            avg = o.get("avg") or {}
            times.append(o.get("time"))
            d = [
                float(avg["motors.0.position"]),
                float(avg["motors.1.position"]),
                float(avg["motors.2.position"]),
                float(avg["motors.3.position"]),
            ]
            q = [deg100_to_rad(x) for x in d]
            tau = [
                float(avg["joints.1.torque"]),
                float(avg["joints.2.torque"]),
                float(avg["joints.3.torque"]),
                float(avg["joints.4.torque"]),
            ]
            d_rows.append(d)
            q_rows.append(q)
            t_rows.append(tau)
    return (
        np.asarray(q_rows, dtype=np.float64),
        np.asarray(t_rows, dtype=np.float64),
        np.asarray(d_rows, dtype=np.float64),
        times,
    )


def build_phi_first_order(q: np.ndarray) -> np.ndarray:
    n = q.shape[0]
    s = np.sin(q)
    c = np.cos(q)
    phi = np.zeros((n, 9), dtype=np.float64)
    phi[:, 0] = 1.0
    for j in range(4):
        phi[:, 1 + 2 * j] = s[:, j]
        phi[:, 2 + 2 * j] = c[:, j]
    return phi


def build_phi_second_harmonic(q: np.ndarray) -> np.ndarray:
    n = q.shape[0]
    phi1 = build_phi_first_order(q)
    s2 = np.sin(2.0 * q)
    c2 = np.cos(2.0 * q)
    extra = np.zeros((n, 8), dtype=np.float64)
    for j in range(4):
        extra[:, 2 * j] = s2[:, j]
        extra[:, 2 * j + 1] = c2[:, j]
    return np.hstack([phi1, extra])


def build_phi_sin_cross_pairs(q: np.ndarray) -> np.ndarray:
    """6 列: sin(qi)*sin(qj), i<j。"""
    n = q.shape[0]
    s = np.sin(q)
    pairs = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
    out = np.zeros((n, 6), dtype=np.float64)
    for k, (i, j) in enumerate(pairs):
        out[:, k] = s[:, i] * s[:, j]
    return out


def build_phi_extended(q: np.ndarray, use_cross: bool) -> tuple[np.ndarray, list[str]]:
    phi2 = build_phi_second_harmonic(q)
    names = [
        "bias",
        "s0",
        "c0",
        "s1",
        "c1",
        "s2",
        "c2",
        "s3",
        "c3",
        "sin2q0",
        "cos2q0",
        "sin2q1",
        "cos2q1",
        "sin2q2",
        "cos2q2",
        "sin2q3",
        "cos2q3",
    ]
    if not use_cross:
        return phi2, names
    cr = build_phi_sin_cross_pairs(q)
    cr_names = ["s0s1", "s0s2", "s0s3", "s1s2", "s1s3", "s2s3"]
    return np.hstack([phi2, cr]), names + cr_names


def fit_ols(y: np.ndarray, phi: np.ndarray, ridge: float = 0.0) -> np.ndarray:
    if ridge > 0.0:
        a = phi.T @ phi + ridge * np.eye(phi.shape[1], dtype=np.float64)
        b = phi.T @ y
        return np.linalg.solve(a, b)
    beta, *_ = np.linalg.lstsq(phi, y, rcond=None)
    return beta


def predict(phi: np.ndarray, beta: np.ndarray) -> np.ndarray:
    return phi @ beta


def rmse(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.sqrt(np.mean((a - b) ** 2)))
