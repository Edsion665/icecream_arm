#!/usr/bin/env python3
"""
重力补偿参数辨识 — Step 1.2 / 1.3（与 gravity_compensation_plan.md 一致）

- q1,q2,q3：motors.1/2/3.angle（deg×100）相对 HOME → rad
- τ：joints.2/3/4.torque（肩/肘/腕，Nm）
- 各关节模型含 **常数项 b0**（传感器/摩擦等偏置）；谐波项仍用 cos 或 cos+sin / R·cos(θ−φ)

模式：
  默认：打印「文档 Step 对照矩阵」+ 简要对比行；写入 gravity_params.h（remedy+bias 优先，与 cos+sin+b0 等价时一致）
  --cos-only：仅用 φ=0 的纯 cos 线性最小二乘，并据此写头文件（文档 Step 1.2 基线）
  --robust：remedy 阶段对 least_squares 使用 Huber 损失（减轻离群点，不重新采集数据）
  --no-md-matrix：不打印与 gravity_compensation_plan.md 对齐的实验/验收表
  --metrics-csv：只打印各分支 CSV（branch,joint,mae,r2），含 cos-only / cos+sin / remedy(L2+Huber)

补救（文档 Step 1.3「角度偏置优化」）：在 cos-only 初值上对 A 与 φ 做联合非线性最小二乘。

验收（文档 §Step 1.2 打印块）：每关节 MAE < 0.3 Nm 且 R² > 0.85 记为 ✓。
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
CAPTURES = ROOT / "captures.jsonl"
OUT_H = ROOT / "User" / "GravityComp" / "gravity_params.h"

# gravity_compensation_plan.md 阶段一验收阈值（打印用）
MD_ACCEPT_MAE_NM = 0.3
MD_ACCEPT_R2 = 0.85

HOME_DEG100 = {0: -298, 1: 8059, 2: 13327, 3: 2700}


def _sample_row(s: dict) -> dict:
    """captures.jsonl 每行：扁平 JSON 或旧版 { avg: {...} }。"""
    return s if "avg" not in s else s["avg"]


def _motor_angle(avg: dict, i: int) -> float:
    for k in (f"motors.{i}.angle", f"motors.{i}.position"):
        if k in avg:
            return float(avg[k])
    raise KeyError(f"motors.{i}.angle")


def _joint_torque(avg: dict, j: int) -> float:
    for k in (f"joints.{j}.torque", f"joints.{j}.力矩"):
        if k in avg:
            return float(avg[k])
    raise KeyError(f"joints.{j}.torque")


def motor_pos_to_rad(motor_pos_deg100: float, motor_idx: int) -> float:
    relative_deg100 = float(motor_pos_deg100) - HOME_DEG100[motor_idx]
    return relative_deg100 / 100.0 * (np.pi / 180.0)


def load_dataset(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with open(path, encoding="utf-8") as f:
        samples = [json.loads(line) for line in f if line.strip()]
    q_list: list[list[float]] = []
    tau_list: list[list[float]] = []
    for s in samples:
        avg = _sample_row(s)
        q1 = motor_pos_to_rad(_motor_angle(avg, 1), 1)
        q2 = motor_pos_to_rad(_motor_angle(avg, 2), 2)
        q3 = motor_pos_to_rad(_motor_angle(avg, 3), 3)
        q_list.append([q1, q2, q3])
        tau_list.append(
            [
                _joint_torque(avg, 2),
                _joint_torque(avg, 3),
                _joint_torque(avg, 4),
            ]
        )
    return np.asarray(q_list, dtype=np.float64), np.asarray(tau_list, dtype=np.float64)


def mae_r2(y: np.ndarray, yp: np.ndarray) -> tuple[float, float]:
    mae = float(np.mean(np.abs(y - yp)))
    ss_res = float(np.sum((y - yp) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    return mae, r2


def ab_to_R_phi(a: float, b: float) -> tuple[float, float]:
    r = float(np.hypot(a, b))
    phi = float(np.arctan2(b, a))
    return r, phi


def angles_from_Q(Q: np.ndarray) -> tuple[np.ndarray, ...]:
    q1 = Q[:, 0]
    q2 = Q[:, 1]
    q3 = Q[:, 2]
    s1 = q1 + q2
    s2 = q1 + q2 + q3
    return q1, q2, q3, s1, s2


def _ones(n: int) -> np.ndarray:
    return np.ones((n,), dtype=np.float64)


def fit_cos_sin(Q: np.ndarray, TAU: np.ndarray, *, bias: bool) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    q1, _, _, s1, s2 = angles_from_Q(Q)
    n = len(q1)
    c1 = [np.cos(q1), np.sin(q1), np.cos(s1), np.sin(s1), np.cos(s2), np.sin(s2)]
    c2 = [np.cos(s1), np.sin(s1), np.cos(s2), np.sin(s2)]
    c3 = [np.cos(s2), np.sin(s2)]
    if bias:
        W1 = np.column_stack([_ones(n)] + c1)
        W2 = np.column_stack([_ones(n)] + c2)
        W3 = np.column_stack([_ones(n)] + c3)
    else:
        W1 = np.column_stack(c1)
        W2 = np.column_stack(c2)
        W3 = np.column_stack(c3)
    t1, *_ = np.linalg.lstsq(W1, TAU[:, 0], rcond=None)
    t2, *_ = np.linalg.lstsq(W2, TAU[:, 1], rcond=None)
    t3, *_ = np.linalg.lstsq(W3, TAU[:, 2], rcond=None)
    p1 = W1 @ t1
    p2 = W2 @ t2
    p3 = W3 @ t3
    pred = np.column_stack([p1, p2, p3])
    return pred, t1, t2, t3


def fit_cos_only_linear(
    Q: np.ndarray, TAU: np.ndarray, *, bias: bool
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """文档 Step 1.2：φ=0，仅 cos 基；bias=True 时每关节加截距。"""
    q1, _, _, s1, s2 = angles_from_Q(Q)
    n = len(q1)
    if bias:
        W1 = np.column_stack([_ones(n), np.cos(q1), np.cos(s1), np.cos(s2)])
        W2 = np.column_stack([_ones(n), np.cos(s1), np.cos(s2)])
        W3 = np.column_stack([_ones(n), np.cos(s2)])
    else:
        W1 = np.column_stack([np.cos(q1), np.cos(s1), np.cos(s2)])
        W2 = np.column_stack([np.cos(s1), np.cos(s2)])
        W3 = np.cos(s2).reshape(-1, 1)
    a1, *_ = np.linalg.lstsq(W1, TAU[:, 0], rcond=None)
    a2, *_ = np.linalg.lstsq(W2, TAU[:, 1], rcond=None)
    a3, *_ = np.linalg.lstsq(W3, TAU[:, 2], rcond=None)
    p1 = W1 @ a1
    p2 = W2 @ a2
    p3 = W3 @ a3
    pred = np.column_stack([p1, p2, p3])
    return pred, a1, a2, a3


def fit_nonlinear_remedy(
    Q: np.ndarray,
    TAU: np.ndarray,
    cos_only_A: tuple[np.ndarray, np.ndarray, np.ndarray],
    robust: bool,
    *,
    bias: bool,
) -> tuple[np.ndarray, tuple[float, ...], tuple[float, ...], tuple[float, ...]] | None:
    """Step 1.3：联合辨识 A 与 φ；bias=True 时每关节增加常数项 b0。初值来自 cos-only。"""
    try:
        from scipy.optimize import least_squares
    except ImportError:
        return None

    q1, _, _, s1, s2 = angles_from_Q(Q)
    a1_0, a2_0, a3_0 = cos_only_A

    loss = "huber" if robust else "linear"
    f_scale = 1.5 if robust else 1.0

    if bias:
        ia1, ia2, ia3 = 1, 2, 3
        ib1, ib2 = 1, 2
        ic1 = 1
    else:
        ia1, ia2, ia3 = 0, 1, 2
        ib1, ib2 = 0, 1
        ic1 = 0

    def fit_j1() -> tuple[np.ndarray, tuple[float, ...]]:
        A10 = float(a1_0[ia1])
        A20 = float(a1_0[ia2])
        A30 = float(a1_0[ia3])
        b0 = float(np.mean(TAU[:, 0])) if not bias else float(a1_0[0])
        if bias:
            x0 = np.array([b0, A10, A20, A30, 0.0, 0.0, 0.0], dtype=np.float64)

            def fun(x: np.ndarray) -> np.ndarray:
                b, A1, A2, A3, f1, f2, f3 = x
                pr = b + A1 * np.cos(q1 + f1) + A2 * np.cos(s1 + f2) + A3 * np.cos(s2 + f3)
                return TAU[:, 0] - pr

            lb = np.array([-20.0, -25.0, -25.0, -25.0, -np.pi, -np.pi, -np.pi])
            ub = np.array([20.0, 25.0, 25.0, 25.0, np.pi, np.pi, np.pi])
        else:
            x0 = np.array([A10, A20, A30, 0.0, 0.0, 0.0], dtype=np.float64)

            def fun(x: np.ndarray) -> np.ndarray:
                A1, A2, A3, f1, f2, f3 = x
                pr = A1 * np.cos(q1 + f1) + A2 * np.cos(s1 + f2) + A3 * np.cos(s2 + f3)
                return TAU[:, 0] - pr

            lb = np.array([-25.0, -25.0, -25.0, -np.pi, -np.pi, -np.pi])
            ub = np.array([25.0, 25.0, 25.0, np.pi, np.pi, np.pi])

        res = least_squares(
            fun,
            x0,
            bounds=(lb, ub),
            loss=loss,
            f_scale=f_scale,
            max_nfev=2000,
        )
        x = res.x
        if bias:
            pr = (
                x[0]
                + x[1] * np.cos(q1 + x[4])
                + x[2] * np.cos(s1 + x[5])
                + x[3] * np.cos(s2 + x[6])
            )
            return pr, tuple(float(v) for v in x)
        pr = (
            x[0] * np.cos(q1 + x[3])
            + x[1] * np.cos(s1 + x[4])
            + x[2] * np.cos(s2 + x[5])
        )
        return pr, (float(x[0]), float(x[1]), float(x[2]), float(x[3]), float(x[4]), float(x[5]))

    def fit_j2() -> tuple[np.ndarray, tuple[float, ...]]:
        B10 = float(a2_0[ib1])
        B20 = float(a2_0[ib2])
        b0 = float(np.mean(TAU[:, 1])) if not bias else float(a2_0[0])
        if bias:
            x0 = np.array([b0, B10, B20, 0.0, 0.0], dtype=np.float64)

            def fun(x: np.ndarray) -> np.ndarray:
                b, B1, B2, p1, p2 = x
                pr = b + B1 * np.cos(s1 + p1) + B2 * np.cos(s2 + p2)
                return TAU[:, 1] - pr

            lb = np.array([-20.0, -25.0, -25.0, -np.pi, -np.pi])
            ub = np.array([20.0, 25.0, 25.0, np.pi, np.pi])
        else:
            x0 = np.array([B10, B20, 0.0, 0.0], dtype=np.float64)

            def fun(x: np.ndarray) -> np.ndarray:
                B1, B2, p1, p2 = x
                pr = B1 * np.cos(s1 + p1) + B2 * np.cos(s2 + p2)
                return TAU[:, 1] - pr

            lb = np.array([-25.0, -25.0, -np.pi, -np.pi])
            ub = np.array([25.0, 25.0, np.pi, np.pi])

        res = least_squares(fun, x0, bounds=(lb, ub), loss=loss, f_scale=f_scale, max_nfev=2000)
        x = res.x
        if bias:
            pr = x[0] + x[1] * np.cos(s1 + x[3]) + x[2] * np.cos(s2 + x[4])
            return pr, tuple(float(v) for v in x)
        pr = x[0] * np.cos(s1 + x[2]) + x[1] * np.cos(s2 + x[3])
        return pr, (float(x[0]), float(x[1]), float(x[2]), float(x[3]))

    def fit_j3() -> tuple[np.ndarray, tuple[float, ...]]:
        C0 = float(a3_0[ic1])
        b0 = float(np.mean(TAU[:, 2])) if not bias else float(a3_0[0])
        if bias:
            x0 = np.array([b0, C0, 0.0], dtype=np.float64)

            def fun(x: np.ndarray) -> np.ndarray:
                b, C1, z = x
                pr = b + C1 * np.cos(s2 + z)
                return TAU[:, 2] - pr

            lb = np.array([-20.0, -25.0, -np.pi])
            ub = np.array([20.0, 25.0, np.pi])
        else:
            x0 = np.array([C0, 0.0], dtype=np.float64)

            def fun(x: np.ndarray) -> np.ndarray:
                C1, z = x
                pr = C1 * np.cos(s2 + z)
                return TAU[:, 2] - pr

            lb = np.array([-25.0, -np.pi])
            ub = np.array([25.0, np.pi])

        res = least_squares(fun, x0, bounds=(lb, ub), loss=loss, f_scale=f_scale, max_nfev=2000)
        x = res.x
        if bias:
            pr = x[0] + x[1] * np.cos(s2 + x[2])
            return pr, tuple(float(v) for v in x)
        pr = x[0] * np.cos(s2 + x[1])
        return pr, (float(x[0]), float(x[1]))

    p1, params1 = fit_j1()
    p2, params2 = fit_j2()
    p3, params3 = fit_j3()
    pred = np.column_stack([p1, p2, p3])
    return pred, params1, params2, params3


def metrics_row(name: str, TAU: np.ndarray, pred: np.ndarray) -> str:
    m1, r1 = mae_r2(TAU[:, 0], pred[:, 0])
    m2, r2 = mae_r2(TAU[:, 1], pred[:, 1])
    m3, r3 = mae_r2(TAU[:, 2], pred[:, 2])
    return (
        f"{name:16s}  肩 MAE={m1:.4f} R²={r1:.4f}  |  肘 MAE={m2:.4f} R²={r2:.4f}  |  腕 MAE={m3:.4f} R²={r3:.4f}"
    )


def joint_pass(mae: float, r2: float) -> bool:
    return mae < MD_ACCEPT_MAE_NM and r2 > MD_ACCEPT_R2


def step11_summary_lines(path: Path) -> list[str]:
    """与文档 Step 1.1 一致：基座关节力矩通道启发式检查。"""
    with open(path, encoding="utf-8") as f:
        samples = [json.loads(line) for line in f if line.strip()]
    j1 = np.array([_joint_torque(_sample_row(s), 1) for s in samples], dtype=np.float64)
    a0 = np.array([_motor_angle(_sample_row(s), 0) for s in samples], dtype=np.float64)
    rho = float(np.corrcoef(a0, j1)[0, 1])
    std = float(np.std(j1))
    mu = float(np.mean(j1))
    ok_std = std < 0.1
    lines = [
        f"  joints.1.torque ↔ motors.0.angle  Pearson ρ = {rho:.4f}（竖直轴：力矩则通常|ρ|不大）",
        f"  joints.1.torque: mean = {mu:.4f} Nm, std = {std:.4f} Nm",
        f"  文档启发式: std < 0.1 → 视为准静态力矩通道  →  {'✓ 通过' if ok_std else '⚠ 未满足'}",
    ]
    return lines


def print_md_experiment_matrix(
    TAU: np.ndarray,
    captures_path: Path,
    rows: list[tuple[str, str, np.ndarray | None]],
) -> None:
    """按文档步骤编号打印对照表；验收标准见 MD_ACCEPT_*。"""
    print("\n" + "=" * 72)
    print("与 gravity_compensation_plan.md 对齐的实验验证（同一批数据）")
    print("验收（每关节）: MAE < {:.1f} Nm 且 R² > {:.2f}".format(MD_ACCEPT_MAE_NM, MD_ACCEPT_R2))
    print("=" * 72)
    print("\n【Step 1.1】数据格式（非拟合） 文件: {}".format(captures_path))
    for line in step11_summary_lines(captures_path):
        print(line)
    print("\n【Step 1.2 / 1.3】静态重力回归 — 拟合对照")
    print(
        f"{'ID':<8} {'说明':<34} {'肩':^14} {'肘':^14} {'腕':^14} {'三角':^5}"
    )
    print("-" * 96)
    for eid, desc, pred in rows:
        if pred is None:
            print(f"{eid:<8} {desc:<34} {'(无 scipy / 跳过)':<46}")
            continue
        flags = []
        cells = []
        for j in range(3):
            m, r = mae_r2(TAU[:, j], pred[:, j])
            ok = joint_pass(m, r)
            flags.append(ok)
            cells.append(f"{m:.2f}/{r:.2f}/{'✓' if ok else '✗'}")
        tri = "✓" if all(flags) else "✗"
        c0, c1, c2 = cells
        print(f"{eid:<8} {desc:<34} {c0:^14} {c1:^14} {c2:^14} {tri:^5}")
    print(
        "\n说明: 表中为 MAE(Nm)/R²/单关节验收；「三角」= 肩肘腕三项均达标。"
        "\n★BASE = 偏置 + cos+sin 线性最小二乘（与文档「cos+sin 等价相位」的推荐基线）。"
        "\n1.3 remedy(+b0) 与 ★BASE 数值常一致（同一解析函数族 + 截距）。"
    )
    print("=" * 72 + "\n")


JOINT_LABELS = ("肩_J1", "肘_J2", "腕_J3")


def emit_metrics_csv(Q: np.ndarray, TAU: np.ndarray) -> None:
    """输出所有拟合分支的 MAE / R²（CSV：branch,joint,mae,r2）。"""
    pred_co_nb, a1_nb, a2_nb, a3_nb = fit_cos_only_linear(Q, TAU, bias=False)
    pred_co_b, a1_b, a2_b, a3_b = fit_cos_only_linear(Q, TAU, bias=True)
    pred_cs_nb, _, _, _ = fit_cos_sin(Q, TAU, bias=False)
    pred_cs_b, _, _, _ = fit_cos_sin(Q, TAU, bias=True)
    branches: list[tuple[str, np.ndarray]] = [
        ("cos_only_phi0_no_bias", pred_co_nb),
        ("cos_only_phi0_plus_bias", pred_co_b),
        ("cos_sin_no_bias", pred_cs_nb),
        ("cos_sin_plus_bias_BASE", pred_cs_b),
    ]
    for robust in (False, True):
        tag = "Huber" if robust else "L2"
        rb = fit_nonlinear_remedy(Q, TAU, (a1_b, a2_b, a3_b), robust, bias=True)
        rnb = fit_nonlinear_remedy(Q, TAU, (a1_nb, a2_nb, a3_nb), robust, bias=False)
        if rb is not None:
            branches.append((f"remedy_plus_bias_{tag}", rb[0]))
        if rnb is not None:
            branches.append((f"remedy_no_bias_{tag}", rnb[0]))

    print("branch,joint,mae,r2")
    for bname, pred in branches:
        for j in range(3):
            m, r = mae_r2(TAU[:, j], pred[:, j])
            print(f"{bname},{JOINT_LABELS[j]},{m:.6f},{r:.6f}")


def render_header(
    mode: str,
    mae1: float,
    mae2: float,
    mae3: float,
    r2_1: float,
    r2_2: float,
    r2_3: float,
    A1: float,
    A2: float,
    A3: float,
    PHI1: float,
    PHI2: float,
    PHI3: float,
    B1: float,
    B2: float,
    PSI1: float,
    PSI2: float,
    C1: float,
    ZETA1: float,
    *,
    bias_j1: float = 0.0,
    bias_j2: float = 0.0,
    bias_j3: float = 0.0,
    extra_comment: str = "",
) -> str:
    date_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return f"""/* gravity_params.h — AUTO GENERATED by scripts/gravity_identification.py
 * Date: {date_str}
 * Mode: {mode}
 * Fitting MAE (Nm): J1={mae1:.4f}  J2={mae2:.4f}  J3={mae3:.4f}
 * R²: J1={r2_1:.4f}  J2={r2_2:.4f}  J3={r2_3:.4f}
{extra_comment} *
 * tau_g1 = GC_BIAS_J1 + A1*cos(q1+PHI1) + A2*cos(q1+q2+PHI2) + A3*cos(q1+q2+q3+PHI3)
 * tau_g2 = GC_BIAS_J2 + B1*cos(q1+q2+PSI1) + B2*cos(q1+q2+q3+PSI2)
 * tau_g3 = GC_BIAS_J3 + C1*cos(q1+q2+q3+ZETA1)
 */
#ifndef GRAVITY_PARAMS_H
#define GRAVITY_PARAMS_H

#define GC_BIAS_J1 {bias_j1:.6f}f
#define GC_BIAS_J2 {bias_j2:.6f}f
#define GC_BIAS_J3 {bias_j3:.6f}f

#define GC_A1    {A1:.6f}f
#define GC_A2    {A2:.6f}f
#define GC_A3    {A3:.6f}f
#define GC_PHI1  {PHI1:.6f}f
#define GC_PHI2  {PHI2:.6f}f
#define GC_PHI3  {PHI3:.6f}f

#define GC_B1    {B1:.6f}f
#define GC_B2    {B2:.6f}f
#define GC_PSI1  {PSI1:.6f}f
#define GC_PSI2  {PSI2:.6f}f

#define GC_C1    {C1:.6f}f
#define GC_ZETA1 {ZETA1:.6f}f

#define GC_SIGN_J0   0
#define GC_SIGN_J1  +1
#define GC_SIGN_J2  +1
#define GC_SIGN_J3  +1

#define GC_MAX_TFF   10.0f

#endif /* GRAVITY_PARAMS_H */
"""


def main() -> None:
    ap = argparse.ArgumentParser(description="重力补偿参数辨识（captures.jsonl）")
    ap.add_argument(
        "--cos-only",
        action="store_true",
        help="仅用 φ=0 的纯 cos 线性拟合写 gravity_params.h（Step 1.2 基线）",
    )
    ap.add_argument(
        "--robust",
        action="store_true",
        help="remedy 非线性阶段使用 Huber 损失（抑制离群点）",
    )
    ap.add_argument(
        "--no-write",
        action="store_true",
        help="不写头文件，仅打印对比",
    )
    ap.add_argument(
        "--no-bias",
        action="store_true",
        help="拟合不含常数项（复现旧版 R² 行为，用于对比）",
    )
    ap.add_argument(
        "--no-md-matrix",
        action="store_true",
        help="不打印与 gravity_compensation_plan.md 对齐的 Step 实验/验收表",
    )
    ap.add_argument(
        "--metrics-csv",
        action="store_true",
        help="仅输出所有拟合分支的 MAE/R²（CSV），不写头文件、不打印其它日志",
    )
    args = ap.parse_args()

    if args.metrics_csv:
        Q, TAU = load_dataset(CAPTURES)
        emit_metrics_csv(Q, TAU)
        return

    use_bias = not args.no_bias

    Q, TAU = load_dataset(CAPTURES)
    n = Q.shape[0]
    print(f"样本数: {n}  来源: {CAPTURES}")
    print(f"常数项 b0: {'开启（默认）' if use_bias else '关闭（--no-bias）'}\n")

    # 文档对照：同时算「有/无截距」两套，矩阵与 remedy 初值互不混淆
    pred_co_nb, a1_nb, a2_nb, a3_nb = fit_cos_only_linear(Q, TAU, bias=False)
    pred_co_b, a1_b, a2_b, a3_b = fit_cos_only_linear(Q, TAU, bias=True)
    pred_cs_nb, _, _, _ = fit_cos_sin(Q, TAU, bias=False)
    pred_cs_b, _, _, _ = fit_cos_sin(Q, TAU, bias=True)

    if use_bias:
        pred_co, a1_co, a2_co, a3_co = pred_co_b, a1_b, a2_b, a3_b
        pred_cs = pred_cs_b
    else:
        pred_co, a1_co, a2_co, a3_co = pred_co_nb, a1_nb, a2_nb, a3_nb
        pred_cs = pred_cs_nb

    remedy_b = fit_nonlinear_remedy(Q, TAU, (a1_b, a2_b, a3_b), args.robust, bias=True)
    remedy_nb = fit_nonlinear_remedy(Q, TAU, (a1_nb, a2_nb, a3_nb), args.robust, bias=False)
    remedy = remedy_b if use_bias else remedy_nb

    if not args.no_md_matrix:
        pred_rm_b = remedy_b[0] if remedy_b is not None else None
        pred_rm_nb = remedy_nb[0] if remedy_nb is not None else None
        md_rows: list[tuple[str, str, np.ndarray | None]] = [
            ("1.2a", "MD Step1.2: cos, φ=0, 无截距", pred_co_nb),
            ("1.2b", "MD Step1.2: cos, φ=0, +截距 b0", pred_co_b),
            ("—", "cos+sin 线性, 无截距", pred_cs_nb),
            ("★BASE", "偏置+cos+sin（推荐基线）", pred_cs_b),
            ("1.3a", "Step1.3 remedy: 非线性φ + b0", pred_rm_b),
            ("1.3b", "Step1.3 remedy: 非线性φ, 无截距", pred_rm_nb),
        ]
        print_md_experiment_matrix(TAU, CAPTURES, md_rows)

    print("=== 对比实验（同一批 captures，不重新采集）===\n")
    if use_bias:
        print(metrics_row("cos+sin(无b0)", TAU, pred_cs_nb))
        print(metrics_row("cos+sin(+b0)★", TAU, pred_cs_b))
    else:
        print(metrics_row("cos+sin(线性,无b0)", TAU, pred_cs_nb))
    label_co = "cos-only+b0" if use_bias else "cos-only(φ=0)"
    print(metrics_row(label_co, TAU, pred_co))
    if remedy is not None:
        pred_rm, p1, p2, p3 = remedy
        tag = "remedy+b0+Huber" if (args.robust and use_bias) else (
            "remedy+b0(非线性φ)" if use_bias else ("remedy+Huber" if args.robust else "remedy(非线性φ)")
        )
        print(metrics_row(tag, TAU, pred_rm))
        print(
            "\n说明：remedy = 初值来自 cos-only 的 A，联合优化 A 与 φ；"
            + (" 使用 Huber。" if args.robust else "")
        )
        if use_bias and not args.robust:
            d = np.max(np.abs(pred_rm - pred_cs))
            if d < 1e-5:
                print(
                    "提示：remedy(+b0, L2) 与 cos+sin(+b0) 数值接近（同一函数族 + 截距）。"
                )
    else:
        print("remedy  — 跳过（未安装 scipy，请: pip install scipy）")
        pred_rm = None

    # 选择写入内容（默认：含 b0 的 remedy；--cos-only：含 b0 的 cos-only；--no-bias：旧 remedy/cos+sin）
    if args.cos_only:
        if not use_bias:
            pred_w, a1, a2, a3 = pred_co, a1_co, a2_co, a3_co
            m1, r1 = mae_r2(TAU[:, 0], pred_w[:, 0])
            m2, r2 = mae_r2(TAU[:, 1], pred_w[:, 1])
            m3, r3 = mae_r2(TAU[:, 2], pred_w[:, 2])
            mode = "cos-only linear (phi=0, no bias)"
            A1, A2, A3 = float(a1[0]), float(a1[1]), float(a1[2])
            B1, B2 = float(a2[0]), float(a2[1])
            C1 = float(a3[0])
            bj1 = bj2 = bj3 = 0.0
        else:
            m1, r1 = mae_r2(TAU[:, 0], pred_co[:, 0])
            m2, r2 = mae_r2(TAU[:, 1], pred_co[:, 1])
            m3, r3 = mae_r2(TAU[:, 2], pred_co[:, 2])
            mode = "cos-only linear (phi=0) + bias"
            bj1, A1, A2, A3 = float(a1_co[0]), float(a1_co[1]), float(a1_co[2]), float(a1_co[3])
            bj2, B1, B2 = float(a2_co[0]), float(a2_co[1]), float(a2_co[2])
            bj3, C1 = float(a3_co[0]), float(a3_co[1])
        PHI1 = PHI2 = PHI3 = 0.0
        PSI1 = PSI2 = 0.0
        ZETA1 = 0.0
        hdr = render_header(
            mode, m1, m2, m3, r1, r2, r3,
            A1, A2, A3, PHI1, PHI2, PHI3, B1, B2, PSI1, PSI2, C1, ZETA1,
            bias_j1=bj1, bias_j2=bj2, bias_j3=bj3,
            extra_comment=" * NOTE: PHI/PSI/ZETA forced 0 (cos-only export)",
        )
    elif remedy is not None and use_bias:
        pred_rm, p1, p2, p3 = remedy
        m1, r1 = mae_r2(TAU[:, 0], pred_rm[:, 0])
        m2, r2 = mae_r2(TAU[:, 1], pred_rm[:, 1])
        m3, r3 = mae_r2(TAU[:, 2], pred_rm[:, 2])
        mode = "remedy+bias Huber" if args.robust else "remedy+bias nonlinear phases"
        bj1, A1, A2, A3, PHI1, PHI2, PHI3 = p1
        bj2, B1, B2, PSI1, PSI2 = p2
        bj3, C1, ZETA1 = p3
        hdr = render_header(
            mode, m1, m2, m3, r1, r2, r3,
            A1, A2, A3, PHI1, PHI2, PHI3, B1, B2, PSI1, PSI2, C1, ZETA1,
            bias_j1=bj1, bias_j2=bj2, bias_j3=bj3,
        )
    elif remedy is not None and not use_bias:
        pred_rm, p1, p2, p3 = remedy
        m1, r1 = mae_r2(TAU[:, 0], pred_rm[:, 0])
        m2, r2 = mae_r2(TAU[:, 1], pred_rm[:, 1])
        m3, r3 = mae_r2(TAU[:, 2], pred_rm[:, 2])
        mode = "remedy Huber" if args.robust else "remedy nonlinear phases"
        A1, A2, A3, PHI1, PHI2, PHI3 = p1
        B1, B2, PSI1, PSI2 = p2
        C1, ZETA1 = p3
        hdr = render_header(
            mode, m1, m2, m3, r1, r2, r3,
            A1, A2, A3, PHI1, PHI2, PHI3, B1, B2, PSI1, PSI2, C1, ZETA1,
            bias_j1=0.0, bias_j2=0.0, bias_j3=0.0,
        )
    else:
        pred_cs_u, t1, t2, t3 = fit_cos_sin(Q, TAU, bias=use_bias)
        if use_bias:
            bj1 = float(t1[0])
            a1_c, a1_s, a2_c, a2_s, a3_c, a3_s = t1[1], t1[2], t1[3], t1[4], t1[5], t1[6]
            bj2 = float(t2[0])
            b1_c, b1_s, b2_c, b2_s = t2[1], t2[2], t2[3], t2[4]
            bj3 = float(t3[0])
            c1_c, c1_s = t3[1], t3[2]
        else:
            bj1 = bj2 = bj3 = 0.0
            a1_c, a1_s, a2_c, a2_s, a3_c, a3_s = t1[0], t1[1], t1[2], t1[3], t1[4], t1[5]
            b1_c, b1_s, b2_c, b2_s = t2[0], t2[1], t2[2], t2[3]
            c1_c, c1_s = t3[0], t3[1]
        A1, PHI1 = ab_to_R_phi(a1_c, a1_s)
        A2, PHI2 = ab_to_R_phi(a2_c, a2_s)
        A3, PHI3 = ab_to_R_phi(a3_c, a3_s)
        B1, PSI1 = ab_to_R_phi(b1_c, b1_s)
        B2, PSI2 = ab_to_R_phi(b2_c, b2_s)
        C1, ZETA1 = ab_to_R_phi(c1_c, c1_s)
        m1, r1 = mae_r2(TAU[:, 0], pred_cs_u[:, 0])
        m2, r2 = mae_r2(TAU[:, 1], pred_cs_u[:, 1])
        m3, r3 = mae_r2(TAU[:, 2], pred_cs_u[:, 2])
        hdr = render_header(
            "cos+sin linear (fallback, no scipy)",
            m1, m2, m3, r1, r2, r3,
            A1, A2, A3, PHI1, PHI2, PHI3, B1, B2, PSI1, PSI2, C1, ZETA1,
            bias_j1=bj1, bias_j2=bj2, bias_j3=bj3,
        )

    if not args.no_write:
        OUT_H.parent.mkdir(parents=True, exist_ok=True)
        OUT_H.write_text(hdr, encoding="utf-8")
        print(f"\n已写入: {OUT_H}")
    else:
        print("\n(--no-write，未写头文件)")


if __name__ == "__main__":
    main()
