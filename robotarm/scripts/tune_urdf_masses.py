#!/usr/bin/env python3
"""
微调 URDF link0–link5 质量参数，最小化 captures.jsonl 上的重力力矩预测 MAE。
使用 differential_evolution 全局搜索，范围 ±30% 当前值。
模型只加载一次，每次评估直接修改 model.inertias[i].mass（无需重写 URDF）。
"""
from __future__ import annotations
import json, re
from pathlib import Path
import numpy as np
from scipy.optimize import differential_evolution

ROOT = Path(__file__).resolve().parent.parent
URDF_PATH = ROOT / "icecream_arm_model" / "ice_cream_v8.SLDASM" / "urdf" / "ice_cream_v8.SLDASM.urdf"
CAPTURES = ROOT / "captures.jsonl"

BASE_MASSES = [0.90937, 0.35782, 0.68284, 0.64216, 0.46259, 0.32088]
LINK_NAMES  = [f"link{i}" for i in range(6)]

# 质心 xyz（link0–link5，来自 URDF <origin xyz=...> in <inertial>）
BASE_COMS = [
    [ 0.0016561,  7.2781e-6,  0.020741],   # link0
    [-0.0003533,  1.3286e-6, -0.015499],   # link1
    [ 0.071245,  -0.0012014, -0.028609],   # link2
    [ 0.053976,   0.028052,  -0.028406],   # link3
    [ 0.017954,  -0.00039937,-0.024807],   # link4
    [ 0.016149,   0.0041715,  0.048405],   # link5
]

# ── 读取验证集 ──────────────────────────────────────────────────────────────
def _load():
    angles, measured = [], []
    with open(CAPTURES, encoding="utf-8") as f:
        for ln in f:
            if not ln.strip():
                continue
            d = json.loads(ln)
            d = d.get("avg", d)
            angles.append([0.0, -float(d["motors.1.angle"]),
                           +float(d["motors.2.angle"]), +float(d["motors.3.angle"]), 0.0])
            measured.append([float(d["joints.2.torque"]),
                             float(d["joints.3.torque"]),
                             float(d["joints.4.torque"])])
    return np.array(angles), np.array(measured)

ANGLES, MEASURED = _load()

# ── 加载模型（一次）──────────────────────────────────────────────────────────
import pinocchio as pin

_PKG_DIRS = [str(ROOT / "icecream_arm_model")]
_MODEL, _, _ = pin.buildModelsFromUrdf(str(URDF_PATH), package_dirs=_PKG_DIRS)
_DATA = _MODEL.createData()

# 预计算每行的 q 向量（不随质量变化）
_QS: list[np.ndarray] = []
for _jdeg in ANGLES:
    _jrad = np.deg2rad(_jdeg / 100.0)
    _q = pin.neutral(_MODEL)
    for _i in range(5):
        _jid = _i + 1
        _j = _MODEL.joints[_jid]
        _th = float(_jrad[_i])
        if _j.nq == 2:
            _q[_j.idx_q]     = np.cos(_th)
            _q[_j.idx_q + 1] = np.sin(_th)
        else:
            _q[_j.idx_q] = _th
    _QS.append(_q)

# inertias 索引：link0→joint[0](universe), link1→joint[1], ..., link5→joint[5]
_BODY_IDS = list(range(6))

def predict_with_params(masses, coms) -> np.ndarray:
    """修改 model.inertias 的 mass 和质心，返回 shape (N,3)"""
    import pinocchio as pin
    for bid, m, c in zip(_BODY_IDS, masses, coms):
        inr = _MODEL.inertias[bid]
        inr.mass = float(m)
        inr.lever = np.array(c, dtype=float)
    preds = []
    for q in _QS:
        pin.computeGeneralizedGravity(_MODEL, _DATA, q)
        tau = _DATA.g
        preds.append([-tau[1], tau[2], tau[3]])
    return np.array(preds)

def mae(x):
    masses = x[:6]
    coms   = np.array(x[6:]).reshape(6, 3)
    preds  = predict_with_params(masses, coms)
    return float(np.mean(np.abs(preds - MEASURED)))

# ── 优化 ────────────────────────────────────────────────────────────────────
def optimize(n_runs=3):
    # 质量 ±10%，质心 ±30mm 全方向
    mass_bounds = [(m * 0.9, m * 1.1) for m in BASE_MASSES]
    com_bounds  = []
    for c in BASE_COMS:
        for v in c:
            com_bounds.append((v - 0.03, v + 0.03))
    bounds = mass_bounds + com_bounds

    x0 = BASE_MASSES + [v for c in BASE_COMS for v in c]
    best_result = None
    for run in range(n_runs):
        print(f"\n--- 优化第 {run+1}/{n_runs} 轮 ---")
        res = differential_evolution(
            mae, bounds,
            seed=run * 42,
            maxiter=300,
            tol=1e-4,
            popsize=12,
            mutation=(0.5, 1.0),
            recombination=0.7,
            workers=1,
            disp=True,
        )
        print(f"  收敛 MAE = {res.fun:.4f} Nm")
        if best_result is None or res.fun < best_result.fun:
            best_result = res
    return best_result

# ── 写回 URDF ───────────────────────────────────────────────────────────────
_MASS_PATTERN = re.compile(r'(<mass\s+value=")([^"]+)(")')
_COM_PATTERN  = re.compile(r'(<origin\s+xyz=")([^"]+)("(?:\s+rpy="[^"]*")?\s*/>(?=\s*</inertial>|[\s\S]*?<mass))', re.DOTALL)

def apply_masses(masses: list[float], coms: list[list[float]] | None = None):
    text = URDF_PATH.read_text(encoding="utf-8")
    # 写回质量
    matches = list(_MASS_PATTERN.finditer(text))
    for i in range(5, -1, -1):
        m = matches[i]
        text = text[:m.start(2)] + f"{masses[i]:.6g}" + text[m.end(2):]
    # 写回质心（inertial 块内的 origin xyz，按出现顺序对应 link0–link5）
    if coms is not None:
        # 找 <inertial> 块内的 <origin xyz=...>
        inertial_origins = list(re.finditer(
            r'(<inertial>.*?<origin\s+xyz=")([^"]+)(")', text, re.DOTALL))
        for i in range(5, -1, -1):
            m = inertial_origins[i]
            xyz_str = f"{coms[i][0]:.7g} {coms[i][1]:.7g} {coms[i][2]:.7g}"
            text = text[:m.start(2)] + xyz_str + text[m.end(2):]
    URDF_PATH.write_text(text, encoding="utf-8")
    print(f"\n已写入 {URDF_PATH}")

# ── 主流程 ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--apply", action="store_true", help="将最优参数写回 URDF")
    args = parser.parse_args()

    x0 = BASE_MASSES + [v for c in BASE_COMS for v in c]
    print(f"基准 MAE : {mae(x0):.4f} Nm")

    best = optimize(args.runs)

    opt_masses = list(best.x[:6])
    opt_coms   = best.x[6:].reshape(6, 3).tolist()

    print("\n=== 最终结果 ===")
    print(f"基准 MAE : {mae(x0):.4f} Nm")
    print(f"优化 MAE : {best.fun:.4f} Nm")
    print("质量变化:")
    for name, orig, opt in zip(LINK_NAMES, BASE_MASSES, opt_masses):
        print(f"  {name}: {orig:.5g} → {opt:.5g}  ({(opt/orig-1)*100:+.1f}%)")
    print("质心变化 (m):")
    for name, orig, opt in zip(LINK_NAMES, BASE_COMS, opt_coms):
        delta = [o - b for o, b in zip(opt, orig)]
        print(f"  {name}: Δxyz=[{delta[0]:+.4f}, {delta[1]:+.4f}, {delta[2]:+.4f}]")

    if args.apply:
        apply_masses(opt_masses, opt_coms)
