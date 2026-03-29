"""
无 URDF：用 captures 中「相对 HOME 的度×100」与 FB 力矩 (N·m) 做耦合 sin/cos 基的最小二乘，
与 STM32 约定一致：q_i 为弧度（与 RPI_DEG100_TO_RAD 相同），tau 与 MIT 前馈同单位。

运行（在 gravity_fit 目录）:
  python fit_gravity_ls.py
输出: out/coefficients.json, out/metrics.json, out/*.png

输入数据: jsonl 中 motors.*.position 为度×100（非弧度）；脚本内部会转为 rad。
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as e:
    print("需要 matplotlib：pip install -r requirements.txt", file=sys.stderr)
    raise SystemExit(1) from e

from fit_common import (
    build_phi_first_order,
    build_phi_second_harmonic,
    fit_ols,
    load_jsonl,
    predict,
    rmse,
)


def main() -> None:
    here = Path(__file__).resolve().parent
    repo_captures = here.parent / "captures_shifted_to_new_home.jsonl"
    default_data = repo_captures if repo_captures.is_file() else here / "captures_shifted_to_new_home.jsonl"
    data_path = Path(os.environ.get("GRAVITY_FIT_DATA", str(default_data)))

    out_dir = here / "out"
    out_dir.mkdir(parents=True, exist_ok=True)

    Q, T = load_jsonl(data_path)
    n = Q.shape[0]
    print(f"样本数: {n}, 数据文件: {data_path}")

    n_val = max(1, n // 5)
    n_tr = n - n_val
    Q_tr, Q_val = Q[:n_tr], Q[n_tr:]
    T_tr, T_val = T[:n_tr], T[n_tr:]

    phi_tr_1 = build_phi_first_order(Q_tr)
    phi_val_1 = build_phi_first_order(Q_val)
    phi_all_1 = build_phi_first_order(Q)

    phi_tr_2 = build_phi_second_harmonic(Q_tr)
    phi_val_2 = build_phi_second_harmonic(Q_val)
    phi_all_2 = build_phi_second_harmonic(Q)

    feature_names_1 = ["bias", "s0", "c0", "s1", "c1", "s2", "c2", "s3", "c3"]
    feature_names_2 = feature_names_1 + [
        "sin2q0",
        "cos2q0",
        "sin2q1",
        "cos2q1",
        "sin2q2",
        "cos2q2",
        "sin2q3",
        "cos2q3",
    ]

    ridge1 = 1e-6
    ridge2 = 1e-4

    betas_1: list[np.ndarray] = []
    betas_2: list[np.ndarray] = []
    for k in range(4):
        betas_1.append(fit_ols(T_tr[:, k], phi_tr_1, ridge=ridge1))
        betas_2.append(fit_ols(T_tr[:, k], phi_tr_2, ridge=ridge2))

    metrics: dict = {
        "n_total": n,
        "n_train": n_tr,
        "n_val": n_val,
        "data_file": str(data_path),
        "models": {},
    }

    pred_val_1 = np.column_stack([predict(phi_val_1, betas_1[k]) for k in range(4)])
    pred_all_1 = np.column_stack([predict(phi_all_1, betas_1[k]) for k in range(4)])
    pred_val_2 = np.column_stack([predict(phi_val_2, betas_2[k]) for k in range(4)])
    pred_all_2 = np.column_stack([predict(phi_all_2, betas_2[k]) for k in range(4)])

    m1_tr = rmse(T_tr, np.column_stack([predict(phi_tr_1, betas_1[k]) for k in range(4)]))
    m1_val = rmse(T_val, pred_val_1)
    m1_all = rmse(T, pred_all_1)

    m2_tr = rmse(T_tr, np.column_stack([predict(phi_tr_2, betas_2[k]) for k in range(4)]))
    m2_val = rmse(T_val, pred_val_2)
    m2_all = rmse(T, pred_all_2)

    metrics["models"]["first_order_9feat"] = {
        "rmse_train_all_axes": m1_tr,
        "rmse_val_all_axes": m1_val,
        "rmse_fit_all_samples": m1_all,
        "ridge": ridge1,
    }
    metrics["models"]["second_harmonic_17feat"] = {
        "rmse_train_all_axes": m2_tr,
        "rmse_val_all_axes": m2_val,
        "rmse_fit_all_samples": m2_all,
        "ridge": ridge2,
    }

    use_second = m2_val <= m1_val
    tag = "second_harmonic" if use_second else "first_order"
    betas_use = betas_2 if use_second else betas_1
    pred_all = pred_all_2 if use_second else pred_all_1
    feat_names = feature_names_2 if use_second else feature_names_1
    metrics["selected_model"] = tag

    print(f"一阶 9 特征 — train RMSE(四轴平均): {m1_tr:.4f}, val: {m1_val:.4f}")
    print(f"二阶 17 特征 — train RMSE: {m2_tr:.4f}, val: {m2_val:.4f}")
    print(f"选用: {tag}")

    coeff = {
        "version": 1,
        "description": "tau[k] = sum_f beta[k][f] * phi[f]; phi 见 feature_names",
        "units": {
            "q_joint_rad": "相对 HOME，与 Motor_States[i].pos 减去 WORLD_HOME_ABS 一致",
            "tau": "N·m",
        },
        "selected_model": tag,
        "feature_names": feat_names,
        "beta": [[float(x) for x in betas_use[k]] for k in range(4)],
        "motor_names": ["M0", "M1", "M2", "M3"],
    }
    (out_dir / "coefficients.json").write_text(
        json.dumps(coeff, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    per_axis = []
    for k in range(4):
        per_axis.append(
            {
                "motor": k,
                "rmse_all": rmse(T[:, k], pred_all[:, k]),
                "max_abs_err": float(np.max(np.abs(T[:, k] - pred_all[:, k]))),
            }
        )
    metrics["per_axis_selected"] = per_axis

    k_folds = min(5, n // 3) if n >= 9 else 3
    if k_folds >= 2:
        fold_rmse_1: list[float] = []
        fold_rmse_2: list[float] = []
        fold_size = n // k_folds
        for fi in range(k_folds):
            lo = fi * fold_size
            hi = n if fi == k_folds - 1 else (fi + 1) * fold_size
            mask = np.ones(n, dtype=bool)
            mask[lo:hi] = False
            if mask.sum() < 9:
                continue
            q_trf, q_valf = Q[mask], Q[~mask]
            t_trf, t_valf = T[mask], T[~mask]
            p1_tr = build_phi_first_order(q_trf)
            p1_va = build_phi_first_order(q_valf)
            p2_tr = build_phi_second_harmonic(q_trf)
            p2_va = build_phi_second_harmonic(q_valf)
            pr1 = np.column_stack(
                [predict(p1_va, fit_ols(t_trf[:, k], p1_tr, ridge=ridge1)) for k in range(4)]
            )
            pr2 = np.column_stack(
                [predict(p2_va, fit_ols(t_trf[:, k], p2_tr, ridge=ridge2)) for k in range(4)]
            )
            fold_rmse_1.append(rmse(t_valf, pr1))
            fold_rmse_2.append(rmse(t_valf, pr2))
        metrics["kfold"] = {
            "k": k_folds,
            "rmse_mean_first_order": float(np.mean(fold_rmse_1)) if fold_rmse_1 else None,
            "rmse_mean_second_harmonic": float(np.mean(fold_rmse_2)) if fold_rmse_2 else None,
        }

    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    idx = np.arange(n)
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    fig.suptitle(f"Gravity torque: measured vs fit ({tag})", fontsize=12)
    for k in range(4):
        axes[k].plot(idx, T[:, k], "o", ms=3, alpha=0.35, label="meas")
        axes[k].plot(idx, pred_all[:, k], "-", lw=1.2, label="fit")
        axes[k].set_ylabel(f"tau{k} (N*m)")
        axes[k].legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("sample index (capture order)")
    fig.tight_layout()
    fig.savefig(out_dir / "fit_vs_measured_by_index.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(9, 8))
    fig.suptitle("Measured vs predicted (per motor)", fontsize=12)
    for k, ax in enumerate(axes.ravel()):
        ax.scatter(T[:, k], pred_all[:, k], s=12, alpha=0.5)
        lo = min(T[:, k].min(), pred_all[:, k].min())
        hi = max(T[:, k].max(), pred_all[:, k].max())
        ax.plot([lo, hi], [lo, hi], "r--", lw=1)
        ax.set_xlabel("meas tau (N*m)")
        ax.set_ylabel("pred tau (N*m)")
        ax.set_title(f"motor {k}")
        ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    fig.savefig(out_dir / "scatter_actual_vs_pred.png", dpi=150)
    plt.close(fig)

    res = T - pred_all
    fig, axes = plt.subplots(4, 1, figsize=(11, 8), sharex=True)
    fig.suptitle("Residual (meas - fit)", fontsize=12)
    for k in range(4):
        axes[k].axhline(0, color="k", lw=0.5)
        axes[k].plot(idx, res[:, k], ".", ms=4)
        axes[k].set_ylabel(f"dtau{k}")
    axes[-1].set_xlabel("sample index")
    fig.tight_layout()
    fig.savefig(out_dir / "residuals_by_index.png", dpi=150)
    plt.close(fig)

    q2_deg = np.degrees(Q[:, 2])
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.scatter(q2_deg, T[:, 2], s=18, alpha=0.45, label="meas tau2")
    ax.scatter(q2_deg, pred_all[:, 2], s=18, alpha=0.45, label="fit tau2")
    ax.set_xlabel("q2 relative HOME (deg)")
    ax.set_ylabel("tau2 (N*m)")
    ax.legend()
    ax.set_title("Motor2 torque vs q2 (deg from HOME)")
    fig.tight_layout()
    fig.savefig(out_dir / "tau2_vs_q2_deg.png", dpi=150)
    plt.close(fig)

    c_snip = """
/* Generated by gravity_fit/fit_gravity_ls.py — add tau_ff to MIT feedforward torque. */
/* 9 terms: bias + sin/cos per joint. 17 terms: append sin(2q),cos(2q) per joint. */
static float Gravity_TauFF_OneMotor(const float q[4], const float *beta, int nfeat) {
    float s[4], c[4], s2[4], c2[4];
    for (int j = 0; j < 4; j++) {
        s[j] = sinf(q[j]);
        c[j] = cosf(q[j]);
        s2[j] = sinf(2.f * q[j]);
        c2[j] = cosf(2.f * q[j]);
    }
    float t = beta[0];
    t += beta[1]*s[0] + beta[2]*c[0];
    t += beta[3]*s[1] + beta[4]*c[1];
    t += beta[5]*s[2] + beta[6]*c[2];
    t += beta[7]*s[3] + beta[8]*c[3];
    if (nfeat >= 17) {
        t += beta[9]*s2[0] + beta[10]*c2[0];
        t += beta[11]*s2[1] + beta[12]*c2[1];
        t += beta[13]*s2[2] + beta[14]*c2[2];
        t += beta[15]*s2[3] + beta[16]*c2[3];
    }
    return t;
}
"""
    (out_dir / "stm32_snippet_reference.c.txt").write_text(c_snip.strip() + "\n", encoding="utf-8")

    print(f"已写入: {out_dir}")
    print("  coefficients.json, metrics.json, *.png, stm32_snippet_reference.c.txt")


if __name__ == "__main__":
    main()
