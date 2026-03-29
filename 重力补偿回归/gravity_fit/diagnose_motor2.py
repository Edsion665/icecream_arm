"""
诊断指定电机轴的拟合残差，列出最差样本（含度×100 与 rad），可选剔除离群后重拟合。

单位说明（与 fit_gravity_ls / STM32 一致）:
  - jsonl 里 motors.k.position: 相对 HOME 的「度×100」整数/浮点
  - 拟合与固件内部: q_k 为弧度 (rad)，q = (deg100/100)*pi/180
  - predict_gravity.py 命令行: 四个数为 q0..q3 的弧度

用法:
  python diagnose_motor2.py
  python diagnose_motor2.py --motor 2 --top 20
  python diagnose_motor2.py --refit --drop-worst 8
  python diagnose_motor2.py --refit --drop-worst 8 --cross
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

from fit_common import (
    build_phi_extended,
    fit_ols,
    load_jsonl_meta,
    predict,
    rmse,
)


def load_beta(path: Path) -> tuple[list[list[float]], list[str]]:
    c = json.loads(path.read_text(encoding="utf-8"))
    return c["beta"], list(c["feature_names"])


def predict_all(phi: np.ndarray, betas: list[np.ndarray]) -> np.ndarray:
    return np.column_stack([predict(phi, betas[k]) for k in range(4)])


def main() -> None:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description="Diagnose gravity fit residuals per motor")
    ap.add_argument("--motor", type=int, default=2, help="motor index 0..3 (default 2)")
    ap.add_argument("--top", type=int, default=20, help="list top N worst |residual|")
    ap.add_argument(
        "--coeff",
        type=Path,
        default=None,
        help="coefficients.json path (default: out/coefficients.json)",
    )
    ap.add_argument(
        "--data",
        type=Path,
        default=None,
        help="captures jsonl (default: ../captures_shifted_to_new_home.jsonl)",
    )
    ap.add_argument("--refit", action="store_true", help="after diagnosis, refit excluding outliers")
    ap.add_argument(
        "--drop-worst",
        type=int,
        default=8,
        help="when --refit: remove this many worst samples for --motor (default 8)",
    )
    ap.add_argument(
        "--cross",
        action="store_true",
        help="when --refit: use 17+6 sin*sin cross terms (23 features)",
    )
    ap.add_argument("--ridge", type=float, default=2e-3, help="ridge lambda for refit")
    args = ap.parse_args()

    m = args.motor
    if m < 0 or m > 3:
        sys.exit("--motor must be 0..3")

    data_path = args.data
    if data_path is None:
        data_path = here.parent / "captures_shifted_to_new_home.jsonl"
        if not data_path.is_file():
            data_path = here / "captures_shifted_to_new_home.jsonl"

    coeff_path = args.coeff or (here / "out" / "coefficients.json")
    if not coeff_path.is_file():
        print(f"缺少 {coeff_path}，请先运行 python fit_gravity_ls.py", file=sys.stderr)
        sys.exit(1)

    Q, T, D100, times = load_jsonl_meta(Path(data_path))
    n = Q.shape[0]
    beta_json, feat_saved = load_beta(coeff_path)
    betas = [np.asarray(b, dtype=np.float64) for b in beta_json]
    nfeat_saved = len(betas[0])

    from fit_common import build_phi_first_order, build_phi_second_harmonic

    if nfeat_saved == 17:
        phi = build_phi_extended(Q, use_cross=False)[0]
    elif nfeat_saved == 23:
        phi = build_phi_extended(Q, use_cross=True)[0]
    elif nfeat_saved == 9:
        phi = build_phi_first_order(Q)
    else:
        print(f"未知特征维数 {nfeat_saved}，按 17 维二阶处理", file=sys.stderr)
        phi = build_phi_second_harmonic(Q)

    pred = predict_all(phi, betas)
    res = T - pred
    r_m = res[:, m]
    abs_r = np.abs(r_m)
    order = np.argsort(-abs_r)

    print(f"数据: {data_path}  (N={n})")
    print(f"系数: {coeff_path}  (features={nfeat_saved})")
    print(f"电机 {m}: RMSE={rmse(T[:, m], pred[:, m]):.4f} N*m, max|err|={float(np.max(abs_r)):.4f}")
    print(f"\n最差 {args.top} 条（按 |tau_meas - tau_pred| 排序，电机{m}）:\n")
    print(
        f"{'idx':>4} {'|res|':>9} {'tau_m':>9} {'pred':>9} "
        f"{'d0':>8} {'d1':>8} {'d2':>8} {'d3':>8}  time"
    )

    out_csv = here / "out" / f"worst_residuals_motor{m}.csv"
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    rows_out: list[dict] = []

    for rank in range(min(args.top, n)):
        i = int(order[rank])
        d0, d1, d2, d3 = D100[i]
        ti = times[i] if i < len(times) else ""
        print(
            f"{i:4d} {abs_r[i]:9.4f} {T[i, m]:9.4f} {pred[i, m]:9.4f} "
            f"{d0:8.1f} {d1:8.1f} {d2:8.1f} {d3:8.1f}  {ti}"
        )
        rows_out.append(
            {
                "rank": rank + 1,
                "sample_index": i,
                "abs_residual_motor_m": abs_r[i],
                "tau_meas": T[i, m],
                "tau_pred": pred[i, m],
                "deg100_0": d0,
                "deg100_1": d1,
                "deg100_2": d2,
                "deg100_3": d3,
                "q0_rad": Q[i, 0],
                "q1_rad": Q[i, 1],
                "q2_rad": Q[i, 2],
                "q3_rad": Q[i, 3],
                "time": ti,
            }
        )

    with out_csv.open("w", newline="", encoding="utf-8") as f:
        if rows_out:
            w = csv.DictWriter(f, fieldnames=list(rows_out[0].keys()))
            w.writeheader()
            w.writerows(rows_out)
    print(f"\n已写 CSV: {out_csv}")

    if not args.refit:
        print("\n提示: 弧度用于 predict_gravity.py 与固件；jsonl 里是度×100。")
        return

    # ----- 重拟合：去掉该轴残差最大的 drop_worst 个样本，四轴一起重估 -----
    drop = min(max(0, args.drop_worst), n - 10)
    if drop <= 0:
        print("样本过少，跳过 --refit", file=sys.stderr)
        return
    bad_idx = set(int(x) for x in order[:drop])
    keep = np.array([i for i in range(n) if i not in bad_idx], dtype=np.int64)
    Qk, Tk = Q[keep], T[keep]
    phi_k, feat_names = build_phi_extended(Qk, use_cross=args.cross)
    ridge = float(args.ridge)

    betas_new: list[np.ndarray] = []
    for k in range(4):
        betas_new.append(fit_ols(Tk[:, k], phi_k, ridge=ridge))

    phi_all, _ = build_phi_extended(Q, use_cross=args.cross)
    pred_new = predict_all(phi_all, betas_new)
    print(f"\n--refit: 剔除电机{m} 上 |残差| 最大的 {drop} 个样本，剩余 {len(keep)}")
    print(f"  特征维: {phi_k.shape[1]} (cross={args.cross}), ridge={ridge}")
    for k in range(4):
        print(
            f"  motor{k} RMSE: {rmse(T[:, k], pred[:, k]):.4f} -> {rmse(T[:, k], pred_new[:, k]):.4f}  "
            f"max|err|: {float(np.max(np.abs(T[:, k] - pred[:, k]))):.4f} -> {float(np.max(np.abs(T[:, k] - pred_new[:, k]))):.4f}"
        )

    out_dir = here / "out"
    tag = "coefficients_refit_drop%d_m%d" % (drop, m)
    if args.cross:
        tag += "_cross23"
    coeff_out = {
        "version": 1,
        "description": f"refit after dropping {drop} worst residuals on motor {m}; cross={args.cross}",
        "units": {
            "q_joint_rad": "relative HOME, same as firmware",
            "tau": "N*m",
        },
        "selected_model": tag,
        "feature_names": feat_names,
        "beta": [[float(x) for x in betas_new[k]] for k in range(4)],
        "motor_names": ["M0", "M1", "M2", "M3"],
        "dropped_sample_indices": sorted(bad_idx),
    }
    out_json = out_dir / f"{tag}.json"
    out_json.write_text(json.dumps(coeff_out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"\n已写: {out_json}")

    meta = {
        "motor_focus": m,
        "dropped_count": drop,
        "dropped_indices": sorted(bad_idx),
        "use_cross": args.cross,
        "ridge": ridge,
        "rmse_before_after": [
            {
                "motor": k,
                "rmse_before": rmse(T[:, k], pred[:, k]),
                "rmse_after": rmse(T[:, k], pred_new[:, k]),
            }
            for k in range(4)
        ],
    }
    (out_dir / f"{tag}_metrics.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    if plt is not None:
        idx = np.arange(n)
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(idx, T[:, m], "o", ms=3, alpha=0.4, label="meas")
        ax.plot(idx, pred[:, m], "-", lw=1, alpha=0.6, label="before")
        ax.plot(idx, pred_new[:, m], "-", lw=1.2, label="after refit")
        for bi in bad_idx:
            ax.axvline(bi, color="r", alpha=0.25, lw=0.8)
        ax.set_ylabel(f"tau{m} (N*m)")
        ax.set_xlabel("sample index")
        ax.legend()
        ax.set_title(f"motor {m}: refit dropped {drop} outliers (red lines)")
        fig.tight_layout()
        png = out_dir / f"{tag}_tau{m}_compare.png"
        fig.savefig(png, dpi=150)
        plt.close(fig)
        print(f"已写图: {png}")


if __name__ == "__main__":
    main()
