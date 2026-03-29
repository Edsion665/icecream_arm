#!/usr/bin/env python3
"""
小型 MLP 对照实验：用 (q1,q2,q3) 预测 (τ肩,τ肘,τ腕)，与 gravity_identification 同一数据。

关于「交叉熵 + 回归」：
  - 交叉熵（CrossEntropyLoss）用于 **分类**：目标为类别下标、输出为各类 logit。
  - 力矩 τ 是 **连续值 (Nm)**，标准做法是 **MSE / Huber / MAE**，与当前最小二乘同属回归。
  - 若坚持用 CE，必须把 τ **分箱成离散类别**（会引入量化误差，且不再是「牛米直接回归」）。
  默认训练损失：**MSE**（均方误差，回归标准做法）。
  sklearn：内部为平方损失；PyTorch：`nn.MSELoss()`；可选 `--huber` 换 SmoothL1。

依赖：numpy + scikit-learn；可选 torch（--backend torch）
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from gravity_identification import (  # noqa: E402
    CAPTURES,
    _ones,
    angles_from_Q,
    fit_cos_sin,
    load_dataset,
    mae_r2,
)

ROOT = Path(__file__).resolve().parent.parent


def predict_cos_sin(Q: np.ndarray, t1: np.ndarray, t2: np.ndarray, t3: np.ndarray, *, bias: bool) -> np.ndarray:
    """用 fit_cos_sin 返回的系数在任意 Q 上预测。"""
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
    return np.column_stack([W1 @ t1, W2 @ t2, W3 @ t3])


def metrics_per_joint(y: np.ndarray, yp: np.ndarray) -> None:
    names = ("肩", "肘", "腕")
    for j in range(3):
        m, r = mae_r2(y[:, j], yp[:, j])
        print(f"  {names[j]}: MAE={m:.4f} Nm  R²={r:.4f}")


def run_sklearn_mlp(
    X_tr: np.ndarray,
    y_tr: np.ndarray,
    X_va: np.ndarray | None,
    hidden: tuple[int, ...],
    seed: int,
    max_iter: int,
) -> tuple[np.ndarray, np.ndarray | None]:
    """在训练集上拟合（损失为平方误差 / MSE）；返回 train 预测，若有验证集则返回 val 预测。"""
    from sklearn.neural_network import MLPRegressor
    from sklearn.preprocessing import StandardScaler

    sx = StandardScaler()
    sy = StandardScaler()
    Xn_tr = sx.fit_transform(X_tr)
    yn_tr = sy.fit_transform(y_tr)

    mlp = MLPRegressor(
        hidden_layer_sizes=hidden,
        activation="relu",
        solver="adam",
        random_state=seed,
        max_iter=max_iter,
        early_stopping=X_va is None,
        validation_fraction=0.15 if X_va is None else 0.0,
        n_iter_no_change=50,
    )
    mlp.fit(Xn_tr, yn_tr)
    pred_tr = sy.inverse_transform(mlp.predict(Xn_tr)).astype(np.float64)
    if X_va is None:
        return pred_tr, None
    Xn_va = sx.transform(X_va)
    pred_va = sy.inverse_transform(mlp.predict(Xn_va)).astype(np.float64)
    return pred_tr, pred_va


def run_torch_mlp(
    X_tr: np.ndarray,
    y_tr: np.ndarray,
    X_va: np.ndarray | None,
    hidden: tuple[int, ...],
    seed: int,
    epochs: int,
    lr: float,
    huber: bool,
) -> tuple[np.ndarray, np.ndarray | None]:
    import torch
    import torch.nn as nn

    torch.manual_seed(seed)
    np.random.seed(seed)

    device = torch.device("cpu")
    Xt = torch.from_numpy(X_tr).float().to(device)
    yt = torch.from_numpy(y_tr).float().to(device)
    xm, xs = Xt.mean(0, keepdim=True), Xt.std(0, keepdim=True).clamp_min(1e-6)
    ym, ys = yt.mean(0, keepdim=True), yt.std(0, keepdim=True).clamp_min(1e-6)
    Xn = (Xt - xm) / xs
    yn = (yt - ym) / ys

    layers: list[nn.Module] = []
    d_in = 3
    for h in hidden:
        layers.extend([nn.Linear(d_in, h), nn.ReLU(inplace=True)])
        d_in = h
    layers.append(nn.Linear(d_in, 3))
    net = nn.Sequential(*layers).to(device)

    opt = torch.optim.Adam(net.parameters(), lr=lr)
    crit = nn.SmoothL1Loss() if huber else nn.MSELoss()

    net.train()
    for _ in range(epochs):
        opt.zero_grad()
        out = net(Xn)
        loss = crit(out, yn)
        loss.backward()
        opt.step()

    net.eval()
    with torch.no_grad():
        pred_tr = (net(Xn) * ys + ym).cpu().numpy().astype(np.float64)
        if X_va is None:
            return pred_tr, None
        Xv = torch.from_numpy(X_va).float().to(device)
        Xnv = (Xv - xm) / xs
        pred_va = (net(Xnv) * ys + ym).cpu().numpy().astype(np.float64)
    return pred_tr, pred_va


def main() -> int:
    ap = argparse.ArgumentParser(description="MLP 重力映射对照（MSE 训练；同 captures.jsonl）")
    ap.add_argument("--captures", type=Path, default=CAPTURES, help="jsonl 路径")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--hidden", type=str, default="64,32", help="隐层宽度，逗号分隔，如 64,32")
    ap.add_argument("--backend", choices=("sklearn", "torch"), default="sklearn")
    ap.add_argument("--epochs", type=int, default=8000, help="仅 torch：训练轮数")
    ap.add_argument("--lr", type=float, default=1e-3, help="仅 torch：学习率")
    ap.add_argument("--huber", action="store_true", help="torch：SmoothL1 代替 MSE；sklearn 仍用平方损失")
    ap.add_argument("--max-iter", type=int, default=10000, help="仅 sklearn：最大迭代")
    ap.add_argument(
        "--val-ratio",
        type=float,
        default=0.2,
        help="验证集比例（默认 0.2）；设为 0 则全数据训练并只在训练集上报告",
    )
    args = ap.parse_args()

    hidden = tuple(int(x) for x in args.hidden.split(",") if x.strip())

    X, y = load_dataset(args.captures)
    n = X.shape[0]
    print(f"样本数: {n}  输入: q1,q2,q3 (rad)  输出: τ肩,τ肘,τ腕 (Nm)")
    print(f"隐层: {hidden}  激活: ReLU  backend: {args.backend}  训练损失: {'SmoothL1' if (args.backend == 'torch' and args.huber) else 'MSE'}\n")

    use_val = args.val_ratio > 1e-6 and n >= 8
    if use_val:
        from sklearn.model_selection import train_test_split

        X_tr, X_va, y_tr, y_va = train_test_split(
            X, y, test_size=args.val_ratio, random_state=args.seed
        )
        print(f"划分: 训练 {X_tr.shape[0]} 条 / 验证 {X_va.shape[0]} 条 (val_ratio={args.val_ratio})\n")
    else:
        X_tr, y_tr = X, y
        X_va, y_va = None, None
        print("划分: 全数据训练（未划分验证集）\n")

    # 线性基线：与 MLP 使用相同训练集；在全数据时与以前一致
    _, t1, t2, t3 = fit_cos_sin(X_tr, y_tr, bias=True)
    pred_ls_tr = predict_cos_sin(X_tr, t1, t2, t3, bias=True)
    print("=== 线性基线 cos+sin + 截距（最小二乘，仅在训练集上拟合）===")
    print("  [训练集]")
    metrics_per_joint(y_tr, pred_ls_tr)
    if use_val:
        pred_ls_va = predict_cos_sin(X_va, t1, t2, t3, bias=True)
        print("  [验证集]")
        metrics_per_joint(y_va, pred_ls_va)

    print("\n=== MLP（损失: MSE 或 torch+Huber；ReLU）===")
    if args.backend == "sklearn":
        pred_m_tr, pred_m_va = run_sklearn_mlp(
            X_tr, y_tr, X_va, hidden, args.seed, args.max_iter
        )
    else:
        try:
            pred_m_tr, pred_m_va = run_torch_mlp(
                X_tr, y_tr, X_va, hidden, args.seed, args.epochs, args.lr, args.huber
            )
        except ImportError:
            print("未安装 torch，请: pip install torch  或改用 --backend sklearn", file=sys.stderr)
            return 1

    print("  [训练集]")
    metrics_per_joint(y_tr, pred_m_tr)
    if pred_m_va is not None:
        print("  [验证集]")
        metrics_per_joint(y_va, pred_m_va)

    loss_note = "sklearn 使用平方损失（等价于最小化 MSE）"
    if args.backend == "torch":
        loss_note = "PyTorch: " + ("SmoothL1(Huber)" if args.huber else "nn.MSELoss()")
    print(f"\n（{loss_note}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
