# icecream / head

上位机 head：Python 源码在 **`src/`** 包内；接收相机 `camera2head` 检测、维护 `tracker` 槽位、按 **v3 十二步状态机** 通过 `speaker` 调用 `head2bridge` 控制机械臂。入口：`python -m src.run`。

## 文档

- [doc/head2bridge.md](doc/head2bridge.md) — head → bridge（`joints` / `pose` / `claw`）
- [doc/camera2head.md](doc/camera2head.md) — camera → head（`detection` v1.1）
- [doc/pi2head.md](doc/pi2head.md) — 树莓派 → head（`start` / `ping`，放行 FSM）

## 启动与待命（pi2head）

1. head 启动后监听 **`pi2head_tcp_port`**（默认 `8778`），并启动相机 ingestion。
2. **收到树莓派 TCP `{"cmd":"start"}` 之前**：以 `idle_bridge_hz`（默认 2Hz）向 bridge 周期下发 **全零** `joints`（`idle_axes_rel_deg`，默认 `[0,0,0,0]`）与 `claw`（腕 0、`idle_grip_state` 默认 0）。
3. 收到 **`start`** 后进入下方 v3 主循环；重复 `start` 不重置已在跑的 FSM。

树莓派示例见 `doc/pi2head.md`。

## v3 主循环（概要）

| 步 | 行为 |
|---|------|
| （首） | 每轮开始清空 `target` / `object` 槽（不沿用上一轮） |
| 1 | `joints` obs1 → `claw`（腕零 + **夹爪张开**） |
| 2 | 在 **obs1** 后等待帧内出现 `target` → `clear` + `apply` target 槽 |
| 3 | `claw`（obs2 入口腕角）→ `joints` obs2 |
| 4 | 在 **obs2** 后等待帧内出现 `object` → `clear` + `apply` object 槽 |
| 5 | `plan`（需同时有 target + object） |
| 6–7 | `claw`（物体腕）→ `pose` 物体 → 到位后 `claw` 抓取；抓取后 **丢弃 object 槽** |
| 8 | 物体抓取后：`joints` **先回 obs1** → 到位后再 `claw` 腕零（回程保持抓取腕角） |
| 9 | 在 **obs1** 下再等待 `target` 帧 → `clear` + `apply`（放置用 target 须重新在 obs1 获得） |
| 10–11 | `claw`（目标腕）→ `pose` 目标 → `claw` 释放；队列目标在到位后出队 |
| 11b | `joints` **先回 obs1** → 到位后再 `claw` 腕零（回程保持放置腕角与张开） |
| 11 末 | **丢弃** target/object 槽 |
| 12 | 循环回（首） |

**槽位语义**：`target` / `object` 均为用完即弃；**仅**在对应观测位（obs1 得 target、obs2 得 object）并经 `wait` + `clear` + `apply` 后的快照可用于本轮后续 `plan` / `pose`。

**为何第二轮 step2 与 step3 同一时刻、像「直接去 obs2」**：ingestion 持续更新 `_last_frame`，槽位虽在每轮清空，**但** `wait_for_roles` 看的是缓存帧；若仍含上一轮的 `target`，等待会立刻通过。默认 **`require_fresh_detection_after_obs: true`**：每次 obs1 / obs2 / 抓取后回 obs1 的 **joints 到位后** 会丢弃该缓存，必须再收到 **新** 的 `POST /api/detection` 才会继续（与「在 obs1/obs2 再获得」一致）。联调单发一帧时可临时设 `false`。

**为何会在 obs1 / obs2 之间来回动很多下**：若 `camera_wait_timeout_s` 为**有限秒数**，等不到 target/object 会抛错，`run_forever` 捕获后**整轮重跑** `run_one_cycle`（又从周期首清空 → obs1 → …），看起来就像反复在观测位之间运动。联调请用 **`camera_wait_timeout_s: null`**（默认）或省略该项，使在 **obs1 一直等 target**、在 **obs2 一直等 object**，直到 ingestion 发来含对应 role 的帧。

可配置项见 `config.example.yaml`（`camera_wait_timeout_s`、`emit_claw_on_every_transition` 等）。

## 安装与运行

### 使用 uv（推荐）

```bash
cd /path/to/icecream/head
uv sync --group dev          # 含 pytest；运行时依赖见 pyproject
cp config.example.yaml config.yaml
uv run python -m src.run --config config.yaml
```

**只跑测试**（使用 uv 管理的 `.venv`）：

```bash
cd /path/to/icecream/head
uv sync --group dev
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/
```

### 使用 pip + venv

```bash
cd /path/to/icecream/head
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
# 推荐在 venv 内：pip install -e .
# 或免安装：PYTHONPATH=. python -m src.run --config config.yaml
cp config.example.yaml config.yaml
PYTHONPATH=. python -m src.run --config config.yaml
```

默认 **ingestion** 监听 `config.yaml` 中 `ingest_host:ingest_port`（示例 `0.0.0.0:8776`）。**bridge** 基址 `bridge_base_url`（示例 `http://127.0.0.1:8775`）。

## curl 联调

**相机（ingestion）：**

```bash
curl -sS -X POST http://127.0.0.1:8776/api/detection \
  -H 'Content-Type: application/json' \
  -d '{"frame":"robot_base","objects":[{"role":"target","class_id":1,"label":"t","position":{"x":0.5,"y":0,"z":0.1},"wrist_yaw_deg":0},{"role":"object","class_id":2,"label":"o","position":{"x":0.4,"y":0,"z":0.1},"wrist_yaw_deg":5}]}'
```

**bridge** 需单独启动（nosim_bridge / arm_control_bridge），端口与 `config.yaml` 一致。

## 测试

```bash
uv sync --group dev
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q
```

（无 uv 时：`pip install pytest` 后同样执行 `pytest -q`。）

`tests/test_tracker.py` 覆盖融合、refresh、`clear_role` / `apply_roles_from_last_frame`、`wait_for_roles`；`tests/test_smoke_cycle.py` 用内存 mock bridge 跑通一整轮 v3 FSM。本地联调可先起 **`tools/mock_bridge.py`**（默认 `127.0.0.1:8775`），再 `PYTHONPATH=. python -m src.run --config config.yaml`，并用 curl 向 ingestion 端口推送 `doc/camera2head.md` 示例 JSON。单次循环调试：`HEAD_SINGLE_CYCLE=1 PYTHONPATH=. python -m src.run --config config.yaml`。
