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

## v3 主循环（八步 + 内层 5–8）

| 步 | 行为 |
|---|------|
| （首） | 清空 `target` / `object` 槽 |
| 1 | `joints` obs1 + 夹爪张开 |
| 2 | obs1 **粗观测** `target` → `target_queue` |
| 3 | 队列**前 N** 项（`max_refine_targets`）：各目标**正上方**精观测 → 写回队列 |
| 4 | `joints` obs2 + 传送带送料（如需）+ 稳定观测 `object` |
| **5–8 内层** | 见下表；队列空或 step8 无法 `plan_pick` 时结束内层 |
| 末 | `POST /api/stepper` 转盘 `turntable_stepper_deg`（默认 +90°）→ 下轮从 step1 |

**内层循环（每放一个 target 一轮）：**

| 步 | 行为 |
|---|------|
| 5 | `plan_pick` → 物体上方 `pose` → **`pose_lin` 竖直下降**（25Hz IK，`vertical_move_speed_m_s`）→ 到位确认 → 夹紧 |
| 6 | 上抬 → 放置腕角 → **`joints` 回 obs1**（Pi 默认速度）；**到位后**再切换放置降速 |
| 7 | 放置位上方 → **`pose_lin` 竖直下降** → 到位确认 → 松爪；上抬用 **`pose_lin_delta`** |
| 8 | 上抬 → 转腕 → **obs2**；队列非空则探视野/传送带 + 再观测 `object`，能 `plan_pick` 则回到 5 |

**槽位语义**：粗 target 在 step2；精坐标在 step3 写回队列；每轮 pick 后丢弃 `object`，step8 前重新观测 `object`。放置后队列项仅标记 `placed`，**stepper 转动后**才整体清空队列。

**配置**：`max_refine_targets`、`place_reobserve_hover_m`、`turntable_stepper_deg`、`stepper_settle_s`、`arm_speed_place_rad_s`（见 `config.example.yaml`）。

**速度**：平时发 `[0,0,0,0]` 用 Pi 默认；**竖直下降/上抬**用 bridge `POST /api/pose_lin`（`lock_xy=true`，25Hz IK 插补，`vertical_move_speed_m_s`）；step6 回 obs1 用 `arm_speed_place_rad_s`（关节段）。

**预接近**：`approach_hover_m` 悬停后 `pose_lin` 竖直到位；到位后 `approach_reached_stable_frames` + `approach_settle_after_descend_s` 再夹爪/松爪。

**为何 step2 与 step3 同一时刻、像「直接去 obs2」**：ingestion 持续更新 `_last_frame`，槽位虽在每轮清空，**但** `wait_for_roles` 看的是缓存帧；若仍含上一轮的 `target`，等待会立刻通过。默认 **`require_fresh_detection_after_obs: true`**：obs1 / obs2 **joints 到位后** 会丢弃该缓存，须再收到 **新** `POST /api/detection` 才继续。联调单发一帧时可临时设 `false`。

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
