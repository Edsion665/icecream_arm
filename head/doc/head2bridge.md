# head2bridge API 规范（icecream 上位机 -> bridge，v1.1 / 与 arm v2.3 同形）

- 文档状态：stable
- 适用场景：icecream **head**（FSM / 脚本）向 **arm_control_bridge** 下达机械臂控制
- 实现：`head/src/speaker.py`（`BridgeClient`）、`head/src/run.py`（`HeadFSM`）
- 相关文档：[`camera2head.md`](camera2head.md)、[`pi2head.md`](pi2head.md)；bridge 侧详规 [`../../arm_control_bridge/doc/head2bridge.md`](../../arm_control_bridge/doc/head2bridge.md)

## 1. 范围（Scope）

本文档定义 head 与 bridge 之间的**控制与响应**约定，并说明 head **如何选用笛卡尔 IK 双模式**。

包含：

- TCP / HTTP JSON（与 arm_control_bridge 同源）
- `BridgeClient.send_pose` / `send_pose_seq` / `send_pose_delta`
- v3 FSM 中各步推荐用法

不包含：bridge → Pi 的 UDP 帧（见 `arm_control_bridge` 内 `bridge2pi` 文档）。

## 2. 传输与端点

**端口以实际启动为准**（同机多服务时常与示例错开）：

| 链路 | 默认（示例） | 配置 |
|---|---|---|
| bridge HTTP | `http://127.0.0.1:8877` | `arm_control_bridge` → `CONFIG.web_test_port` |
| head → bridge | 同上 | `head/config.yaml` → `bridge_base_url` |
| bridge TCP | `0.0.0.0:9888` | bridge 监听端口 |
| head ingestion | `0.0.0.0:8776` | `ingest_port` |

`bridge_base_url` **必须与** bridge HTTP 基址一致，否则 `send_pose_seq` 等会 404 或连错服务。

## 3. 接口总览

1. **关节（4 电机）**：`joints` / `joints_delta` → `send_joints`
2. **手腕 + 夹爪**：`claw` → `send_claw`
3. **笛卡尔（IK 双模式）**：`pose` / `pose_seq` / `pose_delta` → 见 §4、§10
4. **其它**：`stop`、`ping`；idle 阶段周期 `joints` + `claw`

## 4. 笛卡尔 IK 双模式（协议层）

与 bridge 文档 §4.5 一致，head 侧通过 **不同 HTTP 路由 / `cmd`** 选择模式：

| 模式 | bridge `cmd` | HTTP | `BridgeClient` 方法 | 行为摘要 |
|---|---|---|---|---|
| **1. 单帧** | `pose` | `POST /api/pose` | `send_pose(x,y,z)` | IK 一次 → 单帧目标 → Pi ramp |
| **2. 序列** | `pose_seq` | `POST /api/pose_seq` | `send_pose_seq(x,y,z)` | PC 25Hz 插值，每帧 j4∝j2+j3 |
| **增量** | `pose_delta` | `POST /api/pose_delta` | `send_pose_delta(dx,dy,dz)` | 在当前目标上增量，走**模式 1** |

公共请求字段（`pose` / `pose_seq`）：`x`, `y`, `z`（m，与 `Position` / 相机 `robot_base` 一致）。

成功且到位时，阻塞 POST 响应含 `actual_pose`（及 `reached` 等），`send_pose` 与 `send_pose_seq` **校验规则相同**（见 `speaker._require_blocking_reached`）。

## 5. 命令结构（字段）

### 5.1 通用

| 字段 | 类型 | 必选 | 说明 |
|---|---|---:|---|
| `cmd` | string | 否 | 与 `type` 二选一 |

### 5.2 `joints` / `joints_delta` / `claw`

与 bridge 规范相同；`claw` 须 `wrist_deg` + `grip_state`/`open_close`（**0=合拢，1=张开**，与 bridge 一致）。

### 5.3 `pose` / `pose_seq` / `pose_delta`

见 §4；`pose_delta` 额外需要 `dx`, `dy`, `dz`。

## 6. 请求与响应

### 6.1 笛卡尔到位字段

| 字段 | 说明 |
|---|---|
| `actual_pose` | 到位后实测 `x/y/z`（m） |
| `reached` | 是否判到位 |
| `error_joints_deg` | 关节误差（度） |

`pose_seq` 播放期间 bridge 对外 `reached: false`（`reach_reason: pose_seq_playing`），播完后与 `pose` 相同。

### 6.2 HTTP 路由

- `POST /api/pose` → `cmd=pose`
- `POST /api/pose_seq` → `cmd=pose_seq`
- `POST /api/pose_delta` → `cmd=pose_delta`
- `POST /api/joints` · `/api/joints_delta` · `/api/claw`
- `GET /api/reached` → 轮询快照（`poll_reached`，一般由 bridge 阻塞 POST 代替）

## 7. curl 联调（不经过 head）

假定 bridge HTTP 为 `127.0.0.1:8877`：

```bash
# 模式 1
curl -sS -X POST http://127.0.0.1:8877/api/pose \
  -H 'Content-Type: application/json' \
  -d '{"cmd":"pose","x":0.35,"y":0.20,"z":0.25}'

# 模式 2（竖直约束序列）
curl -sS -X POST http://127.0.0.1:8877/api/pose_seq \
  -H 'Content-Type: application/json' \
  -d '{"cmd":"pose_seq","x":0.35,"y":0.20,"z":0.25}'

# 上抬 10cm（模式 1 增量）
curl -sS -X POST http://127.0.0.1:8877/api/pose_delta \
  -H 'Content-Type: application/json' \
  -d '{"cmd":"pose_delta","dx":0,"dy":0,"dz":0.10}'
```

## 8. TCP 最小联调

```json
{"cmd":"pose","x":0.35,"y":0.20,"z":0.25}
{"cmd":"pose_seq","x":0.35,"y":0.20,"z":0.25}
{"cmd":"pose_delta","dx":0,"dy":0,"dz":0.10}
```

## 9. 错误模型

与 bridge 一致：`400` + `error` 字符串；运动超时 `408`；`error: superseded` 表示被新命令抢占。

## 10. head 逻辑：如何调用 IK 双模式

### 10.1 代码入口

```
head/src/run.py          HeadFSM._pose_to / _pose_delta_to
        ↓
head/src/speaker.py      BridgeClient.send_pose | send_pose_seq | send_pose_delta
        ↓ HTTP
arm_control_bridge       engine: pose 单帧 IK | pose_seq 序列 | pose_delta
```

**当前默认**：`_pose_to()` → `send_pose()`（模式 1）。  
**竖直段**：改为 `send_pose_seq()` 或给 `_pose_to(..., vertical=True)`（见 §10.3）。

### 10.2 终端直接测 `send_pose_seq`（与 FSM 相同客户端）

```bash
cd /path/to/icecreamarm/head
PYTHONPATH=. python3 -c "
from src.config import load_settings
from src.speaker import BridgeClient
s = load_settings('config.yaml')
sp = BridgeClient(s)
rep = sp.send_pose_seq(0.35, 0.20, 0.25, context='pick_vertical')
print('ok=', rep.ok, 'body=', rep.body)
"
```

`context` 只出现在 head 日志，不进 JSON body。

### 10.3 v3 FSM 步骤与推荐模式

| 代码位置 | 标签示例 | 现实现 | 推荐 |
|---|---|---|---|
| `_pose_to` → step6 物体位 | `step6_object_pose` | `send_pose` | **`send_pose_seq`** |
| `_pose_to` → step10 放置位 | `step10_target_pose` | `send_pose` | **`send_pose_seq`** |
| `_place_reobserve_at_hover` | `*_goto_hover` | `send_pose` | **`send_pose_seq`**（竖直要求高时） |
| `_pose_delta_to` → step8/11b 上抬 | `*_retreat_lift` | `send_pose_delta` | 保持（短 Z 向） |
| step1/3 `joints`、idle | — | `send_joints` | 不涉及 IK 模式 |

**原则**：

- 需要 **j2/j3/j4 同步、末端竖直** 的笛卡尔接近 → `send_pose_seq`
- 已接近目标后的 **纯上抬 / 小增量** → `send_pose_delta`（内部单帧 IK）
- 粗调、待命、竖直要求低 → `send_pose`

### 10.4 在 FSM 中接入（示例）

**方式 A — 按步骤显式调用**（改动最小）：

```python
# step6 抓取接近
self.speaker.send_pose_seq(
    work.x, work.y, work.z, context="step6_object_pose"
)
```

**方式 B — 扩展 `_pose_to` 统一入口**（推荐长期维护）：

```python
def _pose_to(self, pos: Position, label: str, *, role: Role, vertical: bool = False) -> None:
    if vertical:
        self.speaker.send_pose_seq(pos.x, pos.y, pos.z, context=label)
    else:
        self.speaker.send_pose(pos.x, pos.y, pos.z, context=label)
```

调用处：

```python
self._pose_to(self._work_pose(obj, "object"), "step6_object_pose", role="object", vertical=True)
self._pose_to(self._work_pose(tgt, "target"), "step10_target_pose", role="target", vertical=True)
```

上抬**不要**改成 `pose_seq`，继续：

```python
self._pose_delta_to(0.0, 0.0, self.settings.retreat_lift_m, "step8_retreat_lift")
```

### 10.5 与其它 head 行为的关系

- **`send_claw` / `send_joints`**：与 IK 模式独立；仍须在 `pose` / `pose_seq` 前后按 FSM 顺序调用。
- **`state_timeout_s`**：对 `send_pose_seq` 同样生效（序列更长时可能需加大超时）。
- **`require_bridge_feedback`**：两种笛卡尔模式均要求 Pi UDP 反馈才判 `ok`。
- **命令抢占**：FSM 下发新 `pose`/`pose_seq`/`joints` 会清空 bridge 未播完的 `pose_seq` 队列。

### 10.6 联调检查清单

1. bridge 已启动且 `web_test_port > 0`（日志中有 HTTP URL）。
2. `config.yaml` 的 `bridge_base_url` 与 bridge 端口一致。
3. §7 `curl` 测通 `pose_seq` 后，再改 FSM 或 §10.2 脚本。
4. 真机确认 `pi2camera` UDP 反馈，否则 `require_bridge_feedback` 会报错。

## 11. 交叉引用

- 感知：[`camera2head.md`](camera2head.md)
- 放行 FSM：[`pi2head.md`](pi2head.md)
- FSM 步骤表：[`../README.md`](../README.md)
- bridge 协议与 bridge 侧 head 集成：[`../../arm_control_bridge/doc/head2bridge.md`](../../arm_control_bridge/doc/head2bridge.md)
