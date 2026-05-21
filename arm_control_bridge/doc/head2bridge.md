# head2bridge API 规范（上位机 -> arm_control_bridge，v2.2）

- 文档状态：stable
- 适用模块：`arm_control_bridge/io/listener.py`、`arm_control_bridge/calculator.py`
- 对端：上位机（UI/策略/脚本）-> `arm_control_bridge`
- 相关文档：`docs/bridge2pi.md`

## 1. 范围（Scope）

本文档定义上位机与 bridge 之间的**控制与响应**约定，包括：

- TCP JSON 行协议（上行命令；v2.2 起笛卡尔命令支持下行到位反馈）
- HTTP JSON API（含 `pose` / `pose_seq` / `pose_delta` 成功时的实际位姿回传）
- 6 维控制语义（4 电机 + 手腕 + 夹爪状态）

不包含：

- bridge 到 Pi 的二进制 UDP 帧细节（见 `docs/bridge2pi.md`）
- Pi 到 STM32 串口协议（见 `docs/pi2stm.md`）

## 2. 传输与端点

### 2.1 TCP（JSON 行）

- 默认监听：`0.0.0.0:9888`
- 编码：UTF-8
- 每行一条 JSON（`\n` 分隔）

### 2.2 HTTP（JSON）

- 默认服务：`127.0.0.1:8877`（`CONFIG.web_test_port`，nosim 默认 `8877`）
- 仅在 `CONFIG.web_test_port > 0` 时启用
- `Content-Type: application/json`

## 3. 接口总览（必须提供的三类接口）

本协议对上位机明确三类控制接口：

1. **关节信息接口（4电机）**
   - 目标：直接控制 4 个电机（J1~J4）的角度。
   - 命令：`joints` / `joints_delta`
2. **舵机信息接口（手腕 + 夹爪）**
   - 目标：控制手腕角度与夹爪开合状态（0/1）。
   - 命令：`claw`
3. **位置控制接口（笛卡尔）**
   - 目标：控制 4 电机到指定末端位置（即笛卡尔模式）。
   - 命令：`pose` / `pose_seq` / `pose_delta`

## 4. 命令结构

### 4.1 通用请求体

| 字段 | 类型 | 必选 | 说明 |
|---|---|---:|---|
| `cmd` | string | 否 | 命令名，与 `type` 二选一 |
| `type` | string | 否 | 命令名别名，与 `cmd` 二选一 |

约束：

- 命令名大小写不敏感。
- `cmd/type` 都缺失：`missing_field: 缺少字段 cmd 或 type`。

### 4.2 关节信息接口（4电机）：`joints`

别名：`joint`、`axes`、`set_joints`

| 字段 | 类型 | 必选 | 单位 | 约束 |
|---|---|---:|---|---|
| `axes_rel_deg` | array[number] | 是 | deg | 长度必须为 4 |

语义：直接更新 J1~J4 的相对标定角（绝对目标）。

错误：

- 长度非法：`invalid_length: axes_rel_deg 必须长度 4`。

### 4.3 关节增量接口（4电机）：`joints_delta`

别名：`delta_joints`、`axes_delta`

| 字段 | 类型 | 必选 | 单位 | 约束 |
|---|---|---:|---|---|
| `deltas_rel_deg` | array[number] | 是 | deg | 长度必须为 4 |

语义：仅对 J1~J4 做增量控制。

错误：

- 长度非法：`invalid_length: deltas_rel_deg 必须长度 4`。

### 4.4 舵机信息接口：`claw`

别名：`wrist`、`gripper`

输入要求（必须同时提供）：

1. `wrist_deg`（手腕角度，单位度）
2. 夹爪状态（`grip_state` 或 `open_close`），最终归一到 `0/1`（**`0=open`，`1=close`**，与 `docs/bridge2pi.md` 一致）

映射到 bridge->pi 帧：

- `wrist_deg -> p_rel_deg[4]`（仅角度）
- `grip_state/open_close -> p_rel_deg[5]`（仅状态：**`0=open`，`1=close`**）

约束：

- 缺字段：`missing_field: claw 需要 wrist_deg + (grip_state/open_close)`。

### 4.5 位置控制接口（笛卡尔）：IK 双模式

笛卡尔命令共用字段（`pose` / `pose_seq`）：

| 字段 | 类型 | 必选 | 单位 |
|---|---|---:|---|
| `x` | number | 是 | m |
| `y` | number | 是 | m |
| `z` | number | 是 | m |

坐标系与 `frontend_pose_to_internal_m` 一致（与 head `Position`、相机 `robot_base` 对齐）。

#### 模式 1：`pose`（单帧 IK，默认）

别名：`set_pose`、`xyz` · HTTP：`POST /api/pose`

语义：bridge 对目标 `(x,y,z)` 做一次 IK，将**最终** J1~J4 相对标定角写入状态，每控制周期向 Pi 下发**当前目标**（由 Pi 侧 ramp 跟踪）。

特点：

- 上位机一次 POST 即可，HTTP 阻塞至 Pi/反馈判到位。
- 延迟低、实现简单。
- j2 / j3 / j4 在 Pi 上各自 ramp，**运动中末端可能无法保持竖直**（几何上 j4 应由 j2+j3 约束）。

适用：空闲待命、大位移粗调、对工具竖直要求不高的段。

#### 模式 2：`pose_seq`（25Hz 关节序列）

别名：字段同 `pose` · HTTP：`POST /api/pose_seq`

语义：bridge 对目标 IK 后，在 PC 侧从**当前** `q_full` 线性插值到 IK 目标，按 `CONFIG.control_hz`（默认 **25Hz**）生成多帧；**每一帧**用 j2+j3 重算 j4（`Q4_OFFSET + coeff*(q2+q3)`），再经既有 UDP 链路下发。牺牲轨迹平滑度，换取 j2/j3/j4 **同步**、末端执行器在运动中尽量保持竖直。

特点：

- 序列播完前 `GET /api/reached` 与阻塞 POST 的 `reached` 均为 `false`（`reach_reason: pose_seq_playing`）。
- 新 `pose` / `pose_seq` / `joints` 等运动会**清空**未播完的序列（抢占）。
- 帧数由当前四轴速度上限与位移估算（`arm_speed_rad_s` × `control_dt`），至少 1 帧。

适用：抓取/放置接近工作点、悬停对准等需要**竖直工具**的段。

#### 对比（选型）

| 项目 | `pose` | `pose_seq` |
|---|---|---|
| 下发内容 | 单帧目标角 | 25Hz 插值序列 |
| j4 与 j2/j3 | Pi 分轴 ramp，易不同步 | 每帧几何约束重算 |
| 到位 HTTP | 一次阻塞至到位 | 一次阻塞至序列+到位 |
| 响应字段 | `actual_pose` 等 | 同左 |

#### `pose_delta`（增量，走模式 1）

别名：`delta_pose`、`nudge` · HTTP：`POST /api/pose_delta`

| 字段 | 类型 | 必选 | 单位 |
|---|---|---:|---|
| `dx` | number | 是 | m |
| `dy` | number | 是 | m |
| `dz` | number | 是 | m |

语义：在当前 `pose_xyz` 目标上叠加增量后，按 **`pose` 单帧 IK** 下发（非序列）。抓取/放置后的**上抬**等短位移常用此命令。

**到达确认（v2.2）**：`pose` / `pose_seq` / `pose_delta` 成功且判到位时，响应须含 `actual_pose`（§5）。

### 4.6 辅机接口：`stepper` / `conveyor`

`stepper`（步进增量，deg）

| 字段 | 类型 | 必选 | 说明 |
|---|---|---:|---|
| `stepper_deg` | number | 是 | 限幅 `[-180, 180]`；写入下一 UDP 周期的 `p_rel_deg[6]`，发出后 bridge 自动清零 |

`conveyor`（传送带启停）

| 字段 | 类型 | 必选 | 说明 |
|---|---|---:|---|
| `run` 或 `conveyor_run` | number | 是 | `0=停`，`1=转`；锁存至 `p_rel_deg[7]` |

HTTP：`POST /api/stepper`、`POST /api/conveyor`。

### 4.7 速度控制接口：`speed`

别名：`set_speed`

| 字段 | 类型 | 必选 | 单位 | 约束 |
|---|---|---:|---|---|
| `axes_rad_s` | array[number] | 是 | rad/s | 长度必须为 4，各轴 >= 0 |

语义：设置 Pi 侧四轴 ramp 速度上限，通过 UDP `omega_rad_s[0:4]` 每帧携带下发。

生效规则（Pi 侧）：
- `axes_rad_s` 任一轴 **> 0**：Pi 用 UDP 值覆盖 `config.max_cmd_speed_rad_s`，立即生效。
- `axes_rad_s` 全为 **0**：Pi 回退到自身 `config.max_cmd_speed_rad_s` 默认值。
- 值持续锁存，直到下一条 `speed` 命令更新。

HTTP：`POST /api/speed`。立即 ack（`{"ok": true}`），不阻塞等待到位。

错误：
- 长度非法：`invalid_length: axes_rad_s 必须长度 4`。
- 值非法：`invalid_value: axes_rad_s 各轴必须 >= 0`。

### 4.8 其他命令

- `stop`（别名 `estop/halt`）：当前仅记录，不执行硬停。
- `ping`：连通性检查。

## 5. 请求与响应

### 5.1 到达反馈字段（笛卡尔，`pose` / `pose_seq` / `pose_delta`）

上位机通过下列字段识别“已到达且带回实测位姿”（推荐字段名为 `actual_pose`；若实现使用等价别名，应在集成说明中列出映射）：

| 字段 | 类型 | 出现时机 | 说明 |
|---|---|---|---|
| `actual_pose` | object | `ok: true` 且笛卡尔运动完成 | 实测末端位置，`x`/`y`/`z`，单位 **m**，与 §4.5 命令坐标系一致 |
| `reached` | boolean | 可选 | `true` 表示本次判定已到达目标（可与容差策略配合） |
| `error_pose_m` | number | 可选 | 末端位置误差范数（m），便于上位机记录或告警 |

约定：

- **仅**对 `pose`、`pose_seq`、`pose_delta`（及 HTTP 同源路由）要求在上行成功路径中返回 `actual_pose`；其它命令（`joints`、`claw` 等）成功时仍可仅为 `{"ok": true}`，除非另有扩展约定。
- 若运动失败、超时或不可达，应 `ok: false` 并在 `error` 中说明，不强制携带 `actual_pose`。

### 5.2 TCP

- 输入：每行一条 JSON。
- 输出（v2.2）：对 `pose` / `pose_seq` / `pose_delta` 等与运动相关的命令，在动作完成并判定到位后，**输出一行 JSON**（UTF-8，`\n` 结尾），语义与 §5.3 HTTP 成功体一致，便于 head 同步读取“已到达 + 实际位姿”。非运动类命令或仅日志路径可由实现决定是否与 HTTP 对齐。

示例（到位后下行一行）：

```json
{"ok": true, "reached": true, "actual_pose": {"x": 0.349, "y": 0.201, "z": 0.248}}
```

### 5.3 HTTP

路由映射：

- `POST /api/pose` -> `cmd=pose`
- `POST /api/pose_seq` -> `cmd=pose_seq`
- `POST /api/pose_delta` -> `cmd=pose_delta`
- `POST /api/joints` -> `cmd=joints`
- `POST /api/joints_delta` -> `cmd=joints_delta`
- `POST /api/claw` -> `cmd=claw`
- `POST /api/stepper` -> `cmd=stepper`
- `POST /api/conveyor` -> `cmd=conveyor`

**笛卡尔**（`pose` / `pose_seq` / `pose_delta`）成功且已到达时，响应体示例：

```json
{
  "ok": true,
  "reached": true,
  "actual_pose": {"x": 0.349, "y": 0.201, "z": 0.248},
  "error_pose_m": 0.002
}
```

**其它命令**成功（无位姿回传要求时）：

```json
{"ok": true}
```

失败：

```json
{"ok": false, "error": "错误描述"}
```

## 6. 错误模型

| 场景 | HTTP | 错误字符串示例 |
|---|---:|---|
| JSON 解析失败 | 400 | `invalid json` |
| 缺少 `cmd/type` | 400 | `missing_field: 缺少字段 cmd 或 type` |
| 数组长度非法 | 400 | `invalid_length: axes_rel_deg 必须长度 4` |
| 第 6 维非法 | 400 | `invalid_value: grip_state 必须为 0/1` |
| 未知命令 | 400 | `unknown_cmd: xxx` |

说明：TCP 路径中的错误会记录并忽略当前命令，不中断服务。

## 7. 最小联调示例

HTTP（端口以 `CONFIG.web_test_port` 为准，默认 `8877`）：

```bash
# 模式 1：单帧 pose
curl -sS -X POST http://127.0.0.1:8877/api/pose \
  -H 'Content-Type: application/json' \
  -d '{"cmd":"pose","x":0.35,"y":0.20,"z":0.25}'

# 模式 2：25Hz 序列 pose_seq
curl -sS -X POST http://127.0.0.1:8877/api/pose_seq \
  -H 'Content-Type: application/json' \
  -d '{"cmd":"pose_seq","x":0.35,"y":0.20,"z":0.25}'
```

TCP（每行一条 JSON，`9888`）：

```json
{"cmd":"joints","axes_rel_deg":[0,10,-90,-70]}
{"cmd":"joints_delta","deltas_rel_deg":[0,0,2,-1]}
{"cmd":"claw","wrist_deg":20,"open_close":"close"}
{"cmd":"stepper","stepper_deg":15}
{"cmd":"conveyor","run":1}
{"cmd":"pose","x":0.35,"y":0.20,"z":0.25}
{"cmd":"pose_seq","x":0.35,"y":0.20,"z":0.25}
{"cmd":"pose_delta","dx":0,"dy":0,"dz":0.10}
{"cmd":"speed","axes_rad_s":[0.5,0.4,0.5,0.5]}
```

## 8. head（icecream）集成：如何调用 IK 双模式

实现位置（icecream 仓库）：

| 层级 | 模块 | 说明 |
|---|---|---|
| HTTP 封装 | `head/src/speaker.py` → `BridgeClient` | `send_pose` / `send_pose_seq` / `send_pose_delta` |
| FSM | `head/src/run.py` → `HeadFSM._pose_to` | 当前默认 `send_pose`；竖直段可改为 `send_pose_seq` |
| 配置 | `head/config.yaml` → `bridge_base_url` | 须与 bridge HTTP 端口一致 |

### 8.1 `BridgeClient` API（推荐）

两种模式**接口相同**：`(x, y, z, context=...)`，阻塞至 bridge 判到位，并校验 `actual_pose`（`require_bridge_feedback` 时还需 Pi UDP 反馈）。

```python
# 模式 1：单帧（现状默认）
self.speaker.send_pose(pos.x, pos.y, pos.z, context="step6_object_pose")

# 模式 2：25Hz 序列，j4 随 j2/j3
self.speaker.send_pose_seq(pos.x, pos.y, pos.z, context="step6_object_pose_vertical")

# 增量上抬（内部仍走 pose 单帧 IK，不是 pose_seq）
self.speaker.send_pose_delta(0.0, 0.0, 0.10, context="step8_retreat_lift")
```

`context` 仅用于 head 日志，**不会**传给 bridge。

### 8.2 FSM 选型建议（v3 十二步）

| 步骤 / 场景 | 推荐命令 | 理由 |
|---|---|---|
| step6 物体抓取位 `pose` | **`pose_seq`** | 接近物体需竖直工具 |
| step8 / 11b 抓取/放置后上抬 | **`pose_delta`** | 短距离 Z 向，单帧即可 |
| step10 放置工作高度 `pose` | **`pose_seq`** | 对准放置点需竖直 |
| step10 悬停重观测 `_goto_hover` | **`pose_seq`** 或 `pose` | 竖直要求高时用 `pose_seq` |
| obs 间 `joints`、idle `joints` | 不涉及笛卡尔 IK | — |

当前 `HeadFSM._pose_to()` 固定调用 `send_pose`。要在 FSM 中启用序列模式，任选其一：

1. **按步骤改调用**（最小改动）：在 step6 / step10 / 悬停等处直接调用 `send_pose_seq`，其余保持 `send_pose`。
2. **统一入口加参数**（便于配置）：

```python
def _pose_to(self, pos: Position, label: str, *, role: Role, vertical: bool = False) -> None:
    fn = self.speaker.send_pose_seq if vertical else self.speaker.send_pose
    fn(pos.x, pos.y, pos.z, context=label)
```

抓取/放置步：`self._pose_to(..., vertical=True)`；上抬仍用 `_pose_delta_to` → `send_pose_delta`。

### 8.3 联调顺序

1. 启动 `arm_control_bridge`（`web_test_port > 0`，默认 HTTP `8877`）。
2. `head/config.yaml` 中 `bridge_base_url` 指向同一地址。
3. 先用 §7 `curl` 验证 `pose_seq`，再改 FSM 或 `python -c` 调用 `BridgeClient.send_pose_seq`。
4. 完整流程：`python -m src.run` + 相机 ingestion + pi2head `start`。

详见 icecream 仓库 `head/doc/head2bridge.md` §10 与 `head/README.md`。

## 9. 迁移说明（旧版 -> 新版）

- 旧版仅明确到 4/5 维。
- 新版统一到 6 维语义，新增第 6 维 `grip_state`。
- 若上位机仍发送旧版 4/5 维，bridge 应按兼容规则工作（未提供的维度保持上次值）。
- **v2.1 → v2.2**：`pose` / `pose_delta`（及 `POST /api/pose`、`/api/pose_delta`）在成功且判定到位时，响应体须包含 `actual_pose`（及可选 `reached`、`error_pose_m`）；TCP 同命令在到位后应输出一行与 HTTP 成功体语义一致的 JSON，供 head 同步确认已到达与实际末端坐标。仅依赖 `{"ok": true}` 判断笛卡尔到位的上位机应改为解析 `actual_pose`（或约定别名）。
- **v2.2 → v2.3（IK 序列）**：新增 `pose_seq` / `POST /api/pose_seq`；head 通过 `BridgeClient.send_pose_seq` 调用，与 `send_pose` 二选一，见 §8。

## 10. 交叉引用

- 下游协议：`docs/bridge2pi.md`
- 相机链路协议：`docs/pi2camera.md`
- head 侧同源说明：icecream 仓库 `head/doc/head2bridge.md`
- 串口链路协议：`docs/pi2stm.md`
