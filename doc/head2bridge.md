# head2bridge API 规范（上位机 -> arm_control_bridge，v2.2）

- 文档状态：stable
- 适用模块：`arm_control_bridge/listener.py`、`arm_control_bridge/calculator.py`
- 对端：上位机（UI/策略/脚本）-> `arm_control_bridge`
- 相关文档：`docs/bridge2pi.md`

## 1. 范围（Scope）

本文档定义上位机与 bridge 之间的**控制与响应**约定，包括：

- TCP JSON 行协议（上行命令；v2.2 起笛卡尔命令支持下行到位反馈）
- HTTP JSON API（含 `pose` / `pose_delta` 成功时的实际位姿回传）
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

- 默认服务：`127.0.0.1:8765`
- 仅在 `--web-port > 0` 时启用
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
   - 命令：`pose` / `pose_delta`

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
2. 夹爪状态（`grip_state` 或 `open_close`），最终归一到 `0/1`

映射到 bridge->pi 帧：

- `wrist_deg -> p_rel_deg[4]`（仅角度）
- `grip_state/open_close -> p_rel_deg[5]`（仅状态：`0=open, 1=close`）

约束：

- 缺字段：`missing_field: claw 需要 wrist_deg + (grip_state/open_close)`。

### 4.5 位置控制接口（笛卡尔）：`pose` / `pose_delta`

`pose`（别名：`set_pose`、`xyz`）

| 字段 | 类型 | 必选 | 单位 |
|---|---|---:|---|
| `x` | number | 是 | m |
| `y` | number | 是 | m |
| `z` | number | 是 | m |

语义：控制 4 电机进入笛卡尔模式，驱动末端（link4）到指定位置。

**到达确认（v2.2）**：运动结束且判定已到达目标工作区后，bridge 须在响应中携带**实际末端位姿**（与命令同一坐标系、单位 m），供上位机确认“已到位”。详见 §5。

`pose_delta`（别名：`delta_pose`、`nudge`）

| 字段 | 类型 | 必选 | 单位 |
|---|---|---:|---|
| `dx` | number | 是 | m |
| `dy` | number | 是 | m |
| `dz` | number | 是 | m |

语义：在当前笛卡尔目标上做增量。

**到达确认（v2.2）**：与 `pose` 相同，到位后响应中返回 `actual_pose`（或等价字段），表示本次增量运动完成后的实测末端位置。

### 4.6 其他命令

- `stop`（别名 `estop/halt`）：当前仅记录，不执行硬停。
- `ping`：连通性检查。

## 5. 请求与响应

### 5.1 到达反馈字段（笛卡尔，`pose` / `pose_delta`）

上位机通过下列字段识别“已到达且带回实测位姿”（推荐字段名为 `actual_pose`；若实现使用等价别名，应在集成说明中列出映射）：

| 字段 | 类型 | 出现时机 | 说明 |
|---|---|---|---|
| `actual_pose` | object | `ok: true` 且笛卡尔运动完成 | 实测末端位置，`x`/`y`/`z`，单位 **m**，与 §4.5 命令坐标系一致 |
| `reached` | boolean | 可选 | `true` 表示本次判定已到达目标（可与容差策略配合） |
| `error_pose_m` | number | 可选 | 末端位置误差范数（m），便于上位机记录或告警 |

约定：

- **仅**对 `pose`、`pose_delta`（及 HTTP 同源路由）要求在上行成功路径中返回 `actual_pose`；其它命令（`joints`、`claw` 等）成功时仍可仅为 `{"ok": true}`，除非另有扩展约定。
- 若运动失败、超时或不可达，应 `ok: false` 并在 `error` 中说明，不强制携带 `actual_pose`。

### 5.2 TCP

- 输入：每行一条 JSON。
- 输出（v2.2）：对 `pose` / `pose_delta` 等与运动相关的命令，在动作完成并判定到位后，**输出一行 JSON**（UTF-8，`\n` 结尾），语义与 §5.3 HTTP 成功体一致，便于 head 同步读取“已到达 + 实际位姿”。非运动类命令或仅日志路径可由实现决定是否与 HTTP 对齐。

示例（到位后下行一行）：

```json
{"ok": true, "reached": true, "actual_pose": {"x": 0.349, "y": 0.201, "z": 0.248}}
```

### 5.3 HTTP

路由映射：

- `POST /api/pose` -> `cmd=pose`
- `POST /api/pose_delta` -> `cmd=pose_delta`
- `POST /api/joints` -> `cmd=joints`
- `POST /api/joints_delta` -> `cmd=joints_delta`
- `POST /api/claw` -> `cmd=claw`

**笛卡尔**（`pose` / `pose_delta`）成功且已到达时，响应体示例：

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

```json
{"cmd":"joints","axes_rel_deg":[0,10,-90,-70]}
{"cmd":"joints_delta","deltas_rel_deg":[0,0,2,-1]}
{"cmd":"claw","wrist_deg":20,"open_close":"close"}
{"cmd":"pose","x":0.35,"y":0.20,"z":0.25}
```

## 8. 迁移说明（旧版 -> 新版）

- 旧版仅明确到 4/5 维。
- 新版统一到 6 维语义，新增第 6 维 `grip_state`。
- 若上位机仍发送旧版 4/5 维，bridge 应按兼容规则工作（未提供的维度保持上次值）。
- **v2.1 → v2.2**：`pose` / `pose_delta`（及 `POST /api/pose`、`/api/pose_delta`）在成功且判定到位时，响应体须包含 `actual_pose`（及可选 `reached`、`error_pose_m`）；TCP 同命令在到位后应输出一行与 HTTP 成功体语义一致的 JSON，供 head 同步确认已到达与实际末端坐标。仅依赖 `{"ok": true}` 判断笛卡尔到位的上位机应改为解析 `actual_pose`（或约定别名）。

## 9. 交叉引用

- 下游协议：`docs/bridge2pi.md`
- 相机链路协议：`docs/pi2camera.md`
- 串口链路协议：`docs/pi2stm.md`
