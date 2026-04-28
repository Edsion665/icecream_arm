# head2bridge API 规范（上位机 -> arm_control_bridge，v2.1）

- 文档状态：stable
- 适用模块：`arm_control_bridge/listener.py`、`arm_control_bridge/calculator.py`
- 对端：上位机（UI/策略/脚本）-> `arm_control_bridge`
- 相关文档：`docs/bridge2pi.md`

## 1. 范围（Scope）

本文档定义上位机到 bridge 的输入接口，包括：

- TCP JSON 行协议
- HTTP JSON API
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

`pose_delta`（别名：`delta_pose`、`nudge`）

| 字段 | 类型 | 必选 | 单位 |
|---|---|---:|---|
| `dx` | number | 是 | m |
| `dy` | number | 是 | m |
| `dz` | number | 是 | m |

语义：在当前笛卡尔目标上做增量。

### 4.6 其他命令

- `stop`（别名 `estop/halt`）：当前仅记录，不执行硬停。
- `ping`：连通性检查。

## 5. 请求与响应

### 5.1 TCP

- 输入：每行一条 JSON。
- 输出：无协议级响应（仅日志）。

### 5.2 HTTP

路由映射：

- `POST /api/pose` -> `cmd=pose`
- `POST /api/pose_delta` -> `cmd=pose_delta`
- `POST /api/joints` -> `cmd=joints`
- `POST /api/joints_delta` -> `cmd=joints_delta`
- `POST /api/claw` -> `cmd=claw`

成功：

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

## 9. 交叉引用

- 下游协议：`docs/bridge2pi.md`
- 相机链路协议：`docs/pi2camera.md`
- 串口链路协议：`docs/pi2stm.md`
