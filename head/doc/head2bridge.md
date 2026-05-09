# head2bridge API 规范（icecream 上位机 -> bridge，v1.0 / 与 arm v2.2 同形）

- 文档状态：stable
- 适用场景：icecream **head**（UI / 策略 / 脚本）向 **bridge**（如 `nosim_bridge`、`arm_control_bridge` 同类 listener）下达机械臂控制指令
- 对端：bridge 进程（TCP/HTTP 监听）
- 相关文档：本仓库 [`camera2head.md`](camera2head.md)；与 isaac-sim 侧 [`arm_control_bridge/doc/head2bridge.md`](../../../isaac-sim/arm_control_bridge/doc/head2bridge.md) 为**同源线格式**（便于同一套 curl 与字段语义）

## 1. 范围（Scope）

本文档定义上位机与 bridge 之间的**控制与响应**约定，包括：

- TCP JSON 行协议（上行命令；笛卡尔命令支持下行到位反馈，与 arm v2.2 一致）
- HTTP JSON API（`pose` / `pose_delta` 成功时可含实际位姿回传）
- 6 维控制语义（4 电机 + 手腕 + 夹爪状态）

不包含：

- bridge 到下游设备的二进制 UDP / 串口细节（见各 bridge 仓库内 `bridge2pi` 等文档）

## 2. 传输与端点

**默认示例端口**（与 isaac `9888` / `8765` 错开，便于同机多服务；生产环境以启动参数为准）：

### 2.1 TCP（JSON 行）

- 默认监听：`0.0.0.0:9898`
- 编码：UTF-8
- 每行一条 JSON（`\n` 分隔）

### 2.2 HTTP（JSON）

- 默认服务：`127.0.0.1:8775`
- 启用条件：由实现决定（例如 `--web-port > 0` 时监听 `8775`）
- `Content-Type: application/json`

## 3. 接口总览

1. **关节信息接口（4 电机）**：`joints` / `joints_delta`
2. **舵机信息接口（手腕 + 夹爪）**：`claw`
3. **位置控制接口（笛卡尔）**：`pose` / `pose_delta`
4. **其它**：`stop`（别名 `estop`/`halt`）、`ping`

## 4. 命令结构

### 4.1 通用请求体

| 字段 | 类型 | 必选 | 说明 |
|---|---|---:|---|
| `cmd` | string | 否 | 命令名，与 `type` 二选一 |
| `type` | string | 否 | 命令名别名，与 `cmd` 二选一 |

约束：

- 命令名大小写不敏感。
- `cmd`/`type` 都缺失：`missing_field: 缺少字段 cmd 或 type`。

### 4.2 关节信息接口：`joints`

别名：`joint`、`axes`、`set_joints`

| 字段 | 类型 | 必选 | 单位 | 约束 |
|---|---|---:|---|---|
| `axes_rel_deg` | array[number] | 是 | deg | 长度必须为 4 |

语义：直接更新 J1~J4 的相对标定角（绝对目标）。

错误：长度非法 → `invalid_length: axes_rel_deg 必须长度 4`。

### 4.3 关节增量接口：`joints_delta`

别名：`delta_joints`、`axes_delta`

| 字段 | 类型 | 必选 | 单位 | 约束 |
|---|---|---:|---|---|
| `deltas_rel_deg` | array[number] | 是 | deg | 长度必须为 4 |

错误：长度非法 → `invalid_length: deltas_rel_deg 必须长度 4`。

### 4.4 舵机信息接口：`claw`

别名：`wrist`、`gripper`

须同时提供：

1. `wrist_deg`（手腕角度，单位度）
2. 夹爪：`grip_state` 或 `open_close`，归一到 `0/1`（**`0=合拢`，`1=张开`**）

缺字段：`missing_field: claw 需要 wrist_deg + (grip_state/open_close)`。

### 4.5 位置控制接口：`pose` / `pose_delta`

`pose`（别名：`set_pose`、`xyz`）

| 字段 | 类型 | 必选 | 单位 |
|---|---|---:|---|
| `x` | number | 是 | m |
| `y` | number | 是 | m |
| `z` | number | 是 | m |

`pose_delta`（别名：`delta_pose`、`nudge`）

| 字段 | 类型 | 必选 | 单位 |
|---|---|---:|---|
| `dx` | number | 是 | m |
| `dy` | number | 是 | m |
| `dz` | number | 是 | m |

**到达确认（与 arm v2.2 一致）**：`pose` / `pose_delta` 成功且判定到位时，响应须携带 `actual_pose`（及可选 `reached`、`error_pose_m`）。详见 §5。

### 4.6 其他命令

- `stop`：实现可仅记录，不保证硬停。
- `ping`：连通性检查。

## 5. 请求与响应

### 5.1 到达反馈字段（笛卡尔）

| 字段 | 类型 | 出现时机 | 说明 |
|---|---|---|---|
| `actual_pose` | object | `ok: true` 且笛卡尔运动完成 | `x`/`y`/`z`，单位 **m** |
| `reached` | boolean | 可选 | 判定已到达目标 |
| `error_pose_m` | number | 可选 | 位置误差范数（m） |

约定：`pose` / `pose_delta` 成功路径应返回 `actual_pose`；失败为 `ok: false` 与 `error`。

### 5.2 TCP

- 输入：每行一条 JSON。
- 输出：对 `pose` / `pose_delta`，到位后可下行一行 JSON，语义与 HTTP 成功体一致。

### 5.3 HTTP

路由映射：

- `POST /api/pose` → `cmd=pose`
- `POST /api/pose_delta` → `cmd=pose_delta`
- `POST /api/joints` → `cmd=joints`
- `POST /api/joints_delta` → `cmd=joints_delta`
- `POST /api/claw` → `cmd=claw`

笛卡尔成功示例：

```json
{
  "ok": true,
  "reached": true,
  "actual_pose": {"x": 0.349, "y": 0.201, "z": 0.248},
  "error_pose_m": 0.002
}
```

其它命令成功：`{"ok": true}`

失败：`{"ok": false, "error": "错误描述"}`

## 6. 错误模型

| 场景 | HTTP | 错误字符串示例 |
|---|---:|---|
| JSON 解析失败 | 400 | `invalid json` |
| 缺少 `cmd/type` | 400 | `missing_field: 缺少字段 cmd 或 type` |
| 数组长度非法 | 400 | `invalid_length: axes_rel_deg 必须长度 4` |
| 夹爪值非法 | 400 | `invalid_value: grip_state 必须为 0/1` |
| 未知命令 | 400 | `unknown_cmd: xxx` |

说明：TCP 路径中的错误可记录并丢弃当前行，不中断服务。

## 7. curl 联调示例

以下假定 HTTP 监听 `127.0.0.1:8775`（按实际端口替换）。

```bash
curl -sS -X POST http://127.0.0.1:8775/api/joints \
  -H 'Content-Type: application/json' \
  -d '{"cmd":"joints","axes_rel_deg":[0,10,-90,-70]}'
```

```bash
curl -sS -X POST http://127.0.0.1:8775/api/joints_delta \
  -H 'Content-Type: application/json' \
  -d '{"cmd":"joints_delta","deltas_rel_deg":[0,0,2,-1]}'
```

```bash
curl -sS -X POST http://127.0.0.1:8775/api/claw \
  -H 'Content-Type: application/json' \
  -d '{"cmd":"claw","wrist_deg":20,"open_close":"close"}'
```

```bash
curl -sS -X POST http://127.0.0.1:8775/api/pose \
  -H 'Content-Type: application/json' \
  -d '{"cmd":"pose","x":0.35,"y":0.20,"z":0.25}'
```

```bash
curl -sS -X POST http://127.0.0.1:8775/api/pose_delta \
  -H 'Content-Type: application/json' \
  -d '{"cmd":"pose_delta","dx":0,"dy":0,"dz":-0.01}'
```

## 8. TCP 最小联调（每行一条 JSON）

```json
{"cmd":"joints","axes_rel_deg":[0,10,-90,-70]}
{"cmd":"joints_delta","deltas_rel_deg":[0,0,2,-1]}
{"cmd":"claw","wrist_deg":20,"open_close":"close"}
{"cmd":"pose","x":0.35,"y":0.20,"z":0.25}
```

## 9. 交叉引用

- 感知上报（相机 → head）：[`camera2head.md`](camera2head.md)
- isaac-sim 同源规范：仓库根下 `isaac-sim/arm_control_bridge/doc/head2bridge.md`（与上文相对链接一致）
