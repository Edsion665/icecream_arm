# head2bridge API 规范（arm_control_bridge ie v1）

- 文档状态：stable
- 适用模块：`arm_control_bridge/listener.py`、`arm_control_bridge/calculator.py`
- 对端：上层应用（UI/策略/脚本）-> `arm_control_bridge`
- 相关文档：`doc/bridge2pi.md`

## 1. 范围（Scope）

本文档定义上层到 `arm_control_bridge` 控制桥的输入接口，包括：

- TCP JSON 行协议
- HTTP JSON API
- 命令字段、错误语义、兼容规则

不包含：

- bridge 到树莓派 UDP 二进制帧（见 `doc/bridge2pi.md`）
- Pi 端电机控制内部实现细节

## 2. 传输与端点

### 2.1 TCP（JSON 行）

- 默认监听：`0.0.0.0:9888`
- 编码：UTF-8
- 边界：每条命令一行 JSON（`\n` 分隔）

### 2.2 HTTP（JSON）

- 默认服务：`127.0.0.1:8765`
- 仅在 `--web-port > 0` 时启用
- `Content-Type: application/json`
- CORS：`Access-Control-Allow-Origin: *`

## 3. 数据结构

### 3.1 通用请求体

| 字段 | 类型 | 必选 | 说明 |
|---|---|---:|---|
| `cmd` | string | 否 | 命令名，与 `type` 二选一 |
| `type` | string | 否 | 命令名别名，与 `cmd` 二选一 |

约束：

- 命令名大小写不敏感，内部统一转小写。
- `cmd/type` 都缺失时报错：`missing_field: 缺少字段 cmd 或 type`。

### 3.2 命令定义

#### `pose`（别名：`set_pose`、`xyz`）

| 字段 | 类型 | 必选 | 单位 | 约束 |
|---|---|---:|---|---|
| `x` | number | 是 | m | 任意实数 |
| `y` | number | 是 | m | 任意实数 |
| `z` | number | 是 | m | 任意实数 |

语义：基座系下 `link4` 原点绝对目标。  
缺字段错误：`missing_field: pose 缺少 x|y|z`。

#### `pose_delta`（别名：`delta_pose`、`nudge`）

| 字段 | 类型 | 必选 | 单位 | 约束 |
|---|---|---:|---|---|
| `dx` | number | 是 | m | 任意实数 |
| `dy` | number | 是 | m | 任意实数 |
| `dz` | number | 是 | m | 任意实数 |

语义：在当前 `pose_xyz` 基础上做增量。  
缺字段错误：`missing_field: pose_delta 需要 dx,dy,dz`。

#### `joints`（别名：`joint`、`axes`、`set_joints`）

| 字段 | 类型 | 必选 | 单位 | 约束 |
|---|---|---:|---|---|
| `axes_rel_deg` | array[number] | 是 | deg | 长度 4 或 5 |

语义：相对标定零位的绝对目标角。  
长度为 4：更新 J1~J4；J5 保持当前值。  
长度为 5：更新 J1~J5（第 5 个元素对应 J5）。  
长度错误：`invalid_length: axes_rel_deg 必须长度 4 或 5`。

#### `joints_delta`（别名：`delta_joints`、`axes_delta`）

| 字段 | 类型 | 必选 | 单位 | 约束 |
|---|---|---:|---|---|
| `deltas_rel_deg` | array[number] | 是 | deg | 长度 4 或 5 |

语义：在当前关节目标上做增量。  
长度为 4：仅对 J1~J4 做增量；J5 保持当前值。  
长度为 5：对 J1~J5 做增量（第 5 个元素对应 J5 增量）。  
长度错误：`invalid_length: deltas_rel_deg 必须长度 4 或 5`。

补充：在 `pose`（笛卡尔）模式下，J5 使用当前内部目标值；当收到包含 J5 的 `joints/joints_delta` 后，该值会被覆盖并持续生效，未收到新的 J5 输入时保持不变。

#### `claw`（别名：`wrist`、`gripper`）

可选输入组合（满足其一即可）：

1. `wrist_deg` + `grip`
2. `servo_deg=[wrist_deg, grip]`（长度 >= 2）

缺字段错误：`missing_field: claw 需要 wrist_deg/grip/servo_deg`。

#### 其他命令

- `stop`（别名：`estop`、`halt`）：当前实现仅记录，不执行硬停。
- `ping`：连通性检查。

## 4. 请求与响应

### 4.1 TCP

- 输入：每行一条 JSON。
- 输出：无协议级响应（仅日志）。

### 4.2 HTTP

路由映射：

- `POST /api/pose` -> `cmd=pose`
- `POST /api/pose_delta` -> `cmd=pose_delta`
- `POST /api/joints` -> `cmd=joints`
- `POST /api/joints_delta` -> `cmd=joints_delta`
- `POST /api/claw` -> `cmd=claw`

成功响应：

```json
{"ok": true}
```

失败响应：

```json
{"ok": false, "error": "错误描述"}
```

## 5. 错误模型

| 场景 | HTTP | 错误字符串示例 |
|---|---:|---|
| JSON 解析失败 | 400 | `invalid json` |
| 缺少 `cmd/type` | 400 | `missing_field: 缺少字段 cmd 或 type` |
| 参数缺失 | 400 | `missing_field: pose_delta 需要 dx,dy,dz` |
| 数组长度非法 | 400 | `invalid_length: axes_rel_deg 必须长度 4 或 5` |
| 未知命令 | 400 | `unknown_cmd: xxx` |
| 路由不存在 | 404 | `{"ok": false}` |

说明：TCP 路径中的错误会记录日志并忽略当前命令，不中断服务。

## 6. 兼容性与版本演进

- 兼容旧命令别名：`set_pose`、`delta_pose`、`set_joints` 等。
- 兼容 5 轴数组输入；当提供第 5 轴时会写入 J5 目标。
- `stop` 预留为后续安全态扩展点。

## 7. 最小联调示例

### 7.1 TCP 示例

```json
{"cmd":"pose","x":0.35,"y":0.20,"z":0.25}
{"cmd":"joints","axes_rel_deg":[0,10,-90,-70]}
{"cmd":"claw","wrist_deg":20,"grip":0.7}
```

### 7.2 HTTP 示例

```bash
curl -X POST "http://127.0.0.1:8765/api/joints" \
  -H "Content-Type: application/json" \
  -d '{"axes_rel_deg":[0,10,-90,-70]}'
```

## 8. 交叉引用

- 下游协议：`doc/bridge2pi.md`
- 关键代码：
  - `arm_control_bridge/listener.py`
  - `arm_control_bridge/calculator.py`
