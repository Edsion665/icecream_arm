# pi2head API 规范（树莓派 -> icecream head，v1.0）

- 文档状态：stable
- 适用场景：树莓派（或现场主控）通过 **TCP JSON 行** 通知 **head** 何时结束「待命」、开始执行既有 v3 状态机（observe → plan → pick → place）
- 对端：**head** 进程内 `src/pi2head_listener.py` 监听；树莓派为 **TCP 客户端**
- 相关文档：[`head2bridge.md`](head2bridge.md)（待命期间 head 向 bridge 下发的全零指令）、[`camera2head.md`](camera2head.md)（相机感知，与 pi2head **端口分离**）

## 1. 范围（Scope）

- 定义树莓派 → head 的**控制信令**（当前仅 `start`、`ping`）。
- 定义 head **启动后、收到 `start` 之前**向 bridge 的待命行为（周期下发全零关节 + claw）。
- **不包含**：相机检测载荷（见 `camera2head.md`）；不包含 head → bridge 的完整运动语义（见 `head2bridge.md`）。

## 2. 传输与端点

### 2.1 TCP（JSON 行）

| 项 | 默认值（`config.yaml`） | 说明 |
|---|---|---|
| 监听地址 | `pi2head_host: 0.0.0.0` | head 绑定 |
| 端口 | `pi2head_tcp_port: 8778` | 与 ingestion `8776`、bridge HTTP `8877` 错开 |
| 编码 | UTF-8 | |
| 帧格式 | 一行一条 JSON，以 `\n` 结束 | 与 `camera2head` / `head2bridge` 线协议一致 |

树莓派侧作为客户端连接：`tcp://<head_ip>:8778`，发送命令行，读取一行响应。

### 2.2 HTTP

本版本 **不提供** pi2head 的 HTTP 路由；仅 TCP。

## 3. 接口总览

| 命令 `cmd`（`type` 等价） | 说明 |
|---|---|
| `start` | 放行 head：结束待命循环，进入 v3 FSM（`src/run.py` `run_forever`） |
| `ping` | 连通性探测，不改变 FSM 状态 |

命令名大小写不敏感。`cmd` 与 `type` 二选一，均缺失返回错误。

## 4. 请求与响应

### 4.1 通用请求体

| 字段 | 类型 | 必选 | 说明 |
|---|---|---:|---|
| `cmd` | string | 否 | 与 `type` 二选一 |
| `type` | string | 否 | 与 `cmd` 二选一 |

### 4.2 `start`

**请求示例（树莓派 → head）：**

```json
{"cmd":"start"}
```

**成功响应（head → 树莓派，单行 JSON + `\n`）：**

```json
{"ok": true, "started": true, "already_started": false}
```

- 首次 `start`：`already_started: false`，head 从待命进入 FSM。
- 重复 `start`：`already_started: true`，仍返回 `ok: true`，FSM 已在运行则保持运行（不重置周期）。

**错误响应示例：**

```json
{"ok": false, "error": "unknown_cmd: foo"}
```

### 4.3 `ping`

**请求：**

```json
{"cmd":"ping"}
```

**响应：**

```json
{"ok": true, "pong": true}
```

## 5. head 待命阶段（收到 `start` 之前）

head 启动顺序（`src/run.py`）：

1. 启动 tracker、相机 ingestion（HTTP/TCP）、**pi2head TCP**。
2. **待命循环**（阻塞 FSM）：
   - 以 `idle_bridge_hz`（默认 **2 Hz**）向 `bridge_base_url` 发送：
     - `POST /api/joints`：`axes_rel_deg` = `idle_axes_rel_deg`（默认 **`[0, 0, 0, 0]`** deg）
     - `POST /api/claw`：`wrist_deg` = `idle_claw_wrist_deg`（默认 **0**），`grip_state` = `idle_grip_state`（默认 **0**，合拢）
   - 不等待 bridge 到位，仅尽力 POST；失败打日志并下一周期重试。
3. pi2head 收到 **`start`** 后退出待命，执行原有 **v3 十二步 FSM**（observe1 → … → place）。

配置项（`config.yaml`）：

| 字段 | 默认 | 说明 |
|---|---|---|
| `pi2head_host` | `0.0.0.0` | pi2head 监听 |
| `pi2head_tcp_port` | `8778` | pi2head 端口 |
| `idle_axes_rel_deg` | `[0,0,0,0]` | 待命关节相对标定角（deg） |
| `idle_bridge_hz` | `2.0` | 待命下发频率（Hz） |
| `idle_claw_wrist_deg` | `0.0` | 待命腕角（deg） |
| `idle_grip_state` | `0` | `0` 合拢 / `1` 张开（与 head2bridge 一致） |

## 6. 树莓派侧示例

```python
import json
import socket

def pi_send_start(head_ip: str, port: int = 8778) -> dict:
    msg = json.dumps({"cmd": "start"}) + "\n"
    with socket.create_connection((head_ip, port), timeout=5.0) as s:
        s.sendall(msg.encode("utf-8"))
        line = b""
        while b"\n" not in line:
            chunk = s.recv(4096)
            if not chunk:
                break
            line += chunk
        return json.loads(line.decode("utf-8").strip())

# rep = pi_send_start("192.168.31.100")
# assert rep.get("ok") is True
```

```bash
# 使用 nc 探测 ping
printf '%s\n' '{"cmd":"ping"}' | nc -q 2 192.168.31.100 8778
```

## 7. 与相机 ingestion 的区分

| 服务 | 默认端口 | 协议文档 | 载荷 |
|---|---|---|---|
| 相机 ingestion | `8776`（HTTP） | `camera2head.md` | `detection` / `objects[]` |
| pi2head | `8778`（TCP） | 本文档 | `start` / `ping` |

请勿将 `start` 发往相机端口，或将检测帧发往 pi2head 端口。

## 8. 版本

- 当前版本：`pi2head v1.0`
