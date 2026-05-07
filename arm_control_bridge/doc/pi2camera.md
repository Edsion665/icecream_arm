# pi2camera API 规范（icecreamPi 仿真广播 v1）

- 文档状态：stable
- 适用模块：`icecreamPi/server.py`、`icecreamPi/infra/state/presenter.py`
- 对端：树莓派（`icecreamPi`）-> 相机/仿真可视化端（WebSocket 客户端）
- 相关文档：`docs/bridge2pi.md`、`docs/head2bridge.md`、`docs/PC_RPI_UDP_PROTOCOL.md`

## 1. 范围（Scope）

本文档定义树莓派到相机/仿真端的状态广播协议，包括：

- WebSocket 传输参数
- 广播消息封装与字段结构
- `link5_hmat`（用于仿真/相机对齐）语义
- 兼容与演进规则

不包含：

- bridge 到 Pi 的 UDP 下发帧（见 `docs/bridge2pi.md`）
- 上层到 bridge 的命令协议（见 `docs/head2bridge.md`）
- 相机侧视觉算法与标定流程

## 2. 传输与端点

- 协议：WebSocket（双向连接，状态单向高频广播）
- 默认监听：`ws://0.0.0.0:8765`
- 配置项：
  - `ARM_CONTROL_WS_HOST`（默认 `0.0.0.0`）
  - `ARM_CONTROL_WS_PORT`（默认 `8765`）
  - `ARM_CONTROL_STATE_PUSH_SEC`（默认 `0.05s`，约 20Hz）
- 广播策略：服务端按固定周期推送最新快照，不做 ACK/重传

## 3. 数据结构

### 3.1 顶层消息封装

每条广播消息为 JSON 对象：

| 字段 | 类型 | 说明 |
|---|---|---|
| `type` | string | 固定为 `state` |
| `data` | object | 状态载荷 |

### 3.2 `data` 载荷结构（当前实现）

| 字段 | 类型 | 说明 |
|---|---|---|
| `calibration_rad` | `number[4]` | 四轴标定零位（rad） |
| `calibration_deg` | `number[4]` | 四轴标定零位（deg） |
| `udp` | object | 最近一次 UDP 下行命令快照 |
| `feedback` | object | 电机反馈与 FK 结果 |
| `runtime` | object | 控制状态与最后重力扭矩 |

### 3.3 `udp` 字段

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| `seq` | number | - | 最近一帧 UDP 序号 |
| `age_ms` | number \| null | ms | UDP 最新帧年龄；未收到时为 `null` |
| `p_rel_deg` | `number[5]` | deg | 相对标定角（第 5 维为 `joint5_rel_deg`） |
| `omega_rad_s` | `number[5]` | rad/s | 角速度（第 5 维预留） |

### 3.4 `feedback` 字段

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| `mit_arm_rad` | `number[4] \| null` | rad | MIT 反馈角 |
| `fb_arm_rad` | `number[4] \| null` | rad | FB 反馈角 |
| `link5_hmat` | `number[4][4] \| null` | 旋转无量纲，平移 m | `base -> link5` 齐次矩阵 |
| `motors` | `array<object>` | - | 各电机状态（如 `id/p/v/t`） |
| `crc_error_count` | number | - | 串口 CRC 错误累计数 |

### 3.5 `runtime` 字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `control_source` | string | 当前控制来源（如 `udp` / `hold`） |
| `safety_reason` | string | 当前安全状态原因（如 `ok` / `udp_timeout`） |
| `last_tau_nm` | `number[4]` | 最近一次下发重力补偿扭矩 |
| `servo_command` | object | 伺服命令缓存（透传） |

## 4. 广播与接收语义

### 4.1 广播语义

- 服务端周期性推送最新状态快照，不区分订阅主题。
- 多客户端并发时，每个连接收到相同格式的最新状态。
- 客户端断连不会影响主控制循环。

### 4.2 仿真字段（`feedback.link5_hmat`）语义

`link5_hmat` 用于仿真端进行机械臂末端姿态显示，定义如下：

- 语义：`base -> link5` 的 4x4 齐次矩阵（行主序嵌套数组）
- 角度来源（混合）：
  - 前 4 轴：优先 `feedback.fb_arm_rad`，回退 `feedback.mit_arm_rad`
  - 第 5 轴：`udp.p_rel_deg[4]`（bridge 下发的 `joint5_rel_deg`，单位 deg）
- 可空条件：
  - 无反馈角
  - 未收到 UDP（或无第 5 轴角）
  - FK 计算异常

### 4.3 仿真端最小接收建议

- 仅消费 `type == "state"` 的消息。
- 以最新帧覆盖渲染，避免堆积历史帧。
- 对 `link5_hmat == null` 做降级（保持上一帧或隐藏模型）。
- 可用 `udp.seq` 与 `udp.age_ms` 做新鲜度监控。

## 5. 错误模型

广播路径（Pi -> 相机）：

- 无应用层 ACK，发送失败仅在服务端内部处理，不影响主循环。
- 单个客户端发送异常会被移除，不影响其他客户端。

接收路径（相机 -> Pi，可选）：

- 客户端可发 JSON 文本命令；解析失败或语义错误由服务端命令处理器内部消化。
- 不定义协议级错误回包；建议相机端将上行命令视为“尽力而为”。

## 6. 兼容性与版本演进

- 当前版本：`pi2camera v1`（WebSocket `type=state`）
- 向后兼容原则：
  - 已有字段保持语义不变
  - 新字段只追加，不删除既有字段
- 演进建议：
  - 若需多类消息，新增 `type`（例如 `type=vision_hint`）并保持 `state` 不变
  - 若需高带宽二进制流，建议独立通道，不复用本状态广播

## 7. 最小联调示例

### 7.1 Python 客户端订阅

```python
import asyncio
import json
import websockets

async def main():
    uri = "ws://127.0.0.1:8765"
    async with websockets.connect(uri) as ws:
        async for raw in ws:
            msg = json.loads(raw)
            if msg.get("type") != "state":
                continue
            data = msg.get("data", {})
            hmat = (data.get("feedback") or {}).get("link5_hmat")
            print("seq=", (data.get("udp") or {}).get("seq"), "link5_hmat_null=", hmat is None)

asyncio.run(main())
```

### 7.2 广播消息示例（节选）

```json
{
  "type": "state",
  "data": {
    "udp": {
      "seq": 1234,
      "age_ms": 18.7,
      "p_rel_deg": [0.0, 10.0, -20.0, 5.0, 0.0],
      "omega_rad_s": [0.0, 0.2, -0.3, 0.1, 0.0]
    },
    "feedback": {
      "link5_hmat": [
        [0.99, 0.01, 0.04, 0.28],
        [-0.01, 1.0, -0.02, 0.07],
        [-0.04, 0.02, 0.99, -0.01],
        [0.0, 0.0, 0.0, 1.0]
      ]
    }
  }
}
```

## 8. 交叉引用

- 上游输入协议：`docs/head2bridge.md`
- bridge 下发协议：`docs/bridge2pi.md`
- UDP 与 WS 总览：`docs/PC_RPI_UDP_PROTOCOL.md`
- 关键代码：
  - `icecreamPi/server.py`
  - `icecreamPi/infra/state/presenter.py`
