# pi2camera API 规范（camera <-> icecreamPi，UDP 双向，v2）

- 文档状态：stable
- 适用模块：`icecreamPi` UDP camera adapter（新增链路）
- 对端：camera <-> 树莓派（`icecreamPi`）
- 相关文档：`docs/bridge2pi.md`、`docs/head2bridge.md`、`docs/pi2stm.md`

## 1. 范围（Scope）

本文档定义 camera 与 Pi 的 UDP 双向协议，包括：

- camera -> pi：2 舵机角上行
- pi -> camera：4 电机反馈 + 1 手腕角计算得到的齐次矩阵广播
- 端口、频率、数据帧结构、校验建议

不包含：

- bridge 到 Pi 的控制下发细节（见 `docs/bridge2pi.md`）
- Pi 到 STM32 串口帧细节（见 `docs/pi2stm.md`）

## 2. 传输与端点

- 协议：UDP（双向，无 ACK）
- 推荐端口：
  - camera -> pi：`9981`
  - pi -> camera：`9982`（广播或定向发送）
- 推荐频率：
  - camera -> pi：`20~30Hz`
  - pi -> camera：`20~25Hz`

## 3. 上行协议（camera -> pi）

### 3.1 固定帧格式

- Python `struct`：`=Iddd`
- 总长度：`28` 字节

| 顺序 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 1 | `seq` | `uint32` | - | 相机上行序号 |
| 2 | `ts` | `float64` | s | 相机时间戳 |
| 3 | `wrist_deg` | `float64` | deg | 手腕舵机角 |
| 4 | `head_like_deg` | `float64` | deg | 第二舵机角（当前按 head 角处理） |

### 3.2 语义说明

- `wrist_deg` 可直接覆盖 Pi 当前手腕目标角来源之一。
- `head_like_deg` 当前用于保留/透传，不参与 `link5_hmat` 计算。

## 4. 下行协议（pi -> camera）

### 4.1 固定帧格式

- 传输形态：UTF-8 JSON（单包）
- 推荐字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `type` | string | 固定 `camera_state` |
| `seq` | number | Pi 下行序号 |
| `ts` | number | Pi 时间戳（秒） |
| `motor_rad` | `number[4]` | 4 电机反馈角（rad） |
| `wrist_deg` | number | 参与 FK 的手腕角（deg） |
| `link5_hmat` | `number[4][4] \| null` | `base -> link5` 齐次矩阵 |
| `grip_state` | number | 夹爪开合状态（0/1） |

### 4.2 `link5_hmat` 计算语义

- 输入角度来源：
  - 前 4 轴：电机反馈角（`motor_rad[0..3]`）
  - 第 5 轴：`wrist_deg`
- 不参与项：
  - 第 6 维夹爪状态（`grip_state`）
  - camera 上行的第二舵机角 `head_like_deg`
- 齐次矩阵构造（与树莓派 `calculator` / WebSocket 一致）：先由解析 FK 得到 link5 在 link0 下的 `^0R_5` 与 `^0t_5`，将 `^0R_5` 按 **\(R_x \cdot R_y \cdot R_z\)**（roll, pitch, yaw，rad）分解后与平移拼成 `^0T_5`（4×4 行主序 JSON）。

### 4.3 Pi 环境变量（UDP 下行广播）

- `ARM_CONTROL_CAMERA_UDP_BROADCAST=1`：启用向 `ARM_CONTROL_CAMERA_UDP_HOST`:`ARM_CONTROL_CAMERA_UDP_PORT` 周期性发送 `camera_state` JSON。
- `ARM_CONTROL_CAMERA_UDP_HOST`：默认 `255.255.255.255`（广播）；也可设为相机单播 IP。
- `ARM_CONTROL_CAMERA_UDP_PORT`：默认 `9982`。
- `ARM_CONTROL_CAMERA_UDP_HZ`：默认 `20`。
- `ARM_CONTROL_CAMERA_UDP_SO_BROADCAST`：默认 `1`；单播时可置 `0`。

## 5. 最小校验策略

camera -> pi：

- 帧长必须为 `28` 字节。
- `wrist_deg/head_like_deg` 建议做范围限幅。

pi -> camera：

- 仅消费 `type == camera_state`。
- `link5_hmat == null` 时使用降级策略（保持上一帧或隐藏模型）。

## 6. 最小联调示例

### 6.1 camera -> pi 解码

```python
import struct

UP_FMT = "=Iddd"
assert struct.calcsize(UP_FMT) == 28

def decode_up(pkt: bytes):
    seq, ts, wrist_deg, head_like_deg = struct.unpack(UP_FMT, pkt)
    return seq, ts, wrist_deg, head_like_deg
```

### 6.2 pi -> camera 示例

```json
{
  "type": "camera_state",
  "seq": 20031,
  "ts": 1714297200.15,
  "motor_rad": [1.57, 1.20, -0.40, 0.30],
  "wrist_deg": 15.0,
  "grip_state": 1,
  "link5_hmat": [
    [0.99, 0.01, 0.04, 0.28],
    [-0.01, 1.0, -0.02, 0.07],
    [-0.04, 0.02, 0.99, -0.01],
    [0.0, 0.0, 0.0, 1.0]
  ]
}
```

## 7. 迁移说明（WS -> UDP）

- 旧版 `pi2camera v1` 以 WebSocket `type=state` 为主。
- 新版 `pi2camera v2` 以 UDP 双向为主。
- WebSocket 可保留为历史兼容通道，但不再作为主协议说明。

## 8. 交叉引用

- bridge 下发协议：`docs/bridge2pi.md`
- 上游控制协议：`docs/head2bridge.md`
- 串口链路协议：`docs/pi2stm.md`
