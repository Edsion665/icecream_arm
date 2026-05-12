# pi2stm API 规范（icecreamPi <-> STM32 串口，v2）

- 文档状态：stable
- 适用模块：`icecreamPi/serial.py`、`icecreamPi/infra/serial/codec.py`
- 对端：`icecreamPi` <-> STM32（UART）
- 相关文档：`docs/bridge2pi.md`、`docs/pi2camera.md`

## 1. 范围（Scope）

本文档定义 Pi 与 STM32 的串口双向协议，包括：

- 下行：Pi -> STM32（控制命令）
- 上行：STM32 -> Pi（反馈状态）
- 统一 39 字节帧结构、字段位宽、XOR 校验

## 2. 传输与串口参数

- 物理层：UART（8N1）
- 推荐波特率：`115200`
- 帧头：`0xAA 0x55`
- 帧尾：`xor`（前 38 字节逐字节异或）
- 帧长：固定 `39` 字节

## 3. 帧总览（双向同构）

| 字节区间 | 长度 | 含义 |
|---|---:|---|
| `[0..1]` | 2 | 帧头 `AA55` |
| `[2..33]` | 32 | 4 路电机段（每路 8 字节） |
| `[34..35]` | 2 | `wrist_us`（uint16，大端） |
| `[36..37]` | 2 | `gripper_us`（uint16，大端） |
| `[38]` | 1 | XOR 校验 |

## 4. 下行（Pi -> STM32）

### 4.1 4 路电机每路 8 字节打包

按 MIT 常见打包格式：

- `p`：16 bit
- `v`：12 bit
- `kp`：12 bit
- `kd`：12 bit
- `t`：12 bit

每路 8 字节布局：

| 字节偏移（路内） | 内容 |
|---:|---|
| 0 | `p_u[15:8]` |
| 1 | `p_u[7:0]` |
| 2 | `v_u[11:4]` |
| 3 | `v_u[3:0] << 4 | kp_u[11:8]` |
| 4 | `kp_u[7:0]` |
| 5 | `kd_u[11:4]` |
| 6 | `kd_u[3:0] << 4 | t_u[11:8]` |
| 7 | `t_u[7:0]` |

### 4.2 数值范围与限幅

- `p`：`[-12.5, 12.5]`
- `v`：`[-45.0, 45.0]`
- `kp`：`[0.0, 500.0]`
- `kd`：`[0.0, 5.0]`
- `t`：`[-18.0, 18.0]`
- `wrist_us`：`[500, 2500]`
- `gripper_us`：`[500, 2500]`

超范围值需在 Pi 侧先限幅后打包。

## 5. 上行（STM32 -> Pi）

上行同样为 39 字节：

- 前 32 字节对应 4 路电机反馈段（每路 8 字节）
- `[34..37]` 为舵机状态回传：
  - `wrist_us = (raw[34] << 8) | raw[35]`
  - `gripper_us = (raw[36] << 8) | raw[37]`
- Pi 侧对 `raw[0:38]` 做 XOR 并与 `raw[38]` 比较

## 6. 校验规则

计算方式：

```python
xor = 0
for b in frame[:38]:
    xor ^= b
ok = (xor & 0xFF) == frame[38]
```

处理建议：

- 校验失败：丢弃该帧并计数。
- 帧头错误或长度非 39：丢弃并等待下一帧头。

## 7. 最小联调示例

### 7.1 Pi 侧编码发送（示意）

```python
from infra.serial.codec import encode_mit_cmd_39

motors = [{"p":0,"v":0,"kp":0,"kd":0,"t":0}] * 4
frame = encode_mit_cmd_39(motors, wrist_us=1500, gripper_us=1500)
assert len(frame) == 39
```

### 7.2 Pi 侧解码上行（示意）

```python
from infra.serial.codec import decode_mit_uplink

motors, servo = decode_mit_uplink(raw39)
print(servo["wrist_us"], servo["gripper_us"])
```

## 8. 迁移说明（35 字节 -> 39 字节）

- 旧版下行常见为 35 字节（仅 4 电机，无 2 舵机字段）。
- 新版统一为 39 字节（4 电机 + 2 舵机 + XOR）。
- 升级时需同步更新：
  - 帧长判断（35/39 -> 39）
  - 舵机字段读写（`[34..37]`）
  - 调试解析逻辑

## 9. 交叉引用

- bridge 到 Pi：`docs/bridge2pi.md`
- Pi 到 camera：`docs/pi2camera.md`
- 上游命令到 bridge：`docs/head2bridge.md`
