# pi2stm API 规范（icecreamPi <-> STM32 串口，v3）

- 文档状态：stable
- 适用模块：`icecreamPi/serial.py`、`icecreamPi/infra/serial/codec.py`
- 对端：`icecreamPi` <-> STM32（UART）
- 相关文档：`docs/bridge2pi.md`、`docs/pi2camera.md`

## 1. 范围（Scope）

本文档定义 Pi 与 STM32 的串口双向协议，包括：

- 下行：Pi -> STM32（控制命令）
- 上行：STM32 -> Pi（反馈状态）
- 统一 **42 字节**帧结构、字段位宽、XOR 校验

在 v2（39 字节）基础上增加：

- 步进电机：**增量转角**（int16，单位 1°，范围 [-180, 180]）
- 传送带：**启停**（uint8，`0`=停，`1`=转）

## 2. 传输与串口参数

- 物理层：UART（8N1）
- 推荐波特率：`115200`
- 帧头：`0xAA 0x55`
- 帧尾：`xor`（前 41 字节逐字节异或）
- 帧长：固定 **`42`** 字节

## 3. 帧总览（双向同构）

| 字节区间 | 长度 | 含义 |
|---|---:|---|
| `[0..1]` | 2 | 帧头 `AA 55` |
| `[2..33]` | 32 | 4 路电机段（每路 8 字节） |
| `[34..35]` | 2 | `wrist_us`（uint16，大端） |
| `[36..37]` | 2 | `gripper_us`（uint16，大端） |
| `[38..39]` | 2 | `stepper_deg`（int16，大端，单位 1°） |
| `[40]` | 1 | `conveyor_run`（`0`=停，`1`=转） |
| `[41]` | 1 | XOR 校验 |

## 4. 下行（Pi -> STM32）

### 4.1 4 路电机每路 8 字节打包

与 v2 相同，按 MIT 常见打包格式：

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
| 3 | `v_u[3:0] << 4 \| kp_u[11:8]` |
| 4 | `kp_u[7:0]` |
| 5 | `kd_u[11:4]` |
| 6 | `kd_u[3:0] << 4 \| t_u[11:8]` |
| 7 | `t_u[7:0]` |

### 4.2 舵机与辅机字段

| 字段 | 类型 | 范围 / 语义 |
|---|---|---|
| `wrist_us` | uint16 BE | `[500, 2500]`，腕部舵机脉宽 us |
| `gripper_us` | uint16 BE | `[500, 2500]`，夹爪舵机脉宽 us |
| `stepper_deg` | int16 BE | `[-180, 180]`，步进**增量角**（度）；`0` 表示本帧不新增步进动作 |
| `conveyor_run` | uint8 | `0` 停止（PB6 低），`1` 旋转（PB6 高） |

电机 / 舵机数值范围与 v2 相同：

- `p`：`[-12.5, 12.5]`
- `v`：`[-45.0, 45.0]`
- `kp`：`[0.0, 500.0]`
- `kd`：`[0.0, 5.0]`
- `t`：`[-18.0, 18.0]`

超范围值需在 Pi 侧先限幅后打包。

### 4.3 STM32 执行语义（下行）

| 字段 | STM32 行为 |
|---|---|
| 电机 32B | 立即 `Motor_MIT_Send_Raw` |
| `wrist_us` / `gripper_us` | 写入舵机目标，由 `Servo_Update()` 渐变 |
| `stepper_deg` | 非 0 时写入步进目标；`Stepper_Update()` **非阻塞**分次计脉冲，主循环可继续跑串口/CAN 服务 |
| `conveyor_run` | 写入传送带目标，`Conveyor_Update()` 中应用到 PB6 |

## 5. 上行（STM32 -> Pi）

上行同样为 **42 字节**，布局与下行一致：

- `[2..33]`：4 路电机原始 CAN 反馈（每路 8 字节）
- `[34..35]`：当前腕部舵机脉宽 `wrist_us`（与下行同缩放）
- `[36..37]`：当前夹爪脉宽 `gripper_us`
- `[38..39]`：`stepper_deg` 步进**逻辑位置**（int16 °，由已发脉冲累计换算，开环）
- `[40]`：`conveyor_run` 当前 PB6 输出（`0` / `1`）
- `[41]`：XOR

解码示例：

```python
wrist_us = (raw[34] << 8) | raw[35]
gripper_us = (raw[36] << 8) | raw[37]
stepper_deg = struct.unpack(">h", raw[38:40])[0]
conveyor_run = raw[40]
```

Pi 侧对 `raw[0:41]` 做 XOR 并与 `raw[41]` 比较。

## 6. 校验规则

```python
xor = 0
for b in frame[:41]:
    xor ^= b
ok = (xor & 0xFF) == frame[41]
```

处理建议：

- 校验失败：丢弃该帧并计数。
- 帧头错误或长度非 42：丢弃并等待下一帧头。

## 7. 最小联调示例

### 7.1 Pi 侧编码下行（示意）

```python
import struct

def encode_mit_cmd_42(motors, wrist_us=1500, gripper_us=1500,
                      stepper_deg=0, conveyor_run=0):
    frame = bytearray(42)
    frame[0], frame[1] = 0xAA, 0x55
    # ... 打包 motors[0..3] 至 frame[2:34]（同 v2）...
    frame[34:36] = struct.pack(">H", wrist_us)
    frame[36:38] = struct.pack(">H", gripper_us)
    frame[38:40] = struct.pack(">h", int(stepper_deg))
    frame[40] = 1 if conveyor_run else 0
    x = 0
    for b in frame[:41]:
        x ^= b
    frame[41] = x
    return bytes(frame)
```

### 7.2 Pi 侧解码上行（示意）

```python
def decode_mit_uplink(raw42):
    assert len(raw42) == 42
    # xor 校验 ...
    motors = parse_motors(raw42[2:34])
    servo = {
        "wrist_us": (raw42[34] << 8) | raw42[35],
        "gripper_us": (raw42[36] << 8) | raw42[37],
    }
    aux = {
        "stepper_deg": struct.unpack(">h", raw42[38:40])[0],
        "conveyor_run": raw42[40],
    }
    return motors, servo, aux
```

## 8. 迁移说明

| 版本 | 帧长 | 变更 |
|---|---:|---|
| v1 | 35 | 仅 4 电机 |
| v2 | 39 | + 腕/爪舵机 `[34..37]` |
| **v3** | **42** | + 步进 `[38..39]` + 传送带 `[40]`，XOR 移至 `[41]` |

升级 checklist：

- 帧长：`39` -> **`42`**
- XOR 下标：`38` -> **`41`**
- 编解码增加 `stepper_deg`、`conveyor_run`
- Pi / STM32 须同版本，**不可**与仅 39 字节的对端互通

## 9. 交叉引用

- bridge 到 Pi：`docs/bridge2pi.md`
- Pi 到 camera：`docs/pi2camera.md`
- 上游命令到 bridge：`docs/head2bridge.md`
