# 树莓派 → STM32 MIT 二进制命令帧编码指南

## 帧结构（35 字节）

```
Byte  0    : 0xAA  (帧头)
Byte  1    : 0x55  (帧头)
Byte  2-9  : 电机 0 MIT 命令（8 字节）
Byte 10-17 : 电机 1 MIT 命令（8 字节）
Byte 18-25 : 电机 2 MIT 命令（8 字节）
Byte 26-33 : 电机 3 MIT 命令（8 字节）
Byte 34    : XOR 校验（byte 0 .. byte 33 逐字节异或）
```

---

## 每个电机 8 字节布局

与 MIT 模式 CAN 命令帧完全相同：

| 字节 | 内容 | 说明 |
|------|------|------|
| 0 | `p[15:8]` | 位置高 8 位 |
| 1 | `p[7:0]`  | 位置低 8 位 |
| 2 | `v[11:4]` | 速度高 8 位 |
| 3 | `(v[3:0] << 4) \| kp[11:8]` | 速度低 4 位 + 刚度高 4 位 |
| 4 | `kp[7:0]` | 刚度低 8 位 |
| 5 | `kd[11:4]` | 阻尼高 8 位 |
| 6 | `(kd[3:0] << 4) \| t[11:8]` | 阻尼低 4 位 + 力矩高 4 位 |
| 7 | `t[7:0]`  | 力矩低 8 位 |

---

## 物理量编码（float → uint）

```
uint = round( (x - x_min) / (x_max - x_min) * (2^bits - 1) )
```

各字段范围与位宽：

| 字段 | 范围 | 位宽 | 最大整数值 |
|------|------|------|-----------|
| p（位置） | -12.5 ~ +12.5 rad | 16 bit | 65535 |
| v（速度） | -45 ~ +45 rad/s   | 12 bit | 4095  |
| kp（刚度）| 0 ~ 500           | 12 bit | 4095  |
| kd（阻尼）| 0 ~ 5             | 12 bit | 4095  |
| t（力矩） | -18 ~ +18 Nm      | 12 bit | 4095  |

---

## Python 编码示例

```python
import struct

P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -45.0, 45.0
T_MIN, T_MAX = -18.0, 18.0

def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) / (x_max - x_min) * ((1 << bits) - 1))

def encode_motor(p, v, kp, kd, t):
    p_i  = float_to_uint(p,  P_MIN, P_MAX, 16)
    v_i  = float_to_uint(v,  V_MIN, V_MAX, 12)
    kp_i = float_to_uint(kp, 0.0,   500.0, 12)
    kd_i = float_to_uint(kd, 0.0,   5.0,   12)
    t_i  = float_to_uint(t,  T_MIN, T_MAX, 12)

    b = bytearray(8)
    b[0] = (p_i >> 8) & 0xFF
    b[1] =  p_i       & 0xFF
    b[2] = (v_i >> 4) & 0xFF
    b[3] = ((v_i & 0xF) << 4) | ((kp_i >> 8) & 0xF)
    b[4] =  kp_i      & 0xFF
    b[5] = (kd_i >> 4) & 0xFF
    b[6] = ((kd_i & 0xF) << 4) | ((t_i >> 8) & 0xF)
    b[7] =  t_i       & 0xFF
    return b

def build_frame(motors):
    """
    motors: list of 4 dicts, each with keys p, v, kp, kd, t
    returns: bytes (35 bytes)
    """
    frame = bytearray([0xAA, 0x55])
    for m in motors:
        frame += encode_motor(m['p'], m['v'], m['kp'], m['kd'], m['t'])
    xor = 0
    for byte in frame:
        xor ^= byte
    frame.append(xor)
    return bytes(frame)
```

### 使用示例

```python
import serial

motors = [
    {'p': 0.0,  'v': 0.0, 'kp': 15.0, 'kd': 1.8, 't': 0.0},
    {'p': 1.57, 'v': 0.0, 'kp': 30.0, 'kd': 2.5, 't': 0.0},
    {'p': 0.5,  'v': 0.0, 'kp': 42.0, 'kd': 2.3, 't': 0.0},
    {'p': 0.0,  'v': 0.0, 'kp': 18.0, 'kd': 0.3, 't': 0.0},
]

frame = build_frame(motors)
assert len(frame) == 35

ser = serial.Serial('/dev/ttyAMA0', 115200)
ser.write(frame)
```

---

## STM32 回传格式

解码成功后，STM32 通过 USART1 回传 4 行文本：

```
MIT0 p=0.0000 v=0.0000 kp=15.00 kd=1.800 t=0.0000
MIT1 p=1.5700 v=0.0000 kp=30.00 kd=2.500 t=0.0000
MIT2 p=0.5000 v=0.0000 kp=42.00 kd=2.300 t=0.0000
MIT3 p=0.0000 v=0.0000 kp=18.00 kd=0.300 t=0.0000
```

校验失败时回传：`MIT_BIN FAIL chk`

---

## 注意事项

- 帧头固定为 `0xAA 0x55`（与上行反馈帧 `0xAA 0x55` 相同，方向相反，不冲突）
- XOR 校验覆盖 byte 0 ~ byte 33（含帧头），byte 34 为校验值
- 当前固件仅解码回传，**不执行 CAN 发送**（后续步骤再接入）
- kp/kd 范围全轴统一（0~500 / 0~5），p/v/t 范围见 `motor_config.h` 中 `MIT_*` 宏
