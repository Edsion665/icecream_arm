# STM32 ↔ 树莓派 串口通信协议

基于 MIT 电机控制帧，将四个电机的数据拼合为单条串口消息。

---

## 电机参数（编解码必须使用）

| 电机 | Pmax (rad) | Vmax (rad/s) | Tmax (Nm) |
|------|-----------|-------------|----------|
| M1   | ±12.5     | ±30         | ±10      |
| M2   | ±12.5     | ±10         | ±28      |
| M3   | ±12.5     | ±10         | ±28      |
| M4   | ±12.5     | ±30         | ±10      |

Kp 范围：0 ~ 500，Kd 范围：0 ~ 5（MIT 协议标准范围，12-bit 量化）

---

## 量化公式

```
uint = (x - x_min) / (x_max - x_min) * (2^bits - 1)
x    = uint / (2^bits - 1) * (x_max - x_min) + x_min
```

- p：16-bit（0 ~ 65535）
- v, kp, kd, t：12-bit（0 ~ 4095）

---

## 下行帧（树莓派 → STM32）

**功能：** 树莓派发送四个电机的控制指令

**总长度：32 字节**（每电机 8 字节 × 4）

### 单电机 MIT 控制帧（8 字节）

与 CAN 总线 MIT 帧格式完全一致：

```
Byte 0:  p[15:8]
Byte 1:  p[7:0]
Byte 2:  v[11:4]
Byte 3:  v[3:0] | kp[11:8]
Byte 4:  kp[7:0]
Byte 5:  kd[11:4]
Byte 6:  kd[3:0] | t[11:8]
Byte 7:  t[7:0]
```

### 完整下行帧结构

```
[M1: 8字节][M2: 8字节][M3: 8字节][M4: 8字节]
偏移 0~7    偏移 8~15   偏移 16~23  偏移 24~31
```

---

## 上行帧（STM32 → 树莓派）

**功能：** STM32 回传四个电机的 CAN 原始反馈字节，不做任何处理

**总长度：34 字节**（帧头 2 字节 + 每电机 8 字节 × 4）

### 帧结构

```
Byte 0:    0xAA  (帧头)
Byte 1:    0x55  (帧头)
Byte 2~9:  M1 原始 CAN 反馈帧（8字节，不足补0x00）
Byte 10~17: M2 原始 CAN 反馈帧
Byte 18~25: M3 原始 CAN 反馈帧
Byte 26~33: M4 原始 CAN 反馈帧
```

### 单电机 CAN 反馈帧格式（MIT 协议）

```
Byte 0:  id[3:0] | err[3:0]
Byte 1:  p[15:8]
Byte 2:  p[7:0]
Byte 3:  v[11:4]
Byte 4:  v[3:0] | t[11:8]
Byte 5:  t[7:0]
Byte 6:  mos_temp   (若 DLC>=8，否则 0x00)
Byte 7:  rotor_temp (若 DLC>=8，否则 0x00)
```

### 树莓派解码示例（Python）

```python
MOTOR_PARAMS = [
    {"p": (-12.5, 12.5), "v": (-30, 30),  "t": (-10, 10)},  # M1
    {"p": (-12.5, 12.5), "v": (-10, 10),  "t": (-28, 28)},  # M2
    {"p": (-12.5, 12.5), "v": (-10, 10),  "t": (-28, 28)},  # M3
    {"p": (-12.5, 12.5), "v": (-30, 30),  "t": (-10, 10)},  # M4
]

def uint_to_float(x, x_min, x_max, bits):
    return x / (2**bits - 1) * (x_max - x_min) + x_min

def decode_uplink(data: bytes) -> list[dict]:
    """解码上行帧（34字节）→ 四电机 p/v/t/err/mos_temp/rotor_temp"""
    assert len(data) == 34 and data[0] == 0xAA and data[1] == 0x55
    result = []
    for i in range(4):
        b = data[2 + i*8 : 2 + i*8 + 8]
        err  = b[0] >> 4
        p    = (b[1] << 8) | b[2]
        v    = (b[3] << 4) | (b[4] >> 4)
        t    = ((b[4] & 0x0F) << 8) | b[5]
        pm   = MOTOR_PARAMS[i]
        result.append({
            "err":        err,
            "p":          uint_to_float(p, *pm["p"], 16),
            "v":          uint_to_float(v, *pm["v"], 12),
            "t":          uint_to_float(t, *pm["t"], 12),
            "mos_temp":   b[6],
            "rotor_temp": b[7],
        })
    return result
```

---

## 串口配置建议

| 参数 | 值 |
|------|----|
| 波特率 | 115200 或 460800 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |

帧边界：下行固定 32 字节；上行以 `0xAA 0x55` 帧头同步，总长 34 字节。
