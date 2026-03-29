# 六轴机械臂重力补偿实施方案

> 基于 icecream_arm / feature/motor_driver 分支
> 目标平台：STM32F103 (72 MHz, 软浮点)
> 方案类型：**模型驱动 + 数据校准混合法**

---

## 一、现状分析与资产盘点

### 1.1 硬件架构

| 层级 | 组件 | 说明 |
|------|------|------|
| 主控 | STM32F103 (72 MHz, Cortex-M3, **无硬件FPU**) | Keil MDK 编译 |
| 电机 | 4× MIT Cheetah协议驱动器 | CAN 1 Mbps，ID 0x01-0x04 |
| 关节5/6 | PWM舵机 (TIM3 50Hz) | 腕部+夹爪，不涉及重力补偿 |
| 上位机 | 树莓派 | UART1 115200 bps，下发 DATA: 帧 |

### 1.2 MIT Protocol CAN 接口（tau_ff 注入点）

```c
// 原型 (motor_can.h)
void Motor_MIT_Send_Raw(int idx, float p, float v, float kp, float kd, float t);
//                                                                            ^^
//                                                                  tau_ff ← 重力补偿注入点
// 物理量程: t ∈ [-18.0, +18.0] Nm (12-bit 编码)
// 低延迟版本（用于TIM4 ISR）:
void Motor_MIT_Send_Raw_NoPostDelay(int idx, float p, float v, float kp, float kd, float t);
```

### 1.3 现有手动补偿（待替换）

```c
// motor_config.h 中的静态数组
#define MOVE_TFF_POSITIVE  {0.0f, 0.35f, 1.20f, 0.0f}   // Joint 0~3 运动正向
#define MOVE_TFF_NEGATIVE  {0.0f, 0.05f, 0.40f, 0.0f}   // Joint 0~3 运动反向
#define HOLD_TFF           {0.0f, 0.10f, 0.80f, 0.0f}   // 保持模式
#define EXTREME_HOLD_TFF   {0.0f, 0.45f, 0.85f, 0.0f}   // 刚性保持
```

**核心问题**：固定值与关节角度无关，臂展越大/越小误差越大，运动过程中补偿不连续。

### 1.4 数据资产盘点

**文件：`captures.jsonl`（74条样本）**

```
字段说明（每行仅下列键，扁平 JSON）：
  motors.x.angle   → 电机编码器关节角 (deg×100)，x=0~3
  joints.x.torque  → 力矩反馈 (Nm)，x=1~4
                     根据物理验证：joints.1.torque ≈ 0.03 Nm（base竖直轴≈无重力矩）✓
```

**Home 校准位置（motor_config.h）：**

| Motor idx | Joint | HOME (deg×100) | HOME (°) | HOME (rad) |
|-----------|-------|----------------|----------|------------|
| 0 | 竖转基座 | −298 | −2.98° | −0.052 |
| 1 | 肩关节 | 8059 | +80.59° | +1.407 |
| 2 | 肘关节 | 13327 | +133.27° | +2.326 |
| 3 | 腕关节 | 2700 | +27.00° | +0.471 |

**URDF 惯性参数（SolidWorks 导出，可信度高）：**

| 连杆 | 质量 (kg) | COM_x (m) | COM_y (m) | COM_z (m) |
|------|-----------|-----------|-----------|-----------|
| link1 | 0.190 | 0 | −0.000388 | 0.00329 |
| link2 | 0.562 | 0.0788 | 0.0784 | 0.0271 |
| link3 | 0.543 | −0.0978 | 0.0318 | −0.0270 |
| link4 | 0.238 | −0.0241 | −0.000153 | −0.0236 |
| link5 | 0.182 | 0.00239 | −0.0122 | 0.0434 |

---

## 二、技术路线选型

### 方案对比

| 方案 | 原理 | STM32可行性 | 精度 | 工作量 |
|------|------|------------|------|--------|
| A: 纯RNEA模型 | 递推Newton-Euler，6×6矩阵 | ⚠️ 软浮点约500µs/次 | 高 | 大 |
| B: 纯数据拟合 | 74样本多项式回归 | ✅ | 中（数据覆盖有限） | 小 |
| **C: 静态重力回归量（推荐）** | 从运动学结构推导cos参数方程，数据校准 | ✅ < 20µs/次 | 高 | 中 |

**选择方案C** 的理由：
1. STM32F103无硬件FPU，`cosf()`约50 cycles，整个3-joint计算 < 400 cycles ≈ **5.5µs @72MHz**，远小于2ms控制周期
2. URDF提供结构先验，可确定三角函数的形式（角度偏置φ）
3. captures.jsonl提供实物校准数据，补偿实际制造误差

---

## 三、重力力矩数学模型

### 3.1 关节轴特征分析（来自URDF joint定义）

| Motor idx | Joint | 旋转轴（世界系） | 重力影响 |
|-----------|-------|-----------------|---------|
| 0 | 基座 joint1 | 垂直 Z 轴 (yaw) | **τ_g0 ≈ 0** |
| 1 | 肩 joint2 | 水平轴 (pitch) | **主要负载** |
| 2 | 肘 joint3 | 斜水平轴 | **次要负载** |
| 3 | 腕 joint4 | 类水平轴 | **远端负载** |

### 3.2 静态重力力矩方程

对串联臂结构，关节 i 的静态重力力矩 = 该关节以远所有连杆COM投影到力矩臂方向的分量之和。

定义相对 home 的关节角：
- q1 = (motor1_pos - HOME1) / 100 × π/180  [肩关节, rad]
- q2 = (motor2_pos - HOME2) / 100 × π/180  [肘关节, rad]
- q3 = (motor3_pos - HOME3) / 100 × π/180  [腕关节, rad]

**肩关节（Motor 1）重力力矩：**
```
τ_g1 = A1·cos(q1 + φ1) + A2·cos(q1 + q2 + φ2) + A3·cos(q1 + q2 + q3 + φ3)
```

**肘关节（Motor 2）重力力矩：**
```
τ_g2 = B1·cos(q1 + q2 + ψ1) + B2·cos(q1 + q2 + q3 + ψ2)
```

**腕关节（Motor 3）重力力矩：**
```
τ_g3 = C1·cos(q1 + q2 + q3 + ζ1)
```

其中：
- **A1, A2, A3, B1, B2, C1**：增益参数（由质量×等效力臂决定，用数据拟合）
- **φ1,φ2,φ3,ψ1,ψ2,ζ1**：角度偏置（由URDF关节结构角决定，可先设0再微调）

---

## 四、三阶段实施计划

---

### ⭐ 阶段一：参数辨识（在树莓派上运行 Python）

**目标**：从 captures.jsonl 拟合出模型参数 A1-C1 和偏置 φ。

**执行时间**：约 2-3 小时

#### Step 1.1：验证数据格式

保存为 `/home/huangjianan/robotics/robotarm/scripts/verify_captures.py`：

```python
#!/usr/bin/env python3
"""验证 captures.jsonl 中 joints.x.torque 是力矩还是角度"""
import json
import numpy as np

with open('captures.jsonl') as f:
    samples = [json.loads(l) for l in f]

# joints.1.torque：基座力矩通道；motors.0.angle：基座编码器角
j1_tau = [s['joints.1.torque'] for s in samples]
a0_enc = [s['motors.0.angle'] for s in samples]

corr = np.corrcoef(a0_enc, j1_tau)[0, 1]
print(f"joints.1.torque 与 motors.0.angle 的相关系数: {corr:.3f}")
print(f"joints.1.torque 均值: {np.mean(j1_tau):.4f}, 标准差: {np.std(j1_tau):.4f}")
print(f"joints.2.torque 范围: [{min(s['joints.2.torque'] for s in samples):.2f}, "
      f"{max(s['joints.2.torque'] for s in samples):.2f}]")

if abs(np.std(j1_tau)) < 0.1 and abs(np.mean(j1_tau)) < 0.2:
    print("✅ 确认: joints.x.torque = 力矩(Nm), 可直接用于拟合")
else:
    print("⚠️  joints.x.torque 可能是关节角度，需重新采集力矩数据")
```

**通过标准**：
- joints.1.torque 标准差 < 0.1 → 确认为力矩数据，继续 Step 1.2
- 否则 → 参考 附录A 重新采集力矩数据

#### Step 1.2：计算关节角度并拟合参数

保存为 `/home/huangjianan/robotics/robotarm/scripts/gravity_identification.py`：

```python
#!/usr/bin/env python3
"""
重力补偿参数辨识脚本
输出: gravity_params.h (直接复制到 STM32 工程)
"""
import json
import numpy as np
from scipy.optimize import minimize

# Home 位置 (deg×100)，来自 motor_config.h
HOME_DEG100 = {0: -298, 1: 8059, 2: 13327, 3: 2700}

def motor_pos_to_rad(motor_pos_deg100, motor_idx):
    """电机绝对位置 → 相对home的关节角度(rad)"""
    relative_deg100 = motor_pos_deg100 - HOME_DEG100[motor_idx]
    return relative_deg100 / 100.0 * (np.pi / 180.0)

# ── 加载数据 ───────────────────────────────────────────────
with open('../captures.jsonl') as f:
    samples = [json.loads(l) for l in f]

Q = []       # 关节角度矩阵 [q1, q2, q3]
TAU = []     # 测量力矩矩阵 [τ1, τ2, τ3]

for s in samples:
    row = s  # 扁平：motors.*.angle, joints.*.torque
    q1 = motor_pos_to_rad(row['motors.1.angle'], 1)
    q2 = motor_pos_to_rad(row['motors.2.angle'], 2)
    q3 = motor_pos_to_rad(row['motors.3.angle'], 3)

    tau1 = row['joints.2.torque']   # 肩关节力矩
    tau2 = row['joints.3.torque']   # 肘关节力矩
    tau3 = row['joints.4.torque']   # 腕关节力矩

    Q.append([q1, q2, q3])
    TAU.append([tau1, tau2, tau3])

Q   = np.array(Q)
TAU = np.array(TAU)
print(f"样本数: {len(Q)}")

# ── 构建回归矩阵 ─────────────────────────────────────────────
# τ_g1 = A1·cos(q1+φ1) + A2·cos(q1+q2+φ2) + A3·cos(q1+q2+q3+φ3)
# τ_g2 = B1·cos(q1+q2+ψ1) + B2·cos(q1+q2+q3+ψ2)
# τ_g3 = C1·cos(q1+q2+q3+ζ1)
#
# 先令 φ=0 做初始线性最小二乘（快速方法）

q1 = Q[:, 0]
q2 = Q[:, 1]不
    np.cos(q1 + q2),
    np.cos(q1 + q2 + q3)
])

# Joint2 (肘): [cos(q1+q2), cos(q1+q2+q3)]
W2 = np.column_stack([
    np.cos(q1 + q2),
    np.cos(q1 + q2 + q3)
])

# Joint3 (腕): [cos(q1+q2+q3)]
W3 = np.cos(q1 + q2 + q3).reshape(-1, 1)

# 线性最小二乘
theta1, res1, _, _ = np.linalg.lstsq(W1, TAU[:, 0], rcond=None)
theta2, res2, _, _ = np.linalg.lstsq(W2, TAU[:, 1], rcond=None)
theta3, res3, _, _ = np.linalg.lstsq(W3, TAU[:, 2], rcond=None)

A1, A2, A3 = theta1
B1, B2     = theta2
C1         = theta3[0]

# ── 评估拟合质量 ─────────────────────────────────────────────
tau1_pred = W1 @ theta1
tau2_pred = W2 @ theta2
tau3_pred = W3 @ theta3

mae1 = np.mean(np.abs(TAU[:, 0] - tau1_pred))
mae2 = np.mean(np.abs(TAU[:, 1] - tau2_pred))
mae3 = np.mean(np.abs(TAU[:, 2] - tau3_pred))

r2_1 = 1 - np.sum((TAU[:, 0] - tau1_pred)**2) / np.sum((TAU[:, 0] - np.mean(TAU[:, 0]))**2)
r2_2 = 1 - np.sum((TAU[:, 1] - tau2_pred)**2) / np.sum((TAU[:, 1] - np.mean(TAU[:, 1]))**2)
r2_3 = 1 - np.sum((TAU[:, 2] - tau3_pred)**2) / np.sum((TAU[:, 2] - np.mean(TAU[:, 2]))**2)

print(f"\n=== 拟合结果 ===")
print(f"Joint1(肩): A1={A1:.4f}, A2={A2:.4f}, A3={A3:.4f}  MAE={mae1:.3f}Nm  R²={r2_1:.3f}")
print(f"Joint2(肘): B1={B1:.4f}, B2={B2:.4f}             MAE={mae2:.3f}Nm  R²={r2_2:.3f}")
print(f"Joint3(腕): C1={C1:.4f}                           MAE={mae3:.3f}Nm  R²={r2_3:.3f}")

# ── 验收判定 ─────────────────────────────────────────────────
print(f"\n=== 验收判定 ===")
PASS = True
for i, (mae, r2, name) in enumerate([(mae1, r2_1, '肩'), (mae2, r2_2, '肘'), (mae3, r2_3, '腕')]):
    ok = mae < 0.3 and r2 > 0.85
    status = "✅ PASS" if ok else "❌ FAIL"
    print(f"  {name}关节: MAE={mae:.3f}Nm (<0.3), R²={r2:.3f} (>0.85)  {status}")
    if not ok:
        PASS = False

if not PASS:
    print("\n⚠️ 部分关节拟合质量不足，建议:")
    print("   1. 检查 joints.x.torque 单位是否为 Nm")
    print("   2. 增加采集数据（特别是关节大角度位置）")
    print("   3. 参考 Step 1.3 加入角度偏置优化")

# ── 生成 STM32 头文件 ─────────────────────────────────────────
output = f"""/* gravity_params.h — AUTO GENERATED by gravity_identification.py
 * Date: $(date)
 * Fitting MAE: Joint1={mae1:.3f}Nm  Joint2={mae2:.3f}Nm  Joint3={mae3:.3f}Nm
 * R²:          Joint1={r2_1:.3f}    Joint2={r2_2:.3f}    Joint3={r2_3:.3f}
 *
 * Model:
 *   tau_g1 = A1*cos(q1+PHI1) + A2*cos(q1+q2+PHI2) + A3*cos(q1+q2+q3+PHI3)
 *   tau_g2 = B1*cos(q1+q2+PSI1) + B2*cos(q1+q2+q3+PSI2)
 *   tau_g3 = C1*cos(q1+q2+q3+ZETA1)
 */
#ifndef GRAVITY_PARAMS_H
#define GRAVITY_PARAMS_H

#define GC_A1    {A1:.6f}f
#define GC_A2    {A2:.6f}f
#define GC_A3    {A3:.6f}f
#define GC_PHI1  0.0f
#define GC_PHI2  0.0f
#define GC_PHI3  0.0f

#define GC_B1    {B1:.6f}f
#define GC_B2    {B2:.6f}f
#define GC_PSI1  0.0f
#define GC_PSI2  0.0f

#define GC_C1    {C1:.6f}f
#define GC_ZETA1 0.0f

/* 符号方向: +1=补偿方向正确, -1=需要取反 (硬件调试后确认) */
#define GC_SIGN_J0   0
#define GC_SIGN_J1  +1
#define GC_SIGN_J2  +1
#define GC_SIGN_J3  +1

/* 安全限幅 (Nm) — 不超过 tau_ff 上限的 60% */
#define GC_MAX_TFF   10.0f

#endif /* GRAVITY_PARAMS_H */
"""

with open('../User/GravityComp/gravity_params.h', 'w') as f:
    f.write(output)
print(f"\n✅ 已生成: User/GravityComp/gravity_params.h")
```

---

### ⭐ 阶段二：STM32 C 实现

**目标**：在 STM32 中实现 `Gravity_Compute()` 并集成到现有控制循环。

**执行时间**：约 2-3 小时（含编译调试）

#### Step 2.1 新建文件：`User/GravityComp/gravity_comp.h`

```c
#ifndef GRAVITY_COMP_H
#define GRAVITY_COMP_H

#include "gravity_params.h"   /* 由 Python 脚本自动生成 */

/**
 * @brief  计算所有关节的静态重力补偿力矩
 * @param  q    关节角度数组 [q0, q1, q2, q3], 相对 HOME 位置 (rad)
 *              q0=base(yaw), q1=shoulder, q2=elbow, q3=wrist
 * @param  tau  输出: 重力补偿力矩 [τ0, τ1, τ2, τ3] (Nm), 已含符号
 */
void Gravity_Compute(const float q[4], float tau[4]);

/**
 * @brief  获取单关节重力补偿力矩（含安全限幅）
 * @param  joint_idx  0~3
 * @param  q          关节角度数组 (rad, 相对HOME)
 * @return tau_ff (Nm), 已限幅至 [-GC_MAX_TFF, +GC_MAX_TFF]
 */
float Gravity_Get_Tff(int joint_idx, const float q[4]);

/**
 * @brief  从 g_motor_state 读取角度并更新全局缓冲区 g_gravity_tff[]
 *         在主循环中以 ≥ 200 Hz 调用
 */
void Gravity_Update_Buffer(void);

/* 全局力矩缓冲区（供 TIM4 ISR 直接读取，避免在 ISR 中调用 cosf） */
extern volatile float g_gravity_tff[4];

#endif /* GRAVITY_COMP_H */
```

#### Step 2.2 新建文件：`User/GravityComp/gravity_comp.c`

```c
#include "gravity_comp.h"
#include "motor_types.h"    /* g_motor_state[].angle */
#include <math.h>

volatile float g_gravity_tff[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/* ---------------------------------------------------------- */
void Gravity_Compute(const float q[4], float tau[4])
{
    /* q[0] = base yaw  — 重力力矩为零 */
    /* q[1] = shoulder  — 主要重力载荷 */
    /* q[2] = elbow     — 次要重力载荷 */
    /* q[3] = wrist     — 远端重力载荷 */

    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    /* 复合角（预计算，减少 cosf 调用次数） */
    float ang_12  = q1 + q2;
    float ang_123 = q1 + q2 + q3;

    tau[0] = 0.0f;  /* base joint: 竖直旋转轴，无重力力矩 */

    /* 肩关节 */
    tau[1] = (float)GC_SIGN_J1 * (
        GC_A1 * cosf(q1      + GC_PHI1) +
        GC_A2 * cosf(ang_12  + GC_PHI2) +
        GC_A3 * cosf(ang_123 + GC_PHI3)
    );

    /* 肘关节 */
    tau[2] = (float)GC_SIGN_J2 * (
        GC_B1 * cosf(ang_12  + GC_PSI1) +
        GC_B2 * cosf(ang_123 + GC_PSI2)
    );

    /* 腕关节 */
    tau[3] = (float)GC_SIGN_J3 * (
        GC_C1 * cosf(ang_123 + GC_ZETA1)
    );
}

/* ---------------------------------------------------------- */
float Gravity_Get_Tff(int joint_idx, const float q[4])
{
    float tau[4];
    Gravity_Compute(q, tau);
    float tff = tau[joint_idx];

    if (tff >  GC_MAX_TFF) tff =  GC_MAX_TFF;
    if (tff < -GC_MAX_TFF) tff = -GC_MAX_TFF;
    return tff;
}

/* ---------------------------------------------------------- */
/* 主循环调用: 读 g_motor_state 并刷新缓冲区                  */
void Gravity_Update_Buffer(void)
{
    /* 将电机绝对角度转换为相对 HOME 的关节角度 */
    extern MotorState g_motor_state[MOTOR_NUM];  /* motor_types.h */

    /* g_motor_state[i].angle 单位需确认（rad 或 deg×100）       */
    /* 若为 deg×100: q[i] = g_motor_state[i].angle / 100.0f * (PI/180.0f) - HOME_RAD[i] */
    /* 若为 rad:     q[i] = g_motor_state[i].angle - HOME_RAD[i]                         */
    static const float HOME_RAD[4] = {
        -298.0f  / 100.0f * 0.017453f,   /* Joint0: -0.0520 rad */
         8059.0f / 100.0f * 0.017453f,   /* Joint1: +1.4072 rad */
         13327.0f/ 100.0f * 0.017453f,   /* Joint2: +2.3261 rad */
         2700.0f / 100.0f * 0.017453f    /* Joint3: +0.4712 rad */
    };

    float q[4];
    for (int i = 0; i < 4; i++) {
        /* ⚠️ 调试时需确认 angle 单位: 见 Step 3.1 调试协议 */
        q[i] = g_motor_state[i].angle - HOME_RAD[i];
    }

    float tau[4];
    Gravity_Compute(q, tau);

    /* 写缓冲区（原子操作足够: float 写在 ARM Cortex-M3 上是原子的） */
    for (int i = 0; i < 4; i++) {
        float clamped = tau[i];
        if (clamped >  GC_MAX_TFF) clamped =  GC_MAX_TFF;
        if (clamped < -GC_MAX_TFF) clamped = -GC_MAX_TFF;
        g_gravity_tff[i] = clamped;
    }
}
```

#### Step 2.3 修改 `motor_control.c` — 运动阶段

定位 `Move_Motor_To_Target()` 中的力矩发送调用，替换静态 TFF：

```c
/* ─── BEFORE ─────────────────────────────────────── */
float tff = Get_Move_Tff(idx, dist);
Motor_MIT_Send_Raw(idx, pos_cmd, vel_cmd, kp, kd, tff);

/* ─── AFTER ──────────────────────────────────────── */
/* 使用预计算的重力补偿力矩（g_gravity_tff 由主循环刷新） */
float tff = g_gravity_tff[idx];
Motor_MIT_Send_Raw(idx, pos_cmd, vel_cmd, kp, kd, tff);
```

对 `Move_Four_Motors_To_Targets()` 及其他多轴函数做同样替换。

#### Step 2.4 修改 `motor_hold_timer.c` — TIM4 ISR

TIM4 ISR 优先级最高 (priority 0)，**不得在其中调用 cosf**：

```c
/* ─── BEFORE ─────────────────────────────────────── */
float tff = Get_Extreme_Hold_Tff(idx);
Motor_MIT_Send_Raw_NoPostDelay(idx, hold_pos, 0.0f, kp_hold, kd_hold, tff);

/* ─── AFTER ──────────────────────────────────────── */
/* 直接读预计算缓冲区（无浮点三角计算开销） */
float tff = g_gravity_tff[idx];
Motor_MIT_Send_Raw_NoPostDelay(idx, hold_pos, 0.0f, kp_hold, kd_hold, tff);
```

#### Step 2.5 修改 `main.c` — 主循环刷新

在主循环中加入缓冲区刷新（与现有 DMA 轮询并列）：

```c
/* main.c while(1) 循环中增加（约每 5ms 调用一次，与控制周期匹配） */
Gravity_Update_Buffer();
```

#### Step 2.6 CMake/Keil 工程配置

在 Keil MDK 工程中：
1. 添加 `User/GravityComp/` 文件夹到 Source Group
2. 添加 `gravity_comp.c` 到编译列表
3. 确认 `math.h` 链接了 `m` 库（GCC 下需 `-lm`）
4. 确认编译器使用 `--cpu Cortex-M3 --fpu=none`（软浮点）

---

### ⭐ 阶段三：调试与验收

#### Step 3.1 上电调试协议（第一次运行前必做）

**安全第一：先以10%增益验证符号方向**

```c
/* gravity_params.h 临时修改（调试阶段） */
#define GC_A1  (原值 * 0.1f)   /* 先用10%增益 */
#define GC_B1  (原值 * 0.1f)
#define GC_C1  (原值 * 0.1f)
```

操作步骤：
1. 机械臂移至 home 位置，切换为 hold 模式
2. **手动降低 Kp** 至正常值的 20%（stiffness 减弱，使重力效果可观察）
3. 观察机械臂是否有意外方向移动
4. 若关节向预期方向轻微抬起 → 符号正确，逐步增加到100%
5. 若关节向下坠 → `GC_SIGN_Ji` 取反后重试

**确认 g_motor_state[i].angle 的单位：**

```c
/* 在 fb_report_timer.c 的调试输出中临时增加: */
sprintf(buf, "ANGLE_DEBUG: %.4f %.4f %.4f %.4f\r\n",
    g_motor_state[0].angle, g_motor_state[1].angle,
    g_motor_state[2].angle, g_motor_state[3].angle);
Serial_SendString(buf);
/* 对比树莓派下发的 a0/a1(deg×100) 与这里的输出，推断单位 */
```

#### Step 3.2 定量验收测试（5个测试位姿）

| 测试编号 | 配置 | 目的 |
|---------|------|------|
| T1 | 所有关节 = 0 (home) | 基准测试 |
| T2 | 肩关节 +45°，其他 = home | 肩关节单独补偿验证 |
| T3 | 肩关节 −45°，其他 = home | 负向补偿验证 |
| T4 | 大臂展开（肩+45°，肘+60°） | 多关节耦合 |
| T5 | 随机配置（来自 captures.jsonl 中未用于训练的样本） | 泛化性 |

**测量方法（在树莓派端）：**

```python
# 在每个测试位姿静止5秒后，采集20次力矩反馈均值
# 对比: 残差力矩 = joints.x.torque (有补偿时的输出)
# 期望: 残差 ≈ 0
```

**通过标准（硬性指标）：**

| 指标 | 通过阈值 | 说明 |
|------|---------|------|
| 各关节残差力矩 | < 0.3 Nm | 补偿后静态残余 |
| 弱Kp(20%)下静态位置误差 | < 0.05 rad (≈2.9°) | 验证补偿有效性 |
| 关节1(肩) R² | > 0.85 | 模型解释力 |
| STM32 CPU 占用增量 | < 1% | `Gravity_Update_Buffer()` 开销 |
| 无异常振荡或发散 | 100%测试位姿通过 | 符号/幅值安全性 |

#### Step 3.3 参数微调流程图

```
启动测试
    │
    ├─ 残差有固定偏置? ─── YES ──→ 调整角度偏置 PHI / PSI / ZETA（+/- 0.1 rad 步进）
    │
    ├─ 残差随位置系统变化? ─ YES ─→ 重新运行 gravity_identification.py 采集更多样本
    │
    ├─ 某关节整体过大/过小? ─ YES ─→ 调整对应 A1/B1/C1 系数 (±20% 步进)
    │
    └─ 所有通过? ─── YES ──→ ✅ 完成，提交代码
```

---

## 五、风险清单与对策

| 风险 | 概率 | 影响 | 对策 |
|------|------|------|------|
| **captures.jsonl 中无有效力矩数据** | 中 | 需重新采集 | 参考附录A，20分钟采集30个位置 |
| **g_motor_state[i].angle 单位不明** | 中 | 角度计算偏差 | Step 3.1 调试协议强制验证 |
| **符号方向反向** | 中 | 正反馈发散 | 10%增益+低Kp先验证符号 |
| **STM32软浮点cosf太慢** | 低 | 主循环超载 | 若>1ms，改为32点余弦查表 |
| **URDF惯性与实物不符** | 中 | 初始参数偏差 | 数据校准可修正，不影响结构 |
| **74样本不足以覆盖关节空间** | 中 | 部分位置误差大 | 额外采集30个极端位置样本 |

---

## 六、工作量与时间线

| 阶段 | 工作内容 | 负责人 | 工时 |
|------|----------|--------|------|
| P1.1 | 验证数据格式 (verify_captures.py) | 树莓派 | 0.5h |
| P1.2 | 运行参数辨识脚本 | 树莓派 | 1h |
| P1.3 | 审核拟合结果，补充数据（若需要） | 人工 | 1h |
| P2.1 | 创建 gravity_comp.h/.c | STM32 | 0.5h |
| P2.2 | 修改 motor_control.c | STM32 | 1h |
| P2.3 | 修改 motor_hold_timer.c + main.c | STM32 | 0.5h |
| P2.4 | Keil 工程配置 + 编译通过 | STM32 | 0.5h |
| P3.1 | 符号验证调试（10%增益） | 硬件 | 1h |
| P3.2 | 全部验收测试通过 | 硬件 | 2h |
| **总计** | | | **~8h** |

---

## 附录A：重新采集力矩数据协议（若阶段一验证失败）

在树莓派采集脚本中，MIT 反馈帧解包后应包含三个字段：
- `position` (rad) — 当前位置
- `velocity` (rad/s) — 当前速度
- `torque` (Nm) — **这才是力矩**

修改采集脚本，将 `torque` 字段写入 captures_v2.jsonl，采集规则：
1. 在关节空间网格采样：q1 ∈ {−60°,−30°, 0°, 30°, 60°}，q2 ∈ {−45°, 0°, 45°}，q3 ∈ {−30°, 0°, 30°}
2. 每个姿态静止5秒，采集均值（同 captures.jsonl 的 settle_sec 逻辑）
3. 最少30个有效样本，建议60个以上

---

## 附录B：cos 查表法（备用，当软浮点开销不可接受时）

```c
/* 32点余弦查表，精度 ≈ 0.004 rad */
static const float COS_TABLE[32] = {
    1.000f,  0.981f,  0.924f,  0.831f,
    0.707f,  0.556f,  0.383f,  0.195f,
    0.000f, -0.195f, -0.383f, -0.556f,
   -0.707f, -0.831f, -0.924f, -0.981f,
   -1.000f, -0.981f, -0.924f, -0.831f,
   -0.707f, -0.556f, -0.383f, -0.195f,
    0.000f,  0.195f,  0.383f,  0.556f,
    0.707f,  0.831f,  0.924f,  0.981f
};

static inline float fast_cosf(float angle) {
    /* 归一化到 [0, 2π) */
    while (angle < 0)         angle += 6.2832f;
    while (angle >= 6.2832f)  angle -= 6.2832f;
    int idx = (int)(angle / 6.2832f * 32.0f) & 31;
    return COS_TABLE[idx];
}
```
