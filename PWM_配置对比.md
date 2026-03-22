# STM32 PWM 配置对比：标准库示例 vs 当前实现

## 1. 标准库官方示例（7PWM_Output，TIM1）

来源：STM32F10x_StdPeriph_Lib V3.5.0 / Project/STM32F10x_StdPeriph_Examples/TIM/7PWM_Output

### 关键配置流程
```c
// 1. 时钟：TIM1 在 APB2，需 AFIO
RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | 
    RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

// 2. GPIO：AF_PP，50MHz
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &GPIO_InitStructure);

// 3. 时基
TIM_TimeBaseStructure.TIM_Prescaler = 0;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

// 4. PWM 通道
TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  // 或 PWM1
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
TIM_OC1Init(TIM1, &TIM_OCInitStructure);
// ... OC2, OC3, OC4

// 5. 使能
TIM_Cmd(TIM1, ENABLE);
TIM_CtrlPWMOutputs(TIM1, ENABLE);  // ★ 仅高级定时器 TIM1/TIM8 需要 MOE
```

---

## 2. 当前 Servo.c（TIM3）配置

| 项目 | 标准库示例 | 当前 Servo | 说明 |
|------|------------|------------|------|
| 定时器 | TIM1 (APB2) | TIM3 (APB1) | TIM3 为通用定时器，无 MOE |
| 时钟使能 | RCC_APB2 | RCC_APB1(TIM3)+APB2(GPIOB+AFIO) | ✓ 正确 |
| GPIO 模式 | AF_PP, 50MHz | AF_PP, 50MHz | ✓ 一致 |
| TIM_DeInit | 无 | 有 | 用于清除残留配置 |
| GPIO_PinRemapConfig | 仅 CL 时用 | 显式 DISABLE | 可能多余 |
| TIM_InternalClockConfig | 无 | 有 | 显式选内部时钟，可选 |
| Preload (OC/ARR) | 无 | 有 | 可选，利于无毛刺更新 |
| TIM_CtrlPWMOutputs | 必须 | 不适用 | TIM2 无 MOE，无需调用 |

---

## 3. 可能的问题点

### 3.1 初始化顺序
- 标准库：RCC → GPIO → TIM_TimeBase → TIM_OCx → TIM_Cmd
- 当前：RCC → TIM_DeInit → GPIO_PinRemapConfig → GPIO → TIM_InternalClock → TIM_TimeBase → TIM_OCx → Preload → TIM_SetCompare → TIM_Cmd

建议：将 **GPIO 配置提前**，与标准库一致，并考虑去掉 `GPIO_PinRemapConfig` 测试。

### 3.2 GPIO_PinRemapConfig
- 默认映射下，PB0=TIM3_CH3、PB1=TIM3_CH4 无需重映射
- 无需调用 GPIO_PinRemapConfig

### 3.3 引脚冲突
- **Serial2_Init** 在 Servo_Init 之前执行，将 PA2 配置为 USART2_TX
- Servo 使用 PB0、PB1，与 Serial2 无直接冲突
- 若板子 PB1 接按键（如 Key 模块），可能冲突，需根据实际硬件选择

### 3.4 时钟计算
- 72MHz 系统时钟，APB1 预分频 /2 → PCLK1=36MHz，TIM3 时钟=72MHz
- psc=71, arr=19999 → 72M/72/20000 = 50Hz ✓（TIM3 同 TIM2，APB1 定时器）

---

## 4. 修改建议

1. **简化 Servo_Init**：按标准库顺序，先 GPIO 再 TIM 试运行
2. **PB0 GPIO 闪烁测试**：`PWM_PB0_GPIO_BLINK_TEST=1` 验证 PB0 硬件是否正常
3. **PB0/PB1 PWM 测试**：`PWM_PB01_TEST_ENABLE=1` 在舵机引脚直接输出 50Hz、1.5ms 脉宽
