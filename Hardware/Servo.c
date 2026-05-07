/**
 * 舵机控制：PB0(腕部)、PB1(机械爪)
 * TIM3 CH3/CH4，50Hz，高电平 500~2500us
 */
#include "Servo.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

/*================ 可配置参数 ====================*/
#define SERVO_MIN_US        500u
#define SERVO_MAX_US        2500u

/* 腕部：1328us=零位，raw=0 对应 1328us，在此基础上加减（500us=-149°，2500us=+211°） */
#define SERVO_WRIST_ZERO_US   1328u
#define SERVO_WRIST_NEG_US    828u   /* 1328-500，负向行程 */
#define SERVO_WRIST_POS_US    1172u  /* 2500-1328，正向行程 */
#define SERVO_WRIST_RAW_SPAN  18000u /* raw ±18000 对应负/正向满行程 */

/* 机械爪：1500us=0°，对称 */
#define SERVO_GRIPPER_CENTER_US 1500u

/* 目标：50Hz = 20ms 周期，1us 分辨率 → 20000 ticks/周期 */
#define SERVO_PERIOD_US      20000u

/* 腕部：raw=0→1328us(零位)，以 1328 为基准，raw 正负在 1328 基础上加减 */
static inline uint16_t raw_to_us_wrist(int16_t raw)
{
    int32_t v = (int32_t)raw;
    uint16_t us;
    if (v < 0)
        us = (uint16_t)(SERVO_WRIST_ZERO_US + (int32_t)v * (int32_t)SERVO_WRIST_NEG_US / (int32_t)SERVO_WRIST_RAW_SPAN);
    else
        us = (uint16_t)(SERVO_WRIST_ZERO_US + (int32_t)v * (int32_t)SERVO_WRIST_POS_US / (int32_t)SERVO_WRIST_RAW_SPAN);
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    return us;
}

/* 机械爪：无输入限幅，0→1500us，仅输出限幅 500~2500us */
static inline uint16_t raw_to_us_gripper(int16_t raw)
{
    int32_t v = (int32_t)raw;
    uint16_t us = (uint16_t)(SERVO_GRIPPER_CENTER_US + (int32_t)v * 1000 / 18000);
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    return us;
}

/* 目标与当前（用于渐变），初值由 Servo_Init 统一赋值 */
static uint16_t s_target_wrist_us;
static uint16_t s_target_gripper_us;
static uint16_t s_current_wrist_us;
static uint16_t s_current_gripper_us;
static uint8_t  s_ramp_speed = 50u;  /* 0~100，默认中等 */

void Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCStructure;
    RCC_ClocksTypeDef RCC_Clocks;
    uint32_t tim_clk;
    uint32_t psc, arr;

    /* 标准库示例顺序：1) 时钟 2) GPIO 3) 时基 4) OC 5) Cmd */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    /* GPIO 配置（PB0=TIM3_CH3，PB1=TIM3_CH4，AF_PP, 50MHz） */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 清除 TIM3 残留配置（标准库示例无，但多模块共用时建议保留） */
    TIM_DeInit(TIM3);

    /* 运行时获取 TIM3 时钟：APB1 预分频>1 时，定时器时钟 = PCLK1*2 */
    RCC_GetClocksFreq(&RCC_Clocks);
    tim_clk = RCC_Clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        tim_clk *= 2u;
    }

    /* 1us/tick：PSC 使 tim_clk/(PSC+1)=1MHz；ARR=SERVO_PERIOD_US-1 → 20ms */
    psc = tim_clk / 1000000u;
    if (psc < 1u) psc = 1u;
    psc -= 1u;
    arr = SERVO_PERIOD_US - 1u;
    if (arr > 65535u) arr = 65535u;

    TIM_InternalClockConfig(TIM3);

    TIM_TimeBaseStructure.TIM_Prescaler         = (uint16_t)psc;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = (uint16_t)arr;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCStructure);
    TIM_OCStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCStructure.TIM_Pulse       = SERVO_WRIST_ZERO_US;  /* 腕部 CH3 初值 1328us */
    TIM_OC3Init(TIM3, &TIM_OCStructure);
    TIM_OCStructure.TIM_Pulse       = SERVO_GRIPPER_CENTER_US;  /* 机械爪 CH4 初值 1500us */
    TIM_OC4Init(TIM3, &TIM_OCStructure);

    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    /* 腕部：1328us 为零位，初值 1328us 即 0° */
    s_target_wrist_us   = SERVO_WRIST_ZERO_US;
    s_current_wrist_us  = SERVO_WRIST_ZERO_US;
    s_target_gripper_us = SERVO_GRIPPER_CENTER_US;
    s_current_gripper_us= SERVO_GRIPPER_CENTER_US;

    TIM_SetCompare3(TIM3, SERVO_WRIST_ZERO_US);
    TIM_SetCompare4(TIM3, SERVO_GRIPPER_CENTER_US);

    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

void Servo_SetWrist(int16_t raw)
{
    s_target_wrist_us = raw_to_us_wrist(raw);
}

void Servo_SetGripper(int16_t raw)
{
    s_target_gripper_us = raw_to_us_gripper(raw);
}

void Servo_SetWristUs(uint16_t us)
{
    if (us < SERVO_MIN_US) {
        us = SERVO_MIN_US;
    }
    if (us > SERVO_MAX_US) {
        us = SERVO_MAX_US;
    }
    s_target_wrist_us = us;
}

void Servo_SetGripperUs(uint16_t us)
{
    if (us < SERVO_MIN_US) {
        us = SERVO_MIN_US;
    }
    if (us > SERVO_MAX_US) {
        us = SERVO_MAX_US;
    }
    s_target_gripper_us = us;
}

uint16_t Servo_GetCurrentWristUs(void)
{
    return s_current_wrist_us;
}

uint16_t Servo_GetCurrentGripperUs(void)
{
    return s_current_gripper_us;
}

void Servo_SetRampSpeed(uint8_t speed)
{
    s_ramp_speed = (speed > 100u) ? 100u : speed;
}

static void ramp_toward(uint16_t *current, uint16_t target, uint8_t speed)
{
    uint16_t cur = *current;
    if (cur == target) return;
    if (speed >= 100u) {
        *current = target;
        return;
    }
    /* 步长：约 1~50 us/次，由 speed 决定 */
    uint16_t step = 1u + (uint16_t)speed * 50u / 100u;
    if (cur < target) {
        if (target - cur <= step)
            cur = target;
        else
            cur += step;
    } else {
        if (cur - target <= step)
            cur = target;
        else
            cur -= step;
    }
    *current = cur;
}

void Servo_Update(void)
{
    ramp_toward(&s_current_wrist_us,   s_target_wrist_us,   s_ramp_speed);
    ramp_toward(&s_current_gripper_us, s_target_gripper_us, s_ramp_speed);
    TIM_SetCompare3(TIM3, s_current_wrist_us);
    TIM_SetCompare4(TIM3, s_current_gripper_us);
}
