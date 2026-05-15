/**
 * 步进电机：PA0=STEP(TIM2_CH1 PWM)，PA1=DIR(GPIO)
 * 共阴极；4 细分，800 脉冲/转
 */
#include "Stepper.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#define STEPPER_STEP_GPIO_PORT      GPIOA
#define STEPPER_STEP_GPIO_PIN       GPIO_Pin_0
#define STEPPER_DIR_GPIO_PORT       GPIOA
#define STEPPER_DIR_GPIO_PIN        GPIO_Pin_1

#define STEPPER_PWM_TIM             TIM2
#define STEPPER_PWM_TIM_RCC         RCC_APB1Periph_TIM2
#define STEPPER_PWM_CHANNEL         TIM_Channel_1

/* 脉冲频率上下限（Hz），可按驱动器手册调整 */
#define STEPPER_STEP_HZ_MIN         1u
#define STEPPER_STEP_HZ_MAX         50000u

static uint32_t s_step_hz;
static uint8_t  s_dir_forward = 1u;
static uint8_t  s_tim_inited;

static uint32_t stepper_tim_clk_hz(void)
{
    RCC_ClocksTypeDef clk;

    RCC_GetClocksFreq(&clk);
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        return clk.PCLK1_Frequency * 2u;
    }
    return clk.PCLK1_Frequency;
}

static void stepper_apply_pwm_hz(uint32_t hz)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;
    uint32_t tim_clk;
    uint32_t target_ticks;
    uint32_t psc;
    uint32_t arr;
    uint16_t pulse;

    if (!s_tim_inited) {
        return;
    }

    if (hz == 0u) {
        TIM_CCxCmd(STEPPER_PWM_TIM, STEPPER_PWM_CHANNEL, TIM_CCx_Disable);
        return;
    }

    tim_clk = stepper_tim_clk_hz();
    target_ticks = tim_clk / hz;
    if (target_ticks < 2u) {
        return;
    }

    psc = (target_ticks + 65535u) / 65536u;
    if (psc < 1u) {
        psc = 1u;
    }
    if (psc > 65536u) {
        psc = 65536u;
    }

    arr = (target_ticks + psc - 1u) / psc;
    if (arr < 2u) {
        arr = 2u;
    }
    if (arr > 65536u) {
        arr = 65536u;
    }

    pulse = (uint16_t)((arr + 1u) / 2u);
    if (pulse < 1u) {
        pulse = 1u;
    }

    TIM_ITConfig(STEPPER_PWM_TIM, TIM_IT_Update, DISABLE);

    tb.TIM_Prescaler         = (uint16_t)(psc - 1u);
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = (uint16_t)(arr - 1u);
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(STEPPER_PWM_TIM, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = pulse;
    TIM_OC1Init(STEPPER_PWM_TIM, &oc);

    TIM_OC1PreloadConfig(STEPPER_PWM_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(STEPPER_PWM_TIM, ENABLE);

    TIM_SetCounter(STEPPER_PWM_TIM, 0);
    TIM_ClearFlag(STEPPER_PWM_TIM, TIM_FLAG_Update);
    TIM_CCxCmd(STEPPER_PWM_TIM, STEPPER_PWM_CHANNEL, TIM_CCx_Enable);
    TIM_Cmd(STEPPER_PWM_TIM, ENABLE);
}

void Stepper_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;

    s_step_hz = 0u;
    s_dir_forward = 1u;
    s_tim_inited = 0u;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(STEPPER_PWM_TIM_RCC, ENABLE);

    /*
     * PA1 = DIR，普通推挽输出（勿配置为 TIM2_CH2）。
     * 默认拉高：正向；需要反向时调用 Stepper_SetDirection(0) 拉低。
     */
    gpio.GPIO_Pin   = STEPPER_DIR_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(STEPPER_DIR_GPIO_PORT, &gpio);
    GPIO_SetBits(STEPPER_DIR_GPIO_PORT, STEPPER_DIR_GPIO_PIN);

    /* PA0 = STEP，TIM2_CH1 复用推挽（F103C8 上 PA0 对应 TIM2_CH1，见数据手册 AF 表） */
    gpio.GPIO_Pin   = STEPPER_STEP_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(STEPPER_STEP_GPIO_PORT, &gpio);

    TIM_DeInit(STEPPER_PWM_TIM);
    TIM_InternalClockConfig(STEPPER_PWM_TIM);

    tb.TIM_Prescaler         = 0;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = 0xFFFF;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(STEPPER_PWM_TIM, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = 0;
    TIM_OC1Init(STEPPER_PWM_TIM, &oc);

    TIM_OC1PreloadConfig(STEPPER_PWM_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(STEPPER_PWM_TIM, ENABLE);
    TIM_ITConfig(STEPPER_PWM_TIM, TIM_IT_Update, DISABLE);

    TIM_SetCounter(STEPPER_PWM_TIM, 0);
    TIM_ClearFlag(STEPPER_PWM_TIM, TIM_FLAG_Update);
    TIM_CCxCmd(STEPPER_PWM_TIM, STEPPER_PWM_CHANNEL, TIM_CCx_Disable);
    TIM_Cmd(STEPPER_PWM_TIM, ENABLE);

    s_tim_inited = 1u;
}

void Stepper_SetDirection(uint8_t forward)
{
    s_dir_forward = forward ? 1u : 0u;
    if (s_dir_forward) {
        GPIO_SetBits(STEPPER_DIR_GPIO_PORT, STEPPER_DIR_GPIO_PIN);
    } else {
        GPIO_ResetBits(STEPPER_DIR_GPIO_PORT, STEPPER_DIR_GPIO_PIN);
    }
}

uint8_t Stepper_GetDirection(void)
{
    return s_dir_forward;
}

void Stepper_SetStepFrequencyHz(uint32_t hz)
{
    if (hz > STEPPER_STEP_HZ_MAX) {
        hz = STEPPER_STEP_HZ_MAX;
    } else if (hz != 0u && hz < STEPPER_STEP_HZ_MIN) {
        hz = STEPPER_STEP_HZ_MIN;
    }

    s_step_hz = hz;
    stepper_apply_pwm_hz(hz);
}

uint32_t Stepper_GetStepFrequencyHz(void)
{
    return s_step_hz;
}

void Stepper_SetSpeedRPM(float rpm)
{
    float hz_f;
    uint32_t hz;

    if (rpm <= 0.0f) {
        Stepper_SetStepFrequencyHz(0u);
        return;
    }

    hz_f = rpm * (float)STEPPER_PULSES_PER_REV / 60.0f;
    if (hz_f > (float)STEPPER_STEP_HZ_MAX) {
        hz = STEPPER_STEP_HZ_MAX;
    } else {
        hz = (uint32_t)(hz_f + 0.5f);
    }
    Stepper_SetStepFrequencyHz(hz);
}

float Stepper_GetSpeedRPM(void)
{
    if (s_step_hz == 0u) {
        return 0.0f;
    }
    return (float)s_step_hz * 60.0f / (float)STEPPER_PULSES_PER_REV;
}
