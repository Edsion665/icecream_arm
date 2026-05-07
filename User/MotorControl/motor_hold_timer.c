#include "motor_hold_timer.h"
#include "motor_config.h"
#include "motor_control.h"
#include "motor_can.h"
#include "../../Hardware/Serial.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"

#if MOTOR_HOLD_TIM4_ENABLE
/* 定时器 ISR 只读此快照；主线程在 Publish / StreamExit(深度归零) 时更新 */
static float s_snap_p[MOTOR_NUM];
static uint8_t s_snap_h[MOTOR_NUM];

static void copy_snap_from_targets(void)
{
    int i;
    for (i = 0; i < MOTOR_NUM; i++) {
        s_snap_p[i] = Current_Targets[i];
        s_snap_h[i] = Motor_Homed[i];
    }
}
#endif

/* ISR 与主线程均访问；勿缓存 */
static volatile uint32_t s_stream_depth;

volatile uint32_t g_MotorHold_IsrApplyCount;
volatile uint32_t g_MotorHold_IsrSkipStreamCount;
volatile uint32_t g_MotorHold_IsrSkipEmgCount;

uint32_t MotorHoldTimer_GetStreamDepth(void)
{
    return (uint32_t)s_stream_depth;
}

void MotorHoldTimer_PublishSnapshot(void)
{
#if MOTOR_HOLD_TIM4_ENABLE
    NVIC_DisableIRQ(TIM4_IRQn);
    copy_snap_from_targets();
    NVIC_EnableIRQ(TIM4_IRQn);
#endif
}

void MotorHoldTimer_StreamEnter(void)
{
    __disable_irq();
    s_stream_depth++;
    __enable_irq();
}

void MotorHoldTimer_StreamExit(void)
{
    __disable_irq();
    if (s_stream_depth > 0u) {
        s_stream_depth--;
    }
#if MOTOR_HOLD_TIM4_ENABLE
    if (s_stream_depth == 0u) {
        copy_snap_from_targets();
    }
#endif
    __enable_irq();
}

#if MOTOR_HOLD_TIM4_ENABLE

static void MotorHoldTimer_OnTick(void)
{
    /* USART1 RX 走 DMA，在此拉串口数据，不提高串口 NVIC 抢占，避免饿死本 ISR */
    Serial_ServiceRxDma();

    if (Emergency_Stop || System_Disabled) {
        g_MotorHold_IsrSkipEmgCount++;
        return;
    }
    if (s_stream_depth > 0u) {
        g_MotorHold_IsrSkipStreamCount++;
        return;
    }
    g_MotorHold_IsrApplyCount++;
    Apply_Rigid_Hold_OnBuffers_NoPostDelay(s_snap_p, s_snap_h);
}

void MotorHoldTimer_Init(void)
{
    TIM_TimeBaseInitTypeDef tb;
    NVIC_InitTypeDef nvic;
    RCC_ClocksTypeDef clk;
    uint32_t timclk;
    uint32_t hz;
    uint32_t target_ticks;
    uint32_t psc;
    uint32_t arr;

    if ((uint32_t)INTERVAL_MS < 1u) {
        hz = 1000u;
    } else {
        hz = 1000u / (uint32_t)INTERVAL_MS;
    }
    if (hz < 1u) {
        hz = 1u;
    }

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    RCC_GetClocksFreq(&clk);
    timclk = clk.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        timclk *= 2u;
    }

    target_ticks = timclk / hz;
    if (target_ticks < 2u) {
        target_ticks = 2u;
    }

    psc = (target_ticks + 65535u - 1u) / 65535u;
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

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler     = (uint16_t)(psc - 1u);
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_Period        = (uint16_t)(arr - 1u);
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &tb);

    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_SetCounter(TIM4, 0);

    nvic.NVIC_IRQChannel                   = TIM4_IRQn;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    /*
     * 抢占 1：低于 CAN_RX(0)，CAN 仍可打断 MIT；高于 USART1(2)，TXE 洪泛不能饿死 MIT。
     * 勿与 CAN 同为抢占 0，否则 MIT ISR 执行期间无法响应 CAN 反馈。
     */
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&nvic);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        MotorHoldTimer_OnTick();
    }
}

#else /* !MOTOR_HOLD_TIM4_ENABLE */

void MotorHoldTimer_Init(void)
{
    /* MOTOR_HOLD_TIM4_ENABLE=0：不启动 TIM4，ISR 内不再下发刚性保持（试验用） */
}

void TIM4_IRQHandler(void)
{
    /* 未使能 TIM4 时不应进入；留空实现满足链接 */
}

#endif /* MOTOR_HOLD_TIM4_ENABLE */
