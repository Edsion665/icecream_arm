#include "fb_report_timer.h"
#include "MotorControl/motor_config.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

void FB_ReportTimer_Init(void)
{
    TIM_TimeBaseInitTypeDef tb;
    RCC_ClocksTypeDef clk;
    uint32_t timclk;
    uint32_t hz = (uint32_t)FB_REPORT_HZ;
    uint32_t target_ticks;
    uint32_t psc;
    uint32_t arr;

    if (hz < 1u) {
        hz = 1u;
    }

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    RCC_GetClocksFreq(&clk);
    timclk = clk.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        timclk *= 2u;
    }

    /* timclk / ((PSC+1)*(ARR+1)) == hz，PSC/ARR 均为 16 位 */
    target_ticks = timclk / hz;
    if (target_ticks < 2u) {
        target_ticks = 2u;
    }

    psc = (target_ticks + 65535u - 1u) / 65535u; /* ceil(target/65535) = PSC+1 下限 */
    if (psc < 1u) {
        psc = 1u;
    }
    if (psc > 65536u) {
        psc = 65536u;
    }

    arr = (target_ticks + psc - 1u) / psc; /* ceil(target/psc) = ARR+1 */
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
    TIM_TimeBaseInit(TIM3, &tb);

    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_SetCounter(TIM3, 0);
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM3, ENABLE);
}

uint8_t FB_ReportTimer_TakePending(void)
{
    if (TIM_GetFlagStatus(TIM3, TIM_FLAG_Update) != RESET) {
        TIM_ClearFlag(TIM3, TIM_FLAG_Update);
        return 1u;
    }
    return 0u;
}
