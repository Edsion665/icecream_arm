#include "fb_report_timer.h"
#include "MotorControl/motor_config.h"
#include "MotorControl/motor_can.h"
#include "../Hardware/Serial.h"
#include "../Hardware/Serial2.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include <stdio.h>

#if MOTOR_DEBUG_LOG_ENABLE
void FB_Report_SendLine(void)
{
    uint8_t i;
    int16_t fb_raw[4];
    char out[80];
    static const float home_abs[4] = WORLD_HOME_ABS;
    const float rad2deg = 180.0f / MOTOR_PI;

    for (i = 0; i < 4; i++) {
        float rel_rad = Motor_States[i].pos - home_abs[i];
        float rel_deg = rel_rad * rad2deg;
        if (rel_deg > RPI_REL_DEG_LIMIT) {
            rel_deg = RPI_REL_DEG_LIMIT;
        } else if (rel_deg < -RPI_REL_DEG_LIMIT) {
            rel_deg = -RPI_REL_DEG_LIMIT;
        }
        fb_raw[i] = (int16_t)(rel_deg * 100.0f);
    }
    snprintf(out, sizeof(out), "FB %d %d %d %d\r\n",
             (int)fb_raw[0], (int)fb_raw[1], (int)fb_raw[2], (int)fb_raw[3]);
    Serial_SendString(out);
    Serial2_SendString(out);
}

void FB_Report_ServicePending(void)
{
    if (FB_ReportTimer_TakePending()) {
        FB_Report_SendLine();
    }
}
#else
void FB_Report_SendLine(void) { }
void FB_Report_ServicePending(void) { }
#endif

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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

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
    TIM_TimeBaseInit(TIM2, &tb);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_SetCounter(TIM2, 0);
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM2, ENABLE);
}

uint8_t FB_ReportTimer_TakePending(void)
{
    if (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) != RESET) {
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
        return 1u;
    }
    return 0u;
}
