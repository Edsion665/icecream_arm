#include "fb_report_timer.h"
#include "MotorControl/motor_config.h"
#include "MotorControl/motor_can.h"
#include "../Hardware/Serial.h"
#include "../Hardware/Serial2.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include <stdio.h>

#if GRAVITY_FF_PI_MODE
#include "MotorControl/gravity_pi_feedforward.h"
#endif

#if MOTOR_DEBUG_LOG_ENABLE
void FB_Report_SendLine(void)
{
    /*
     * CAN 反馈位置场为 16 位无符号；与 motor_utils.uint_to_float 一致：
     *   0～65535 线性映射到 [Runtime_P_Min, Runtime_P_Max]（默认 MIT ±12.5 rad，同步驱动器后以 Runtime_* 为准）。
     * Motor_States[i].pos 即为该状态空间下的弧度。
     */
#if GRAVITY_FF_PI_MODE
    char out[384];
    float tcmd[4];
    float tpi[4];

    GravityPi_GetLastMitTorqueCmd(tcmd);
    GravityPi_GetPiReceivedTau(tpi);
    snprintf(out, sizeof(out),
             "FB %.5f %.5f %.5f %.5f j1:%.3f j2:%.3f j3:%.3f j4:%.3f "
             "tcmd1:%.4f tcmd2:%.4f tcmd3:%.4f tcmd4:%.4f "
             "tpi1:%.4f tpi2:%.4f tpi3:%.4f tpi4:%.4f "
             "tau_ok:%lu tau_fail:%lu last_tau:%d\r\n",
             Motor_States[0].pos, Motor_States[1].pos,
             Motor_States[2].pos, Motor_States[3].pos,
             Motor_States[0].tor, Motor_States[1].tor,
             Motor_States[2].tor, Motor_States[3].tor,
             tcmd[0], tcmd[1], tcmd[2], tcmd[3],
             tpi[0], tpi[1], tpi[2], tpi[3],
             (unsigned long)GravityPi_GetTauParseOkCount(),
             (unsigned long)GravityPi_GetTauParseFailCount(),
             (int)GravityPi_GetLastTauParseResult());
#else
    char out[224];

    snprintf(out, sizeof(out),
             "FB %.5f %.5f %.5f %.5f j1:%.3f j2:%.3f j3:%.3f j4:%.3f\r\n",
             Motor_States[0].pos, Motor_States[1].pos,
             Motor_States[2].pos, Motor_States[3].pos,
             Motor_States[0].tor, Motor_States[1].tor,
             Motor_States[2].tor, Motor_States[3].tor);
#endif
    Serial_SendString(out);
    Serial2_SendString(out);
}
#else
void FB_Report_SendLine(void) { }
#endif

#if MOTOR_DEBUG_LOG_ENABLE || GRAVITY_FF_PI_MODE
void FB_Report_ServicePending(void)
{
    if (FB_ReportTimer_TakePending()) {
#if MOTOR_DEBUG_LOG_ENABLE
        FB_Report_SendLine();
#endif
#if GRAVITY_FF_PI_MODE
        GravityPi_ApplyAll();
#endif
    }
}
#else
void FB_Report_ServicePending(void) { }
#endif

#if MOTOR_DEBUG_LOG_ENABLE || GRAVITY_FF_PI_MODE
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
#else
void FB_ReportTimer_Init(void) { }
uint8_t FB_ReportTimer_TakePending(void) { return 0u; }
#endif
