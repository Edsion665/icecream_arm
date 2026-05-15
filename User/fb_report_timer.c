#include "fb_report_timer.h"
#include "MotorControl/motor_config.h"
#include "MotorControl/motor_can.h"
#include "../Hardware/Serial.h"
#include "../Hardware/Servo.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#if MOTOR_DEBUG_LOG_ENABLE || MIT_HEX_MODE
/*
 * 上行二进制帧：0xAA 0x55 + 4×8字节原始CAN + wrist_us(BE) + gripper_us(BE) + XOR = 39字节
 */
static void FB_Report_SendBinaryUplink(const Motor_Status_t snap[MOTOR_NUM])
{
    uint8_t frame[39];
    int i, b;
    uint8_t x;
    uint16_t wrist_us, gripper_us;

    frame[0] = 0xAA;
    frame[1] = 0x55;
    for (i = 0; i < 4; i++) {
        uint8_t *dst = &frame[2 + i * 8];
        for (b = 0; b < 8; b++) {
            dst[b] = (b < (int)snap[i].raw_dlc) ? snap[i].raw_frame[b] : 0x00;
        }
    }
    wrist_us   = Servo_GetCurrentWristUs();
    gripper_us = Servo_GetCurrentGripperUs();
    frame[34] = (uint8_t)(wrist_us >> 8);
    frame[35] = (uint8_t)(wrist_us & 0xFFu);
    frame[36] = (uint8_t)(gripper_us >> 8);
    frame[37] = (uint8_t)(gripper_us & 0xFFu);
    x = 0;
    for (i = 0; i < 38; i++) {
        x ^= frame[i];
    }
    frame[38] = x;
    Serial_SendArray(frame, 39);
}
#endif

#if MOTOR_DEBUG_LOG_ENABLE || MIT_HEX_MODE
void FB_Report_SendLine(void)
{
    int k;
    Motor_Status_t snap[MOTOR_NUM];

    for (k = 0; k < MOTOR_NUM; k++) {
        snap[k] = Motor_States[k];
    }
    FB_Report_SendBinaryUplink(snap);
}
#else
void FB_Report_SendLine(void) { }
#endif

#if MOTOR_DEBUG_LOG_ENABLE || MIT_HEX_MODE
void FB_Report_ServicePending(void)
{
    if (FB_ReportTimer_TakePending()) {
        FB_Report_SendLine();
    }
}
#else
void FB_Report_ServicePending(void) { }
#endif

#if MOTOR_DEBUG_LOG_ENABLE || MIT_HEX_MODE
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

    RCC_APB1PeriphClockCmd(FB_REPORT_TIM_PERIPH_RCC, ENABLE);
    TIM_DeInit(FB_REPORT_TIM_PERIPH);

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
    TIM_TimeBaseInit(FB_REPORT_TIM_PERIPH, &tb);

    TIM_ClearFlag(FB_REPORT_TIM_PERIPH, TIM_FLAG_Update);
    TIM_SetCounter(FB_REPORT_TIM_PERIPH, 0);
    TIM_ITConfig(FB_REPORT_TIM_PERIPH, TIM_IT_Update, DISABLE);
    TIM_Cmd(FB_REPORT_TIM_PERIPH, ENABLE);
}

uint8_t FB_ReportTimer_TakePending(void)
{
    if (TIM_GetFlagStatus(FB_REPORT_TIM_PERIPH, TIM_FLAG_Update) != RESET) {
        TIM_ClearFlag(FB_REPORT_TIM_PERIPH, TIM_FLAG_Update);
        return 1u;
    }
    return 0u;
}
#else
void FB_ReportTimer_Init(void) { }
uint8_t FB_ReportTimer_TakePending(void) { return 0u; }
#endif
