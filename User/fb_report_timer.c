#include "fb_report_timer.h"
#include "MotorControl/motor_config.h"
#include "MotorControl/motor_can.h"
#include "../Hardware/Serial.h"
#include "../Hardware/Servo.h"
#include "../Hardware/Stepper.h"
#include "../Hardware/Conveyor.h"
#include "serial_frame.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#if MOTOR_DEBUG_LOG_ENABLE || MIT_HEX_MODE
/*
 * 上行 v3：42 字节，布局见 pi2stm.md / serial_frame.h
 */
static void FB_Report_SendBinaryUplink(const Motor_Status_t snap[MOTOR_NUM])
{
    uint8_t frame[RPI_BIN_FRAME_LEN];
    int i, b;
    uint16_t wrist_us, gripper_us;
    int16_t stepper_deg;

    frame[RPI_OFF_HEADER0] = 0xAA;
    frame[1] = 0x55;
    for (i = 0; i < 4; i++) {
        uint8_t *dst = &frame[RPI_OFF_MOTOR0 + i * 8];
        for (b = 0; b < 8; b++) {
            dst[b] = (b < (int)snap[i].raw_dlc) ? snap[i].raw_frame[b] : 0x00;
        }
    }
    wrist_us   = Servo_GetCurrentWristUs();
    gripper_us = Servo_GetCurrentGripperUs();
    frame[RPI_OFF_WRIST_US]     = (uint8_t)(wrist_us >> 8);
    frame[RPI_OFF_WRIST_US + 1] = (uint8_t)(wrist_us & 0xFFu);
    frame[RPI_OFF_GRIPPER_US]     = (uint8_t)(gripper_us >> 8);
    frame[RPI_OFF_GRIPPER_US + 1] = (uint8_t)(gripper_us & 0xFFu);

    stepper_deg = Stepper_GetLogicalDeg();
    frame[RPI_OFF_STEPPER_DEG]     = (uint8_t)((uint16_t)stepper_deg >> 8);
    frame[RPI_OFF_STEPPER_DEG + 1] = (uint8_t)((uint16_t)stepper_deg & 0xFFu);
    frame[RPI_OFF_CONVEYOR_RUN] = Conveyor_GetRun() ? 1u : 0u;

    frame[RPI_OFF_XOR] = 0u;
    for (i = 0; i < (int)RPI_OFF_XOR; i++) {
        frame[RPI_OFF_XOR] ^= frame[i];
    }
    Serial_SendArray(frame, RPI_BIN_FRAME_LEN);
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

    psc = (target_ticks + 65535u - 1u) / 65536u;
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
