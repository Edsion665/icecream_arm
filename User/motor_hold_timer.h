#ifndef MOTOR_HOLD_TIMER_H
#define MOTOR_HOLD_TIMER_H

#include <stdint.h>

/* TIM4：按 INTERVAL_MS 周期在 ISR 中下发 MIT 刚性保持（读快照，不依赖主循环节拍）
 * 总开关见 motor_config.h：MOTOR_HOLD_TIM4_ENABLE（0=关闭试验，1=开启） */
void MotorHoldTimer_Init(void);
void MotorHoldTimer_PublishSnapshot(void);

/* 插补/轨迹循环内由主线程发 MIT 时关闭定时保持，避免与轨迹指令打架 */
void MotorHoldTimer_StreamEnter(void);
void MotorHoldTimer_StreamExit(void);

/* 调试：插补深度（>0 时 TIM4 不发保持，由 Move_* 发 MIT） */
uint32_t MotorHoldTimer_GetStreamDepth(void);

/* TIM4 ISR 内统计（可选自接调试器查看；主循环已不再打印） */
extern volatile uint32_t g_MotorHold_IsrApplyCount;
extern volatile uint32_t g_MotorHold_IsrSkipStreamCount;
extern volatile uint32_t g_MotorHold_IsrSkipEmgCount;

#endif
