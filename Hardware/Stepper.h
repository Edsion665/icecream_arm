#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h>

/*
 * 步进电机（共阴极驱动器 STEP/DIR）
 * - PA0：STEP，TIM2_CH1 PWM（脉冲频率 = 步进速率）
 * - PA1：DIR，普通 GPIO（非 TIM2_CH2）；Init 默认拉高=正向，运行中可改电平
 *
 * 驱动器：4 细分，800 脉冲/转（与 STEPPER_MICROSTEP / STEPPER_PULSES_PER_REV 一致）
 *
 * 定时器（STM32F103C8 中容量）：PA0 = TIM2_CH1；FB 上报已迁至 TIM6，与 TIM2 无冲突。
 */

#define STEPPER_MICROSTEP           4u
#define STEPPER_PULSES_PER_REV      800u

void Stepper_Init(void);

/* PA1：1=正向（拉高），0=反向（拉低）；Init 后默认为正向 */
void Stepper_SetDirection(uint8_t forward);
uint8_t Stepper_GetDirection(void);

/* STEP 脉冲频率 Hz（0=关闭 PWM 输出） */
void Stepper_SetStepFrequencyHz(uint32_t hz);
uint32_t Stepper_GetStepFrequencyHz(void);

/* 转速 r/min，内部按 800 脉冲/转换算为 Hz */
void Stepper_SetSpeedRPM(float rpm);
float Stepper_GetSpeedRPM(void);

#endif /* __STEPPER_H */
