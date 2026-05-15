#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h>

/*
 * 步进电机（共阴极 STEP/DIR）
 * - PB7：STEP，TIM4_CH2 PWM（F103C8 上 PB7 对应 TIM4_CH2；TIM6 无引脚输出）
 * - PA1：DIR，GPIO；Init 默认拉高=正向
 *
 * TIM2 专供 fb_report_timer（25Hz FB 上行）；步进使用 TIM4。
 * MIT_HEX_MODE=1 时 MOTOR_HOLD_TIM4_ENABLE=0，TIM4 不与电机保持冲突。
 */
#define STEPPER_MICROSTEP           4u
#define STEPPER_PULSES_PER_REV      800u

void Stepper_Init(void);

void Stepper_SetDirection(uint8_t forward);
uint8_t Stepper_GetDirection(void);

void Stepper_SetStepFrequencyHz(uint32_t hz);
uint32_t Stepper_GetStepFrequencyHz(void);

void Stepper_SetSpeedRPM(float rpm);
float Stepper_GetSpeedRPM(void);

#endif /* __STEPPER_H */
