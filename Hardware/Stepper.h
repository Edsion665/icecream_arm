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

/* 1=main 中 Stepper_Init 后以恒定转速试转（需自行确认机械安全） */
#ifndef STEPPER_TEST_ENABLE         
#define STEPPER_TEST_ENABLE         1
#endif

#if STEPPER_TEST_ENABLE
#define STEPPER_TEST_MOVE_COUNT     5u
#ifndef STEPPER_TEST_MOVE1_DEG
#define STEPPER_TEST_MOVE1_DEG      45.0f
#endif
#ifndef STEPPER_TEST_MOVE1_RPM
#define STEPPER_TEST_MOVE1_RPM      10.0f
#endif
#ifndef STEPPER_TEST_MOVE2_DEG
#define STEPPER_TEST_MOVE2_DEG      (-45.0f)
#endif
#ifndef STEPPER_TEST_MOVE2_RPM
#define STEPPER_TEST_MOVE2_RPM      10.0f
#endif
#ifndef STEPPER_TEST_MOVE3_DEG
#define STEPPER_TEST_MOVE3_DEG      90.0f
#endif
#ifndef STEPPER_TEST_MOVE3_RPM
#define STEPPER_TEST_MOVE3_RPM      15.0f
#endif
#ifndef STEPPER_TEST_MOVE4_DEG
#define STEPPER_TEST_MOVE4_DEG      (-90.0f)
#endif
#ifndef STEPPER_TEST_MOVE4_RPM
#define STEPPER_TEST_MOVE4_RPM      20.0f
#endif
#ifndef STEPPER_TEST_MOVE5_DEG
#define STEPPER_TEST_MOVE5_DEG      180.0f
#endif
#ifndef STEPPER_TEST_MOVE5_RPM
#define STEPPER_TEST_MOVE5_RPM      25.0f
#endif
#ifndef STEPPER_TEST_MOVE_PAUSE_MS
#define STEPPER_TEST_MOVE_PAUSE_MS  500u
#endif
#endif

void Stepper_Init(void);

/* 试转：设方向 + 转速；脉冲由 TIM4 CH2 硬件 PWM 持续输出，无需中断里再调 */
void Stepper_TestRunConstant(float rpm, uint8_t forward);
void Stepper_TestStop(void);
#if STEPPER_TEST_ENABLE
/* 按 STEPPER_TEST_MOVEx_DEG/RPM 顺序调用 Stepper_MoveDegrees，档间停顿 */
void Stepper_TestRunMoveSequence(void);
#endif

/* 增量转角 [-180,180]°，负=反向(PA1 低)；rpm 为到达过程的转速；阻塞至脉冲计满后 Stop */
void Stepper_MoveDegrees(float deg, float rpm);
int32_t Stepper_GetPosSteps(void);

void Stepper_SetDirection(uint8_t forward);
uint8_t Stepper_GetDirection(void);

void Stepper_SetStepFrequencyHz(uint32_t hz);
uint32_t Stepper_GetStepFrequencyHz(void);

void Stepper_SetSpeedRPM(float rpm);
float Stepper_GetSpeedRPM(void);

#endif /* __STEPPER_H */
