#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h>

/*
 * 步进电机（共阴极 STEP/DIR）
 * - PB7：STEP，TIM4_CH2 PWM
 * - PA1：DIR；高=正向
 * TIM2 专供 fb_report；TIM4 专供本模块 STEP 脉冲。
 */

/*================ 机械参数 ================*/
#define STEPPER_MICROSTEP           4u
#define STEPPER_PULSES_PER_REV      800u

/*================ 转角运动参数（改这里即可调速度/限幅） ================*/
#ifndef STEPPER_MOVE_RPM
#define STEPPER_MOVE_RPM            100.0f   /* Stepper_MoveDegrees 运行转速 */
#endif
#ifndef STEPPER_DIR_SETUP_MS
#define STEPPER_DIR_SETUP_MS        2u      /* DIR 建立时间，再出 STEP */
#endif
/* 与 main 循环 Delay_ms(INTERVAL_MS) 一致，用于非阻塞 DIR 等待 */
#ifndef STEPPER_UPDATE_PERIOD_MS
#define STEPPER_UPDATE_PERIOD_MS    2u
#endif

/*================ 上电测试 ================*/
#ifndef STEPPER_TEST_ENABLE
#define STEPPER_TEST_ENABLE         0
#endif

#if STEPPER_TEST_ENABLE
#define STEPPER_TEST_MOVE_COUNT     5u
#ifndef STEPPER_TEST_MOVE1_DEG
#define STEPPER_TEST_MOVE1_DEG      45.0f
#endif
#ifndef STEPPER_TEST_MOVE2_DEG
#define STEPPER_TEST_MOVE2_DEG      (-45.0f)
#endif
#ifndef STEPPER_TEST_MOVE3_DEG
#define STEPPER_TEST_MOVE3_DEG      90.0f
#endif
#ifndef STEPPER_TEST_MOVE4_DEG
#define STEPPER_TEST_MOVE4_DEG      (-90.0f)
#endif
#ifndef STEPPER_TEST_MOVE5_DEG
#define STEPPER_TEST_MOVE5_DEG      180.0f
#endif
#ifndef STEPPER_TEST_MOVE_PAUSE_MS
#define STEPPER_TEST_MOVE_PAUSE_MS  500u
#endif
#endif

/*================ API ================*/
void Stepper_Init(void);

void Stepper_SetDirection(uint8_t forward);
uint8_t Stepper_GetDirection(void);

void Stepper_SetStepFrequencyHz(uint32_t hz);
uint32_t Stepper_GetStepFrequencyHz(void);
void Stepper_SetSpeedRPM(float rpm);
float Stepper_GetSpeedRPM(void);

void Stepper_Stop(void);
int32_t Stepper_GetPosSteps(void);

/* 下行目标：增量角 °，非 0 时由 Stepper_Update 非阻塞执行 */
void Stepper_SetTargetDeltaDeg(float deg);
void Stepper_Update(void);
uint8_t Stepper_IsBusy(void);
int16_t Stepper_GetLogicalDeg(void);

/* 阻塞式一次转完（测试用）；正常运行请用 SetTargetDeltaDeg + Update */
void Stepper_MoveDegrees(float deg);

#if STEPPER_TEST_ENABLE
void Stepper_TestRunMoveSequence(void);
#endif

#endif /* __STEPPER_H */
