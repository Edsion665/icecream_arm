#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor_config.h"
#include "motor_can.h"

/*================ 全局变量声明 ================*/
extern float Current_Targets[MOTOR_NUM];
extern uint8_t Motor_Homed[MOTOR_NUM];

/*================ 启动对齐：避免上电跳动 ================*/
void Sync_CurrentTargets_From_Feedback(void);

/*================ 安全控制 ================*/
void Safe_Control(int idx, float p, float v, float kp, float kd, float t);

#endif /* MOTOR_CONTROL_H */
