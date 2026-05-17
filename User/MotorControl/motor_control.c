#include "motor_control.h"

/*================ 全局变量定义 ================*/
float Current_Targets[MOTOR_NUM];
uint8_t Motor_Homed[MOTOR_NUM];

static const float lock_kp[MOTOR_NUM] = LOCK_KP;
static const float lock_kd[MOTOR_NUM] = LOCK_KD;
static const float extreme_hold_kp[MOTOR_NUM] = EXTREME_HOLD_KP;
static const float extreme_hold_kd[MOTOR_NUM] = EXTREME_HOLD_KD;
static const float extreme_hold_tff[MOTOR_NUM] = EXTREME_HOLD_TFF;
static const float hold_tff[MOTOR_NUM] = HOLD_TFF;

static float Get_Hold_Tff(int idx)
{
    if (idx == 1) {
        return MOTOR2_HOLD_TFF;
    }
    return hold_tff[idx];
}

static float Get_Extreme_Hold_Tff(int idx)
{
    return extreme_hold_tff[idx];
}

void Safe_Control(int idx, float p, float v, float kp, float kd, float t)
{
    if (Emergency_Stop) {
        return;
    }

    if (t < Runtime_T_Min[idx]) {
        t = Runtime_T_Min[idx];
    }
    if (t > Runtime_T_Max[idx]) {
        t = Runtime_T_Max[idx];
    }

    if (kd < 0.15f) {
        kd = 0.15f;
    }
    if (kp < 0.0f) {
        kp = 0.0f;
    }

    Motor_MIT_Send_Raw(idx, p, v, kp, kd, t);
}

void Sync_CurrentTargets_From_Feedback(void)
{
    int i;

    Read_All_Current_Positions();

    for (i = 0; i < MOTOR_NUM; i++) {
        if (Motor_Feedback_Received[i]) {
            Current_Targets[i] = Motor_States[i].pos;
        }
    }
}

/*
 * 读反馈前向总线发的 MIT：使用 lock_kp/kd 保持刚度。
 */
void Request_Motor_Feedback(int idx)
{
    if (idx < 0 || idx >= MOTOR_NUM) {
        return;
    }
    if (Emergency_Stop || System_Disabled) {
        return;
    }

    if (Motor_Homed[idx]) {
        Safe_Control(idx,
                     Current_Targets[idx],
                     0.0f,
                     extreme_hold_kp[idx],
                     extreme_hold_kd[idx],
                     Get_Extreme_Hold_Tff(idx));
    } else {
        Safe_Control(idx,
                     Current_Targets[idx],
                     0.0f,
                     lock_kp[idx],
                     lock_kd[idx],
                     Get_Hold_Tff(idx));
    }
}
