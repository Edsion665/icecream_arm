#include "motor_control.h"
#include "motor_hold_timer.h"

/*================ 全局变量定义 ================*/
float Current_Targets[MOTOR_NUM];
uint8_t Motor_Homed[MOTOR_NUM];

#if !MIT_HEX_MODE
static const float lock_kp[MOTOR_NUM] = LOCK_KP;
static const float lock_kd[MOTOR_NUM] = LOCK_KD;
static const float extreme_hold_kp[MOTOR_NUM] = EXTREME_HOLD_KP;
static const float extreme_hold_kd[MOTOR_NUM] = EXTREME_HOLD_KD;
static const float extreme_hold_tff[MOTOR_NUM] = EXTREME_HOLD_TFF;
static const float hold_tff[MOTOR_NUM] = HOLD_TFF;
#else
/* MIT_HEX=1 时不在 motor_config 暴露 LOCK/EXTREME；读反馈/（若启用）保持仍用同数值 */
static const float lock_kp[MOTOR_NUM] = { 10.0f, 16.0f, 16.0f, 10.0f };
static const float lock_kd[MOTOR_NUM] = { 1.2f, 1.8f, 1.8f, 1.2f };
static const float extreme_hold_kp[MOTOR_NUM] = { 18.0f, 62.0f, 42.0f, 24.0f };
static const float extreme_hold_kd[MOTOR_NUM] = { 1.8f, 3.2f, 2.8f, 1.8f };
static const float extreme_hold_tff[MOTOR_NUM] = { 0.0f, 0.0f, 0.0f, 0.0f };
static const float hold_tff[MOTOR_NUM] = { 0.0f, 0.10f, 0.80f, 0.0f };
#endif

static float Get_Hold_Tff(int idx)
{
    if (idx == 1) {
#if !MIT_HEX_MODE
        return MOTOR2_HOLD_TFF;
#else
        return 0.20f;
#endif
    }
    return hold_tff[idx];
}

static float Get_Extreme_Hold_Tff(int idx)
{
    return extreme_hold_tff[idx];
}

static void Safe_Control_NoPostDelay(int idx, float p, float v, float kp, float kd, float t);

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

static void Safe_Control_NoPostDelay(int idx, float p, float v, float kp, float kd, float t)
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

    Motor_MIT_Send_Raw_NoPostDelay(idx, p, v, kp, kd, t);
}

void Apply_Rigid_Hold_OnBuffers_NoPostDelay(const float *pos, const uint8_t *homed)
{
    int i;

    if (pos == 0 || homed == 0) {
        return;
    }

    for (i = 0; i < MOTOR_NUM; i++) {
        if (Emergency_Stop) {
            return;
        }
        if (homed[i]) {
            Safe_Control_NoPostDelay(i, pos[i], 0.0f,
                                     extreme_hold_kp[i], extreme_hold_kd[i],
                                     Get_Extreme_Hold_Tff(i));
        } else {
            Safe_Control_NoPostDelay(i, pos[i], 0.0f,
                                     lock_kp[i], lock_kd[i],
                                     Get_Hold_Tff(i));
        }
    }
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
    MotorHoldTimer_PublishSnapshot();
}

/*
 * 读反馈前向总线发的 MIT：必须与刚性保持一致。
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
