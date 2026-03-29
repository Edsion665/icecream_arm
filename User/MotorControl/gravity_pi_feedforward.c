#include "gravity_pi_feedforward.h"

#if GRAVITY_FF_PI_MODE

#include "motor_control.h"
#include "motor_can.h"

static float s_tau_buf[4];
static uint8_t s_have_tau;

static const float s_pi_mit_kp[MOTOR_NUM] = GRAVITY_FF_PI_MIT_KP;
static const float s_pi_mit_kd[MOTOR_NUM] = GRAVITY_FF_PI_MIT_KD;

/* 最近一次 MIT 包中的 τ（与 Safe_Control 内限幅后一致） */
static float s_last_mit_torque_cmd[4];

volatile uint32_t g_gravity_pi_tau_parse_ok_cnt;
volatile uint32_t g_gravity_pi_tau_parse_fail_cnt;
/* -1=自上次上电后尚未处理过 TAU 行；0=上一帧 TAU 校验/解析失败；1=成功 */
volatile int8_t g_gravity_pi_last_tau_parse_result = -1;

void GravityPi_OnTorqueLine(const float tau[4])
{
    int i;

    for (i = 0; i < MOTOR_NUM; i++) {
        s_tau_buf[i] = tau[i];
    }
    s_have_tau = 1u;
}

void GravityPi_NotifyTauParseResult(int success)
{
    if (success) {
        g_gravity_pi_tau_parse_ok_cnt++;
        g_gravity_pi_last_tau_parse_result = 1;
    } else {
        g_gravity_pi_tau_parse_fail_cnt++;
        g_gravity_pi_last_tau_parse_result = 0;
    }
}

uint32_t GravityPi_GetTauParseOkCount(void)
{
    return g_gravity_pi_tau_parse_ok_cnt;
}

uint32_t GravityPi_GetTauParseFailCount(void)
{
    return g_gravity_pi_tau_parse_fail_cnt;
}

int8_t GravityPi_GetLastTauParseResult(void)
{
    return g_gravity_pi_last_tau_parse_result;
}

uint8_t GravityPi_HasReceivedTorque(void)
{
    return s_have_tau;
}

/* 串口解析得到的四轴力矩（未乘 GRAVITY_FF_GLOBAL_SCALE），无 TAU 时为 0 */
void GravityPi_GetPiReceivedTau(float out[4])
{
    int i;

    for (i = 0; i < MOTOR_NUM; i++) {
        out[i] = s_have_tau ? s_tau_buf[i] : 0.0f;
    }
}

/* 上一周期写入 MIT 的 τ（Nm），已做 Runtime_T 限幅，与 CAN 发包一致 */
void GravityPi_GetLastMitTorqueCmd(float out[4])
{
    int i;

    for (i = 0; i < MOTOR_NUM; i++) {
        out[i] = s_last_mit_torque_cmd[i];
    }
}

void GravityPi_ApplyAll(void)
{
    int i;

    if (Emergency_Stop || System_Disabled) {
        return;
    }

    for (i = 0; i < MOTOR_NUM; i++) {
        float t;
        if (s_have_tau) {
            t = s_tau_buf[i] * GRAVITY_FF_GLOBAL_SCALE;
        } else {
            t = Get_Extreme_Hold_Tff(i);
        }
        if (t < Runtime_T_Min[i]) {
            t = Runtime_T_Min[i];
        }
        if (t > Runtime_T_Max[i]) {
            t = Runtime_T_Max[i];
        }
        s_last_mit_torque_cmd[i] = t;

        {
            float p_fb = Motor_States[i].pos;
            Safe_Control(i,
                         p_fb,
                         0.0f,
                         s_pi_mit_kp[i],
                         s_pi_mit_kd[i],
                         t);
        }
    }
}

#endif /* GRAVITY_FF_PI_MODE */
