#ifndef GRAVITY_PI_FEEDFORWARD_H
#define GRAVITY_PI_FEEDFORWARD_H

#include "motor_config.h"

#if GRAVITY_FF_PI_MODE

#include <stdint.h>

/* 树莓派下发四轴前馈力矩（Nm）后调用；缓冲至下一帧前保持，不自动清零 */
void GravityPi_OnTorqueLine(const float tau[4]);

/* TIM2 周期：MIT(p=反馈角,v=0,kp=0,kd=GRAVITY_FF_PI_MIT_KD,t=缓冲×GRAVITY_FF_GLOBAL_SCALE)，无 Current_Targets */
void GravityPi_ApplyAll(void);

uint8_t GravityPi_HasReceivedTorque(void);

/* 串口解析得到的四轴力矩（未乘 GRAVITY_FF_GLOBAL_SCALE） */
void GravityPi_GetPiReceivedTau(float out[4]);

/* 上一周期写入 MIT 的 τ（Nm，已 Runtime_T 限幅） */
void GravityPi_GetLastMitTorqueCmd(float out[4]);

/* TAU 帧解析结果统计（供 FB 与调试） */
void GravityPi_NotifyTauParseResult(int success);
uint32_t GravityPi_GetTauParseOkCount(void);
uint32_t GravityPi_GetTauParseFailCount(void);
int8_t GravityPi_GetLastTauParseResult(void);

#endif /* GRAVITY_FF_PI_MODE */

#endif /* GRAVITY_PI_FEEDFORWARD_H */
