#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

/*
 * 舵机控制模块：PB0(腕部)、PB1(机械爪)
 * - 50Hz PWM：1500us=0°，500us=-180°，2500us=180°
 * - 输入：-18000~18000（度×100），0 对应中位
 * - 速度：Servo_SetRampSpeed 控制渐变步进
 */

/* 初始化：配置 TIM3 CH3/CH4，PB0/PB1 输出 50Hz 舵机 PWM */
void Servo_Init(void);

/* 腕部/机械爪：输入 -18000~18000（度×100），0=中位 1500us */
void Servo_SetWrist(int16_t raw);
void Servo_SetGripper(int16_t raw);

/* 渐变速度：0~100，越大步进越快；影响 Servo_Update 的步长 */
void Servo_SetRampSpeed(uint8_t speed);

/* 主循环周期调用：将当前 PWM 渐变至目标（若启用渐变） */
void Servo_Update(void);

#endif /* __SERVO_H */
