#ifndef MOTOR_UTILS_H
#define MOTOR_UTILS_H

#include "stm32f10x.h"
#include "../../System/Delay.h"

/*================ 基础工具函数 ================*/
uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float u32_to_float(uint32_t u);

#endif /* MOTOR_UTILS_H */
