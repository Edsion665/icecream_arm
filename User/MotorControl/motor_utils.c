#include "motor_utils.h"

/*================ 基础工具函数 ================*/
uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / (float)((1 << bits) - 1) + x_min;
}

float u32_to_float(uint32_t u)
{
    union {
        uint32_t u32;
        float    f32;
    } cvt;
    cvt.u32 = u;
    return cvt.f32;
}
