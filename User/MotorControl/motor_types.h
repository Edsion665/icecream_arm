#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdint.h>

/*================ 故障状态定义 ================*/
#define MOTOR_ERR_DISABLE      0x0
#define MOTOR_ERR_ENABLE       0x1
#define MOTOR_ERR_OVERVOLT     0x8
#define MOTOR_ERR_UNDERVOLT    0x9
#define MOTOR_ERR_OVERCURRENT  0xA
#define MOTOR_ERR_MOS_TEMP     0xB
#define MOTOR_ERR_COIL_TEMP    0xC
#define MOTOR_ERR_CAN_LOST     0xD
#define MOTOR_ERR_OVERLOAD     0xE

/*================ 电机状态结构体 ================*/
typedef struct {
    float pos;      /* 位置 */
    float vel;      /* 速度 */
    float tor;      /* 扭矩 */
    uint8_t err;    /* 错误码 */
    uint8_t mos_temp;   /* MOS管温度 */
    uint8_t rotor_temp; /* 转子温度 */
} Motor_Status_t;

/*================ 寄存器响应结构体 ================*/
typedef struct {
    volatile uint8_t got;       /* 是否收到响应 */
    volatile uint8_t op;        /* 操作类型 */
    volatile uint8_t rid;       /* 寄存器ID */
    volatile uint32_t data_u32; /* 数据 */
} Motor_RegResp_t;

#endif /* MOTOR_TYPES_H */
