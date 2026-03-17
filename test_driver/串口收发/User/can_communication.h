#ifndef __CAN_COMMUNICATION_H
#define __CAN_COMMUNICATION_H

#include "stm32f10x.h"
#include "motor_control.h"

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

/*================ CAN通信函数 ================*/
void CAN_Hardware_Init(void);
void CAN_Send_Blocking(CanTxMsg *tx);
void Motor_MIT_Send_Raw(int idx, float p, float v, float kp, float kd, float t);
void Motor_Send_Special(uint32_t id, uint8_t cmd);
void Motor_Read_Register_Request(int idx, uint8_t rid);
void Request_Motor_Feedback(int idx);
uint8_t Wait_Register_Response(int idx, uint8_t rid, uint32_t wait_ms);
uint8_t Read_Register_Float(int idx, uint8_t rid, float *out);
void Read_All_Current_Positions(void);
void Sync_MIT_Range_From_Driver(void);

/*================ 工具函数 ================*/
uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float u32_to_float(uint32_t u);

#endif
