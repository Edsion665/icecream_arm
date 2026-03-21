#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include "stm32f10x.h"
#include "motor_types.h"
#include "motor_config.h"

/*================ 全局变量声明 ================*/
extern uint32_t Motor_IDs[MOTOR_NUM];
extern uint32_t Motor_Master_IDs[MOTOR_NUM];
extern float Runtime_P_Min[MOTOR_NUM];
extern float Runtime_P_Max[MOTOR_NUM];
extern float Runtime_V_Min[MOTOR_NUM];
extern float Runtime_V_Max[MOTOR_NUM];
extern float Runtime_T_Min[MOTOR_NUM];
extern float Runtime_T_Max[MOTOR_NUM];
extern volatile Motor_Status_t Motor_States[MOTOR_NUM];
extern volatile uint8_t Motor_Feedback_Received[MOTOR_NUM];
extern volatile Motor_RegResp_t Motor_RegResp[MOTOR_NUM];
extern volatile uint8_t Emergency_Stop;
extern volatile uint8_t System_Disabled;
extern volatile uint8_t Fault_Motor;
extern volatile uint8_t Fault_Code;

/*================ CAN 发送函数 ================*/
void CAN_Send_Blocking(CanTxMsg *tx);
void Motor_MIT_Send_Raw(int idx, float p, float v, float kp, float kd, float t);
void Motor_MIT_Send_Raw_NoPostDelay(int idx, float p, float v, float kp, float kd, float t);
void Motor_Send_Special(uint32_t id, uint8_t cmd);
void Motor_Read_Register_Request(int idx, uint8_t rid);

/*================ 反馈/寄存器读取 ================*/
/* 拉反馈时仍对 idx 轴发「当前目标 + 保持刚度」的 MIT，禁止全零包（否则驱动器会瞬时失力） */
void Request_Motor_Feedback(int idx);
uint8_t Wait_Register_Response(int idx, uint8_t rid, uint32_t wait_ms);
uint8_t Read_Register_Float(int idx, uint8_t rid, float *out);
void Sync_MIT_Range_From_Driver(void);
void Read_All_Current_Positions(void);

/*================ 故障判断 ================*/
uint8_t Motor_Is_Fault(int idx);
void Latch_Fault(uint8_t idx, uint8_t code);
void Disable_All_Motors(void);

/*================ 硬件初始化 ================*/
void Hardware_Init(void);

#endif /* MOTOR_CAN_H */
