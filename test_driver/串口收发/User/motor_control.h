#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "stm32f10x.h"

/*================ 用户配置参数区 ================*/
#define MOTOR_NUM       4

/* --- 刚度(Kp) 与 阻尼(Kd) 调节 --- */
#define KP_MOVE_BASE    15.0f
#define KP_MOVE_HEAVY   65.0f
#define KP_MOVE_LIGHT   18.0f

#define KP_HOLD_SOFT    8.0f
#define KP_HOLD_STRONG  30.0f

#define KD_GENERAL      3.5f
#define KD_LIGHT        1.2f

/* --- 动作逻辑与安全配置 --- */
#define HOMING_SPEED     0.55f
#define COLLISION_TOR    28.0f
#define INTERVAL_MS      2

/*================ 必要新增参数：请与上位机“控制幅值”保持一致 ================*/
#define MIT_P_MIN       (-12.5f)
#define MIT_P_MAX       ( 12.5f)
#define MIT_V_MIN       (-2.0f)   /* 降低速度限制 */
#define MIT_V_MAX       ( 2.0f)   /* 降低速度限制 */
#define MIT_T_MIN       (-18.0f)
#define MIT_T_MAX       ( 18.0f)

/*================ 极限测试目标位置 ================*/
#define MOTOR4_PRESET_POS   1.70f
#define MOTOR3_TEST_POS     2.00f
#define MOTOR2_TEST_POS     2.40f

/*================ 物理限位定义 ================*/
extern const float P_LIMIT_MIN[MOTOR_NUM];
extern const float P_LIMIT_MAX[MOTOR_NUM];

/*================ 电机状态结构体 ================*/
typedef struct {
    float pos;          /* 当前角度（弧度），用于上报树莓派与轨迹计算 */
    float vel;
    float tor;
    uint8_t err;
    uint8_t mos_temp;
    uint8_t rotor_temp;
} Motor_Status_t;

typedef struct {
    volatile uint8_t got;
    volatile uint8_t op;
    volatile uint8_t rid;
    volatile uint32_t data_u32;
} Motor_RegResp_t;

/*================ 全局变量声明 ================*/
extern uint32_t Motor_IDs[MOTOR_NUM];
extern uint32_t Motor_Master_IDs[MOTOR_NUM];
extern float Motor_Home[MOTOR_NUM];
extern uint8_t Servo1_Angle;
extern uint8_t Servo2_Angle;

extern float Extreme_Hold_Kp[MOTOR_NUM];
extern float Extreme_Hold_Kd[MOTOR_NUM];
extern float Extreme_Hold_Tff[MOTOR_NUM];
extern volatile uint8_t Extreme_Test_Hold_Active;

extern float Move_Kp[MOTOR_NUM];
extern float Move_Kd[MOTOR_NUM];
extern float Move_Speed_Limit[MOTOR_NUM];
extern float Hold_Kp[MOTOR_NUM];
extern float Hold_Kd[MOTOR_NUM];
extern float Lock_Kp[MOTOR_NUM];
extern float Lock_Kd[MOTOR_NUM];
extern float Stabilize_Kp[MOTOR_NUM];
extern float Stabilize_Kd[MOTOR_NUM];

extern float Move_Tff_Positive[MOTOR_NUM];
extern float Move_Tff_Negative[MOTOR_NUM];
extern float Hold_Tff[MOTOR_NUM];
extern float Motor2_Move_Tff_Pos;
extern float Motor2_Move_Tff_Neg;
extern float Motor2_Hold_Tff;

extern float Runtime_P_Min[MOTOR_NUM];
extern float Runtime_P_Max[MOTOR_NUM];
extern float Runtime_V_Min[MOTOR_NUM];
extern float Runtime_V_Max[MOTOR_NUM];
extern float Runtime_T_Min[MOTOR_NUM];
extern float Runtime_T_Max[MOTOR_NUM];

extern float Current_Targets[MOTOR_NUM];
extern volatile Motor_Status_t Motor_States[MOTOR_NUM];
extern volatile uint8_t Motor_Feedback_Received[MOTOR_NUM];
extern uint8_t Motor_Homed[MOTOR_NUM];

extern volatile uint8_t Emergency_Stop;
extern volatile uint8_t System_Disabled;
extern volatile uint8_t Fault_Motor;
extern volatile uint8_t Fault_Code;
extern volatile Motor_RegResp_t Motor_RegResp[MOTOR_NUM];

/*================ 电机控制函数 ================*/
/* 全局速度倍率接口：所有轨迹规划中的速度都乘以该倍率，默认 0.5f。
 * 建议从上位机或 main.c 只通过此接口调节动作快慢，避免修改分散的宏。 */
void Motor_SetSpeedScale(float scale);

void Move_Motor_To_Target(int motor_idx, float target_p, uint8_t mark_homed_after, uint8_t use_shortest_wrap_for_base);
void Move_Two_Motors_To_Targets(int motor_a, float target_a, int motor_b, float target_b, uint8_t mark_a_homed_after, uint8_t mark_b_homed_after);
void Move_All_Motors_To_Targets(float target_0, float target_1, float target_2, float target_3);
void Apply_Rigid_Hold_One_Cycle(void);
void Hold_All_Rigid(uint32_t hold_ms);
void Safe_Control(int idx, float p, float v, float kp, float kd, float t);
float Get_Move_Tff(int idx, float dist);
float Get_Hold_Tff(int idx);
float Get_Extreme_Hold_Tff(int idx);
void Disable_All_Motors(void);
void Latch_Fault(uint8_t idx, uint8_t code);

/*================ 极限测试流程 ================*/
void Extreme_Test_Sequence(void);

/*================ 串口解析与运动执行 ================*/
void Parse_Serial_And_Move_Motors(void);

#endif

