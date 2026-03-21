#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor_config.h"
#include "motor_can.h"

/*================ 全局变量声明 ================*/
extern float Current_Targets[MOTOR_NUM];
extern uint8_t Motor_Homed[MOTOR_NUM];
extern volatile uint8_t Extreme_Test_Hold_Active;

/*================ 启动对齐：避免上电跳动 ================*/
void Sync_CurrentTargets_From_Feedback(void);

/*================ 前馈力矩补偿 ================*/
float Get_Move_Tff(int idx, float dist);
float Get_Hold_Tff(int idx);
float Get_Extreme_Hold_Tff(int idx);

/*================ 安全控制 ================*/
void Safe_Control(int idx, float p, float v, float kp, float kd, float t);

/*================ 核心：刚性保持 ================*/
void Apply_Rigid_Hold_One_Cycle(void);
/* 供 TIM 保持 ISR 使用：无帧间 Delay_us，从缓冲区读位置/homed */
void Apply_Rigid_Hold_OnBuffers_NoPostDelay(const float *pos, const uint8_t *homed);
void Hold_All_Rigid(uint32_t hold_ms);

/*================ 通用单轴轨迹函数 ================*/
void Move_Motor_To_Target(int motor_idx, float target_p, uint8_t mark_homed_after, uint8_t use_shortest_wrap_for_base);
void Move_Motor_To_Rel(int motor_idx, float target_rel, uint8_t mark_homed_after, uint8_t use_shortest_wrap_for_base);
float Move_Motor_To_Rel_FromFeedback(int motor_idx, float target_rel, uint8_t mark_homed_after, uint8_t use_shortest_wrap_for_base);

/*================ 双轴同步轨迹函数 ================*/
void Move_Two_Motors_To_Targets(int motor_a, float target_a,
                               int motor_b, float target_b,
                               uint8_t mark_a_homed_after,
                               uint8_t mark_b_homed_after);
void Move_Two_Motors_To_Rels(int motor_a, float target_rel_a,
                             int motor_b, float target_rel_b,
                             uint8_t mark_a_homed_after,
                             uint8_t mark_b_homed_after);

/*================ 四轴同步轨迹函数 ================*/
void Move_Four_Motors_To_Targets(int m0, float t0,
                                 int m1, float t1,
                                 int m2, float t2,
                                 int m3, float t3,
                                 uint8_t mark_homed_after[4]);
void Move_Four_Motors_To_Rels(int m0, float r0,
                              int m1, float r1,
                              int m2, float r2,
                              int m3, float r3,
                              uint8_t mark_homed_after[4]);

/* 从真实反馈起步、相对 HOME 目标的四轴同步版本（推荐给树莓派世界坐标命令使用） */
void Move_Four_Motors_FromFeedback_To_Rels(int m0, float r0,
                                           int m1, float r1,
                                           int m2, float r2,
                                           int m3, float r3,
                                           uint8_t mark_homed_after[4]);

#endif /* MOTOR_CONTROL_H */
