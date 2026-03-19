#ifndef MOTOR_WORLD_EXEC_H
#define MOTOR_WORLD_EXEC_H

#include <stdint.h>

typedef enum {
    MOTORWORLD_OK = 0,
    MOTORWORLD_ERR_IDX_OOB = -10,
    MOTORWORLD_ERR_WORLD_NOT_READY = -11,
    MOTORWORLD_ERR_LIMIT_VIOLATION = -12,
    MOTORWORLD_ERR_MOVE_ABORTED = -13
} MotorWorld_Status_t;

typedef struct {
    /* ===== 输入量语义（全部单位：弧度 rad） =====
       current_abs      : CAN 反馈的当前绝对角
       target_world_rel : 外部目标（相对 HOME 的角度）
       target_abs       : 换算后的绝对目标角
       delta_abs        : 当前还需要转动多少（target_abs - current_abs）
    */
    float current_abs;
    float target_world_rel;
    float target_abs;
    float delta_abs;

    /* 执行策略信息 */
    uint8_t used_wrap;      /* 是否对 delta 使用了 (-pi,pi] wrap */
    uint8_t corrections;    /* 实际做了几次闭环修正 */

    /* 结果信息 */
    float final_abs;        /* 执行后最终反馈角（读回） */
    float final_error;      /* final_abs - target_abs */
} MotorWorld_Result_t;

/*
  统一入口：给定目标 world_rel（相对 HOME），由 MCU 端读取反馈、计算 delta，
  并执行与原工程一致的轨迹/保持（不改控制参数），同时做：
  - 关节连续旋转策略：不可整圈 -> wrap 到 (-pi,pi]
  - 机械限位检查：目标超限直接拒绝
  - 到位判断与闭环修正：误差过大则最多重复修正 ARRIVAL_MAX_CORRECTIONS 次
  - 调试日志：可选串口输出关键中间量
*/
MotorWorld_Status_t MotorWorld_MoveToWorldRel(uint8_t idx,
                                              float target_world_rel,
                                              uint8_t mark_homed_after,
                                              MotorWorld_Result_t *out);

/*
  绝对角目标入口：target_abs 直接是驱动反馈坐标系下的绝对角（rad）。
  适用场景：你从电机 CAN 直接读到/或上位机直接给出绝对角目标，不需要世界坐标换算。
*/
MotorWorld_Status_t MotorWorld_MoveToAbs(uint8_t idx,
                                         float target_abs,
                                         uint8_t mark_homed_after,
                                         MotorWorld_Result_t *out);

#endif /* MOTOR_WORLD_EXEC_H */

