#include "motor_world_exec.h"

#include "motor_control.h"
#include "motor_can.h"
#include "motor_config.h"

#include "../Coordinate/world_coord.h"
#include "../../Hardware/Serial.h"

#include <math.h>

/* 连续旋转策略：1=允许整圈（不 wrap），0=不允许整圈（wrap 到 (-pi,pi]）；idx=0 始终不 wrap */
static const uint8_t s_continuous[MOTOR_NUM] = JOINT_CONTINUOUS_ROTATE;

/* 可选日志输出 */
#if MOTOR_DEBUG_LOG_ENABLE
#define MW_LOG(...) Serial_Printf(__VA_ARGS__)
#else
#define MW_LOG(...) do { } while (0)
#endif

static uint8_t idx_oob(uint8_t idx)
{
    return (idx >= MOTOR_NUM) ? 1U : 0U;
}

MotorWorld_Status_t MotorWorld_MoveToWorldRel(uint8_t idx,
                                              float target_world_rel,
                                              uint8_t mark_homed_after,
                                              MotorWorld_Result_t *out)
{
    WorldCoord_Status_t wst;
    float current_abs, target_abs, delta_abs;
    float final_abs, err;
    uint8_t used_wrap = 0;
    uint8_t corr = 0;

    if (out == 0) return MOTORWORLD_ERR_MOVE_ABORTED;
    if (idx_oob(idx)) return MOTORWORLD_ERR_IDX_OOB;
    if (!WorldCoord_IsReady()) return MOTORWORLD_ERR_WORLD_NOT_READY;

    /* 1) 读取反馈：得到 current_abs（绝对角，rad） */
    Read_All_Current_Positions();
    current_abs = Motor_States[idx].pos;

    /* 2) 坐标换算：world_rel -> target_abs，并计算 delta_abs = target_abs - current_abs */
    wst = WorldCoord_DeltaToTarget(idx, current_abs, target_world_rel, &delta_abs, &target_abs);
    if (wst != WORLD_OK) return MOTORWORLD_ERR_WORLD_NOT_READY;

    /*
      3) 连续旋转策略（底座 idx=0 不做最短路径 wrap）：
         - idx!=0 且 s_continuous[idx]==0：wrap 到 (-pi,pi]
         - 其余：保持原 delta
    */
    if (idx != 0U && s_continuous[idx] == 0U) {
        float wrapped = WorldCoord_WrapDeltaPi(delta_abs);
        if (wrapped != delta_abs) used_wrap = 1;
        delta_abs = wrapped;
        target_abs = current_abs + delta_abs;
    }

    /* 输出关键中间量，便于对照：current_abs / target_world_rel / target_abs / delta_abs */
    MW_LOG("[MW] idx=%d cur_abs=%.4f world_rel=%.4f tgt_abs=%.4f delta=%.4f wrap=%d\r\n",
           (int)idx, current_abs, target_world_rel, target_abs, delta_abs, (int)used_wrap);

    /*
      4) 执行运动：
         - 先把 Current_Targets 对齐到反馈角，确保“从实际角度出发”
         - 调用原轨迹 Move_Motor_To_Target
    */
    Current_Targets[idx] = current_abs;
    Move_Motor_To_Target((int)idx, target_abs, mark_homed_after, 0);

    /*
      5) 到位判断 + 闭环修正：
         读回反馈，如果误差超过 ARRIVAL_TOL_RAD，则最多重复修正 ARRIVAL_MAX_CORRECTIONS 次。
    */
    for (corr = 0; corr <= ARRIVAL_MAX_CORRECTIONS; corr++) {
        Read_All_Current_Positions();
        final_abs = Motor_States[idx].pos;
        err = final_abs - target_abs;

        if (fabsf(err) <= ARRIVAL_TOL_RAD) {
            break;
        }

        if (corr == ARRIVAL_MAX_CORRECTIONS) {
            MW_LOG("[MW] idx=%d arrival fail err=%.4f tol=%.4f\r\n", (int)idx, err, ARRIVAL_TOL_RAD);
            break;
        }

        MW_LOG("[MW] idx=%d correcting #%d err=%.4f\r\n", (int)idx, (int)(corr + 1), err);
        Current_Targets[idx] = final_abs;
        Move_Motor_To_Target((int)idx, target_abs, mark_homed_after, 0);
    }

    /* 6) 返回结果 */
    out->current_abs = current_abs;
    out->target_world_rel = target_world_rel;
    out->target_abs = target_abs;
    out->delta_abs = delta_abs;
    out->used_wrap = used_wrap;
    out->corrections = corr;
    out->final_abs = final_abs;
    out->final_error = err;

    return MOTORWORLD_OK;
}

MotorWorld_Status_t MotorWorld_MoveToAbs(uint8_t idx,
                                         float target_abs,
                                         uint8_t mark_homed_after,
                                         MotorWorld_Result_t *out)
{
    float current_abs, delta_abs;
    float final_abs, err;
    uint8_t used_wrap = 0;
    uint8_t corr = 0;

    if (out == 0) return MOTORWORLD_ERR_MOVE_ABORTED;
    if (idx_oob(idx)) return MOTORWORLD_ERR_IDX_OOB;

    /* 1) 读取反馈：得到 current_abs（绝对角，rad） */
    Read_All_Current_Positions();
    current_abs = Motor_States[idx].pos;

    /* 2) 直接计算 delta_abs（不经过世界坐标换算） */
    delta_abs = target_abs - current_abs;

    /* 3) 连续旋转策略：底座 idx=0 不 wrap */
    if (idx != 0U && s_continuous[idx] == 0U) {
        float wrapped = WorldCoord_WrapDeltaPi(delta_abs);
        if (wrapped != delta_abs) used_wrap = 1;
        delta_abs = wrapped;
        target_abs = current_abs + delta_abs;
    }

    MW_LOG("[MW] idx=%d cur_abs=%.4f tgt_abs=%.4f delta=%.4f wrap=%d (ABS)\r\n",
           (int)idx, current_abs, target_abs, delta_abs, (int)used_wrap);

    /* 4) 执行运动 */
    Current_Targets[idx] = current_abs;
    Move_Motor_To_Target((int)idx, target_abs, mark_homed_after, 0);

    /* 5) 到位判断 + 闭环修正 */
    for (corr = 0; corr <= ARRIVAL_MAX_CORRECTIONS; corr++) {
        Read_All_Current_Positions();
        final_abs = Motor_States[idx].pos;
        err = final_abs - target_abs;

        if (fabsf(err) <= ARRIVAL_TOL_RAD) {
            break;
        }

        if (corr == ARRIVAL_MAX_CORRECTIONS) {
            MW_LOG("[MW] idx=%d arrival fail err=%.4f tol=%.4f\r\n", (int)idx, err, ARRIVAL_TOL_RAD);
            break;
        }

        MW_LOG("[MW] idx=%d correcting #%d err=%.4f\r\n", (int)idx, (int)(corr + 1), err);
        Current_Targets[idx] = final_abs;
        Move_Motor_To_Target((int)idx, target_abs, mark_homed_after, 0);
    }

    /* 6) 回填结果（target_world_rel 对 ABS 入口无意义，置为 0） */
    out->current_abs = current_abs;
    out->target_world_rel = 0.0f;
    out->target_abs = target_abs;
    out->delta_abs = delta_abs;
    out->used_wrap = used_wrap;
    out->corrections = corr;
    out->final_abs = final_abs;
    out->final_error = err;

    return MOTORWORLD_OK;
}

