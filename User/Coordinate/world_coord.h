#ifndef WORLD_COORD_H
#define WORLD_COORD_H

#include <stdint.h>

/* 轴数量由外部在编译期提供；若未提供则回退为 4（尽量减少强耦合） */
#ifndef WORLD_MOTOR_NUM
#define WORLD_MOTOR_NUM 4
#endif

/*================ 世界坐标 / HOME 坐标系 ================*/
/*
  本模块只做“坐标换算/角度差计算”，不直接依赖 CAN 或控制参数。

  术语约定：
  - abs：绝对角（电机驱动反馈/控制所使用的坐标系，通常就是 Motor_States[i].pos）
  - world_rel：相对世界坐标（相对 HOME 的角度，树莓派下发的就是这个）

  关系：
    target_abs = home_abs[idx] + target_world_rel
    delta      = target_abs - current_abs
*/
typedef enum {
    WORLD_OK = 0,
    WORLD_ERR_NOT_INIT = -1,
    WORLD_ERR_NULLPTR  = -2,
    WORLD_ERR_IDX_OOB  = -3
} WorldCoord_Status_t;

typedef enum {
    WORLD_HOME_SOURCE_UNKNOWN = 0,
    WORLD_HOME_SOURCE_FIXED_CONFIG = 1, /* 编译期固定值（motor_config.h 的 WORLD_HOME_ABS） */
    WORLD_HOME_SOURCE_HOST_SET     = 2  /* 上位机/外部运行时下发（调用 WorldCoord_SetHomeAbs） */
} WorldCoord_HomeSource_t;

WorldCoord_Status_t WorldCoord_InitFixedHomeFromConfig(void);
WorldCoord_Status_t WorldCoord_SetHomeAbs(const float home_abs[WORLD_MOTOR_NUM], WorldCoord_HomeSource_t source);
WorldCoord_Status_t WorldCoord_GetHomeAbs(uint8_t idx, float *out_home_abs);
WorldCoord_HomeSource_t WorldCoord_GetHomeSource(void);
uint8_t WorldCoord_IsReady(void);

/*================ 坐标换算 ================*/
/* 将“相对世界坐标角度(world_rel)”换成“绝对目标角度(abs)” */
WorldCoord_Status_t WorldCoord_AbsFromRel(uint8_t idx, float target_world_rel, float *out_target_abs);
/* 将“绝对角度(abs)”换成“相对世界坐标角度(world_rel)” */
WorldCoord_Status_t WorldCoord_RelFromAbs(uint8_t idx, float abs_pos, float *out_world_rel);

/*================ 核心：计算需要旋转多少 ================*/
/*
  current_abs: 通过 CAN 反馈获得的当前实际角度(绝对坐标)
  target_world_rel: 外部传入的目标角度(相对世界/HOME 坐标)
  return: 在当前基础上还需要旋转的角度 Δ = target_abs - current_abs
*/
WorldCoord_Status_t WorldCoord_DeltaToTarget(uint8_t idx,
                                             float current_abs,
                                             float target_world_rel,
                                             float *out_delta,
                                             float *out_target_abs);

/* 可选：将 Δ 约束到 (-pi, pi]，用于需要“最短转动”的轴 */
float WorldCoord_WrapDeltaPi(float delta);

#endif /* WORLD_COORD_H */

