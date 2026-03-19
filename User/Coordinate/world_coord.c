#include "world_coord.h"
#include <math.h>
#include "../MotorControl/motor_config.h"

/*
  固定 HOME 世界坐标（单位：弧度）。
  说明：
  - 这是“世界坐标零点”的绝对角定义，与上电时电机处于什么角度无关。
  - 如果你的机械零位或装配发生变化，只需要修改 HOME 数组即可，
    不需要改控制参数、轨迹形状或 CAN 协议。
*/
static float g_home_abs[WORLD_MOTOR_NUM];
static uint8_t g_ready = 0;
static WorldCoord_HomeSource_t g_source = WORLD_HOME_SOURCE_UNKNOWN;

uint8_t WorldCoord_IsReady(void)
{
    return g_ready;
}

WorldCoord_HomeSource_t WorldCoord_GetHomeSource(void)
{
    return g_source;
}

WorldCoord_Status_t WorldCoord_SetHomeAbs(const float home_abs[WORLD_MOTOR_NUM], WorldCoord_HomeSource_t source)
{
    uint8_t i;
    if (home_abs == 0) return WORLD_ERR_NULLPTR;
    for (i = 0; i < WORLD_MOTOR_NUM; i++) {
        g_home_abs[i] = home_abs[i];
    }
    g_ready = 1;
    g_source = source;
    return WORLD_OK;
}

WorldCoord_Status_t WorldCoord_InitFixedHomeFromConfig(void)
{
    /*
      HOME 来源说明：
      - 本工程当前采用：编译期固定值（motor_config.h 的 WORLD_HOME_ABS）
      - 若将来改为“标定值/上位机下发”，只需改为调用 WorldCoord_SetHomeAbs(...)
        并把 source 设为对应类型即可。
    */
    const float home[WORLD_MOTOR_NUM] = WORLD_HOME_ABS;
    return WorldCoord_SetHomeAbs(home, WORLD_HOME_SOURCE_FIXED_CONFIG);
}

WorldCoord_Status_t WorldCoord_GetHomeAbs(uint8_t idx, float *out_home_abs)
{
    if (!g_ready) return WORLD_ERR_NOT_INIT;
    if (out_home_abs == 0) return WORLD_ERR_NULLPTR;
    if (idx >= WORLD_MOTOR_NUM) return WORLD_ERR_IDX_OOB;
    *out_home_abs = g_home_abs[idx];
    return WORLD_OK;
}

WorldCoord_Status_t WorldCoord_AbsFromRel(uint8_t idx, float target_world_rel, float *out_target_abs)
{
    float home;
    WorldCoord_Status_t st;
    if (out_target_abs == 0) return WORLD_ERR_NULLPTR;
    st = WorldCoord_GetHomeAbs(idx, &home);
    if (st != WORLD_OK) return st;
    /* abs = HOME(abs) + world_rel */
    *out_target_abs = home + target_world_rel;
    return WORLD_OK;
}

WorldCoord_Status_t WorldCoord_RelFromAbs(uint8_t idx, float abs_pos, float *out_world_rel)
{
    float home;
    WorldCoord_Status_t st;
    if (out_world_rel == 0) return WORLD_ERR_NULLPTR;
    st = WorldCoord_GetHomeAbs(idx, &home);
    if (st != WORLD_OK) return st;
    /* world_rel = abs - HOME(abs) */
    *out_world_rel = abs_pos - home;
    return WORLD_OK;
}

WorldCoord_Status_t WorldCoord_DeltaToTarget(uint8_t idx,
                                             float current_abs,
                                             float target_world_rel,
                                             float *out_delta,
                                             float *out_target_abs)
{
    float target_abs;
    WorldCoord_Status_t st;
    if ((out_delta == 0) || (out_target_abs == 0)) return WORLD_ERR_NULLPTR;

    st = WorldCoord_AbsFromRel(idx, target_world_rel, &target_abs);
    if (st != WORLD_OK) return st;

    /*
      三个量在这里被明确区分：
      - current_abs: 当前绝对角（来自 CAN 反馈）
      - target_world_rel: 目标相对 HOME 角（外部/树莓派下发）
      - delta: 最终需要转动的增量（在当前基础上还差多少）
    */
    *out_target_abs = target_abs;
    *out_delta = target_abs - current_abs;
    return WORLD_OK;
}

float WorldCoord_WrapDeltaPi(float delta)
{
    const float two_pi = 2.0f * 3.1415926535f;
    /*
      约束到 (-pi, pi]（左开右闭）：
      - 用于“不能绕整圈”的轴（例如底座），避免为了到目标而转 2π 的情况
      - 注意：是否使用 wrap 由上层策略决定，本函数只负责数学处理
    */
    delta = fmodf(delta, two_pi);
    if (delta > 3.1415926535f)  delta -= two_pi;
    if (delta <= -3.1415926535f) delta += two_pi;
    return delta;
}

