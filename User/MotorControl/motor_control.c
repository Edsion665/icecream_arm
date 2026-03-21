#include "motor_control.h"
#include "motor_utils.h"
#include "../Coordinate/world_coord.h"
#include <math.h>

/*================ 全局变量定义 ================*/
float Current_Targets[MOTOR_NUM];
uint8_t Motor_Homed[MOTOR_NUM];
volatile uint8_t Extreme_Test_Hold_Active = 0;

/* 配置参数数组 */
static const float move_kp[MOTOR_NUM] = MOVE_KP;
static const float move_kd[MOTOR_NUM] = MOVE_KD;
static const float move_speed_limit[MOTOR_NUM] = MOVE_SPEED_LIMIT;
static const float lock_kp[MOTOR_NUM] = LOCK_KP;
static const float lock_kd[MOTOR_NUM] = LOCK_KD;
static const float extreme_hold_kp[MOTOR_NUM] = EXTREME_HOLD_KP;
static const float extreme_hold_kd[MOTOR_NUM] = EXTREME_HOLD_KD;
static const float extreme_hold_tff[MOTOR_NUM] = EXTREME_HOLD_TFF;
static const float move_tff_positive[MOTOR_NUM] = MOVE_TFF_POSITIVE;
static const float move_tff_negative[MOTOR_NUM] = MOVE_TFF_NEGATIVE;
static const float hold_tff[MOTOR_NUM] = HOLD_TFF;

/*================ 启动对齐：避免上电跳动 ================*/
void Sync_CurrentTargets_From_Feedback(void)
{
    int i;

    /* 拉取一次反馈，确保 Motor_States[].pos 更新 */
    Read_All_Current_Positions();

    for (i = 0; i < MOTOR_NUM; i++) {
        /*
         * 关键：只在“确实收到反馈”时才同步目标。
         * 否则 Motor_States[i].pos 可能仍为 0/旧值，导致上电刚性保持把电机拉到错误位置而抽动。
         */
        if (Motor_Feedback_Received[i]) {
            Current_Targets[i] = Motor_States[i].pos;
        }
    }
}

/*================ 前馈力矩补偿 ================*/
float Get_Move_Tff(int idx, float dist)
{
    if (idx == 1) {
        return (dist >= 0.0f) ? MOTOR2_MOVE_TFF_POS : MOTOR2_MOVE_TFF_NEG;
    }
    return (dist >= 0.0f) ? move_tff_positive[idx] : move_tff_negative[idx];
}

float Get_Hold_Tff(int idx)
{
    if (idx == 1) return MOTOR2_HOLD_TFF;
    return hold_tff[idx];
}

float Get_Extreme_Hold_Tff(int idx)
{
    return extreme_hold_tff[idx];
}

/*================ 安全控制 ================*/
void Safe_Control(int idx, float p, float v, float kp, float kd, float t)
{
    if (Emergency_Stop) return;

    /* 不做软件 P_LIMIT 夹紧；位置仅受驱动器 MIT 包范围等约束 */

    if (t < Runtime_T_Min[idx]) t = Runtime_T_Min[idx];
    if (t > Runtime_T_Max[idx]) t = Runtime_T_Max[idx];

    if (kd < 0.15f) kd = 0.15f;
    if (kp < 0.0f) kp = 0.0f;

    Motor_MIT_Send_Raw(idx, p, v, kp, kd, t);
}

/*================ 核心：刚性保持 ================*/
void Apply_Rigid_Hold_One_Cycle(void)
{
    int i;

    for (i = 0; i < MOTOR_NUM; i++) {
        if (Motor_Homed[i]) {
            Safe_Control(i,
                         Current_Targets[i],
                         0.0f,
                         extreme_hold_kp[i],
                         extreme_hold_kd[i],
                         Get_Extreme_Hold_Tff(i));
        } else {
            Safe_Control(i,
                         Current_Targets[i],
                         0.0f,
                         lock_kp[i],
                         lock_kd[i],
                         Get_Hold_Tff(i));
        }
    }
}

void Hold_All_Rigid(uint32_t hold_ms)
{
    uint32_t cycles = hold_ms / INTERVAL_MS;
    uint32_t c;

    for (c = 0; c < cycles; c++) {
        if (Emergency_Stop) return;
        Apply_Rigid_Hold_One_Cycle();
        Delay_ms(INTERVAL_MS);
    }
}

/*================ 通用单轴轨迹函数 ================*/
void Move_Motor_To_Target(int motor_idx, float target_p, uint8_t mark_homed_after, uint8_t use_shortest_wrap_for_base)
{
    float start_p = Current_Targets[motor_idx];
    float dist, total_time, ratio, blend, vel_shape;
    float cmd_p, cmd_v, move_tff;
    float speed_limit;
    int steps, s, i;

    (void)use_shortest_wrap_for_base; /* 已取消底座最短路径与 P_LIMIT，参数保留兼容旧调用 */

    dist = target_p - start_p;
    move_tff = Get_Move_Tff(motor_idx, dist);
    speed_limit = move_speed_limit[motor_idx] * MOTION_SPEED_SCALE;

    if (fabsf(dist) < 0.001f) {
        Current_Targets[motor_idx] = target_p;
        if (mark_homed_after) Motor_Homed[motor_idx] = 1;
        return;
    }

    total_time = fabsf(dist) * 3.1415926535f / (2.0f * speed_limit);
    if (total_time < 0.25f) total_time = 0.25f;

    steps = (int)(total_time / (INTERVAL_MS * 0.001f));
    if (steps < 80) steps = 80;

    for (s = 0; s <= steps; s++) {
        if (Emergency_Stop) return;

        if (Motor_Is_Fault(motor_idx)) {
            Latch_Fault((uint8_t)motor_idx, Motor_States[motor_idx].err);
            return;
        }

        ratio     = (float)s / (float)steps;
        blend     = 0.5f - 0.5f * cosf(3.1415926535f * ratio);
        vel_shape = 0.5f * 3.1415926535f * sinf(3.1415926535f * ratio);

        cmd_p = start_p + dist * blend;
        cmd_v = (dist / total_time) * vel_shape;

        Current_Targets[motor_idx] = cmd_p;

        for (i = 0; i < MOTOR_NUM; i++) {
            if (i == motor_idx) {
                Safe_Control(i, cmd_p, cmd_v,
                             move_kp[i], move_kd[i], move_tff);
            } else if (Motor_Homed[i]) {
                Safe_Control(i, Current_Targets[i], 0.0f,
                             extreme_hold_kp[i], extreme_hold_kd[i], Get_Extreme_Hold_Tff(i));
            } else {
                Safe_Control(i, Current_Targets[i], 0.0f,
                             lock_kp[i], lock_kd[i], Get_Hold_Tff(i));
            }
        }

        Delay_ms(INTERVAL_MS);
    }

    Current_Targets[motor_idx] = target_p;
    if (mark_homed_after) Motor_Homed[motor_idx] = 1;
}

void Move_Motor_To_Rel(int motor_idx, float target_rel, uint8_t mark_homed_after, uint8_t use_shortest_wrap_for_base)
{
    float abs_p;
    if (WorldCoord_AbsFromRel((uint8_t)motor_idx, target_rel, &abs_p) != WORLD_OK) {
        return;
    }
    Move_Motor_To_Target(motor_idx, abs_p, mark_homed_after, use_shortest_wrap_for_base);
}

float Move_Motor_To_Rel_FromFeedback(int motor_idx, float target_rel, uint8_t mark_homed_after, uint8_t use_shortest_wrap_for_base)
{
    float current_abs, target_abs, delta;
    WorldCoord_Status_t st;

    if (motor_idx < 0 || motor_idx >= MOTOR_NUM) return 0.0f;

    /*
      关键点：用“反馈角度”作为运动起点，而不是用 Current_Targets 里残留的旧目标。
      否则上电/掉电恢复/外部跳变指令时，轨迹会从错误起点插补，造成看起来“突然跳动”。
    */
    Read_All_Current_Positions();
    current_abs = Motor_States[motor_idx].pos;
    Current_Targets[motor_idx] = current_abs;

    /*
      树莓派给的是 world_rel（相对 HOME）。
      这里同时计算出：target_abs（绝对目标角）与 delta（仍需转动的增量）。
    */
    st = WorldCoord_DeltaToTarget((uint8_t)motor_idx, current_abs, target_rel, &delta, &target_abs);
    if (st != WORLD_OK) return 0.0f;

    (void)use_shortest_wrap_for_base;

    /*
      这里仍然调用原来的 Move_Motor_To_Target（轨迹与保持参数完全不变），
      从而保证“运动效果”一致，只改变最终转到的角度。
    */
    Move_Motor_To_Target(motor_idx, target_abs, mark_homed_after, 0);
    return delta;
}

/*================ 双轴同步轨迹函数 ================*/
void Move_Two_Motors_To_Targets(int motor_a, float target_a,
                               int motor_b, float target_b,
                               uint8_t mark_a_homed_after,
                               uint8_t mark_b_homed_after)
{
    float start_a = Current_Targets[motor_a];
    float start_b = Current_Targets[motor_b];
    float dist_a, dist_b;
    float total_time_a, total_time_b, total_time;
    float speed_a, speed_b;
    float ratio, blend, vel_shape;
    float cmd_p_a, cmd_v_a, cmd_p_b, cmd_v_b;
    float tff_a, tff_b;
    int steps, s, i;

    dist_a = target_a - start_a;
    dist_b = target_b - start_b;

    tff_a = Get_Move_Tff(motor_a, dist_a);
    tff_b = Get_Move_Tff(motor_b, dist_b);

    speed_a = move_speed_limit[motor_a] * MOTION_SPEED_SCALE;
    speed_b = move_speed_limit[motor_b] * MOTION_SPEED_SCALE;

    if (fabsf(dist_a) < 0.001f && fabsf(dist_b) < 0.001f) {
        Current_Targets[motor_a] = target_a;
        Current_Targets[motor_b] = target_b;
        if (mark_a_homed_after) Motor_Homed[motor_a] = 1;
        if (mark_b_homed_after) Motor_Homed[motor_b] = 1;
        return;
    }

    total_time_a = (fabsf(dist_a) < 0.001f) ? 0.25f : fabsf(dist_a) * 3.1415926535f / (2.0f * speed_a);
    total_time_b = (fabsf(dist_b) < 0.001f) ? 0.25f : fabsf(dist_b) * 3.1415926535f / (2.0f * speed_b);

    total_time = (total_time_a > total_time_b) ? total_time_a : total_time_b;
    if (total_time < 0.25f) total_time = 0.25f;

    steps = (int)(total_time / (INTERVAL_MS * 0.001f));
    if (steps < 80) steps = 80;

    for (s = 0; s <= steps; s++) {
        if (Emergency_Stop) return;

        if (Motor_Is_Fault(motor_a)) {
            Latch_Fault((uint8_t)motor_a, Motor_States[motor_a].err);
            return;
        }
        if (Motor_Is_Fault(motor_b)) {
            Latch_Fault((uint8_t)motor_b, Motor_States[motor_b].err);
            return;
        }

        ratio     = (float)s / (float)steps;
        blend     = 0.5f - 0.5f * cosf(3.1415926535f * ratio);
        vel_shape = 0.5f * 3.1415926535f * sinf(3.1415926535f * ratio);

        cmd_p_a = start_a + dist_a * blend;
        cmd_p_b = start_b + dist_b * blend;

        cmd_v_a = (dist_a / total_time) * vel_shape;
        cmd_v_b = (dist_b / total_time) * vel_shape;

        Current_Targets[motor_a] = cmd_p_a;
        Current_Targets[motor_b] = cmd_p_b;

        for (i = 0; i < MOTOR_NUM; i++) {
            if (i == motor_a) {
                Safe_Control(i, cmd_p_a, cmd_v_a,
                             move_kp[i], move_kd[i], tff_a);
            } else if (i == motor_b) {
                Safe_Control(i, cmd_p_b, cmd_v_b,
                             move_kp[i], move_kd[i], tff_b);
            } else if (Motor_Homed[i]) {
                Safe_Control(i, Current_Targets[i], 0.0f,
                             extreme_hold_kp[i], extreme_hold_kd[i], Get_Extreme_Hold_Tff(i));
            } else {
                Safe_Control(i, Current_Targets[i], 0.0f,
                             lock_kp[i], lock_kd[i], Get_Hold_Tff(i));
            }
        }

        Delay_ms(INTERVAL_MS);
    }

    Current_Targets[motor_a] = target_a;
    Current_Targets[motor_b] = target_b;

    if (mark_a_homed_after) Motor_Homed[motor_a] = 1;
    if (mark_b_homed_after) Motor_Homed[motor_b] = 1;
}

void Move_Two_Motors_To_Rels(int motor_a, float target_rel_a,
                             int motor_b, float target_rel_b,
                             uint8_t mark_a_homed_after,
                             uint8_t mark_b_homed_after)
{
    float abs_a, abs_b;
    if (WorldCoord_AbsFromRel((uint8_t)motor_a, target_rel_a, &abs_a) != WORLD_OK) return;
    if (WorldCoord_AbsFromRel((uint8_t)motor_b, target_rel_b, &abs_b) != WORLD_OK) return;
    Move_Two_Motors_To_Targets(motor_a, abs_a, motor_b, abs_b, mark_a_homed_after, mark_b_homed_after);
}

/*================ 四轴同步轨迹函数 ================*/
void Move_Four_Motors_To_Targets(int m0, float t0,
                                 int m1, float t1,
                                 int m2, float t2,
                                 int m3, float t3,
                                 uint8_t mark_homed_after[4])
{
    int motors[4] = {m0, m1, m2, m3};
    float target[4] = {t0, t1, t2, t3};
    float start[4];
    float dist[4];
    float tff[4];
    float speed[4];
    float total_time[4];
    float total_time_max;
    int steps, s, i, k;

    /* 0) 简单有效性检查：编号范围 + 不允许重复轴号 */
    for (k = 0; k < 4; k++) {
        int idx = motors[k];
        if (idx < 0 || idx >= MOTOR_NUM) return;
        for (i = 0; i < k; i++) {
            if (motors[i] == idx) {
                /* 重复轴号：调用者配置错误，直接返回 */
                return;
            }
        }
    }

    /* 1) 起点/位移计算（起点使用 Current_Targets） */
    for (k = 0; k < 4; k++) {
        int idx = motors[k];

        start[k] = Current_Targets[idx];
        dist[k]  = target[k] - start[k];
        tff[k]   = Get_Move_Tff(idx, dist[k]);
        speed[k] = move_speed_limit[idx] * MOTION_SPEED_SCALE;
        if (speed[k] <= 0.001f) speed[k] = 0.001f;
    }

    /* 2) 特殊情况：所有位移都极小，直接落点并标记 homed */
    {
        uint8_t all_small = 1;
        for (k = 0; k < 4; k++) {
            if (fabsf(dist[k]) >= 0.001f) {
                all_small = 0;
                break;
            }
        }
        if (all_small) {
            for (k = 0; k < 4; k++) {
                int idx = motors[k];
                Current_Targets[idx] = target[k];
                if (mark_homed_after && mark_homed_after[k]) {
                    Motor_Homed[idx] = 1;
                }
            }
            return;
        }
    }

    /* 3) 计算每轴时间，选择最大值为同步时间 */
    total_time_max = 0.25f;
    for (k = 0; k < 4; k++) {
        if (fabsf(dist[k]) < 0.001f) {
            total_time[k] = 0.25f;
        } else {
            total_time[k] = fabsf(dist[k]) * 3.1415926535f / (2.0f * speed[k]);
            if (total_time[k] < 0.25f) total_time[k] = 0.25f;
        }
        if (total_time[k] > total_time_max) {
            total_time_max = total_time[k];
        }
    }

    steps = (int)(total_time_max / (INTERVAL_MS * 0.001f));
    if (steps < 80) steps = 80;

    /* 4) 插补循环：4 轴同步走同一个 ratio/blend/vel_shape */
    for (s = 0; s <= steps; s++) {
        float ratio     = (float)s / (float)steps;
        float blend     = 0.5f - 0.5f * cosf(3.1415926535f * ratio);
        float vel_shape = 0.5f * 3.1415926535f * sinf(3.1415926535f * ratio);

        if (Emergency_Stop) return;

        /* 故障检查：任一轴故障则直接锁存并退出 */
        for (k = 0; k < 4; k++) {
            int idx = motors[k];
            if (Motor_Is_Fault(idx)) {
                Latch_Fault((uint8_t)idx, Motor_States[idx].err);
                return;
            }
        }

        /* 计算每轴当前指令位置/速度 */
        float cmd_p[4];
        float cmd_v[4];
        for (k = 0; k < 4; k++) {
            cmd_p[k] = start[k] + dist[k] * blend;
            cmd_v[k] = (dist[k] / total_time_max) * vel_shape;
        }

        /* 更新 Current_Targets */
        for (k = 0; k < 4; k++) {
            int idx = motors[k];
            Current_Targets[idx] = cmd_p[k];
        }

        /* 下发控制：4 轴为移动控制，其余轴按原保持策略 */
        for (i = 0; i < MOTOR_NUM; i++) {
            int found = -1;
            for (k = 0; k < 4; k++) {
                if (motors[k] == i) {
                    found = k;
                    break;
                }
            }

            if (found >= 0) {
                int idx = motors[found];
                Safe_Control(idx, cmd_p[found], cmd_v[found],
                             move_kp[idx], move_kd[idx], tff[found]);
            } else if (Motor_Homed[i]) {
                Safe_Control(i, Current_Targets[i], 0.0f,
                             extreme_hold_kp[i], extreme_hold_kd[i], Get_Extreme_Hold_Tff(i));
            } else {
                Safe_Control(i, Current_Targets[i], 0.0f,
                             lock_kp[i], lock_kd[i], Get_Hold_Tff(i));
            }
        }

        Delay_ms(INTERVAL_MS);
    }

    /* 5) 最终落点与 homed 标记 */
    for (k = 0; k < 4; k++) {
        int idx = motors[k];
        Current_Targets[idx] = target[k];
        if (mark_homed_after && mark_homed_after[k]) {
            Motor_Homed[idx] = 1;
        }
    }
}

void Move_Four_Motors_To_Rels(int m0, float r0,
                              int m1, float r1,
                              int m2, float r2,
                              int m3, float r3,
                              uint8_t mark_homed_after[4])
{
    float t0, t1, t2, t3;
    if (WorldCoord_AbsFromRel((uint8_t)m0, r0, &t0) != WORLD_OK) return;
    if (WorldCoord_AbsFromRel((uint8_t)m1, r1, &t1) != WORLD_OK) return;
    if (WorldCoord_AbsFromRel((uint8_t)m2, r2, &t2) != WORLD_OK) return;
    if (WorldCoord_AbsFromRel((uint8_t)m3, r3, &t3) != WORLD_OK) return;

    Move_Four_Motors_To_Targets(m0, t0, m1, t1, m2, t2, m3, t3, mark_homed_after);
}

/* 从真实反馈起步、四轴相对 HOME 同步运动（不做底座 WrapDeltaPi、不做 P_LIMIT） */
void Move_Four_Motors_FromFeedback_To_Rels(int m0, float r0,
                                           int m1, float r1,
                                           int m2, float r2,
                                           int m3, float r3,
                                           uint8_t mark_homed_after[4])
{
    int motors[4] = {m0, m1, m2, m3};
    float target_rel[4] = {r0, r1, r2, r3};
    float start[4];
    float target_abs[4];
    float dist[4];
    float tff[4];
    float speed[4];
    float total_time[4];
    float total_time_max;
    int steps, s, i, k;

    /* 0) 轴号合法性 + 不允许重复轴号 */
    for (k = 0; k < 4; k++) {
        int idx = motors[k];
        if (idx < 0 || idx >= MOTOR_NUM) return;
        for (i = 0; i < k; i++) {
            if (motors[i] == idx) {
                return;
            }
        }
    }

    /* 1) 用真实反馈作为起点，并同步 Current_Targets 防止跳动 */
    Read_All_Current_Positions();
    for (k = 0; k < 4; k++) {
        int idx = motors[k];
        start[k] = Motor_States[idx].pos;
        Current_Targets[idx] = start[k];
    }

    /* 2) 目标：相对 HOME -> 绝对目标（线性，与 WorldCoord_AbsFromRel 一致） */
    for (k = 0; k < 4; k++) {
        int idx = motors[k];
        float delta;
        WorldCoord_Status_t st = WorldCoord_DeltaToTarget((uint8_t)idx,
                                                          start[k],
                                                          target_rel[k],
                                                          &delta,
                                                          &target_abs[k]);
        if (st != WORLD_OK) return;

        dist[k]  = target_abs[k] - start[k];
        tff[k]   = Get_Move_Tff(idx, dist[k]);
        speed[k] = move_speed_limit[idx] * MOTION_SPEED_SCALE;
        if (speed[k] <= 0.001f) speed[k] = 0.001f;
    }

    /* 3) 所有位移都极小时直接落点 */
    {
        uint8_t all_small = 1;
        for (k = 0; k < 4; k++) {
            if (fabsf(dist[k]) >= 0.001f) {
                all_small = 0;
                break;
            }
        }
        if (all_small) {
            for (k = 0; k < 4; k++) {
                int idx = motors[k];
                Current_Targets[idx] = target_abs[k];
                if (mark_homed_after && mark_homed_after[k]) {
                    Motor_Homed[idx] = 1;
                }
            }
            return;
        }
    }

    /* 4) 计算每轴时间，选择最大值为同步时间 */
    total_time_max = 0.25f;
    for (k = 0; k < 4; k++) {
        if (fabsf(dist[k]) < 0.001f) {
            total_time[k] = 0.25f;
        } else {
            total_time[k] = fabsf(dist[k]) * 3.1415926535f / (2.0f * speed[k]);
            if (total_time[k] < 0.25f) total_time[k] = 0.25f;
        }
        if (total_time[k] > total_time_max) {
            total_time_max = total_time[k];
        }
    }

    steps = (int)(total_time_max / (INTERVAL_MS * 0.001f));
    if (steps < 80) steps = 80;

    /* 5) 插补循环 */
    for (s = 0; s <= steps; s++) {
        float ratio     = (float)s / (float)steps;
        float blend     = 0.5f - 0.5f * cosf(3.1415926535f * ratio);
        float vel_shape = 0.5f * 3.1415926535f * sinf(3.1415926535f * ratio);

        if (Emergency_Stop) return;

        /* 故障检测 */
        for (k = 0; k < 4; k++) {
            int idx = motors[k];
            if (Motor_Is_Fault(idx)) {
                Latch_Fault((uint8_t)idx, Motor_States[idx].err);
                return;
            }
        }

        /* 计算每轴当前指令位置/速度 */
        float cmd_p[4];
        float cmd_v[4];
        for (k = 0; k < 4; k++) {
            cmd_p[k] = start[k] + dist[k] * blend;
            cmd_v[k] = (dist[k] / total_time_max) * vel_shape;
        }

        /* 更新 Current_Targets */
        for (k = 0; k < 4; k++) {
            int idx = motors[k];
            Current_Targets[idx] = cmd_p[k];
        }

        /* 下发控制 */
        for (i = 0; i < MOTOR_NUM; i++) {
            int found = -1;
            for (k = 0; k < 4; k++) {
                if (motors[k] == i) {
                    found = k;
                    break;
                }
            }

            if (found >= 0) {
                int idx = motors[found];
                Safe_Control(idx, cmd_p[found], cmd_v[found],
                             move_kp[idx], move_kd[idx], tff[found]);
            } else if (Motor_Homed[i]) {
                Safe_Control(i, Current_Targets[i], 0.0f,
                             extreme_hold_kp[i], extreme_hold_kd[i], Get_Extreme_Hold_Tff(i));
            } else {
                Safe_Control(i, Current_Targets[i], 0.0f,
                             lock_kp[i], lock_kd[i], Get_Hold_Tff(i));
            }
        }

        Delay_ms(INTERVAL_MS);
    }

    /* 6) 最终落点与 homed 标记 */
    for (k = 0; k < 4; k++) {
        int idx = motors[k];
        Current_Targets[idx] = target_abs[k];
        if (mark_homed_after && mark_homed_after[k]) {
            Motor_Homed[idx] = 1;
        }
    }
}


