#include "motor_control.h"
#include "can_communication.h"
#include "Serial.h"
#include <math.h>

/*================ 额外流程参数 ================*/
#define MOTION_SPEED_SCALE       0.5f  /* 默认全局速度倍率（可运行时通过 Motor_SetSpeedScale 调整） */
#define RETURN_SETTLE_MS         80
#define FINAL_HOLD_MS            120
#define REG_READ_WAIT_MS         30
#define DRIVER_RANGE_SYNC_ENABLE 1

#define PI 3.1415926535f

/*================ 物理限位定义 ================*/
const float P_LIMIT_MIN[MOTOR_NUM] = {-12.50f, -0.45f, -0.70f, -0.70f};
const float P_LIMIT_MAX[MOTOR_NUM] = { 12.50f,  2.70f,  2.70f,  2.70f};

/*================ 全局变量定义 ================*/
uint32_t Motor_IDs[MOTOR_NUM] = {0x01, 0x02, 0x03, 0x04};
uint32_t Motor_Master_IDs[MOTOR_NUM] = {0x11, 0x12, 0x13, 0x14};
float Motor_Home[MOTOR_NUM] = {3.134f, 1.439f, 2.483f, 1.478f};
uint8_t Servo1_Angle = 0;
uint8_t Servo2_Angle = 0;

float Extreme_Hold_Kp[MOTOR_NUM] = {
    18.0f,   /* 电机1 */
    62.0f,   /* 电机2：主承重，4340可给更硬 */
    42.0f,   /* 电机3 */
    24.0f    /* 电机4 */
};

float Extreme_Hold_Kd[MOTOR_NUM] = {
    1.8f,
    3.2f,
    2.8f,
    1.8f
};

float Extreme_Hold_Tff[MOTOR_NUM] = {
    0.0f,
    0.45f,
    0.85f,
    0.0f
};

volatile uint8_t Extreme_Test_Hold_Active = 0;

/* 全局速度倍率，所有 Move_* 轨迹规划都会乘以该因子。
 * 默认值来自 MOTION_SPEED_SCALE，可以通过 Motor_SetSpeedScale 在运行时统一调节快慢。 */
static float g_speed_scale = MOTION_SPEED_SCALE;

void Motor_SetSpeedScale(float scale)
{
    /* 夹在一个保守区间内，避免误设置为 0 或特别大导致运动异常 */
    if (scale < 0.1f) {
        scale = 0.1f;
    } else if (scale > 2.0f) {
        scale = 2.0f;
    }
    g_speed_scale = scale;
}

float Move_Kp[MOTOR_NUM] = {
    KP_MOVE_BASE,
    30.0f,
    42.0f,
    KP_MOVE_LIGHT
};

float Move_Kd[MOTOR_NUM] = {
    2.0f,
    1.8f,
    2.2f,
    KD_LIGHT
};

float Move_Speed_Limit[MOTOR_NUM] = {
    0.05f,  /* 电机1：进一步降低速度，避免乱转 */
    0.08f,  /* 电机2：降低速度 */
    0.05f,  /* 电机3：降低速度 */
    0.08f   /* 电机4：降低速度 */
};

float Hold_Kp[MOTOR_NUM] = {
    KP_HOLD_SOFT,
    9.0f,
    12.0f,
    KP_HOLD_SOFT
};

float Hold_Kd[MOTOR_NUM] = {
    1.0f,
    1.2f,
    1.6f,
    1.0f
};

float Lock_Kp[MOTOR_NUM] = {
    10.0f, 16.0f, 16.0f, 10.0f
};

float Lock_Kd[MOTOR_NUM] = {
    1.2f, 1.8f, 1.8f, 1.2f
};

float Stabilize_Kp[MOTOR_NUM] = {
    10.0f, 16.0f, 20.0f, 10.0f
};

float Stabilize_Kd[MOTOR_NUM] = {
    1.2f, 1.8f, 2.2f, 1.2f
};

float Move_Tff_Positive[MOTOR_NUM] = {0.0f, 0.35f, 1.20f, 0.0f};
float Move_Tff_Negative[MOTOR_NUM] = {0.0f, 0.05f, 0.40f, 0.0f};
float Hold_Tff[MOTOR_NUM]          = {0.0f, 0.10f, 0.80f, 0.0f};

float Motor2_Move_Tff_Pos = 0.45f;
float Motor2_Move_Tff_Neg = 0.08f;
float Motor2_Hold_Tff     = 0.20f;

float Runtime_P_Min[MOTOR_NUM] = {MIT_P_MIN, MIT_P_MIN, MIT_P_MIN, MIT_P_MIN};
float Runtime_P_Max[MOTOR_NUM] = {MIT_P_MAX, MIT_P_MAX, MIT_P_MAX, MIT_P_MAX};
float Runtime_V_Min[MOTOR_NUM] = {MIT_V_MIN, MIT_V_MIN, MIT_V_MIN, MIT_V_MIN};
float Runtime_V_Max[MOTOR_NUM] = {MIT_V_MAX, MIT_V_MAX, MIT_V_MAX, MIT_V_MAX};
float Runtime_T_Min[MOTOR_NUM] = {MIT_T_MIN, MIT_T_MIN, MIT_T_MIN, MIT_T_MIN};
float Runtime_T_Max[MOTOR_NUM] = {MIT_T_MAX, MIT_T_MAX, MIT_T_MAX, MIT_T_MAX};

float Current_Targets[MOTOR_NUM];
volatile Motor_Status_t Motor_States[MOTOR_NUM];
volatile uint8_t Motor_Feedback_Received[MOTOR_NUM];
uint8_t Motor_Homed[MOTOR_NUM];

volatile uint8_t Emergency_Stop = 0;
volatile uint8_t System_Disabled = 0;
volatile uint8_t Fault_Motor = 0xFF;
volatile uint8_t Fault_Code  = 0x0;
volatile Motor_RegResp_t Motor_RegResp[MOTOR_NUM];

/*================ 基础工具函数 ================*/
static void Delay_ms(uint32_t ms)
{
    SysTick->LOAD = SystemCoreClock / 1000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while (ms--) {
        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
    }
    SysTick->CTRL = 0;
}

// static void Delay_us(uint32_t us)
// {
//     volatile uint32_t i = us * 8;
//     while (i--) { ; }
// }

static uint8_t Motor_Is_Fault(int idx)
{
    uint8_t e = Motor_States[idx].err;
    // 只在确实有严重故障时返回1，避免误触发
    // 0(MOTOR_ERR_DISABLE)和1(MOTOR_ERR_ENABLE)都是正常状态
    // 8及以上才是真正的故障
    if (e == MOTOR_ERR_DISABLE || e == MOTOR_ERR_ENABLE) return 0;
    // 对于其他值，只在8及以上时视为故障
    if (e >= 8) return 1;
    return 0;
}

/*================ 电机控制函数 ================*/
void Disable_All_Motors(void)
{
    int i, r;

    if (System_Disabled) return;

    for (r = 0; r < 3; r++) {
        for (i = 0; i < MOTOR_NUM; i++) {
            Motor_Send_Special(Motor_IDs[i], 0xFD);
        }
        Delay_ms(2);
    }

    System_Disabled = 1;
}

void Latch_Fault(uint8_t idx, uint8_t code)
{
    if (!Emergency_Stop) {
        Fault_Motor = idx;
        Fault_Code  = code;
        Emergency_Stop = 1;
    }
}

float Get_Move_Tff(int idx, float dist)
{
    if (idx == 1) {
        return (dist >= 0.0f) ? Motor2_Move_Tff_Pos : Motor2_Move_Tff_Neg;
    }
    return (dist >= 0.0f) ? Move_Tff_Positive[idx] : Move_Tff_Negative[idx];
}

float Get_Hold_Tff(int idx)
{
    if (idx == 1) return Motor2_Hold_Tff;
    return Hold_Tff[idx];
}

float Get_Extreme_Hold_Tff(int idx)
{
    return Extreme_Hold_Tff[idx];
}

void Safe_Control(int idx, float p, float v, float kp, float kd, float t)
{
    if (Emergency_Stop) return;

    if (p < P_LIMIT_MIN[idx]) p = P_LIMIT_MIN[idx];
    if (p > P_LIMIT_MAX[idx]) p = P_LIMIT_MAX[idx];

    if (t < Runtime_T_Min[idx]) t = Runtime_T_Min[idx];
    if (t > Runtime_T_Max[idx]) t = Runtime_T_Max[idx];

    if (kd < 0.15f) kd = 0.15f;
    if (kp < 0.0f) kp = 0.0f;

    Motor_MIT_Send_Raw(idx, p, v, kp, kd, t);
}

void Apply_Rigid_Hold_One_Cycle(void)
{
    int i;

    for (i = 0; i < MOTOR_NUM; i++) {
        if (Motor_Homed[i]) {
            Safe_Control(i,
                         Current_Targets[i],
                         0.0f,
                         Extreme_Hold_Kp[i],
                         Extreme_Hold_Kd[i],
                         Get_Extreme_Hold_Tff(i));
        } else {
            Safe_Control(i,
                         Current_Targets[i],
                         0.0f,
                         Lock_Kp[i],
                         Lock_Kd[i],
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

void Move_Motor_To_Target(int motor_idx, float target_p, uint8_t mark_homed_after, uint8_t use_shortest_wrap_for_base)
{
    float start_p = Current_Targets[motor_idx];
    float dist, total_time, ratio, blend, vel_shape;
    float cmd_p, cmd_v, move_tff;
    float speed_limit;
    int steps, s, i;

    if ((motor_idx == 0) && use_shortest_wrap_for_base) {
        float rem  = fmodf(start_p, 2.0f * PI);
        float diff = target_p - rem;
        if (diff > PI)  diff -= 2.0f * PI;
        if (diff < -PI) diff += 2.0f * PI;
        target_p = start_p + diff;
    }

    if (target_p < P_LIMIT_MIN[motor_idx]) target_p = P_LIMIT_MIN[motor_idx];
    if (target_p > P_LIMIT_MAX[motor_idx]) target_p = P_LIMIT_MAX[motor_idx];

    dist = target_p - start_p;
    move_tff = Get_Move_Tff(motor_idx, dist);
    speed_limit = Move_Speed_Limit[motor_idx] * g_speed_scale;

    if (fabsf(dist) < 0.001f) {
        Current_Targets[motor_idx] = target_p;
        if (mark_homed_after) Motor_Homed[motor_idx] = 1;
        return;
    }

    total_time = fabsf(dist) * PI / (2.0f * speed_limit);
    if (total_time < 0.5f) total_time = 0.5f;  /* 增加最小运动时间，使运动更慢 */

    steps = (int)(total_time / (INTERVAL_MS * 0.001f));
    if (steps < 160) steps = 160;  /* 增加最少步数，使运动更平滑 */

    for (s = 0; s <= steps; s++) {
        if (Emergency_Stop) return;

        if (Motor_Is_Fault(motor_idx)) {
            Latch_Fault((uint8_t)motor_idx, Motor_States[motor_idx].err);
            return;
        }

        ratio     = (float)s / (float)steps;
        blend     = 0.5f - 0.5f * cosf(PI * ratio);
        vel_shape = 0.5f * PI * sinf(PI * ratio);

        cmd_p = start_p + dist * blend;
        cmd_v = (dist / total_time) * vel_shape;

        Current_Targets[motor_idx] = cmd_p;

        for (i = 0; i < MOTOR_NUM; i++) {
            if (i == motor_idx) {
                Safe_Control(i, cmd_p, cmd_v,
                             Move_Kp[i], Move_Kd[i], move_tff);
            } else if (Motor_Homed[i]) {
                Safe_Control(i, Current_Targets[i], 0.0f,
                             Extreme_Hold_Kp[i], Extreme_Hold_Kd[i], Get_Extreme_Hold_Tff(i));
            } else {
                Safe_Control(i, Current_Targets[i], 0.0f,
                             Lock_Kp[i], Lock_Kd[i], Get_Hold_Tff(i));
            }
        }

        Delay_ms(INTERVAL_MS);
    }

    Current_Targets[motor_idx] = target_p;
    if (mark_homed_after) Motor_Homed[motor_idx] = 1;
}

void Move_Two_Motors_To_Targets(int motor_a, float target_a, int motor_b, float target_b, uint8_t mark_a_homed_after, uint8_t mark_b_homed_after)
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

    if (target_a < P_LIMIT_MIN[motor_a]) target_a = P_LIMIT_MIN[motor_a];
    if (target_a > P_LIMIT_MAX[motor_a]) target_a = P_LIMIT_MAX[motor_a];
    if (target_b < P_LIMIT_MIN[motor_b]) target_b = P_LIMIT_MIN[motor_b];
    if (target_b > P_LIMIT_MAX[motor_b]) target_b = P_LIMIT_MAX[motor_b];

    dist_a = target_a - start_a;
    dist_b = target_b - start_b;

    tff_a = Get_Move_Tff(motor_a, dist_a);
    tff_b = Get_Move_Tff(motor_b, dist_b);

    speed_a = Move_Speed_Limit[motor_a] * g_speed_scale;
    speed_b = Move_Speed_Limit[motor_b] * g_speed_scale;

    if (fabsf(dist_a) < 0.001f && fabsf(dist_b) < 0.001f) {
        Current_Targets[motor_a] = target_a;
        Current_Targets[motor_b] = target_b;
        if (mark_a_homed_after) Motor_Homed[motor_a] = 1;
        if (mark_b_homed_after) Motor_Homed[motor_b] = 1;
        return;
    }

    total_time_a = (fabsf(dist_a) < 0.001f) ? 0.25f : fabsf(dist_a) * PI / (2.0f * speed_a);
    total_time_b = (fabsf(dist_b) < 0.001f) ? 0.25f : fabsf(dist_b) * PI / (2.0f * speed_b);

    total_time = (total_time_a > total_time_b) ? total_time_a : total_time_b;
    if (total_time < 0.5f) total_time = 0.5f;  /* 增加最小运动时间，使运动更慢 */

    steps = (int)(total_time / (INTERVAL_MS * 0.001f));
    if (steps < 160) steps = 160;  /* 增加最少步数，使运动更平滑 */

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
        blend     = 0.5f - 0.5f * cosf(PI * ratio);
        vel_shape = 0.5f * PI * sinf(PI * ratio);

        cmd_p_a = start_a + dist_a * blend;
        cmd_p_b = start_b + dist_b * blend;

        cmd_v_a = (dist_a / total_time) * vel_shape;
        cmd_v_b = (dist_b / total_time) * vel_shape;

        Current_Targets[motor_a] = cmd_p_a;
        Current_Targets[motor_b] = cmd_p_b;

        for (i = 0; i < MOTOR_NUM; i++) {
            if (i == motor_a) {
                Safe_Control(i, cmd_p_a, cmd_v_a,
                             Move_Kp[i], Move_Kd[i], tff_a);
            } else if (i == motor_b) {
                Safe_Control(i, cmd_p_b, cmd_v_b,
                             Move_Kp[i], Move_Kd[i], tff_b);
            } else if (Motor_Homed[i]) {
                Safe_Control(i, Current_Targets[i], 0.0f,
                             Extreme_Hold_Kp[i], Extreme_Hold_Kd[i], Get_Extreme_Hold_Tff(i));
            } else {
                Safe_Control(i, Current_Targets[i], 0.0f,
                             Lock_Kp[i], Lock_Kd[i], Get_Hold_Tff(i));
            }
        }

        Delay_ms(INTERVAL_MS);
    }

    Current_Targets[motor_a] = target_a;
    Current_Targets[motor_b] = target_b;

    if (mark_a_homed_after) Motor_Homed[motor_a] = 1;
    if (mark_b_homed_after) Motor_Homed[motor_b] = 1;
}

void Move_All_Motors_To_Targets(float target_0, float target_1, float target_2, float target_3)
{
    float starts[MOTOR_NUM], dists[MOTOR_NUM], tff[MOTOR_NUM];
    float total_time = 1.0f;   /* 至少 1s，确保运动非常缓慢 */
    int steps, s, i;
    float ratio, blend, vel_shape;  /* 插值：blend 位置，vel_shape 速度形状 */
    float cmd_p[MOTOR_NUM], cmd_v[MOTOR_NUM];
    float targets[MOTOR_NUM] = { target_0, target_1, target_2, target_3 };

    /* 限幅到物理限位，并计算各轴起点、位移、前馈；取最慢轴时间作为总时间 */
    for (i = 0; i < MOTOR_NUM; i++) {
        if (targets[i] < P_LIMIT_MIN[i]) targets[i] = P_LIMIT_MIN[i];
        if (targets[i] > P_LIMIT_MAX[i]) targets[i] = P_LIMIT_MAX[i];
        starts[i] = Current_Targets[i];
        dists[i] = targets[i] - starts[i];
        tff[i] = Get_Move_Tff(i, dists[i]);
        if (fabsf(dists[i]) > 0.001f) {
            float t = fabsf(dists[i]) * PI / (2.0f * Move_Speed_Limit[i] * g_speed_scale);
            if (t > total_time) total_time = t;
        }
    }

    if (total_time < 1.0f) total_time = 1.0f;  /* 增加最小运动时间，使运动更慢 */
    steps = (int)(total_time / (INTERVAL_MS * 0.001f));
    if (steps < 200) steps = 200;   /* 增加最少步数，使运动更平滑 */

    /* 按步插值：余弦平滑，每步发一次 MIT 指令 */
    for (s = 0; s <= steps; s++) {
        if (Emergency_Stop) return;
        
        // 临时禁用故障检测，避免移动过程中的误触发
        // for (i = 1; i < MOTOR_NUM; i++) {
        //     if (Motor_Is_Fault(i)) {
        //         Latch_Fault((uint8_t)i, Motor_States[i].err);
        //         return;
        //     }
        // }

        ratio     = (float)s / (float)steps;                      /* 0~1 进度 */
        blend     = 0.5f - 0.5f * cosf(PI * ratio);              /* 余弦缓动 */
        vel_shape = 0.5f * PI * sinf(PI * ratio);                /* 对应速度曲线 */

        for (i = 0; i < MOTOR_NUM; i++) {
            cmd_p[i] = starts[i] + dists[i] * blend;
            cmd_v[i] = (dists[i] / total_time) * vel_shape;
            Current_Targets[i] = cmd_p[i];
        }

        for (i = 1; i < MOTOR_NUM; i++) {  // 从1开始，跳过第一个电机
            Safe_Control(i, cmd_p[i], cmd_v[i], Move_Kp[i], Move_Kd[i], tff[i]);
        }
        Delay_ms(INTERVAL_MS);
    }

    /* 更新当前目标与“已回位”标志，便于后续刚性保持 */
    for (i = 0; i < MOTOR_NUM; i++) {
        Current_Targets[i] = targets[i];
        Motor_Homed[i] = 1;
    }
}

/*================ 极限测试流程 ================*/
void Extreme_Test_Sequence(void)
{
    Read_All_Current_Positions();

    /* 底座视为已就位，并从一开始就刚性保持 */
    Motor_Homed[0] = 1;

    /* 先把腕关节抬起 */
    Move_Motor_To_Target(3, MOTOR4_PRESET_POS, 1, 0);

    /* 再同步抬起电机3和电机2 */
    Move_Two_Motors_To_Targets(2, MOTOR3_TEST_POS,
                               1, MOTOR2_TEST_POS,
                               1, 1);

    /* 到达展开位后，立即进入硬保持 */
    Extreme_Test_Hold_Active = 1;
    Hold_All_Rigid(FINAL_HOLD_MS);

    /* 电机4 回到 0 */
    Move_Motor_To_Target(3, 0.0f, 1, 0);
    Hold_All_Rigid(RETURN_SETTLE_MS);

    /* 电机3 回到 0 */
    Move_Motor_To_Target(2, 0.0f, 1, 0);
    Hold_All_Rigid(RETURN_SETTLE_MS);

    /* 电机2 回到 0 */
    Move_Motor_To_Target(1, 0.0f, 1, 0);
    Hold_All_Rigid(RETURN_SETTLE_MS);
}

/*================ 串口上报：CAN 电机实际角度（与接收格式一致） ================*/
static void Send_Current_Motor_Angles_To_Serial(void)
{
    int i;
    float deg;
    uint8_t buf[SERIAL_RX_PACKET_LEN];

    /* 四电机：实际弧度 -> 相对 Home 的度数 -> 0~255 */
    for (i = 0; i < MOTOR_NUM; i++) {
        deg = (Motor_States[i].pos - Motor_Home[i]) * (180.0f / PI);
        if (deg > 180.0f)  deg = 180.0f;
        if (deg < -180.0f) deg = -180.0f;
        buf[i] = (uint8_t)((deg + 180.0f) * 255.0f / 360.0f);
    }
    buf[4] = Servo1_Angle;
    buf[5] = Servo2_Angle;

    Serial_SendByte(0xFF);
    Serial_SendArray(buf, SERIAL_RX_PACKET_LEN);
    Serial_SendByte(0xFE);
}

/*================ 树莓派串口解析与运动执行 ================*/
extern uint8_t Serial_RxPacket[];
void Parse_Serial_And_Move_Motors(void)
{
    int i;
    float deg, target_rad[MOTOR_NUM];
    float current_pos[MOTOR_NUM];

    /* 前 4 字节：相对 Home 的 ±180° -> 绝对弧度 */
    for (i = 0; i < MOTOR_NUM; i++) {
        deg = ((float)Serial_RxPacket[i] / 255.0f) * 360.0f - 180.0f;
        target_rad[i] = Motor_Home[i] + deg * (PI / 180.0f);
    }
    Servo1_Angle = Serial_RxPacket[4];
    Servo2_Angle = Serial_RxPacket[5];

    /* 通过USART2发送树莓派数据给上位机监控（运动前） */
    for (i = 0; i < MOTOR_NUM; i++) {
        current_pos[i] = Motor_States[i].pos;
    }
    Serial2_SendMonitorData((uint8_t *)Serial_RxPacket, current_pos);

    /* 四轴同步运动到目标 */
    Move_All_Motors_To_Targets(target_rad[0], target_rad[1], target_rad[2], target_rad[3]);

    /* 运动结束后短暂刚性保持 50ms，再读 CAN 反馈的静止角度并按相同格式上报树莓派 */
    Hold_All_Rigid(50);
    Send_Current_Motor_Angles_To_Serial();

    /* 通过USART2发送树莓派数据和电机实际角度给上位机监控（运动后） */
    for (i = 0; i < MOTOR_NUM; i++) {
        current_pos[i] = Motor_States[i].pos;
    }
    Serial2_SendMonitorData((uint8_t *)Serial_RxPacket, current_pos);
}
