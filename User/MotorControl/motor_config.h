#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <stdint.h>

/*================ 类型与故障码（原 motor_types.h） ================*/
#define MOTOR_ERR_DISABLE      0x0
#define MOTOR_ERR_ENABLE       0x1
#define MOTOR_ERR_OVERVOLT     0x8
#define MOTOR_ERR_UNDERVOLT    0x9
#define MOTOR_ERR_OVERCURRENT  0xA
#define MOTOR_ERR_MOS_TEMP     0xB
#define MOTOR_ERR_COIL_TEMP    0xC
#define MOTOR_ERR_CAN_LOST     0xD
#define MOTOR_ERR_OVERLOAD     0xE

typedef struct {
    float pos;
    float vel;
    float tor;
    uint16_t tor_raw12;
    uint8_t err;
    uint8_t mos_temp;
    uint8_t rotor_temp;
    uint8_t raw_frame[8];
    uint8_t raw_dlc;
} Motor_Status_t;

typedef struct {
    volatile uint8_t got;
    volatile uint8_t op;
    volatile uint8_t rid;
    volatile uint32_t data_u32;
} Motor_RegResp_t;

/*================ 用户配置参数区 (保留原参数，不删除不改) ================*/
#define MOTOR_NUM       4
#define MOTOR_IDS       {0x01, 0x02, 0x03, 0x04}
#define MOTOR_MASTER_IDS {0x11, 0x12, 0x13, 0x14}

/*================ 全系统角度单位约定 ================*/
/* 本工程内所有角度统一使用：弧度(rad) */

#define INTERVAL_MS      2

/*
 * MIT_HEX_MODE=1：STM32 仅作执行器，四轴 MIT 来自树莓派 39 字节二进制帧；
 * TIM4 刚性保持关闭（与 6fee739 一致）。
 */
#ifndef MIT_HEX_MODE
#define MIT_HEX_MODE  1
#endif

#define MOTOR_HOLD_TIM4_ENABLE   1
#if MIT_HEX_MODE
#undef MOTOR_HOLD_TIM4_ENABLE
#define MOTOR_HOLD_TIM4_ENABLE   0
#endif

/*================ 必要新增参数：请与上位机“控制幅值”保持一致 ================*/
#define MIT_P_MIN       (-12.5f)
#define MIT_P_MAX       ( 12.5f)
#define MIT_V_MIN       (-45.0f)
#define MIT_V_MAX       ( 45.0f)
#define MIT_T_MIN       (-18.0f)
#define MIT_T_MAX       ( 18.0f)

/* MIT_HEX_MODE=0 时：TIM4 刚性保持与读反馈 MIT 使用下列 Kp/Kd/Tff（MIT_HEX=1 时不在此配置） */
#if !MIT_HEX_MODE
#define EXTREME_HOLD_KP {\
    18.0f,\
    62.0f,\
    42.0f,\
    24.0f\
}

#define EXTREME_HOLD_KD {\
    1.8f,\
    3.2f,\
    2.8f,\
    1.8f\
}

#define EXTREME_HOLD_TFF {\
    0.0f,\
    0.0f,\
    0.0f,\
    0.0f\
}

#define LOCK_KP {10.0f, 16.0f, 16.0f, 10.0f}
#define LOCK_KD {1.2f, 1.8f, 1.8f, 1.2f}

#define HOLD_TFF          {0.0f, 0.10f, 0.80f, 0.0f}

#define MOTOR2_HOLD_TFF     0.20f
#endif

/*================ 物理限位 ================*/
/* 已关闭：motor_control / motor_world_exec 不再做软件 P_LIMIT 夹紧（机械安全请靠硬件/驱动器） */

/*================ 调试日志开关 ================*/
/*
 * 1=打开：FB 回传；主循环轮询 FB_Report_ServicePending
 * 0=关闭：无 FB 定时发送
 */
#define MOTOR_DEBUG_LOG_ENABLE   1

#if MIT_HEX_MODE
#define FB_REPORT_HZ             25
#else
#define FB_REPORT_HZ             10
#endif

/*================ 额外流程参数 ================*/
#define REG_READ_WAIT_MS         30
#define DRIVER_RANGE_SYNC_ENABLE 1

#endif /* MOTOR_CONFIG_H */
