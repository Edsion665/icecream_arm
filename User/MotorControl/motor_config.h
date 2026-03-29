#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/*================ 用户配置参数区 (保留原参数，不删除不改) ================*/
#define MOTOR_NUM       4
#define MOTOR_IDS       {0x01, 0x02, 0x03, 0x04}
#define MOTOR_MASTER_IDS {0x11, 0x12, 0x13, 0x14}

/*================ 全系统角度单位约定 ================*/
/* 本工程内所有角度统一使用：弧度(rad) */

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
#define MIT_V_MIN       (-45.0f)
#define MIT_V_MAX       ( 45.0f)
#define MIT_T_MIN       (-18.0f)
#define MIT_T_MAX       ( 18.0f)

/*================ 极限测试目标位置 ================*/
#define MOTOR4_PRESET_POS   1.70f
#define MOTOR3_TEST_POS     2.00f
#define MOTOR2_TEST_POS     2.40f

/*================ 硬性保持参数（用于所有到位后的持续刚性保持） ================*/
#define EXTREME_HOLD_KP {\
    18.0f,   /* 电机1 */\
    62.0f,   /* 电机2：主承重，4340可给更硬 */\
    42.0f,   /* 电机3 */\
    24.0f    /* 电机4 */\
}

#define EXTREME_HOLD_KD {\
    1.8f,\
    3.2f,\
    2.8f,\
    1.8f\
}

/* 到位后的前馈扭矩，用于持续托住重力（试跑可全 0 关固定前馈，恢复时取消下面注释块） */
#define EXTREME_HOLD_TFF {\
    0.0f,\
    0.0f,\
    0.0f,\
    0.0f\
}
/*
#define EXTREME_HOLD_TFF_ORIG {\
    0.0f,\
    0.45f,\
    0.85f,\
    0.0f\
}
*/

/*================ 每轴运动参数 ================*/
#define MOVE_KP {\
    KP_MOVE_BASE,\
    30.0f,\
    42.0f,\
    KP_MOVE_LIGHT\
}

#define MOVE_KD {\
    2.0f,\
    1.8f,\
    2.2f,\
    KD_LIGHT\
}

#define MOVE_SPEED_LIMIT {\
    HOMING_SPEED,\
    0.30f,\
    0.22f,\
    0.35f\
}

#define HOLD_KP {\
    KP_HOLD_SOFT,\
    9.0f,\
    12.0f,\
    KP_HOLD_SOFT\
}

#define HOLD_KD {\
    1.0f,\
    1.2f,\
    1.6f,\
    1.0f\
}

#define LOCK_KP {10.0f, 16.0f, 16.0f, 10.0f}
#define LOCK_KD {1.2f, 1.8f, 1.8f, 1.2f}

#define STABILIZE_KP {10.0f, 16.0f, 20.0f, 10.0f}
#define STABILIZE_KD {1.2f, 1.8f, 2.2f, 1.2f}

/*================ 前馈力矩补偿 ================*/
#define MOVE_TFF_POSITIVE {0.0f, 0.35f, 1.20f, 0.0f}
#define MOVE_TFF_NEGATIVE {0.0f, 0.05f, 0.40f, 0.0f}
#define HOLD_TFF          {0.0f, 0.10f, 0.80f, 0.0f}

#define MOTOR2_MOVE_TFF_POS 0.45f
#define MOTOR2_MOVE_TFF_NEG 0.08f
#define MOTOR2_HOLD_TFF     0.20f

/*================ 物理限位 ================*/
/* 已关闭：motor_control / motor_world_exec 不再做软件 P_LIMIT 夹紧（机械安全请靠硬件/驱动器） */

/*================ 世界坐标(HOME) / 树莓派关节零位标定 ================*/
#ifndef MOTOR_PI
#define MOTOR_PI 3.1415926535f
#endif
/* 与 DATA/FB 相同的线性标度：整数为度×100，换成驱动器使用的 rad */
#define RPI_DEG100_TO_RAD(d100) \
    ((((float)(d100)) * 0.01f) * (MOTOR_PI / 180.0f))

/*
 * 机械标定零位（输入：度×100）：摆到该姿态后，上位机坐标系记为 0,0,0,0。
 * 修改标定只需改下面四个数，WORLD_HOME_ABS 会自动一致。
 */
/* 标定更新：机械零位处 FB 相对旧 HOME 为 8976,-233,194,83(度×100) → 新 RAW = 旧 + 该校准读数 */
#define RPI_HOME_RAW_DEG100_J0  (8678)
#define RPI_HOME_RAW_DEG100_J1  (7826)
#define RPI_HOME_RAW_DEG100_J2  (13521)
#define RPI_HOME_RAW_DEG100_J3  (2783)

/*
 * DATA 前 4 轴表示「相对零位的角度」（度×100），软件限制在 ±RPI_REL_DEG_LIMIT（度），
 * 即 ±180° → ±18000；超出部分截断到边界。
 */
#define RPI_REL_DEG_LIMIT       180.0f

/* HOME 绝对角（rad）：与 RPI 零位相同，供 WorldCoord / Move_* 相对世界角 等使用 */
#define WORLD_HOME_ABS { \
    RPI_DEG100_TO_RAD(RPI_HOME_RAW_DEG100_J0), \
    RPI_DEG100_TO_RAD(RPI_HOME_RAW_DEG100_J1), \
    RPI_DEG100_TO_RAD(RPI_HOME_RAW_DEG100_J2), \
    RPI_DEG100_TO_RAD(RPI_HOME_RAW_DEG100_J3) \
}

/*
  关节是否允许连续旋转（1=可连续旋转，不做 wrap；0=不可整圈旋转，使用最短角 wrap）
  你已明确：底座(idx=0) 不允许整圈旋转 -> 必须为 0
*/
#define JOINT_CONTINUOUS_ROTATE {0, 0, 0, 0}

/*================ 到位判断 / 闭环修正参数 ================*/
/* 允许的到位误差（rad），用于闭环修正等 */
#define ARRIVAL_TOL_RAD          0.02f
/*
 * 插补「位移可忽略」阈值（rad）：小于此则不走 Move 循环、不 StreamEnter（TIM4 保持不断）。
 * 须略大于典型稳态跟踪误差 + 读反馈抖动；过小则同一 DATA 反复下发仍常跑插补 → 体感像失力。
 * 若仍偶发：可再试 0.15f（约 8.6°）；过大则「已到点附近」不再纠偏，精度变差。
 */
#define MOVE_DIST_TOL_RAD        0.09f
/* 运动结束后，最多做几次闭环修正（不改控制参数，只重复执行同轨迹/保持） */
#define ARRIVAL_MAX_CORRECTIONS  2

/*
 * 树莓派重力前馈：串口 TAU: 四轴力矩(Nm)，STM32 缓冲后由 TIM2 周期发 MIT。
 * Pi 模式为「力矩 + 阻尼」：Kp=0 关位置环；Kd 为速度阻尼；p 在固件里用反馈角（见 gravity_pi_feedforward.c），
 * 不使用 Current_Targets，避免上位轨迹/目标角干扰。
 */
#ifndef GRAVITY_FF_PI_MODE
#define GRAVITY_FF_PI_MODE       1
#endif
#ifndef GRAVITY_FF_GLOBAL_SCALE
#define GRAVITY_FF_GLOBAL_SCALE  1.0f
#endif
#define GRAVITY_FF_PI_MIT_KP { 0.0f, 0.0f, 0.0f, 0.0f }
#define GRAVITY_FF_PI_MIT_KD { 1.8f, 2.5f, 2.3f, 0.3f }

/*
 * TIM4 周期刚性保持（ISR 内 Apply_Rigid_Hold_OnBuffers_NoPostDelay）
 * GRAVITY_FF_PI_MODE=1 时强制为 0：MIT 仅由 TIM2 下发，避免与 Pi 前馈双通道。
 */
#define MOTOR_HOLD_TIM4_ENABLE   1
#if GRAVITY_FF_PI_MODE
#undef MOTOR_HOLD_TIM4_ENABLE
#define MOTOR_HOLD_TIM4_ENABLE   0
#endif

/*================ 调试日志开关 ================*/
/*
 * 1=打开：FB 回传 + 插补/Hold 周期内轮询 FB（避免主循环被 Move_* 阻塞时无角度回传）
 * 0=关闭：无 FB、无插补内轮询
 */
#define MOTOR_DEBUG_LOG_ENABLE   1

/* FB 行回传频率（Hz），由 TIM2 产生标志供主循环轮询；与 Pi 前馈同节拍 */
#define FB_REPORT_HZ             100

/*================ 额外流程参数 ================*/
#define MOTION_SPEED_SCALE       2.0f
#define RETURN_SETTLE_MS         80
#define FINAL_HOLD_MS            120
#define REG_READ_WAIT_MS         30
#define DRIVER_RANGE_SYNC_ENABLE 1

#endif /* MOTOR_CONFIG_H */
