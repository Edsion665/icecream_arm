/**
  ******************************************************************************
  * @file    main.c
  * @brief   四轴机械臂控制：CAN 电机 + 树莓派串口指令
  *          - 树莓派下发 6 字节（4 电机相对 Home ±180° + 2 舵机），STM32 解析并运动
  *          - 运动结束后将 CAN 读取的电机实际角度按相同格式回传树莓派
  *          - 上电初始位置为 CAN 反馈的当前角度；Motor_Home 仅作坐标参考
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "motor_control.h"
#include "can_communication.h"
#include "Serial.h"
#include "Delay.h"
#include <string.h>
#include <math.h>

/*================ 用户配置参数区 (保留原参数，不删除不改) ================*/
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
// 这些参数在motor_control.h中定义
// #define MIT_P_MIN       (-12.5f)
// #define MIT_P_MAX       ( 12.5f)
// #define MIT_V_MIN       (-2.0f)   /* 降低速度限制 */
// #define MIT_V_MAX       ( 2.0f)   /* 降低速度限制 */
// #define MIT_T_MIN       (-18.0f)
// #define MIT_T_MAX       ( 18.0f)

/* 极限测试目标位置 */
#define MOTOR4_PRESET_POS   1.70f
#define MOTOR3_TEST_POS     2.00f
#define MOTOR2_TEST_POS     2.40f

/* 额外流程参数 */
#define MOTION_SPEED_SCALE       0.5f  /* 降低速度缩放因子，使运动更慢 */
#define RETURN_SETTLE_MS         80
#define FINAL_HOLD_MS            120
#define REG_READ_WAIT_MS         30
#define DRIVER_RANGE_SYNC_ENABLE 1

#define PI 3.1415926535f


/*================ 主函数 ================*/
int main(void)
{
    int i, m;

    SystemInit();
    CAN_Hardware_Init();   /* CAN、GPIO 等 */
    Serial_Init();     /* 串口 PA9(TX)/PA10(RX)，与树莓派通信 */
    Delay_ms(3000);

    for (i = 0; i < MOTOR_NUM; i++) {
        Motor_RegResp[i].got = 0;
    }

    /* 使能所有电机（0xFC 进入控制模式） */
    for (i = 0; i < 10; i++) {
        for (m = 0; m < MOTOR_NUM; m++) {
            Motor_Send_Special(Motor_IDs[m], 0xFC);
        }
        Delay_ms(20);
    }

    /* 使能后立即发送零扭矩命令，确保电机保持稳定 */
    for (i = 0; i < 3; i++) {
        for (m = 0; m < MOTOR_NUM; m++) {
            Motor_MIT_Send_Raw(m, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
        Delay_ms(50);
    }

#if DRIVER_RANGE_SYNC_ENABLE
    Sync_MIT_Range_From_Driver();   /* 与驱动器内部 PMAX/VMAX/TMAX 同步 */
#endif

    /* 初始化USART2用于上位机监控 */
    Serial2_Init();

    /* 初始化Current_Targets为安全值，避免未初始化导致的问题 */
    for (i = 0; i < MOTOR_NUM; i++) {
        Current_Targets[i] = 0.0f;
        // 初始化Motor_States
        Motor_States[i].pos = 0.0f;
        Motor_States[i].vel = 0.0f;
        Motor_States[i].tor = 0.0f;
        Motor_States[i].err = 0;
        Motor_States[i].mos_temp = 0;
        Motor_States[i].rotor_temp = 0;
        Motor_Feedback_Received[i] = 0;
        Motor_Homed[i] = 0;
    }
    
    /* 先读取当前实际位置 */
    Read_All_Current_Positions();
    
    /* 初始化Current_Targets为当前实际位置 */
    for (i = 0; i < MOTOR_NUM; i++) {
        Current_Targets[i] = Motor_States[i].pos;
    }
    
    /* 发送当前位置给上位机监控 */
    Serial2_SendString("[STM32 Monitor Data]\r\n");
    Serial2_SendString("Current Motor Positions:\r\n");
    char buffer[50];
    for (i = 0; i < MOTOR_NUM; i++) {
        sprintf(buffer, "M%d: %.3f rad\r\n", i+1, Motor_States[i].pos);
        Serial2_SendString(buffer);
    }
    Serial2_SendString("Moving to Initial Positions...\r\n");
    Serial2_SendString("[End of Data]\r\n\r\n");
    

    
    /* 使用运动规划平滑移动到初始位置（速度已经设置很慢） */
    // Move_All_Motors_To_Targets(3.134f, -0.178f, -0.770f, 0.791f);
    
    /* 运动完成后进入刚性保持 */
    Hold_All_Rigid(500); /* 给予足够的保持时间 */
    for (i = 0; i < MOTOR_NUM; i++) {
        Motor_Homed[i] = 1;   /* 视为已就位，后续用刚性保持 */
    }
    
    /* 发送初始位置给上位机监控 */
    Serial2_SendString("[STM32 Monitor Data]\r\n");
    Serial2_SendString("Initial Motor Positions Set:\r\n");
    Serial2_SendString("M1: 3.134 rad\r\n");
    Serial2_SendString("M2: -0.178 rad\r\n");
    Serial2_SendString("M3: -0.770 rad\r\n");
    Serial2_SendString("M4: 0.791 rad\r\n");
    Serial2_SendString("[End of Data]\r\n\r\n");

    /* 树莓派串口控制主循环：有数据则解析并运动+回传，无数据则刚性保持当前目标 */
    // while (1) {
    //     if (Emergency_Stop) {
    //         // 只在真正的紧急情况下禁用电机
    //         // 这里可以添加故障原因的上报
    //         Disable_All_Motors();
    //         Delay_ms(10);
    //         continue;
    //     }

    //     if (Serial_GetRxFlag()) {
    //         Parse_Serial_And_Move_Motors();   /* 解析 -> 运动 -> 50ms 保持 -> 回传实际角度 */
    //     } else {
    //         Apply_Rigid_Hold_One_Cycle();     /* 保持当前目标位置 */
    //     }
    //     Delay_ms(INTERVAL_MS);
    // }
}
