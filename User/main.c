#include "MotorControl/motor_config.h"
#include "MotorControl/motor_types.h"
#include "MotorControl/motor_can.h"
#include "MotorControl/motor_control.h"
#include "MotorControl/motor_utils.h"
#include "Coordinate/world_coord.h"
#include "../Hardware/Serial.h"
#include "../Hardware/Serial2.h"
#include "../Hardware/Servo.h"
#include "../Hardware/PWM.h"
#include "serial_frame.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "fb_report_timer.h"
#include "motor_hold_timer.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* 周期性 FB：实现见 fb_report_timer.c（主循环 + 插补内均调用 ServicePending） */

/* 1=PB0/PB1 PWM 测试（50Hz，1.5ms 中位），直接验证舵机引脚；0=正常舵机控制 */
#define PWM_PB01_TEST_ENABLE  0

/* 与 motor_config.h 中 WORLD_HOME_ABS 同源（标定零位对应的绝对 rad） */
static const float s_rpi_home_abs_rad[4] = WORLD_HOME_ABS;

static float Rpi_ClampRelDeg(float deg)
{
    if (deg > RPI_REL_DEG_LIMIT) {
        return RPI_REL_DEG_LIMIT;
    }
    if (deg < -RPI_REL_DEG_LIMIT) {
        return -RPI_REL_DEG_LIMIT;
    }
    return deg;
}

/*================ 处理树莓派 6 个角度：前 4 为相对零位的度×100，±180° 内 -> 绝对 rad；后 2 为舵机 ================*/
static void Process_Rpi_Raw6(const int16_t raw[6])
{
    float target_abs[4];
    uint8_t i;
    char out[160];

    /* 后 2 路：舵机，-18000~18000（度×100）直接映射到 500~2500us */
    Servo_SetWrist(raw[4]);
    Servo_SetGripper(raw[5]);

    /* 前 4 路：相对标定零位的角（度×100），限幅 ±RPI_REL_DEG_LIMIT 后再换算绝对目标 */
    for (i = 0; i < 4; i++) {
        float rel_deg = ((float)raw[i]) / 100.0f;
        rel_deg       = Rpi_ClampRelDeg(rel_deg);
        float rel_rad = rel_deg * MOTOR_PI / 180.0f;
        target_abs[i] = s_rpi_home_abs_rad[i] + rel_rad;
    }

    snprintf(out, sizeof(out),
            "RES abs_rad: %.4f %.4f %.4f %.4f (rel deg*100->HOME+clamp, drv rad)\r\n",
            target_abs[0], target_abs[1], target_abs[2], target_abs[3]);
    Serial_SendString(out);
    Serial2_SendString(out);

    Read_All_Current_Positions();
    /* 插补起点与反馈一致（与 Move_Four_Motors_FromFeedback_To_Rels 开头相同） */
    for (i = 0; i < 4; i++) {
        Current_Targets[i] = Motor_States[i].pos;
    }

    /*
     * 已在 MOVE_DIST_TOL_RAD 内则不再跑四轴插补：避免重复同一 DATA 时
     * StreamEnter 长时间关掉 TIM4 保持（轨迹用 move_kp，体感像失力）。
     */
    {
        uint8_t already_there = 1u;
        for (i = 0; i < 4; i++) {
            if (fabsf(target_abs[i] - Motor_States[i].pos) >= MOVE_DIST_TOL_RAD) {
                already_there = 0u;
                break;
            }
        }
        if (already_there) {
            for (i = 0; i < 4; i++) {
                Current_Targets[i] = target_abs[i];
                Motor_Homed[i] = 1u;
            }
            MotorHoldTimer_PublishSnapshot();
            return;
        }
    }

    {
        uint8_t mark[4] = {1, 1, 1, 1};
        Move_Four_Motors_To_Targets(0, target_abs[0],
                                    1, target_abs[1],
                                    2, target_abs[2],
                                    3, target_abs[3],
                                    mark);
    }
}

/*================ CAN 中断 ================*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx;
    int i;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        CAN_Receive(CAN1, CAN_FIFO0, &rx);

        for (i = 0; i < MOTOR_NUM; i++) {
            if (rx.StdId == (Motor_Master_IDs[i] & 0x7FF)) {

                /* ---------- 寄存器响应 ---------- */
                if ((rx.DLC == 8) && ((rx.Data[2] == 0x33) || (rx.Data[2] == 0x55))) {
                    uint16_t canid = (uint16_t)rx.Data[0] | ((uint16_t)rx.Data[1] << 8);
                    if ((canid & 0x7FF) == (Motor_IDs[i] & 0x7FF)) {
                        Motor_RegResp[i].op = rx.Data[2];
                        Motor_RegResp[i].rid = rx.Data[3];
                        Motor_RegResp[i].data_u32 =
                            ((uint32_t)rx.Data[7] << 24) |
                            ((uint32_t)rx.Data[6] << 16) |
                            ((uint32_t)rx.Data[5] << 8)  |
                            ((uint32_t)rx.Data[4]);
                        Motor_RegResp[i].got = 1;
                        break;
                    }
                }

                /* ---------- 正常反馈 ---------- */
                if (rx.DLC >= 6) {
                    uint8_t id_err = rx.Data[0];
                    uint8_t err = (uint8_t)(id_err >> 4);

                    uint16_t p_int = ((uint16_t)rx.Data[1] << 8) | rx.Data[2];
                    uint16_t v_int = ((uint16_t)rx.Data[3] << 4) | (rx.Data[4] >> 4);
                    uint16_t t_int = (((uint16_t)rx.Data[4] & 0x0F) << 8) | rx.Data[5];

                    Motor_States[i].err = err;
                    Motor_States[i].pos = uint_to_float(p_int, Runtime_P_Min[i], Runtime_P_Max[i], 16);
                    Motor_States[i].vel = uint_to_float(v_int, Runtime_V_Min[i], Runtime_V_Max[i], 12);
                    Motor_States[i].tor = uint_to_float(t_int, Runtime_T_Min[i], Runtime_T_Max[i], 12);

                    if (rx.DLC >= 8) {
                        Motor_States[i].mos_temp   = rx.Data[6];
                        Motor_States[i].rotor_temp = rx.Data[7];
                    }

                    Motor_Feedback_Received[i] = 1;

                    /* 只保留驱动真实故障保护，不再做软件软过扭矩停机 */
                    if (Motor_Is_Fault(i)) {
                        Latch_Fault((uint8_t)i, err);
                    }
                    break;
                }
            }
        }

        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

/*================ 主函数 ================*/
int main(void)
{
    int i, m;

    SystemInit();
    Hardware_Init();
    Delay_ms(3000);

#if MOTOR_DEBUG_LOG_ENABLE
    /* 调试日志输出依赖串口初始化（USART1 波特率见 Serial.h SERIAL_USART1_BAUD） */
    Serial_Init();
#endif

    /* 串口2（USART2, PA2/PA3）：仅用于定时发送测试帧 */
    Serial2_Init();

#if MOTOR_DEBUG_LOG_ENABLE
    /* TIM2：按 FB_REPORT_HZ 置位；FB 发 USART1+2，须在两路串口均 Init 之后 */
    FB_ReportTimer_Init();
#endif

    /* 世界坐标(HOME)初始化 + 目标对齐（避免上电跳动） */
    WorldCoord_InitFixedHomeFromConfig();

    /* 使能 */
    for (i = 0; i < 25; i++) {
        for (m = 0; m < MOTOR_NUM; m++) {
            Motor_Send_Special(Motor_IDs[m], 0xFC);
        }
        Delay_ms(10);
    }

#if DRIVER_RANGE_SYNC_ENABLE
    /* 与驱动器内部 PMAX/VMAX/TMAX 保持一致 */
    Sync_MIT_Range_From_Driver();
#endif

    /*
     * 上电对齐关键点：
     * - 先使能驱动，再等待至少一次有效反馈
     * - 再将 Current_Targets 同步到反馈角，避免因“未收到反馈 pos=0”导致上电抽动
     */
    {
        uint32_t wait_ms;
        uint8_t motor_connected = 0;
        for (wait_ms = 0; wait_ms < 800; wait_ms += 20) {
            uint8_t all_ok = 1;
            Read_All_Current_Positions();
            for (m = 0; m < MOTOR_NUM; m++) {
                if (!Motor_Feedback_Received[m]) {
                    all_ok = 0;
                    break;
                }
            }
            if (all_ok) {
                motor_connected = 1;
                break;
            }
            Delay_ms(20);
        }
        Sync_CurrentTargets_From_Feedback();
        if (motor_connected) {
            Serial_SendString("motor connected!\r\n");
            Serial2_SendString("motor connected!\r\n");
        }
    }

    /* TIM4：按 INTERVAL_MS 周期 ISR 下发 MIT 保持（快照）；主线程阻塞时仍维持上一拍目标 */
    MotorHoldTimer_Init();
    MotorHoldTimer_PublishSnapshot();

#if PWM_PB01_TEST_ENABLE
    /* PB0/PB1 PWM 测试：50Hz，1.5ms 脉宽（舵机中位） */
    PWM_Init();
#else
    /* 舵机：PB0 腕部、PB1 机械爪，50Hz PWM */
    Servo_Init();
#endif

    /* 串口1（USART1）：按行字符串控制；串口2（USART2）：转发串口1 收发 */

    /* 串口1 解析状态：收到 start 后永久开启电机角度解析 */
    static uint8_t s_parsing_enabled = 0;

    while (1) {
        /* USART1 RX 经 DMA；此处拉数据降低行解析延迟（MIT 由 TIM4 快照发） */
        Serial_ServiceRxDma();

        /* 串口1：每轮主循环最多处理 1 行 */
        {
            char line[RPI_LINE_MAX_LEN];
            if (Serial_GetNextLine(line, sizeof(line))) {
                char tmp[RPI_LINE_MAX_LEN];
                uint8_t i;

                /* 串口2：转发串口1 接收到的内容 */
                Serial2_SendString(line);
                Serial2_SendString("\r\n");

                /* 转小写便于比较 */
                for (i = 0; line[i] != '\0'; i++) {
                    tmp[i] = (line[i] >= 'A' && line[i] <= 'Z') ?
                             (char)(line[i] - 'A' + 'a') : line[i];
                }
                tmp[i] = '\0';

                if (strcmp(tmp, "start") == 0) {
                    /* 1) 先回复 OK（串口1 + 串口2） */
                    Serial_SendString("OK\r\n");
                    Serial2_SendString("OK\r\n");
                    /* 2) 开启后续电机角度解析，且不再关闭 */
                    s_parsing_enabled = 1;
                    /* 勿用 continue：会跳过本周期 Apply_Rigid_Hold（原 while 里 continue 只取下一行） */
                } else if (s_parsing_enabled) {
                    /* 帧头 DATA: + 6 整数 + 校验 *XX */
                    int16_t raw6[6];
                    if (SerialFrame_ParseData(line, raw6)) {
                        Process_Rpi_Raw6(raw6);
                    }
                }
            }
        }

#if MOTOR_DEBUG_LOG_ENABLE
        FB_Report_ServicePending();
#endif

        if (Emergency_Stop) {
            Disable_All_Motors();
            Delay_ms(10);
            continue;
        }

        /* 刚性保持由 TIM4 ISR 按快照周期下发；此处刷新快照使 Current_Targets 与 homed 及时同步 */
        MotorHoldTimer_PublishSnapshot();

#if !PWM_PB01_TEST_ENABLE
        Servo_Update();
#endif

        Delay_ms(INTERVAL_MS);
    }
}
