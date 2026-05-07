#include "MotorControl/motor_config.h"
#include "MotorControl/motor_can.h"
#include "MotorControl/motor_control.h"
#include "../Hardware/Serial.h"
#include "../Hardware/Servo.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "fb_report_timer.h"
#include "MotorControl/motor_hold_timer.h"

/* 周期性 FB：实现见 fb_report_timer.c（主循环 + 插补内均调用 ServicePending） */

/* 1=PB0/PB1 PWM 测试（50Hz，1.5ms 中位），直接验证舵机引脚；0=正常舵机控制 */
#define PWM_PB01_TEST_ENABLE  0

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
    /* USART1：调试 FB 上行（波特率见 Serial.h SERIAL_USART1_BAUD） */
    Serial_Init();
#endif

#if MOTOR_DEBUG_LOG_ENABLE
    /* TIM2：按 FB_REPORT_HZ 置位，供主循环轮询 FB */
    FB_ReportTimer_Init();
#endif

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
        }
    }

    /* TIM4：按 INTERVAL_MS 周期 ISR 下发 MIT 保持（快照）；主线程阻塞时仍维持上一拍目标 */
    MotorHoldTimer_Init();
    MotorHoldTimer_PublishSnapshot();

    /* 舵机：PB0 腕部、PB1 机械爪，50Hz PWM（PWM.c 已移出工程，不再保留 PB01 独立测试分支） */
    Servo_Init();

    while (1) {
        /* USART1 RX 经 DMA 排空；MIT 由 TIM4 ISR 周期下发 */
        Serial_ServiceRxDma();

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
