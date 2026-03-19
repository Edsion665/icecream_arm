#include "MotorControl/motor_config.h"
#include "MotorControl/motor_types.h"
#include "MotorControl/motor_can.h"
#include "MotorControl/motor_control.h"
#include "MotorControl/motor_utils.h"
#include "Coordinate/world_coord.h"
#include "../Hardware/Serial.h"
#include "../Hardware/Serial2.h"
#include <stdio.h>
#include <string.h>

/*================ 处理树莓派 6 个角度：计算->四电机同步运动->回传 ================*/
static void Process_Rpi_Raw6(const int16_t raw[6])
{
    float world_rel_rad[4];
    float delta_deg100[4] = {0};
    float cur_deg100[4]   = {0};
    uint8_t i;

    /* 前 4 个电机角：相对世界坐标(HOME)的目标角（度*100 -> rad） */
    for (i = 0; i < 4; i++) {
        float deg = ((float)raw[i]) / 100.0f;
        world_rel_rad[i] = deg * 3.1415926535f / 180.0f;
    }

    /* 3) 使用当前反馈计算每轴“理论将要转动的增量”（仅供上位机监控显示） */
    Read_All_Current_Positions();
    for (i = 0; i < 4; i++) {
        float current_abs = Motor_States[i].pos;
        float delta_abs = 0.0f;
        float target_abs = 0.0f;
        WorldCoord_Status_t st = WorldCoord_DeltaToTarget(i,
                                                          current_abs,
                                                          world_rel_rad[i],
                                                          &delta_abs,
                                                          &target_abs);
        if (st == WORLD_OK) {
            float deg = delta_abs * 180.0f / 3.1415926535f;
            delta_deg100[i] = (float)((int16_t)(deg * 100.0f));
        }
    }

    /* 4) 统一四轴同步运动：从反馈起步 -> 相对 HOME 目标 -> 限位/最短路径 -> 余弦轨迹 */
    {
        uint8_t mark[4] = {1, 1, 1, 1};
        Move_Four_Motors_FromFeedback_To_Rels(0, world_rel_rad[0],
                                              1, world_rel_rad[1],
                                              2, world_rel_rad[2],
                                              3, world_rel_rad[3],
                                              mark);
    }

    /* 5) 串口1回传反馈（字符串）：FB a b c d（单位：度*100） */
    {
        int16_t fb_raw[4];
        char out[96];

        Read_All_Current_Positions();
        for (i = 0; i < 4; i++) {
            float abs_pos = Motor_States[i].pos;
            float rel_rad = 0.0f;
            WorldCoord_Status_t st = WorldCoord_RelFromAbs(i, abs_pos, &rel_rad);
            if (st != WORLD_OK) rel_rad = 0.0f;
            fb_raw[i] = (int16_t)(rel_rad * 180.0f / 3.1415926535f * 100.0f);
            cur_deg100[i] = (float)fb_raw[i];
        }

        snprintf(out, sizeof(out), "FB %d %d %d %d\r\n",
                 (int)fb_raw[0], (int)fb_raw[1], (int)fb_raw[2], (int)fb_raw[3]);
        Serial_SendString(out);
    }

    /* 6) 通过串口2将“树莓派命令 + 计算增量 + 当前角度”发给上位机（0x10 包） */
    {
        uint8_t buf2[2 + 1 + (6 + 4 + 4) * 2 + 1 + 2];
        uint8_t idx2 = 0;
        uint8_t chk2 = 0;
        uint8_t j;

        buf2[idx2++] = 0x55;
        buf2[idx2++] = 0xAA;
        buf2[idx2++] = 0x10;

        /* 6 个命令角：raw[]（度*100） */
        for (j = 0; j < 6; j++) {
            int16_t v = raw[j];
            buf2[idx2++] = (uint8_t)((v >> 8) & 0xFF);
            buf2[idx2++] = (uint8_t)(v & 0xFF);
        }

        /* 4 个 delta（度*100） */
        for (j = 0; j < 4; j++) {
            int16_t v = (int16_t)delta_deg100[j];
            buf2[idx2++] = (uint8_t)((v >> 8) & 0xFF);
            buf2[idx2++] = (uint8_t)(v & 0xFF);
        }

        /* 4 个当前角（相对世界坐标，度*100） */
        for (j = 0; j < 4; j++) {
            int16_t v = (int16_t)cur_deg100[j];
            buf2[idx2++] = (uint8_t)((v >> 8) & 0xFF);
            buf2[idx2++] = (uint8_t)(v & 0xFF);
        }

        /* 校验：从 CMD(0x10) 开始到数据区最后一个字节 */
        for (j = 2; j < idx2; j++) {
            chk2 += buf2[j];
        }
        buf2[idx2++] = chk2;
        buf2[idx2++] = 0x0D;
        buf2[idx2++] = 0x0A;

        Serial2_SendArray(buf2, idx2);
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
    /* 调试日志输出依赖串口初始化（默认 USART1@9600） */
    Serial_Init();
#endif

    /* 上位机监控串口（USART2, PA2/PA3），用于发送树莓派指令及电机角度等调试信息 */
    Serial2_Init();

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
        for (wait_ms = 0; wait_ms < 800; wait_ms += 20) {
            uint8_t all_ok = 1;
            Read_All_Current_Positions();
            for (m = 0; m < MOTOR_NUM; m++) {
                if (!Motor_Feedback_Received[m]) {
                    all_ok = 0;
                    break;
                }
            }
            if (all_ok) break;
            Delay_ms(20);
        }
    }
    Sync_CurrentTargets_From_Feedback();
    /* 串口1（USART1）：按行字符串控制；串口2（USART2）：调试输出 */

    while (1) {
        /* 简易字符串命令：一行 6 个角度（度*100），以空格/逗号分隔均可 */
        if (g_rpi_line_ready) {
            char line[RPI_LINE_MAX_LEN];
            int a0, a1, a2, a3, a4, a5;
            g_rpi_line_ready = 0;

            /* 拷贝到本地缓冲，避免边收边改 */
            snprintf(line, sizeof(line), "%s", g_rpi_line);

            /* 先把收到的原始字符串发到串口2，方便上位机观察 */
            Serial2_SendString("[RX] ");
            Serial2_SendString(line);
            Serial2_SendString("\r\n");

            /* 握手：整行 start（忽略大小写）就回 OK */
            {
                char tmp[RPI_LINE_MAX_LEN];
                uint8_t i;
                snprintf(tmp, sizeof(tmp), "%s", line);
                for (i = 0; tmp[i] != '\0'; i++) {
                    if (tmp[i] >= 'A' && tmp[i] <= 'Z') {
                        tmp[i] = (char)(tmp[i] - 'A' + 'a');
                    }
                }
                if (strcmp(tmp, "start") == 0) {
                    Serial_SendString("OK\r\n");
                    goto after_line;
                }
            }

            {
                /* 解析 6 个整数：支持空格或逗号分隔 */
                if (sscanf(line, "%d%*[, ]%d%*[, ]%d%*[, ]%d%*[, ]%d%*[, ]%d",
                           &a0, &a1, &a2, &a3, &a4, &a5) == 6) {
                    int16_t raw6[6];
                    raw6[0] = (int16_t)a0;
                    raw6[1] = (int16_t)a1;
                    raw6[2] = (int16_t)a2;
                    raw6[3] = (int16_t)a3;
                    raw6[4] = (int16_t)a4;
                    raw6[5] = (int16_t)a5;
                    Process_Rpi_Raw6(raw6);
                } else {
                    /* 字符串格式不对就忽略（你说不需要协议，这里不做复杂错误处理） */
                    Serial2_SendString("[RX] parse fail\r\n");
                }
            }

after_line:
            ;
        }

        if (Emergency_Stop) {
            Disable_All_Motors();
            Delay_ms(10);
            continue;
        }

        /* 全程只做硬性保持，不再做柔性保持 */
        Apply_Rigid_Hold_One_Cycle();
        Delay_ms(INTERVAL_MS);
    }
}
