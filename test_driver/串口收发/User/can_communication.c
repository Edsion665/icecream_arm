#include "can_communication.h"
#include "motor_control.h"
#include <string.h>

/*================ CAN硬件初始化 ================*/
void CAN_Hardware_Init(void)
{
    GPIO_InitTypeDef       GPIO_InitStruct;
    CAN_InitTypeDef        CAN_InitStruct;
    CAN_FilterInitTypeDef  FilterInitStruct;
    NVIC_InitTypeDef       NVIC_InitStruct;

    memset(&GPIO_InitStruct,  0, sizeof(GPIO_InitStruct));
    memset(&CAN_InitStruct,   0, sizeof(CAN_InitStruct));
    memset(&FilterInitStruct, 0, sizeof(FilterInitStruct));
    memset(&NVIC_InitStruct,  0, sizeof(NVIC_InitStruct));

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_SetBits(GPIOA, GPIO_Pin_8);

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    CAN_DeInit(CAN1);

    CAN_InitStruct.CAN_TTCM = DISABLE;
    CAN_InitStruct.CAN_ABOM = ENABLE;
    CAN_InitStruct.CAN_AWUM = DISABLE;
    CAN_InitStruct.CAN_NART = DISABLE;
    CAN_InitStruct.CAN_RFLM = DISABLE;
    CAN_InitStruct.CAN_TXFP = DISABLE;
    CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStruct.CAN_SJW  = CAN_SJW_1tq;
    CAN_InitStruct.CAN_BS1  = CAN_BS1_9tq;
    CAN_InitStruct.CAN_BS2  = CAN_BS2_8tq;
    CAN_InitStruct.CAN_Prescaler = 2;
    CAN_Init(CAN1, &CAN_InitStruct);

    FilterInitStruct.CAN_FilterNumber = 0;
    FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
    FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
    FilterInitStruct.CAN_FilterIdHigh = 0x0000;
    FilterInitStruct.CAN_FilterIdLow = 0x0000;
    FilterInitStruct.CAN_FilterMaskIdHigh = 0x0000;
    FilterInitStruct.CAN_FilterMaskIdLow = 0x0000;
    FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    FilterInitStruct.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&FilterInitStruct);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/*================ 基础工具函数 ================*/
uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / (float)((1 << bits) - 1) + x_min;
}

float u32_to_float(uint32_t u)
{
    union {
        uint32_t u32;
        float    f32;
    } cvt;
    cvt.u32 = u;
    return cvt.f32;
}

/*================ CAN 发送基础 ================*/
void CAN_Send_Blocking(CanTxMsg *tx)
{
    uint32_t wait_cnt = 50000;
    while (((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0U) && wait_cnt--) {
        ;
    }
    CAN_Transmit(CAN1, tx);
}

/*================ CAN 发送 ================*/
void Motor_MIT_Send_Raw(int idx, float p, float v, float kp, float kd, float t)
{
    uint8_t d[8];
    CanTxMsg tx;

    uint16_t p_i  = float_to_uint(p,  Runtime_P_Min[idx], Runtime_P_Max[idx], 16);
    uint16_t v_i  = float_to_uint(v,  Runtime_V_Min[idx], Runtime_V_Max[idx], 12);
    uint16_t kp_i = float_to_uint(kp, 0.0f, 500.0f, 12);
    uint16_t kd_i = float_to_uint(kd, 0.0f, 5.0f, 12);
    uint16_t t_i  = float_to_uint(t,  Runtime_T_Min[idx], Runtime_T_Max[idx], 12);

    d[0] = (uint8_t)(p_i >> 8);
    d[1] = (uint8_t)(p_i & 0xFF);
    d[2] = (uint8_t)(v_i >> 4);
    d[3] = (uint8_t)(((v_i & 0x0F) << 4) | (kp_i >> 8));
    d[4] = (uint8_t)(kp_i & 0xFF);
    d[5] = (uint8_t)(kd_i >> 4);
    d[6] = (uint8_t)(((kd_i & 0x0F) << 4) | (t_i >> 8));
    d[7] = (uint8_t)(t_i & 0xFF);

    tx.StdId = Motor_IDs[idx] & 0x7FF;
    tx.IDE   = CAN_Id_Standard;
    tx.RTR   = CAN_RTR_Data;
    tx.DLC   = 8;
    memcpy(tx.Data, d, 8);

    CAN_Send_Blocking(&tx);
    for (volatile uint32_t i = 0; i < 1000; i++); /* 短暂延时 */
}

void Motor_Send_Special(uint32_t id, uint8_t cmd)
{
    CanTxMsg tx;
    uint8_t d[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,cmd};

    tx.StdId = id & 0x7FF;
    tx.IDE   = CAN_Id_Standard;
    tx.RTR   = CAN_RTR_Data;
    tx.DLC   = 8;
    memcpy(tx.Data, d, 8);

    CAN_Send_Blocking(&tx);
    for (volatile uint32_t i = 0; i < 1000; i++); /* 短暂延时 */
}

void Motor_Read_Register_Request(int idx, uint8_t rid)
{
    CanTxMsg tx;

    tx.StdId = 0x7FF;
    tx.IDE   = CAN_Id_Standard;
    tx.RTR   = CAN_RTR_Data;
    tx.DLC   = 4;
    tx.Data[0] = (uint8_t)(Motor_IDs[idx] & 0xFF);
    tx.Data[1] = (uint8_t)((Motor_IDs[idx] >> 8) & 0xFF);
    tx.Data[2] = 0x33;
    tx.Data[3] = rid;

    CAN_Send_Blocking(&tx);
    for (volatile uint32_t i = 0; i < 1000; i++); /* 短暂延时 */
}

/*================ 反馈/寄存器读取 ================*/
void Request_Motor_Feedback(int idx)
{
    // 发送当前目标位置，而不是零位置，避免电机突然转动
    Motor_MIT_Send_Raw(idx, Current_Targets[idx], 0.0f, 0.0f, 0.0f, 0.0f);
}

uint8_t Wait_Register_Response(int idx, uint8_t rid, uint32_t wait_ms)
{
    uint32_t t;
    for (t = 0; t < wait_ms; t++) {
        if (Motor_RegResp[idx].got && (Motor_RegResp[idx].rid == rid)) {
            return 1;
        }
        for (volatile uint32_t i = 0; i < 8000; i++); /* 1ms 延时 */
    }
    return 0;
}

uint8_t Read_Register_Float(int idx, uint8_t rid, float *out)
{
    Motor_RegResp[idx].got = 0;
    Motor_Read_Register_Request(idx, rid);

    if (!Wait_Register_Response(idx, rid, 30)) return 0;

    *out = u32_to_float(Motor_RegResp[idx].data_u32);
    return 1;
}

void Read_All_Current_Positions(void)
{
    int retry, i;
    uint8_t all_ok;

    for (i = 0; i < MOTOR_NUM; i++) {
        Motor_Feedback_Received[i] = 0;
    }

    for (retry = 0; retry < 20; retry++) {
        all_ok = 1;
        for (i = 0; i < MOTOR_NUM; i++) {
            if (!Motor_Feedback_Received[i]) {
                /* 发送当前目标位置以获取反馈，避免拉动尚未初始化的电机 */
                Motor_MIT_Send_Raw(i, Current_Targets[i], 0.0f, 0.0f, 0.0f, 0.0f);
                all_ok = 0;
            }
        }
        if (all_ok) break;
        for (volatile uint32_t i = 0; i < 160000; i++); /* 20ms 延时，增加等待时间 */
    }
    
    /* 确保所有电机都有反馈，如果没有，至少发送零扭矩命令保持稳定 */
    if (!all_ok) {
        for (i = 0; i < MOTOR_NUM; i++) {
            Motor_MIT_Send_Raw(i, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
}

void Sync_MIT_Range_From_Driver(void)
{
    int i;
    float pmax, vmax, tmax;

    for (i = 0; i < MOTOR_NUM; i++) {
        if (Read_Register_Float(i, 0x0A, &pmax)) Runtime_P_Max[i] = pmax;
        if (Read_Register_Float(i, 0x0B, &vmax)) Runtime_V_Max[i] = vmax;
        if (Read_Register_Float(i, 0x0C, &tmax)) Runtime_T_Max[i] = tmax;
        Runtime_P_Min[i] = -Runtime_P_Max[i];
        Runtime_V_Min[i] = -Runtime_V_Max[i];
        Runtime_T_Min[i] = -Runtime_T_Max[i];
    }
}

/*================ CAN中断处理函数 ================*/
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

                /* ---------- 正常反馈：解析驱动器上报的 pos/vel/tor，供主循环与上报树莓派 ---------- */
                if (rx.DLC >= 6) {
                    uint8_t id_err = rx.Data[0];
                    uint8_t err = (uint8_t)(id_err >> 4);

                    uint16_t p_int = ((uint16_t)rx.Data[1] << 8) | rx.Data[2];   /* 位置 16bit */
                    uint16_t v_int = ((uint16_t)rx.Data[3] << 4) | (rx.Data[4] >> 4);
                    uint16_t t_int = (((uint16_t)rx.Data[4] & 0x0F) << 8) | rx.Data[5];

                    Motor_States[i].err = err;
                    Motor_States[i].pos = uint_to_float(p_int, Runtime_P_Min[i], Runtime_P_Max[i], 16);  /* 实际角度(rad) */
                    Motor_States[i].vel = uint_to_float(v_int, Runtime_V_Min[i], Runtime_V_Max[i], 12);
                    Motor_States[i].tor = uint_to_float(t_int, Runtime_T_Min[i], Runtime_T_Max[i], 12);

                    if (rx.DLC >= 8) {
                        Motor_States[i].mos_temp   = rx.Data[6];
                        Motor_States[i].rotor_temp = rx.Data[7];
                    }

                    Motor_Feedback_Received[i] = 1;

                    /* 只保留驱动真实故障保护，不再做软件软过扭矩停机 */
                    /* 暂时注释掉故障检测，避免误触发 */
                    // if (err != MOTOR_ERR_ENABLE && err != MOTOR_ERR_DISABLE) {
                    //     Latch_Fault((uint8_t)i, err);
                    // }
                    break;
                }
            }
        }

        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}
