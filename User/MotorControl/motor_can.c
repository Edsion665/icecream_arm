#include "motor_can.h"
#include "motor_utils.h"
#include <string.h>

/* 定义在 motor_control.c；Read_All 内同步目标用 */
extern float Current_Targets[MOTOR_NUM];

/*================ 全局变量定义 ================*/
uint32_t Motor_IDs[MOTOR_NUM] = MOTOR_IDS;
uint32_t Motor_Master_IDs[MOTOR_NUM] = MOTOR_MASTER_IDS;
float Runtime_P_Min[MOTOR_NUM] = {MIT_P_MIN, MIT_P_MIN, MIT_P_MIN, MIT_P_MIN};
float Runtime_P_Max[MOTOR_NUM] = {MIT_P_MAX, MIT_P_MAX, MIT_P_MAX, MIT_P_MAX};
float Runtime_V_Min[MOTOR_NUM] = {MIT_V_MIN, MIT_V_MIN, MIT_V_MIN, MIT_V_MIN};
float Runtime_V_Max[MOTOR_NUM] = {MIT_V_MAX, MIT_V_MAX, MIT_V_MAX, MIT_V_MAX};
float Runtime_T_Min[MOTOR_NUM] = {MIT_T_MIN, MIT_T_MIN, MIT_T_MIN, MIT_T_MIN};
float Runtime_T_Max[MOTOR_NUM] = {MIT_T_MAX, MIT_T_MAX, MIT_T_MAX, MIT_T_MAX};
volatile Motor_Status_t Motor_States[MOTOR_NUM];
volatile uint8_t Motor_Feedback_Received[MOTOR_NUM];
volatile Motor_RegResp_t Motor_RegResp[MOTOR_NUM];
volatile uint8_t Emergency_Stop = 0;
volatile uint8_t System_Disabled = 0;
volatile uint8_t Fault_Motor = 0xFF;
volatile uint8_t Fault_Code  = MOTOR_ERR_DISABLE;

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
    Delay_us(200);
}

/* 定时器 ISR 等高频路径：省略帧间延时，避免拉长中断时间 */
void Motor_MIT_Send_Raw_NoPostDelay(int idx, float p, float v, float kp, float kd, float t)
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
    Delay_us(200);
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
    Delay_us(200);
}

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

/*================ 反馈/寄存器读取 ================*/
/* Request_Motor_Feedback：实现见 motor_control.c（须带刚度发 MIT，不能发全零） */

/*
 * 仅轮询 Motor_RegResp + Delay_ms，不向电机发任何 CAN/MIT；
 * 寄存器请求由 Motor_Read_Register_Request 发出（非全零 MIT 包）。
 */
uint8_t Wait_Register_Response(int idx, uint8_t rid, uint32_t wait_ms)
{
    uint32_t t;
    for (t = 0; t < wait_ms; t++) {
        if (Motor_RegResp[idx].got && (Motor_RegResp[idx].rid == rid)) {
            return 1;
        }
        Delay_ms(1);
    }
    return 0;
}

uint8_t Read_Register_Float(int idx, uint8_t rid, float *out)
{
    Motor_RegResp[idx].got = 0;
    Motor_Read_Register_Request(idx, rid);

    if (!Wait_Register_Response(idx, rid, REG_READ_WAIT_MS)) return 0;

    *out = u32_to_float(Motor_RegResp[idx].data_u32);
    return 1;
}

void Sync_MIT_Range_From_Driver(void)
{
    int i;
    float pmax, vmax, tmax;

    for (i = 0; i < MOTOR_NUM; i++) {
        if (Read_Register_Float(i, 0x15, &pmax)) {
            if (pmax > 0.01f) {
                Runtime_P_Max[i] = pmax;
                Runtime_P_Min[i] = -pmax;
            }
        }
        if (Read_Register_Float(i, 0x16, &vmax)) {
            if (vmax > 0.01f) {
                Runtime_V_Max[i] = vmax;
                Runtime_V_Min[i] = -vmax;
            }
        }
        if (Read_Register_Float(i, 0x17, &tmax)) {
            if (tmax > 0.01f) {
                Runtime_T_Max[i] = tmax;
                Runtime_T_Min[i] = -tmax;
            }
        }
    }
}

void Read_All_Current_Positions(void)
{
    int retry, i;
    uint8_t all_ok;

    /*
     * 清「本轮收到」标志前：用已有效的反馈把 Current_Targets 对齐真实角。
     * 否则紧接着 Request 会用旧目标（常见为 0）发带刚度的 MIT → 上电/读反馈瞬间猛拉。
     */
    for (i = 0; i < MOTOR_NUM; i++) {
        if (Motor_Feedback_Received[i]) {
            Current_Targets[i] = Motor_States[i].pos;
        }
    }

    for (i = 0; i < MOTOR_NUM; i++) {
        Motor_Feedback_Received[i] = 0;
    }

    for (retry = 0; retry < 10; retry++) {
        /* 等待间隙里若某轴已回包，先把目标对齐该轴，再向仍未回包轴发 MIT */
        for (i = 0; i < MOTOR_NUM; i++) {
            if (Motor_Feedback_Received[i]) {
                Current_Targets[i] = Motor_States[i].pos;
            }
        }

        all_ok = 1;
        for (i = 0; i < MOTOR_NUM; i++) {
            if (!Motor_Feedback_Received[i]) {
                Request_Motor_Feedback(i);
                all_ok = 0;
            }
        }
        if (all_ok) break;
        Delay_ms(10);
    }
}

/*================ 故障判断 ================*/
uint8_t Motor_Is_Fault(int idx)
{
    uint8_t e = Motor_States[idx].err;
    if (e == MOTOR_ERR_ENABLE || e == MOTOR_ERR_DISABLE) return 0;
    return 1;
}

/*================ 硬件初始化 ================*/
void Hardware_Init(void)
{
    GPIO_InitTypeDef       GPIO_InitStruct;
    CAN_InitTypeDef        CAN_InitStruct;
    CAN_FilterInitTypeDef  FilterInitStruct;
    NVIC_InitTypeDef       NVIC_InitStruct;

    memset(&GPIO_InitStruct,  0, sizeof(GPIO_InitStruct));
    memset(&CAN_InitStruct,   0, sizeof(CAN_InitStruct));
    memset(&FilterInitStruct, 0, sizeof(FilterInitStruct));
    memset(&NVIC_InitStruct,  0, sizeof(NVIC_InitStruct));

    /* 全片统一用 Group2；须在任何 IRQ 优先级配置之前调用一次 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

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
    
    /* Set SysTick interrupt priority to lowest */
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
}
