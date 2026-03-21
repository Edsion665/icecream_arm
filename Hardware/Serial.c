#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "Serial.h"

/* 兼容旧符号（新逻辑用队列 + Serial_GetNextLine） */
volatile uint8_t g_rpi_line_ready = 0;
char g_rpi_line[RPI_LINE_MAX_LEN];

#define RPI_LINE_QUEUE_DEPTH  8
static char s_line_q[RPI_LINE_QUEUE_DEPTH][RPI_LINE_MAX_LEN];
static volatile uint8_t s_q_head = 0;
static volatile uint8_t s_q_tail = 0;
static volatile uint8_t s_q_count = 0;
static uint8_t s_rpi_line_idx = 0;
static char s_rx_build[RPI_LINE_MAX_LEN];

static void queue_push_line(const char *line)
{
    if (s_q_count >= RPI_LINE_QUEUE_DEPTH) {
        /* 队列满：丢弃最旧一行，腾出位置 */
        s_q_head = (uint8_t)((s_q_head + 1u) % RPI_LINE_QUEUE_DEPTH);
        s_q_count--;
    }
    strncpy(s_line_q[s_q_tail], line, RPI_LINE_MAX_LEN - 1u);
    s_line_q[s_q_tail][RPI_LINE_MAX_LEN - 1u] = '\0';
    s_q_tail = (uint8_t)((s_q_tail + 1u) % RPI_LINE_QUEUE_DEPTH);
    s_q_count++;

    /* 可选：镜像最后一行到 g_rpi_line，便于调试观察 */
    strncpy(g_rpi_line, line, RPI_LINE_MAX_LEN - 1u);
    g_rpi_line[RPI_LINE_MAX_LEN - 1u] = '\0';
    g_rpi_line_ready = 1;
}

uint8_t Serial_GetNextLine(char *out, uint16_t cap)
{
    if (out == 0 || cap < 2u) {
        return 0u;
    }
    if (s_q_count == 0u) {
        return 0u;
    }

    strncpy(out, s_line_q[s_q_head], (size_t)cap - 1u);
    out[cap - 1u] = '\0';
    s_q_head = (uint8_t)((s_q_head + 1u) % RPI_LINE_QUEUE_DEPTH);
    s_q_count--;

    if (s_q_count == 0u) {
        g_rpi_line_ready = 0;
    }
    return 1u;
}

void Serial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /*
     * NVIC 分组须在 Hardware_Init() 里已对全片配置；
     * 此处禁止再调 NVIC_PriorityGroupConfig，以免改写 CAN 优先级语义。
     * 抢占优先级数字越大优先级越低：USART1 用 3，低于 CAN_RX0 的 0。
     */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
        ;
    }
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++) {
        Serial_SendByte(Array[i]);
    }
}

void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++) {
        Serial_SendByte(String[i]);
    }
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

int fputc(int ch, FILE *f)
{
    (void)f;
    Serial_SendByte((uint8_t)ch);
    return ch;
}

void Serial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    Serial_SendString(String);
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
        uint8_t RxData = (uint8_t)USART_ReceiveData(USART1);

        if (RxData == '\r' || RxData == '\n') {
            if (s_rpi_line_idx > 0) {
                if (s_rpi_line_idx >= (RPI_LINE_MAX_LEN - 1)) {
                    s_rpi_line_idx = (RPI_LINE_MAX_LEN - 1);
                }
                s_rx_build[s_rpi_line_idx] = '\0';
                queue_push_line(s_rx_build);
                s_rpi_line_idx = 0;
            }
        } else {
            if (s_rpi_line_idx < (RPI_LINE_MAX_LEN - 1)) {
                s_rx_build[s_rpi_line_idx++] = (char)RxData;
            }
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
