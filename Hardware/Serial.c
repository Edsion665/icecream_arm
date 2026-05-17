#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "Serial.h"

/*================ USART1 发送：环形缓冲 + TXE 中断 ================*/
#define SERIAL1_TX_BUF_SIZE  1024u
static uint8_t s_tx1[SERIAL1_TX_BUF_SIZE];
static volatile uint16_t s_tx1_in  = 0;
static volatile uint16_t s_tx1_out = 0;

/*================ USART1 接收：DMA 循环缓冲 ================*/
#define USART1_DMA_RX_BUF_SIZE  512u
static uint8_t s_usart1_dma_rx[USART1_DMA_RX_BUF_SIZE];
static volatile uint16_t s_dma_rx_proc_idx;
static uint8_t s_serial_dma_inited;

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
        s_q_head = (uint8_t)((s_q_head + 1u) % RPI_LINE_QUEUE_DEPTH);
        s_q_count--;
    }
    strncpy(s_line_q[s_q_tail], line, RPI_LINE_MAX_LEN - 1u);
    s_line_q[s_q_tail][RPI_LINE_MAX_LEN - 1u] = '\0';
    s_q_tail = (uint8_t)((s_q_tail + 1u) % RPI_LINE_QUEUE_DEPTH);
    s_q_count++;

    strncpy(g_rpi_line, line, RPI_LINE_MAX_LEN - 1u);
    g_rpi_line[RPI_LINE_MAX_LEN - 1u] = '\0';
    g_rpi_line_ready = 1;
}

/*================ 二进制下行帧接收状态机 ================*/
#define BIN_FRAME_LEN  RPI_BIN_FRAME_LEN
#define BIN_QUEUE_DEPTH 4u

static uint8_t  s_bin_buf[BIN_FRAME_LEN];
static uint8_t  s_bin_idx = 0;

static uint8_t  s_bin_q[BIN_QUEUE_DEPTH][BIN_FRAME_LEN];
static volatile uint8_t s_bin_q_head  = 0;
static volatile uint8_t s_bin_q_tail  = 0;
static volatile uint8_t s_bin_q_count = 0;

static void bin_queue_push(const uint8_t *frame)
{
    uint8_t i;
    if (s_bin_q_count >= BIN_QUEUE_DEPTH) {
        s_bin_q_head = (uint8_t)((s_bin_q_head + 1u) % BIN_QUEUE_DEPTH);
        s_bin_q_count--;
    }
    for (i = 0; i < BIN_FRAME_LEN; i++) {
        s_bin_q[s_bin_q_tail][i] = frame[i];
    }
    s_bin_q_tail = (uint8_t)((s_bin_q_tail + 1u) % BIN_QUEUE_DEPTH);
    s_bin_q_count++;
}

uint8_t Serial_GetNextBinFrame(RpiBinFrame_t *out)
{
    uint8_t i;
    if (out == 0 || s_bin_q_count == 0u) {
        return 0u;
    }
    for (i = 0; i < BIN_FRAME_LEN; i++) {
        out->data[i] = s_bin_q[s_bin_q_head][i];
    }
    s_bin_q_head = (uint8_t)((s_bin_q_head + 1u) % BIN_QUEUE_DEPTH);
    s_bin_q_count--;
    return 1u;
}

static void process_rx_byte(uint8_t RxData)
{
    if (s_bin_idx == 0) {
        if (RxData == 0xAAu) {
            s_bin_buf[s_bin_idx++] = RxData;
        } else {
            goto text_path;
        }
        return;
    }
    if (s_bin_idx == 1) {
        if (RxData == 0x55u) {
            s_bin_buf[s_bin_idx++] = RxData;
        } else {
            s_bin_idx = 0;
            goto text_path;
        }
        return;
    }

    s_bin_buf[s_bin_idx++] = RxData;
    if (s_bin_idx >= BIN_FRAME_LEN) {
        uint8_t xc = 0;
        uint8_t i;
        for (i = 0; i < BIN_FRAME_LEN - 1u; i++) {
            xc ^= s_bin_buf[i];
        }
        if (xc == s_bin_buf[BIN_FRAME_LEN - 1u]) {
            bin_queue_push(s_bin_buf);
        }
        s_bin_idx = 0;
    }
    return;

text_path:
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
}

void Serial_ServiceRxDma(void)
{
    uint16_t w;
    uint16_t proc;

    if (!s_serial_dma_inited) {
        return;
    }

    w = (uint16_t)(USART1_DMA_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5));
    proc = s_dma_rx_proc_idx;

    while (proc != w) {
        process_rx_byte(s_usart1_dma_rx[proc]);
        proc++;
        if (proc >= USART1_DMA_RX_BUF_SIZE) {
            proc = 0;
        }
        s_dma_rx_proc_idx = proc;
    }
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
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
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

    USART_InitStructure.USART_BaudRate            = SERIAL_USART1_BAUD;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)s_usart1_dma_rx;
    DMA_InitStructure.DMA_DIR                  = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize           = USART1_DMA_RX_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc        = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc            = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize   = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize       = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode                 = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority             = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M                  = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel5, ENABLE);

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

    s_tx1_in = 0;
    s_tx1_out = 0;
    s_dma_rx_proc_idx = 0;
    s_bin_idx = 0;

    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);

    s_serial_dma_inited = 1u;
}

void Serial_SendByte(uint8_t Byte)
{
    for (;;) {
        uint16_t in;
        uint16_t next;

        NVIC_DisableIRQ(USART1_IRQn);
        in   = s_tx1_in;
        next = (uint16_t)((in + 1u) % SERIAL1_TX_BUF_SIZE);
        if (next != s_tx1_out) {
            s_tx1[in] = Byte;
            s_tx1_in  = next;
            USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
            NVIC_EnableIRQ(USART1_IRQn);
            return;
        }
        NVIC_EnableIRQ(USART1_IRQn);
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
    uint8_t i = 0;

    if (String == 0) {
        return;
    }

    while (String[i] != '\0') {
        NVIC_DisableIRQ(USART1_IRQn);
        while (String[i] != '\0') {
            uint16_t in   = s_tx1_in;
            uint16_t next = (uint16_t)((in + 1u) % SERIAL1_TX_BUF_SIZE);
            if (next == s_tx1_out) {
                break;
            }
            s_tx1[in] = String[i++];
            s_tx1_in  = next;
        }
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
        NVIC_EnableIRQ(USART1_IRQn);

        if (String[i] == '\0') {
            return;
        }
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
    Serial_ServiceRxDma();

    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET) {
        volatile uint32_t sr = USART1->SR;
        volatile uint32_t dr = USART1->DR;
        (void)sr;
        (void)dr;
        s_rpi_line_idx = 0;
        s_bin_idx = 0;
    }

    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
        if (s_tx1_out != s_tx1_in) {
            USART_SendData(USART1, s_tx1[s_tx1_out]);
            s_tx1_out = (uint16_t)((s_tx1_out + 1u) % SERIAL1_TX_BUF_SIZE);
        }
        if (s_tx1_out == s_tx1_in) {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
    }
}
