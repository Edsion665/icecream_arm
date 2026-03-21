#include "stm32f10x.h"
#include "Serial2.h"

/* USART2 仅调试 TX：环形缓冲 + TXE，避免忙等打断电机保持周期 */
#define SERIAL2_TX_BUF_SIZE  1024u
static uint8_t s_tx2[SERIAL2_TX_BUF_SIZE];
static volatile uint16_t s_tx2_in  = 0;
static volatile uint16_t s_tx2_out = 0;

void Serial2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = SERIAL_USART2_BAUD;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    s_tx2_in = 0;
    s_tx2_out = 0;

    NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

void Serial2_SendByte(uint8_t Byte)
{
    for (;;) {
        uint16_t in;
        uint16_t next;

        NVIC_DisableIRQ(USART2_IRQn);
        in   = s_tx2_in;
        next = (uint16_t)((in + 1u) % SERIAL2_TX_BUF_SIZE);
        if (next != s_tx2_out) {
            s_tx2[in] = Byte;
            s_tx2_in  = next;
            USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
            NVIC_EnableIRQ(USART2_IRQn);
            return;
        }
        NVIC_EnableIRQ(USART2_IRQn);
    }
}

void Serial2_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i++) {
        Serial2_SendByte(Array[i]);
    }
}

void Serial2_SendString(char *String)
{
    uint16_t i = 0;

    if (String == 0) {
        return;
    }

    while (String[i] != '\0') {
        NVIC_DisableIRQ(USART2_IRQn);
        while (String[i] != '\0') {
            uint16_t in   = s_tx2_in;
            uint16_t next = (uint16_t)((in + 1u) % SERIAL2_TX_BUF_SIZE);
            if (next == s_tx2_out) {
                break;
            }
            s_tx2[in] = String[i++];
            s_tx2_in  = next;
        }
        USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
        NVIC_EnableIRQ(USART2_IRQn);

        if (String[i] == '\0') {
            return;
        }
    }
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
        if (s_tx2_out != s_tx2_in) {
            USART_SendData(USART2, s_tx2[s_tx2_out]);
            s_tx2_out = (uint16_t)((s_tx2_out + 1u) % SERIAL2_TX_BUF_SIZE);
        }
        if (s_tx2_out == s_tx2_in) {
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
    }
}
