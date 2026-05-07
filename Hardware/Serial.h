#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

/* USART1（PA9/PA10）：与树莓派/上位机波特率必须一致 */
#ifndef SERIAL_USART1_BAUD
#define SERIAL_USART1_BAUD  115200u
#endif

void Serial_Init(void);
/* 轮询 DMA 环形 RX 并入队文本行；由 main / TIM4 / USART1 TX 中断调用 */
void Serial_ServiceRxDma(void);
/* 发送：入队由 TXE 中断送出，不长时间占用 CPU（避免拖死电机保持周期） */
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

#endif
