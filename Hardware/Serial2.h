#ifndef __SERIAL2_H
#define __SERIAL2_H

#include <stdint.h>

/* USART2（PA2/PA3）：调试转发 TX，可按 USB 转串口能力再调高 */
#ifndef SERIAL_USART2_BAUD
#define SERIAL_USART2_BAUD  460800u
#endif

void Serial2_Init(void);
/* 非阻塞入队 + TXE 中断发送 */
void Serial2_SendByte(uint8_t Byte);
void Serial2_SendArray(uint8_t *Array, uint16_t Length);
void Serial2_SendString(char *String);

#endif

