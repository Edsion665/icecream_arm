#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

void Serial_Init(void);
/* 发送：入队由 TXE 中断送出，不长时间占用 CPU（避免拖死电机保持周期） */
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

/*================ 简易字符串命令接收（USART1） ================*/
#define RPI_LINE_MAX_LEN  96
extern volatile uint8_t g_rpi_line_ready;
extern char g_rpi_line[RPI_LINE_MAX_LEN];

/* 从中断入队的完整行中取出一行；无数据返回 0 */
uint8_t Serial_GetNextLine(char *out, uint16_t cap);

#endif
