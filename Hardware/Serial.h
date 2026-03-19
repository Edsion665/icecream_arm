#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

/*================ 简易字符串命令接收（USART1） ================*/
/* 接收一行 ASCII（以 \\r 或 \\n 结束），由中断填充；主循环轮询 g_rpi_line_ready */
#define RPI_LINE_MAX_LEN  96
extern volatile uint8_t g_rpi_line_ready;
extern char g_rpi_line[RPI_LINE_MAX_LEN];

#endif
