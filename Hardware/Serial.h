#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

/* USART1（PA9/PA10）：与树莓派/上位机波特率必须一致 */
#ifndef SERIAL_USART1_BAUD
#define SERIAL_USART1_BAUD  115200u
#endif

void Serial_Init(void);
void Serial_ServiceRxDma(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

/*================ 文本行接收（USART1） ================*/
#define RPI_LINE_MAX_LEN  96
extern volatile uint8_t g_rpi_line_ready;
extern char g_rpi_line[RPI_LINE_MAX_LEN];
uint8_t Serial_GetNextLine(char *out, uint16_t cap);

/*================ 二进制帧（RPI <-> STM32，v3 固定 42 字节） ================*/
#define RPI_BIN_FRAME_LEN  42u
typedef struct {
    uint8_t data[RPI_BIN_FRAME_LEN];
} RpiBinFrame_t;

uint8_t Serial_GetNextBinFrame(RpiBinFrame_t *out);

#endif
