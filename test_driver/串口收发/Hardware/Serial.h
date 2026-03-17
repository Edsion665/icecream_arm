#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

#define SERIAL_RX_PACKET_LEN  6   /* 树莓派下发：电机1~4 + 舵机1~2 */
#define SERIAL_TX_PACKET_LEN  4

extern uint8_t Serial_TxPacket[SERIAL_TX_PACKET_LEN];
extern uint8_t Serial_RxPacket[SERIAL_RX_PACKET_LEN];

/* USART1 - 树莓派通信 */
void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

void Serial_SendPacket(void);
uint8_t Serial_GetRxFlag(void);

/* USART2 - 上位机监控 */
void Serial2_Init(void);
void Serial2_SendByte(uint8_t Byte);
void Serial2_SendArray(uint8_t *Array, uint16_t Length);
void Serial2_SendString(char *String);
void Serial2_SendNumber(uint32_t Number, uint8_t Length);

/**
  * @brief  通过USART2发送树莓派接收的数据和电机实际角度给上位机
  * @param  rpi_data: 树莓派下发的6字节数据 [M1,M2,M3,M4,S1,S2]
  * @param  motor_pos: 4个电机的实际角度（弧度）
  * @note   发送格式：[0xAA][0x55] + 树莓派6字节 + 电机4角度(4字节float*4) + [0xFE]
  */
void Serial2_SendMonitorData(uint8_t *rpi_data, float *motor_pos);

#endif
