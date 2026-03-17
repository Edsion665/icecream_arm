#ifndef __SERIAL_COMMUNICATION_H
#define __SERIAL_COMMUNICATION_H

#include "stm32f10x.h"

// 串口缓冲区大小
#define USART1_RECEIVE_BUFFER_SIZE 1024
#define USART2_RECEIVE_BUFFER_SIZE 1024

// 串口接收缓冲区
extern uint8_t USART1_ReceiveBuffer[USART1_RECEIVE_BUFFER_SIZE];
extern uint16_t USART1_ReceiveLength;
extern uint8_t USART2_ReceiveBuffer[USART2_RECEIVE_BUFFER_SIZE];
extern uint16_t USART2_ReceiveLength;

#endif /* __SERIAL_COMMUNICATION_H */
