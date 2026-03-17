#include "serial_communication.h"

// 串口接收缓冲区
uint8_t USART1_ReceiveBuffer[USART1_RECEIVE_BUFFER_SIZE];
uint16_t USART1_ReceiveLength = 0;
uint8_t USART2_ReceiveBuffer[USART2_RECEIVE_BUFFER_SIZE];
uint16_t USART2_ReceiveLength = 0;
