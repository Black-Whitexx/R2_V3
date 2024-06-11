//
// Created by BxW on 2024/6/11.
//

#ifndef R2_MASTER_V3_USART_H
#define R2_MASTER_V3_USART_H
#define UART_NUM 5

#include "global_Inc.h"

uint8_t Uart_Send(UART_HandleTypeDef *uart,uint8_t *data);
uint8_t Uart_Get(UART_HandleTypeDef *uart,uint8_t *data);

#endif //R2_MASTER_V3_USART_H
