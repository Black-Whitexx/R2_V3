//
// Created by BxW on 2024/6/11.
//

#include "Uart.h"

uint8_t Uart1_Buffer[32];
uint8_t Uart2_Buffer[32];
uint8_t Uart3_Buffer[32];
uint8_t Uart4_Buffer[32];
uint8_t Uart5_Buffer[32];

uint8_t Uart_Send(UART_HandleTypeDef *uart,uint8_t *data){
    if(HAL_OK == HAL_UART_Transmit_DMA(uart,data,32)){
        return 0;
    }
    else
        return 1;
}
uint8_t Uart_Get(UART_HandleTypeDef *uart,uint8_t *data){
    if(uart == &huart1){
        copyArray(Uart1_Buffer,data,32);
    }
    else if(uart == &huart2){
        copyArray(Uart2_Buffer,data,32);
    }
    else if(uart == &huart3){
        copyArray(Uart3_Buffer,data,32);
    }
    else if(uart == &huart4){
        copyArray(Uart4_Buffer,data,32);
    }
    else if(uart == &huart5){
        copyArray(Uart5_Buffer,data,32);
    }
    else
        return 1;

    return 0;
}

void USART1_IRQHandler(void)
{

    __HAL_UART_CLEAR_IDLEFLAG(&huart1);                     //清除空闲中断标志（否则会�??????????????????直不断进入中断）
    HAL_UART_DMAStop(&huart1);                        //停止DMA接收

    HAL_UART_Receive_DMA(&huart1, Uart1_Buffer, 64);   //重启串口接收中断，开始DMA传输

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);             //重启串口空闲中断，防止被32自动清除标志空闲中断标志�??????????????????

    HAL_UART_IRQHandler(&huart1);

}

void USART2_IRQHandler(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);                     //清除空闲中断标志（否则会�??????????????????直不断进入中断）
    HAL_UART_DMAStop(&huart2);                        //停止DMA接收

    HAL_UART_Receive_DMA(&huart2, Uart2_Buffer, 64);   //重启串口接收中断，开始DMA传输

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);             //重启串口空闲中断，防止被32自动清除标志空闲中断标志�??????????????????

    HAL_UART_IRQHandler(&huart2);
}

void USART3_IRQHandler(void)
{

    __HAL_UART_CLEAR_IDLEFLAG(&huart3);                     //清除空闲中断标志（否则会�??????????????????直不断进入中断）
    HAL_UART_DMAStop(&huart3);                        //停止DMA接收

    HAL_UART_Receive_DMA(&huart3, Uart3_Buffer, 64);   //重启串口接收中断，开始DMA传输

    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);             //重启串口空闲中断，防止被32自动清除标志空闲中断标志�??????????????????

    HAL_UART_IRQHandler(&huart3);

}
void USART4_IRQHandler(void)
{

    __HAL_UART_CLEAR_IDLEFLAG(&huart4);                     //清除空闲中断标志（否则会�??????????????????直不断进入中断）
    HAL_UART_DMAStop(&huart4);                        //停止DMA接收

    HAL_UART_Receive_DMA(&huart4, Uart4_Buffer, 64);   //重启串口接收中断，开始DMA传输

    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);             //重启串口空闲中断，防止被32自动清除标志空闲中断标志�??????????????????

    HAL_UART_IRQHandler(&huart4);

}
void USART5_IRQHandler(void)
{

    __HAL_UART_CLEAR_IDLEFLAG(&huart5);                     //清除空闲中断标志（否则会�??????????????????直不断进入中断）
    HAL_UART_DMAStop(&huart5);                        //停止DMA接收

    HAL_UART_Receive_DMA(&huart5, Uart5_Buffer, 64);   //重启串口接收中断，开始DMA传输

    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);             //重启串口空闲中断，防止被32自动清除标志空闲中断标志�??????????????????

    HAL_UART_IRQHandler(&huart5);

}
