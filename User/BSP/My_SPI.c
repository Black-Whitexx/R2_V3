//
// Created by BxW on 2024/6/11.
//
//考虑弃用，spi的HAL函数已经实现过了，我是傻逼
#include "My_SPI.h"

uint8_t rxbuf[32];

//uint8_t SPI_Read(SPI_HandleTypeDef *spi,uint8_t *data){
//    if(spi->Instance == SPI4){
//        copyArray(rxbuf,data,32);
//        return 0;
//    }
//    return 1;
//}

uint8_t SPI_Read(SPI_HandleTypeDef *spi,uint8_t *data){
    if(spi->Instance == SPI4){
        HAL_SPI_Receive(&hspi4,data,32,500);
        return 0;
    }
    return 1;
}

uint8_t SPI_Write(SPI_HandleTypeDef *spi,uint8_t *data){
    while(HAL_OK != HAL_SPI_Transmit(spi,data,32,500));
    return 0;
}

//void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi){
//    __HAL_SPI_DISABLE_IT(&hspi4,SPI_IT_RXNE);
//    if(hspi->Instance == SPI4){
//        HAL_SPI_Receive_IT(&hspi4,rxbuf,32);
//    }
//    __HAL_SPI_ENABLE_IT(&hspi4,SPI_IT_RXNE);
//}