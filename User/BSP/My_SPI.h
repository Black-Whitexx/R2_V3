//
// Created by BxW on 2024/6/11.
//

#ifndef R2_MASTER_V3_MY_SPI_H
#define R2_MASTER_V3_MY_SPI_H

#include "global_Inc.h"
#include "spi.h"

uint8_t SPI_Read(SPI_HandleTypeDef *spi,uint8_t *data);
uint8_t SPI_Write(SPI_HandleTypeDef *spi,uint8_t *data);

#endif //R2_MASTER_V3_MY_SPI_H
