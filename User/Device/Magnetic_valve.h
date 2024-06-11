//
// Created by BxW on 2024/6/11.
//

#ifndef R2_MASTER_V3_MAGNETIC_VALVE_H
#define R2_MASTER_V3_MAGNETIC_VALVE_H

#define Magnetic_valve_NUM 2

#include "global_Inc.h"

typedef struct Magnetic_valve_t{
    uint8_t number;
    uint8_t status;
}Magnetic_valve;

uint8_t Magnetic_valve_Set(uint8_t number, uint8_t status);
uint8_t  Magnetic_valve_Get(uint8_t number,uint8_t *status);

#endif //R2_MASTER_V3_MAGNETIC_VALVE_H
