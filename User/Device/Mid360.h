//
// Created by BxW on 2024/6/11.
//

#ifndef R2_MASTER_V3_MID360_H
#define R2_MASTER_V3_MID360_H
#define R 0.28432f

#include "global_Inc.h"
#include "Uart.h"
#include "arm_math.h"

typedef struct {
    float locx;
    float locy;
    float locz;
    float yaw;
}RaDar_Frame;

typedef struct
{
    uint8_t flag;
    float vision_x;
    float vision_y;
}Vision_Frame;

uint8_t Mid360_Set();
uint8_t Mid360_Get();

#endif //R2_MASTER_V3_MID360_H
