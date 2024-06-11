//
// Created by BxW on 2024/6/11.
//

#ifndef R2_MASTER_V3_VESC_H
#define R2_MASTER_V3_VESC_H

#define VESC_ID 127
#define VESC_CAN_PTR &hfdcan1

#include "global_Inc.h"

uint8_t VESC_Set(int32_t rpm);
uint8_t VESC_Get(void);

#endif //R2_MASTER_V3_VESC_H
