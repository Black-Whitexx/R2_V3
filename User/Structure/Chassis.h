//
// Created by BxW on 2024/6/11.
//

#ifndef R2_MASTER_V3_CHASSIS__H
#define R2_MASTER_V3_CHASSIS_H


#include "DT35.h"
#include "Mid360.h"
#include "arm_math.h"

typedef struct {
    uint8_t Red_Or_Blue;//想到好名字了再改
    double axis_x;
    double axis_y;
    uint8_t dt35_or_not;
    uint8_t Mid360_or_not;
}Chassis_Status;


uint8_t Chassis_Set_Status(Chassis_Status *chassisStatus);
uint8_t Chassis_Get_Status(Chassis_Status *chassisStatus);

#endif //R2_MASTER_V3_CHASSIS__H
