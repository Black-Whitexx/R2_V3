//
// Created by BxW on 2024/6/10.
//

#ifndef R2_MASTER_V3_DJI_MOTOR_H
#define R2_MASTER_V3_DJI_MOTOR_H
#include "stm32g4xx.h"
#include "My_Can.h"
//描述大疆电机的状态和相关中间变量
typedef struct
{
    uint8_t init_flag;//初始化标志
    uint16_t position;//转子角度 abs angle_now range:[0,8191]
    int16_t speed;//转子速度 rpm
    int16_t torque_current;//扭矩（以电流值为单位）
    uint8_t temp;//电机温度
    int32_t total_angle;//转子转过的总角度
    int32_t last_total_angle;//记录上一次转子的总角度
    float actual_total_angle;//将[0-8192]转到[0-360]
    int32_t turns;//转子转过的总圈数
    uint16_t offset_angle;//上电时的转子位置（初始位置）
    uint16_t last_position;//abs angle_now range:[0,8191]

}MotorFrame;

void Dji_Motor_Set(uint8_t id,int32_t current);
void Dji_Motor_Get(uint8_t id,MotorFrame *motorFrame);

#endif //R2_MASTER_V3_DJI_MOTOR_H
