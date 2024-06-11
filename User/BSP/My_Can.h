/**
  ******************************************************************************
  * @file           : My_Can.h
  * @author         : 86153
  * @brief          : None
  * @attention      : None
  * @date           : 2024/4/27
  ******************************************************************************
  */

#ifndef R2_MASTER_V3_MY_CAN_H
#define R2_MASTER_V3_MY_CAN_H

#include "stm32g4xx_hal.h"
#include "global_Inc.h"

#define Can1Peripheral_NUM 4
#define Can2Peripheral_NUM 4
#define Can3Peripheral_NUM 4

//typedef struct
//{
//    uint16_t angle;//转子角度 abs angle_now range:[0,8191]
//    int16_t speed;//转子速度 rpm
//    int16_t torque_current;//扭矩（以电流值为单位）
//    uint8_t temp;//电机温度
//    int32_t total_angle;//转子转过的总角度
//    int32_t last_total_angle;//记录上一次转子的总角度
//    float actual_total_angle;//将[0.8192]转到[0.360]
//    int16_t cnt;//转子转过的总圈数
//    uint16_t offset_angle;//上电时的转子位置（初始位置）
//    uint16_t last_angle;//abs angle_now range:[0,8191]
//    uint32_t msg_cnt;//消息计数值，收到一次就+1
//}MotorInfo_t;

typedef union
{
    uint8_t data_8[8];
    uint16_t data_16[4];
    int32_t data_int[2];
    uint32_t data_uint[2];
    float data_f[2];
}union_64;

typedef union
{
    uint8_t data_8[4];
    uint16_t data_16[2];
    int32_t data_int;
    uint32_t data_uint;
    float data_f;
}union_32;

typedef struct {
    uint32_t id;
    uint8_t data[8];
}Can_Frame;//CAN帧数据格式

uint8_t MyCan_ReadData(FDCAN_HandleTypeDef *hcan,uint32_t id,uint8_t *data);
uint8_t MyCan_SendData(FDCAN_HandleTypeDef *hcan,uint32_t id,uint8_t *data);

//电机反馈报文接收相关宏
//#define MOTOR_NUM 8

//void MotorInfo_Record(MotorInfo_t *ptr, uint8_t const* data);
//void Set_Current(FDCAN_HandleTypeDef *_hfdcan, int16_t id_range, int16_t current1, int16_t current2, int16_t current3, int16_t current4);
void FDCan1_Filter_config(void);
void FDCan2_Filter_config(void);
void FDCan3_Filter_config(void);

#endif //R2_MASTER_V3_MY_CAN_H
