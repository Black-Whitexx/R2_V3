//
// Created by BxW on 2024/6/10.
//

#include "Dji_Motor.h"
#include "global_Inc.h"
#include "My_Can.h"
//extern uint8_t fdcan_buffer[MOTOR_NUM][8];//从下层MYCAN文件中引用全局变量，进行处理

/**
 * @brief 对大疆电机的反馈报文进行解包
 * @param ptr 目标电机
 * @param data 8字节can报文
 */
//void updateDjiMotor(MotorInfo_t *MotorInfo)
//{
//    /*首次上电时获取电机转子的初始位置*/
//    if(MotorInfo->init_flag == 0)
//    {
//        MotorInfo->offset_angle = MotorInfo->position = ((data[0] << 8) | data[1]);    //转子位置
//        MotorInfo->init_flag = 1;     //记录初始位置标志位
//    }
//    else
//    {
//        /*转子信息获取*/
//        MotorInfo->last_position = MotorInfo->position;//迭代位置
//        MotorInfo->position = ((data[0] << 8) | data[1]);    //转子位置
//        MotorInfo->speed = (int16_t)((data[2] << 8) | data[3]);    //转子速度
//        MotorInfo->torque_current = (int16_t)((data[4] << 8) | data[5]);    //转子扭矩
//        MotorInfo->temp = data[6];   //电机温度
//
//        /*判断圈数是否加减*/
//        if(MotorInfo->position - MotorInfo->last_position > 4095)
//            MotorInfo->turns--;
//        else if(MotorInfo->position - MotorInfo->last_position < -4095)
//            MotorInfo->turns++;
//
//        /*计算转过的总角度*/
//        MotorInfo->total_angle = MotorInfo->turns * 8192 + MotorInfo->position - MotorInfo->offset_angle;
//        MotorInfo->actual_total_angle = (float)MotorInfo->total_angle * 360.0f / 8192;
//    }
//}
/**
* @brief  使用fdcan发送四个rm电机的电流(堵塞式)
* @param  id_range 发送id 前四个电机为200，后四个为1FF
* @param  currentx 该组中第x个电机的电流
*/
void Dji_Motor_Set(uint8_t id,int32_t current){
    if(0 <= id && id <= 4){
        uint8_t txbuf[8] = {0};
        txbuf[id-1] = (current >> 8) & 0xff;
        txbuf[id-1+1] = (current) & 0xff;
        MyCan_SendData(&hfdcan1,0x1ff,txbuf);
    }
    else if(5 <= id && id <= 8){
        uint8_t txbuf[8] = {0};
        txbuf[id-1] = (current >> 8) & 0xff;
        txbuf[id-1+1] = (current) & 0xff;
        MyCan_SendData(&hfdcan1,0x200,txbuf);
    }

}
void Dji_Motor_Get(uint8_t id,MotorFrame *motorFrame){
    uint8_t rxbuf[8] = {0};
    MyCan_ReadData(&hfdcan1,0x200+id,rxbuf);
    /*首次上电时获取电机转子的初始位置*/
    if(motorFrame->init_flag == 0)
    {
        motorFrame->offset_angle = motorFrame->position = ((rxbuf[0] << 8) | rxbuf[1]);    //转子位置
        motorFrame->init_flag = 1;     //记录初始位置标志位
    }
    else
    {
        /*转子信息获取*/
        motorFrame->last_position = motorFrame->position;//迭代位置
        motorFrame->position = ((rxbuf[0] << 8) | rxbuf[1]);    //转子位置
        motorFrame->speed = (int16_t)((rxbuf[2] << 8) | rxbuf[3]);    //转子速度
        motorFrame->torque_current = (int16_t)((rxbuf[4] << 8) | rxbuf[5]);    //转子扭矩
        motorFrame->temp = rxbuf[6];   //电机温度

        /*判断圈数是否加减*/
        if(motorFrame->position - motorFrame->last_position > 4095)
            motorFrame->turns--;
        else if(motorFrame->position - motorFrame->last_position < -4095)
            motorFrame->turns++;

        /*计算转过的总角度*/
        motorFrame->total_angle = motorFrame->turns * 8192 + motorFrame->position - motorFrame->offset_angle;
        motorFrame->actual_total_angle = (float)motorFrame->total_angle * 360.0f / 8192;
    }
}
