//
// Created by BxW on 2024/6/11.
//

#include "Chassis.h"

//TODO:坐标系有误，待改
PID_Para LeftFront, RightFront, LeftBack, RightBack, Claw_RollBack, Filter_Board;

uint8_t Chassis_Set_Status(Chassis_Status *chassisStatus) {

}

uint8_t Chassis_Get_Status(Chassis_Status *chassisStatus) {

}

uint8_t Chassis_Get_Location_Angle(uint8_t Red_or_Blue, float *x, float *y, float *angle, uint8_t locater_device) {
    RaDar_Frame raDar_buf;
    RS_Frame rs_buf;
    Mid360_Get(&raDar_buf, &rs_buf);
    *angle = raDar_buf.yaw;

    if (locater_device == DEVICE_DT35) {//目前仅适配3区
        float dt35_buf[3];
        DT35_Get(1, &dt35_buf[0]);
        DT35_Get(2, &dt35_buf[1]);
        DT35_Get(3, &dt35_buf[2]);
        //1号（后）和3号（右）数据有效
        if (Red_or_Blue == RED) {
            *x = dt35_buf[0];
            *y = dt35_buf[2];
        } else if (Red_or_Blue == BLUE) {
            *x = dt35_buf[0];
            *y = dt35_buf[1];
        }
        return 0;
    } else if (locater_device == DEVICE_MID360) {
        *x = raDar_buf.locx;
        *y = raDar_buf.locy;
        return 0;
    } else
        return 1;
}

uint8_t Chassis_Move(uint8_t Red_or_Blue, float x, float y, float angle, uint8_t locater_device) {
    float x_now, y_now, angle_now;
    Chassis_Get_Location_Angle(Red_or_Blue, &x_now, &y_now,&angle_now, locater_device);
    float xSpeed = 0.0f, ySpeed = 0.0f;
    float dis = 0.0f;//当前点与目标点的距离
    float vel = 0.0f, omega = 0.0f;//速度,角速度
    float err_x = (x - x_now);//x差值
    float err_y = (y - y_now);//y差值
    float delta_angle = (angle - angle_now);//角度差值
    float max_out = 0.0f, allErr_x = 0.0f, allErr_y = 0.0f;
    static float all_dis = 0.0f;

    if (cnt == 0) {
        allErr_x = (x - x_now);
        allErr_y = (y - x_now);
        arm_sqrt_f32(allErr_x * allErr_x + allErr_y * allErr_y, &all_dis);
        cnt = 1;
    }
    //计算向量长度
    arm_sqrt_f32((float32_t)(err_x * err_x + err_y * err_y), &dis);
    //根据距离选取不同速度
    if (dis / all_dis >= 0.9) max_out = 1.5f;
    else if (dis / all_dis >= 0.8 && dis / all_dis < 0.9) max_out = 2.0f;
    else if (dis / all_dis >= 0.7 && dis / all_dis < 0.8) max_out = 2.5f;
    else if (dis / all_dis < 0.7) max_out = 3.0f;
    //计算平动速度向量
    vel = PID_Process(&Translation_PID, 0, -dis, max_out, 0.01f);
    //速度向量取绝对值
    arm_abs_f32(&vel, &vel, 1);
    //计算角速度
    omega = PID_Process(&Turn_PID, 0, -delta_angle, 1.3f, 0.1f);

    //线速度分解为x和y的分量
    xSpeed = vel * arm_cos_f32(atan2f(err_y, err_x));
    ySpeed = vel * arm_sin_f32(atan2f(err_y, err_x));

    //将车身x，y速度转换为轮子的x，y速度
    SGW2Wheels(xSpeed, ySpeed, omega, angle_now);
    return 0;
}

/**
 * @brief 四全向轮底层解算函数
 * @param vel_x 世界坐标系中车的x轴速度 m/s
 * @param vel_y 世界坐标系中车的y轴速度 m/s
 * @param omega 旋转角速度 rad/s
 */
void SGW2Wheels(float vel_x, float vel_y, float omega, float theta)
{
    /********直接计算方法********/
    theta = theta * angle2rad;
    float sin_ang = arm_sin_f32(theta);
    float cos_ang = arm_cos_f32(theta);

    Wheels_vel[0] = ( ((-cos_ang - sin_ang) * vel_x + (-sin_ang + cos_ang) * vel_y ) / (float)sqrt(2) + l * omega ) * 60 * 19 / pi / D;
    Wheels_vel[1] = ( ((-cos_ang + sin_ang) * vel_x + (-sin_ang - cos_ang) * vel_y ) / (float)sqrt(2) + l * omega ) * 60 * 19 / pi / D;
    Wheels_vel[2] = ( ((cos_ang + sin_ang) * vel_x + (sin_ang - cos_ang) * vel_y ) / (float)sqrt(2) + l * omega ) * 60 * 19 / pi / D;
    Wheels_vel[3] = ( ((cos_ang - sin_ang) * vel_x + (sin_ang + cos_ang) * vel_y ) / (float)sqrt(2) + l * omega ) * 60 * 19 / pi / D;


}