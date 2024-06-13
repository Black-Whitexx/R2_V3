//
// Created by BxW on 2024/6/11.
//

#include "Mid360.h"

uint8_t Mid360_Set(){
    return 0;
}

uint8_t Mid360_Get(RaDar_Frame *raDarFrame, RS_Frame *rs_Frame){
    uint8_t rxbuf[32];
    Uart_Get(&huart2,rxbuf);
        if(rxbuf[0] == 0xAE)//0:帧头；1-12：坐标和角度；13：flag;17-24:realsense计算的xy距离
        {
            raDarFrame->locx = (float)(rxbuf[1] | rxbuf[2] << 8 | rxbuf[3] << 16 | rxbuf[4] << 24) / 1000.0f;
            raDarFrame->locy = (float)(rxbuf[5] | rxbuf[6] << 8 | rxbuf[7] << 16 | rxbuf[8] << 24) / 1000.0f;
            raDarFrame->yaw = (float)(rxbuf[9] | rxbuf[10] << 8 | rxbuf[11] << 16 | rxbuf[12] << 24) * 180.0f / PI / 1000.0f;

            raDarFrame->locy = raDarFrame->locy + R * arm_cos_f32(raDarFrame->yaw * PI / 180.0f) - R;
            raDarFrame->locx = raDarFrame->locx - R * arm_sin_f32(raDarFrame->yaw * PI / 180.0f);

            rs_Frame->flag = rxbuf[13];
            rs_Frame->vision_x = (float)(rxbuf[17] | rxbuf[18] << 8 | rxbuf[19] << 16 | rxbuf[20] << 24 );
            rs_Frame->vision_y = (float)(rxbuf[21] | rxbuf[22] << 8 | rxbuf[23] << 16 | rxbuf[24] << 24 );
            return 0;
        }

    return 1;
}