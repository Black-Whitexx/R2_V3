//
// Created by BxW on 2024/6/11.
//

#include "DT35.h"
#include "Uart.h"
uint8_t DT35_Set(){
    return 0;
}
uint8_t DT35_Get(uint8_t number,float *distance){
    uint8_t dt35_buffer[32];
    Uart_Get(&huart4,dt35_buffer);
    if(dt35_buffer[0] == 0x06){
        if(number <= 3) {
            uint32_t dis_buf;
            dis_buf = (dt35_buffer[number] << 24) + (dt35_buffer[number + 1] << 16) + (dt35_buffer[number + 2] << 8) +
                      (dt35_buffer[number + 3] << 0);
            *distance = dis_buf / 10000.00;
        }
        else
            return 1;
    }
    return 0;
}