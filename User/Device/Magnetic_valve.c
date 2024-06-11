//
// Created by BxW on 2024/6/11.
//

#include "Magnetic_valve.h"

Magnetic_valve Magnetic_valve_status[Magnetic_valve_NUM];

uint8_t Magnetic_valve_Set(uint8_t number, uint8_t status){
    //HAL_GPIO_WritePin()
    return 0;
}
uint8_t  Magnetic_valve_Get(uint8_t number,uint8_t *status){
    for (int i = 0; i < Magnetic_valve_NUM; i++) {
        if (Magnetic_valve_status[i].number == number) {
            *status = Magnetic_valve_status[i].status;
            return 0;
        }
    }
    return 1;
}