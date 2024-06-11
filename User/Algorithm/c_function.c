//
// Created by BxW on 2024/6/11.
//

#include "c_function.h"
#include "stm32g4xx.h"
//数组复制
void copyArray(uint8_t *source, uint8_t *destination, uint8_t size) {
    for(int i = 0; i < size; i++) {
        destination[i] = source[i];
    }
}