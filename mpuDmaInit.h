#ifndef MPUDMAINIT_H
#define MPUDMAINIT_H

#include "mbed.h"

extern DMA_HandleTypeDef hdma_i2c2_rx;
extern I2C_HandleTypeDef i2cHandle;


void MPU_GPIO_Init();
void MPU_DMA_Init();
void MPU_I2C_Init();
void MPU_NVIC_Init();
void MPU_POST_Init();

#endif
