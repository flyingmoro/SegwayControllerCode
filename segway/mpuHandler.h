#ifndef MPUHANDLER_H
#define MPUHANDLER_H

#include <mbed.h>
#include <stdint.h>
#include "MPU6050.h"


typedef struct mpuData {
    uint16_t acceleration_x;
    uint16_t acceleration_y;
    uint16_t acceleration_z;
    uint16_t angularRate_alpha;
    uint16_t angularRate_beta;
    uint16_t angularRate_gamma;
} MpuData;




void getMPUReadings(MpuData *mpuReadings);






#endif
