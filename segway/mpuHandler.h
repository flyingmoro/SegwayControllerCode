#ifndef MPUHANDLER_H
#define MPUHANDLER_H

#include <mbed.h>
#include <stdint.h>
#include "MPU6050.h"


typedef struct mpuData {
    int16_t rawAcceleration_x;
    int16_t rawAcceleration_y;
    int16_t rawAcceleration_z;
    int16_t rawAngularRate_alpha;
    int16_t rawAngularRate_beta;
    int16_t rawAngularRate_gamma;
    float roll;
    float gyroXAngle;
    float compXAngle;
    float kalXAngle;
    float pitch;
    float gyroYAngle;
    float compYAngle;
    float kalYAngle;
    // float yaw;
    // float gyroZAngle;
    // float compZAngle;
    // float kalZAngle;
} MpuData;




void updateMpuReadings(MpuData *mpuReadings);






#endif
