#ifndef MPUHANDLER_H
#define MPUHANDLER_H

#include <mbed.h>
#include <stdint.h>
#include "MPU6050.h"


typedef struct mpuData {
    uint16_t rawAcceleration_x;
    uint16_t rawAcceleration_y;
    uint16_t rawAcceleration_z;
    uint16_t rawAngularRate_alpha;
    uint16_t rawAngularRate_beta;
    uint16_t rawAngularRate_gamma;
    float roll;
    float gyroXAngle;
    float compXAngle;
    float kalXAngle;
    float pitch;
    float gyroYAngle;
    float compYAngle;
    float kalYAngle;
    float yaw;
    float gyroZAngle;
    float compZAngle;
    float kalZAngle;
} MpuData;




void updateMpuReadings(MpuData *mpuReadings);






#endif
