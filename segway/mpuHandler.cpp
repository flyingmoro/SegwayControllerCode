#include "mpuHandler.h"


MPU6050 mpu(PB_11, PB_10);

void getMPUReadings(MpuData *mpuReadings) {
    static int accRaw[3] = {0}, gyroRaw[3] = {0};
    mpu.getAcceleroRaw(accRaw);
    mpu.getGyroRaw(gyroRaw);

    mpuReadings->acceleration_x = accRaw[0];
    mpuReadings->acceleration_y = accRaw[1];
    mpuReadings->acceleration_z = accRaw[2];
    mpuReadings->angularRate_alpha = gyroRaw[0];
    mpuReadings->angularRate_beta = gyroRaw[1];
    mpuReadings->angularRate_gamma = gyroRaw[2];

}
