#include "mpuHandler.h"
#include "Kalman.h"

// MPU6050 mpu(PB_11, PB_10);
MPU6050 mpu(PF_15, PF_14);

#define DELTA_T 0.001
#define RAD_TO_DEG 57.3

Kalman kalmanX;
Kalman kalmanY;

void updateMpuReadings(MpuData *mpuReadings) {
    static int accRaw[3] = {0}, gyroRaw[3] = {0};
    mpu.getAcceleroRaw(accRaw);
    mpu.getGyroRaw(gyroRaw);

    mpuReadings->rawAcceleration_x = accRaw[1];
    mpuReadings->rawAcceleration_y = accRaw[2];
    mpuReadings->rawAcceleration_z = accRaw[0];
    mpuReadings->rawAngularRate_alpha = gyroRaw[1];
    mpuReadings->rawAngularRate_beta = gyroRaw[2];
    mpuReadings->rawAngularRate_gamma = gyroRaw[0];

    mpuReadings->roll = atan2(mpuReadings->rawAcceleration_y, mpuReadings->rawAcceleration_x) * RAD_TO_DEG;
    mpuReadings->pitch  = atan(-mpuReadings->rawAcceleration_z / sqrt(mpuReadings->rawAcceleration_z * mpuReadings->rawAcceleration_z + mpuReadings->rawAcceleration_x * mpuReadings->rawAcceleration_x)) * RAD_TO_DEG;


    float gyroXrate = mpuReadings->rawAngularRate_gamma / 131.0;
    float gyroYrate = mpuReadings->rawAngularRate_beta / 131.0;

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if (mpuReadings->roll< -90 && mpuReadings->kalXAngle > 90)  || (mpuReadings->roll > 90 && mpuReadings-> kalXAngle < -90 )
    {
        kalmanX.setAngle(mpuReadings->roll);
        mpuReadings->compXAngle = mpuReadings->roll;
        mpuReadings->kalXAngle = mpuReadings->roll;
        mpuReadings->gyroXAngle = mpuReadings->roll;
    }
    else
    {
        // Calculate the angle using a Kalman filter
        mpuReadings->kalXAngle = kalmanX.getAngle(mpuReadings->roll, gyroXrate, DELTA_T);
    }
    if (abs(mpuReadings->kalXAngle) > 90)
    {
        // Invert rate, so it fits the restriced accelerometer reading
        gyroYrate = -gyroYrate;
    }
    mpuReadings->kalYAngle = kalmanY.getAngle(mpuReadings->pitch, gyroYrate, DELTA_T);

    // Calculate gyro angle without any filter
    mpuReadings->gyroXAngle += gyroXrate * DELTA_T;
    mpuReadings->gyroYAngle += gyroYrate * DELTA_T;
    //gyroXangle += kalmanX.getRate() * DELTA_T; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * DELTA_T;

    // Calculate the angle using a Complimentary filter
    mpuReadings->compXAngle = 0.93 * (mpuReadings->compXAngle + gyroXrate * DELTA_T) + 0.07 * mpuReadings->roll;
    mpuReadings->compYAngle = 0.93 * (mpuReadings->compYAngle + gyroYrate * DELTA_T) + 0.07 * mpuReadings->pitch;

    // Reset the gyro angle when it has drifted too much
    if (mpuReadings->gyroXAngle < -180 || mpuReadings->gyroXAngle > 180) {
        mpuReadings->gyroXAngle = mpuReadings->kalXAngle;
    }
    if (mpuReadings->gyroYAngle < -180 || mpuReadings->gyroYAngle > 180) {
        mpuReadings->gyroYAngle = mpuReadings->kalYAngle;
    }

}
