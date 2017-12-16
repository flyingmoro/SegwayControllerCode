#include <mbed.h>
#include <stdint.h>
#include "MPU6050.h"
#include "microRay.h"

// Serial pc(USBTX, USBRX, 38400);
MPU6050 mpu(PB_11, PB_10);
DigitalOut testPin(PE_15);

int main(void) {
    microRayInit();
    static int accRaw[3] = {0}, gyroRaw[3] = {0};

    if (mpu.testConnection() == true) {
        // pc.printf("connection established.");
    }
    while(1) {

        testPin = 1;
        mpu.getAcceleroRaw(accRaw);
        mpu.getGyroRaw(gyroRaw);
        testPin = 0;

        mr_acc_x = accRaw[0];
        mr_acc_y = accRaw[1];
        mr_acc_z = accRaw[2];
        mr_gyro_x = gyroRaw[0];
        mr_gyro_y = gyroRaw[1];
        mr_gyro_z = gyroRaw[2];

        microRayCommunicate();
        // pc.printf("accX: %06i, accY: %06i, accZ: %06i, ", accRaw[0], accRaw[1], accRaw[2]);
        // pc.printf("gyroX: %06i, gyroY: %06i, gyroZ: %06i, \n", gyroRaw[0], gyroRaw[1], gyroRaw[2]);
        wait(0.01);
    }
}
