#include <mbed.h>
#include "mpuDma.h"
#include "microRay.h"

void init();
void loop();

Timer loopTimer;
Serial pc(USBTX, USBRX, 115200);
MpuData mpuData;



int main() {
    init();

    while(1) {
        loopTimer.start();
        loop();
        // dutyCycleTime = loopTimer.read_us();
        while(loopTimer.read_us() < loopCycleTimeUs)
        {
            // wait for loop cycle period to decay
        }
        loopTimer.reset();
    }
}

void init() {
    microRayInit();
    // initMPU6050();
}


void loop() {

    // for debugging, see if microcontroller is running
    pulsi += 1;
    if (pulsi > 100) {
        pulsi = 0;
    }

    mpuData = getMpuData();

    debugChannel = mpuData.rawAngularRate_alpha;


    // microRay output stuff

    // mr_rawAccX = mpuData.rawAcceleration_x;
    // mr_rawAccY = mpuData.rawAcceleration_y;
    // mr_rawAccZ = mpuData.rawAcceleration_z;
    // mr_rawDegPX = mpuData.rawAngularRate_alpha / 131.0 + 18;
    // mr_rawDegPY = mpuData.rawAngularRate_beta / 131.0;
    // mr_rawDegPZ = mpuData.rawAngularRate_gamma / 131.0;
    //
    // mr_roll = mpuData.roll;
    // mr_gyroXAngle = mpuData.gyroXAngle;
    // mr_compXAngle = mpuData.compXAngle;
    // mr_kalXAngle = mpuData.kalXAngle;
    microRayCommunicate();

}
