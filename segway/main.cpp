#include "mbed.h"
#include "microRay.h"

#include "oDriveCommunicator.h"
#include "encoderHandling.h"
#include "mpuHandler.h"
#include "ultraSonic.h"

int init();
void loop();
void firstTorqueWriteComplete(int event);
void secondTorqueWriteComplete(int event);

Serial pc(USBTX, USBRX);
Timer loopTimer;

DigitalOut greenLed(LED1);
DigitalOut blueLed(LED2);
DigitalOut redLed(LED3);


Position worldPosition;
MpuData mpuData;
UltraSonicRanges sonicRanges;

//mbed compile -t GCC_ARM -m detect -f
int main()
{
    pc.printf("Starting monoBot\n");
    int initReturnCode = init();
    if (initReturnCode != 0) {
        pc.printf("failed initializing monoBot. Error code: %i\n", initReturnCode);
        while (1) {
            redLed = !redLed;
            wait(0.5);
        }
    }


    while(1)
    {
        loopTimer.start();
        loop();
        while(loopTimer.read_us() < loopCycleTimeUs)
        {
            // wait for loop cycle period to decay
        }
        loopTimer.reset();
    }
}

int init() {
    initEncoder();
    microRayInit();
    initUltraSonic(&sonicRanges);
    return 0;
}


void loop() {
    setCurrentBothMotors(setCurrentMotorZero, setCurrentMotorOne);
    refreshPosition(&worldPosition);
    getMPUReadings(&mpuData);
    encoderLeftWheel = worldPosition.x;
    encoderRightWheel = worldPosition.y;
    pulsi += 1;
    if (pulsi > 100) {
        pulsi = 0;
    }
    accX = mpuData.acceleration_x;
    sonic = sonicRanges.forwardLeftMM;
    microRayCommunicate();
}
