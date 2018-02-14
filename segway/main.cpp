#include "mbed.h"
#include "microRay.h"
#include "Kalman.h"

#include "oDriveCommunicator.h"
#include "encoderHandling.h"
#include "mpuHandler.h"
// #include "mpuDma.h"
#include "ultraSonic.h"
#include "controllers.h"

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
SensorDataCollection sensorReadingsForControl;
TargetValues currentTargets;

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
    microRayInit();
    initEncoder();
    // initMPU6050();
    initUltraSonic(&sonicRanges);
    return 0;
}


void loop() {

    // debugging, see if microcontroller is running
    pulsi += 1;
    if (pulsi > 100) {
        pulsi = 0;
    }

    updatePosition(&worldPosition);
    updateMpuReadings(&mpuData);
    encoderLeftWheel = worldPosition.x;
    encoderRightWheel = worldPosition.y;
    accX = mpuData.rawAcceleration_x;
    sonic = sonicRanges.forwardLeftMM;

    sensorReadingsForControl.speed = 1.0;
    sensorReadingsForControl.beta = mpuData.rawAngularRate_beta;
    sensorReadingsForControl.gammaP = 1.0;
    updateControlTargets(&sensorReadingsForControl, &currentTargets);

    setCurrentBothMotors(setCurrentMotorZero, setCurrentMotorOne);


    // mpuData = getMpuData();


    controllerOutputDebug = currentTargets.motorZero;
    mr_gyroXAngle = mpuData.gyroXAngle;
    mr_compXAngle = mpuData.compXAngle;
    mr_kalXAngle = mpuData.kalXAngle;
    microRayCommunicate();
}
