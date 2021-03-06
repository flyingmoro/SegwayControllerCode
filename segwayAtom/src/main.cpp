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

Serial pc(USBTX, USBRX, 115200);
Timer loopTimer;

DigitalOut greenLed(LED1);
DigitalOut blueLed(LED2);
DigitalOut redLed(LED3);


Position worldPosition;
MpuData mpuData;
SonicRangeFinder rangeFinder(PE_12, PF_12);
SensorDataCollection sensorReadingsForControl;
TargetValues currentTargets;

//mbed compile -t GCC_ARM -m detect -f
int main() {
    pc.printf("Starting monoBot\n");
    int initReturnCode = init();
    if (initReturnCode != 0) {
        pc.printf("failed initializing monoBot. Error code: %i\n", initReturnCode);
        while (1) {
            redLed = !redLed;
            wait(0.5);
        }
    }


    while(1) {
        loopTimer.start();
        loop();
        dutyCycleTime = loopTimer.read_us();
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
    return 0;
}


void loop() {

    // for debugging, see if microcontroller is running
    pulsi += 1;
    if (pulsi > 100) {
        pulsi = 0;
    }

    // read encoders and calculate world position and derivatives
    updatePosition(&worldPosition);

    // get latest ultra sonic readings
    sonic = rangeFinder.getRangeInMM();

    // read the gyro and acceleration sensor (MPU6050)
    // duration approx 600us
    updateMpuReadings(&mpuData);

    // collect data for control algorythm and calculate target motor current
    sensorReadingsForControl.speed = 1.0;
    sensorReadingsForControl.beta = mpuData.rawAngularRate_beta;
    sensorReadingsForControl.gammaP = 1.0;
    updateControlTargets(&sensorReadingsForControl, &currentTargets);

    // apply current to motors
    setCurrentBothMotors(setCurrentMotorZero, setCurrentMotorOne);


    // microRay output stuff
    controllerOutputDebug = currentTargets.motorZero;

    encoderLeftWheel = worldPosition.x;
    encoderRightWheel = worldPosition.y;

    mr_rawAccX = mpuData.rawAcceleration_x;
    mr_rawAccY = mpuData.rawAcceleration_y;
    mr_rawAccZ = mpuData.rawAcceleration_z;
    mr_rawDegPX = mpuData.rawAngularRate_alpha / 131.0 + 18;
    mr_rawDegPY = mpuData.rawAngularRate_beta / 131.0;
    mr_rawDegPZ = mpuData.rawAngularRate_gamma / 131.0;

    mr_roll = mpuData.roll;
    mr_gyroXAngle = mpuData.gyroXAngle;
    mr_compXAngle = mpuData.compXAngle;
    mr_kalXAngle = mpuData.kalXAngle;
    microRayCommunicate();

}
