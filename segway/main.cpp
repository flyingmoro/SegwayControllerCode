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
Timer segwayDebugTimer;

DigitalOut greenLed(LED1);
DigitalOut blueLed(LED2);
DigitalOut redLed(LED3);
float mpuAngleBuffer[4];

Position worldPosition;
MpuData mpuData;
SonicRangeFinder rangeFinder(PE_12, PF_12);
SensorDataCollection sensorReadingsForControl;
TargetValues currentTargets;



//mbed compile -t GCC_ARM -m NUCLEO_F746ZG -f
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
        mr_dutyCycleTime = loopTimer.read_us();
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
    initODrive();
    // while (1){}
    return 0;
}


void loop() {

    // for debugging, see if microcontroller is running
    mr_pulsi += 1;
    if (mr_pulsi > 100) {
        mr_pulsi = 0;
    }




    // read encoders and calculate world position and derivatives
    updatePosition(&worldPosition);





    // // get latest ultra sonic reading
    // mr_sonic = rangeFinder.getRangeInMM();



    segwayDebugTimer.start();

    // read the gyro and acceleration sensor (MPU6050)
    // duration approx 600us
    updateMpuReadings(&mpuData);

    mr_debugTimer = segwayDebugTimer.read_us();
    segwayDebugTimer.stop();
    segwayDebugTimer.reset();











    // collect data for control algorythm and calculate target motor current
    sensorReadingsForControl.x = worldPosition.x * 1000;
    sensorReadingsForControl.y = worldPosition.y * 1000;
    sensorReadingsForControl.speed = worldPosition.forwardSpeed;
    sensorReadingsForControl.gamma = worldPosition.gamma;
    sensorReadingsForControl.gammaP = worldPosition.gammaP * 0.0174;
    // sensorReadingsForControl.beta = mpuData.kalYAngle * 0.0174;
    sensorReadingsForControl.beta = mpuData.compYAngle * 0.0174;
    updateControlTargets(&sensorReadingsForControl, &currentTargets);

    // disable motors in case of tilt exeedance
    if (abs(mpuData.compYAngle) > 45.0) {
        currentTargets.motorZero = 0.0;
        currentTargets.motorOne = 0.0;
    }

    // disable motors in case of bad mpu data
    // check, if mpuData is refreshing over time, else motors off
    int i = 0;
    for (i = 1; i < 4; i++) {
        mpuAngleBuffer[i-1] = mpuAngleBuffer[i];
    }
    mpuAngleBuffer[3] = mpuData.rawAcceleration_x;
    int mpuDataOk = 0;
    for (i = 1; i < 4; i++) {
        if(mpuAngleBuffer[i] != mpuAngleBuffer[i-1]) {
            mpuDataOk = 1;
        }
    }
    if (mpuDataOk == 0) {
        currentTargets.motorZero = 0.0;
        currentTargets.motorOne = 0.0;
    }

    // // apply current to motors
    // if (mr_controllerMasterSwitch == 1) {
    //     setCurrentBothMotors(currentTargets.motorZero, -1 * currentTargets.motorOne);
    // } else {
    //     // set current manually from microRay
    //     setCurrentBothMotors(mr_currentMotorZero, mr_currentMotorOne);
    // }













    // microRay output stuff
    mr_controllerOutputMotorZero = currentTargets.motorZero;
    mr_controllerOutputMotorOne = currentTargets.motorOne;

    mr_encoderLeftWheel = worldPosition.encLeft;
    mr_encoderRightWheel = worldPosition.encRight;
    mr_worldX = worldPosition.x;
    mr_worldY = worldPosition.y;
    mr_worldGamma = worldPosition.gamma;

    mr_rawAccX = mpuData.rawAcceleration_x;
    mr_rawAccY = mpuData.rawAcceleration_y;
    mr_rawAccZ = mpuData.rawAcceleration_z;
    mr_rawDegPX = mpuData.rawAngularRate_alpha / 131.0 + 18;
    mr_rawDegPY = mpuData.rawAngularRate_beta / 131.0;
    mr_rawDegPZ = mpuData.rawAngularRate_gamma / 131.0;

    mr_betaTangens = mpuData.pitch;
    mr_betaRawIntegral = mpuData.gyroYAngle;
    mr_betaComplementary = mpuData.compYAngle;
    mr_betaKalman = mpuData.kalYAngle;
    mr_speed = worldPosition.forwardSpeed;
    microRayCommunicate();

}
