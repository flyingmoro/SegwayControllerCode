#include "oDriveCommunicator.h"

#include "mbed.h"
void firstTorqueWriteComplete(int event);
void secondTorqueWriteComplete(int event);

Serial oDrive(PG_14, PG_9, 921600);
char torqueCmdOne[11];
char torqueCmdTwo[11];
event_callback_t firstWriteEventComplete = firstTorqueWriteComplete;
event_callback_t secondWriteEventComplete = secondTorqueWriteComplete;
// float motorZeroTarget = 0.0f;
// float motorOneTarget = 0.0f;
volatile int lastTransmissionComplete = 1;


typedef struct CurrentCommand {
    int32_t parameterNumber;
    int32_t parameterValueInt;
    float parameterValueFloat;
} CurrentCommand;


int initODrive() {
    oDrive.printf("$t!");
    return 0;
}

void firstTorqueWriteComplete(int event) {
    // sprintf(torqueCmdTwo, "$c %i %.2f!", 1, motorOneTarget);
    oDrive.write((uint8_t *)torqueCmdTwo, sizeof(torqueCmdTwo), secondWriteEventComplete, SERIAL_EVENT_TX_COMPLETE);
}

void secondTorqueWriteComplete(int event) {
    lastTransmissionComplete = 1;
}


int setCurrentBothMotors(float currentMotorZero, float currentMotorOne) {
    if (0) { }
    // if (lastTransmissionComplete == 0) {
    //     return -1;
    // }
    else {
        // lastTransmissionComplete = 0;
        // motorOneTarget = currentMotorOne;
        sprintf(torqueCmdOne, "$c %i %.2f!", 0, currentMotorZero);
        sprintf(torqueCmdTwo, "$c %i %.2f!", 1, currentMotorOne);
        oDrive.write((uint8_t *)torqueCmdOne, sizeof(torqueCmdOne), firstWriteEventComplete, SERIAL_EVENT_TX_COMPLETE);
        return 0;
    }
    return 0;
}
