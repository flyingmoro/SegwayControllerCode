#include <mbed.h>
#include "microRay.h"

void firstTorqueWriteComplete(int event);
void secondTorqueWriteComplete(int event);

DigitalOut blueLed(LED2);
DigitalOut greenLed(LED1);
DigitalOut togglePin(PF_13);

Serial oDrive(PG_14, PG_9, 115200);

char torqueCmdOne[11];
char torqueCmdTwo[11];

event_callback_t firstWriteEventComplete = firstTorqueWriteComplete;
event_callback_t secondWriteEventComplete = secondTorqueWriteComplete;


void firstTorqueWriteComplete(int event) {
    sprintf(torqueCmdTwo, "$c %i %.2f!", 1, mR_torqueMotorTwo);
    oDrive.write((uint8_t *)torqueCmdTwo, sizeof(torqueCmdTwo), secondWriteEventComplete, SERIAL_EVENT_TX_COMPLETE);
}

void secondTorqueWriteComplete(int event) {
}

int main() {
    microRayInit();
    while(1) {
        togglePin = !togglePin;
        greenLed = !greenLed;
        blueLed = !greenLed;
        sprintf(torqueCmdOne, "$c %i %.2f!", 0, mR_torqueMotorOne);
        oDrive.write((uint8_t *)torqueCmdOne, sizeof(torqueCmdOne), firstWriteEventComplete, SERIAL_EVENT_TX_COMPLETE);
        hallo += 0.1f;
        microRayCommunicate();
        togglePin = !togglePin;
        wait(0.005);
    }
}
