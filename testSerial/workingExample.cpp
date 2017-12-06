#include <mbed.h>
//#include "microRay.h"

void pcReadComplete(int);
void pcSendComplete(int);

DigitalOut greenLed(LED1);

Serial oDrive(PG_14, PG_9, 115200);
Serial pc(USBTX, USBRX, 115200);

event_callback_t pcEventReceiveComplete = pcReadComplete;
uint8_t singleInBuffer = 0;
void pcReadComplete(int events) {
    pc.printf("incoming %c\n", singleInBuffer);

    // retrigger async receive of serial data
    pc.read(&singleInBuffer, 1, pcEventReceiveComplete);
}

event_callback_t pcEventWriteComplete = pcSendComplete;
void pcSendComplete(int event) {
}

int main() {
    //microRayInit();
    char outBuffer[] = "duda\n0";
    pc.read(&singleInBuffer, 1, pcEventReceiveComplete);
    while(1) {
        greenLed = !greenLed;
        oDrive.printf("$c %i %.3f!", 0, 1.1f);
        pc.write((uint8_t *)&outBuffer, sizeof(outBuffer), pcEventWriteComplete, SERIAL_EVENT_TX_COMPLETE);
        //oDrive.printf("$c %i %.3f!", 1, mR_torqueMotorTwo);
        //hallo += 0.1f;
        //microRayCommunicate();
        wait(1);
    }
}
