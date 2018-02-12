#include "ultraSonic.h"

Ticker measurementStartTicker;
Timer forwardLeftTimer;
InterruptIn forwardLeftEchoPin(PE_14);
DigitalOut forwardLeftTriggerPin(PE_12);
bool forwardLeftMeasurementInProgress = false;


UltraSonicRanges *rangesPtr;

void forwardLeftFinished();
void triggerMeasurement();
void forwardLeftStartClock();


void initUltraSonic(UltraSonicRanges *ranges) {
    rangesPtr = ranges;
    forwardLeftEchoPin.rise(&forwardLeftStartClock);
    forwardLeftEchoPin.fall(&forwardLeftFinished);
    measurementStartTicker.attach(&triggerMeasurement, 0.02);
}

void triggerMeasurement() {
    if (forwardLeftMeasurementInProgress == false) {
        forwardLeftMeasurementInProgress = true;
        forwardLeftTriggerPin = 1;
        wait(0.00001);
        forwardLeftTriggerPin = 0;
    }
}

void forwardLeftStartClock() {
    forwardLeftTimer.start();
}

void forwardLeftFinished() {
    rangesPtr->forwardLeft = (uint32_t)forwardLeftTimer.read_us();
    rangesPtr->forwardLeftMM = rangesPtr->forwardLeft / 5.82;
    forwardLeftTimer.stop();
    forwardLeftTimer.reset();
    forwardLeftMeasurementInProgress = false;
}
