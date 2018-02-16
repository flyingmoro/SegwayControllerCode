#include "ultraSonic.h"

Ticker measurementStartTicker;
Timer forwardLeftTimer;
InterruptIn forwardLeftEchoPin(PE_14);
DigitalOut forwardLeftTriggerPin(PE_12);
bool forwardLeftMeasurementInProgress = false;


UltraSonicRanges *rangesPtr;

void forwardLeftFinished();
void startMeasurement();
void forwardLeftStartClock();


void initUltraSonic(UltraSonicRanges *ranges) {
    rangesPtr = ranges;
    forwardLeftEchoPin.rise(&forwardLeftStartClock);
    forwardLeftEchoPin.fall(&forwardLeftFinished);
    measurementStartTicker.attach(&startMeasurement, 0.02);
}

void startMeasurement() {
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








SonicRangeFinder::SonicRangeFinder(PinName _triggerPinName,
                                   PinName _echoPinName)
                                   : triggerPin(_triggerPinName)
                                   , echoPin(_echoPinName)
                                   , testPin(PE_13) {

    // echoPin.rise(Callback<void()>(this, &SonicRangeFinder::startClock));
    // echoPin.fall(Callback<void()>(this, &SonicRangeFinder::finished));
    // echoPin.rise(this, &SonicRangeFinder::startClock);
    // echoPin.fall(this, &SonicRangeFinder::finished);

    // delay first measurement
    timeout.attach(Callback<void()>(this, &SonicRangeFinder::startMeasurement), 0.00001);
};

void SonicRangeFinder::startMeasurement() {
    timeout.detach();

    // send start condition (10us high on trigger pin, 0.000002 results in around 10us)
    triggerPin = 1;
    timeout.attach(Callback<void()>(this, &SonicRangeFinder::startMeasurementFinished), 0.000002);
    testPin = 1;
}

void SonicRangeFinder::startMeasurementFinished() {
    testPin = 0;
    triggerPin = 0;
    // echoPin.rise(Callback<void()>(this, &SonicRangeFinder::startClock));
    echoPin.rise(this, &SonicRangeFinder::startClock);

    timeout.detach();
    timeout.attach(Callback<void()>(this, &SonicRangeFinder::measurementTimedOut), 1.0);
}

void SonicRangeFinder::startClock() {
    testPin = 1;
    timer.start();
    // echoPin.fall(Callback<void()>(this, &SonicRangeFinder::finished));
    echoPin.fall(this, &SonicRangeFinder::finished);
}

void SonicRangeFinder::finished() {
    testPin = 0;
    timer.stop();
    ITRangeInMM = (uint32_t)timer.read_us() / 5.82;
    timer.reset();
    timeout.detach();

    // 0.02s waiting needed before next measurement
    timeout.attach(Callback<void()>(this, &SonicRangeFinder::startMeasurement), 0.02);
}

void SonicRangeFinder::measurementTimedOut() {
    timeout.detach();

    // try again
    // 0.02s waiting needed before next measurement
    timeout.attach(Callback<void()>(this, &SonicRangeFinder::startMeasurement), 0.02);
}


float SonicRangeFinder::getRangeInMM() {
    float tempRange;
    __disable_irq();
    tempRange = ITRangeInMM;
    __enable_irq();
    return tempRange;
}
