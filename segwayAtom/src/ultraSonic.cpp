#include "ultraSonic.h"


SonicRangeFinder::SonicRangeFinder(PinName _triggerPinName,
                                   PinName _echoPinName)
                                   : triggerPin(_triggerPinName)
                                   , echoPin(_echoPinName)
                                   , testPin(PE_13) {

    echoPin.rise(Callback<void()>(this, &SonicRangeFinder::startClock));
    echoPin.fall(Callback<void()>(this, &SonicRangeFinder::finished));

    // delay first measurement
    timeout.attach(Callback<void()>(this, &SonicRangeFinder::startMeasurement), 0.00001);
};

void SonicRangeFinder::startMeasurement() {
    timeout.detach();

    // send start condition (10us high on trigger pin, 0.000002 results in approx 10us)
    triggerPin.write(1);
    timeout.attach(Callback<void()>(this, &SonicRangeFinder::startMeasurementFinished), 0.000002);
    // testPin.write(1);
}

void SonicRangeFinder::startMeasurementFinished() {
    // testPin.write(0);
    triggerPin.write(0);
    timeout.detach();
    timeout.attach(Callback<void()>(this, &SonicRangeFinder::measurementTimedOut), 1.0);
}

void SonicRangeFinder::startClock() {
    // testPin.write(1);
    timer.start();
}

void SonicRangeFinder::finished() {
    // testPin.write(0);
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
