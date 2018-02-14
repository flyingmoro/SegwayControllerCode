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





//
//
// class SonicRangeFinder {
//     public:
//         SonicRangeFinder(PinName, PinName, UltraSonicRanges *, float);
//         void updateRanges();
//     private:
//         void triggerMeasurement();
//         void forwardLeftStartClock();
//         void forwardLeftFinished();
//         UltraSonicRanges * _rangesPtr;
//         bool _forwardLeftMeasurementInProgress = false;
//         Ticker measurementStartTicker;
//         Timer forwardLeftTimer;
//         InterruptIn forwardLeftEchoPin;
//         DigitalOut forwardLeftTriggerPin;
//         float _measurementTiming;
// };
//
// SonicRangeFinder::SonicRangeFinder(PinName echoPinName,
//                                    PinName triggerPinName,
//                                    UltraSonicRanges *ranges,
//                                    float measurementTimingInS) :
//                                    forwardLeftEchoPin(echoPinName),
//                                    forwardLeftTriggerPin(triggerPinName),
//                                    _rangesPtr(rangesPtr),
//                                    measurementStartTicker(),
//                                    _measurementTiming(measurementTimingInS) {
//
//     forwardLeftEchoPin.rise(&forwardLeftStartClock);
//     forwardLeftEchoPin.fall(&forwardLeftFinished);
//     measurementStartTicker.attach(&triggerMeasurement, _measurementTiming);
// };
//
// void SonicRangeFinder::triggerMeasurement() {
//     if (forwardLeftMeasurementInProgress == false) {
//         forwardLeftMeasurementInProgress = true;
//         forwardLeftTriggerPin = 1;
//         wait(0.00001);
//         forwardLeftTriggerPin = 0;
//     }
// }
//
// void SonicRangeFinder::forwardLeftStartClock() {
//     forwardLeftTimer.start();
// }
//
// void SonicRangeFinder::forwardLeftFinished() {
//     rangesPtr->forwardLeft = (uint32_t)forwardLeftTimer.read_us();
//     rangesPtr->forwardLeftMM = rangesPtr->forwardLeft / 5.82;
//     forwardLeftTimer.stop();
//     forwardLeftTimer.reset();
//     forwardLeftMeasurementInProgress = false;
// }












// class SonicRangeFinder {
//     public:
//         SonicRangeFinder(PinName echoPinName,
//                          PinName triggerPinName,
//                          UltraSonicRanges *rangesPtr,
//                          float measurementTimingInS) :
//                          _echoPinName(echoPinName),
//                          _triggerPinName(triggerPinName),
//                          _rangesPtr(rangesPtr),
//                          _measurementTiming(measurementTimingInS) {
//              forwardLeftEchoPin.rise(&forwardLeftStartClock);
//              forwardLeftEchoPin.fall(&forwardLeftFinished);
//              measurementStartTicker.attach(&triggerMeasurement, _measurementTiming);
//         }
//         void updateRanges();
//     private:
//         void triggerMeasurement();
//         void forwardLeftStartClock();
//         void forwardLeftFinished();
//         PinName _echoPinName;
//         PinName _triggerPinName;
//         UltraSonicRanges * _rangesPtr;
//         float _measurementTiming;
//         bool _forwardLeftMeasurementInProgress = false;
//         UltraSonicRanges *rangesPtr;
//         Ticker measurementStartTicker;
//         Timer forwardLeftTimer;
//         InterruptIn forwardLeftEchoPin;
//         DigitalOut forwardLeftTriggerPin;
// };
//
// // SonicRangeFinder::SonicRangeFinder(PinName echoPinName,
// //                                    PinName triggerPinName,
// //                                    UltraSonicRanges *ranges,
// //                                    float measurementTimingInS) {
// //
// //
// //
// //     // InterruptIn forwardLeftEchoPin(echoPinName);
// //     // DigitalOut forwardLeftTriggerPin(triggerPinName);
// //
// //     forwardLeftEchoPin.rise(&forwardLeftStartClock);
// //     forwardLeftEchoPin.fall(&forwardLeftFinished);
// //     measurementStartTicker.attach(&triggerMeasurement, _measurementTiming);
// // };
//
// void SonicRangeFinder::triggerMeasurement() {
//     if (forwardLeftMeasurementInProgress == false) {
//         forwardLeftMeasurementInProgress = true;
//         forwardLeftTriggerPin = 1;
//         wait(0.00001);
//         forwardLeftTriggerPin = 0;
//     }
// }
//
// void SonicRangeFinder::forwardLeftStartClock() {
//     forwardLeftTimer.start();
// }
//
// void SonicRangeFinder::forwardLeftFinished() {
//     rangesPtr->forwardLeft = (uint32_t)forwardLeftTimer.read_us();
//     rangesPtr->forwardLeftMM = rangesPtr->forwardLeft / 5.82;
//     forwardLeftTimer.stop();
//     forwardLeftTimer.reset();
//     forwardLeftMeasurementInProgress = false;
// }
