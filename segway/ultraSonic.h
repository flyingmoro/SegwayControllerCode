#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <mbed.h>
#include <stdint.h>


typedef struct ultraSonicRanges {
    uint32_t forwardLeft;
    float forwardLeftMM;
    uint32_t forwardRight;
    uint32_t backwardLeft;
    uint32_t backwardRight;
} UltraSonicRanges;



void initUltraSonic(UltraSonicRanges *ranges);
// void getUltraSonsicRanges(UltraSonicRanges *ranges);






#endif
