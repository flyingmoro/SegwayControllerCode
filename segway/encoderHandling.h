#ifndef ENCODERHANDLING_H
#define ENCODERHANDLING_H

#include <stdint.h>



typedef struct position {
    uint16_t encLeft;
    uint16_t encRight;
    float x;
    float y;
    float forwardSpeed;
    float gamma;
    float gammaP;
} Position;

// #define ENCODER_COUNT 8192
#define ENCODER_COUNT 65535
#define METERS_PER_ENCODER_STEP (0.000327*0.125/1.09) //1.09 correction term
#define WHEEL_SPAN 0.184  // Radabstand in metern
#define PI 3.1415927
#define NEG_PI -3.141
#define TWO_PI 6.282

#define DELTA_T 0.001

void initEncoder();
void updatePosition(Position *worldPosition);






#endif
