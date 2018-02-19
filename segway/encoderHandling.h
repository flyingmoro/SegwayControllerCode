#ifndef ENCODERHANDLING_H
#define ENCODERHANDLING_H

#include <stdint.h>

typedef struct position {
    uint16_t encLeft;
    uint16_t encRight;
    float x;
    float y;
    float gamma;
} Position;

#define METERS_PER_ENCODER_STEP (0.000327*0.125)
#define WHEEL_SPAN 0.184  // Radabstand in metern
#define PI 3.1415927
#define NEG_PI -3.141
#define TWO_PI 6.282

void initEncoder();
void updatePosition(Position *worldPosition);






#endif
