#ifndef ENCODERHANDLING_H
#define ENCODERHANDLING_H

#include <stdint.h>

typedef struct position {
    uint16_t x;
    uint16_t y;
    float gamma;
} Position;



void initEncoder();
void refreshPosition(Position *worldPosition);






#endif
