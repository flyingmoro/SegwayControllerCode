#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "mbed.h"

typedef struct sensorDataCollection {
    float x;
    float y;
    float gamma;
    float speed;
    float gammaP;
    float beta;
} SensorDataCollection;


typedef struct targetValues {
    float motorZero;
    float motorOne;
} TargetValues;

void updateControlTargets(SensorDataCollection * sensorData, TargetValues * targets);



#endif
