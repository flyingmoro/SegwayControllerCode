#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "mbed.h"

typedef struct sensorDataCollection {
    float speed;
    float beta;
    float gammaP;
} SensorDataCollection;


typedef struct targetValues {
    float motorZero;
    float motorOne;
} TargetValues;

void updateControlTargets(SensorDataCollection * sensorData, TargetValues * targets);



#endif
