#include "controllers.h"
#include "microRay.h"


#define DELTA_T 0.001

TargetValues currentTargetPoints;

float eOldSpeed = 0.0;
float speedIntegral = 0.0;
float eOldBeta = 0.0;


void updateControlTargets(SensorDataCollection * sensorData, TargetValues * targets) {

    // velocity control
    float eSpeed = speedSetPoint - sensorData->speed;
    float speedTarget = kPidSpeed * (eSpeed + (eSpeed - eOldSpeed) + tgSpeed * speedIntegral);
    eOldSpeed = eSpeed;
    speedIntegral += eSpeed;

    // beta control
    float eBeta = speedTarget - sensorData->beta;
    float betaTarget = kPidBeta * (eBeta + (eBeta - eOldBeta) * tvBeta / DELTA_T);
    eOldBeta = eBeta;

    // gammaP control
    float eGammaP = gammaPSetPoint - sensorData->gammaP;
    float gammaPTarget = kPidGammaP * eGammaP;

    // noninteracting control calculation
    // add output of velocity control and beta controllers
    float sumOfBetaTarget = speedTarget + betaTarget;

    // make two current targets out of a beta target and a gamma target
    targets->motorZero = 0.5 * sumOfBetaTarget - gammaPTarget;
    targets->motorOne = 0.5 * sumOfBetaTarget + gammaPTarget;
}
