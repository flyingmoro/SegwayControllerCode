#include "controllers.h"
#include "microRay.h"


#define DELTA_T 0.001

TargetValues currentTargetPoints;

float eOldSpeed = 0.0;
float speedIntegral = 0.0;
float eOldBeta = 0.0;

#define OFF_MODE 0
#define TILT_MODE 1
#define VELOCITY_MODE 2
#define POSITION_MODE_FORWARD 3
#define POSITION_MODE_FORWARD_AND_BACKWARD 4

#define TURNING_MODE 1


#define VELOCITY_CONTROL_INTEGRATOR_LIMIT 100.0

float betaTarget = 0.0f;
float additionalSpeedDueDistance = 0.0;
float additionalGammaPDueGamma = 0.0;
float gammaPTarget = 0.0;
float speedTarget = 0.0;

void updateControlTargets(SensorDataCollection * sensorData, TargetValues * targets) {

    // clear all controller outputs
    betaTarget = 0.0f;
    speedTarget = 0.0f;
    additionalSpeedDueDistance = 0.0f;
    additionalGammaPDueGamma = 0.0f;
    gammaPTarget = 0.0f;


    // tilt control
    if(mr_controlModeStraight > 0) {
        float eBeta = 0.0 - sensorData->beta;
        betaTarget = kPidBeta * (eBeta + (eBeta - eOldBeta) * tvBeta / DELTA_T);
        eOldBeta = eBeta;
    }




    //position control
    float dx1 = cos(sensorData->gamma)*(xSetPoint -sensorData->x) + sin(sensorData->gamma)*(ySetPoint - sensorData->y);
    float dy1 = -sin(sensorData->gamma)*(xSetPoint -sensorData->x) + cos(sensorData->gamma)*(ySetPoint - sensorData->y);
    float dgamma1 = atan2(dy1, dx1);
    float distance = dx1*dx1+dy1*dy1;

    // moving forward only
    if(mr_controlModeStraight >= POSITION_MODE_FORWARD) {
        if( distance > deathZoneRadius) {
            if(abs(dgamma1)< 2.96) {
                // ~170°
                additionalSpeedDueDistance = distance*kPidDistance;
            }
            additionalGammaPDueGamma = dgamma1*kPidGamma;
        }
    }

    // moving forward and backward
    if (mr_controlModeStraight >= POSITION_MODE_FORWARD_AND_BACKWARD) {

        //forward (overwrites values from POSITION_MODE_FORWARD)
        if ((dgamma1> -M_PI/2) && (dgamma1 < M_PI/2)) {
            if (distance > deathZoneRadius) {
                if (abs(dgamma1)< 1.48) {
                    // ~85°
                    additionalSpeedDueDistance = distance*kPidDistance;
                }
                additionalGammaPDueGamma = dgamma1 * kPidGamma;
            }
        }

        //backward (overwrites values from POSITION_MODE_FORWARD)
        else {
            float dgamma1 = atan2(-dy1, -dx1);
            if (distance > deathZoneRadius) {
                // ~85°
                if(abs(dgamma1)< 1.48) {
                    additionalSpeedDueDistance = -distance*kPidDistance;
                }
                additionalGammaPDueGamma = dgamma1*kPidGamma;
            }
        }
    }

    // limit turn rate addition from position control
    if (additionalGammaPDueGamma > limitAdditionalGammaPDueGamma) {
        additionalGammaPDueGamma = limitAdditionalGammaPDueGamma;
    }
    if (additionalGammaPDueGamma < (-limitAdditionalGammaPDueGamma)) {
        additionalGammaPDueGamma = -limitAdditionalGammaPDueGamma;
    }

    // limit speed addition from position control
    if(additionalSpeedDueDistance > limitAdditionalSpeedDueDistance) {
            additionalSpeedDueDistance = limitAdditionalSpeedDueDistance;
    }
    if(additionalSpeedDueDistance < (-limitAdditionalSpeedDueDistance)) {
            additionalSpeedDueDistance = -limitAdditionalSpeedDueDistance;
    }

    // velocity control
    if(mr_controlModeStraight >= VELOCITY_MODE) {
        float eSpeed = speedSetPoint + additionalSpeedDueDistance - sensorData->speed;
        if (abs(speedIntegral + eSpeed) < VELOCITY_CONTROL_INTEGRATOR_LIMIT) {
            speedIntegral += eSpeed * DELTA_T;
        }
        // speedTarget = kPidSpeed * (eSpeed + tgSpeed * speedIntegral);
        speedTarget = kPidSpeed * eSpeed + kPidSpeed * tgSpeed * speedIntegral;
        eOldSpeed = eSpeed;
    }


    // turn rate control
    if(mr_controlModeTurning >= TURNING_MODE) {
        float eGammaP = gammaPSetPoint + additionalGammaPDueGamma - sensorData->gammaP;
        gammaPTarget = kPidGammaP * eGammaP;
    }



    // noninteracting control calculation
    // sum up output of velocity control and tilt controllers
    float sumOfBetaTarget = speedTarget + betaTarget;

    // make two current targets out of a beta target and a gamma target
    targets->motorZero = 0.5 * sumOfBetaTarget - gammaPTarget;
    targets->motorOne = 0.5 * sumOfBetaTarget + gammaPTarget;

    // for debugging
    mr_controllerBetaTarget = betaTarget;
    mr_controllerSpeedTarget = speedTarget;
}
