#include "controllers.h"
#include "microRay.h"


#define DELTA_T 0.001

TargetValues currentTargetPoints;

float eOldSpeed = 0.0;
float speedIntegral = 0.0;
float eOldBeta = 0.0;

#define POSITION_MODE_FORWARD 1
#define POSITION_MODE_FORWARD_BACKWARD 2
#define VELOCITY_MODE 3
#define TILT_MODE 4

int setPOSITION_MODE_FORWARD = 0;
int setPOSITION_MODE_FORWARD_BACKWARD = 0;
int setVELOCITY_MODE = 0;
int setROTATING_VELOCITY_MODE = 0;
float additionalSpeedDueDistance = 0.0;
float additionalGammaPDueGamma = 0.0;
float gammaPTarget = 0.0;
float speedTarget = 0.0;

void updateControlTargets(SensorDataCollection * sensorData, TargetValues * targets) {

    switch(controlMode)
    {
        case POSITION_MODE_FORWARD:
            setPOSITION_MODE_FORWARD = 1;
            setPOSITION_MODE_FORWARD_BACKWARD = 0;
            setVELOCITY_MODE = 1;
            setROTATING_VELOCITY_MODE = 1;
            break;
        case POSITION_MODE_FORWARD_BACKWARD:
            setPOSITION_MODE_FORWARD = 0;
            setPOSITION_MODE_FORWARD_BACKWARD = 1;
            setVELOCITY_MODE = 1;
            setROTATING_VELOCITY_MODE = 1;
            break;
        case VELOCITY_MODE:
            setPOSITION_MODE_FORWARD = 0;
            setPOSITION_MODE_FORWARD_BACKWARD = 0;
            setVELOCITY_MODE = 1;
            setROTATING_VELOCITY_MODE = 1;
            break;
        case TILT_MODE:
            setPOSITION_MODE_FORWARD = 0;
            setPOSITION_MODE_FORWARD_BACKWARD = 0;
            setVELOCITY_MODE = 0;
            setROTATING_VELOCITY_MODE = 0;
            break;
    }

    //position control
    float dx1 = cos(sensorData->gamma)*(xSetPoint -sensorData->x) + sin(sensorData->gamma)*(ySetPoint - sensorData->y);
    float dy1 = -sin(sensorData->gamma)*(xSetPoint -sensorData->x) + cos(sensorData->gamma)*(ySetPoint - sensorData->y);
    float dgamma1 = atan2(dy1, dx1);
    float distance = dx1*dx1+dy1*dy1;

    if(setPOSITION_MODE_FORWARD) // moving forward only
    {
        if( distance > deathZoneRadius)
        {
            if(abs(dgamma1)< 2.96) // ~170°
            {
                additionalSpeedDueDistance = distance*kPidDistance;
                if(additionalSpeedDueDistance > limitAdditionalSpeedDueDistance)
                {
                    additionalSpeedDueDistance = 0.5;
                }
            }
            else{
                additionalSpeedDueDistance = 0;
            }
            additionalGammaPDueGamma = dgamma1*kPidGamma;
            if(additionalGammaPDueGamma > limitAdditionalGammaPDueGamma)
            {
                additionalGammaPDueGamma = limitAdditionalGammaPDueGamma;
            }
            if(additionalGammaPDueGamma < (-limitAdditionalGammaPDueGamma))
            {
                additionalGammaPDueGamma = -limitAdditionalGammaPDueGamma;
            }
        }
        else{
            additionalSpeedDueDistance = 0;
            additionalGammaPDueGamma = 0;
        }
    }
    else if(setPOSITION_MODE_FORWARD_BACKWARD) // moving forward and backward
    {
        if((dgamma1> -M_PI/2) && (dgamma1 < M_PI/2)) //forward
        {
            if( distance > deathZoneRadius)
            {
                if(abs(dgamma1)< 1.48) // ~85°
                {
                    additionalSpeedDueDistance = distance*kPidDistance;
                    if(additionalSpeedDueDistance > limitAdditionalSpeedDueDistance)
                    {
                        additionalSpeedDueDistance = 0.5;
                    }
                }
                else{
                    additionalSpeedDueDistance = 0;
                }
                additionalGammaPDueGamma = dgamma1*kPidGamma;
                if(additionalGammaPDueGamma > limitAdditionalGammaPDueGamma)
                {
                    additionalGammaPDueGamma = limitAdditionalGammaPDueGamma;
                }
                if(additionalGammaPDueGamma < (-limitAdditionalGammaPDueGamma))
                {
                    additionalGammaPDueGamma = -limitAdditionalGammaPDueGamma;
                }
            }
            else{
                additionalSpeedDueDistance = 0;
                additionalGammaPDueGamma = 0;
            }
        }
        else{                                        //backward
            float dgamma1 = atan2(-dy1, -dx1);
            if( distance > deathZoneRadius)
            {
                if(abs(dgamma1)< 1.48) // ~85°
                {
                    additionalSpeedDueDistance = -distance*kPidDistance;
                    if(additionalSpeedDueDistance > limitAdditionalSpeedDueDistance)
                    {
                        additionalSpeedDueDistance = -0.5;
                    }
                }
                else{
                    additionalSpeedDueDistance = 0;
                }
                additionalGammaPDueGamma = dgamma1*kPidGamma;
                if(additionalGammaPDueGamma > limitAdditionalGammaPDueGamma)
                {
                    additionalGammaPDueGamma = limitAdditionalGammaPDueGamma;
                }
                if(additionalGammaPDueGamma < (-limitAdditionalGammaPDueGamma))
                {
                    additionalGammaPDueGamma = -limitAdditionalGammaPDueGamma;
                }
            }
            else{
                additionalSpeedDueDistance = 0;
                additionalGammaPDueGamma = 0;
            }
        }
    }
    else{
        additionalSpeedDueDistance = 0;
        additionalGammaPDueGamma = 0;
    }


    // velocity control
    if(setVELOCITY_MODE)
    {
        float eSpeed = speedSetPoint + additionalSpeedDueDistance - sensorData->speed;
        speedTarget = kPidSpeed * (eSpeed + tgSpeed * speedIntegral);
        eOldSpeed = eSpeed;
        speedIntegral += eSpeed;
    }
    else {
        float speedTarget = 0;
    }

    // tilt control
    float eBeta = speedTarget - sensorData->beta;
    float betaTarget = kPidBeta * (eBeta + (eBeta - eOldBeta) * tvBeta / DELTA_T);
    eOldBeta = eBeta;

    // rotating velocity control
    if(setROTATING_VELOCITY_MODE)
    {
        float eGammaP = gammaPSetPoint + additionalGammaPDueGamma - sensorData->gammaP;
        gammaPTarget = kPidGammaP * eGammaP;
    }
    else{
        gammaPTarget = 0;
    }

    // noninteracting control calculation
    // add output of velocity control and tilt controllers
    float sumOfBetaTarget = speedTarget + betaTarget;

    // make two current targets out of a beta target and a gamma target
    targets->motorZero = 0.5 * sumOfBetaTarget - gammaPTarget;
    targets->motorOne = 0.5 * sumOfBetaTarget + gammaPTarget;

    if ((sensorData->beta > 1.3) || (sensorData->beta < -1.3)) {
        targets->motorZero = 0.0;
        targets->motorOne = 0.0;
    }
}
