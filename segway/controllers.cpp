#include "controllers.h"
#include "microRay.h"

#define DELTA_T 0.001

TargetValues currentTargetPoints;

#define OFF_MODE 0
#define TILT_MODE 1
#define VELOCITY_MODE 2
#define TURNING_MODE 3
#define POSITION_MODE_FORWARD 4
#define POSITION_MODE_FORWARD_AND_BACKWARD 5
#define POSITION_MODE_VECTOR 6


#define maxFilter 100
#define VELOCITY_CONTROL_INTEGRATOR_LIMIT 0.5
#define VELOCITY_ESPEED_LIMIT 1

float eOldSpeed = 0.0;
float speedIntegral = 0.0;
float newIntegral = 0.0;
float eOldBeta = 0.0;
float beta_buffer[maxFilter] = {0};
float speed_buffer[maxFilter] = {0};
float gammaP_buffer[maxFilter] = {0};
float beta_filter = 0.0;
float speed_filter = 0.0;
float gammaP_filter = 0.0;
float betaTarget = 0.0f;
float additionalSpeedDueDistance = 0.0;
float additionalGammaPDueGamma = 0.0;
float gammaPTarget = 0.0;
float speedTarget = 0.0;

int beta_Pointer = 0;
int speed_Pointer = 0;
int gammaP_Pointer = 0;
int direction = 0;



void updateControlTargets(SensorDataCollection * sensorData, TargetValues * targets) {


    // clear all controller outputs
    betaTarget = 0.0f;
    speedTarget = 0.0f;
    additionalSpeedDueDistance = 0.0f;
    additionalGammaPDueGamma = 0.0f;
    gammaPTarget = 0.0f;

    // tilt control
    if(mr_controlMode > 0) {
        //filter
        beta_filter = 0;
        if(size_beta_buffer > maxFilter)
        {
            size_beta_buffer = maxFilter;
        }
        beta_buffer[beta_Pointer] = sensorData->beta;
        beta_Pointer = (beta_Pointer+1)% size_beta_buffer;
        for(int i = 0; i < size_beta_buffer; i++){
            beta_filter += beta_buffer[i]/size_beta_buffer;
        }
        //controller
        float eBeta = 0.0 - beta_filter;
        betaTarget = kPidBeta * (eBeta + (eBeta - eOldBeta) * tvBeta / DELTA_T);
        eOldBeta = eBeta;
    }


    //position control
    float dx1 = cos(sensorData->gamma)*(xSetPoint -sensorData->x) + sin(sensorData->gamma)*(ySetPoint - sensorData->y);
    float dy1 = -sin(sensorData->gamma)*(xSetPoint -sensorData->x) + cos(sensorData->gamma)*(ySetPoint - sensorData->y);
    float dgamma1 = atan2(dy1, dx1);
    float distance = sqrt(dx1*dx1+dy1*dy1);


    //moving forward only
    if(mr_controlMode == POSITION_MODE_FORWARD) {
        if( distance > deathZoneRadius) {            
            additionalSpeedDueDistance = distance*kPidDistance;

            if(abs(dgamma1)< 2.96) // direction logging ~170°
            {
                direction = 0;
                additionalGammaPDueGamma = dgamma1 * kPidGamma;
            }
            else{
                if(direction == 0)
                {
                    if(dgamma1 > 0)
                    {
                        direction = -1;
                    }
                    else{
                        direction = 1;
                    }
                }
                additionalGammaPDueGamma = abs(dgamma1)*direction*kPidGamma;
            }
        }
    }

    // moving forward and backward
    if (mr_controlMode == POSITION_MODE_FORWARD_AND_BACKWARD) {

        //forward
        if ((dgamma1> -M_PI/2) && (dgamma1 < M_PI/2)) {
            if (distance > deathZoneRadius) {
                additionalSpeedDueDistance = distance*kPidDistance;
                if (abs(dgamma1)< 1.48) // direction logging ~85°
                {                    
                    direction = 0;
                    additionalGammaPDueGamma = dgamma1 * kPidGamma;
                }
                else{
                    if(direction == 0)
                    {
                        if(dgamma1 > 0)
                        {
                            direction = 1;
                        }
                        else{
                            direction = -1;
                        }
                    }
                    additionalGammaPDueGamma = abs(dgamma1)*direction*kPidGamma;
                }
            }
        }

        //backward
        else {
            dgamma1 = atan2(-dy1, -dx1);
            if (distance > deathZoneRadius) {
                additionalSpeedDueDistance = -distance*kPidDistance;
                if(abs(dgamma1)< 1.48) // direction logging ~85°
                {                    
                    direction = 0;
                    additionalGammaPDueGamma = dgamma1 * kPidGamma;
                }
                else{
                    if(direction == 0)
                    {
                        if(dgamma1 > 0)
                        {
                            direction = 1;
                        }
                        else{
                            direction = -1;
                        }
                    }
                    additionalGammaPDueGamma = abs(dgamma1)*direction*kPidGamma;
                }
            }
        }
    }

    // moving with scalar
    if(mr_controlMode == POSITION_MODE_VECTOR) {
        if( distance > deathZoneRadius) {
            additionalSpeedDueDistance = dx1*kPidDistance;
            additionalGammaPDueGamma = dgamma1 * kPidGamma;
        }
    }

    // limit turn rate addition from position control
    if (additionalGammaPDueGamma > limitAdditionalGammaPDueGamma) {
        additionalGammaPDueGamma = limitAdditionalGammaPDueGamma;
    }
    else if(additionalGammaPDueGamma < (-limitAdditionalGammaPDueGamma)) {
        additionalGammaPDueGamma = -limitAdditionalGammaPDueGamma;
    }

    // limit speed addition from position control
    if(additionalSpeedDueDistance > limitAdditionalSpeedDueDistance) {
            additionalSpeedDueDistance = limitAdditionalSpeedDueDistance;
    }
    else if(additionalSpeedDueDistance < (-limitAdditionalSpeedDueDistance)) {
            additionalSpeedDueDistance = -limitAdditionalSpeedDueDistance;
    }

    // velocity control

    if(mr_controlMode >= VELOCITY_MODE) {
        //filter
        speed_filter = 0;
        if(size_speed_buffer > maxFilter)
        {
            size_speed_buffer = maxFilter;
        }
        speed_buffer[speed_Pointer] = sensorData->speed;
        speed_Pointer = (speed_Pointer+1)% size_speed_buffer;
        for(int i = 0; i < size_speed_buffer; i++){
            speed_filter += (speed_buffer[i]/size_speed_buffer);
        }
        //controller
        float eSpeed = speedSetPoint + additionalSpeedDueDistance - speed_filter;
        newIntegral = speedIntegral + eSpeed * DELTA_T;
        if (abs(newIntegral) < VELOCITY_CONTROL_INTEGRATOR_LIMIT) //limit speedIntegral
        {
            speedIntegral += eSpeed * DELTA_T;
        }

        if(eSpeed > VELOCITY_ESPEED_LIMIT) //limit eSpeed
        {
            eSpeed = VELOCITY_ESPEED_LIMIT;
        }
        else if(eSpeed < (-VELOCITY_ESPEED_LIMIT))
        {
            eSpeed = -VELOCITY_ESPEED_LIMIT;
        }
        speedTarget = kPidSpeed * eSpeed + kPidSpeed * tgSpeed * speedIntegral;        
    }


    // turn rate control


    if(mr_controlMode >= TURNING_MODE) {
        //filter
        gammaP_filter = 0;
        if(size_gammaP_buffer > maxFilter)
        {
            size_gammaP_buffer = maxFilter;
        }
        gammaP_buffer[gammaP_Pointer] = sensorData->gammaP;
        gammaP_Pointer = (gammaP_Pointer+1)% size_gammaP_buffer;
        for(int i = 0; i < size_gammaP_buffer; i++){
            gammaP_filter += gammaP_buffer[i]/size_gammaP_buffer;
        }
        //controller
        float eGammaP = gammaPSetPoint + additionalGammaPDueGamma - gammaP_filter;
        gammaPTarget = kPidGammaP * eGammaP;
    }


    // noninteracting control calculation
    // sum up output of velocity control and tilt controllers
    float sumOfBetaTarget = speedTarget + betaTarget;

    // make two current targets out of a beta target and a gamma target
    targets->motorZero = 0.5 * sumOfBetaTarget - gammaPTarget;
    targets->motorOne = 0.5 * sumOfBetaTarget + gammaPTarget;

    // for debugging
    mr_speedFilter = speed_filter;
    mr_betaFilter = beta_filter*57.296;
    mr_dx1 = dx1;
    mr_dy1 = dy1;
    mr_dgamma1 = dgamma1;
    mr_distance = distance;
    mr_speedIntegralPart = kPidSpeed * tgSpeed * speedIntegral;
    mr_gammaPFilter = gammaP_filter;
    mr_controllerGammaPTarget = gammaPTarget;
    mr_controllerBetaTarget = betaTarget;
    mr_controllerSpeedTarget = speedTarget;
}
