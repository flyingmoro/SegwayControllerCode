#ifndef CONFIG_H
#define CONFIG_H

#define ARDUINO_SERIAL

// must have parameters
#define loopCycleTimeUs                            2000
#define CHANNELS_AVAILABLE_COUNT                     12
#define CHANNELS_REQUESTED_COUNT                      3
#define CHANNELS_UNREQUESTED_COUNT                    9
#define PARAMETER_COUNT                               2
#define SPECIAL_COMMANDS_COUNT                        2
#define BAUD_RATE                                115200
#define INT_TYPE                                      1
#define FLOAT_TYPE                                    2

// All requested channels
#define mrBeta                                   (messageOutBuffer.channels[0])
#define mrSpeed                                  (messageOutBuffer.channels[1])
#define mrE                                      (messageOutBuffer.channels[2])

// All unrequested channels
#define mrWorldX                                 (unrequestedChannels[0])
#define mrWorldY                                 (unrequestedChannels[1])
#define mrWorldGamma                             (unrequestedChannels[2])
#define mrAlphaDeriv                             (unrequestedChannels[3])
#define mrBetaDeriv                              (unrequestedChannels[4])
#define mrGammaDeriv                             (unrequestedChannels[5])
#define mrAX                                     (unrequestedChannels[6])
#define mrAY                                     (unrequestedChannels[7])
#define mrAZ                                     (unrequestedChannels[8])

// all parameters
#define mR_k_pid                                 (parameters[0]).valueFloat
#define mR_tv                                    (parameters[1]).valueFloat

// all special parameters
#define loopCycleTimeExceededByUs                (specialCommands[0])
#define serialTransmissionLag                    (specialCommands[1])


void microRayInit();
void microRayCommunicate();


#include <stdint.h>


typedef struct MessageIn
{
    int32_t parameterNumber;
    union {
        int32_t parameterValueInt;
        float parameterValueFloat;
    };
} MessageIn;

typedef struct MessageOut
{
    uint32_t loopStartTime;
#if !defined(SUPPRESS_PARAM_CONFIRMATION)
    uint32_t parameterNumber;
    union {
        int32_t parameterValueInt;
        float parameterValueFloat;
    };
#endif
    float channels[CHANNELS_REQUESTED_COUNT];
} MessageOut;

extern MessageOut messageOutBuffer;



typedef struct Parameter {
    uint8_t dataType;
    union {
        int32_t valueInt;
        float valueFloat;
    };
} Parameter;

extern Parameter parameters[PARAMETER_COUNT];
extern float specialCommands[SPECIAL_COMMANDS_COUNT];


// storage for unrequested channels
// requested channels are stored in messageOutBuffer
extern float unrequestedChannels[CHANNELS_UNREQUESTED_COUNT];
#endif