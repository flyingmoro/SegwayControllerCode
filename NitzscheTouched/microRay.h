#ifndef CONFIG_H
#define CONFIG_H

#define ARDUINO_SERIAL

// must have parameters
#define loopCycleTimeUs                            2000
#define CHANNELS_AVAILABLE_COUNT                     10
#define CHANNELS_REQUESTED_COUNT                      4
#define CHANNELS_UNREQUESTED_COUNT                    6
#define PARAMETER_COUNT                               2
#define SPECIAL_COMMANDS_COUNT                        2
#define BAUD_RATE                                115200
#define INT_TYPE                                      1
#define FLOAT_TYPE                                    2

// All requested channels
#define mrWorldX                                 (messageOutBuffer.channels[0])
#define mrWorldY                                 (messageOutBuffer.channels[1])
#define mrWorldGamma                             (messageOutBuffer.channels[2])
#define mrGamma                                  (messageOutBuffer.channels[3])

// All unrequested channels
#define mrAlphaDeriv                             (unrequestedChannels[0])
#define mrBetaDeriv                              (unrequestedChannels[1])
#define mrGammaDeriv                             (unrequestedChannels[2])
#define mrAX                                     (unrequestedChannels[3])
#define mrAY                                     (unrequestedChannels[4])
#define mrAZ                                     (unrequestedChannels[5])

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