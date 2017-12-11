#ifndef CONFIG_H
#define CONFIG_H

#define ARDUINO_SERIAL

// must have parameters
#define loopCycleTimeUs                            2000
#define CHANNELS_AVAILABLE_COUNT                     14
#define CHANNELS_REQUESTED_COUNT                      3
#define CHANNELS_UNREQUESTED_COUNT                   11
#define PARAMETER_COUNT                               2
#define SPECIAL_COMMANDS_COUNT                        3
#define BAUD_RATE                                115200
#define INT_TYPE                                      1
#define FLOAT_TYPE                                    2
#define RECORD_BUFFER_LENGTH                         54
#define PAUSE_AFTER_RECORD                            0

// All requested channels
#define mrWorldX                                 (messageOutBuffer.channels[0])
#define mrWorldY                                 (messageOutBuffer.channels[1])
#define mrWorldGamma                             (messageOutBuffer.channels[2])

// All unrequested channels
#define mrAlphaDeriv                             (unrequestedChannels[0])
#define mrBetaDeriv                              (unrequestedChannels[1])
#define mrGammaDeriv                             (unrequestedChannels[2])
#define mrAX                                     (unrequestedChannels[3])
#define mrAY                                     (unrequestedChannels[4])
#define mrAZ                                     (unrequestedChannels[5])
#define mrBetaA                                  (unrequestedChannels[6])
#define mrBetaPS                                 (unrequestedChannels[7])
#define mrBetaFiltered                           (unrequestedChannels[8])
#define mrE                                      (unrequestedChannels[9])
#define mrSpeed                                  (unrequestedChannels[10])

// all parameters
#define mR_k_pid                                 (parameters[0]).valueFloat
#define mR_tv                                    (parameters[1]).valueFloat

// all special parameters
#define loopCycleTimeExceededByUs                (specialCommands[0])
#define serialTransmissionLag                    (specialCommands[1])
#define mrRecordModeEnable                       (specialCommands[2])


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
extern int specialCommands[SPECIAL_COMMANDS_COUNT];


// storage for unrequested channels
// requested channels are stored in messageOutBuffer
extern float unrequestedChannels[CHANNELS_UNREQUESTED_COUNT];

#define RECORD_MODE 0
#define RECORD_TRANSMISSION_MODE 1
#define LIVE_MODE 2
#define RECORD_WAIT_MODE 3
#endif