#ifndef CONFIG_H
#define CONFIG_H

#define ARDUINO_SERIAL

// must have parameters
#define loopCycleTimeUs                            2000
#define CHANNELS_AVAILABLE_COUNT                     14
#define CHANNELS_REQUESTED_COUNT                     14
#define CHANNELS_UNREQUESTED_COUNT                    0
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
#define mrAlphaDeriv                             (messageOutBuffer.channels[3])
#define mrBetaDeriv                              (messageOutBuffer.channels[4])
#define mrGammaDeriv                             (messageOutBuffer.channels[5])
#define mrAX                                     (messageOutBuffer.channels[6])
#define mrAY                                     (messageOutBuffer.channels[7])
#define mrAZ                                     (messageOutBuffer.channels[8])
#define mrBetaA                                  (messageOutBuffer.channels[9])
#define mrBetaPS                                 (messageOutBuffer.channels[10])
#define mrBetaFiltered                           (messageOutBuffer.channels[11])
#define mrE                                      (messageOutBuffer.channels[12])
#define mrSpeed                                  (messageOutBuffer.channels[13])

// All unrequested channels

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