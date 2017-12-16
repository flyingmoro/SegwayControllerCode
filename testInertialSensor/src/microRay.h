#ifndef CONFIG_H
#define CONFIG_H

#define MBED_OS_SERIAL

// must have parameters
#define loopCycleTimeUs                            5000
#define CHANNELS_AVAILABLE_COUNT                      6
#define CHANNELS_REQUESTED_COUNT                      6
#define CHANNELS_UNREQUESTED_COUNT                    0
#define PARAMETER_COUNT                               1
#define SPECIAL_COMMANDS_COUNT                        3
#define BAUD_RATE                                115200
#define INT_TYPE                                      1
#define FLOAT_TYPE                                    2
#define RECORD_BUFFER_LENGTH                          1
#define PAUSE_AFTER_RECORD                            0

// All requested channels
#define mr_acc_x                                 (messageOutBuffer.channels[0])
#define mr_acc_y                                 (messageOutBuffer.channels[1])
#define mr_acc_z                                 (messageOutBuffer.channels[2])
#define mr_gyro_x                                (messageOutBuffer.channels[3])
#define mr_gyro_y                                (messageOutBuffer.channels[4])
#define mr_gyro_z                                (messageOutBuffer.channels[5])

// All unrequested channels

// all parameters
#define mr_Param1                                (parameters[0]).valueFloat

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