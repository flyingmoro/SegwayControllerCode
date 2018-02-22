#ifndef CONFIG_H
#define CONFIG_H

#define MBED_OS_UDP

// must have parameters
#define loopCycleTimeUs                            1000
#define CHANNELS_AVAILABLE_COUNT                     22
#define CHANNELS_REQUESTED_COUNT                     22
#define CHANNELS_UNREQUESTED_COUNT                    0
#define PARAMETER_COUNT                              10
#define SPECIAL_COMMANDS_COUNT                        3
#define BAUD_RATE                                115200
#define INT_TYPE                                      1
#define FLOAT_TYPE                                    2
#define RECORD_BUFFER_LENGTH                          1
#define PAUSE_AFTER_RECORD                            0
#define MESSAGE_SKIP_MODE                             0

// All requested channels
#define mr_pulsi                                 (messageOutBuffer.channels[0])
#define mr_sonic                                 (messageOutBuffer.channels[1])
#define mr_rawAccX                               (messageOutBuffer.channels[2])
#define mr_rawAccY                               (messageOutBuffer.channels[3])
#define mr_rawAccZ                               (messageOutBuffer.channels[4])
#define mr_rawDegPX                              (messageOutBuffer.channels[5])
#define mr_rawDegPY                              (messageOutBuffer.channels[6])
#define mr_rawDegPZ                              (messageOutBuffer.channels[7])
#define mr_betaRaw                               (messageOutBuffer.channels[8])
#define mr_betaTangens                           (messageOutBuffer.channels[9])
#define mr_betaRawIntegral                       (messageOutBuffer.channels[10])
#define mr_betaComplementary                     (messageOutBuffer.channels[11])
#define mr_betaKalman                            (messageOutBuffer.channels[12])
#define mr_dutyCycleTime                         (messageOutBuffer.channels[13])
#define mr_worldX                                (messageOutBuffer.channels[14])
#define mr_worldY                                (messageOutBuffer.channels[15])
#define mr_worldGamma                            (messageOutBuffer.channels[16])
#define mr_encoderLeftWheel                      (messageOutBuffer.channels[17])
#define mr_encoderRightWheel                     (messageOutBuffer.channels[18])
#define mr_controllerOutputMotorZero             (messageOutBuffer.channels[19])
#define mr_controllerOutputMotorOne              (messageOutBuffer.channels[20])
#define mr_debugTimer                            (messageOutBuffer.channels[21])

// All unrequested channels

// all parameters
#define mr_currentMotorZero                      (parameters[0]).valueFloat
#define mr_currentMotorOne                       (parameters[1]).valueFloat
#define speedSetPoint                            (parameters[2]).valueFloat
#define gammaPSetPoint                           (parameters[3]).valueFloat
#define kPidSpeed                                (parameters[4]).valueFloat
#define tgSpeed                                  (parameters[5]).valueFloat
#define kPidBeta                                 (parameters[6]).valueFloat
#define tvBeta                                   (parameters[7]).valueFloat
#define kPidGammaP                               (parameters[8]).valueFloat
#define mr_letTheControllerControl               (parameters[9]).valueInt

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
    uint16_t statusFlags;
    uint16_t parameterNumber;
#if !defined(SUPPRESS_PARAM_CONFIRMATION)
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
#define WAIT_MODE 4

#define STATUS_BAD_DATA 0
#define STATUS_SKIPPED 1
#endif