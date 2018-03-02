#ifndef CONFIG_H
#define CONFIG_H

#define MBED_OS_UDP

// must have parameters
#define loopCycleTimeUs                            1000
#define CHANNELS_AVAILABLE_COUNT                     34
#define CHANNELS_REQUESTED_COUNT                     10
#define CHANNELS_UNREQUESTED_COUNT                   24
#define PARAMETER_COUNT                              24
#define SPECIAL_COMMANDS_COUNT                        3
#define BAUD_RATE                                115200
#define INT_TYPE                                      1
#define FLOAT_TYPE                                    2
#define RECORD_BUFFER_LENGTH                          1
#define PAUSE_AFTER_RECORD                            0
#define MESSAGE_SKIP_MODE                             0

// All requested channels
#define mr_worldX                                (messageOutBuffer.channels[0])
#define mr_worldY                                (messageOutBuffer.channels[1])
#define mr_worldGamma                            (messageOutBuffer.channels[2])
#define mr_speed                                 (messageOutBuffer.channels[3])
#define mr_speedFilter                           (messageOutBuffer.channels[4])
#define mr_gammaPFilter                          (messageOutBuffer.channels[5])
#define mr_dx1                                   (messageOutBuffer.channels[6])
#define mr_dy1                                   (messageOutBuffer.channels[7])
#define mr_dgamma1                               (messageOutBuffer.channels[8])
#define mr_distance                              (messageOutBuffer.channels[9])

// All unrequested channels
#define mr_pulsi                                 (unrequestedChannels[0])
#define mr_sonic                                 (unrequestedChannels[1])
#define mr_rawAccX                               (unrequestedChannels[2])
#define mr_rawAccY                               (unrequestedChannels[3])
#define mr_rawAccZ                               (unrequestedChannels[4])
#define mr_rawDegPX                              (unrequestedChannels[5])
#define mr_rawDegPY                              (unrequestedChannels[6])
#define mr_rawDegPZ                              (unrequestedChannels[7])
#define mr_betaRaw                               (unrequestedChannels[8])
#define mr_betaTangens                           (unrequestedChannels[9])
#define mr_betaRawIntegral                       (unrequestedChannels[10])
#define mr_betaComplementary                     (unrequestedChannels[11])
#define mr_betaKalman                            (unrequestedChannels[12])
#define mr_dutyCycleTime                         (unrequestedChannels[13])
#define mr_encoderLeftWheel                      (unrequestedChannels[14])
#define mr_encoderRightWheel                     (unrequestedChannels[15])
#define mr_controllerOutputMotorZero             (unrequestedChannels[16])
#define mr_controllerOutputMotorOne              (unrequestedChannels[17])
#define mr_debugTimer                            (unrequestedChannels[18])
#define mr_controllerSpeedTarget                 (unrequestedChannels[19])
#define mr_controllerBetaTarget                  (unrequestedChannels[20])
#define mr_betaFilter                            (unrequestedChannels[21])
#define mr_controllerGammaPTarget                (unrequestedChannels[22])
#define mr_speedIntegralPart                     (unrequestedChannels[23])

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
#define mr_controllerMasterSwitch                (parameters[9]).valueInt
#define xSetPoint                                (parameters[10]).valueFloat
#define ySetPoint                                (parameters[11]).valueFloat
#define kPidDistance                             (parameters[12]).valueFloat
#define kPidGamma                                (parameters[13]).valueFloat
#define deathZoneRadius                          (parameters[14]).valueFloat
#define limitAdditionalSpeedDueDistance          (parameters[15]).valueFloat
#define limitAdditionalGammaPDueGamma            (parameters[16]).valueFloat
#define complementaryRatioRawData                (parameters[17]).valueFloat
#define kalmannQ                                 (parameters[18]).valueFloat
#define mr_controlModeTurning                    (parameters[19]).valueInt
#define mr_controlModeStraight                   (parameters[20]).valueInt
#define size_beta_buffer                         (parameters[21]).valueInt
#define size_speed_buffer                        (parameters[22]).valueInt
#define size_gammaP_buffer                       (parameters[23]).valueInt

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