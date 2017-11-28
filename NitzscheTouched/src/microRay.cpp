#include "microRay.h"

void prepareOutMessage();
void prepareInMessage();
void sendMessage();
void receiveMessage();
void recordMessage();

unsigned long lastTime = 0;
bool lastMessageSendComplete = false;

bool record = false;
bool transmitRecordBuffer = false;
int recordingCounter = 0;
int recordingSendCounter = 0;


// storage for unrequested channels
float unrequestedChannels[CHANNELS_UNREQUESTED_COUNT];

#if !defined(SUPPRESS_PARAM_CONFIRMATION)
int parameterSendCounter = 0;
#endif

int receivedBytesCount = 0;
int sendBytesCount = 0;

MessageOut messageOutBuffer;
MessageIn messageInBuffer;

MessageOut recordBuffer[RECORD_BUFFER_LENGTH];

void prepareOutMessage(unsigned long loopStartTime)
{
    // map rotating parameters
    // on each cycle, only one of the "controlled parameters" is send to the pc

    messageOutBuffer.loopStartTime = loopStartTime;

    #if !defined(SUPPRESS_PARAM_CONFIRMATION)
    messageOutBuffer.parameterNumber = parameterSendCounter;
    if (parameterSendCounter < 0) {
        messageOutBuffer.parameterValueFloat = specialCommands[(parameterSendCounter + 1) * -1];
    }
    else {
        switch (parameters[parameterSendCounter].dataType) {
            case INT_TYPE:
                messageOutBuffer.parameterValueInt = parameters[parameterSendCounter].valueInt;
                break;
            case FLOAT_TYPE:
                messageOutBuffer.parameterValueFloat = parameters[parameterSendCounter].valueFloat;
                break;
            default:
                break;
        }
    }

    // increment the counter for sending the "slow parameters"
    parameterSendCounter += 1;
    if (parameterSendCounter >= PARAMETER_COUNT)
    {
        parameterSendCounter = SPECIAL_COMMANDS_COUNT * -1;
    }
#endif
}

void prepareInMessage() {
    if (messageInBuffer.parameterNumber >= 0) {
        switch (parameters[messageInBuffer.parameterNumber].dataType) {
            case INT_TYPE:
                parameters[messageInBuffer.parameterNumber].valueInt = messageInBuffer.parameterValueInt;
                break;
            case FLOAT_TYPE:
                parameters[messageInBuffer.parameterNumber].valueFloat = messageInBuffer.parameterValueFloat;
                break;
            default:
                break;
        }
    }
    else {
        if (messageInBuffer.parameterNumber == -3) {
            if (messageInBuffer.parameterValueFloat != specialCommands[(messageInBuffer.parameterNumber + 1) * -1]) {
                if (messageInBuffer.parameterValueFloat > 0.5) {
                    record = true;
                }
                else {
                    record = false;
                    transmitRecordBuffer = true;
                }
            }
        }
        specialCommands[(messageInBuffer.parameterNumber + 1) * -1] = messageInBuffer.parameterValueFloat;
    }
}

void microRayCommunicate()
{
    receiveMessage();

#ifndef mrDEBUG

    if (record) {
        recordMessage();
        recordBuffer[recordingCounter] = messageOutBuffer;
        recordingCounter += 1;
        if (recordingCounter > RECORD_BUFFER_LENGTH) {
            recordingCounter = 0;
        }
    }
    else if (transmitRecordBuffer) {
        // blocks until finished
        transmitRecordBuffer = false;

        for (recordingSendCounter = 0; recordingSendCounter < RECORD_BUFFER_LENGTH; recordingSendCounter++) {
            int nextMessageIndex = recordingSendCounter + recordingCounter;
            if (nextMessageIndex > RECORD_BUFFER_LENGTH){
                nextMessageIndex -= RECORD_BUFFER_LENGTH;
            }
            messageOutBuffer = recordBuffer[nextMessageIndex];
            sendMessage();
            while (!lastMessageSendComplete) {
                // wait
            }
        }
        recordingCounter = 0;
    }
    else {
        sendMessage();
    }

#endif
}

Parameter parameters[] = {
    { 2, { .valueFloat = 1.0f} },
    { 2, { .valueFloat = 0.065f} }
};

float specialCommands[] = {
    0.0f,
    0.0f,
    0.0f
};


#include <arduino.h>

void receiveMessage();
void readIncomingBytesIntoBuffer();
void appendByteToBuffer(uint8_t inByte);
void shiftGivenPositionToBufferStart(int position);
int seekForFullMessage();
void extractMessage(int messageStartPosition);


#define OUT_START_BYTE (char)7
#define OUT_STOP_BYTE (char)8

#define IN_MESSAGE_SIZE 8
#define IN_BUFFER_SIZE ((IN_MESSAGE_SIZE+2)*2)

#define IN_START_BYTE (char)7
#define IN_STOP_BYTE (char)8

int serialBufferSize = 0;
void microRayInit() {
    Serial.begin(BAUD_RATE);
    serialBufferSize = Serial.availableForWrite();
}


void sendMessage() {
    if (!transmitRecordBuffer) {
        prepareOutMessage(micros());
    }

    // check, if there is still data not send from the previous loop
    #ifdef SERIAL_TX_BUFFER_SIZE
        int bytesStillInBuffer = SERIAL_TX_BUFFER_SIZE - Serial.availableForWrite();
        if (bytesStillInBuffer > 1) {
            serialTransmissionLag = bytesStillInBuffer;
        }
    #endif

    lastMessageSendComplete = false;
    Serial.write(7);
    Serial.write((byte *)&messageOutBuffer, sizeof(messageOutBuffer));
    Serial.write(8);

    // wait for message to complete during recording transfer
    if (transmitRecordBuffer) {
        while (Serial.availableForWrite() < serialBufferSize) {
            // wait
        }
        lastMessageSendComplete = true;
    }
}


void recordMessage() {
    prepareOutMessage(micros());
}




uint8_t rawMessageInBuffer[IN_BUFFER_SIZE];
uint8_t rawMessageInBufferTemp[IN_BUFFER_SIZE];
int16_t bufferPosition = 0;

void receiveMessage() {
    readIncomingBytesIntoBuffer();
    int foundMessageStartPosition = seekForFullMessage();

    if(foundMessageStartPosition > -1) {
        extractMessage(foundMessageStartPosition);
        prepareInMessage();
    }
}

void readIncomingBytesIntoBuffer() {
    while (Serial.available()) {
        appendByteToBuffer(Serial.read());
    }
}

void appendByteToBuffer(uint8_t inByte) {
    // prevent buffer from overfilling
    // shift whole buffer one to the left to free last position
    if(bufferPosition >= IN_BUFFER_SIZE) {
        shiftGivenPositionToBufferStart(1);
    }
    rawMessageInBuffer[bufferPosition] = inByte;
    bufferPosition += 1;
}

void shiftGivenPositionToBufferStart(int position) {
    // copy and shift
    int i;
    for(i = position; i < bufferPosition; i++) {
        rawMessageInBufferTemp[i - position] = rawMessageInBuffer[i];
    }

    // actualize bufferPosition
    bufferPosition = bufferPosition - position;

    // copy back
    for(i = 0; i < bufferPosition; i++) {
        rawMessageInBuffer[i] = rawMessageInBufferTemp[i];
    }
}

int seekForFullMessage() {
    int i;
    for (i = 0; i < bufferPosition - IN_MESSAGE_SIZE; i++) {
        if (rawMessageInBuffer[i] == IN_START_BYTE) {
            int expectedStopBytePosition = i + IN_MESSAGE_SIZE + 1;
            if (rawMessageInBuffer[expectedStopBytePosition] == IN_STOP_BYTE) {
                return i;
            }
        }
    }
    return -1;
}

void extractMessage(int messageStartPosition) {
    memcpy(&messageInBuffer.parameterNumber, &rawMessageInBuffer[messageStartPosition + 1], 4);
    memcpy(&messageInBuffer.parameterValueInt, &rawMessageInBuffer[messageStartPosition + 1 + 4], 4);
    shiftGivenPositionToBufferStart(messageStartPosition + IN_MESSAGE_SIZE + 2);
}
