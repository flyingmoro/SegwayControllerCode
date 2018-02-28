#include "microRay.h"

void prepareOutMessage();
void prepareInMessage();
void sendMessage();
void receiveMessage();
void record();
void recordMessage();
void transmitRecordings();

unsigned long lastTime = 0;
volatile bool lastMessageSendComplete = true;

int sendMode = LIVE_MODE;
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
        messageOutBuffer.parameterValueInt = specialCommands[(parameterSendCounter + 1) * -1];
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
        // toggle recording mode
        if (messageInBuffer.parameterNumber == -3) {
            if (messageInBuffer.parameterValueInt != specialCommands[(messageInBuffer.parameterNumber + 1) * -1]) {
                if (messageInBuffer.parameterValueInt == 1) {
                    sendMode = RECORD_MODE;
                }
                else if (messageInBuffer.parameterValueInt == 0) {
                    sendMode = RECORD_TRANSMISSION_MODE;
                }
            }
        }
        specialCommands[(messageInBuffer.parameterNumber + 1) * -1] = messageInBuffer.parameterValueInt;
    }
}

void microRayCommunicate()
{
    receiveMessage();

#ifndef mrDEBUG
    switch (sendMode) {
        case RECORD_MODE:
            record();
            break;
        case RECORD_TRANSMISSION_MODE:
            transmitRecordings();
            break;
        case LIVE_MODE:
            sendMessage();
            break;
        case WAIT_MODE:
            break;
        default:
            break;
    }
#endif
}

void record() {
    recordMessage();
    recordBuffer[recordingCounter] = messageOutBuffer;
    recordingCounter += 1;
    if (recordingCounter > RECORD_BUFFER_LENGTH) {
        recordingCounter = 0;
    }
}

void transmitRecordings() {
    // blocks until finished
    for (recordingSendCounter = 0; recordingSendCounter < RECORD_BUFFER_LENGTH; recordingSendCounter++) {
        int nextMessageIndex = recordingSendCounter + recordingCounter;
        if (nextMessageIndex > RECORD_BUFFER_LENGTH){
            nextMessageIndex -= RECORD_BUFFER_LENGTH;
        }
        messageOutBuffer = recordBuffer[nextMessageIndex];
        sendMessage();
        receiveMessage();
        // while (lastMessageSendComplete == false) {
            // wait
        // }
    }
    recordingCounter = 0;
    sendMode = WAIT_MODE;
}

Parameter parameters[] = {
    { 2, { .valueFloat = 0.0f} },
    { 2, { .valueFloat = 0.0f} },
    { 2, { .valueFloat = 0.0f} },
    { 2, { .valueFloat = 0.0f} },
    { 2, { .valueFloat = -10.0f} },
    { 2, { .valueFloat = 0.3f} },
    { 2, { .valueFloat = 80.0f} },
    { 2, { .valueFloat = 0.07f} },
    { 2, { .valueFloat = 0.0f} },
    { 1, { .valueInt = 1} },
    { 2, { .valueFloat = 0.0f} },
    { 2, { .valueFloat = 0.0f} },
    { 2, { .valueFloat = 0.0f} },
    { 2, { .valueFloat = 0.0f} },
    { 2, { .valueFloat = 0.1f} },
    { 2, { .valueFloat = 0.5f} },
    { 2, { .valueFloat = 3.0f} },
    { 2, { .valueFloat = 0.0010000000475f} },
    { 2, { .valueFloat = 0.0f} },
    { 1, { .valueInt = 0} },
    { 1, { .valueInt = 2} },
    { 1, { .valueInt = 30} },
    { 1, { .valueInt = 10} },
    { 1, { .valueInt = 10} }
};

int specialCommands[] = {
    0,
    0,
    0
};


// #include <mbed.h>
#include <EthernetInterface.h>
#include <SocketAddress.h>

#define SERVER_IP       "192.168.0.133"
#define OWN_IP          "192.168.0.15"
#define PORT            10000
#define NET_MASK        "255.255.0.0"
#define GATEWAY         "192.168.0.1"

EthernetInterface eth;
UDPSocket socket;
SocketAddress udp_server_address(SERVER_IP, PORT);
SocketAddress dump_address(SERVER_IP, PORT);

Timer dutyCycleTimer;
Timer debugTimer;
Serial mRserial(USBTX, USBRX);

// DigitalOut greenLed(LED1);
// DigitalOut blueLed(LED2);
// DigitalOut redLed(LED3);



void microRayInit() {
    messageOutBuffer.statusFlags = 0;
    // Bring up the network interface

    // laut Dokumentation Initialisierung mit static IP eigentlich möglich bei mbed OS, geht aber nicht
    // es gibt wohl einen Branch von mbed mit einem Patch
    //// eth.init(&ownIP, &mask, &Gateway);

    // Ethernetconnection with DHCP
    eth.set_network(OWN_IP, NET_MASK, GATEWAY);
    eth.connect();
    const char *ip = eth.get_ip_address();
    const char *mac = eth.get_mac_address();

    mRserial.printf("IP address is: %s\n", ip ? ip : "No IP");
    mRserial.printf("MAC address is: %s\n", mac ? mac : "No MAC");

    socket.open(&eth);
    socket.bind(PORT);
    socket.set_blocking(false);

    // the timer is used for duty cycle duration measurement
    dutyCycleTimer.reset();
    dutyCycleTimer.start();
}


void sendMessage() {
    if (sendMode == LIVE_MODE) {
        prepareOutMessage((unsigned long)dutyCycleTimer.read_high_resolution_us());
    }

    // send data to the pc via Ethernet with mbed os
    debugTimer.reset();
    debugTimer.start();
    lastMessageSendComplete = false;
    sendBytesCount = socket.sendto(udp_server_address, (char *)&messageOutBuffer, sizeof(messageOutBuffer));
    lastMessageSendComplete = true;
//    SEND_TIMER = (float)debugTimer.read_us();

    // #ifdef DEBUG
    // mRserial.printf("SEND: %d %d %f\n", sendBytesCount, messageOutBuffer.parameterNumber, messageOutBuffer.parameterValue);
    // #endif
}



void receiveMessage() {
    // receive a command
    debugTimer.reset();
    debugTimer.start();
    int receivedBytesCount = (float)socket.recvfrom(&dump_address, (char *)&messageInBuffer, sizeof(messageInBuffer));
//    RECEIVE_TIMER = (float)debugTimer.read_us();

    if (receivedBytesCount > 0) {
        prepareInMessage();
    }

    // // if a command has been received
    // if (RECEIVED_BYTES_COUNT > 0.5f)
    // {
    //
    //     if (messageInBuffer.parameterNumber >= 0) {
    //         parameters[messageInBuffer.parameterNumber] = messageInBuffer.parameterValueInt;
    //     }
    //     else {
    //         specialCommands[(messageInBuffer.parameterNumber + 1) * -1] = messageInBuffer.parameterValueInt;
    //     }
    //     // for debugging
    //     // LAST_COMMAND_ID = messageInBuffer.parameterNumber;
    //     // LAST_COMMAND_VALUE = messageInBuffer.parameterValueInt;
    //     // blue led blinkiblinki on successfull receive
    //     // blueLed = 1;
    // }
    // else
    // {
    //     // red led on until next loop cycle
    //     // redLed = 1;
    // }
    //
    // #ifdef DEBUG
    // mRserial.printf("RECV: %f %d %f\n", RECEIVED_BYTES_COUNT, messageInBuffer.parameterNumber, messageInBuffer.value);
    // #endif
}

void recordMessage() {
    prepareOutMessage((unsigned long)dutyCycleTimer.read_high_resolution_us());
}
