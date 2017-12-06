#include <mbed.h>

class ODrive {
        PinName txPin;
        PinName rxPin;
        int baudRate;
        float setTorqueOne;
        float setTorqueTwo;
        Serial odrivePort;
        char torqueCmdOne[11];
        char torqueCmdTwo[11];
        event_callback_t firstWriteEventComplete;
        event_callback_t secondWriteEventComplete;
        static const char torqueCmdTemplate[] = "$c %i %.3f!";
    public:
        Serial(PinName tx, PinName rx) : odrivePort(tx, rx) {
            firstWriteEventComplete = firstTorqueWriteComplete;
            secondWriteEventComplete = secondTorqueWriteComplete
            // torqueCmdTemplate = "$c %i %.3f!";
        }
        void setTorqueMotorOne(float);
        void setTorqueMotorTwo(float);
        void setTorqueBothMotors(float, float);
    private:
        void firstTorqueWriteComplete(int event);
        void secondTorqueWriteComplete(int event);
};

// ODrive::ODrive () {
//     txPin = PG_14;
//     rxPin = PG_9;
//     baudRate = 115200;
// }

// ODrive::ODrive (PinName tx, PinName rx) {
//     txPin = tx;
//     rxPin = rx;
//     baudRate = 115200;
//
// }

ODrive::ODrive (PinName tx, PinName rx, int baud) {
    // txPin = tx;
    // rxPin = rx;
    odrivePort.baud(baud);
}

void ODrive::setTorqueMotorOne(float torque) {
    sprintf(torqueCmdOne, "$c %i %.3f!", 0, setTorqueOne);
    odrivePort.write((uint8_t *)torqueCmdOne, sizeof(torqueCmdOne), firstWriteEventComplete, SERIAL_EVENT_TX_COMPLETE);
}

void ODrive::setTorqueMotorTwo(float torque) {
    sprintf(torqueCmdOne, "$c %i %.3f!", 1, setTorqueTwo);
    odrivePort.write((uint8_t *)torqueCmdOne, sizeof(torqueCmdOne), firstWriteEventComplete, SERIAL_EVENT_TX_COMPLETE);
}

void ODrive::firstTorqueWriteComplete(int event) {
    sprintf(torqueCmdTwo, "$c %i %.3f!", 0, setTorqueTwo);
    odrivePort.write((uint8_t *)torqueCmdTwo, sizeof(torqueCmdTwo), secondWriteEventComplete, SERIAL_EVENT_TX_COMPLETE);
}

void ODrive::secondTorqueWriteComplete(int event) {
}
