#include "mbed.h"

class Counter {
public:
    Counter(PinName pin) : _interrupt(pin) {        // create the InterruptIn on the pin specified to Counter
        _interrupt.rise(callback(this, &Counter::increment)); // attach increment function of this counter instance
    }

    void increment() {
        _count++;
    }

    int read() {
        return _count;
    }

private:
    InterruptIn _interrupt;
    volatile int _count;
};

Counter counter(USER_BUTTON);
Serial pc(USBTX, USBRX, 115200);

int main() {
    while(1) {
        pc.printf("Count so far: %d\n", counter.read());
        wait(2);
    }
}
