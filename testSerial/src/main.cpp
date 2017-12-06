#include <mbed.h>

Serial pc(USBTX, USBRX);

int main() {
    while(1) {
        pc.printf("Hallo");
        wait(0.5);
    }
}
