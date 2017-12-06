#include <mbed.h>

#define PWM_DUTY_CYCLE 0.0001f

void firstStep();
void secondStep();
void thirdStep();
void fourthStep();

DigitalOut phase_U_Led(LED1);
DigitalOut phase_V_Led(LED2);
DigitalOut phase_W_Led(LED3);

DigitalOut phase_U(PG_9);
DigitalOut phase_V(PG_14);
DigitalOut phase_W(PF_15);

// PortOut uvwPort(Port1, 0b00000000);

Ticker masterTicker;
Timeout doSecondStep;
Timeout doThirdStep;
Timeout doFourthStep;

volatile float startOfPeriodTwo = 0.0f;
volatile float startOfPeriodThree = 0.0f;
volatile float startOfPeriodFour = 0.0f;

int main() {
    masterTicker.attach(&firstStep, PWM_DUTY_CYCLE);

    while(1) {
        startOfPeriodTwo = 0.2f * PWM_DUTY_CYCLE;
        startOfPeriodThree = 0.5f * PWM_DUTY_CYCLE;
        startOfPeriodFour = 0.7f * PWM_DUTY_CYCLE;
        wait(2.0f);
        // lengthOfPeriodOne = 0.3f;
        // lengthOfPeriodTwo = 0.1f;
        // lengthOfPeriodThree = 0.5f;
        // wait(2.0f);
    }
}


void firstStep() {
    phase_U = 1;
    phase_V = 0;
    phase_W = 0;

    phase_U_Led = 1;
    phase_V_Led = 0;
    phase_W_Led = 0;

    doSecondStep.attach(&secondStep, startOfPeriodTwo);
    doThirdStep.attach(&thirdStep, startOfPeriodThree);
    doFourthStep.attach(&fourthStep, startOfPeriodFour);

}

void secondStep() {
    phase_U = 0;
    phase_V = 0;
    phase_W = 0;

    phase_U_Led = 0;
    phase_V_Led = 0;
    phase_W_Led = 0;
}

void thirdStep() {
    phase_U = 1;
    phase_V = 1;
    phase_W = 0;

    phase_U_Led = 1;
    phase_V_Led = 1;
    phase_W_Led = 0;
}

void fourthStep() {
    phase_U = 0;
    phase_V = 0;
    phase_W = 0;

    phase_U_Led = 0;
    phase_V_Led = 0;
    phase_W_Led = 0;
}
