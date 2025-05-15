#define NOLOADED_RUN
#define EACH_ENCODER_HPP

// #define THREE_ENCODER_HPP

#include "../include/motor/motor.hpp"

#include <wiringPi.h>
#include <stdio.h>

#ifdef THREE_ENCODER_HPP
int main() {
    startEncoders();
    setMotorPWM(0xff, 0x00, 0x00);
    printf("Motor started\n");
    printf("%d\t%d\t%d\n", counter1, counter2, counter3);
    delay(25000);
    stopMotors();

    printf("%" PRId64 "\t%" PRId64 "\t%" PRId64 "\n", counter1, counter2, counter3);

    return 0;
}
#endif

#ifdef EACH_ENCODER_HPP

#include <wiringPiI2C.h>
#include <inttypes.h>

volatile int64_t counter = 0;

int H1 = 27;
int H2 = 0;

void update() {
    if (digitalRead(H2))
        counter++;
    else 
        counter--;
}

int main() {
    wiringPiSetup();
    wiringPiI2CSetup(0x0f);
    pinMode(H1, INPUT);
    pullUpDnControl(H1, PUD_UP);
    pinMode(H2, INPUT);
    pullUpDnControl(H2, PUD_UP);
    wiringPiISR(H1, INT_EDGE_RISING, update);

    delay(5000);

    printf("Final Counter Value: %" PRId64 "\n", counter);
    return 0;
}

#endif
