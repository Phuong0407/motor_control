#define NOLOADED_RUN

#include "../include/motor/motor.hpp"

#include <wiringPi.h>
#include <stdio.h>

int main() {
    startEncoders();
    setMotorPWM(0xff, 0xff, 0xff);
    printf("Motor started\n");
    printf("%d\t%d\t%d\n", counter1, counter2, counter3);
    delay(10000);
    printf("%d\t%d\t%d\n", counter1, counter2, counter3);
    return 0;
}
