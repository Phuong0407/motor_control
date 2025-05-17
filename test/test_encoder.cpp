#define NOLOADED_RUN
#define EACH_ENCODER_HPP

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <inttypes.h>
#include <stdio.h>

volatile int64_t counter = 0;

int H1 = 3;
int H2 = 4;

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