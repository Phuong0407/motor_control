#include <wiringPi.h>
#include <stdio.h>

volatile int count = 0;

void testISR() {
    count++;
    printf("Interrupt Triggered! Count: %d\n", count);
}

int main() {
    wiringPiSetup();
    pinMode(27, INPUT);
    pullUpDnControl(27, PUD_UP);
    wiringPiISR(27, INT_EDGE_RISING, &testISR);

    while (1) {
        delay(500);
    }

    return 0;
}