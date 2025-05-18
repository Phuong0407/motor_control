#ifndef ENCODER_H
#define ENCODER_H

#include "robot.h"

void updateCounter1() { digitalRead(MOTOR1_H2) ? ++counter1 : --counter1; }
void updateCounter2() { digitalRead(MOTOR2_H2) ? ++counter2 : --counter2; }
void updateCounter3() { digitalRead(MOTOR3_H2) ? ++counter3 : --counter3; }

void startEncoders() {
    wiringPiSetup();
    i2c_fd1 = wiringPiI2CSetup(0x0f);
    i2c_fd2 = wiringPiI2CSetup(0x0d);
    
    pinMode(MOTOR1_H1, INPUT); pullUpDnControl(MOTOR1_H1, PUD_UP);
    pinMode(MOTOR2_H1, INPUT); pullUpDnControl(MOTOR2_H2, PUD_UP);
    pinMode(MOTOR3_H1, INPUT); pullUpDnControl(MOTOR3_H2, PUD_UP);

    wiringPiISR(MOTOR1_H1, INT_EDGE_RISING, updateCounter1);
    wiringPiISR(MOTOR2_H1, INT_EDGE_RISING, updateCounter2);
    wiringPiISR(MOTOR3_H1, INT_EDGE_RISING, updateCounter3);
}

#endif // ENCODER_H