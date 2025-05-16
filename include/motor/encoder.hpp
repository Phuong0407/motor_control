#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "config.h"

#include <cmath>
#include <memory>
#include <vector>
#include <stdio.h>
#include <inttypes.h>

inline void initEncoder(int i2c_addr, int H1, int H2, int &i2c_fd) {
    wiringPiSetup();
    wiringPiI2CSetup(i2c_addr);
    pinMode(H1, INPUT); pullUpDnControl(H1, PUD_UP);
    pinMode(H2, INPUT); pullUpDnControl(H2, PUD_UP);
}

volatile int64_t counter1 = 0;
volatile int64_t counter2 = 0;
volatile int64_t counter3 = 0;

void updateCounter1() { digitalRead(MOTOR1_H2) ? ++counter1 : --counter1; }
void updateCounter2() { digitalRead(MOTOR2_H2) ? ++counter2 : --counter2; }
void updateCounter3() { digitalRead(MOTOR3_H2) ? ++counter3 : --counter3; }

void startEncoders() {
    initEncoder(encoder_addr1, MOTOR1_H1, MOTOR1_H2, i2c_fd1);
    initEncoder(encoder_addr1, MOTOR2_H1, MOTOR2_H2, i2c_fd1);
    initEncoder(encoder_addr2, MOTOR3_H1, MOTOR3_H2, i2c_fd2);
    wiringPiISR(MOTOR1_H1, INT_EDGE_RISING, updateCounter1);
    wiringPiISR(MOTOR2_H1, INT_EDGE_RISING, updateCounter2);
    wiringPiISR(MOTOR3_H1, INT_EDGE_RISING, updateCounter3);
}

inline void clearEncoders() {
    counter1 = 0;
    counter2 = 0;
    counter3 = 0;
}

inline void measureAngularVelocity(double &omega1, double &omega2, double &omega3) {
    int delay_ms = static_cast<int>(smpl_itv * 1000);
    int64_t prev_ticks1, prev_ticks2, prev_ticks3;
    int64_t curr_ticks1, curr_ticks2, curr_ticks3;

    prev_ticks1 = counter1;
    prev_ticks2 = counter2;
    prev_ticks3 = counter3;
    delay(delay_ms);
    curr_ticks1 = counter1;
    curr_ticks2 = counter2;
    curr_ticks3 = counter3;

    omega1 = static_cast<double>(prev_ticks1 - curr_ticks1) / smpl_itv / COUNTER_PER_REV;
    omega2 = static_cast<double>(curr_ticks2 - prev_ticks2) / smpl_itv / COUNTER_PER_REV;
    omega3 = static_cast<double>(curr_ticks3 - prev_ticks3) / smpl_itv / COUNTER_PER_REV;
}

inline void measureAngularVelocity12(double &omega1, double &omega2) {
    int delay_ms = static_cast<int>(smpl_itv * 1000);
    int64_t prev_ticks1, prev_ticks2;
    int64_t curr_ticks1, curr_ticks2;

    prev_ticks1 = counter1;
    prev_ticks2 = counter2;
    delay(delay_ms);
    curr_ticks1 = counter1;
    curr_ticks2 = counter2;

    omega1 = static_cast<double>(prev_ticks1 - curr_ticks1) / smpl_itv / COUNTER_PER_REV;
    omega2 = static_cast<double>(curr_ticks2 - prev_ticks2) / smpl_itv / COUNTER_PER_REV;
}

inline void measureAngularVelocity3(double &omega3) {
    int delay_ms = static_cast<int>(smpl_itv * 1000);
    int64_t prev_ticks3, curr_ticks3;

    prev_ticks3 = counter3;
    delay(delay_ms);
    curr_ticks3 = counter3;

    omega3 = static_cast<double>(curr_ticks3 - prev_ticks3) / smpl_itv / COUNTER_PER_REV;
}

#endif // ENCODER_HPP