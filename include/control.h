#ifndef CONTROL_H
#define CONTROL_H

#include "robot.h"
#include "pid.h"
#include "timing.h"

#include <stdio.h>
#include <cmath>
#include <algorithm>

void * controlMotor1(void *arg) {
    int64_t prev_ticks1 = 0, curr_ticks1 = 0;
    while(true) {
        prev_ticks1 = counter1;
        delay(100);
        curr_ticks1 = counter1;
        measured1   = static_cast<double>(prev_ticks1 - curr_ticks1) / 0.1;

        double LOWER = ref1 - MIN_ERROR_TPS;
        double UPPER = ref1 + MIN_ERROR_TPS;
        double err1  = ref1 - measured1;

        if (measured1 >= LOWER && measured1 <= UPPER) err1 = 0.0;

        if (std::abs(err1) > 1e-6) computed1 = compute1();
    }
}

void * controlMotor2(void *arg) {
    int64_t prev_ticks2 = 0, curr_ticks2 = 0;
    while(true) {
        prev_ticks2 = counter2;
        delay(100);
        curr_ticks2 = counter2;
        measured2   = static_cast<double>(curr_ticks2 - prev_ticks2) / 0.1;

        double LOWER = ref1 - MIN_ERROR_TPS;
        double UPPER = ref1 + MIN_ERROR_TPS;
        double err2  = ref2 - measured2;

        if (measured1 >= LOWER && measured1 <= UPPER) err2 = 0.0;

        if (std::abs(err2) > 1e-6) computed2 = compute2();
    }
}

void * controlMotor3(void *arg) {
    int64_t prev_ticks3 = 0, curr_ticks3 = 0;
    while(true) {
        prev_ticks3 = counter3;
        delay(100);
        curr_ticks3 = counter3;
        measured3   = static_cast<double>(prev_ticks3 - curr_ticks3) / 0.1;

        double LOWER = ref1 - MIN_ERROR_TPS;
        double UPPER = ref1 + MIN_ERROR_TPS;
        double err3  = ref3 - measured3;

        if (measured1 >= LOWER && measured1 <= UPPER) err3 = 0.0;

        if (std::abs(err3) > 1e-6) computed3 = compute();
    }
}

void * monitorMotorsSpeed(void *arg) {
    while(true) {
        printf("Motor 1 Speed (ticks/s):\tref\t=\t%.3f\tmeasured\t=\t%.3f\tcomputed=%.3f\n", ref1, measured1, computed1);
        printf("Motor 2 Speed (ticks/s):\tref\t=\t%.3f\tmeasured\t=\t%.3f\tcomputed=%.3f\n", ref2, measured2, computed2);
        printf("Motor 3 Speed (ticks/s):\tref\t=\t%.3f\tmeasured\t=\t%.3f\tcomputed=%.3f\n", ref3, measured3, computed3);
        microsleep(1000000);
    }
}

#endif // CONTROL_H