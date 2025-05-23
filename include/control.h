#ifndef CONTROL_H
#define CONTROL_H

#include "robot.h"
#include "pid.h"
#include "timing.h"
#include "motor.h"

#include <stdio.h>
#include <cmath>
#include <algorithm>

void * controlMotor1(void *arg) {
    int64_t prev_ticks1 = 0, curr_ticks1 = 0;
    while(!TERMINATE_PROGRAM) {
        pthread_mutex_lock(&THROTTLE_MUTEX);
        bool THROTTLE = THROTTLE_MODE;
        pthread_mutex_unlock(&THROTTLE_MUTEX);

        if (THROTTLE) {
            delay(100);
            continue;
        }

        prev_ticks1 = counter1;
        delay(100);
        curr_ticks1 = counter1;
        measured1   = static_cast<double>(prev_ticks1 - curr_ticks1) / 0.1;

        double LOWER = ref1 - MIN_ERROR_TPS;
        double UPPER = ref1 + MIN_ERROR_TPS;
        double err1  = ref1 - measured1;

        if (measured1 >= LOWER && measured1 <= UPPER) err1 = 0.0;

        if (std::abs(err1) > 1e-6) computed1 = computePID1();
        setMotor1();
    }
    return nullptr;
}



void * controlMotor2(void *arg) {
    int64_t prev_ticks2 = 0, curr_ticks2 = 0;
    while(!TERMINATE_PROGRAM) {
        pthread_mutex_lock(&THROTTLE_MUTEX);
        bool THROTTLE = THROTTLE_MODE;
        pthread_mutex_unlock(&THROTTLE_MUTEX);

        if (THROTTLE) {
            delay(100);
            continue;
        }

        prev_ticks2 = counter2;
        delay(100);
        curr_ticks2 = counter2;
        measured2   = static_cast<double>(curr_ticks2 - prev_ticks2) / 0.1;

        double LOWER = ref2 - MIN_ERROR_TPS;
        double UPPER = ref2 + MIN_ERROR_TPS;
        double err2  = ref2 - measured2;

        if (measured2 >= LOWER && measured2 <= UPPER) err2 = 0.0;

        if (std::abs(err2) > 1e-6) computed2 = computePID2();
        setMotor2();
    }
    return nullptr;
}



void * controlMotor3(void *arg) {
    int64_t prev_ticks3 = 0, curr_ticks3 = 0;
    while(!TERMINATE_PROGRAM) {
        pthread_mutex_lock(&THROTTLE_MUTEX);
        bool THROTTLE = THROTTLE_MODE;
        pthread_mutex_unlock(&THROTTLE_MUTEX);

        if (THROTTLE) {
            delay(100);
            continue;
        }

        prev_ticks3 = counter3;
        delay(100);
        curr_ticks3 = counter3;
        measured3   = static_cast<double>(prev_ticks3 - curr_ticks3) / 0.1;

        double LOWER = ref3 - MIN_ERROR_TPS;
        double UPPER = ref3 + MIN_ERROR_TPS;
        double err3  = ref3 - measured3;

        if (measured3 >= LOWER && measured3 <= UPPER) err3 = 0.0;

        if (std::abs(err3) > 1e-6) computed3 = computePID3();
        setMotor3();
    }
    return nullptr;
}



void turnLeftFullThrottle() {
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0xffff);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, 0x09);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0xffff);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, 0x09);
}

void turnRightFullThrottle() {
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0xff00);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, 0x06);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0xffff);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, 0x06);
}

void * overcomeStuckState(void *arg) {
    delay(4000);
    while (!TERMINATE_PROGRAM) {
        volatile int64_t prev_ticks1 = counter1;
        volatile int64_t prev_ticks2 = counter2;
        volatile int64_t prev_ticks3 = counter3;
        delay(100);
        volatile int64_t curr_ticks1 = counter1;
        volatile int64_t curr_ticks2 = counter2;
        volatile int64_t curr_ticks3 = counter3;
        double prev1 = static_cast<double>(curr_ticks1 - prev_ticks1) / 0.1;
        double prev2 = static_cast<double>(curr_ticks2 - prev_ticks2) / 0.1;
        double prev3 = static_cast<double>(curr_ticks3 - prev_ticks3) / 0.1;
        delay(500);

        prev_ticks1 = counter1;
        prev_ticks2 = counter2;
        prev_ticks3 = counter3;
        delay(100);
        curr_ticks1 = counter1;
        curr_ticks2 = counter2;
        curr_ticks3 = counter3;

        double curr1 = static_cast<double>(curr_ticks1 - prev_ticks1) / 0.1;
        double curr2 = static_cast<double>(curr_ticks2 - prev_ticks2) / 0.1;
        double curr3 = static_cast<double>(curr_ticks3 - prev_ticks3) / 0.1;


        if (std::abs(prev1) <= STUCK_THRES && std::abs(curr1) <= STUCK_THRES &&
            std::abs(prev2) <= STUCK_THRES && std::abs(curr2) <= STUCK_THRES &&
            std::abs(prev3) <= STUCK_THRES && std::abs(curr3) <= STUCK_THRES) {

            pthread_mutex_lock(&THROTTLE_MUTEX);
            THROTTLE_MODE = true;
            pthread_mutex_unlock(&THROTTLE_MUTEX);

            if (TURN_LEFT)
                turnLeftFullThrottle();
            else if (TURN_RIGHT)
                turnRightFullThrottle();
            delay(100);
        } else {
            pthread_mutex_lock(&THROTTLE_MUTEX);
            THROTTLE_MODE = false;
            pthread_mutex_unlock(&THROTTLE_MUTEX);
        }
    }
    return nullptr;
}

void * handleNoLineFound(void *arg) {
    delay(4000);
    while (!TERMINATE_PROGRAM) {
        pthread_mutex_lock(&VISION_MUTEX);
        bool NO_LINE = !CONTAIN_LINE;
        pthread_mutex_unlock(&VISION_MUTEX);

        if (NO_LINE) {
            printf("[INFO] NO LINE FOUND. MOTORS STOPPED.\n");
            TERMINATE_PROGRAM = true;
            return nullptr;
        }
    }
    return nullptr;
}

void * monitorMotorsSpeed(void *arg) {
    while(!TERMINATE_PROGRAM) {
        printf("\n");
        if (THROTTLE_MODE && TURN_LEFT)
            printf("[INFO] TURN LEFT FULL THROTTLE MODE.\n");
        if (THROTTLE_MODE && TURN_RIGHT)
            printf("[INFO] TURN RIGHT FULL THROTTLE MODE.\n");

        printf("Motor 1 Speed (ticks/s):\tref\t=\t%.3f\tmeasured\t=\t%.3f\tcomputed=%.3f\n", ref1, measured1, computed1);
        printf("Motor 2 Speed (ticks/s):\tref\t=\t%.3f\tmeasured\t=\t%.3f\tcomputed=%.3f\n", ref2, measured2, computed2);
        printf("Motor 3 Speed (ticks/s):\tref\t=\t%.3f\tmeasured\t=\t%.3f\tcomputed=%.3f\n", ref3, measured3, computed3);
        printf("\n");
        delay(1000);
    }
    return nullptr;
}

#endif // CONTROL_H