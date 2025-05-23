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
    while(true) {

        pthread_mutex_lock(&INTERACTION_MUTEX);
        while (INTERACTION_MODE) {
            pthread_cond_wait(&CONTROL_COND, &INTERACTION_MUTEX);
        }
        pthread_mutex_unlock(&INTERACTION_MUTEX);

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
    while(true) {

        pthread_mutex_lock(&INTERACTION_MUTEX);
        while (INTERACTION_MODE) {
            pthread_cond_wait(&CONTROL_COND, &INTERACTION_MUTEX);
        }
        pthread_mutex_unlock(&INTERACTION_MUTEX);

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

        double LOWER = ref1 - MIN_ERROR_TPS;
        double UPPER = ref1 + MIN_ERROR_TPS;
        double err2  = ref2 - measured2;

        if (measured1 >= LOWER && measured1 <= UPPER) err2 = 0.0;

        if (std::abs(err2) > 1e-6) computed2 = computePID2();
        setMotor2();
    }
    return nullptr;
}



void * controlMotor3(void *arg) {
    int64_t prev_ticks3 = 0, curr_ticks3 = 0;
    while(true) {

        pthread_mutex_lock(&INTERACTION_MUTEX);
        while (INTERACTION_MODE) {
            pthread_cond_wait(&CONTROL_COND, &INTERACTION_MUTEX);
        }
        pthread_mutex_unlock(&INTERACTION_MUTEX);

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

        double LOWER = ref1 - MIN_ERROR_TPS;
        double UPPER = ref1 + MIN_ERROR_TPS;
        double err3  = ref3 - measured3;

        if (measured1 >= LOWER && measured1 <= UPPER) err3 = 0.0;

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
    microsleep(100);
}

void turnRightFullThrottle() {
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0xff00);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, 0x06);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0xffff);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, 0x06);
    microsleep(100);
}

void stopAllMotors () {
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0x0000);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0x0000);
}

void * overcomeStuckState(void *arg) {
    delay(4000);
    while (true) {
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

            if (TURN_LEFT) {
                turnLeftFullThrottle();
                printf("[INFO] TURN LEFT FULL THROTTLE MODE.\n");
            }
            if (TURN_RIGHT) {
                turnRightFullThrottle();
                printf("[INFO] TURN RIGHT FULL THROTTLE MODE.\n");
            }
            delay(100);
        } else {
            pthread_mutex_lock(&THROTTLE_MUTEX);
            THROTTLE_MODE = false;
            pthread_mutex_unlock(&THROTTLE_MUTEX);
        }
    }
    return nullptr;
}

void * processInteractionMode(void *arg) {
    double curr_angle = 0.0, prev_angle = 0.0;
    while (true) {
        pthread_mutex_lock(&INTERACTION_MUTEX);
        bool INTERACTION_MODE_ACTIVE = INTERACTION_MODE;
        pthread_mutex_unlock(&INTERACTION_MUTEX);

        if (INTERACTION_MODE_ACTIVE) {
            printf("[INPUT] ENTER ANGLE: ");
            scanf("%lf", &curr_angle);

            double dev_angle = curr_angle - prev_angle;
            double signal = kp_angle * curr_angle + kd_angle * dev_angle;
            prev_angle = curr_angle;

            int lin_speed   = 200;
            int left_speed  = lin_speed + signal;
            int right_speed = lin_speed - signal;
            setAllMotors(left_speed, right_speed, lin_speed);
            delay(1000);
            if (CONTAIN_LINE) {
                pthread_mutex_lock(&INTERACTION_MUTEX);
                INTERACTION_MODE = false;
                pthread_cond_broadcast(&CONTROL_COND);
                pthread_mutex_unlock(&INTERACTION_MUTEX);
                printf("[INFO] LINE FOUND. RETURNING TO AUTONOMOUS MODE.\n");
            } else {
                char ch;
                printf("[INFO] LINE STILL NOT FOUND. CONTINUE INTERACTION MODE? (y/n): ");
                scanf(" %c", &ch);

                if (ch == 'y' || ch == 'Y') {
                    printf("[INFO] CONTINUING INTERACTION MODE.\n");
                } else {
                    pthread_mutex_lock(&INTERACTION_MUTEX);
                    INTERACTION_MODE = false;
                    pthread_cond_broadcast(&CONTROL_COND);
                    pthread_mutex_unlock(&INTERACTION_MUTEX);
                    stopAllMotors();
                    printf("[INFO] EXITING INTERACTION MODE. MOTORS STOPPED.\n");
                }
            }
        }
    }
    return nullptr;
}

void * handleNoLineFound(void *arg) {
    delay(4000);
    while (true) {
        if (!CONTAIN_LINE) {
            pthread_mutex_lock(&INTERACTION_MUTEX);

            printf("[INFO] NO LINE FOUND.\n");
            printf("[INFO] USER HAVE TO NAVIGATE MANUALLY IN INTERACTION MODE. ENTER INTERACTION MODE? (y/n): \n");

            char ch = 0;
            scanf(" %c", &ch);
            if (ch == 'y' || ch == 'Y') {
                INTERACTION_MODE = true;
                printf("[INFO] INTERACTION MODE ACTIVATED.\n");
            }
        } else {
            pthread_mutex_lock(&INTERACTION_MUTEX);
            INTERACTION_MODE = false;
            printf("[INFO] INTERACTION MODE NOT ACTIVATED. MOTORS STOPPED.\n");
            pthread_cond_broadcast(&CONTROL_COND);
            pthread_mutex_unlock(&INTERACTION_MUTEX);
        }
        delay(100);
    }
    return nullptr;
}

void * monitorMotorsSpeed(void *arg) {
    while(true) {
        printf("\n");
        if (THROTTLE_MODE && TURN_LEFT)
            printf("[INFO] TURN LEFT FULL THROTTLE MODE.\n");
        if (THROTTLE_MODE && TURN_RIGHT)
            printf("[INFO] TURN RIGHT FULL THROTTLE MODE.\n");

        printf("Motor 1 Speed (ticks/s):\tref\t=\t%.3f\tmeasured\t=\t%.3f\tcomputed=%.3f\n", ref1, measured1, computed1);
        printf("Motor 2 Speed (ticks/s):\tref\t=\t%.3f\tmeasured\t=\t%.3f\tcomputed=%.3f\n", ref2, measured2, computed2);
        printf("Motor 3 Speed (ticks/s):\tref\t=\t%.3f\tmeasured\t=\t%.3f\tcomputed=%.3f\n", ref3, measured3, computed3);
        printf("\n");
        microsleep(1000000);
    }
    return nullptr;
}

#endif // CONTROL_H