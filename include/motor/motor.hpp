#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "pid.hpp"

#include <time.h>
#include <errno.h>
#include <stdio.h>

#include <cmath>
#include <algorithm>

#include <inttypes.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

//=======================================================
// TIMING UTILITIES
//=======================================================
void microsleep(int microseconds) {
    struct timespec ts;
    ts.tv_sec = microseconds / 1000000;
    ts.tv_nsec = (microseconds % 1000000) * 1000;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
        continue;
    }
}





//=======================================================
// HARDWARE DEFINITIONS
//=======================================================
static constexpr int    ADDRESS1                = 0x0f;
static constexpr int    ADDRESS2                = 0x0d;
static constexpr int    MOTOR1_H1               = 21;
static constexpr int    MOTOR1_H2               = 22;
static constexpr int    MOTOR2_H1               = 3;
static constexpr int    MOTOR2_H2               = 4;
static constexpr int    MOTOR3_H1               = 27;
static constexpr int    MOTOR3_H2               = 0;
static constexpr int    MAX_PWM                 = 255;
static constexpr int    DEAD_PWM                = 40;


//=======================================================
// THIS IS THE NUMBER OF COUNT FOR 0.1 ms
//=======================================================
#ifdef NOLOAD_RUN
static constexpr double MAX_TICKS                 = 12.0;
#else // LOAD_RUN
static constexpr double MAX_TICKS                 = 9.0;
#endif



static constexpr double SAFETY_OFFSET           = 0.8;
static constexpr double DEADZONE_SCALEUP        = 0.843137254901961;
static constexpr double ERROR_THRESHOLD_PERCENT = 0.10;
static constexpr double MIN_ERROR_TPS           = 11.0;





//=======================================================
// ENCODER HANDLING
//=======================================================
static          int    i2c_fd1      = -1;
static          int    i2c_fd2      = -1;

volatile        int64_t counter1    = 0;
volatile        int64_t counter2    = 0;
volatile        int64_t counter3    = 0;

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





//=======================================================
// MOTOR CONTROL FUNCTIONS
//=======================================================
int computePWMFromUnsignedRPS(double u_rps) {
    double norm_rps = std::clamp(u_rps / MAX_TICKS, 0.0, 1.0);
    int pwm_value = static_cast<int>(MAX_PWM * norm_rps * SAFETY_OFFSET);
    if (pwm_value < DEAD_PWM) return 0;
    return static_cast<int>((pwm_value - DEAD_PWM) / DEADZONE_SCALEUP);
}

inline int computeDirection(int dir) {
    return (dir == +1) ? 0x06 : 0x0a;
}

inline int computeDirection(int dir1, int dir2) {
    if (dir1 == 0) dir1 = +1;
    if (dir2 == 0) dir2 = +1;

    if (dir1 == +1 && dir2 == +1) return 0x06;
    if (dir1 == -1 && dir2 == +1) return 0x0a;
    if (dir1 == +1 && dir2 == -1) return 0x05;
    return 0x09;
}





//=======================================================
// GLOBAL VARIABLES, PARALLEL CONTROL MOTOR THREADS
//=======================================================
int pwm1 = 0, pwm2 = 0, pwm3 = 0;
int dir1 = 0, dir2 = 0, dir3 = 0;
double ref1 = 0.0, ref2 = 0.0, ref3 = 0.0;
double measured1 = 0.0, measured2 = 0.0, measured3 = 0.0;
double computed1 = 0.0, computed2 = 0.0, computed3 = 0.0;
PID pid1, pid2, pid3;





//=======================================================
// MOTOR CONTROL FUNCTIONS
//=======================================================
void setMotor1(double rps1) {
    dir1 = (rps1 > 0) ? +1 : -1;
    pwm1 = computePWMFromUnsignedRPS(std::abs(rps1));
    int xdir12 = computeDirection(dir1, dir2);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, xdir12);
}

void setMotor2(double rps2) {
    dir2 = (rps2 > 0) ? +1 : -1;
    pwm2 = computePWMFromUnsignedRPS(std::abs(rps2));
    int xdir12 = computeDirection(dir1, dir2);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, xdir12);
}

void setMotor3(double rps3) {
    dir3 = (rps3 > 0) ? +1 : -1;
    pwm3 = computePWMFromUnsignedRPS(std::abs(rps3));
    int xdir3 = computeDirection(dir3);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, (pwm3 << 8));
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, xdir3);
}

void controlMotor1(void *arg) {
    int64_t prev_ticks1 = 0, curr_ticks1 = 0;
    while(true) {
        prev_ticks1 = counter1;
        delay(100);
        curr_ticks1 = counter1;
        measured1 = static_cast<double>(prev_ticks1 - curr_ticks1) / 0.1;

        double LOWER = ref1 - MIN_ERROR_TPS;
        double UPPER = ref1 + MIN_ERROR_TPS;
        double err1 = ref1 - measured1;

        if (measured1 >= LOWER && measured1 <= UPPER) err1 = 0.0;

        if (std::abs(err1) > 1e-6) {
            computed1 = pid1.compute(ref1, measured1);
            setMotor1(computed1);
        }
    }
}

void controlMotor2(void *arg) {
    int64_t prev_ticks2 = 0, curr_ticks2 = 0;
    while(true) {
        prev_ticks2 = counter2;
        delay(100);
        curr_ticks2 = counter2;
        measured2 = static_cast<double>(curr_ticks2 - prev_ticks2) / 0.1;

        double LOWER = ref1 - MIN_ERROR_TPS;
        double UPPER = ref1 + MIN_ERROR_TPS;
        double err2 = ref2 - measured2;

        if (measured1 >= LOWER && measured1 <= UPPER) err2 = 0.0;

        if (std::abs(err2) > 1e-6) {
            computed2 = pid2.compute(ref2, measured2);
            setMotor2(computed2);
        }
    }
}

void controlMotor3(void *arg) {
    int64_t prev_ticks3 = 0, curr_ticks3 = 0;
    while(true) {
        prev_ticks3 = counter3;
        delay(100);
        curr_ticks3 = counter3;
        measured3 = static_cast<double>(prev_ticks3 - curr_ticks3) / 0.1;

        double LOWER = ref1 - MIN_ERROR_TPS;
        double UPPER = ref1 + MIN_ERROR_TPS;
        
        double err3 = ref3 - measured3;

        if (measured1 >= LOWER && measured1 <= UPPER) err3 = 0.0;


        if (std::abs(err3) > 1e-6) {
            computed3 = pid3.compute(ref3, measured3);
            setMotor3(computed3);
        }
    }
}





//=======================================================
// MONITORING MOTORS
//=======================================================
void monitorMotorsSpeed(void *arg) {
    while(true) {
        printf("=====================================================\n");
        printf("Motor 1 Speed[ticks/s]:\tref\t=\t%.3f\tmeasured\t=\t%.3f\tcomputed=%.3f\n", ref1, measured1, computed1);
        printf("Motor 2 Speed[ticks/s]:\tref\t=\t%.3f\tmeasured\t=\t%.3f\tcomputed=%.3f\n", ref2, measured2, computed2);
        printf("Motor 3 Speed[ticks/s]:\tref\t=\t%.3f\tmeasured\t=\t%.3f\tcomputed=%.3f\n", ref3, measured3, computed3);
        printf("=====================================================\n");
        microsleep(1000000);
    }
}






















void measureAngularVelocity(double &omega1, double &omega2, double &omega3) {
    int64_t prev_ticks1, prev_ticks2, prev_ticks3;
    int64_t curr_ticks1, curr_ticks2, curr_ticks3;

    prev_ticks1 = counter1; prev_ticks2 = counter2; prev_ticks3 = counter3;
    delay(100);
    curr_ticks1 = counter1; curr_ticks2 = counter2; curr_ticks3 = counter3;

    omega1 = static_cast<double>(prev_ticks1 - curr_ticks1) / 0.1;
    omega2 = static_cast<double>(curr_ticks2 - prev_ticks2) / 0.1;
    omega3 = static_cast<double>(prev_ticks3 - curr_ticks3) / 0.1;
}


// #define ALPHA 0.5  // Low-pass filter smoothing factor

// double applyLowPassFilter(double new_value, double prev_value) {
//     return ALPHA * new_value + (1 - ALPHA) * prev_value;
// }

// void measureAngularVelocity(double &omega1, double &omega2, double &omega3) {
//     static double filtered_omega1 = 0.0;
//     static double filtered_omega2 = 0.0;
//     static double filtered_omega3 = 0.0;

//     int64_t prev_ticks1, prev_ticks2, prev_ticks3;
//     int64_t curr_ticks1, curr_ticks2, curr_ticks3;

//     unsigned long startTime = millis();
//     prev_ticks1 = counter1;
//     prev_ticks2 = counter2;
//     prev_ticks3 = counter3;

//     // Delay for 100 ms
//     delay(100);

//     unsigned long endTime = millis();
//     double elapsedTime = (endTime - startTime) / 1000.0; // Convert to seconds

//     curr_ticks1 = counter1;
//     curr_ticks2 = counter2;
//     curr_ticks3 = counter3;

//     // Calculate raw angular velocities
//     double raw_omega1 = static_cast<double>(curr_ticks1 - prev_ticks1) / elapsedTime;
//     double raw_omega2 = static_cast<double>(curr_ticks2 - prev_ticks2) / elapsedTime;
//     double raw_omega3 = static_cast<double>(curr_ticks3 - prev_ticks3) / elapsedTime;

//     // Apply Low-Pass Filtering
//     omega1 = applyLowPassFilter(raw_omega1, filtered_omega1);
//     omega2 = applyLowPassFilter(raw_omega2, filtered_omega2);
//     omega3 = applyLowPassFilter(raw_omega3, filtered_omega3);

//     // Update filtered values for next iteration
//     filtered_omega1 = omega1;
//     filtered_omega2 = omega2;
//     filtered_omega3 = omega3;
// }


#endif // MOTOR_HPP