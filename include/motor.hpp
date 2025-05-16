#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <cmath>
#include <vector>
#include <algorithm>

#include <stdio.h>
#include <inttypes.h>

static constexpr double SAFTY_OFFSET    = 0.8;
static constexpr int DEAD_PWM           = 70;
static constexpr int BACKWARD           = -1;
static constexpr int FORWARD            = +1;

constexpr int driver_addr1              = 0x0f;
constexpr int driver_addr2              = 0x0d;
int i2c_fd1;
int i2c_fd2;

constexpr double COUNTER_PER_REV        = 144.0;

static constexpr int MOTOR1_H1          = 21;
static constexpr int MOTOR1_H2          = 22;

static constexpr int MOTOR2_H1          = 3;
static constexpr int MOTOR2_H2          = 4;

static constexpr int MOTOR3_H1          = 27;
static constexpr int MOTOR3_H2          = 0;

void setMotorPWM(int pwm1, int pwm2, int pwm3) {
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, 0x06);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, (pwm3 << 8));
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, 0x06);
}

int computePWMFromNormedRPS(double norm_rps) {
    double clamped = std::clamp(norm_rps, -1.0, 1.0);
    return static_cast<int>(std::round(255.0 * clamped * SAFTY_OFFSET));
}

int computeDirection(int dir1, int dir2) {
    if (dir1 == FORWARD && dir2 == FORWARD)
        return 0x06;
    if (dir1 == FORWARD && dir2 == BACKWARD)
        return 0x05;
    if (dir1 == BACKWARD && dir2 == FORWARD)
        return 0x0a;
    else
        return 0x09;
}

void setThreeMotors(int pwm1, int dir1, int pwm2, int dir2, int pwm3, int dir3) {
    int dir12 = computeDirection(dir1, dir2);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, dir12);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, (pwm3 << 8));
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, dir3);
}

void stopMotors() {
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0x0000);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0x0000);
}

void initEncoder(int i2c_addr, int H1, int H2, int &i2c_fd) {
    wiringPiSetup();
    i2c_fd = wiringPiI2CSetup(i2c_addr);
    pinMode(H1, INPUT); pullUpDnControl(H1, PUD_UP);
    pinMode(H2, INPUT); pullUpDnControl(H2, PUD_UP);
}

volatile int64_t counter1 = 0;
volatile int64_t counter2 = 0;
volatile int64_t counter3 = 0;

void updateCounter1() { digitalRead(MOTOR1_H2) ? ++counter1 : --counter1; }
void updateCounter2() { digitalRead(MOTOR2_H2) ? ++counter2 : --counter2; }
void updateCounter3() { digitalRead(MOTOR3_H2) ? ++counter3 : --counter3; }

void getTicks(int64_t &ticks1, int64_t &ticks2, int64_t &ticks3) {
    ticks1 = counter1; ticks2 = counter2; ticks3 = counter3;
}

void attachEncoderToInterrupts() {
    wiringPiISR(MOTOR1_H1, INT_EDGE_RISING, updateCounter1);
    wiringPiISR(MOTOR2_H1, INT_EDGE_RISING, updateCounter2);
    wiringPiISR(MOTOR3_H1, INT_EDGE_RISING, updateCounter3);
}

void startEncoders() {
    initEncoder(driver_addr1, MOTOR1_H1, MOTOR1_H2, i2c_fd1);
    initEncoder(driver_addr1, MOTOR2_H1, MOTOR2_H2, i2c_fd1);
    initEncoder(driver_addr2, MOTOR3_H1, MOTOR3_H2, i2c_fd2);
    attachEncoderToInterrupts();
}

void clearEncoders() {
    counter1 = 0;
    counter2 = 0;
    counter3 = 0;
}

void measureAngularVelocity(double &omega1, double &omega2, double &omega3, double smpl_itv) {
    int delay_ms = static_cast<int>(smpl_itv * 1000);
    int64_t prev_ticks1, prev_ticks2, prev_ticks3;
    int64_t curr_ticks1, curr_ticks2, curr_ticks3;

    getTicks(prev_ticks1, prev_ticks2, prev_ticks3);
    delay(delay_ms);
    getTicks(curr_ticks1, curr_ticks2, curr_ticks3);

    omega1 = static_cast<double>(prev_ticks1 - curr_ticks1) / smpl_itv / COUNTER_PER_REV;
    omega2 = static_cast<double>(curr_ticks2 - prev_ticks2) / smpl_itv / COUNTER_PER_REV;
    omega3 = static_cast<double>(curr_ticks3 - prev_ticks3) / smpl_itv / COUNTER_PER_REV;
}


#endif // MOTOR_HPP