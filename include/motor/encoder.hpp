#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <cmath>
#include <memory>
#include <vector>
#include <stdio.h>
#include <inttypes.h>

constexpr int encoder_addr1 = 0x0f;
constexpr int encoder_addr2 = 0x0d;
int i2c_fd1;
int i2c_fd2;

#ifndef ENCODER_PARAMETER
#define ENCODER_PARAMETER

static constexpr double COUNTER_PER_REV = 144.0;

/**
 * @brief Dock      D5          D24             D16
 * @brief BCM       5   6       24      25      16      17
 * @brief wPi       21  22      3       4       27      0
 */
static constexpr int MOTOR1_H1 = 21;
static constexpr int MOTOR1_H2 = 22;

static constexpr int MOTOR2_H1 = 3;
static constexpr int MOTOR2_H2 = 4;

static constexpr int MOTOR3_H1 = 27;
static constexpr int MOTOR3_H2 = 0;

#endif // ENCODER_PARAMETER

volatile int64_t counter1 = 0;
volatile int64_t counter2 = 0;
volatile int64_t counter3 = 0;

void initEncoder(int i2c_addr, int H1, int H2, int &i2c_fd) {
    if (wiringPiSetup() == -1) {
        printf("[ERROR] wiringPiSetup failed.\n");
        return;
    }
    i2c_fd = wiringPiI2CSetup(i2c_addr);
    if (i2c_fd < 0) printf("[ERROR] I2C setup failed.\n");

    pinMode(H1, INPUT); pullUpDnControl(H1, PUD_UP);
    pinMode(H2, INPUT); pullUpDnControl(H2, PUD_UP);

    if (wiringPiI2CRead(i2c_fd) == -1) {
        printf("[ERROR] I2C device is not responding.\n");
        return;
    }
    printf("[INFO] Encoder initialized on pins H1 = %d, H2 = %d.\n", H1, H2);
}

void updateCounter1() { digitalRead(MOTOR1_H2) ? ++counter1 : --counter1; }
void updateCounter2() { digitalRead(MOTOR2_H2) ? ++counter2 : --counter2; }
void updateCounter3() { digitalRead(MOTOR3_H2) ? ++counter3 : --counter3; }

void getTicks(int64_t &ticks1, int64_t &ticks2, int64_t &ticks3) {
    ticks1 = counter1; ticks2 = counter2; ticks3 = counter3;
}

void printAttachementInfo(int AttchmState, int H1) {
    if (AttchmState < 0) {
        printf("[ERROR] Failed to attach ISR to pin %d.\n", H1);
        return;
    } else printf("[INFO] ISR attached to pin %d\n", H1);
}

void attachEncoderInterrupts() {
    int AttchmState1 = wiringPiISR(MOTOR1_H1, INT_EDGE_RISING, updateCounter1);
    int AttchmState2 = wiringPiISR(MOTOR2_H1, INT_EDGE_RISING, updateCounter2);
    int AttchmState3 = wiringPiISR(MOTOR3_H1, INT_EDGE_RISING, updateCounter3);
    printAttachementInfo(AttchmState1, MOTOR1_H1);
    printAttachementInfo(AttchmState2, MOTOR2_H1);
    printAttachementInfo(AttchmState3, MOTOR3_H1);
}

void startEncoders() {
    initEncoder(encoder_addr1, MOTOR1_H1, MOTOR1_H2, i2c_fd1);
    initEncoder(encoder_addr1, MOTOR2_H1, MOTOR2_H2, i2c_fd1);
    initEncoder(encoder_addr2, MOTOR3_H1, MOTOR3_H2, i2c_fd2);
    attachEncoderInterrupts();
}


/**
 * @brief Measures angular velocity for all motors.
 * @param omega1 Reference to store angular velocity for motor 1.
 * @param omega2 Reference to store angular velocity for motor 2.
 * @param omega3 Reference to store angular velocity for motor 3.
 * @param smpl_itv Sampling interval in seconds.
 */
void measureAngularVelocity(double &omega1, double &omega2, double &omega3, double smpl_itv) {
    int64_t prev_ticks1, prev_ticks2, prev_ticks3;
    int64_t curr_ticks1, curr_ticks2, curr_ticks3;

    getTicks(prev_ticks1, prev_ticks2, prev_ticks3);
    int delay_ms = static_cast<int>(smpl_itv * 1000);
    delay(delay_ms);
    getTicks(curr_ticks1, curr_ticks2, curr_ticks3);

    omega1 = static_cast<double>(prev_ticks1 - curr_ticks1) / smpl_itv / COUNTER_PER_REV;
    omega2 = static_cast<double>(curr_ticks2 - prev_ticks2) / smpl_itv / COUNTER_PER_REV;
    omega3 = static_cast<double>(curr_ticks3 - prev_ticks3) / smpl_itv / COUNTER_PER_REV;
}

#endif // ENCODER_HPP