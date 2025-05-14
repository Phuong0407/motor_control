/**
 * @file motor.hpp
 * @brief measure rps, PID, and motor commands.
 */

#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <cmath>
#include <memory>
#include <vector>
#include <stdio.h>
#include <inttypes.h>


/**
 * @brief calibration result
 */
#ifdef NOLOADED_RUN
#define MAX_RPS1 0.859
#define MAX_RPS2 0.853
#define MAX_RPS3 0.820
#else // LOADED_RUN
#define MAX_RPS1 0.630
#define MAX_RPS2 0.630
#define MAX_RPS3 0.623
#endif

#ifndef I2C_FD_ADDRESS
#define I2C_FD_ADDRESS

int i2c_fd1;
int i2c_fd2;

#endif // I2C_FD_ADDRESS

#ifndef ENCODER
#define ENCODER

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
    initEncoder(0x0f, MOTOR1_H1, MOTOR1_H2, i2c_fd1);
    initEncoder(0x0f, MOTOR2_H1, MOTOR2_H2, i2c_fd1);
    initEncoder(0x0d, MOTOR3_H1, MOTOR3_H2, i2c_fd2);
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

#endif // ENCODER




#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#ifndef PID_CONTROL_SAMPLING
#define PID_CONTROL_SAMPLING
static constexpr double smpl_itv    = 0.1;
static constexpr double cutoff_freq = 4.0;
#endif // PID_CONTROL_SAMPLING

#ifndef PID_PARAMETER
#define PID_PARAMETER

static constexpr double kp = 6.0;
static constexpr double ki = 3.0;
static constexpr double kd = 0.015;
static constexpr double max_out = 1.0;

#endif // PID_PARAMETER

#ifndef PID_SETTING
#define PID_SETTING

#include "pid.hpp"

PID pid1(kp, ki, kd, max_out, cutoff_freq, smpl_itv);
PID pid2(kp, ki, kd, max_out, cutoff_freq, smpl_itv);
PID pid3(6.0, 2.5, 0.015, max_out, 0.0, smpl_itv);

#endif // PID_SETTING
#endif // PID_CONTROLLER



#ifndef MOTOR_COMMAND
#define MOTOR_COMMAND

#define SAFTY_OFFSET 0.8

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

int computeDirection(double norm_lrps, double norm_rrps)
{
    bool lforward = (norm_lrps > 0);
    bool rforward = (norm_rrps > 0);
    if ( lforward &&  rforward)
        return 0x06;
    if ( lforward && !rforward)
        return 0x05;
    if (!lforward &&  rforward)
        return 0x0a;
    else
        return 0x09;
}

void setLeftRightMotorNormalized(double norm_lrps, double norm_rrps) {
    int dir  = computeDirection(norm_lrps, norm_rrps);
    int pwm1 = computePWMFromNormedRPS(norm_lrps);
    int pwm2 = computePWMFromNormedRPS(norm_rrps);
    int pwm  = (pwm1 << 8) | pwm2;
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, pwm);
    delay(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, dir);
}

void setFrontMotorNormalized(double norm_frps) {
    int dir = (norm_frps < 0) ? 0x06 : 0x09;
    int pwm3 = computePWMFromNormedRPS(norm_frps);
    int pwm = (pwm3 << 8);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, pwm);
    delay(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, dir);
}

void stop_motor() {
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0x0000);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0x0000);
}

#endif // MOTOR_COMMAND



#ifndef MOTOR_CONTROL_ALGO
#define MOTOR_CONTROL_ALGO

static double constexpr timeout_seconds = 3.0;

void controlAngularVelocity(
    double ref_rps1,
    double ref_rps2,
    double ref_rps3
) {
    constexpr double MIN_ERROR_RPS = 0.08;
    constexpr double ERROR_THRESHOLD_PERCENT = 0.10;
    constexpr int STABLE_CYCLES_REQUIRED = 5;
    int stable_cycle_count = 0;
    
    double lerror = 0.0, rerror = 0.0, ferror = 0.0;
    
    uint64_t start_time = millis();
    
    while (true) {
        uint64_t loop_start = millis();
        double omega1, omega2, omega3;
        measureAngularVelocity(omega1, omega2, omega3, smpl_itv);
    
        lerror = std::abs(ref_rps1 - omega1);
        rerror = std::abs(ref_rps2 - omega2);
        ferror = std::abs(ref_rps3 - omega3);
    
        double l_thresh = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref_rps1), MIN_ERROR_RPS);
        double r_thresh = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref_rps2), MIN_ERROR_RPS);
        double f_thresh = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref_rps3), MIN_ERROR_RPS);

        double norm_rps1 = (lerror >= l_thresh) ? pid1.compute(ref_rps1 / MAX_RPS1, omega1 / MAX_RPS1) : omega1 / MAX_RPS1;
        double norm_rps2 = (rerror >= r_thresh) ? pid2.compute(ref_rps2 / MAX_RPS2, omega2 / MAX_RPS2) : omega2 / MAX_RPS2;
        double norm_rps3 = (ferror >= f_thresh) ? pid3.compute(ref_rps3 / MAX_RPS3, omega3 / MAX_RPS3) : omega3 / MAX_RPS3;

        setLeftRightMotorNormalized(norm_rps1, norm_rps2);
        setFrontMotorNormalized(norm_rps3);
    
        // printf(
        //     "Measured RPS: %.3f\t%.3f\t%.3f\t"
        //     "Computed RPS: %.3f\t%.3f\t%.3f\n",
        //     omega1, omega2, omega3,
        //     norm_rps1 * MAX_RPS1, norm_rps2 * MAX_RPS2, norm_rps3 * MAX_RPS3
        // );

        bool stable = (lerror < l_thresh) && (rerror < r_thresh) && (ferror < f_thresh);

        if (stable) stable_cycle_count++;
        else stable_cycle_count = 0;

        uint64_t current_time = millis();
        double elapsed_time = (current_time - start_time) / 1000.0;

        if (stable_cycle_count >= STABLE_CYCLES_REQUIRED) {
            printf("[INFO] Motor control stabilized after %.1f seconds.\n", elapsed_time);
            break;
        }
        if (elapsed_time >= timeout_seconds) {
            printf("[WARNING] Motor control timed out after %.1f seconds.\n", timeout_seconds);
            break;
        }

        int remaining_delay = static_cast<int>(smpl_itv * 1000) - millis() + loop_start;
        if (remaining_delay > 0) {
            delay(remaining_delay);
        }
    }
}

#endif // MOTOR_CONTROL_ALGO
#endif // MOTOR_CONTROL_HPP
