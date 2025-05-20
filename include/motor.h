#ifndef MOTOR_H
#define MOTOR_H

#include "robot.h"

inline int computePWMFromUnsignedRPS(double u_rps) {
    double norm_rps = std::clamp(u_rps / MAX_TPS, 0.0, 1.0);
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


void * setMotor1(void *arg) {
    dir1 = (computed1 > 0) ? +1 : -1;
    pwm1 = computePWMFromUnsignedRPS(std::abs(ref1));
    int xdir12 = computeDirection(dir1, dir2);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, xdir12);
    return nullptr;
}

void * setMotor2(void *arg) {
    dir2 = (computed2 > 0) ? +1 : -1;
    pwm2 = computePWMFromUnsignedRPS(std::abs(ref3));
    int xdir12 = computeDirection(dir1, dir2);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, xdir12);
    return nullptr;
}

void * setMotor3(void *arg) {
    dir3 = (computed3 > 0) ? +1 : -1;
    pwm3 = computePWMFromUnsignedRPS(std::abs(ref3));
    int xdir3 = computeDirection(dir3);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, (pwm3 << 8));
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, xdir3);
    return nullptr;
}

#endif // MOTOR_H