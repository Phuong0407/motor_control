#ifndef MOTOR_H
#define MOTOR_H

#include "robot.h"

inline int computePWMFromUnsignedRPS(double utps) {
    double NORMED_TPS = std::clamp(utps / MAX_TPS, 0.0, 1.0);
    int pwm_value = static_cast<int>(MAX_PWM * NORMED_TPS * SAFETY_OFFSET);
    if (pwm_value < DEAD_PWM) return 0;
    return static_cast<int>((pwm_value - DEAD_PWM) / DEADZONE_SCALEUP);
}

inline int computeDirection(int dir) {
    return (dir == +1) ? 0x06 : 0x09;
}

inline int computeDirection(int dir1, int dir2) {
    if (dir1 == 0) dir1 = +1;
    if (dir2 == 0) dir2 = +1;

    if (dir1 == +1 && dir2 == +1) return 0x06;
    if (dir1 == -1 && dir2 == +1) return 0x0a;
    if (dir1 == +1 && dir2 == -1) return 0x05;
    return 0x09;
}

void setAllMotors(int speed1, int speed2, int speed3) {
    int xdir12  = computeDirection(speed1, speed2);
    int xdir3   = computeDirection(speed3);
    int PWM1    = computePWMFromUnsignedRPS(std::abs(speed1));
    int PWM2    = computePWMFromUnsignedRPS(std::abs(speed2));
    int PWM3    = computePWMFromUnsignedRPS(std::abs(speed3));

    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, xdir12);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (PWM1 << 8) | PWM2);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, xdir3);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, (PWM3 << 8));
}

void setMotor1() {
    dir1 = (computed1 > 0) ? +1 : -1;
    pwm1 = computePWMFromUnsignedRPS(std::abs(computed1));
    int xdir12 = computeDirection(dir1, dir2);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, xdir12);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
}

void setMotor2() {
    dir2 = (computed2 > 0) ? +1 : -1;
    pwm2 = computePWMFromUnsignedRPS(std::abs(computed2));
    int xdir12 = computeDirection(dir1, dir2);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, xdir12);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
}

void setMotor3() {
    dir3 = (computed3 > 0) ? +1 : -1;
    pwm3 = computePWMFromUnsignedRPS(std::abs(computed3));
    int xdir3 = computeDirection(dir3);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, xdir3);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, (pwm3 << 8));
}

void setMotor12(double TPS) {
    int dir12 = (TPS > 0.0) ? 0x06 : 0x09;
    double NORMED_TPS = std::clamp(TPS / MAX_TPS, 0.0, 1.0);
    int pwm = static_cast<int>(MAX_PWM * NORMED_TPS);
    if (pwm < DEAD_PWM) pwm = 0;
    else pwm = static_cast<int>((pwm_value - DEAD_PWM) / DEADZONE_SCALEUP);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, dir12);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm << 8) | pwm);

}

void stopAllMotors () {
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0x0000);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0x0000);
}

#endif // MOTOR_H