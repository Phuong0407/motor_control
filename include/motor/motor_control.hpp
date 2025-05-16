#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include "config.h"
#include "encoder.hpp"
#include "motorstate.hpp"
#include "pid.hpp"
#include <cmath>

static constexpr double ERROR_THRESHOLD_PERCENT = 0.10;
static constexpr int STABLE_CYCLES_REQUIRED     = 3;

class MotorControl {
private:
    int computeDirection(int dir) {
        if (dir == FORWARD)
            return 0x06;
        if (dir == BACKWARD)
            return 0x09;
        return 0x06;
    }
    int computeDirection12(int dir1, int dir2) {
        if (dir1 == 0)
            return computeDirection(dir2);
        if (dir2 == 0)
            return computeDirection(dir1);
        if (dir1 == FORWARD && dir2 == FORWARD)
            return 0x06;
        if (dir1 == BACKWARD && dir2 == FORWARD)
            return 0x0a;
        if (dir1 == FORWARD && dir2 == BACKWARD)
            return 0x05;
        if (dir1 == BACKWARD && dir2 == BACKWARD)
            return 0x09;
        else
            return 0x06;
    }

    int computeDirection3(int dir) {
        if (dir == LEFT)
            return 0x06;
        if (dir == RIGHT)
            return 0x09;
        return 0x06;
    }

    void setMotor1(int pwm1, int dir1) {
        int dir12 = computeDirection12(dir1, motor2.dir);
        wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | motor2.pwm);
        wiringPiI2CWriteReg16(i2c_fd1, 0xaa, dir12);
    }

    void setMotor2(int pwm2, int dir2) {
        int dir12 = computeDirection12(motor1.dir, dir2);
        wiringPiI2CWriteReg16(i2c_fd1, 0x82, (motor1.pwm << 8) | pwm2);
        wiringPiI2CWriteReg16(i2c_fd1, 0xaa, dir12);
    }

    void setMotor3(int pwm3, int dir3) {
        dir3 = computeDirection3(dir3);
        wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm3 << 8));
        wiringPiI2CWriteReg16(i2c_fd1, 0xaa, dir3);
    }

public:
    PID pid1, pid2, pid3;
    MotorState motor1, motor2, motor3;
    double measured1 = 0.0, measured2 = 0.0, measured3 = 0.0;
    double computed1 = 0.0, computed2 = 0.0, computed3 = 0.0;

    MotorControl() = default;

    MotorControl(
        double kp1,
        double ki1,
        double kd1,
        double kp2,
        double ki2,
        double kd2,
        double kp3,
        double ki3,
        double kd3
    ) {
        pid1.setUpPIDParamters(kp1, ki1, kd1);
        pid2.setUpPIDParamters(kp2, ki2, kd2);
        pid3.setUpPIDParamters(kp3, ki3, kd3);
        startEncoders();
    }

    void setUpController() {
        startEncoders();
    }

    void controlMotor1(double ref1) {
        int StableCycleCount = 0;
        while(StableCycleCount < STABLE_CYCLES_REQUIRED) {
            measured1 = measureAngularVelocity1();
            double err1 = ref1 - measured1;
            double err_thres = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref1), MIN_ERROR_RPS);
            int pwm1, pwm2, dir1, dir2;
            if (std::abs(err1) > err_thres) {
                computed1 = pid1.compute(err1);
                printf("Motor 1: %.3f\t%.3f\t%.3f\n", measured1, computed1, err1 * 100.0);
                pwm1 = computePWMFromRPS(computed1);
                dir1 = (computed1 > 0) ? FORWARD : BACKWARD;
                setMotor1(pwm1, dir1);
                StableCycleCount = 0;
            } else
                StableCycleCount++;
        }
    }

    void controlMotor2(double ref2) {
        int StableCycleCount = 0;
        while(StableCycleCount < STABLE_CYCLES_REQUIRED) {
            measured1 = measureAngularVelocity2();
            double err2 = ref2 - measured1;
            double err_thres = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref2), MIN_ERROR_RPS);
            int pwm2, pwm1, dir2, dir1;
            if (std::abs(err2) > err_thres) {
                computed2 = pid2.compute(err2);
                printf("Motor 2: %.3f\t%.3f\n", measured2, computed2);
                pwm2 = computePWMFromRPS(computed2);
                dir2 = (computed2 > 0) ? FORWARD : BACKWARD;
                setMotor2(pwm2, dir2);
                StableCycleCount = 0;
            } else
                StableCycleCount++;
        }
    }

    void controlMotor3(double ref3) {
        int StableCycleCount = 0;
        while(StableCycleCount < STABLE_CYCLES_REQUIRED) {
            measured3 = measureAngularVelocity3();
            double err3 = ref3 - measured3;
            double err_thres = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref3), MIN_ERROR_RPS);
            int pwm3, dir3;
            if (std::abs(err3) > err_thres) {
                computed3 = pid3.compute(err3);
                printf("Motor 2: %.3f\t%.3f\n", measured3, computed3);
                pwm3 = computePWMFromRPS(computed3);
                dir3 = (computed3 > 0) ? FORWARD : BACKWARD;
                setMotor2(pwm3, dir3);
                StableCycleCount = 0;
            } else
                StableCycleCount++;
        }
    }

    void stopMotors() {
        wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0x0000);
        wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0x0000);
    }
};



#endif // MOTOR_CONTROL_HPP