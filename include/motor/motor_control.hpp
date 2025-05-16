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
    int computeDirection12(int dir1, int dir2) {
        if (dir1 == FORWARD && dir2 == FORWARD)
            return 0x06;
        if (dir1 == FORWARD && dir2 == BACKWARD)
            return 0x05;
        if (dir1 == BACKWARD && dir2 == FORWARD)
            return 0x0a;
        else
            return 0x09;
    }

    int computeDirection3(int dir) {
        if (dir == LEFT)
            return 0x06;
        if (dir == RIGHT)
            return 0x05;
        return 0x06;
    }

    void setMotor1(int pwm1, int dir1) {
        int pwm2 = motor2.pwm;
        int dir12 = computeDirection12(dir1, motor2.dir);
        wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
        wiringPiI2CWriteReg16(i2c_fd1, 0xaa, dir12);
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
            measureAngularVelocity1(measured1);
            double err1 = ref1 - measured1;
            double err_thres = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref1), MIN_ERROR_RPS);
            int pwm1, pwm2, dir1, dir2;
            if (std::abs(err1) > err_thres) {
                computed1 = pid1.compute(err1);
                pwm1 = computePWMFromRPS(computed1);
                dir1 = (computed1 > 0) ? FORWARD : BACKWARD;
                StableCycleCount = 0;
                setMotor1(pwm1, dir1);
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