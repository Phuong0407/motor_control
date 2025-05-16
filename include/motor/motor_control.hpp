#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include "config.h"
#include "encoder.hpp"
#include "motorstate.hpp"
#include "pid.hpp"


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

public:
    PID pid1, pid2, pid3;
    MotorState motor1, motor2, motor3;
    double measured1 = 0.0, measured2 = 0.0, measured3 = 0.0;
    double computed1 = 0.0, computed2 = 0.0, computed3 = 0.0;

    void setUpController() {
        startEncoders();
    }

    void controlMotor12(double ref1, double ref2) {
        measureAngularVelocity12(measured1, measured2);
        computed1 = pid1.compute(ref1, measured1);
        computed2 = pid2.compute(ref2, measured2);
        
    }

    void stopMotors() {
        wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0x0000);
        wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0x0000);
    }
};



#endif // MOTOR_CONTROL_HPP