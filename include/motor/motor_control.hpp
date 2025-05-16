#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include "config.h"
#include "encoder.hpp"
#include "motorstate.hpp"
#include "pid.hpp"

class MotorControl {
public:
    PID pid1, pid2, pid3;
    double measured1 = 0.0, measured2 = 0.0, measured3 = 0.0;
    double computed1 = 0.0, computed2 = 0.0, computed3 = 0.0;

    void setUpController() {
        startEncoders();
    }

    void controlMotor12(double ref1, double ref2) {
        measureAngularVelocity12(measured1, measured2);
    }
};



#endif // MOTOR_CONTROL_HPP