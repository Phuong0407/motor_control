/**
 * @brief MotorUnit — Controls a motor via Grove I2C Motor Driver and measures RPM with encoder (DG01D-E)
 * @file encoder.hpp
 * @date April 18, 2025
 * 
 */

#ifndef PID_CONTROLER_HPP
#define PID_CONTROLER_HPP

#include <algorithm>

/**
 * @brief PIDController class for implementing a PID control algorithm.
 * 
 */

class PIDController {
private:
    double kp;
    double ki;
    double kd;

    double setpoint;
    double integral;
    double previous_error;

public:
    PIDController(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), setpoint(0), integral(0), previous_error(0) {}

    void setSetpoint(double new_setpoint) {
        setpoint = new_setpoint;
    }

    double compute(double measured_value, double dt) {
        double error = setpoint - measured_value;
        integral += error * dt;
        double derivative = (error - previous_error) / dt;
        previous_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};


#endif // PID_CONTROLER_HPP