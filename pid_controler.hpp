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
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @param integral Accumulated error for the integral term.
 * @param previous_error Previous error value for the derivative term.
 * @details This class implements a PID control algorithm use to control the relative position of motor shaft.
 * @param setpoint Desired target value, in angle.
 * @param measured_value Current measured value, in angle.
 */

class PIDController {
private:
    double kp;
    double ki;
    double kd;

    double max_integral = 200;

    double setpoint;
    double integral;
    double previous_error;

public:
    PIDController(double kp = 0.05, double ki = 0.05, double kd = 0.05)
    : kp(kp), ki(ki), kd(kd), setpoint(0), integral(0), previous_error(0) {}

    void setSetpoint(double new_setpoint) {
        setpoint = new_setpoint;
    }

    double compute(double measured_value, double dt) {
        double error = setpoint - measured_value;
        integral += error * dt;
        integral = std::clamp(integral, -max_integral, max_integral);
        double derivative = (error - previous_error) / dt;
        previous_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

#endif // PID_CONTROLER_HPP