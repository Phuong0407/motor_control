#ifndef MOTORCONTROLLER_HPP
#define MOTORCONTROLLER_HPP

#include "encoder.hpp"
#include "motorstate.hpp"
#include "pid.hpp"
#include <chrono>
using namespace std::chrono;

static constexpr double ERROR_THRESHOLD_PERCENT = 0.10;
static constexpr int STABLE_CYCLE_REQUIRED = 5;

class MotorController {
private:
    PID pid1, pid2, pid3;
    MotorState motor1, motor2, motor3;
    double ref1 = 0.0, ref2 = 0.0, ref3 = 0.0;
    double measured1 = 0.0, measured2 = 0.0, measured3 = 0.0;
    double computed1 = 0.0, computed2 = 0.0, computed3 = 0.0;

    int computeDirection(int dir);
    int computeDirection(int dir1, int dir2);

public:
    MotorController() = default;
    void setMotorController(
        double kp1, double ki1, double kd1, double cutoff_freq1, double max_rps1,
        double kp2, double ki2, double kd2, double cutoff_freq2, double max_rps2,
        double kp3, double ki3, double kd3, double cutoff_freq3, double max_rps3
    );

    void controlMotor1();
    void controlMotor2();
    void controlMotor3();
    void setMotor1(int pwm1, int dir1);
    void setMotor2(int pwm2, int dir2);
    void setMotor3(int pwm3, int dir3);

    inline void setMotor1Reference(double ref1) { this->ref1 = ref1; }
    inline void setMotor2Reference(double ref2) { this->ref2 = ref2; }
    inline void setMotor3Reference(double ref3) { this->ref3 = ref3; }
    inline double getMotor1Reference() const { return ref1; }
    inline double getMotor2Reference() const { return ref2; }
    inline double getMotor3Reference() const { return ref3; }
};

int MotorController::computeDirection(int dir) {
    if (dir == +1)
        return 0x06;
    if (dir == -1)
        return 0x0a;
    return 0x06;
}

int MotorController::computeDirection(int dir1, int dir2) {
    if (dir1 == 0) dir1 = +1;
    if (dir2 == 0) dir2 = +1;

    if (dir1 == +1 && dir2 == +1)
        return 0x06;
    if (dir1 == -1 && dir2 == +1)
        return 0x0a;
    if (dir1 == +1 && dir2 == -1)
        return 0x05;
    if (dir1 == -1 && dir2 == -1)
        return 0x09;
    else
        return 0x06;
}

void MotorController::setMotor1(int pwm1, int dir1) {
    int dir12 = computeDirection(dir1, motor2.dir);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | motor2.pwm);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, dir12);
}

void MotorController::setMotor2(int pwm2, int dir2) {
    int dir12 = computeDirection(motor1.dir, dir2);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (motor1.pwm << 8) | pwm2);
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, dir12);
}

void MotorController::setMotor3(int pwm3, int dir3) {
    dir3 = computeDirection(dir3);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, (pwm3 << 8));
    microsleep(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, dir3);
}

void MotorController::controlMotor1() {
    int stabilization_cycle_count = 0;
    auto start_time = high_resolution_clock::now();
    while(true) {
        measured1 = measureAngularVelocity1();
        double err1 = ref1 - measured1;
        double err_thres = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref1), MIN_ERROR_RPS);
        if (std::abs(err1) > err_thres + 1e-6) {
            computed1 = pid1.compute(ref1, measured1);
            printf("[INFO] Motor 1: %.3f\t%.3f\t%.3f\n", measured1, computed1, err1 * 100.0);
            motor1.setMotorStateRPS(computed1);
            setMotor1(motor1.getPWM(), motor1.getDirection());
        }
        else {
            stabilization_cycle_count++;
        }
        if (stabilization_cycle_count >= STABLE_CYCLE_REQUIRED) {
            auto end_time = high_resolution_clock::now();
            double stabilization_time = duration_cast<milliseconds>(end_time - start_time).count() / 1000.0;
            printf("[INFO] Motor 1 stabilized in %.3f seconds with rps = %.3f\n", stabilization_time, measureAngularVelocity1());
        }
    }
}

void MotorController::controlMotor2() {
    int stabilization_cycle_count = 0;
    auto start_time = high_resolution_clock::now();
    while(true) {
        measured2 = measureAngularVelocity2();
        double err2 = ref2 - measured2;
        double err_thres = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref2), MIN_ERROR_RPS);
        if (std::abs(err2) > err_thres + 1e-6) {
            computed2 = pid2.compute(ref2, measured2);
            printf("[INFO] Motor 2: %.3f\t%.3f\t%.3f\n", measured2, computed2, err2 * 100.0);
            motor2.setMotorStateRPS(computed2);
            setMotor2(motor2.getPWM(), motor2.getDirection());
        }
        else {
            stabilization_cycle_count++;
        }
        if (stabilization_cycle_count >= STABLE_CYCLE_REQUIRED) {
            auto end_time = high_resolution_clock::now();
            double stabilization_time = duration_cast<milliseconds>(end_time - start_time).count() / 1000.0;
            printf("[INFO] Motor 2 stabilized in %.3f seconds with rps = %.3f\n", stabilization_time, measureAngularVelocity2());
        }
    }
}

void MotorController::controlMotor3() {
    int stabilization_cycle_count = 0;
    auto start_time = high_resolution_clock::now();
    while(true) {
        measured3 = measureAngularVelocity3();
        double err3 = ref3 - measured3;
        double err_thres = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref3), MIN_ERROR_RPS);
        if (std::abs(err3) > err_thres + 1e-6) {
            computed3 = pid3.compute(ref3, measured3);
            printf("[INFO] Motor 3: %.3f\t%.3f\t%.3f\n", measured3, computed3, err3 * 100.0);
            motor3.setMotorStateRPS(computed3);
            setMotor3(motor3.getPWM(), motor3.getDirection());
        }
        else {
            stabilization_cycle_count++;
        }
        if (stabilization_cycle_count >= STABLE_CYCLE_REQUIRED) {
            auto end_time = high_resolution_clock::now();
            double stabilization_time = duration_cast<milliseconds>(end_time - start_time).count() / 1000.0;
            printf("[INFO] Motor 3 stabilized in %.3f seconds with rps = %.3f\n", stabilization_time, measureAngularVelocity3());
        }
    }
}

void MotorController::setMotorController(
    double kp1, double ki1, double kd1, double cutoff_freq1, double max_rps1,
    double kp2, double ki2, double kd2, double cutoff_freq2, double max_rps2,
    double kp3, double ki3, double kd3, double cutoff_freq3, double max_rps3
) {
    pid1.setPIDParameters(kp1, ki1, kd1, cutoff_freq1, max_rps1);
    pid2.setPIDParameters(kp2, ki2, kd2, cutoff_freq2, max_rps2);
    pid3.setPIDParameters(kp3, ki3, kd3, cutoff_freq3, max_rps3);
    motor1.setMotorState(0, 0);
    motor2.setMotorState(0, 0);
    motor3.setMotorState(0, 0);
    startEncoders();
}

#endif // MOTORCONTROLLER_HPP