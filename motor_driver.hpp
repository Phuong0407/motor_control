#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "encoder.hpp"
#include "pid_controler.hpp"

#include <cmath>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>

#define MOVE_FORWARD    1
#define MOVE_BACKWARD   2
#define TURN_LEFT       3
#define TURN_RIGHT      4
#define HOLD_POSITION   0


inline bool checkTickSetLogic(int ms1, int ms2, int dir) {
    if (ms1 == 0 && ms2 == 0 && dir == HOLD_POSITION)
        return true;
    if ((ms1 == ms2) && (dir == MOVE_FORWARD || dir == MOVE_BACKWARD))
        return true;
    if ((ms1 < ms2) && dir == TURN_RIGHT)
        return true;
    if ((ms1 < ms2) && dir == TURN_LEFT)
        return true;
    else
        return false;
}





#define MAX_RPS         0.6250
#define SAFTY_OFFSET    0.60





class MotorDriver {
private:
    double dt;
    int i2c_fd[2];
    double ref_omega[3];
    double measured_omega[3];
    double controlled_omega[3];
    int64_t previous_ticks[3];
    int64_t current_ticks[3];
    int previous_cmd[3];
    PIDController pid1, pid2, pid3;

    inline void initMotor(int driver1_addr = 0x0f, int driver2_addr = 0x0d) {
        i2c_fd[0] = wiringPiI2CSetup(driver1_addr);
        i2c_fd[1] = wiringPiI2CSetup(driver2_addr);
    }

    inline double computeThrottleRPS(double rps) {
        double clamped_rps = std::clamp(rps, -MAX_RPS, MAX_RPS);
        return clamped_rps / MAX_RPS * SAFTY_OFFSET;
    }

    inline int computeDirection(double l_rps, double r_rps) {
        if (l_rps > 0 && r_rps > 0)
            return 0x0a;
        else if (l_rps > 0 && r_rps < 0)
            return 0x06;
        else if (l_rps < 0 && r_rps > 0)
            return 0x09;
        else
            return 0x05;
    }

    void setLeftMotor(double l_rps, double r_rps) {
        int dir = computeDirection(l_rps, r_rps);
        double throttle = computeThrottleRPS(l_rps);
        int int_command = static_cast<int>(std::round(255.0 * throttle));
        int pwm1 = std::abs(int_command);
        int pwm2 = previous_cmd[1];
        int pwm_out = (pwm1 << 8) | pwm2;
        wiringPiI2CWriteReg16(i2c_fd[0], 0x82, pwm_out);
        wiringPiI2CWriteReg16(i2c_fd[0], 0xaa, dir);
        previous_cmd[0] = pwm1;
    }

    void setRightMotor(double l_rps, double r_rps) {
        int dir = computeDirection(l_rps, r_rps);
        double throttle = computeThrottleRPS(r_rps);
        int int_command = static_cast<int>(std::round(255.0 * throttle));
        int pwm1 = previous_cmd[0];
        int pwm2 = std::abs(int_command);
        int pwm_out = (pwm1 << 8) | pwm2;
        wiringPiI2CWriteReg16(i2c_fd[0], 0x82, pwm_out);
        wiringPiI2CWriteReg16(i2c_fd[0], 0xaa, dir);
        previous_cmd[1] = pwm2;
    }

    void setFrontMotor(double f_rps) {
        int dir = 0;
        if (f_rps < 0)
            dir = 0x06;
        else
            dir = 0x09;

        double throttle = computeThrottleRPS(f_rps);
        int int_command = static_cast<int>(std::round(255.0 * throttle));
        int pwm3 = std::abs(int_command);
        int pwm_out = (pwm3 << 8);
        wiringPiI2CWriteReg16(i2c_fd[1], 0x82, pwm_out);
        wiringPiI2CWriteReg16(i2c_fd[1], 0xaa, dir);
        previous_cmd[2] = pwm3;
    }


public:
    MotorDriver(
        double kp,
        double ki,
        double kd,
        double dt,
        int driver1_addr = 0x0f,
        int driver2_addr = 0x0d
    ) :
    dt(dt),
    pid1(kp, kd, ki),
    pid2(kp, kd, ki),
    pid3(kp, kd, ki)
    {
        i2c_fd[0] = i2c_fd[1] = -1;
        ref_omega[0] = ref_omega[1] = ref_omega[2] = 0.0;
        measured_omega[0] = measured_omega[1] = measured_omega[2] = 0.0;        
        controlled_omega[0] = controlled_omega[1] = controlled_omega[2] = 0.0;        
        previous_ticks[0] = previous_ticks[1] = previous_ticks[2] = 0;    
        current_ticks[0] = current_ticks[1] = current_ticks[2] = 0;
        previous_cmd[0] = previous_cmd[1] = previous_cmd[2] = 0;

        declareEncoders(driver1_addr, driver2_addr);
        attachEncoderInterrupts();
        initMotor(driver1_addr, driver2_addr);
    }

    void setReferenceOmega(const double (&ref_omega)[3]) {
        for (std::size_t i = 0; i < 3; ++i) {
            this->ref_omega[i] = ref_omega[i];
        }
        this->pid1.setSetpoint(ref_omega[0]);
        this->pid2.setSetpoint(ref_omega[1]);
        this->pid3.setSetpoint(ref_omega[2]);
    }

    inline void getPreviousTicks() {
        previous_ticks[0] = encoders[0]->getCounter();
        previous_ticks[1] = encoders[1]->getCounter();
        previous_ticks[2] = encoders[2]->getCounter();
    }
    inline void getCurrentTicks() {
        current_ticks[0] = encoders[0]->getCounter();
        current_ticks[1] = encoders[1]->getCounter();
        current_ticks[2] = encoders[2]->getCounter();
    }
    inline void computeAngularVelocity(double dt) {
        measured_omega[0] = (current_ticks[0] - previous_ticks[0]) / dt;
        measured_omega[1] = (current_ticks[1] - previous_ticks[1]) / dt;
        measured_omega[2] = (current_ticks[2] - previous_ticks[2]) / dt;
    }

    void control() {
        while (true) {
            getPreviousTicks();
            auto t0 = std::chrono::steady_clock::now();
            double dt1 = 0.5;
            double dt2 = 0.1;
            getCurrentTicks();
            computeAngularVelocity(dt1);
            controlled_omega[0] = pid1.compute(measured_omega[0], dt2);
            controlled_omega[1] = pid2.compute(measured_omega[1], dt2);
            controlled_omega[2] = pid3.compute(measured_omega[2], dt2);
            setLeftMotor(controlled_omega[0], measured_omega[1]);
            setRightMotor(controlled_omega[0], controlled_omega[1]);
            setFrontMotor(controlled_omega[2]);
        }
    }
    void measureAngularVelocity() {
        double dt = 0.0;
        getPreviousTicks();
        auto t_start = std::chrono::steady_clock::now();
        while(true) {
            auto t_now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = t_now - t_start;
            if (elapsed.count() >= 0.2) {
                getCurrentTicks();
                dt = elapsed.count();
                break;
            }
        }
        computeAngularVelocity(dt);
        std::cout << measured_omega[0] << "\t" << measured_omega[1] << "\t" << measured_omega[2] << "\n";
    }

    void set_motor_pwm(int pwm1 = 0xff, int pwm2 = 0xff, int pwm3 = 0xff) {
        wiringPiI2CWriteReg16(i2c_fd[0], 0x82, (pwm1 << 8) | pwm2);
        wiringPiI2CWriteReg16(i2c_fd[0], 0xaa, 0x06);
        wiringPiI2CWriteReg16(i2c_fd[1], 0x82, (pwm3 << 8));
        wiringPiI2CWriteReg16(i2c_fd[1], 0xaa, 0x06);
    }
    void stop_motor() {
        wiringPiI2CWriteReg16(i2c_fd[0], 0x82, 0x0000);
        wiringPiI2CWriteReg16(i2c_fd[1], 0x82, 0x0000);
    }
};

#endif // MOTOR_DRIVER_HPP
