#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "encoder.hpp"
#include "motor_pid.hpp"

#include <cmath>
#include <chrono>
#include <thread>
#include <algorithm>
#include <stdio.h>
#include <iomanip>
#include <fstream>

#ifdef NOLOADED_RUN
#define MAX_RPS 0.8681
#else // LOADED_RUN
#define MAX_RPS 0.6250
#endif

#define SAFTY_OFFSET 0.8
#define DEADZONE_PWM 70

class MotorDriver {
private:
    int i2c_fd[2];
    double smpl_intv;
    MotorPID pid1, pid2, pid3;

    inline void initMotor(int driver1_addr, int driver2_addr) {
        i2c_fd[0] = wiringPiI2CSetup(driver1_addr);
        i2c_fd[1] = wiringPiI2CSetup(driver2_addr);
    }

    int computePWMFromNormRPS(double norm_rps) {
        double clamped = std::clamp(norm_rps, -1.0, 1.0);
        int raw_pwm = static_cast<int>(std::round(255.0 * clamped * SAFTY_OFFSET));
        return raw_pwm;
    }

    inline int overcomeDeadZonePWM(int pwm) {
        if (pwm < DEADZONE_PWM)
            return 0;
        return DEADZONE_PWM + (pwm * (255 - DEADZONE_PWM)) / 255;
    }

    inline int computeDirection(double norm_lrps, double norm_rrps)
    {
        bool lforward = (norm_lrps > 0);
        bool rforward = (norm_rrps > 0);
        if ( lforward &&  rforward)
            return 0x06;
        if ( lforward && !rforward)
            return 0x05;
        if (!lforward &&  rforward)
            return 0x0a;
        else
            return 0x09;
    }

public:
    MotorDriver(
        double kp,
        double ki,
        double kd,
        double smpl_intv,
        double cutoff_freq,
        int driver1_addr,
        int driver2_addr
    ) :
    smpl_intv(smpl_intv),
    pid1(kp, kd, ki, 1.0, cutoff_freq, smpl_intv),
    pid2(kp, kd, ki, 1.0, cutoff_freq, smpl_intv),
    pid3(kp, kd, ki, 1.0, cutoff_freq, smpl_intv)
    {
        declareEncoders(driver1_addr, driver2_addr);
        attachEncoderInterrupts();
        initMotor(driver1_addr, driver2_addr);
    }

    void measureAngularVelocity(double &omega1, double &omega2, double &omega3, double smpl_itv) {
        int64_t prev_ticks0 = encoders[0]->getCounter();
        int64_t prev_ticks1 = encoders[1]->getCounter();
        int64_t prev_ticks2 = encoders[2]->getCounter();
    
        int delay_ms = static_cast<int>(smpl_itv * 1000);
        delay(delay_ms);
    
        int64_t curr_ticks0 = encoders[0]->getCounter();
        int64_t curr_ticks1 = encoders[1]->getCounter();
        int64_t curr_ticks2 = encoders[2]->getCounter();
    
        omega1 = static_cast<double>(prev_ticks0 - curr_ticks0) / smpl_itv / COUNTER_PER_REV;
        omega2 = static_cast<double>(curr_ticks1 - prev_ticks1) / smpl_itv / COUNTER_PER_REV;
        omega3 = static_cast<double>(curr_ticks2 - prev_ticks2) / smpl_itv / COUNTER_PER_REV;
    }
    
    void controlAngularVelocity(
        double ref_rps1,
        double ref_rps2,
        double ref_rps3,
        double timeout_seconds
    ) {
        constexpr double MIN_ERROR_RPS = 0.08;
        constexpr double ERROR_THRESHOLD_PERCENT = 0.10;
        constexpr int STABLE_CYCLES_REQUIRED = 5;
        int stable_cycle_count = 0;
        
        double lerror = 0.0, rerror = 0.0, ferror = 0.0;
        
        uint64_t start_time = millis();
        while (true) {
            uint64_t loop_start = millis();
            double omega1, omega2, omega3;
            measureAngularVelocity(omega1, omega2, omega3, smpl_intv);
    
            lerror = std::abs(ref_rps1 - omega1);
            rerror = std::abs(ref_rps2 - omega2);
            ferror = std::abs(ref_rps3 - omega3);
        
            double l_thresh = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref_rps1), MIN_ERROR_RPS);
            double r_thresh = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref_rps2), MIN_ERROR_RPS);
            double f_thresh = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref_rps3), MIN_ERROR_RPS);
    
            double norm_rps1 = (lerror >= l_thresh) ? pid1.compute(ref_rps1 / MAX_RPS, omega1 / MAX_RPS) : omega1 / MAX_RPS;
            double norm_rps2 = (rerror >= r_thresh) ? pid2.compute(ref_rps2 / MAX_RPS, omega2 / MAX_RPS) : omega2 / MAX_RPS;
            double norm_rps3 = (ferror >= f_thresh) ? pid3.compute(ref_rps3 / MAX_RPS, omega3 / MAX_RPS) : omega3 / MAX_RPS;
    
            setLeftRightMotorNormalized(norm_rps1, norm_rps2);
            setFrontMotorNormalized(norm_rps3);
        
            printf(
                "Measured RPS: %.3f\t%.3f\t%.3f\t"
                "Computed RPS: %.3f\t%.3f\t%.3f\n",
                omega1, omega2, omega3,
                norm_rps1 * MAX_RPS, norm_rps2 * MAX_RPS, norm_rps3 * MAX_RPS
            );
    
            bool stable = (lerror < l_thresh) && (rerror < r_thresh) && (ferror < f_thresh);
    
            if (stable) stable_cycle_count++;
            else stable_cycle_count = 0;
    
            uint64_t current_time = millis();
            double elapsed_time = (current_time - start_time) / 1000.0;
    
            if (stable_cycle_count >= STABLE_CYCLES_REQUIRED) {
                printf("[INFO] Motor control stabilized after %.1f seconds.\n", elapsed_time);
                break;
            }
            if (elapsed_time >= timeout_seconds) {
                printf("[WARNING] Motor control timed out after %.1f seconds.\n", timeout_seconds);
                break;
            }
    
            int remaining_delay = static_cast<int>(smpl_intv * 1000) - millis() + loop_start;
            if (remaining_delay > 0) {
                delay(remaining_delay);
            }
        }
    }

    void set_motor_pwm(int pwm1, int pwm2, int pwm3) {
        wiringPiI2CWriteReg16(i2c_fd[0], 0x82, (pwm1 << 8) | pwm2);
        wiringPiI2CWriteReg16(i2c_fd[0], 0xaa, 0x06);
        wiringPiI2CWriteReg16(i2c_fd[1], 0x82, (pwm3 << 8));
        wiringPiI2CWriteReg16(i2c_fd[1], 0xaa, 0x06);
    }
    void stop_motor() {
        wiringPiI2CWriteReg16(i2c_fd[0], 0x82, 0x0000);
        wiringPiI2CWriteReg16(i2c_fd[1], 0x82, 0x0000);
    }

    void setLeftRightMotorNormalized(double norm_lrps, double norm_rrps) {
        int dir = computeDirection(norm_lrps, norm_rrps);
        int pwm1 = computePWMFromNormRPS(norm_lrps);
        int pwm2 = computePWMFromNormRPS(norm_rrps);
        int pwm  = (pwm1 << 8) | pwm2;
        wiringPiI2CWriteReg16(i2c_fd[0], 0x82, pwm);
        delay(1);
        wiringPiI2CWriteReg16(i2c_fd[0], 0xaa, dir);
    }

    void setFrontMotorNormalized(double norm_frps) {
        int dir = (norm_frps < 0) ? 0x06 : 0x09;
        int pwm3 = computePWMFromNormRPS(norm_frps);
        int pwm = (pwm3 << 8);
        wiringPiI2CWriteReg16(i2c_fd[1], 0x82, pwm);
        delay(1);
        wiringPiI2CWriteReg16(i2c_fd[1], 0xaa, dir);
    }
};

#endif // MOTOR_HPP