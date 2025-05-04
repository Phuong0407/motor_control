#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "encoder.hpp"
#include "pid_controller.hpp"

#include <cmath>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>
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
    PIDController pid1, pid2, pid3;

    inline void initMotor(int driver1_addr, int driver2_addr) {
        i2c_fd[0] = wiringPiI2CSetup(driver1_addr);
        i2c_fd[1] = wiringPiI2CSetup(driver2_addr);
    }

    int computePWMFromNormRPS(double norm_rps) {
        double clamped = std::clamp(norm_rps, -1.0, 1.0);
        int raw_pwm = static_cast<int>(std::round(255.0 * clamped * SAFTY_OFFSET));
//        return raw_pwm;
        return overcomeDeadZonePWM(std::abs(raw_pwm));
    }

    inline int overcomeDeadZonePWM(int pwm) {
        if (pwm < DEADZONE_PWM)
            return 0;
        return DEADZONE_PWM + (pwm * (255 - DEADZONE_PWM)) / 255;
    }

    inline int computeDirection(double lcurrps, double rcurrps)
    {
        bool lcw = (lcurrps > 0);
        bool rcw = (rcurrps > 0);
        if ( lcw &&  rcw)
            return 0x0a;
        if ( lcw && !rcw)
            return 0x06;
        if (!lcw &&  rcw)
            return 0x09;
        else
            return 0x05;
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
    pid1(kp, kd, ki, MAX_RPS, cutoff_freq, smpl_intv),
    pid2(kp, kd, ki, MAX_RPS, cutoff_freq, smpl_intv),
    pid3(kp, kd, ki, MAX_RPS, cutoff_freq, smpl_intv)
    {
        i2c_fd[0] = i2c_fd[1] = -1;
        declareEncoders(driver1_addr, driver2_addr);
        attachEncoderInterrupts();
        initMotor(driver1_addr, driver2_addr);
    }

    void controlAngularVelocity(
        double ref_rps1,
        double ref_rps2,
        double ref_rps3
    )
    {
        constexpr double ERROR_THRESHOLD = 0.05;
        constexpr int STABLE_CYCLES_REQUIRED = 5;
    
        int stable_cycle_count = 0;
        double lerror = 0.0, rerror = 0.0, ferror = 0.0;
    
        while (true) {
            auto start = std::chrono::steady_clock::now();
    
            double omega1, omega2, omega3;
            measureAngularVelocity(omega1, omega2, omega3, smpl_intv);
    
            lerror = std::abs(ref_rps1 - omega1);
            rerror = std::abs(ref_rps2 - omega2);
            ferror = std::abs(ref_rps3 - omega3);
    
            double ref_norm1 = ref_rps1 / MAX_RPS;
            double ref_norm2 = ref_rps2 / MAX_RPS;
            double ref_norm3 = ref_rps3 / MAX_RPS;
    
            double omega_norm1 = omega1 / MAX_RPS;
            double omega_norm2 = omega2 / MAX_RPS;
            double omega_norm3 = omega3 / MAX_RPS;
    
            double norm_rps1 = lerror >= ERROR_THRESHOLD ? pid1.compute(ref_norm1, omega_norm1) : omega_norm1;
            double norm_rps2 = rerror >= ERROR_THRESHOLD ? pid2.compute(ref_norm2, omega_norm2) : omega_norm2;
            double norm_rps3 = ferror >= ERROR_THRESHOLD ? pid3.compute(ref_norm3, omega_norm3) : omega_norm3;

            std::cout << std::fixed << std::setprecision(3)
                      << "measured rps" << "\t" << omega1 << "\t" << omega2 << "\t"
                      << "norm rps" << "\t" << norm_rps1 << "\t" << norm_rps2
                      << "error " << lerror / ref_rps1 * 100.0 << "%" << "\t"
                      << rerror / ref_rps2 * 100.0 << "%" << "\n";

            setLeftRightMotorNormalized(norm_rps1, norm_rps2);
            setFrontMotorNormalized(norm_rps3);
    
            bool stable = std::abs(lerror) < ERROR_THRESHOLD &&
                          std::abs(rerror) < ERROR_THRESHOLD &&
                          std::abs(ferror) < ERROR_THRESHOLD;
    
            if (stable)
                stable_cycle_count++;
            else
                stable_cycle_count = 0;
    
            if (stable_cycle_count >= STABLE_CYCLES_REQUIRED)
                break;
    
            std::this_thread::sleep_until(start + std::chrono::duration<double>(smpl_intv));
        }
    }

    void measureAngularVelocity(double &omega1, double &omega2, double &omega3, double smpl_itv) {
        int64_t prev_ticks0, prev_ticks1, prev_ticks2;
        int64_t curr_ticks0, curr_ticks1, curr_ticks2;

        prev_ticks0 = encoders[0]->getCounter();
        prev_ticks1 = encoders[1]->getCounter();
        prev_ticks2 = encoders[2]->getCounter();

        std::this_thread::sleep_for(std::chrono::duration<double>(smpl_itv));

        curr_ticks0 = encoders[0]->getCounter();
        curr_ticks1 = encoders[1]->getCounter();
        curr_ticks2 = encoders[2]->getCounter();

        omega1 = static_cast<double>(curr_ticks0 - prev_ticks0) / smpl_itv / COUNTER_PER_REV;
        omega2 = static_cast<double>(curr_ticks1 - prev_ticks1) / smpl_itv / COUNTER_PER_REV;
        omega3 = static_cast<double>(curr_ticks2 - prev_ticks2) / smpl_itv / COUNTER_PER_REV;

        std::ofstream outFile("omega_output.txt", std::ios::app);
        outFile     << std::setprecision(3) << omega1 << "\t" << omega2 << "\t" << omega3 << "\n";
        outFile.close();
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
        wiringPiI2CWriteReg16(i2c_fd[0], 0xaa, dir);
    }

    void setFrontMotorNormalized(double norm_frps) {
        int dir = (norm_frps < 0) ? 0x06 : 0x09;
        int pwm3 = computePWMFromNormRPS(norm_frps);
        int pwm = (pwm3 << 8);
        wiringPiI2CWriteReg16(i2c_fd[1], 0x82, pwm);
        wiringPiI2CWriteReg16(i2c_fd[1], 0xaa, dir);
    }
};

#endif // MOTOR_DRIVER_HPP
