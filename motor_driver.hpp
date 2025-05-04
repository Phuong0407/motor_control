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
#define MAX_RPS 0.6250
#else
#define MAX_RPS 0.8681
#endif

#define SAFTY_OFFSET 0.6

class MotorDriver {
private:
    int i2c_fd[2];
    double smpl_intv;
    PIDController pid1, pid2, pid3;

    inline void initMotor(int driver1_addr, int driver2_addr) {
        i2c_fd[0] = wiringPiI2CSetup(driver1_addr);
        i2c_fd[1] = wiringPiI2CSetup(driver2_addr);
    }

    inline double computeThrottleRPS(double rps) {
        double clamped_rps = std::clamp(rps, -MAX_RPS, MAX_RPS);
        return clamped_rps * SAFTY_OFFSET;
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
    ) {
        const double error_threshold = 0.05;
        const int stable_cycles_required = 5;
        int stable_cycle_count = 0;

        double lerror = 0.0, rerror = 0.0, ferror = 0.0;
        while (true) {
            auto start = std::chrono::steady_clock::now();
            double omega1, omega2, omega3;
            measureAngularVelocity(omega1, omega2, omega3, smpl_intv);

            lerror = ref_rps1 - omega1;
            rerror = ref_rps2 - omega2;
            ferror = ref_rps3 - omega3;

            std::cout << std::setprecision(3) << "error " << lerror/ref_rps1 * 100.0 << "%\t" << rerror/ref_rps2 *100.0 << "%\n";

            double set_rps1 = std::abs(lerror) >= error_threshold ? pid1.compute(ref_rps1, omega1) : omega1;
            double set_rps2 = std::abs(rerror) >= error_threshold ? pid2.compute(ref_rps2, omega2) : omega2;
            double set_rps3 = std::abs(ferror) >= error_threshold ? pid3.compute(ref_rps3, omega3) : omega3;

            setLeftRightMotor(set_rps1, set_rps2);
            setFrontMotor(set_rps3);

            bool stable = std::abs(lerror) < error_threshold &&
                          std::abs(rerror) < error_threshold &&
                          std::abs(ferror) < error_threshold;

            if (stable)
                stable_cycle_count++;
            else
                stable_cycle_count = 0;

            if (stable_cycle_count >= stable_cycles_required)
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

        auto t_start = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::duration<double>(smpl_itv));

        curr_ticks0 = encoders[0]->getCounter();
        curr_ticks1 = encoders[1]->getCounter();
        curr_ticks2 = encoders[2]->getCounter();

        auto t_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = t_end - t_start;
        double smpl_itv_msm = elapsed.count();

        omega1 = static_cast<double>(curr_ticks0 - prev_ticks0) / smpl_itv_msm / COUNTER_PER_REV;
        omega2 = static_cast<double>(curr_ticks1 - prev_ticks1) / smpl_itv_msm / COUNTER_PER_REV;
        omega3 = static_cast<double>(curr_ticks2 - prev_ticks2) / smpl_itv_msm / COUNTER_PER_REV;

        std::ofstream outFile("omega_output.txt", std::ios::app);
//        std::cout   << std::setprecision(3) << omega1 << "\t" << omega2 << "\t" << omega3 << "\n";
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

    void setLeftRightMotor(double lrps, double rrps) {
        int dir = computeDirection(lrps, rrps);
        double lthrottle = computeThrottleRPS(lrps);
        double rthrottle = computeThrottleRPS(rrps);
        int lint_command = static_cast<int>(std::round(255.0 * lthrottle));
        int rint_command = static_cast<int>(std::round(255.0 * rthrottle));
        int pwm1    = std::abs(lint_command);
        int pwm2    = std::abs(rint_command);
        int pwm     = (pwm1 << 8) | pwm2;
        wiringPiI2CWriteReg16(i2c_fd[0], 0x82, pwm);
        wiringPiI2CWriteReg16(i2c_fd[0], 0xaa, dir);
    }

    void setFrontMotor(double f_rps) {
        int dir = (f_rps < 0) ? 0x06 : 0x09;
        double throttle = computeThrottleRPS(f_rps);
        int int_command = static_cast<int>(std::round(255.0 * throttle));
        int pwm3 = std::abs(int_command);
        int pwm_out = (pwm3 << 8);
        wiringPiI2CWriteReg16(i2c_fd[1], 0x82, pwm_out);
        wiringPiI2CWriteReg16(i2c_fd[1], 0xaa, dir);
    }

};

#endif // MOTOR_DRIVER_HPP
