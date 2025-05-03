#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "encoder.hpp"
#include "pid_controler.hpp"

#include <cmath>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>
#include <iomanip>

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





#define MAX_RPS_LOADED      0.6250
#define MAX_RPS_NOLOADED    0.8681
#define SAFTY_OFFSET        0.70





class MotorDriver {
private:
    double dt;
    int i2c_fd[2];
    double ref_omega[3];
    double measured_omega[3];
    double controlled_omega[3];
    int previous_cmd[3];
    PIDController pid1, pid2, pid3;

    inline void initMotor(int driver1_addr = 0x0f, int driver2_addr = 0x0d) {
        i2c_fd[0] = wiringPiI2CSetup(driver1_addr);
        i2c_fd[1] = wiringPiI2CSetup(driver2_addr);
    }

    inline double computeThrottleRPS(double rps, bool loaded_run = true) {
        if (loaded_run) {
            double clamped_rps = std::clamp(rps, -MAX_RPS_LOADED, MAX_RPS_LOADED);
            return clamped_rps / MAX_RPS_LOADED;
        }
        else {
            double clamped_rps = std::clamp(rps, -MAX_RPS_NOLOADED, MAX_RPS_NOLOADED);
            return clamped_rps / MAX_RPS_NOLOADED;
        }
        return 0.0;
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

    void setLeftMotor(double lrps, double rrps) {
        int dir = computeDirection(lrps, rrps);
        double throttle = computeThrottleRPS(lrps);
        int int_command = static_cast<int>(std::round(255.0 * throttle));
        int pwm1 = std::abs(int_command);
        int pwm2 = previous_cmd[1];
        int pwm_out = (pwm1 << 8) | pwm2;
        wiringPiI2CWriteReg16(i2c_fd[0], 0x82, pwm_out);
        wiringPiI2CWriteReg16(i2c_fd[0], 0xaa, dir);
        previous_cmd[0] = pwm1;
    }

    void setRightMotor(double lrps, double rrps) {
        int dir = computeDirection(lrps, rrps);
        double throttle = computeThrottleRPS(rrps);
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
        previous_cmd[0] = previous_cmd[1] = previous_cmd[2] = 0;

        declareEncoders(driver1_addr, driver2_addr);
        attachEncoderInterrupts();
        initMotor(driver1_addr, driver2_addr);
    }

    void setReferenceOmega(const double (&ref_omega)[3]) {
        for (std::size_t i = 0; i < 3; ++i) {
            this->ref_omega[i] = ref_omega[i];
        }
    }

    void controlAngularVelocity(double ref_rps1, double ref_rps2, double ref_rps3) {
        double lerror = 0.0, rerror = 0.0, ferror = 0.0;
        while (true) {
            measureAngularVelocity();
            lerror = ref_rps1 - measured_omega[0];
            rerror = ref_rps2 - measured_omega[1];
            ferror = ref_rps3 - measured_omega[2];
            double set_rps1 = 0.0;
            double set_rps2 = 0.0;

            if (std::abs(lerror) < 0.02) {
                set_rps1 = pid1.compute(ref_rps1, measured_omega[0], 0.2);
            }
            if (std::abs(rerror) < 0.02) {
                set_rps2 = pid2.compute(ref_rps2, measured_omega[1], 0.2);
            }
            setLeftRightMotor(set_rps1, set_rps2, false);
        }
    }


    void measureAngularVelocity(double smpl_itv = 0.4) {
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

        measured_omega[0] = static_cast<double>(curr_ticks0 - prev_ticks0) / smpl_itv_msm / COUNTER_PER_REV;
        measured_omega[1] = static_cast<double>(curr_ticks1 - prev_ticks1) / smpl_itv_msm / COUNTER_PER_REV;
        measured_omega[2] = static_cast<double>(curr_ticks2 - prev_ticks2) / smpl_itv_msm / COUNTER_PER_REV;

        std::cout << std::setprecision(3) << measured_omega[0] << "\t" << measured_omega[1] << "\t" << measured_omega[2] << "\n";
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

    void setLeftRightMotor(double lrps, double rrps, bool loaded_run = true) {
        int dir = computeDirection(lrps, rrps);
        double lthrottle = computeThrottleRPS(lrps, loaded_run);
        double rthrottle = computeThrottleRPS(rrps, loaded_run);
        int lint_command = static_cast<int>(std::round(255.0 * lthrottle));
        int rint_command = static_cast<int>(std::round(255.0 * rthrottle));
        int pwm1 = std::abs(lint_command);
        int pwm2 = std::abs(rint_command);
        int pwm  = (pwm1 << 8) | pwm2;
        wiringPiI2CWriteReg16(i2c_fd[0], 0x82, pwm);
        wiringPiI2CWriteReg16(i2c_fd[0], 0xaa, dir);
        previous_cmd[0] = pwm1;
        previous_cmd[1] = pwm2;
    }
};

#endif // MOTOR_DRIVER_HPP