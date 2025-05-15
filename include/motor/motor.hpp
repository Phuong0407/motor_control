/**
 * @file motor.hpp
 * @brief measure rps, PID, and motor commands.
 */

#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include "encoder.hpp"
#include "pid.hpp"

/**
 * @brief calibration result
 */
#ifdef NOLOADED_RUN
#define MAX_RPS1 0.859
#define MAX_RPS2 0.853
#define MAX_RPS3 0.820
#else // LOADED_RUN
#define MAX_RPS1 0.630
#define MAX_RPS2 0.630
#define MAX_RPS3 0.623
#endif

#ifndef MOTOR_COMMAND
#define MOTOR_COMMAND

static constexpr double SAFTY_OFFSET    = 0.8;
static constexpr int BACKWARD           = -1;
static constexpr int FORWARD            = +1;

void setMotorPWM(int pwm1, int pwm2, int pwm3) {
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, 0x06);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, (pwm3 << 8));
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, 0x06);
}

int computePWMFromNormedRPS(double norm_rps) {
    double clamped = std::clamp(norm_rps, -1.0, 1.0);
    return static_cast<int>(std::round(255.0 * clamped * SAFTY_OFFSET));
}

int computeDirection(int dir1, int dir2) {
    if (dir1 == FORWARD && dir2 == FORWARD)
        return 0x06;
    if (dir1 == FORWARD && dir2 == BACKWARD)
        return 0x05;
    if (dir1 == BACKWARD && dir2 == FORWARD)
        return 0x0a;
    else
        return 0x09;
}

void setThreeMotors(int pwm1, int dir1, int pwm2, int dir2, int pwm3, int dir3) {
    int dir12   = computeDirection(dir1, dir2);
    
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
    delay(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, dir12);

    wiringPiI2CWriteReg16(i2c_fd2, 0x82, (pwm3 << 8));
    delay(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, dir3);
}

void stopMotor() {
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0x0000);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0x0000);
}

#endif // MOTOR_COMMAND



#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#ifndef PID_CONTROL_SAMPLING
#define PID_CONTROL_SAMPLING
static constexpr double smpl_itv    = 0.1;
static constexpr double cutoff_freq = 4.0;
#endif // PID_CONTROL_SAMPLING
#endif // PID_CONTROLLER




#ifndef MOTOR_CONTROL_ALGO
#define MOTOR_CONTROL_ALGO

static double constexpr timeout_seconds = 1.0;

class MotorState {
private:
    int pwm;
    int dir;

public:
    MotorState() = default;
    inline int getPWM() const { return pwm; }
    inline int getDirection() const { return dir; }
    inline void setPWM(int pwm) { this->pwm = pwm; }
    inline void setDir(int pwm) { this->dir = dir; }
};



class Motor {
private:
    MotorState motor1;
    MotorState motor2;
    MotorState motor3;

    PID pid1;
    PID pid2;
    PID pid3;
public:
    Motor() = default;
    void setPIDParameters(double kp, double ki, double kd, double max_out) {
        pid1 = PID(kp, ki, kd, max_out, cutoff_freq, smpl_itv);
        pid2 = PID(kp, ki, kd, max_out, cutoff_freq, smpl_itv);
        pid3 = PID(kp, ki, kd, max_out, cutoff_freq, smpl_itv);
    }

    int controlAngularVelocity(
        double ref_rps1,
        double ref_rps2,
        double ref_rps3
    ) {
        constexpr double MIN_ERROR_RPS = 0.08;
        constexpr double ERROR_THRESHOLD_PERCENT = 0.10;
        constexpr int STABLE_CYCLES_REQUIRED = 5;
        int stable_cycle_count = 0;

        int pwm1, pwm2, pwm3;
        double omega1, omega2, omega3;
        double norm_rps1, norm_rps2, norm_rps3;

        double lerror, rerror, ferror;
        uint64_t start_time = millis();
        while (true) {
            uint64_t loop_start = millis();
            measureAngularVelocity(omega1, omega2, omega3, smpl_itv);
        
            lerror = std::abs(ref_rps1 - omega1);
            rerror = std::abs(ref_rps2 - omega2);
            ferror = std::abs(ref_rps3 - omega3);
        
            double l_thresh = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref_rps1), MIN_ERROR_RPS);
            double r_thresh = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref_rps2), MIN_ERROR_RPS);
            double f_thresh = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref_rps3), MIN_ERROR_RPS);
            int dir1, dir2, dir3;
            if (lerror >= l_thresh) {
                norm_rps1 = pid1.compute(ref_rps1 / MAX_RPS1, omega1 / MAX_RPS1);
                if (norm_rps1 < 0) dir1 = BACKWARD;
                else dir1 = FORWARD;
                pwm1 = computePWMFromNormedRPS(norm_rps1);
            } else pwm1 = motor1.getPWM();

            if (rerror >= r_thresh) {
                norm_rps2 = pid2.compute(ref_rps2 / MAX_RPS2, omega2 / MAX_RPS2);
                if (norm_rps2 < 0) dir2 = BACKWARD;
                else dir2 = FORWARD;
                pwm2 = computePWMFromNormedRPS(norm_rps2);
            } else pwm2 = motor2.getPWM();

            if (ferror >= f_thresh) {
                norm_rps3 = pid3.compute(ref_rps3 / MAX_RPS3, omega3 / MAX_RPS3);
                if (norm_rps3 < 0) dir3 = BACKWARD;
                else dir3 = FORWARD;
                pwm3 = computePWMFromNormedRPS(norm_rps3);
            } else pwm3 = motor3.getPWM();

            motor1.setPWM(pwm1); motor2.setPWM(pwm2); motor3.setPWM(pwm3);
            setThreeMotors(pwm1, dir1, pwm2, dir2, pwm3, dir3);

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
                return false;
            }

            int remaining_delay = static_cast<int>(smpl_itv * 1000) - millis() + loop_start;
            if (remaining_delay > 0) {
                delay(remaining_delay);
            }
        }
        return true;
    }
};

#endif // MOTOR_CONTROL_ALGO
#endif // MOTOR_CONTROL_HPP