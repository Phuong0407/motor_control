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
static constexpr int LEFT               = +1;
static constexpr int RIGHT              = -1;

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

int computeDirection(int dir) {
    if (dir == LEFT)
        return 0x06;
    if (dir == RIGHT)
        return 0x05;
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
    int dir12 = computeDirection(dir1, dir2);
    dir3 = computeDirection(dir3);
    
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
    delay(1);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, dir12);
    delay(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, (pwm3 << 8));
    delay(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, dir3);
    printf("i2c_fd1: %d, i2c_fd2: %d\n", i2c_fd1, i2c_fd2);
}

void stopMotors() {
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

static double constexpr TIMEOUT_SECS = 20.0;

class MotorState {
private:
    int pwm;
    int dir;

public:
    MotorState() {
        pwm = 0;
        dir = FORWARD;
    }

    inline int getPWM() const { return pwm; }
    inline int getDir() const { return dir; }
    inline void setPWM(int pwm) { this->pwm = pwm; }
    inline void setDir(int pwm) { this->dir = dir; }
    inline void setMotorState(int pwm, int dir) {
        this->pwm = pwm;
        this->dir = dir;
    }
};


static constexpr double MIN_ERROR_RPS           = 0.08;
static constexpr double ERROR_THRESHOLD_PERCENT = 0.10;
static constexpr int STABLE_CYCLES_REQUIRED     = 3;

class Motor {
private:
    double MAX_RPS[3] = {MAX_RPS1, MAX_RPS2, MAX_RPS3};
    PID pid[3];
    MotorState motor[3];

    bool computeMotorState(
        int motor_id,
        double ref_rps,
        double omega,
        int &pwm,
        int &dir
    )
    {
        double err = std::abs(ref_rps - omega);
        double err_thres = std::max(ERROR_THRESHOLD_PERCENT * std::abs(ref_rps), MIN_ERROR_RPS);
        if (err > err_thres) {
            double norm_rps = pid[motor_id].compute(ref_rps / MAX_RPS[motor_id], omega / MAX_RPS[motor_id]);
            pwm = computePWMFromNormedRPS(norm_rps);
            if (norm_rps < 0) dir = BACKWARD;
            else dir = FORWARD;
            return false;
        } else {
            pwm = motor[motor_id].getPWM();
            dir = motor[motor_id].getDir();
            return true;
        }
        return true;
    }

    void updateStblCycCounter(int motor_id, bool stble, int *StbleCycCount) {
        if (stble)
            StbleCycCount++;
        else
            StbleCycCount = 0;
    }

    void printStableLog(int motor_id, int &stblCycCount, double &stble_time, double elapsed_time, bool &logged) {
        if (stblCycCount == STABLE_CYCLES_REQUIRED && !logged) {
            stble_time = elapsed_time;
            printf("[INFO] Motor %d stabilized after %.1f seconds.\n", motor_id + 1, stble_time);
            logged = true;
        }
    }

public:
    Motor() { startEncoders(); }

    void setPIDParameters(double kp, double ki, double kd, double max_out) {
        pid[0] = PID(kp, ki, kd, max_out, cutoff_freq, smpl_itv);
        pid[1] = PID(kp, ki, kd, max_out, cutoff_freq, smpl_itv);
        pid[2] = PID(kp, ki, kd, max_out, cutoff_freq, smpl_itv);
    }

    bool controlAngularVelocity(double ref_rps1, double ref_rps2, double ref_rps3) {
        double omega[3];
        int pwm[3], dir[3];

        int StbleCycCount[3] = {0, 0, 0};
        bool stble[3] = {false, false, false};
        double ref_rps[3] = {ref_rps1, ref_rps2, ref_rps3};
        double stble_time[3] = {0.0, 0.0, 0.0};
        bool logState[3] = {false, false, false};
        printf("rps1: %.3f, rps2: %.3f, rps3: %.3f\n", ref_rps1, ref_rps2, ref_rps3);
        uint64_t start_time = millis();
        while (
            StbleCycCount[0] < STABLE_CYCLES_REQUIRED ||
            StbleCycCount[1] < STABLE_CYCLES_REQUIRED ||
            StbleCycCount[2] < STABLE_CYCLES_REQUIRED)
        {
            uint64_t loop_start = millis();

            measureAngularVelocity(omega[0], omega[1], omega[2], smpl_itv);
            for (int i = 0; i < 3; i++) {
                stble[i] = computeMotorState(i, ref_rps[i], omega[i], pwm[i], dir[i]);
                motor[i].setMotorState(pwm[i], dir[i]);
            }
            printf("\n");
            setThreeMotors(pwm[0], dir[0], pwm[1], dir[1], pwm[2], dir[2]);
            printf("[INFO] Motor pwm: pwm %d, pwm %d, pwm %d\t", motor[0].getPWM(), motor[1].getPWM(), motor[2].getPWM());
            printf("[INFO] Motor dir: dir %d, dir %d, dir %d\n", motor[0].getDir(), motor[1].getDir(), motor[2].getDir());

            uint64_t current_time = millis();
            double elapsed_time = (current_time - start_time) / 1000.0;
            
            for (int i = 0; i < 3; i++) {
                updateStblCycCounter(i, stble[i], &StbleCycCount[i]);
                if (StbleCycCount[i] == STABLE_CYCLES_REQUIRED) {
                    printStableLog(i, StbleCycCount[i], stble_time[i], elapsed_time, logState[i]);
                }
            }
            if (elapsed_time >= TIMEOUT_SECS) {
                printf("[WARNING] Motor control timed out after %.1f seconds.\n", TIMEOUT_SECS);
                return false;
            }
        }
        return true;
    }
    ~Motor() {
        stopMotors();
        clearEncoders();
    }
};

#endif // MOTOR_CONTROL_ALGO
#endif // MOTOR_CONTROL_HPP