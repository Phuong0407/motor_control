#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "encoder.hpp"
#include "pid_controler.hpp"

#include <cmath>
#include <chrono>

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
    EncoderManager encoder_manager;

    void initMotor(int driver1_addr = 0x0f, int driver2_addr = 0x0d) {
        i2c_fd[0] = wiringPiI2CSetup(0x0f);
        i2c_fd[1] = wiringPiI2CSetup(0x0d);
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
    pid3(kp, kd, ki),
    encoder_manager(3)
    {
        i2c_fd[0] = i2c_fd[1] = -1;
        ref_omega[0] = ref_omega[1] = ref_omega[2] = 0.0;
        measured_omega[0] = measured_omega[1] = measured_omega[2] = 0.0;        
        controlled_omega[0] = controlled_omega[1] = controlled_omega[2] = 0.0;        
        previous_ticks[0] = previous_ticks[1] = previous_ticks[2] = 0;    
        current_ticks[0] = current_ticks[1] = current_ticks[2] = 0;
        previous_cmd[0] = previous_cmd[1] = previous_cmd[2] = 0;

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
        previous_ticks[0] = encoder1->getCounter();
        previous_ticks[1] = encoder2->getCounter();
        previous_ticks[2] = encoder3->getCounter();
    }
    inline void getCurrentTicks() {
        current_ticks[0] = encoder1->getCounter();
        current_ticks[1] = encoder2->getCounter();
        current_ticks[2] = encoder3->getCounter();
    }
    inline void measureAngularVelocity(double dt) {
        measured_omega[0] = (current_ticks[0] - previous_ticks[0]) / dt;
        measured_omega[1] = (current_ticks[1] - previous_ticks[1]) / dt;
        measured_omega[2] = (current_ticks[2] - previous_ticks[2]) / dt;
    }

    void control() {
        getPreviousTicks();
        auto t0 = std::chrono::steady_clock::now();
        double dt1 = 0.5;
        double dt2 = 0.1;
        getCurrentTicks();
        measureAngularVelocity(dt1);
        controlled_omega[0] = pid1.compute(measured_omega[0], dt2);
        controlled_omega[1] = pid2.compute(measured_omega[1], dt2);
        controlled_omega[2] = pid3.compute(measured_omega[2], dt2);
    }
};

#endif // MOTOR_DRIVER_HPP