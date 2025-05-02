#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "encoder.hpp"
#include "pid_controler.hpp"

#include <cmath>

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
    const static int number_motors = 3;
    EncoderManager encoder_manager;
    bool loaded_run;
    double dt;
    PIDController pid;
    int i2c_fd1;
    int i2c_fd2;
    double ref_omega_1;
    double ref_omega_2;
    double ref_omega_4;
    int pwm_out;

    void initMotor(int driver1_addr = 0x0f, int driver2_addr = 0x0d) {
        this->i2c_fd1 = wiringPiI2CSetup(0x0f);
        this->i2c_fd2 = wiringPiI2CSetup(0x0d);
    }

public:
    MotorDriver(
        int number_motors,
        bool loaded_run,
        double kp,
        double ki,
        double kd,
        double dt,
        int driver1_addr = 0x0f,
        int driver2_addr = 0x0d
    ) :
    encoder_manager(number_motors),
    loaded_run(loaded_run),
    dt(dt),
    pid(kp, kd, ki),
    i2c_fd1(-1),
    i2c_fd2(-1),
    ref_tick1(0),
    ref_tick2(0),
    ref_tick3(0),
    pwm_out(0)
    {
        initMotor(driver1_addr, driver2_addr);
    }

    void setReferenceTick(int64_t ref_tick1, int64_t ref_tick2, int64_t ref_tick3) {
        this->ref_tick1 = ref_tick1;
        this->ref_tick2 = ref_tick2;
        this->ref_tick3 = ref_tick3;
    }

    void setMotorTick() {

    }
};

#endif // MOTOR_DRIVER_HPP