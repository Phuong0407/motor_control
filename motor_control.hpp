#ifndef MOTOR_CONTROL_HPP
#include "encoder.hpp"
#include "pid_controler.hpp"

// CALIBRATION RESULT
// 0xffff
// 1000 ms = -86, 90
// 200 ms  = -10, 10

class MotorControl {
private:
    double dt;
    int pwm_out;
    PIDController pid;
    int i2c_fd;
    int number_motors;
    EncoderManager encoder_manager;
    std::vector<int64_t> ref_tick;
    
    void setMotorTicks(const std::vector<int> &ticks, const std::vector<int> &direction) {
        for (unsigned int i = 0; i < number_motors; i = i + 2) {
            int ms_left  = (ticks[i] / 86) * 1000 + (ticks[i] % 86) / 10 * 200;
            int ms_right = ticks[i + 1] / 90 * 1000 + (ticks[i + 1] % 86) / 10 * 200;
            if (ms_left >= ms_right) {
                wiringPiI2CWriteReg16(i2c_fd, 0x82, 0xffff);
                wiringPiI2CWriteReg16(i2c_fd, 0xaa, 0x06);
                delay(ms_right);
                wiringPiI2CWriteReg16(i2c_fd, 0x82, 0xff00);
                wiringPiI2CWriteReg16(i2c_fd, 0xaa, 0x06);
                delay(ms_left - ms_right);
            } else {
                wiringPiI2CWriteReg16(i2c_fd, 0x82, 0xffff);
                wiringPiI2CWriteReg16(i2c_fd, 0xaa, 0x06);
                delay(ms_left);
                wiringPiI2CWriteReg16(i2c_fd, 0x82, 0x00ff);
                wiringPiI2CWriteReg16(i2c_fd, 0xaa, 0x06);
                delay(ms_right - ms_left);
            }
            wiringPiI2CWriteReg16(i2c_fd, 0x82, 0x0000);
        }
    }

public:
    MotorControl(
        int number_motors,
        double kp,
        double ki,
        double kd,
        double dt
    ) :
    i2c_fd(-1), dt(dt), pwm_out(0),
    pid(kp, kd, ki),
    number_motors(number_motors),
    encoder_manager(number_motors),
    ref_tick(0)
    {}

    void setReferenceTick(std::vector<int64_t> &ref_tick) {
        this->ref_tick = std::move(ref_tick);
    }

    void initMotor() {
        this->i2c_fd = wiringPiI2CSetup(0x0f);
    }

    void setMotorTick() {
        std::vector<int64_t> motor_counter;
        while(std::abs(1 - ref_tick[0]) <= 10) {
            encoder_manager.getMotorCounter(motor_counter);
            double computed_tick = pid.compute(static_cast<double>(motor_counter[0]), dt);
            // clamped it to nearest integer;
            setMotor();
        }

    }
};

#endif // MOTOR_CONTROL_HPP