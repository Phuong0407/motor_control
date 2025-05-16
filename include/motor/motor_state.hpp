#ifndef MOTOR_STATE_HPP
#define MOTOR_STATE_HPP

#define MAX_PWM 255

class MotorState{
public:
    int pwm;
    int dir;
    double ref;
    double measure;
    double compute;
    double max_rps;
    double min_rps;
};

#endif // MOTOR_STATE_HPP