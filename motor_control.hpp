#ifndef MOTOR_CONTROL_HPP
#include "encoder.hpp"
#include "pid_controler.hpp"

enum DIRECTION{
    LEFT,
    RIGHT,
    FOWARD,
    BACKWARD
};
class MotorControl {
    private:
        int speed;
        DIRECTION dir;

    public:

    
};

#endif // MOTOR_CONTROL_HPP