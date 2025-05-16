#include "motor_control.hpp"

int main() {
    
    MotorControl controller(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    controller.controlMotor1(0.625);
    controller.stopMotors();
    return 0;
}