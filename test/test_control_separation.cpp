#include "motor_control.hpp"

int main() {
    
    MotorControl controller(6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    controller.controlMotor2(0.625);
    controller.stopMotors();
    return 0;
}