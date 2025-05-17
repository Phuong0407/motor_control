#include "motorcontroller.hpp"

int main() {
    // Example usage of MotorController
    MotorController motorController;
    motorController.setMotorController(
        5.0, 0.5, 0.01, 4.0, 0.86,
        5.0, 0.5, 0.01, 4.0, 0.86,
        5.0, 0.5, 0.01, 4.0, 0.86
    );
    motorController.setMotor1Reference(0.625);
    motorController.setMotor2Reference(0.625);
    motorController.setMotor3Reference(0.625);

    // motorController.controlMotor1();
    motorController.controlMotor2();
    return 0;
}