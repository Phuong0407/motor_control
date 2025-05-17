#include "motorcontroller.hpp"

int main() {
    // Example usage of MotorController
    MotorController motorController;
    motorController.setMotorController(
        6.0, 0.5, 0.05, 4.0, 0.86,
        6.0, 0.5, 0.05, 4.0, 0.86,
        6.0, 0.5, 0.05, 4.0, 0.86
    );
    motorController.setMotor1Reference(0.5);
    motorController.setMotor2Reference(-0.5);
    motorController.setMotor3Reference(0.625);

    // motorController.controlMotor1();
    // motorController.controlMotor2();
    motorController.controlMotor3();
    return 0;
}