#define NOLOADED_RUN
#include "../include/motor/motor.hpp"
#include <iostream>

int main() {
    Motor motor;
    motor.setPIDParameters(6.0, 0.5, 0.01, 1.0);
    bool control_state = motor.controlAngularVelocity(0.625, 0.625, 0.0);
    if (control_state) {
        printf("\nMotor control successful.\n");
    } else {
        printf("\nMotor control failure.\n");
    }

    wiringPiI2CWriteReg16(i2c_fd2, 0x82, (255));
    delay(1);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, 0x06);

    motor.~Motor();

    return 0;
}
