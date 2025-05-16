#define NOLOADED_RUN
#include "../include/motor/motor.hpp"
#include <iostream>

int main() {
    // Motor motor;
    // motor.setPIDParameters(6.0, 0.5, 0.01, 1.0);
    // bool control_state = motor.controlAngularVelocity(0.625, 0.625, 0.625);
    // if (control_state) {
    //     printf("\nMotor control successful.\n");
    // } else {
    //     printf("\nMotor control failure.\n");
    // }

    // wiringPiI2CWriteReg16(i2c_fd2, 0x82, (255));
    // delay(1);
    // wiringPiI2CWriteReg16(i2c_fd2, 0xaa, 0x06);

    // motor.~Motor();
    startEncoders();
    std:: cout << i2c_fd1 << std::endl;
    std:: cout << i2c_fd2 << std::endl;

    setThreeMotors(255, 1, 255, 1, 255, 1);
    delay(1000);
    setMotorPWM(0, 0, 0);

    return 0;
}
