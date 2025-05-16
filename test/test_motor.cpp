#define NOLOADED_RUN
#include "../include/motor/motor.hpp"
#include <iostream>

#define TEST_MOTOR

#ifdef TEST_PID

int main() {
    Motor motor;
    motor.setPIDParameters(6.0, 0.5, 0.01, 1.0);
    bool control_state = motor.controlAngularVelocity(0.625, 0.625, 0.0);
    if (control_state) {
        printf("\nMotor control successful.\n");
    } else {
        printf("\nMotor control failure.\n");
    }

    motor.~Motor();
    return 0;
}
#endif

#ifdef TEST_MOTOR
int main() {
    startEncoders();

    // setThreeMotors(255, 1, 255, 1, 200, LEFT);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0xffff);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, 0x06);
    delay(5000);
    setMotorPWM(0, 0, 0);
}
#endif