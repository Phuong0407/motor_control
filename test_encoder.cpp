#include "encoder.hpp"

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <iostream>
#include <string>

void setMotorCommand(int ms = 200, int speed = 0xffff) {
    int i2c_fd1 = wiringPiI2CSetup(0x0f);
    int i2c_fd2 = wiringPiI2CSetup(0x0d);

    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0xffff);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, 0x06);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0xffff);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, 0x06);

    delay(ms);

    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0x0000);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0x0000);
}

int main() {
    declareEncoders(0x0f, 0x0d);
    attachEncoderInterrupts();

    setMotorCommand();

    for (int i = 0; i < NUM_ENCODERS; ++i) {
        std::cout << "ENCODER COUNTER " << (i + 1)
                  << " = " << encoders[i]->getCounter() << "\n";
    }
    cleanupEncoders();
    return 0;
}
