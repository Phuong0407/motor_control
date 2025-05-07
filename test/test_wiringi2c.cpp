#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>

int main() {
    int i2c_fd = wiringPiI2CSetup(0x0f);
    std::cout << wiringPiI2CWriteReg16(i2c_fd, 0x82, 0xffff) << "\n";
    std::cout << wiringPiI2CWriteReg16(i2c_fd, 0xaa, 0x0a) << "\n";

    delay(5000);
    wiringPiI2CWriteReg16(i2c_fd, 0x82, 0x0000);
}
