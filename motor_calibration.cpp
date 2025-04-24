#include "encoder.hpp"

void setMotorCommand() {
    int i2c_fd = wiringPiI2CSetup(0x0f);

    wiringPiI2CWriteReg16(i2c_fd, 0x82, 0x255);
    wiringPiI2CWriteReg16(i2c_fd, 0xaa, 0x06);
}

int main() {
    MotorEncoder encoder1(21, 22);
    MotorEncoder encoder2(2, 3);

    setMotorCommand();
    
    std::cout << "counter 1 = " << encoder1.getCounter() << std::endl;
    std::cout << "counter 2 = " << encoder2.getCounter() << std::endl;
}