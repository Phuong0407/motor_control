#include "encoder_v3.hpp"

void setMotorCommand() {
    int i2c_fd = wiringPiI2CSetup(0x0f);
    int counter = 0;
    while (counter <= 50) {
        wiringPiI2CWriteReg16(i2c_fd, 0x82, 0xffff);
        wiringPiI2CWriteReg16(i2c_fd, 0xaa, 0x06);
        delay(100);
        counter++;
    }
    wiringPiI2CWriteReg16(i2c_fd, 0x82, 0x0000);
}

int main() {
    EncoderManager encoder_manager;
    setMotorCommand();
    std::cout << encoder5->getCounter() << std::endl;
    std::cout << encoder22->getCounter() << std::endl;
}