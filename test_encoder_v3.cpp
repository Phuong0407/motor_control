#include "encoder.hpp"

void setMotorCommand(int ms = 1000) {
    int i2c_fd = wiringPiI2CSetup(0x0f);
    wiringPiI2CWriteReg16(i2c_fd, 0x82, 0xffff);
    wiringPiI2CWriteReg16(i2c_fd, 0xaa, 0x06);
    delay(ms);
    wiringPiI2CWriteReg16(i2c_fd, 0x82, 0x0000);
}

int main() {
    EncoderManager encoder_manager;
    setMotorCommand();
    std::cout << "After run motor" << "\n";
    std::cout << "ENCODER COUNTER 1 = " << encoder1->getCounter() << "\n";
    std::cout << "ENCODER COUNTER 2 = " << encoder2->getCounter() << "\n";
}
