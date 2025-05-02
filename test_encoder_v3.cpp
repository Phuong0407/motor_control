#include "encoder.hpp"

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <iostream>
#include <string>

void setMotorCommand(int ms = 1000, int speed = 0xffff) {
    int i2c_fd1 = wiringPiI2CSetup(0x0f);
    int i2c_fd2 = wiringPiI2CSetup(0x0d);

    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0x0000);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, 0x06);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0xffff);
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, 0x06);

    delay(ms);
    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0x0000);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0x0000);
}

int main(int argc, char* argv[]) {
    int ms = std::stoi(argv[1]);
    int speed = std::stoi(argv[2], nullptr, 0);

//int main() {
    EncoderManager encoder_manager(3);
    setMotorCommand(ms, speed);

    setMotorCommand();

    std::cout << "Run motor at speed 0x" << std::hex << speed << std::dec << " during " << ms << " [ms]" << "\n";
    std::cout << "After run motor" << "\n";
    std::cout << "ENCODER COUNTER 1 = " << encoder1->getCounter() << "\n";
    std::cout << "ENCODER COUNTER 2 = " << encoder2->getCounter() << "\n";
    std::cout << "ENCODER COUNTER 3 = " << encoder3->getCounter() << "\n";
}
