#include "encoder.hpp"

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <iostream>

void set_motor_pwm(int pwm1 = 0xff, int pwm2 = 0xff, int pwm3 = 0xff, int ms = 1000) {
    int i2c_fd1 = wiringPiI2CSetup(0x0f);
    int i2c_fd2 = wiringPiI2CSetup(0x0d);

    wiringPiI2CWriteReg16(i2c_fd1, 0x82, (pwm1 << 8) | pwm2);
    wiringPiI2CWriteReg16(i2c_fd1, 0xaa, 0x06);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, (pwm3 << 8));
    wiringPiI2CWriteReg16(i2c_fd2, 0xaa, 0x06);

    delay(ms);

    wiringPiI2CWriteReg16(i2c_fd1, 0x82, 0x0000);
    wiringPiI2CWriteReg16(i2c_fd2, 0x82, 0x0000);
}

int main(int argc, char* argv[]) {
    if (argc < 4)
        return 0;
    
    int pwm1 = std::stoi(argv[1], nullptr, 0);
    int pwm2 = std::stoi(argv[2], nullptr, 0);
    int pwm3 = std::stoi(argv[3], nullptr, 0);
    int ms   = std::stoi(argv[4]);

    declareEncoders(0xff, 0xdd);
    attachEncoderInterrupts();

    set_motor_pwm(pwm1, pwm2, pwm3, ms);
 
    std::cout << std::hex << "0x" << pwm1 << " 0x" << pwm2 << " 0x" << pwm3 << " " << std::dec << ms << " [ms].\n"; 
    for (int i = 0; i < NUM_ENCODERS; ++i) {
        std::cout << "ENCODER COUNTER " << (i + 1)
                  << " = " << encoders[i]->getCounter() << "\n";
    }
    cleanupEncoders();

    return 0;
}
