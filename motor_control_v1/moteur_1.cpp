


#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>

int main() {
    std::cout << "Starting motor test..." << std::endl;

    wiringPiSetup();
    std::cout << "WiringPi initialized." << std::endl;

    int fd = wiringPiI2CSetup(0x0f);
    if (fd == -1) {
        std::cerr << "Failed to init I2C. Check wiring and address!" << std::endl;
        return 1;
    }

    std::cout << "I2C device found. Writing to motor registers..." << std::endl;

    wiringPiI2CWriteReg16(fd, 0x82, 0xffff);

    std::cout << "Commands sent!" << std::endl;
    
    delay(5000); // Let motors spin for 2 seconds

    // Stop motors (optional, set speed to 0)
    wiringPiI2CWriteReg16(fd, 0x82, 0x0000);

    return 0;
}
