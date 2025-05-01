#include <wiringPi.h>
#include <wiringPiI2C.h>

int main() {
    wiringPiSetup();

    // First driver at address 0x0F (controls wheel 1 and 2)
    int fd1 = wiringPiI2CSetup(0x0F);

    // Second driver at address 0x0E (controls wheel 3)
    int fd2 = wiringPiI2CSetup(0x0E);

    // Set speed and direction for driver 1
    wiringPiI2CWriteReg16(fd1, 0x82, 0xffff); // speed
    wiringPiI2CWriteReg16(fd1, 0xAA, 0x06);   // direction

    // Set speed and direction for driver 2 (wheel 3)
    wiringPiI2CWriteReg16(fd2, 0x82, 0xffff); // speed
    wiringPiI2CWriteReg16(fd2, 0xAA, 0x06);   // direction for M1 only

    delay(3000);

    // Stop all motors
    wiringPiI2CWriteReg16(fd1, 0x82, 0x0000);
    wiringPiI2CWriteReg16(fd2, 0x82, 0x0000);

    return 0;
}
