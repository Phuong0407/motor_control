#include <wiringPi.h>
#include <wiringPiI2C.h>

int main() {
    wiringPiSetup();
    int fd = wiringPiI2CSetup(0x0f);
    if (fd == -1) return 1;

    // Set motor speed to max (assuming 16-bit PWM)
    wiringPiI2CWriteReg16(fd, 0x82, 0xFFFF);

    delay(50); // Let the speed settle

    // Set motor direction / enable (assuming 0x05 is forward)
    wiringPiI2CWriteReg16(fd, 0xAA, 0x06);

    delay(5000); // Let motors spin for 2 seconds

    // Stop motors (optional, set speed to 0)
    wiringPiI2CWriteReg16(fd, 0x82, 0x0000);

    return 0;
}
