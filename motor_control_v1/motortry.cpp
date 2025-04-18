#include <wiringPi.h>
#include <wiringPiI2C.h>

int main() {
    wiringPiSetup();
    int fd = wiringPiI2CSetup(0x0f);
    
    int vitesse = 0x255;

    wiringPiI2CWriteReg16(fd, 0x82, vitesse);
    wiringPiI2CWriteReg16(fd, 0xaa, 0x06);

    
 /*int vitesse = 0x0000;
    while (vitesse < 0xffff) {
        vitesse += 0x0101;
        if (vitesse > 0xffff) vitesse = 0xffff;
        wiringPiI2CWriteReg16(fd, 0x82, vitesse);
        delay(100);
    }*/

    delay(3000);
    wiringPiI2CWriteReg16(fd, 0x82, 0x0000);


    return 0;
}
