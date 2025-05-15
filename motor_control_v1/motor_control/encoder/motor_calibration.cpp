/**
 * @brief THIS IS THE HEADER FOR THE HALL SENSOR MANIPULATION OF DG01D-E
 * @file velocity_calibration.h
 * @ingroup motor controller
 * @date April 13, 2025
 */

#include <wiringPiI2C.h>

#include "encoder.h"
#include <iostream>
#include <chrono>
#include <thread>

static int i2c_fd = -1;

//    wiringPiI2CWriteReg16(fd, 0x82, 0xafaf);
//    wiringPiI2CWriteReg16(fd, 0xaa, 0x05);



void setMotorCommand() {
    wiringPiSetup();
    i2c_fd = wiringPiI2CSetup(0x0f);
    int time_intv = 0;
    while(time_intv <= 5000) {
        wiringPiI2CWriteReg16(i2c_fd, 0x82, 0xffff);
        wiringPiI2CWriteReg16(i2c_fd, 0xaa, 0x06);
//        wiringPiI2CWriteReg16(i2c_fd, 0xa5, 0x0a);

	delay(100);
        time_intv += 100;
    }
}

void stopMotor() {
    if (i2c_fd == -1) return;
    wiringPiI2CWriteReg16(i2c_fd, 0x82, 0);
    wiringPiI2CWriteReg16(i2c_fd, 0xa1, 0x0a);
//    wiringPiI2CWriteReg16(i2c_fd, 0xa5, 0x0a);
}


int main() {
    init_encoder();

    setMotorCommand();
    double rpm = measureRPM();
    stopMotor();
    printf("%f\n", rpm);
    return 0;
}
