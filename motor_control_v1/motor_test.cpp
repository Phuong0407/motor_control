#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <unistd.h> // for sleep

#define MOTOR_ADDR 0x0f  // make sure this matches your actual driver

void setMotors(int fd, int leftSpeed, int rightSpeed) {
    // Clamp to safe limits
    leftSpeed = std::max(-200, std::min(200, leftSpeed));
    rightSpeed = std::max(-200, std::min(200, rightSpeed));

    // Determine absolute values for speed
    uint8_t l = std::abs(leftSpeed);
    uint8_t r = std::abs(rightSpeed);

    // Determine direction register value
    uint8_t dir = 0x00;

    if (leftSpeed > 0 && rightSpeed > 0) {
        dir = 0x0A;  // both forward
    } else if (leftSpeed < 0 && rightSpeed < 0) {
        dir = 0x05;  // both reverse
    } else if (leftSpeed > 0 && rightSpeed < 0) {
        dir = 0x09;  // left forward, right reverse (spin left)
    } else if (leftSpeed < 0 && rightSpeed > 0) {
        dir = 0x06;  // left reverse, right forward (spin right)
    } else if (leftSpeed == 0 && rightSpeed == 0) {
        dir = 0x00;  // stop
    } else if (leftSpeed == 0 && rightSpeed > 0) {
        dir = 0x0A;  // only right forward
    } else if (leftSpeed == 0 && rightSpeed < 0) {
        dir = 0x05;  // only right reverse
    } else if (leftSpeed > 0 && rightSpeed == 0) {
        dir = 0x0A;  // only left forward
    } else if (leftSpeed < 0 && rightSpeed == 0) {
        dir = 0x05;  // only left reverse
    }

    // Write direction
    wiringPiI2CWriteReg16(fd, 0xAA, dir);

    // Combine speeds into 16-bit value (left in high byte, right in low byte)
    uint16_t speedVal = (l << 8) | r;

    // Write speed
    wiringPiI2CWriteReg16(fd, 0x82, speedVal);
}



int main() {
    wiringPiSetup();
    int fd = wiringPiI2CSetup(MOTOR_ADDR);

    if (fd < 0) {
        std::cerr << "Failed to init I2C communication.\n";
        return -1;
    }

    std::cout << "I2C motor driver connected.\n";

   // Full speed forward
    std::cout << "Moving forward...\n";
    setMotors(fd, 200, 200);
    sleep(2);

    // Full speed reverse
    std::cout << "Moving forward...\n";
    setMotors(fd, -200, -200);
    sleep(2);

    // Turn left
    std::cout << "Turning right...\n";
    setMotors(fd, 75, 200);
    sleep(2);

    // Stop
    setMotors(fd, 0, 0);
}
