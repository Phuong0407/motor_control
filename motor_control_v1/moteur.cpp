#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>

int counter = 0;
const int pulses_per_second = 45;
const float Kp = 1;
const float Kd = 0.05;  // Derivative gain
const float Ki = 0.01;  // Integral gain

volatile int last_counter = 0;
float integral = 0.0;
float last_error = 0.0;

void count() {
    counter++;
}

int main() {

    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize WiringPi!" << std::endl;
        return 1;
    }

    int fd = wiringPiI2CSetup(0x0f);
    if (fd == -1) {
        std::cerr << "Failed to initialize I2C communication!" << std::endl;
        return 1;
    }

    wiringPiI2CWriteReg16(fd, 0x82, 0xafaf);
    wiringPiI2CWriteReg16(fd, 0xaa, 0x05);

    int encoderPin = 5;
    wiringPiISR(encoderPin, INT_EDGE_RISING, &count);


    unsigned long start_time = millis();
    unsigned long last_time = start_time;
    unsigned long elapsed_time = 0;

    while (millis() - start_time < 15000) {
        unsigned long current_time = millis();
        elapsed_time = current_time - last_time;

        if (elapsed_time >= 100) { 
            int current_speed = counter - last_counter;
            float error = pulses_per_second / 10 - current_speed;

            integral += error * (elapsed_time / 100.0);
            float derivative = (error - last_error) / (elapsed_time / 100.0);
            float control_signal = Kp * error + Ki * integral + Kd * derivative;

            float new_speed = 0.05 + control_signal;
            new_speed = std::max(0.0f, std::min(new_speed, 1.0f));
            
            int speed_int = static_cast<int>(new_speed * 255);

            wiringPiI2CWriteReg16(fd, 0x82, 0xff00 | speed_int);

            last_counter = counter;
            last_error = error;
            last_time = current_time;
        }

        delay(10);
    }

    wiringPiI2CWriteReg16(fd, 0x82, 0x0000);
    std::cout << "Motor stopped." << std::endl;
    return 0;
}

