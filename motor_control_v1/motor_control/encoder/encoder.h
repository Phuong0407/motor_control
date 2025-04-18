/**
 * @brief THIS IS THE HEADER FOR THE HALL SENSOR MANIPULATION OF DG01D-E
 * @file velocity_measurement.h
 * @ingroup motor controller
 * @date April 4, 2025
 */

#ifndef VELOCITY_MEASUREMENT_H
#define VELOCITY_MEASUREMENT_H

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <iostream>
#include <chrono>
#include <thread>

#define I2C_ADDRESS 0x0f
#define ENCODER_INPUT INPUT
#define ENCODER_OUTPUT OUTPUT
#define COUNTER_PER_TURN 144

const int H1_PIN = 21;
const int H2_PIN = 22;
volatile int counter = 0;

void setPinMode() {
    pinMode(H1_PIN, INPUT);
    pinMode(H2_PIN, INPUT);
}

void enableEncoderPullUpResistor() {
    pullUpDnControl(H1_PIN, PUD_UP);
    pullUpDnControl(H2_PIN, PUD_UP);
}

void init_encoder() {
    if (wiringPiSetup() == -1)
        throw std::runtime_error("WiringPi setup failed!");
    int i2c_fd = wiringPiI2CSetup(I2C_ADDRESS);
    if (i2c_fd < 0)
        throw std::runtime_error("I2C setup failed! Check address and wiring.");

    setPinMode();
    enableEncoderPullUpResistor();

    if (wiringPiI2CRead(i2c_fd) == -1)
        throw std::runtime_error("I2C device not responding.");
    std::cout << "[INFO] Encoder initialized successfully on pins " << H1_PIN << " and " << H2_PIN << ".\n";
}

void reset_position(void) { counter = 0; }

void update_motor_position(void) {
    uint8_t H2_STATE = digitalRead(H2_PIN);
    if (H2_STATE) {
        counter++;
    } else {
        counter--;
    }
}

double measureRPM() {
    reset_position();
    if (wiringPiISR(H2_PIN, INT_EDGE_RISING, update_motor_position) < 0)
        throw std::runtime_error("Failed to setup interrupt on H1_PIN.");

    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = start_time + std::chrono::seconds(1);

    while (std::chrono::high_resolution_clock::now() < end_time) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

//    wiringPiISRStop(H1_PIN);

    std::cout << "counter = " << counter << std::endl;
    double revolutions = static_cast<double>(counter) / COUNTER_PER_TURN;
    double rpm = revolutions * 60.0;
    return rpm;
}

#endif
