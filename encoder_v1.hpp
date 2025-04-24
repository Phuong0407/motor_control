/**
 * @brief MotorUnit — Controls a motor via Grove I2C Motor Driver and measures RPM with encoder (DG01D-E)
 * @file encoder.hpp
 * @date April 24, 2025
 */

#ifndef MOTOR_ENCODER_HPP
#define MOTOR_ENCODER_HPP

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <map>
#include <mutex>
#include <atomic>
#include <stdexcept>
#include <stdio.h>
#include <iostream>

static constexpr int COUNTER_PER_REV = 144;
static constexpr double ANGLE_PER_TICK = 2.5;

std::map<int, MotorEncoder*> MotorEncoderInstances;

class MotorEncoder {
private:
    int H1_PIN, H2_PIN;
    std::atomic<int64_t> counter;

public:
    MotorEncoder(int H1, int H2, int I2C_ADDRESS = 0x0f)
        : H1_PIN(H1), H2_PIN(H2), counter(0) {
        
        if (wiringPiSetup() == -1)
            throw std::runtime_error("[ERROR] wiringPiSetup failed!");

        int I2C_FD = wiringPiI2CSetup(I2C_ADDRESS);
        if (I2C_FD < 0)
            throw std::runtime_error("[ERROR] I2C setup failed! Check address/wiring.");

        pinMode(H1_PIN, INPUT);
        pinMode(H2_PIN, INPUT);
        pullUpDnControl(H1_PIN, PUD_UP);
        pullUpDnControl(H2_PIN, PUD_UP);

        printf("[INFO] Encoder ISR initialized on pins H1= %d, H2 = %d\n", H1_PIN, H2_PIN);
    }

    void resetCounter() { counter.store(0); }

    void updateCounter() {
        if (digitalRead(H2_PIN)) {
            ++counter;
        } else {
            --counter;
        }
    }

    double measureShaftPosition() const {
        return static_cast<double>(counter.load()) * ANGLE_PER_TICK;
    }

    int64_t getCounter() const {
        return counter.load();
    }
};

// add here function to call updateCounter() when interrupt occurs
// TODO 

void attachEncoderInterrupt() {
    for (const auto& motor : MotorEncoderInstances) {
        wiringPiISR(motor.first, INT_EDGE_RISING, void (*function)(void));
    }

}


#endif // MOTOR_ENCODER_HPP 
