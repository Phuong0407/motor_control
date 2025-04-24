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

void handleEncoderInterrupt(int pin);

class MotorEncoder {
private:
    int H1_PIN, H2_PIN;
    std::atomic<int64_t> counter;

    void updateCounter() {
        if (digitalRead(H2_PIN)) {
            ++counter;
        } else {
            --counter;
        }
    }

    friend void handleEncoderInterrupt(int pin);

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

        {
            std::lock_guard<std::mutex> lock(MotorEncoderInstancesMutex);
            MotorEncoderInstances[H1_PIN] = this;
        }

        attachEncoderInterrupt(H1_PIN);
        printf("[INFO] Encoder ISR initialized on pins H1= %d, H2 = %d\n", H1_PIN, H2_PIN);
    }

    void resetCounter() { counter.store(0); }

    double measureShaftPosition() const {
        return static_cast<double>(counter.load()) * ANGLE_PER_TICK;
    }

    int64_t getCounter() const {
        return counter.load();
    }
};

inline std::map<int, MotorEncoder*> MotorEncoderInstances;
inline std::mutex MotorEncoderInstancesMutex;

inline void handleEncoderInterrupt(int pin) {
    std::lock_guard<std::mutex> lock(MotorEncoderInstancesMutex);
    if (MotorEncoderInstances.find(pin) != MotorEncoderInstances.end()) {
        MotorEncoderInstances[pin]->updateCounter();
    }
}

inline void encoderISR_0() { handleEncoderInterrupt(0); }
inline void encoderISR_1() { handleEncoderInterrupt(1); }
inline void encoderISR_2() { handleEncoderInterrupt(2); }
inline void encoderISR_3() { handleEncoderInterrupt(3); }
inline void encoderISR_4() { handleEncoderInterrupt(4); }
inline void encoderISR_5() { handleEncoderInterrupt(5); }
inline void encoderISR_6() { handleEncoderInterrupt(6); }
inline void encoderISR_7() { handleEncoderInterrupt(7); }

inline void attachEncoderInterrupt(int pin) {
    switch (pin) {
        case 0: wiringPiISR(0, INT_EDGE_BOTH, encoderISR_0); break;
        case 1: wiringPiISR(1, INT_EDGE_BOTH, encoderISR_1); break;
        case 2: wiringPiISR(2, INT_EDGE_BOTH, encoderISR_2); break;
        case 3: wiringPiISR(3, INT_EDGE_BOTH, encoderISR_3); break;
        case 4: wiringPiISR(4, INT_EDGE_BOTH, encoderISR_4); break;
        case 5: wiringPiISR(5, INT_EDGE_BOTH, encoderISR_5); break;
        case 6: wiringPiISR(6, INT_EDGE_BOTH, encoderISR_6); break;
        case 7: wiringPiISR(7, INT_EDGE_BOTH, encoderISR_7); break;
        default:
            throw std::invalid_argument("Unsupported pin for encoder ISR registration.");
    }
}

#endif // MOTOR_ENCODER_HPP 
