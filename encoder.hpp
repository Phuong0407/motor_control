/**
 * @brief MotorUnit — Controls a motor via Grove I2C Motor Driver and measures RPM with encoder (DG01D-E)
 * @file encoder.hpp
 * @date April 18, 2025
 * 
 */

#ifndef MOTOR_ENCODER_HPP
#define MOTOR_ENCODER_HPP

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <map>
#include <mutex>
#include <chrono>
#include <thread>
#include <atomic>
#include <iostream>
#include <stdexcept>
#include <algorithm>

/**
 * @brief MotorEncoder class for handling motor encoders.
 * This class uses the WiringPi library to interface with the GPIO pins of a Raspberry Pi.
 * It provides methods to connect and measure multiple encoders at the same time
 * using @param map<int, MotorEncoder*> instance_map.
 * The key is the H1_PIN and the value is a pointer to the MotorEncoder instance.
 * @note construction attaches the motor into ISR globally.
 * @note The class is designed to be thread-safe using std::mutex and std::atomic.
 * @note The counter will increase or decrease based on the state of @param H2_PIN when @param H1_PIN is triggered.
 * @remark Motor will be controlled based on shaft angle.
 * @remark Another method based on velocity is not implemented yet.
 * @remark @param COUNTER_PER_REV is set to 144, which is specific for motor DC01D E PH.
 * @remark @param ANGLE_PER_TICK is set to 144, which is specific for motor DC01D E PH.
 */

class MotorEncoder {
private:
    static constexpr int COUNTER_PER_REV = 144;
    static constexpr double ANGLE_PER_TICK = 2.5;

    int H1_PIN, H2_PIN;
    std::atomic<int64_t> counter;

    static std::map<int, MotorEncoder*> instance_map;
    static std::mutex map_mutex;

    void updateCounter() {
        if (digitalRead(H2_PIN)) {
            counter++;
        } else {
            counter--;
        }
    }

public:
    /**
     * @brief Constructor for MotorEncoder class.
     * @param H1 Pin number for the first encoder signal.
     * @param H2 Pin number for the second encoder signal.
     * @throws std::runtime_error if wiringPi setup fails or ISR attachment fails.
     * @note pull up resistors are enabled for both pins.
     * @note lock_guard is used to ensure thread safety when accessing the instance_map.
     * @note The constructor sets the pin modes and attaches the ISR for H1_PIN.
     */
    MotorEncoder(int H1, int H2) : H1_PIN(H1), H2_PIN(H2), counter(0) {
        if (wiringPiSetup() == -1)
            throw std::runtime_error("wiringPiSetup failed!");

        pinMode(H1_PIN, INPUT);
        pinMode(H2_PIN, INPUT);
        pullUpDnControl(H1_PIN, PUD_UP);
        pullUpDnControl(H2_PIN, PUD_UP);

        {
            std::lock_guard<std::mutex> lock(map_mutex);
            instance_map[H1_PIN] = this;
        }

        if (wiringPiISR(H1_PIN, INT_EDGE_RISING, &MotorEncoder::globalISR) < 0)
            throw std::runtime_error("Failed to attach ISR to H1_PIN");
    }

    void resetCounter() {
        counter = 0;
    }

    // double measureRPM(int duration_ms = 1000) {
    //     resetCounter();
    //     auto start = std::chrono::high_resolution_clock::now();
    //     auto end = start + std::chrono::milliseconds(duration_ms);

    //     while (std::chrono::high_resolution_clock::now() < end) {
    //         std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //     }

    //     int ticks = counter.load();
    //     double revolutions = static_cast<double>(ticks) / COUNTER_PER_REV;
    //     return (revolutions * 60.0 * 1000.0) / duration_ms;
    // }

    const double measureShaftPosition() const {
        return static_cast<double>(counter.load()) * ANGLE_PER_TICK;
    }

    static void globalISR() {
        std::lock_guard<std::mutex> lock(map_mutex);
        for (const auto& [pin, instance] : instance_map) {
            if (digitalRead(pin)) {
                instance->updateCounter();
            }
        }
    }
};

std::map<int, MotorEncoder*> MotorEncoder::instance_map;
std::mutex MotorEncoder::map_mutex;

#endif // MOTOR_ENCODER_HPP