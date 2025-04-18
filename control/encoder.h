/**
 *
 * @brief MotorUnit — Controls a motor via Grove I2C Motor Driver and measures RPM with encoder (DG01D-E)
 * @file motor_unit.h
 * @date April 18, 2025
 *
 */

#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <WiringPi.h>
#include <WiringPiI2C.h>

#include <stdio.h>
#include <map>
#include <thread>
#include <mutex>
#include <stdexcept>
#include <atom>
#include <algorithm>


class MotorEncoder {
private:
    int H1_PIN, H2_PIN;
    std::atomic<int> counter;

    static std::map<int, MotorEncoder*> instance_map; // [H1_PIN, MotorEncoderID (pointer)]
    static std::mutex map_mutex;

public:
    static constexpr int COUNTER_PER_REV = 144; // For DG01D-E

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

    double measureRPM(int duration_ms = 1000) {
        resetCounter();
        auto start = std::chrono::high_resolution_clock::now();
        auto end = start + std::chrono::milliseconds(duration_ms);

        while (std::chrono::high_resolution_clock::now() < end) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        int ticks = counter.load();
        double revolutions = static_cast<double>(ticks) / COUNTER_PER_REV;
        return (revolutions * 60.0 * 1000.0) / duration_ms;
    }

    void updateCounter() {
        if (digitalRead(H2_PIN)) {
            counter++;
        } else {
            counter--;
        }
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






#endif // MOTOR_ENCODER_H
