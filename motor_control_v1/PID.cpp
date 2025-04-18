/**
 * @brief MotorUnit — Controls a motor via Grove I2C Motor Driver and measures RPM with encoder (DG01D-E)
 * @brief This will control the motor by a PID, need FINE TUNNING of parameters

 * @file motor_unit.h
 * @date April 16, 2025
 */

#ifndef MOTOR_UNIT_H
#define MOTOR_UNIT_H

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <map>
#include <atomic>
#include <mutex>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <iostream>
#include <cmath>
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



class PIDController {
    private:
        double kp, ki, kd;
        double integral;
        double previous_error;
        double output_min, output_max;

    public:
        PIDController(double kp, double ki, double kd, double out_min = -255, double out_max = 255)
            : kp(kp), ki(ki), kd(kd), integral(0), previous_error(0), output_min(out_min), output_max(out_max) {}

        double compute(double setpoint, double measured, double dt) {
            double error = setpoint - measured;
            integral += error * dt;
            double derivative = (error - previous_error) / dt;
            previous_error = error;

            double output = kp * error + ki * integral + kd * derivative;
            return std::clamp(output, output_min, output_max);
        }

        void reset() {
            integral = 0;
            previous_error = 0;
        }
};



class MotorControler {
private:
    MotorEncoder encoder;
    int I2C_FD;
    const double MAX_RPM = 35.0;

    int getCMDFromRPM(double rpm) {
        const double MAX_RPM = 35.0;
        double clamped = std::clamp(std::abs(rpm), 0.0, MAX_RPM);
        return static_cast<int>((clamped / MAX_RPM) * 255.0);
    }


public:
    MotorControler(int H1, int H2) : encoder(H1, H2) {
        I2C_FD = wiringPiI2CSetup(0x0f);
        if (I2C_FD < 0)
            throw std::runtime_error("Failed to setup I2C for motor driver");
    }

    void setBothMotorsRPM(int rpm1, int rpm2) {
        uint8_t direction;
        if (rpm1 >= 0 && rpm2 >= 0) {
            direction = 0x0A;
        } else if (rpm1 < 0 && rpm2 < 0) {
            direction = 0x05;
        } else if (rpm1 >= 0 && rpm2 < 0) {
            direction = 0x06;
        } else {
            direction = 0x09;
        }

        int speed1 = std::min(255, std::abs(getCMDFromRPM(rpm1)));
        int speed2 = std::min(255, std::abs(getCMDFromRPM(rpm2)));
        uint16_t combinedSpeed = (speed1 << 8) | speed2;

        wiringPiI2CWriteReg16(I2C_FD, 0x82, combinedSpeed);
        wiringPiI2CWriteReg16(I2C_FD, 0xAA, direction);
    }

    void stop() {
        wiringPiI2CWriteReg16(I2C_FD, 0x82, 0);
    }

    double measureRPM(int duration_ms = 1000) {
        return encoder.measureRPM(duration_ms);
    }

    void runDualPIDControl(double target_rpm1, double target_rpm2, int duration_ms = 3000) {
        PIDController pid1(4.0, 0.5, 0.2);
	PIDController pid2(4.0, 0.5, 0.2);

        const int interval_ms = 100;
        int elapsed = 0;

        while (elapsed < duration_ms) {
            auto start = std::chrono::steady_clock::now();

            double actual_rpm1 = encoder.measureRPM(interval_ms);
            double actual_rpm2 = actual_rpm1;

            double control_rpm1 = pid1.compute(target_rpm1, actual_rpm1, interval_ms / 1000.0);
            double control_rpm2 = pid2.compute(target_rpm2, actual_rpm2, interval_ms / 1000.0);

            setBothMotorsRPM(static_cast<int>(control_rpm1), static_cast<int>(control_rpm2));

            auto end = std::chrono::steady_clock::now();
            elapsed += std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        }

        stop();
    }

};

#endif // MOTOR_UNIT_H
