#ifndef MOTOR_ENCODER_HPP
#define MOTOR_ENCODER_HPP

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <memory>
#include <atomic>
#include <stdexcept>
#include <iostream>

static constexpr int COUNTER_PER_REV = 144;
static constexpr double ANGLE_PER_TICK = 2.5;

class MotorEncoder {
private:
    int H1_PIN, H2_PIN;
    std::atomic<int64_t> counter;

public:
    MotorEncoder(int H1, int H2, int I2C_ADDRESS = 0x0f)
        : H1_PIN(H1), H2_PIN(H2), counter(0)
    {
        if (wiringPiSetup() == -1)
            throw std::runtime_error("wiringPiSetup failed");

        int I2C_FD = wiringPiI2CSetup(I2C_ADDRESS);
        if (I2C_FD < 0)
            throw std::runtime_error("I2C setup failed");

        pinMode(H1_PIN, INPUT);
        pinMode(H2_PIN, INPUT);
        pullUpDnControl(H1_PIN, PUD_UP);
        pullUpDnControl(H2_PIN, PUD_UP);

        if (wiringPiI2CRead(I2C_FD) == -1)
            std::cerr << "[WARNING] I2C device is not responding.\n";

        std::cout << "[INFO] Encoder initialized on pins H1 = " 
                  << H1_PIN << ", H2 = " << H2_PIN << "\n";
    }

    void updateCounter() {
        std::cout << "Enter the update counter of " << H1_PIN << "\n";
        digitalRead(H2_PIN) ? ++counter : --counter;
    }

    int getH1() const { return H1_PIN; }
    int getH2() const { return H2_PIN; }
    int64_t getCounter() const { return counter.load(); }
    double measureShaftPosition() const {
        return static_cast<double>(counter.load()) * ANGLE_PER_TICK;
    }
};

static std::unique_ptr<MotorEncoder> encoder5 = nullptr;
static std::unique_ptr<MotorEncoder> encoder22 = nullptr;

void isr5()  { if (encoder5)  encoder5->updateCounter(); }
void isr22() { if (encoder22) encoder22->updateCounter(); }

class EncoderManager {
private:
    void declareEncoders() {
        encoder5  = std::make_unique<MotorEncoder>(21, 22);
        encoder22 = std::make_unique<MotorEncoder>(2, 3);
    }

    void attachEncoderInterrupts() {
        if (wiringPiISR(21, INT_EDGE_RISING, isr5) < 0)
            throw std::runtime_error("Failed to attach ISR to pin 5");
        std::cout << "[INFO] ISR attached to pin 5\n";

        // if (wiringPiISR(22, INT_EDGE_RISING, isr22) < 0)
        //     throw std::runtime_error("Failed to attach ISR to pin 22");
        // std::cout << "[INFO] ISR attached to pin 22\n";
    }

public:
    EncoderManager() {
        declareEncoders();
        attachEncoderInterrupts();
    }
};

#endif // MOTOR_ENCODER_HPP