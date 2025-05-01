#ifndef MOTOR_ENCODER_HPP
#define MOTOR_ENCODER_HPP

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <memory>
#include <atomic>
#include <stdexcept>
#include <iostream>
#include <vector>

static constexpr int COUNTER_PER_REV = 144;
static constexpr double ANGLE_PER_TICK = 2.5;

class MotorEncoder {
private:
    int H1_PIN, H2_PIN;
    int64_t counter;

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
        digitalRead(H2_PIN) ? ++counter : --counter;
    }

    void resetCounter() {
        counter = 0;
    }

    int getH1() const { return H1_PIN; }
    int getH2() const { return H2_PIN; }
    int64_t getCounter() const { return counter; }
    double measureShaftPosition() const {
        return static_cast<double>(counter) * ANGLE_PER_TICK;
    }
};

static std::unique_ptr<MotorEncoder> encoder1 = nullptr;
static std::unique_ptr<MotorEncoder> encoder2 = nullptr;
static std::unique_ptr<MotorEncoder> encoder3 = nullptr;
static std::unique_ptr<MotorEncoder> encoder4 = nullptr;
static std::unique_ptr<MotorEncoder> encoder5 = nullptr;
static std::unique_ptr<MotorEncoder> encoder6 = nullptr;

void isr1() { if (encoder1) encoder1->updateCounter(); }
void isr2() { if (encoder2) encoder2->updateCounter(); }
void isr3() { if (encoder3) encoder3->updateCounter(); }
void isr4() { if (encoder4) encoder4->updateCounter(); }
void isr5() { if (encoder5) encoder5->updateCounter(); }
void isr6() { if (encoder6) encoder6->updateCounter(); }

class EncoderManager {
private:
    const std::vector<std::pair<int, int>> encoder_pin_table = {
        {21, 22},
        {3 , 4 },
    };

    void declareEncoders() {
        encoder1  = std::make_unique<MotorEncoder>(encoder_pin_table[0].first, encoder_pin_table[0].second);
        encoder1  = std::make_unique<MotorEncoder>(encoder_pin_table[1].first, encoder_pin_table[1].second);
    }

    void attachEncoderInterrupts() {
        if (wiringPiISR(, INT_EDGE_RISING, isr5) < 0)
            throw std::runtime_error("Failed to attach ISR to D5");
        std::cout << "[INFO] ISR attached to pin 21\n";

        if (wiringPiISR(3, INT_EDGE_RISING, isr22) < 0)
            throw std::runtime_error("Failed to attach ISR to D22");
        std::cout << "[INFO] ISR attached to pin 2\n";
    }

public:
    EncoderManager() {
        declareEncoders();
        attachEncoderInterrupts();
        encoder5->resetCounter();
        encoder22->resetCounter();
    }
};

#endif // MOTOR_ENCODER_HPP
