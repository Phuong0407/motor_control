#ifndef MOTOR_ENCODER_HPP
#define MOTOR_ENCODER_HPP

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <memory>
#include <atomic>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <string>

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

using EncoderPtr = std::unique_ptr<MotorEncoder>;
static EncoderPtr encoder1 = nullptr;
static EncoderPtr encoder2 = nullptr;
static EncoderPtr encoder3 = nullptr;
static EncoderPtr encoder4 = nullptr;
static EncoderPtr encoder5 = nullptr;
static EncoderPtr encoder6 = nullptr;

using IsrFunc = void (*)();
void isr1() { if (encoder1) encoder1->updateCounter(); }
void isr2() { if (encoder2) encoder2->updateCounter(); }
void isr3() { if (encoder3) encoder3->updateCounter(); }
void isr4() { if (encoder4) encoder4->updateCounter(); }
void isr5() { if (encoder5) encoder5->updateCounter(); }
void isr6() { if (encoder6) encoder6->updateCounter(); }

// BCM = grove-hat-base numbering
// wPi = wiringPiNumbering, use with <wiringPi.h> and <wiringPiI2C.h>
// ==============================================================================
// ==============================================================================
// +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+
// | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
// +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
// |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
// |   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
// |   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
// |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | IN   | TxD     | 15  | 14  |
// |     |     |      0v |      |   |  9 || 10 | 1 | IN   | RxD     | 16  | 15  |
// |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
// |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
// |  22 |   3 | GPIO. 3 |   IN | 1 | 15 || 16 | 1 | IN   | GPIO. 4 | 4   | 23  |
// |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
// |  10 |  12 |    MOSI |   IN | 0 | 19 || 20 |   |      | 0v      |     |     |
// |   9 |  13 |    MISO |   IN | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
// |  11 |  14 |    SCLK |   IN | 0 | 23 || 24 | 1 | IN   | CE0     | 10  | 8   |
// |     |     |      0v |      |   | 25 || 26 | 1 | IN   | CE1     | 11  | 7   |
// |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
// |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
// |   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
// |  13 |  23 | GPIO.23 |   IN | 0 | 33 || 34 |   |      | 0v      |     |     |
// |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
// |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
// |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
// +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
// | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
// +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+
// ==============================================================================
// ==============================================================================

class EncoderManager {
private:
    std::vector<EncoderPtr*> encoder_vars = {
        &encoder1, &encoder2, &encoder3,
        &encoder4, &encoder5, &encoder6
    };
    std::vector<IsrFunc> isr_functions = {
        isr1, isr2, isr3,
        isr4, isr5, isr6
    };

    const std::vector<std::pair<int, int>> encoder_pin_table = {
        {21, 22}, // D5  = {5 , 6 }
        {3 , 4 }, // D22 = {22, 23}
        {27, 0 }, // D16 = {16, 17}
        {5 , 6 }, // D24 = {24, 25}
        {1 , 24}, // D18 = {18, 19}
        {25, 2 }  // D26 = {26, 27}
    };

    void declareEncoders() {
        for (std::size_t i = 0; i < number_encoder; ++i) {
            int H1 = encoder_pin_table[i].first;
            int H2 = encoder_pin_table[i].second;
            *encoder_vars[i] = std::make_unique<MotorEncoder>(H1, H2);
        }
    }

    void attachEncoderInterrupts() {
        for (std::size_t i = 0; i < number_encoder; ++i) {
            int H1 = encoder_pin_table[i].first;
            if (wiringPiISR(H1, INT_EDGE_RISING, isr_functions[i]) < 0)
                throw std::runtime_error("Failed to attach ISR to pin " + std::to_string(H1));
            std::cout << "[INFO] ISR attached to pin " << H1 << "\n";
        }
    }

    int number_encoder = 0;

public:
    EncoderManager(int number_encoder = 2) :
    number_encoder(number_encoder)    
    {
        declareEncoders();
        attachEncoderInterrupts();
        for (std::size_t i = 0; i < number_encoder; ++i) {
            (*encoder_vars[i])->resetCounter();
        }
    }

    void getMotorCounter(std::vector<int64_t> &motor_counter) {
        for (std::size_t i = 0; i < number_encoder; ++i) {
            motor_counter[i] = (*encoder_vars[i])->getCounter();
        }
    }
};

#endif // MOTOR_ENCODER_HPP
