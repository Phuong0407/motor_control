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

constexpr int NUM_ENCODERS = 3;
static constexpr int COUNTER_PER_REV = 144;
static constexpr double ANGLE_PER_TICK = 2.5;

class MotorEncoder {
private:
    int H1, H2;
    volatile int64_t counter;

    void initEncoder(int i2c_addr) {
        if (wiringPiSetup() == -1)
            throw std::runtime_error("[ERROR] wiringPiSetup failed");

        int i2c_fd = wiringPiI2CSetup(i2c_addr);
        if (i2c_fd < 0)
            throw std::runtime_error("[ERROR] I2C setup failed");

        pinMode(H1, INPUT);
        pinMode(H2, INPUT);
        pullUpDnControl(H1, PUD_UP);
        pullUpDnControl(H2, PUD_UP);

        if (wiringPiI2CRead(i2c_fd) == -1)
            std::cerr << "[ERROR] I2C device is not responding.\n";

        std::cout   << "[INFO] Encoder initialized on pins H1 = "
                    << H1 << ", H2 = " << H2 << "\n";
    }

public:
    MotorEncoder(int H1, int H2, int i2c_addr = 0x0f) :
    H1(H1), H2(H2), counter(0)
    {
        initEncoder(i2c_addr);
    }

    void updateCounter() {
        digitalRead(H2) ? ++counter : --counter;
    }

    void resetCounter() {
        counter = 0;
    }

    int64_t getCounter() const { return counter; }
};

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

inline MotorEncoder* encoders[NUM_ENCODERS] = { nullptr };

static void isr0() { if (encoders[0]) encoders[0]->updateCounter(); }
static void isr1() { if (encoders[1]) encoders[1]->updateCounter(); }
static void isr2() { if (encoders[2]) encoders[2]->updateCounter(); }

typedef void (*IsrFunc)();
static IsrFunc isr_functions[NUM_ENCODERS] = {
    isr0, isr1, isr2
};

static const int encoder_pin_table[NUM_ENCODERS][2] = {
    {21, 22}, // D5  = {5 , 6 }
    {3 , 4 }, // D22 = {22, 23}
    {27, 0 }, // D16 = {16, 17}
};

inline void declareEncoders(int driver1_addr, int driver2_addr) {
    for (int i = 0; i < NUM_ENCODERS; ++i) {
        int H1 = encoder_pin_table[i][0];
        int H2 = encoder_pin_table[i][1];
        int addr = (i == NUM_ENCODERS - 1) ? driver2_addr : driver1_addr;
        encoders[i] = new MotorEncoder(H1, H2, addr);
    }
}

inline void cleanupEncoders() {
    for (int i = 0; i < NUM_ENCODERS; ++i) {
        delete encoders[i];
        encoders[i] = nullptr;
    }
}

inline void attachEncoderInterrupts() {
    for (std::size_t i = 0; i < NUM_ENCODERS; ++i) {
        int H1 = encoder_pin_table[i][0];
        if (wiringPiISR(H1, INT_EDGE_RISING, isr_functions[i]) < 0)
            throw std::runtime_error("[ERROR] Failed to attach ISR to pin " + std::to_string(H1));
        std::cout << "[INFO] ISR attached to pin " << H1 << "\n";
    }
}

#endif // MOTOR_ENCODER_HPP
