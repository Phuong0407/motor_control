#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <memory>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <string>

constexpr int NUM_ENCODERS              = 3;
static constexpr double COUNTER_PER_REV = 144.0;
static constexpr double ANGLE_PER_TICK  = 2.5;

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

#endif // ENCODER_HPP