#pragma once
#include <atomic>
#include <thread>
#include <chrono>

class MockMotorEncoder {
private:
    int64_t counter;

public:
    MockMotorEncoder() : counter(0) {}

    void updateCounter() {
        ++counter;
    }

    int64_t getCounter() const {
        return counter;
    }

    void simulateTicks() {
        std::thread([this]() {
            const int tick_rate = 4320;
            const auto tick_interval = std::chrono::microseconds(1000000 / tick_rate);
            while (true) {
                updateCounter();
                std::this_thread::sleep_for(tick_interval);
            }
        }).detach();
    }
};

using MockEncoderPtr = std::unique_ptr<MockMotorEncoder>;