#pragma once
#include "mock_encoder.hpp"
#include <vector>
#include <memory>

class MockEncoderManager {
private:
    std::vector<MockEncoderPtr> encoders;

public:
    MockEncoderManager(std::size_t num) {
        encoders.reserve(num);
        for (std::size_t i = 0; i < num; ++i)
            encoders.emplace_back(std::make_unique<MockMotorEncoder>());
    }

    void getMotorCounter(int64_t* counters[], std::size_t counter_numbers) {
        for (std::size_t i = 0; i < counter_numbers; ++i) {
            *counters[i] = encoders[i]->getCounter();
        }
    }

    void simulateTicks() {
        for (auto& encoder : encoders)
            encoder->updateCounter();
    }
};