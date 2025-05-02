#include <benchmark/benchmark.h>
#include <memory>
#include <vector>
#include <cstdint>

class Encoder {
private:
    int64_t counter;

public:
    Encoder(int64_t initial = 0) : counter(initial) {}
    int64_t getCounter() const { return counter; }
};

constexpr std::size_t NUM_ENCODERS = 6;
std::vector<std::unique_ptr<Encoder>> encoders;

void setupEncoders() {
    if (encoders.empty()) {
        for (std::size_t i = 0; i < NUM_ENCODERS; ++i)
            encoders.emplace_back(std::make_unique<Encoder>(123 + i));
    }
}

void getMotorCounter(int64_t* counters[], std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        *counters[i] = encoders[i]->getCounter();
    }
}

static void BM_getMotorCounter_function(benchmark::State& state) {
    int64_t v1, v2, v3, v4, v5, v6;
    int64_t* counters[] = {&v1, &v2, &v3, &v4, &v5, &v6};

    for (auto _ : state) {
        getMotorCounter(counters, NUM_ENCODERS);
        benchmark::DoNotOptimize(v1);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(v3);
        benchmark::DoNotOptimize(v4);
        benchmark::DoNotOptimize(v5);
        benchmark::DoNotOptimize(v6);
    }
}
BENCHMARK(BM_getMotorCounter_function);

static void BM_getMotorCounter_raw(benchmark::State& state) {
    for (auto _ : state) {
        int64_t v1 = encoders[0]->getCounter();
        int64_t v2 = encoders[1]->getCounter();
        int64_t v3 = encoders[2]->getCounter();
        int64_t v4 = encoders[3]->getCounter();
        int64_t v5 = encoders[4]->getCounter();
        int64_t v6 = encoders[5]->getCounter();
        benchmark::DoNotOptimize(v1);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(v3);
        benchmark::DoNotOptimize(v4);
        benchmark::DoNotOptimize(v5);
        benchmark::DoNotOptimize(v6);
    }
}
BENCHMARK(BM_getMotorCounter_raw);

int main(int argc, char** argv) {
    setupEncoders();
    ::benchmark::Initialize(&argc, argv);
    ::benchmark::RunSpecifiedBenchmarks();
}