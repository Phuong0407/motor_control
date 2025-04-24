#include "pid_controler.hpp"

#include <iostream>
#include <chrono>

int main() {
    PIDController pid(0.1, 0.2, 0.005);
    pid.setSetpoint(100.0); // Set desired angle
    double measured_value = 90.0; // Current angle
    double dt = 0.2; // Time step in seconds
    int iteration = 0;

    auto start_time = std::chrono::high_resolution_clock::now();

    while (std::abs(measured_value - 100.0) > 0.5) {
        measured_value += pid.compute(measured_value, dt);
        std::cout << "Measured Value: " << measured_value << std::endl;
        iteration++;
        std::cout << "Iteration: " << iteration << std::endl;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_time = end_time - start_time;
    std::cout << "Elapsed Time: " << elapsed_time.count() << " seconds" << std::endl;
}