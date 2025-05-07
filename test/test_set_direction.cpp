#include <random>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

#define NOLOAD_RUN

#include "motor_driver.hpp"

int main(int argc, char* argv[]) {
    if (argc < 5)
        return 0;

    double kp       = std::stod(argv[1]);
    double ki       = std::stod(argv[2]);
    double kd       = std::stod(argv[3]);
    double run_itv  = std::stod(argv[4]);
    double smpl_itv = std::stod(argv[5]);

    double cutoff_freq = 20.0;

    MotorDriver motor_driver(kp, ki, kd, smpl_itv, cutoff_freq, 0x0f, 0x0d);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(-0.7, 0.7);  // normalized velocity range

    auto t_start = std::chrono::steady_clock::now();
    double omega1 = 0.0, omega2 = 0.0, omega3 = 0.0;
    double time = 0.0;

    while (time <= run_itv) {
        double left_speed = 0.5 / 0.8681;
        double right_speed = 0.5 / 0.8681;

        std::cout << "set velocity\t" << std::setprecision(3) << std::fixed << left_speed << "\t" << right_speed << "\t";
        motor_driver.setLeftRightMotorNormalized(left_speed, right_speed);

        auto start = std::chrono::steady_clock::now();
        std::this_thread::sleep_until(start + std::chrono::duration<double>(smpl_itv));

        motor_driver.measureAngularVelocity(omega1, omega2, omega3, smpl_itv);
        std::cout << "measure velocity\t" << std::setprecision(3) << std::fixed << omega1 << "\t" << omega2 << "\n";

        time += smpl_itv;
    }

    motor_driver.stop_motor();
    cleanupEncoders();
    return 0;
}

