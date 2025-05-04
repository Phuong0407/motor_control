#define NOLOADED_RUN
#include "motor_driver.hpp"

int main(int argc, char* argv[]) {
    if (argc < 5)
        return 0;
    
    double kp       = std::stod(argv[1]);
    double ki       = std::stod(argv[2]);
    double kd       = std::stod(argv[3]);
    double run_itv  = std::stod(argv[4]);
    double smpl_itv = std::stod(argv[5]);

    double cutoff_freq = 50.0;

    MotorDriver motor_driver(kp, ki, kd, smpl_itv, cutoff_freq, 0x0f, 0x0d);

    auto t_start = std::chrono::steady_clock::now();
    motor_driver.controlAngularVelocity(0.5, 0.5, 0.0);

    motor_driver.stop_motor();
    cleanupEncoders();
    return 0;
}
