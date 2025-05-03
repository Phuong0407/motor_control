#include "motor_driver.hpp"

int main(int argc, char* argv[]) {
    if (argc < 5)
        return 0;
    
    int pwm1        = std::stoi(argv[1], nullptr, 0);
    int pwm2        = std::stoi(argv[2], nullptr, 0);
    int pwm3        = std::stoi(argv[3], nullptr, 0);
    double run_itv  = std::stod(argv[4]);
    double smpl_itv = std::stod(argv[5]);

    MotorDriver motor_driver(0.05, 0.05, 0.05, 0.2, 0x0f, 0x0d);

    auto t_start = std::chrono::steady_clock::now();
    motor_driver.setLeftRightMotor(0.2, 0.2, false);
    while (true) {
        auto t_now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = t_now - t_start;

        if (elapsed.count() >= run_itv)
            break;

        motor_driver.measureAngularVelocity(smpl_itv);
        std::this_thread::sleep_for(std::chrono::duration<double>(smpl_itv));
    }
    motor_driver.stop_motor();
    cleanupEncoders();
    return 0;
}
