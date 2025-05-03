#include "motor_driver.hpp"

int main(int argc, char* argv[]) {
    if (argc < 5)
        return 0;
    
    double kp       = std::stod(argv[1]);
    double ki       = std::stod(argv[2]);
    double kd       = std::stod(argv[3]);
    double run_itv  = std::stod(argv[4]);
    double smpl_itv = std::stod(argv[5]);

    MotorDriver motor_driver(kp, ki, kd, 0.01, 0x0f, 0x0d);

    auto t_start = std::chrono::steady_clock::now();
    motor_driver.controlAngularVelocity(0.5, 0.5, 0.0, smpl_itv, false);
    
//    motor_driver.setLeftRightMotor(0.6, 0, 0);

    while (true) {
        auto t_now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = t_now - t_start;

        if (elapsed.count() >= run_itv)
            break;

        std::cout << "stable motor velocity.\n";
        motor_driver.measureAngularVelocity(smpl_itv);
        std::this_thread::sleep_for(std::chrono::duration<double>(smpl_itv));
    }
    motor_driver.stop_motor();
    cleanupEncoders();
    return 0;
}
