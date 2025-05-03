#include "motor_driver.hpp"

int main(int argc, char* argv[]) {
    if (argc < 4)
        return 0;
    
    int pwm1 = std::stoi(argv[1], nullptr, 0);
    int pwm2 = std::stoi(argv[2], nullptr, 0);
    int pwm3 = std::stoi(argv[3], nullptr, 0);
    int ms   = std::stoi(argv[4]);

    MotorDriver motor_driver(0.05, 0.05, 0.05, 0.2, 0x0f, 0x0d);

    auto t_start = std::chrono::steady_clock::now();
    while(true) {
        auto t_now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = t_now - t_start;

        if (elapsed.count() > 20.0)
            break;

        motor_driver.set_motor_pwm(pwm1, pwm2, pwm3); 
        motor_driver.measureAngularVelocity();
    }

    motor_driver.stop_motor();
    cleanupEncoders();
    return 0;
}
