#include "motor_driver.hpp"

int main(int argc, char* argv[]) {
    if (argc < 4)
        return 0;
    
    int pwm1 = std::stoi(argv[1], nullptr, 0);
    int pwm2 = std::stoi(argv[2], nullptr, 0);
    int pwm3 = std::stoi(argv[3], nullptr, 0);
    int ms   = std::stoi(argv[4]);

    MotorDriver motor_driver(0.05, 0.05, 0.05, 0.2, 0x0f, 0x0d);
    int counter = 0;
    while(counter <= 200) {
        motor_driver.set_motor_pwm(pwm1, pwm2, pwm3, ms); 
        motor_driver.measureAngularVelocity();
        motor_driver.set_motor_pwm(pwm1, pwm2, pwm3, 1000);
        motor_driver.measureAngularVelocity();
        counter++;
    }
        cleanupEncoders();
    return 0;
}
