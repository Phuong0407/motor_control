#define NOLOAD_RUN

#include "motor.hpp"

#include <thread>

int main() {
    startEncoders();
    pid1.setPIDParameters(6.0, 0.5, 0.01, 4.0, MAX_RPS);
    pid2.setPIDParameters(6.0, 0.5, 0.01, 4.0, MAX_RPS);
    pid3.setPIDParameters(6.0, 0.5, 0.01, 4.0, MAX_RPS);

    ref1 = 0.625, ref2 = 0.625, ref3 = 0.625;
    
    std::thread motor1Thread(controlMotor1, nullptr);
    std::thread motor2Thread(controlMotor2, nullptr);
    std::thread motor3Thread(controlMotor3, nullptr);
    std::thread monitorThread(monitorMotorsSpeed, nullptr);
    motor1Thread.join();
    motor2Thread.join();
    motor3Thread.join();
    monitorThread.join();

    return 0;
}