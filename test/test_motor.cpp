#define NOLOAD_RUN

#include "motor.hpp"

#define MULTITHREAD_CONTROL_CPP
// #define MEASURE_MOTOR_CPP

#ifdef MULTITHREAD_CONTROL_CPP

#include <thread>
#include <string>
#include <iostream>

int main(int argc, char *argv[]) {
    double kp       = std::stod(argv[1]);
    double ki       = std::stod(argv[2]);
    double kd       = std::stod(argv[3]);
    double cutoff   = std::stod(argv[4]);

    startEncoders();
    pid1.setPIDParameters(kp, ki, kd, cutoff, MAX_TICKS);
    pid2.setPIDParameters(6.0, 0.5, 0.01, 4.0, MAX_TICKS);
    pid3.setPIDParameters(6.0, 0.5, 0.01, 4.0, MAX_TICKS);

    ref1 = 90.0, ref2 = 0.0, ref3 = 0.0;
    
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

#endif // MULTITHREAD_CONTROL_CPP

#ifdef MEASURE_MOTOR_CPP

int main() {
    startEncoders();
    double omega1, omega2, omega3;
    while(true) {
        measureAngularVelocity(omega1, omega2, omega3);
        printf("omega1\t=\t%.3f,\tomega2\t=\t%.3f,\tomega3\t=\t%.3f\n", omega1, omega2, omega3);
        delay(1000);
    }
}

#endif // MEASURE_MOTOR_CPP