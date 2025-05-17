#include <iostream>
#include <thread>
#include <csignal>
#include <chrono>

#include "motorcontroller.hpp"

volatile bool run = true;

int main() {
    MotorController motorController;
    motorController.setMotorController(
        6.0, 0.5, 0.05, 4.0, 0.86,
        6.0, 0.5, 0.05, 4.0, 0.86,
        6.0, 0.5, 0.05, 4.0, 0.86
    );

    motorController.setMotor1Reference(0.5);
    motorController.setMotor2Reference(0.7);
    motorController.setMotor3Reference(0.625);

    std::thread motor1Thread(&MotorController::controlMotor1, &motorController);
    std::thread motor2Thread(&MotorController::controlMotor2, &motorController);
    std::thread motor3Thread(&MotorController::controlMotor3, &motorController);
    std::thread monitorThread(&MotorController::monitorMotorsSpeed, &motorController);

    motor1Thread.join();
    motor2Thread.join();
    motor3Thread.join();
    monitorThread.join();

    return 0;
}