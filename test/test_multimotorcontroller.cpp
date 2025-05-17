#include <iostream>
#include <thread>
#include <csignal>
#include <chrono>

#include "motorcontroller.hpp"

volatile bool run = true;

void monitorMotors(MotorController& motorController) {
    while (run) {
        std::cout << "Motor 1 Speed: " << motorController.getMotor1Reference() << std::endl;
        std::cout << "Motor 2 Speed: " << motorController.getMotor2Reference() << std::endl;
        std::cout << "Motor 3 Speed: " << motorController.getMotor3Reference() << std::endl;
        std::cout << "---------------------------------------" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

int main() {
    // MotorController object setup
    MotorController motorController;
    motorController.setMotorController(
        6.0, 0.5, 0.05, 4.0, 0.86,   // Motor 1
        6.0, 0.5, 0.05, 4.0, 0.86,   // Motor 2
        6.0, 0.5, 0.05, 4.0, 0.86    // Motor 3
    );

    // Setting target references
    motorController.setMotor1Reference(0.5);
    motorController.setMotor2Reference(0.7);
    motorController.setMotor3Reference(0.625);

    // Thread creation for motor control and monitoring
    std::thread motor1Thread(&MotorController::controlMotor1, &motorController);
    std::thread motor2Thread(&MotorController::controlMotor2, &motorController);
    std::thread motor3Thread(&MotorController::controlMotor3, &motorController);
    std::thread monitorThread(monitorMotors, std::ref(motorController));

    // Join threads
    motor1Thread.join();
    motor2Thread.join();
    motor3Thread.join();
    monitorThread.join();

    return 0;
}