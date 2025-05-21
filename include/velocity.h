#ifndef VELOCITY_H
#define VELOCITY_H

#include "robot.h"
#include <cmath>

double old_error = 0.0;

void computeRobotVelocity() {
    double deviation = static_cast<double>(x - framewidth / 2);
    if (deviation > 0.0 && std::abs(deviation) >= TURN_RIGHT_THRES)
        turn_right = true;
    if (deviation < 0.0 && std::abs(deviation) >= TURN_LEFT_THRES)
        turn_left = true;

    omega = computeDirectionControlSignal(deviation);
}

void computeRefTPSFromVelocity() {
    ref1 = (1.0 / r) * (base_speed - L1 * omega) * 144.0;
    ref2 = (1.0 / r) * (base_speed + L1 * omega) * 144.0;
    ref3 = (1.0 / r) * omega * L2 * 144.0;
}

void * computeRefTPSFromVision(void * arg) {
    while(true) {
        computeRobotVelocity();
        computeRefTPSFromVelocity();
    }
    return nullptr;
}

#endif // VELOCITY_H