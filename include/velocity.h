#ifndef VELOCITY_H
#define VELOCITY_H

#include "robot.h"
#include <cmath>

double old_error = 0.0;

inline double computeDirectionControlSignal(double error) {
    double derivative = error - old_error;
    old_error = error;
    return kp_dir * error + kd_dir * derivative / 0.1;
}

void * computeRefTPSFromVision(void * arg) {
    while(!TERMINATE_PROGRAM) {
        double deviation = static_cast<double>(x - FRAMEWIDTH / 2);
        if (deviation > 0.0 && std::abs(deviation) >= TURN_RIGHT_THRES)
            TURN_RIGHT = true;
        if (deviation < 0.0 && std::abs(deviation) >= TURN_LEFT_THRES)
            TURN_LEFT = true;

        omega = computeDirectionControlSignal(deviation);

        ref1 = (1.0 / r) * (base_speed - L1 * omega) * COUNTER_PER_REV;
        ref2 = (1.0 / r) * (base_speed + L1 * omega) * COUNTER_PER_REV;
        ref3 = (1.0 / r) * omega * L2 * COUNTER_PER_REV;
    }
    return nullptr;
}

#endif // VELOCITY_H